#include "Config.hpp"
#include "Exception.hpp"
#include "Print.hpp"
#include "BoundingBox.hpp"
#include "Brim.hpp"
#include "ClipperUtils.hpp"
#include "Extruder.hpp"
#include "Flow.hpp"
#include "Geometry/ConvexHull.hpp"
#include "I18N.hpp"
#include "ShortestPath.hpp"
#include "Thread.hpp"
#include "Time.hpp"
#include "GCode.hpp"
#include "GCode/WipeTower.hpp"
#include "GCode/WipeTower2.hpp"
#include "WipeTowerReplay.hpp"
#include "Utils.hpp"
#include "PrintConfig.hpp"
#include "MixedNozzleConfig.hpp"
#include "MaterialType.hpp"
#include "Layer.hpp"
#include "LocalesUtils.hpp"
#include "Model.hpp"
#include "format.hpp"
#include <float.h>
#include <cstdlib>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <sstream>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/log/trivial.hpp>
#include <boost/regex.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/nowide/iostream.hpp>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

//BBS: add json support
#include "nlohmann/json.hpp"

#include "GCode/ConflictChecker.hpp"
#include "ParameterUtils.hpp"

#include <codecvt>

using namespace nlohmann;

// Mark string for localization and translate.
#define L(s) Slic3r::I18N::translate(s)

namespace Slic3r {

Print::SlicingPipelineHookFn Print::s_slicing_pipeline_hook_fn = nullptr;

template class PrintState<PrintStep, psCount>;
template class PrintState<PrintObjectStep, posCount>;

PrintRegion::PrintRegion(const PrintRegionConfig &config) : PrintRegion(config, config.hash()) {}
PrintRegion::PrintRegion(PrintRegionConfig &&config) : PrintRegion(std::move(config), config.hash()) {}

//BBS
// ORCA: Now this is a parameter
//float Print::min_skirt_length = 0;

struct FilamentType {
    std::string name;
    int min_temp;
    int max_temp;
    std::string temp_type;
};

namespace {

WipeTowerReplayTiming replay_wipe_tower_timing(
    const std::vector<std::vector<WipeTower::ToolChangeResult>> &tool_changes,
    const WipeTowerReplayContext &context)
{
    WipeTowerReplayTiming timing;
    for (const auto &layer : tool_changes) {
        for (const WipeTower::ToolChangeResult &change : layer) {
            const bool has_payload = !change.gcode.empty() || !change.nozzle_change_result.gcode.empty() ||
                                     !change.extrusions.empty();
            if (!has_payload)
                continue;
            if (std::isfinite(change.elapsed_time) && change.elapsed_time >= 0.f)
                timing.writer_motion_seconds += double(change.elapsed_time);
            else
                timing.writer_motion_complete = false;

        }
    }

    if (context.request_native_timing && !context.native_command_stream.empty()) {
        try {
            GCodeProcessor processor;
            processor.reset();
            processor.initialize_from_context(context.nozzle_group_result);
            processor.initialize_result_moves();
            // Keep T-driven occupancy and per-tool motion limits, but leave load/unload/tool-change
            // statistics to the stateful transition adapter so T commands are not charged twice.
            PrintConfig motion_config = context.config;
            motion_config.machine_load_filament_time.value = 0.;
            motion_config.machine_unload_filament_time.value = 0.;
            motion_config.machine_tool_change_time.value = 0.;
            processor.apply_config(motion_config);
            // Only a complete native command stream is safe to parse. Raw ToolChangeResult snippets lack
            // the writer's active tool, start position and E mode, so their concatenation has no valid time.
            processor.process_buffer(context.native_command_stream);
            processor.finalize(false);
            timing.native_parser_seconds = processor.get_time(PrintEstimatedStatistics::ETimeMode::Normal);
            timing.native_parser_replayed = std::isfinite(timing.native_parser_seconds) &&
                                             timing.native_parser_seconds >= 0.;
        } catch (...) {
            // Keep the writer provenance; a parser failure is reported below as uncertainty.
            timing.native_parser_replayed = false;
        }
    }
    timing.interval_uncertain = !timing.writer_motion_complete;
    if (!context.request_native_timing || context.native_command_stream.empty() ||
        !context.complete_native_command_context ||
        !timing.native_parser_replayed)
        timing.interval_uncertain = true;
    return timing;
}

// Times short G-code snippets (tower rows, the printer's change G-code) with the slicer's own
// estimator, so the time check prices them the way the exported G-code is timed: acceleration,
// corners, Z moves and dwells included. One processor serves every snippet. Each snippet starts
// where the caller says, reached by an untimed move, and ends with a zero dwell that empties the
// planner, so its time is the processor's clock after it less the clock before it. Load, unload and tool change times are
// zeroed, as in replay_wipe_tower_timing() above, because the caller prices those itself.
class SnippetTimer
{
public:
    SnippetTimer(const PrintConfig &config, const std::shared_ptr<MultiNozzleUtils::LayeredNozzleGroupResult> &groups)
        : m_config(config), m_groups(groups)
    {
        this->restart();
    }

    // Seconds for gcode run from (x, y, z), or NaN when it cannot be timed.
    double seconds(const std::string &gcode, double x, double y, double z)
    {
        // The clock is read as a float, so it is started again before it grows large enough to
        // round a short snippet's time.
        if (m_ok && m_processor.get_time(PrintEstimatedStatistics::ETimeMode::Normal) > 1000.f)
            this->restart();
        if (!m_ok || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            return std::numeric_limits<double>::quiet_NaN();
        try {
            // Move to the start untimed: two moves, so the dwell has enough queued to empty the planner,
            // then read the clock. Arcs are relative to the true position, so the start is reached
            // by moving there rather than declared with G92.
            m_processor.process_buffer(Slic3r::format("G1 X%1% Y%2% Z%3% F60000\nG1 Z%4%\nG92 E0\nG4 S0\n",
                                                      x, y, z + 1., z));
            const double before = m_processor.get_time(PrintEstimatedStatistics::ETimeMode::Normal);
            m_processor.process_buffer(gcode);
            m_processor.process_buffer("\nG4 S0\n");
            const double elapsed = m_processor.get_time(PrintEstimatedStatistics::ETimeMode::Normal) - before;
            return std::isfinite(elapsed) && elapsed >= 0. ? elapsed : std::numeric_limits<double>::quiet_NaN();
        } catch (...) {
            m_ok = false;
            return std::numeric_limits<double>::quiet_NaN();
        }
    }

private:
    void restart()
    {
        try {
            m_processor.reset();
            m_processor.initialize_from_context(m_groups);
            m_processor.initialize_result_moves();
            PrintConfig motion_config = m_config;
            motion_config.machine_load_filament_time.value = 0.;
            motion_config.machine_unload_filament_time.value = 0.;
            motion_config.machine_tool_change_time.value = 0.;
            m_processor.apply_config(motion_config);
            m_processor.process_buffer("G90\nM83\n");
            m_ok = true;
        } catch (...) {
            m_ok = false;
        }
    }

    const PrintConfig                                           &m_config;
    std::shared_ptr<MultiNozzleUtils::LayeredNozzleGroupResult>  m_groups;
    GCodeProcessor                                               m_processor;
    bool                                                         m_ok { false };
};

} // namespace

// The prime a warm return lays when only `share` of its idle re-prime above the floor is kept.
// Share 1 is the full re-prime, share 0 is the floor.
static float mixed_nozzle_shared_reprime_volume(const PrintConfig &config, size_t filament_id, float floor_mm3,
                                                double idle_seconds, float share)
{
    const float full = mixed_nozzle_idle_reprime_volume(config, filament_id, floor_mm3, idle_seconds);
    return floor_mm3 + std::clamp(share, 0.f, 1.f) * (full - floor_mm3);
}

WipeTowerReplayResult replay_wipe_tower_plan(
    const std::vector<WipeTowerPlanEvent> &plan_events,
    WipeTowerReplayContext &context)
{
    WipeTowerReplayResult replay;
    if (plan_events.empty()) {
        // A fine-only alternative may require no tower events. Do not call the generator's
        // non-empty-plan path, and do not invent a support object or a handoff for that case.
        replay.support_depth = replay.footprint = replay.depth_residual = replay.bbox_residual = 0.;
        replay.replay_ok = replay.structure_qualified = true;
        if (context.request_native_timing)
            replay.timing = replay_wipe_tower_timing({}, context);
        return replay;
    }
    context.tower.set_used_filament_ids(std::vector<int>(context.used_filament_ids.begin(),
                                                          context.used_filament_ids.end()));
    context.tower.set_filament_categories(context.filament_categories);

    for (const WipeTowerPlanEvent &event : plan_events) {
        ++replay.expected_switches;
        if (event.old_tool == event.new_tool)
            --replay.expected_switches;

        float prime_extruder_change = event.prime_extruder_change;
        float ram_length_override = event.ram_override_mm;
        if (event.automatic_prime) {
            if (event.incoming_extruder < 0)
                return replay;
            const auto handoff = mixed_nozzle_handoff_deposits(
                context.config, event.incoming_filament, size_t(event.incoming_extruder),
                context.requested_width, event.layer_height);
            if (!handoff)
                return replay;
            prime_extruder_change = mixed_nozzle_shared_reprime_volume(
                context.config, event.incoming_filament, handoff->prime_volume_mm3, event.idle_seconds,
                context.reprime_share);
            replay.automatic_prime_volumes.emplace_back(prime_extruder_change);
        }
        if (event.automatic_ram) {
            if (event.outgoing_extruder < 0)
                return replay;
            const auto handoff = mixed_nozzle_handoff_deposits(
                context.config, event.outgoing_filament, size_t(event.outgoing_extruder),
                context.requested_width, event.layer_height);
            if (!handoff)
                return replay;
            ram_length_override = handoff->ram_length_mm;
            replay.automatic_ram_lengths.emplace_back(ram_length_override);
        }

        context.tower.plan_toolchange(
            event.z, event.layer_height, event.old_tool, event.new_tool,
            prime_extruder_change, event.prime_nozzle_change, event.purge_volume,
            event.force_emit, ram_length_override);
        if (event.disable_last_layer_fill)
            context.tower.set_last_layer_extruder_fill(false);
    }

    context.tower.generate_new(replay.tool_changes);
    replay.timing = replay_wipe_tower_timing(replay.tool_changes, context);
    replay.structure_qualified = context.audit_structure && context.audit_structure(replay.tool_changes);
    if (context.audit_structure && !replay.structure_qualified)
        return replay;

    size_t emitted_switches = 0;
    size_t positive_extrusions = 0;
    double emitted_min_x = std::numeric_limits<double>::infinity();
    double emitted_min_y = std::numeric_limits<double>::infinity();
    double emitted_max_x = -std::numeric_limits<double>::infinity();
    double emitted_max_y = -std::numeric_limits<double>::infinity();
    double max_extrusion_width = 0.;
    for (const auto &layer : replay.tool_changes) {
        for (const auto &tool_change : layer) {
            if (tool_change.is_tool_change)
                ++emitted_switches;
            replay.structural_emission_count += tool_change.structural_emissions.size();
            for (const WipeTower::Extrusion &extrusion : tool_change.extrusions) {
                if (!std::isfinite(extrusion.pos.x()) || !std::isfinite(extrusion.pos.y()) ||
                    !std::isfinite(extrusion.width) || extrusion.width < 0.f) {
                    return replay;
                }
                if (extrusion.width > 0.f) {
                    ++positive_extrusions;
                    emitted_min_x = std::min(emitted_min_x, double(extrusion.pos.x()));
                    emitted_min_y = std::min(emitted_min_y, double(extrusion.pos.y()));
                    emitted_max_x = std::max(emitted_max_x, double(extrusion.pos.x()));
                    emitted_max_y = std::max(emitted_max_y, double(extrusion.pos.y()));
                    max_extrusion_width = std::max(max_extrusion_width, double(extrusion.width));
                }
            }
        }
    }
    if (emitted_switches != replay.expected_switches ||
        (replay.expected_switches > 0 && positive_extrusions == 0))
        return replay;

    replay.bbx = context.tower.get_bbx();
    const double bbx_width = double(replay.bbx.max.x() - replay.bbx.min.x());
    const double bbx_depth = double(replay.bbx.max.y() - replay.bbx.min.y());
    const double nominal_width = context.tower.width();
    const double depth = context.tower.get_depth();
    const double brim = context.tower.get_brim_width();
    const double bbox_tolerance = 0.5;
    replay.bbox_residual = std::max(0., nominal_width - bbx_width);
    replay.depth_residual = std::max(0., depth - bbx_depth);
    if (positive_extrusions > 0) {
        const double emitted_margin = std::max(0.5, brim + 0.5 * max_extrusion_width);
        if (emitted_min_x < double(replay.bbx.min.x()) - emitted_margin ||
            emitted_max_x > double(replay.bbx.max.x()) + emitted_margin ||
            emitted_min_y < double(replay.bbx.min.y()) - emitted_margin ||
            emitted_max_y > double(replay.bbx.max.y()) + emitted_margin)
            return replay;
        replay.depth_residual = std::max(replay.depth_residual,
            std::max(0., (emitted_max_y - emitted_min_y) - bbx_depth));
    }
    if (!std::isfinite(nominal_width) || nominal_width <= 0. ||
        !std::isfinite(depth) || depth <= 0. || !std::isfinite(bbx_width) || bbx_width <= 0. ||
        !std::isfinite(bbx_depth) || bbx_depth <= 0. ||
        (std::isfinite(context.width_cap) && nominal_width > context.width_cap + EPSILON) ||
        bbx_width + 2. * brim + 0.5 < nominal_width || bbx_depth + 2. * brim + 0.5 < depth ||
        !std::isfinite(replay.bbox_residual) || replay.bbox_residual > bbox_tolerance ||
        !std::isfinite(replay.depth_residual) || replay.depth_residual > bbox_tolerance)
        return replay;

    replay.support_height = context.tower.get_height();
    replay.support_depth = depth;
    if (!std::isfinite(replay.support_height) || replay.support_height < 0.)
        return replay;
    replay.footprint = bbx_width * bbx_depth;
    replay.replay_ok = true;
    return replay;
}

AutoPadWidthWalk walk_auto_pad_width(float seed, float minimum_width, float width_cap,
                                     size_t max_iterations, std::set<long> &replayed_widths,
                                     const AutoPadWidthBuildFn &build)
{
    AutoPadWidthWalk walk;
    float requested_width = seed;
    // Set for a deliberate rebuild at the planner's own width. That width usually shares the
    // previous build's replay key, so the rebuild skips the cycle check; it still counts against
    // max_iterations.
    bool rebuilding = false;
    for (size_t iteration = 0; iteration < max_iterations; ++iteration) {
        const long key = std::lround(double(requested_width) * 1000.);
        const bool fresh_width = replayed_widths.insert(key).second;
        if (!fresh_width && !rebuilding) {
            // Returning to a width this walk already built is a loop, and every width on it goes to the
            // fallback. Widths built only by an earlier walk were already handled by that walk.
            const auto seen = std::find_if(walk.path.begin(), walk.path.end(), [key](float width) {
                return std::lround(double(width) * 1000.) == key;
            });
            if (seen == walk.path.end()) {
                walk.end = AutoPadWalkEnd::Merged;
                return walk;
            }
            walk.end = AutoPadWalkEnd::Cycle;
            walk.cycle.assign(seen, walk.path.end());
            return walk;
        }
        rebuilding = false;
        const AutoPadWidthBuild result = build(requested_width);
        walk.path.push_back(requested_width);
        if (std::isfinite(result.generated_width) && result.generated_width > width_cap + EPSILON &&
            (walk.above_cap_width <= 0.f || result.generated_width < walk.above_cap_width))
            walk.above_cap_width = result.generated_width;
        if (!result.replay_ok) {
            walk.end = AutoPadWalkEnd::ReplayFailed;
            return walk;
        }
        const float generated_width = result.generated_width;
        if (!std::isfinite(generated_width) || generated_width < minimum_width - EPSILON ||
            generated_width > width_cap + EPSILON) {
            walk.end = AutoPadWalkEnd::OutOfRange;
            return walk;
        }
        if (std::abs(generated_width - requested_width) <= 0.001f) {
            // Rebuild once at the planner's actual width before accepting the fixed point, so the width
            // resolver and the final Type1 rows agree even when align_ceil() moved the first trial.
            if (std::abs(generated_width - requested_width) > 0.00001f) {
                requested_width = generated_width;
                rebuilding      = true;
                continue;
            }
            walk.end   = AutoPadWalkEnd::Settled;
            walk.width = generated_width;
            return walk;
        }
        requested_width = generated_width;
    }
    walk.end   = AutoPadWalkEnd::Unsettled;
    walk.width = requested_width;
    return walk;
}

std::vector<float> auto_pad_fallback_widths(const std::vector<AutoPadWidthWalk> &walks)
{
    std::set<float> widths;
    for (const AutoPadWidthWalk &walk : walks) {
        if (walk.end == AutoPadWalkEnd::Cycle)
            widths.insert(walk.cycle.begin(), walk.cycle.end());
        // A walk that ran out of iterations was still converging; the width it would have tried next
        // is the closest it got.
        else if (walk.end == AutoPadWalkEnd::Unsettled && std::isfinite(walk.width) && walk.width > 0.f)
            widths.insert(walk.width);
    }
    return std::vector<float>(widths.begin(), widths.end());
}

void Print::clear()
{
	std::scoped_lock<std::mutex> lock(this->state_mutex());
    // The following call should stop background processing if it is running.
    this->invalidate_all_steps();
	for (PrintObject *object : m_objects)
		delete object;
	m_objects.clear();
    m_print_regions.clear();
    m_model.clear_objects();
    m_statistics_by_extruder_count.clear();
}

bool Print::has_tpu_filament() const
{
    for (unsigned int filament_id : m_wipe_tower_data.tool_ordering.all_extruders()) {
        std::string filament_name = m_config.filament_type.get_at(filament_id);
        if (filament_name == "TPU") {
            return true;
        }
    }
    return false;
}

// Called by Print::apply().
// This method only accepts PrintConfig option keys.
bool Print::invalidate_state_by_config_options(const ConfigOptionResolver &new_config, const std::vector<t_config_option_key> &opt_keys)
{
    if (opt_keys.empty())
        return false;

    // Cache the plenty of parameters, which influence the G-code generator only,
    // or they are only notes not influencing the generated G-code.
    static std::unordered_set<std::string> steps_gcode = {
        //BBS
        "additional_cooling_fan_speed",
        "reduce_crossing_wall",
        "max_travel_detour_distance",
        "printable_area",
        //BBS: add bed_exclude_area
        "bed_exclude_area",
        "thumbnail_size",
        "before_layer_change_gcode",
        "enable_pressure_advance",
        "pressure_advance",
        "enable_overhang_bridge_fan",
        "overhang_fan_speed",
        "overhang_fan_threshold",
        "slow_down_for_layer_cooling",
        "default_acceleration",
        "deretraction_speed",
        "close_fan_the_first_x_layers",
        "machine_end_gcode",
        "printing_by_object_gcode",
        "filament_end_gcode",
        "post_process",
        // "plugins" is the derived manifest backing the plugin-picker options; on its own it only
        // affects G-code export. The specific option (e.g. slicing_pipeline_plugin) drives any re-slice.
        "plugins",
        "extruder_clearance_height_to_rod",
        "extruder_clearance_height_to_lid",
        "extruder_clearance_radius",
        "nozzle_height",
        "extruder_colour",
        "extruder_offset",
        "filament_flow_ratio",
        "reduce_fan_stop_start_freq",
        "dont_slow_down_outer_wall",
        "fan_cooling_layer_time",
        "full_fan_speed_layer",
        "initial_layer_fan_speed",
        "fan_kickstart",
        "part_cooling_fan_min_pwm",
        "fan_speedup_overhangs",
        "fan_speedup_time",
        "filament_colour",
        "default_filament_colour",
        "filament_diameter",
         "volumetric_speed_coefficients",
        "filament_density",
        "filament_cost",
        "filament_notes",
        "outer_wall_acceleration",
        "inner_wall_acceleration",
        "initial_layer_acceleration",
        "top_surface_acceleration",
        "bridge_acceleration",
        "travel_acceleration",
        "sparse_infill_acceleration",
        "internal_solid_infill_acceleration",
        // BBS
        "supertack_plate_temp_initial_layer",
        "cool_plate_temp_initial_layer",
        "textured_cool_plate_temp_initial_layer",
        "eng_plate_temp_initial_layer",
        "hot_plate_temp_initial_layer",
        "textured_plate_temp_initial_layer",
        "gcode_add_line_number",
        "layer_change_gcode",
        "time_lapse_gcode",
        "wrapping_detection_gcode",
        "fan_min_speed",
        "fan_max_speed",
        "printable_height",
        "slow_down_min_speed",
        "max_volumetric_extrusion_rate_slope",
        "max_volumetric_extrusion_rate_slope_segment_length",
        "extrusion_rate_smoothing_external_perimeter_only",
        "reduce_infill_retraction",
        "filename_format",
        "retraction_minimum_travel",
        "retract_before_wipe",
        // Orca:
        "retract_after_wipe",
        "retract_when_changing_layer",
        "retraction_length",
        "retract_length_toolchange",
        "z_hop",
        "travel_slope",
        "retract_lift_above",
        "retract_lift_below", 
        "retract_lift_enforce",
        "retract_restart_extra",
        "retract_restart_extra_toolchange",
        "retraction_speed",
        "use_firmware_retraction",
        "slow_down_layer_time",
        "standby_temperature_delta",
        "preheat_time",
        "preheat_steps",
        "machine_start_gcode",
        "filament_start_gcode",
        "change_filament_gcode",
        "wipe",
        // BBS
        "wipe_distance",
        "curr_bed_type",
        "nozzle_volume",
        "nozzle_hrc",
        "required_nozzle_HRC",
        "upward_compatible_machine",
        "is_infill_first",
        // Orca
        "chamber_temperature",
        "chamber_minimal_temperature",
        "thumbnails",
        "thumbnails_format",
        "center_of_surface_pattern",
        "separated_infills",
        "seam_gap",
        "role_based_wipe_speed",
        "wipe_speed",
        "use_relative_e_distances",
        "accel_to_decel_enable",
        "accel_to_decel_factor",
        "wipe_on_loops",
        "gcode_comments",
        "gcode_label_objects", 
        "exclude_object",
        "support_material_interface_fan_speed",
        "internal_bridge_fan_speed", // ORCA: Add support for separate internal bridge fan speed control
        "ironing_fan_speed",
        "single_extruder_multi_material_priming",
        "activate_air_filtration",
        "activate_air_filtration_during_print",
        "activate_air_filtration_on_completion",
        "during_print_exhaust_fan_speed",
        "complete_print_exhaust_fan_speed",
        "activate_chamber_temp_control",
        "manual_filament_change",
        "disable_m73",
        "use_firmware_retraction",
        "enable_long_retraction_when_cut",
        "long_retractions_when_cut",
        "retraction_distances_when_cut",
        "filament_long_retractions_when_cut",
        "filament_retraction_distances_when_cut",
        "grab_length",
        "bed_temperature_formula",
        "filament_notes",
        "process_notes",
        "printer_notes",
        "use_3mf",
        // Project metadata (intent ledger, provenance, composition report) reaches the G-code header
        // through full_print_config, but no slicing step reads it. The composed filament values are
        // diffed on their own keys.
        "mixed_nozzle_filament_explicit_keys",
        "mixed_nozzle_filament_provenance",
        "mixed_nozzle_filament_binding",
        // Same for the differs-from-system mask: only the G-code handoff and extruder-change prime read
        // it through full_print_config, and the values it marks are diffed on their own keys.
        "different_settings_to_system"
    };

    static std::unordered_set<std::string> steps_ignore;

    std::vector<PrintStep> steps;
    std::vector<PrintObjectStep> osteps;
    bool invalidated = false;
    const auto *new_mixed_nozzle_mode = new_config.option<ConfigOptionEnum<MixedNozzleSlicingMode>>("mixed_nozzle_slicing_mode");
    const bool mixed_nozzle_active = is_mixed_nozzle_slicing_enabled(m_config) ||
                                     (new_mixed_nozzle_mode != nullptr &&
                                      new_mixed_nozzle_mode->value != MixedNozzleSlicingMode::Off);
    const bool regional_layering_active = is_mixed_nozzle_body_split(m_config) ||
                                          (new_mixed_nozzle_mode != nullptr &&
                                           new_mixed_nozzle_mode->value == MixedNozzleSlicingMode::BodySplit);
    static constexpr std::array<const char *, 17> regional_layering_keys{
        "initial_layer_print_height", "nozzle_diameter", "min_layer_height", "max_layer_height",
        "print_sequence", "first_layer_print_sequence", "other_layers_print_sequence",
        "other_layers_print_sequence_nums", "toolchange_ordering", "enable_prime_tower",
        "spiral_mode", "resonance_avoidance", "filament_adaptive_volumetric_speed",
        "slicing_pipeline_plugin", "enable_filament_dynamic_map", "regional_grid_phase_rule",
        "mixed_nozzle_allowed_cadence_ratios"
    };

    for (const t_config_option_key &opt_key : opt_keys) {
        if (regional_layering_active && one_of(opt_key, regional_layering_keys))
            osteps.emplace_back(posSlice);

        if (mixed_nozzle_active && opt_key == "mixed_nozzle_coarse_layer_height")
            osteps.emplace_back(posSlice);

        if (opt_key == "mixed_nozzle_slicing_mode") {
            osteps.emplace_back(posSlice);
        } else if (steps_gcode.find(opt_key) != steps_gcode.end()) {
            // These options only affect G-code export or they are just notes without influence on the generated G-code,
            // so there is nothing to invalidate.
            steps.emplace_back(psGCodeExport);
        } else if (steps_ignore.find(opt_key) != steps_ignore.end()) {
            // These steps have no influence on the G-code whatsoever. Just ignore them.
        } else if (
               opt_key == "skirt_type"
            || opt_key == "skirt_loops"
            || opt_key == "skirt_speed"
            || opt_key == "skirt_height"
            || opt_key == "min_skirt_length"
            || opt_key == "single_loop_draft_shield"
            || opt_key == "draft_shield"
            || opt_key == "skirt_distance"
            || opt_key == "skirt_start_angle"
            || opt_key == "ooze_prevention"
            || opt_key == "wipe_tower_x"
            || opt_key == "wipe_tower_y"
            || opt_key == "wipe_tower_rotation_angle") {
            // The tower gcode itself is position-independent (position and rotation are applied
            // at export), except that the wait_for_temp_on_wipe_tower park bakes a bed-relative
            // side choice into it (WipeTower2::toolchange_Change) — regenerate it when the tower
            // moves. Gating on the old config is safe: both inputs of wait_for_temp_enabled
            // invalidate psWipeTower themselves when they are part of the same diff.
            if ((opt_key == "wipe_tower_x" || opt_key == "wipe_tower_y" || opt_key == "wipe_tower_rotation_angle")
                && WipeTower2::wait_for_temp_enabled(m_config))
                steps.emplace_back(psWipeTower);
            steps.emplace_back(psSkirtBrim);
        } else if (
               opt_key == "slicing_pipeline_plugin"
            || opt_key == "print_plugin_config_overrides"
            || opt_key == "regional_interface_tolerance"
            || opt_key == "initial_layer_print_height"
            || opt_key == "nozzle_diameter"
            || opt_key == "filament_shrink"
            || opt_key == "filament_shrinkage_compensation_z"
            || opt_key == "resolution"
            || opt_key == "precise_z_height"
            // Spiral Vase forces different kind of slicing than the normal model:
            // In Spiral Vase mode, holes are closed and only the largest area contour is kept at each layer.
            // Therefore toggling the Spiral Vase on / off requires complete reslicing.
            || opt_key == "spiral_mode") {
            osteps.emplace_back(posSlice);
        } else if (
               opt_key == "filament_map"
            || opt_key == "filament_map_mode") {
            // These decide which physical extruder each filament resolves to, and
            // SlicingParameters::create_from_config() reads that for layer height limits, so a remap has to
            // re-slice. filament_nozzle_map stays with the tower keys below: it only picks a sub-nozzle
            // inside the extruder filament_map already named, which no geometry reads.
            osteps.emplace_back(posSlice);
            steps.emplace_back(psWipeTower);
            steps.emplace_back(psSkirtBrim);
        } else if (
               opt_key == "print_sequence"
            || opt_key == "filament_type"
            || opt_key == "chamber_temperature"
            || opt_key == "nozzle_temperature_initial_layer"
            || opt_key == "filament_minimal_purge_on_wipe_tower"
            || opt_key == "filament_max_volumetric_speed"
            || opt_key == "filament_adaptive_volumetric_speed"
            || opt_key == "filament_loading_speed"
            || opt_key == "filament_loading_speed_start"
            || opt_key == "filament_unloading_speed"
            || opt_key == "filament_unloading_speed_start"
            || opt_key == "filament_toolchange_delay"
            || opt_key == "filament_cooling_moves"
            || opt_key == "filament_stamping_loading_speed"
            || opt_key == "filament_stamping_distance"
            || opt_key == "filament_cooling_initial_speed"
            || opt_key == "filament_cooling_final_speed"
            || opt_key == "filament_ramming_parameters"
            || opt_key == "filament_multitool_ramming"
            || opt_key == "filament_multitool_ramming_volume"
            || opt_key == "filament_multitool_ramming_flow"
            || opt_key == "filament_max_volumetric_speed"
            || opt_key == "gcode_flavor"
            || opt_key == "single_extruder_multi_material"
            || opt_key == "nozzle_temperature"
            // BBS
            || opt_key == "supertack_plate_temp"
            || opt_key == "cool_plate_temp"
            || opt_key == "textured_cool_plate_temp"
            || opt_key == "eng_plate_temp"
            || opt_key == "hot_plate_temp"
            || opt_key == "textured_plate_temp"
            || opt_key == "enable_prime_tower"
            || opt_key == "enable_wrapping_detection"
            || opt_key == "prime_tower_enable_framework"
            || opt_key == "prime_tower_width"
            || opt_key == "prime_tower_brim_width"
            || opt_key == "wipe_tower_type"
            || opt_key == "prime_tower_skip_points"
            || opt_key == "prime_tower_flat_ironing"
            || opt_key == "enable_tower_interface_features"
            || opt_key == "first_layer_print_sequence"
            || opt_key == "other_layers_print_sequence"
            || opt_key == "other_layers_print_sequence_nums" 
            || opt_key == "toolchange_ordering"
            || opt_key == "extruder_ams_count"
            || opt_key == "extruder_nozzle_stats"
            || opt_key == "filament_nozzle_map"
            || opt_key == "filament_volume_map"
            || opt_key == "filament_adhesiveness_category"
            || opt_key == "filament_tower_interface_pre_extrusion_dist"
            || opt_key == "filament_tower_interface_pre_extrusion_length"
            || opt_key == "filament_tower_ironing_area"
            || opt_key == "filament_tower_interface_purge_volume"
            || opt_key == "filament_tower_interface_print_temp"
            || opt_key == "wipe_tower_bridging"
            || opt_key == "wipe_tower_extra_flow"
            || opt_key == "wipe_tower_no_sparse_layers"
            || opt_key == "flush_volumes_matrix"
            || opt_key == "prime_volume"
            || opt_key == "mixed_nozzle_extruder_change_prime_volume"
            || opt_key == "flush_into_infill"
            || opt_key == "flush_into_support"
            || opt_key == "initial_layer_infill_speed"
            || opt_key == "travel_speed"
            || opt_key == "travel_speed_z"
            || opt_key == "initial_layer_speed"
            || opt_key == "initial_layer_travel_speed"
            || opt_key == "initial_layer_travel_acceleration"
            || opt_key == "initial_layer_travel_jerk"
            || opt_key == "slow_down_layers"
            || opt_key == "idle_temperature"
            || opt_key == "wipe_tower_cone_angle"
            || opt_key == "wipe_tower_extra_spacing"
            || opt_key == "wipe_tower_max_purge_speed"
            || opt_key == "wipe_tower_wall_type"
            || opt_key == "wipe_tower_extra_rib_length"
            || opt_key == "wipe_tower_rib_width"
            || opt_key == "wipe_tower_fillet_wall"
            || opt_key == "wipe_tower_filament"
            || opt_key == "wiping_volumes_extruders"
            || opt_key == "enable_filament_ramming"
            || opt_key == "tool_change_on_wipe_tower"
            || opt_key == "wait_for_temp_on_wipe_tower"
            || opt_key == "purge_in_prime_tower"
            || opt_key == "z_offset"
            || opt_key == "support_multi_bed_types"
            ) {
            steps.emplace_back(psWipeTower);
            steps.emplace_back(psSkirtBrim);
        } else if (opt_key == "filament_soluble"
                || opt_key == "filament_is_support"
                || opt_key == "filament_printable"
                || opt_key == "filament_change_length"
                || opt_key == "independent_support_layer_height") {
            steps.emplace_back(psWipeTower);
            // Soluble support interface / non-soluble base interface produces non-soluble interface layers below soluble interface layers.
            // Thus switching between soluble / non-soluble interface layer material may require recalculation of supports.
            //FIXME Killing supports on any change of "filament_soluble" is rough. We should check for each object whether that is necessary.
            osteps.emplace_back(posSupportMaterial);
            osteps.emplace_back(posSimplifySupportPath);
        } else if (
               opt_key == "initial_layer_line_width"
            || opt_key == "min_layer_height"
            || opt_key == "max_layer_height"
            //|| opt_key == "resolution"
            //BBS: when enable arc fitting, we must re-generate perimeter
            || opt_key == "enable_arc_fitting"
            || opt_key == "print_order"
            || opt_key == "wall_sequence") {
            osteps.emplace_back(posPerimeters);
            osteps.emplace_back(posEstimateCurledExtrusions);
            osteps.emplace_back(posInfill);
            osteps.emplace_back(posSupportMaterial);
			osteps.emplace_back(posSimplifyPath);
            osteps.emplace_back(posSimplifyInfill);
            osteps.emplace_back(posSimplifySupportPath);
            steps.emplace_back(psSkirtBrim);
            // Under a mode the nozzles' layer height limits decide the object's own layers (the cadence and
            // the support's limits in SlicingParameters::create_from_config()), and a raft's levels and first
            // layer, so the object is sliced again.
            if ((opt_key == "min_layer_height" || opt_key == "max_layer_height") && is_mixed_nozzle_slicing_enabled(m_config))
                osteps.emplace_back(posSlice);
        }
        else if (opt_key == "z_hop_types") {
            osteps.emplace_back(posDetectOverhangsForLift);
        } else {
            // for legacy, if we can't handle this option let's invalidate all steps
            //FIXME invalidate all steps of all objects as well?
            invalidated |= this->invalidate_all_steps();
            // Continue with the other opt_keys to possibly invalidate any object specific steps.
        }
    }

    sort_remove_duplicates(steps);
    for (PrintStep step : steps)
        invalidated |= this->invalidate_step(step);
    sort_remove_duplicates(osteps);
    for (PrintObjectStep ostep : osteps)
        for (PrintObject *object : m_objects)
            invalidated |= object->invalidate_step(ostep);

    return invalidated;
}

void Print::set_calib_params(const Calib_Params& params) {
    m_calib_params = params;
    m_calib_params.mode = params.mode;
}

bool Print::invalidate_step(PrintStep step)
{
    // Cost-affecting edits, including tower-only and filament speed edits, must reopen the native
    // geometry choice.
    std::vector<PrintObject *> reconsider;
    for (PrintObject *object : m_objects)
        if (object->m_feature_economics_applied) {
            object->m_feature_economics_applied = false;
            reconsider.push_back(object);
        }
    for (PrintObject *object : reconsider)
        object->invalidate_step(posPrepareInfill);
	bool invalidated = Inherited::invalidate_step(step);
    // Propagate to dependent steps.
    if (step != psGCodeExport)
        invalidated |= Inherited::invalidate_step(psGCodeExport);
    return invalidated;
}

// returns true if an object step is done on all objects
// and there's at least one object
bool Print::is_step_done(PrintObjectStep step) const
{
    if (m_objects.empty())
        return false;
    std::scoped_lock<std::mutex> lock(this->state_mutex());
    for (const PrintObject *object : m_objects)
        if (! object->is_step_done_unguarded(step))
            return false;
    return true;
}

// returns 0-based indices of used extruders
std::vector<unsigned int> Print::object_extruders() const
{
    std::vector<unsigned int> extruders;
    extruders.reserve(m_print_regions.size() * m_objects.size() * 3);

    //Orca: Collect extruders from all regions.
    for (const PrintObject *object : m_objects)
		for (const PrintRegion &region : object->all_regions())
        	region.collect_object_printing_extruders(*this, extruders);

    for (const PrintObject* object : m_objects) {
        const ModelObject* mo = object->model_object();
        for (const ModelVolume* mv : mo->volumes) {
            std::vector<int> volume_extruders = mv->get_extruders();
            for (int extruder : volume_extruders) {
                assert(extruder > 0);
                extruders.push_back(extruder - 1);
            }
        }

        // layer range
        for (auto layer_range : mo->layer_config_ranges) {
            if (layer_range.second.has("extruder")) {
                //BBS: actually when user doesn't change filament by height range(value is default 0), height range should not save key "extruder".
                //Don't know why height range always save key "extruder" because of no change(should only save difference)...
                //Add protection here to avoid overflow
                auto value = layer_range.second.option("extruder")->getInt();
                if (value > 0)
                    extruders.push_back(value - 1);
            }
        }
    }
    sort_remove_duplicates(extruders);
    return extruders;
}

// returns 0-based indices of used extruders
std::vector<unsigned int> Print::support_material_extruders() const
{
    std::vector<unsigned int> extruders;
    bool support_uses_current_extruder = false;
    // BBS
    auto num_extruders = (unsigned int)m_config.filament_diameter.size();

    for (PrintObject *object : m_objects) {
        if (object->has_support_material()) {
        	assert(object->config().support_filament >= 0);
            if (object->config().support_filament == 0)
                support_uses_current_extruder = true;
            else {
            	unsigned int i = (unsigned int)object->config().support_filament - 1;
                extruders.emplace_back((i >= num_extruders) ? 0 : i);
            }
        	assert(object->config().support_interface_filament >= 0);
            if (object->config().support_interface_filament == 0)
                support_uses_current_extruder = true;
            else {
            	unsigned int i = (unsigned int)object->config().support_interface_filament - 1;
                extruders.emplace_back((i >= num_extruders) ? 0 : i);
            }
        }
    }

    if (support_uses_current_extruder)
        // Add all object extruders to the support extruders as it is not know which one will be used to print supports.
        append(extruders, this->object_extruders());

    sort_remove_duplicates(extruders);
    return extruders;
}

// returns 0-based indices of used extruders
std::vector<unsigned int> Print::extruders(bool conside_custom_gcode) const
{
    std::vector<unsigned int> extruders = this->object_extruders();
    append(extruders, this->support_material_extruders());

    if (conside_custom_gcode) {
        //BBS
        // Size by the filament domain, not the colour vector: a ToolChange past the end of an unset
        // colour list would otherwise be dropped from the print's extruder set.
        int num_extruders = m_config.filament_diameter.size();
        if (m_model.plates_custom_gcodes.find(m_model.curr_plate_index) != m_model.plates_custom_gcodes.end()) {
            for (auto item : m_model.plates_custom_gcodes.at(m_model.curr_plate_index).gcodes) {
                if (item.type == CustomGCode::Type::ToolChange && item.extruder <= num_extruders)
                    extruders.push_back((unsigned int)(item.extruder - 1));
            }
        }
    }

    // If a wipe tower filament is explicitly set, ensure it participates in tool ordering.
    if (has_wipe_tower() && config().wipe_tower_filament != 0 && extruders.size() > 1) {
        assert(config().wipe_tower_filament > 0 && config().wipe_tower_filament < int(config().nozzle_diameter.size()));
        extruders.emplace_back(config().wipe_tower_filament - 1); // config value is 1-based
    }

    sort_remove_duplicates(extruders);
    return extruders;
}

unsigned int Print::num_object_instances() const
{
	unsigned int instances = 0;
    for (const PrintObject *print_object : m_objects)
        instances += (unsigned int)print_object->instances().size();
    return instances;
}

double Print::max_allowed_layer_height() const
{
    double nozzle_diameter_max = 0.;
    for (unsigned int extruder_id : this->extruders())
        nozzle_diameter_max = std::max(nozzle_diameter_max, m_config.nozzle_diameter.get_at(extruder_id));
    return nozzle_diameter_max;
}

std::vector<ObjectID> Print::print_object_ids() const
{
    std::vector<ObjectID> out;
    // Reserve one more for the caller to append the ID of the Print itself.
    out.reserve(m_objects.size() + 1);
    for (const PrintObject *print_object : m_objects)
        out.emplace_back(print_object->id());
    return out;
}

bool Print::has_infinite_skirt() const
{
    // Orca: unclear why (m_config.ooze_prevention && this->extruders().size() > 1) logic is here, removed.
    // return (m_config.draft_shield == dsEnabled && m_config.skirt_loops > 0) || (m_config.ooze_prevention && this->extruders().size() > 1);

    return (m_config.draft_shield == dsEnabled && m_config.skirt_loops > 0);
}

bool Print::has_skirt() const
{
    return (m_config.skirt_height > 0);
}

bool Print::has_brim() const
{
    return std::any_of(m_objects.begin(), m_objects.end(), [](PrintObject *object) { return object->has_brim(); });
}

//BBS
std::vector<size_t> Print::layers_sorted_for_object(float start, float end, std::vector<LayerPtrs> &layers_of_objects, std::vector<BoundingBox> &boundingBox_for_objects, VecOfPoints &objects_instances_shift)
{
    std::vector<size_t> idx_of_object_sorted;
    size_t              idx = 0;
    for (const auto &object : m_objects) {
        idx_of_object_sorted.push_back(idx++);
        object->get_certain_layers(start, end, layers_of_objects, boundingBox_for_objects);
    }
    std::sort(idx_of_object_sorted.begin(), idx_of_object_sorted.end(),
              [boundingBox_for_objects](auto left, auto right) { return boundingBox_for_objects[left].area() > boundingBox_for_objects[right].area(); });

    objects_instances_shift.clear();
    objects_instances_shift.reserve(m_objects.size());
    for (const auto& object : m_objects)
        objects_instances_shift.emplace_back(object->get_instances_shift_without_plate_offset());

    return idx_of_object_sorted;
};

StringObjectException Print::sequential_print_clearance_valid(const Print &print, Polygons *polygons, std::vector<std::pair<Polygon, float>>* height_polygons)
{
    StringObjectException single_object_exception;
    const auto& print_config = print.config();
    Polygons exclude_polys = get_bed_excluded_area(print_config);
    const Vec3d print_origin = print.get_plate_origin();
    std::for_each(exclude_polys.begin(), exclude_polys.end(),
                  [&print_origin](Polygon& p) { p.translate(scale_(print_origin.x()), scale_(print_origin.y())); });

    struct print_instance_info
    {
        const PrintInstance *print_instance;
        BoundingBox    bounding_box;
        Polygon        hull_polygon;
        int                  object_index;
        double         arrange_score;
        double               height;
    };
    auto find_object_index = [](const Model& model, const ModelObject* obj) {
        for (int index = 0; index < model.objects.size(); index++)
        {
            if (model.objects[index] == obj)
                return index;
        }
        return -1;
    };

    auto [object_skirt_offset, _] = print.object_skirt_offset();
    std::vector<struct print_instance_info> print_instance_with_bounding_box;
    {
        // sequential_print_horizontal_clearance_valid
        Polygons convex_hulls_other;
        if (polygons != nullptr)
            polygons->clear();
        std::vector<size_t> intersecting_idxs;

        // Shrink the extruder_clearance_radius a tiny bit, so that if the object arrangement algorithm placed the objects
        // exactly by satisfying the extruder_clearance_radius, this test will not trigger collision.
        float obj_distance = print.is_all_objects_are_short() ? scale_(std::max(0.5f * MAX_OUTER_NOZZLE_DIAMETER, object_skirt_offset) - 0.1) : scale_(0.5 * print.config().extruder_clearance_radius.value + object_skirt_offset - 0.1);

        for (const PrintObject *print_object : print.objects()) {
            assert(! print_object->model_object()->instances.empty());
            assert(! print_object->instances().empty());
            
            // Orca: check convex hull intersection for each instance individually to handle rotation/offset differences correctly
            // Now we check that no instance of convex_hull intersects any of the previously checked object instances.
            for (const PrintInstance &instance : print_object->instances()) {
                Polygon convex_hull0 = print_object->model_object()->convex_hull_2d(Geometry::assemble_transform(
                            { 0.0, 0.0, instance.model_instance->get_offset().z() }, instance.model_instance->get_rotation(), instance.model_instance->get_scaling_factor(), instance.model_instance->get_mirror()));

                Polygon convex_hull_no_offset = convex_hull0, convex_hull;
                auto tmp = offset(convex_hull_no_offset, obj_distance, jtRound, scale_(0.1));
                if (!tmp.empty()) { // tmp may be empty due to clipper's bug, see STUDIO-2452
                    convex_hull = tmp.front();
                    // instance.shift is a position of a centered object, while model object may not be centered.
                    // Convert the shift from the PrintObject's coordinates into ModelObject's coordinates by removing the centering offset.
                    convex_hull.translate(instance.shift - print_object->center_offset());
                }
                convex_hull_no_offset.translate(instance.shift - print_object->center_offset());
                //juedge the exclude area
                if (!intersection(exclude_polys, convex_hull_no_offset).empty()) {
                    if (single_object_exception.string.empty()) {
                        single_object_exception.string = (boost::format(L("%1% is too close to exclusion area. There may be collisions when printing.")) %instance.model_instance->get_object()->name).str();
                        // single_object_exception.object = instance.model_instance->get_object();
                        //ORCA: Pass ModelInstance instead of ModelObject
                        single_object_exception.object = instance.model_instance;
                    }
                    else {
                        single_object_exception.string += "\n"+(boost::format(L("%1% is too close to exclusion area. There may be collisions when printing.")) %instance.model_instance->get_object()->name).str();
                        single_object_exception.object = nullptr;
                    }
                    //if (polygons) {
                    //    intersecting_idxs.emplace_back(convex_hulls_other.size());
                    //}
                }

                // if output needed, collect indices (inside convex_hulls_other) of intersecting hulls
                for (size_t i = 0; i < convex_hulls_other.size(); ++i) {
                    if (! intersection(convex_hulls_other[i], convex_hull).empty()) {
                        bool has_exception = false;
                        if (single_object_exception.string.empty()) {
                            single_object_exception.string = (boost::format(L("%1% is too close to others, and collisions may be caused.")) %instance.model_instance->get_object()->name).str();
                            // single_object_exception.object = instance.model_instance->get_object();
                            //ORCA: Pass ModelInstance instead of ModelObject for better selection
                            single_object_exception.object = instance.model_instance;
                            has_exception                  = true;
                        }
                        else {
                            single_object_exception.string += "\n"+(boost::format(L("%1% is too close to others, and collisions may be caused.")) %instance.model_instance->get_object()->name).str();
                            // single_object_exception.object = nullptr; 
                            // ORCA: Keep the first object so jump works
                            // has_exception                  = true;
                            has_exception                  = true;
                        }

                        if (polygons) {
                            intersecting_idxs.emplace_back(i);
                            intersecting_idxs.emplace_back(convex_hulls_other.size());
                        }

                        if (has_exception) break;
                    }
                }
                struct print_instance_info print_info {&instance, convex_hull.bounding_box(), convex_hull};
                print_info.height = instance.print_object->height();
                print_info.object_index = find_object_index(print.model(), print_object->model_object());
                print_instance_with_bounding_box.push_back(std::move(print_info));
                convex_hulls_other.emplace_back(std::move(convex_hull));
            }
        }
        if (!intersecting_idxs.empty()) {
            // use collected indices (inside convex_hulls_other) to update output
            std::sort(intersecting_idxs.begin(), intersecting_idxs.end());
            intersecting_idxs.erase(std::unique(intersecting_idxs.begin(), intersecting_idxs.end()), intersecting_idxs.end());
            for (size_t i : intersecting_idxs) {
                polygons->emplace_back(std::move(convex_hulls_other[i]));
            }
        }
    }

    // calc sort order
    double hc1              = scale_(print.config().extruder_clearance_height_to_lid); // height to lid
    double hc2              = scale_(print.config().extruder_clearance_height_to_rod); // height to rod
    double printable_height = scale_(print.config().printable_height);

#if 0 //do not sort anymore, use the order in object list
    auto bed_points = get_bed_shape(print_config);
    float bed_width = bed_points[1].x() - bed_points[0].x();
    // 如果扩大以后的多边形的距离小于这个值，就需要严格保证从左到右的打印顺序，否则会撞工具头右侧
    float unsafe_dist = scale_(print_config.extruder_clearance_max_radius.value - print_config.extruder_clearance_radius.value);
    struct VecHash
    {
        size_t operator()(const Vec2i32 &n1) const
        {
            return std::hash<coord_t>()(int(n1(0) * 100 + 100)) + std::hash<coord_t>()(int(n1(1) * 100 + 100)) * 101;
        }
    };
    std::unordered_set<Vec2i32, VecHash> left_right_pair; // pairs in this vector must strictly obey the left-right order
    for (size_t i = 0; i < print_instance_with_bounding_box.size();i++) {
        auto &inst         = print_instance_with_bounding_box[i];
        inst.index         = i;
        Point pt           = inst.bounding_box.center();
        inst.arrange_score = pt.x() / 2 + pt.y(); // we prefer print row-by-row, so cost on x-direction is smaller
    }
    for (size_t i = 0; i < print_instance_with_bounding_box.size(); i++) {
        auto &inst         = print_instance_with_bounding_box[i];
        auto &l            = print_instance_with_bounding_box[i];
        for (size_t j = 0; j < print_instance_with_bounding_box.size(); j++) {
            if (j != i) {
                auto &r        = print_instance_with_bounding_box[j];
                auto ly1       = l.bounding_box.min.y();
                auto ly2       = l.bounding_box.max.y();
                auto ry1       = r.bounding_box.min.y();
                auto ry2       = r.bounding_box.max.y();
                auto lx1       = l.bounding_box.min.x();
                auto rx1       = r.bounding_box.min.x();
                auto lx2       = l.bounding_box.max.x();
                auto rx2       = r.bounding_box.max.x();
                auto inter_min = std::max(ly1, ry1);
                auto inter_max = std::min(ly2, ry2);
                auto inter_y   = inter_max - inter_min;

                // 如果y方向的重合超过轮廓的膨胀量，说明两个物体在一行，应该先打左边的物体，即先比较二者的x坐标。
                // If the overlap in the y direction exceeds the expansion of the contour, it means that the two objects are in a row and the object on the left should be hit first, that is, the x coordinates of the two should be compared first.
                if (inter_y > scale_(0.5 * print.config().extruder_clearance_radius.value)) {
                    if (std::max(rx1 - lx2, lx1 - rx2) < unsafe_dist) {
                        if (lx1 > rx1) {
                            left_right_pair.insert({j, i});
                            BOOST_LOG_TRIVIAL(debug) << "in-a-row, print_instance " << r.print_instance->model_instance->get_object()->name << "(" << r.arrange_score << ")"
                                                     << " -> " << l.print_instance->model_instance->get_object()->name << "(" << l.arrange_score << ")";
                        } else {
                            left_right_pair.insert({i, j});
                            BOOST_LOG_TRIVIAL(debug) << "in-a-row, print_instance " << l.print_instance->model_instance->get_object()->name << "(" << l.arrange_score << ")"
                                                     << " -> " << r.print_instance->model_instance->get_object()->name << "(" << r.arrange_score << ")";
                        }
                    }
                }
                if (l.height > hc1 && r.height < hc1) {
                    // 当前物体超过了顶盖高度，必须后打
                    left_right_pair.insert({j, i});
                    BOOST_LOG_TRIVIAL(debug) << "height>hc1, print_instance " << r.print_instance->model_instance->get_object()->name << "(" << r.arrange_score << ")"
                                             << " -> " << l.print_instance->model_instance->get_object()->name << "(" << l.arrange_score << ")";
                }
                else if (l.height > hc2 && l.height > r.height && l.arrange_score<r.arrange_score) {
                    // 如果当前物体的高度超过滑杆，且比r高，就给它加一点代价，尽量让高的物体后打（只有物体高度超过滑杆时才有必要按高度来）
                    if (l.arrange_score < r.arrange_score)
                        l.arrange_score = r.arrange_score + 10;
                    BOOST_LOG_TRIVIAL(debug) << "height>hc2, print_instance " << inst.print_instance->model_instance->get_object()->name
                                             << ", right=" << r.print_instance->model_instance->get_object()->name << ", l.score: " << l.arrange_score
                                             << ", r.score: " << r.arrange_score;
                }
            }
        }
    }
    // 多做几次代价传播，因为前一次有些值没有更新。
    // TODO 更好的办法是建立一颗树，一步到位。不过我暂时没精力搞，先就这样吧
    for (int k=0;k<5;k++)
    for (auto p : left_right_pair) {
        auto &l = print_instance_with_bounding_box[p(0)];
        auto &r = print_instance_with_bounding_box[p(1)];
        if(r.arrange_score<l.arrange_score)
            r.arrange_score = l.arrange_score + 10;
    }

    BOOST_LOG_TRIVIAL(debug) << "bed width: " << unscale_(bed_width) << ", unsafe_dist:" << unscale_(unsafe_dist) << ", height_to_lid: " << unscale_(hc1) << ", height_to_rod:" << unscale_(hc2) << ", final dependency:";
    for (auto p : left_right_pair) {
        auto &l         = print_instance_with_bounding_box[p(0)];
        auto &r         = print_instance_with_bounding_box[p(1)];
        BOOST_LOG_TRIVIAL(debug) << "print_instance " << I18N::translate(l.print_instance->model_instance->get_object()->name) << "(" << l.arrange_score << ")"
                                 << " -> " << I18N::translate(r.print_instance->model_instance->get_object()->name) << "(" << r.arrange_score << ")";
    }
    // sort the print instance
    std::sort(print_instance_with_bounding_box.begin(), print_instance_with_bounding_box.end(),
        [](print_instance_info& l, print_instance_info& r) {return l.arrange_score < r.arrange_score;});

    for (auto &inst : print_instance_with_bounding_box)
        BOOST_LOG_TRIVIAL(debug) << "after sorting print_instance " << inst.print_instance->model_instance->get_object()->name << ", score: " << inst.arrange_score
                                 << ", height:"<< inst.height;
#else
    // sort the print instance
    std::sort(print_instance_with_bounding_box.begin(), print_instance_with_bounding_box.end(),
        [](print_instance_info& l, print_instance_info& r) {return l.object_index < r.object_index;});

    for (auto &inst : print_instance_with_bounding_box)
        BOOST_LOG_TRIVIAL(debug) << "after sorting print_instance " << inst.print_instance->model_instance->get_object()->name << ", object_index: " << inst.object_index
                                 << ", height:"<< inst.height;

#endif
    // sequential_print_vertical_clearance_valid
    {
        // Ignore the last instance printed.
        //print_instance_with_bounding_box.pop_back();
        /*bool has_interlaced_objects = false;
        for (int k = 0; k < print_instance_count; k++)
        {
            auto inst = print_instance_with_bounding_box[k].print_instance;
            auto bbox = print_instance_with_bounding_box[k].bounding_box;
            auto iy1 = bbox.min.y();
            auto iy2 = bbox.max.y();

            for (int i = 0; i < k; i++)
            {
                auto& p = print_instance_with_bounding_box[i].print_instance;
                auto bbox2 = print_instance_with_bounding_box[i].bounding_box;
                auto py1 = bbox2.min.y();
                auto py2 = bbox2.max.y();
                auto inter_min = std::max(iy1, py1); // min y of intersection
                auto inter_max = std::min(iy2, py2); // max y of intersection. length=max_y-min_y>0 means intersection exists
                if (inter_max - inter_min > 0) {
                    has_interlaced_objects = true;
                    break;
                }
            }
            if (has_interlaced_objects)
                break;
        }*/

        // if objects are not overlapped on y-axis, they will not collide even if they are taller than extruder_clearance_height_to_rod
        int print_instance_count = print_instance_with_bounding_box.size();
        std::map<const PrintInstance*, std::pair<Polygon, float>> too_tall_instances;
        for (int k = 0; k < print_instance_count; k++)
        {
            auto inst = print_instance_with_bounding_box[k].print_instance;
            // 只需要考虑喷嘴到滑杆的偏移量，这个比整个工具头的碰撞半径要小得多
            // Only the offset from the nozzle to the slide bar needs to be considered, which is much smaller than the collision radius of the entire tool head.
            auto bbox = print_instance_with_bounding_box[k].bounding_box.inflated(-scale_(0.5 * print.config().extruder_clearance_radius.value + object_skirt_offset));
            auto iy1 = bbox.min.y();
            auto iy2 = bbox.max.y();
            (const_cast<ModelInstance*>(inst->model_instance))->arrange_order = k+1;
            double height = (k == (print_instance_count - 1))?printable_height:hc1;
            /*if (has_interlaced_objects) {
                if ((k < (print_instance_count - 1)) && (inst->print_object->height() > hc2)) {
                    too_tall_instances[inst] = std::make_pair(print_instance_with_bounding_box[k].hull_polygon, unscaled<double>(hc2));
                }
            }
            else {
                if ((k < (print_instance_count - 1)) && (inst->print_object->height() > hc1)) {
                    too_tall_instances[inst] = std::make_pair(print_instance_with_bounding_box[k].hull_polygon, unscaled<double>(hc1));
                }
            }*/

            for (int i = k+1; i < print_instance_count; i++)
            {
                auto& p = print_instance_with_bounding_box[i].print_instance;
                auto bbox2 = print_instance_with_bounding_box[i].bounding_box;
                auto py1 = bbox2.min.y();
                auto py2 = bbox2.max.y();
                auto inter_min = std::max(iy1, py1); // min y of intersection
                auto inter_max = std::min(iy2, py2); // max y of intersection. length=max_y-min_y>0 means intersection exists
                if (inter_max - inter_min > 0) {
                    height = hc2;
                    break;
                }
            }
            if (height < inst->print_object->max_z())
                too_tall_instances[inst] = std::make_pair(print_instance_with_bounding_box[k].hull_polygon, unscaled<double>(height));
        }

        if (too_tall_instances.size() > 0) {
            //return {, inst->model_instance->get_object()};
            for (auto& iter: too_tall_instances) {
                if (single_object_exception.string.empty()) {
                    single_object_exception.string = (boost::format(L("%1% is too tall, and collisions will be caused.")) %iter.first->model_instance->get_object()->name).str();
                    single_object_exception.object = iter.first->model_instance->get_object();
                }
                else {
                    single_object_exception.string += "\n" + (boost::format(L("%1% is too tall, and collisions will be caused.")) %iter.first->model_instance->get_object()->name).str();
                    single_object_exception.object = nullptr;
                }
                if (height_polygons)
                    height_polygons->emplace_back(std::move(iter.second));
            }
        }
    }

    return single_object_exception;
}

//BBS
static StringObjectException layered_print_cleareance_valid(const Print &print, StringObjectException *warning)
{
    std::vector<const PrintInstance*> print_instances_ordered = sort_object_instances_by_model_order(print, true);
    if (print_instances_ordered.size() < 1)
        return {};

    const auto& print_config = print.config();
    Polygons exclude_polys = get_bed_excluded_area(print_config);
    const Vec3d print_origin = print.get_plate_origin();
    std::for_each(exclude_polys.begin(), exclude_polys.end(),
                  [&print_origin](Polygon& p) { p.translate(scale_(print_origin.x()), scale_(print_origin.y())); });

    Pointfs wrapping_detection_area = print_config.wrapping_exclude_area.values;
    Polygon wrapping_poly;
    for (size_t i = 0; i < wrapping_detection_area.size(); ++i) {
        auto pt = wrapping_detection_area[i];
        wrapping_poly.points.emplace_back(scale_(pt.x() + print_origin.x()), scale_(pt.y() + print_origin.y()));
    }

    Polygons convex_hulls_other;
    // Orca: check convex hull intersection for each instance individually
    for (auto& inst : print_instances_ordered) {
        Polygons current_instance_hulls;
        for (const ModelVolume *v : inst->print_object->model_object()->volumes) {
            if (!v->is_model_part()) continue;
            
            auto volume_hull = v->get_convex_hull_2d(Geometry::assemble_transform(Vec3d::Zero(), inst->model_instance->get_rotation(),
                                                                                  inst->model_instance->get_scaling_factor(), inst->model_instance->get_mirror()));
            volume_hull.translate(inst->shift - inst->print_object->center_offset());

            if (!intersection(exclude_polys, volume_hull).empty()) {
                // return {inst->model_instance->get_object()->name + L(" is too close to exclusion area, there may be collisions when printing.") + "\n",
                //        inst->model_instance->get_object()};
                //ORCA: Pass ModelInstance instead of ModelObject
                return {inst->model_instance->get_object()->name + L(" is too close to exclusion area, there may be collisions when printing.") + "\n",
                        inst->model_instance};
            }

            if (print_config.enable_wrapping_detection.value && !intersection(wrapping_poly, volume_hull).empty()) {
                // return {inst->model_instance->get_object()->name + L(" is too close to clumping detection area, there may be collisions when printing.") + "\n",
                //        inst->model_instance->get_object()};
                //ORCA: Pass ModelInstance instead of ModelObject
                return {inst->model_instance->get_object()->name + L(" is too close to clumping detection area, there may be collisions when printing.") + "\n",
                        inst->model_instance};
            }
            current_instance_hulls.emplace_back(volume_hull);
        }

        if (!intersection(convex_hulls_other, current_instance_hulls).empty()) {
            if (warning) {
                if (warning->string.empty()) {
                    warning->string = (boost::format(L("%1% is too close to others, and collisions may be caused.")) % inst->model_instance->get_object()->name).str();
                    // warning->object = inst->model_instance->get_object();
                    //ORCA: Pass ModelInstance instead of ModelObject for better selection
                    warning->object = inst->model_instance;
                } else {
                    warning->string += "\n" + (boost::format(L("%1% is too close to others, and collisions may be caused.")) % inst->model_instance->get_object()->name).str();
                    // ORCA: Keep the first object so jump works
                    if (!warning->object) warning->object = inst->model_instance;
                }
                warning->is_warning = true;
                warning->type = STRING_EXCEPT_OBJECT_COLLISION_IN_LAYER_PRINT;
            }
        }
        append(convex_hulls_other, current_instance_hulls);
    }

    //BBS: add the wipe tower check logic
    const PrintConfig &       config   = print.config();
    int                 filaments_count = print.extruders().size();
    int                 plate_index = print.get_plate_index();
    const Vec3d         plate_origin = print.get_plate_origin();
    float               x            = config.wipe_tower_x.get_at(plate_index) + plate_origin(0);
    float               y            = config.wipe_tower_y.get_at(plate_index) + plate_origin(1);
    float               width        = config.prime_tower_width.value;
    float               a            = config.wipe_tower_rotation_angle.value;
    //float               v            = config.wiping_volume.value;

    float        depth                     = print.wipe_tower_data(filaments_count).depth;
    //float        brim_width                = print.wipe_tower_data(filaments_count).brim_width;

    if (config.wipe_tower_wall_type.value == WipeTowerWallType::wtwRib)
        width = depth;

    Polygons convex_hulls_temp;
    if (print.has_wipe_tower()) {
        if (!print.is_step_done(psWipeTower)) {
            Polygon wipe_tower_convex_hull;
            wipe_tower_convex_hull.points.emplace_back(scale_(x), scale_(y));
            wipe_tower_convex_hull.points.emplace_back(scale_(x + width), scale_(y));
            wipe_tower_convex_hull.points.emplace_back(scale_(x + width), scale_(y + depth));
            wipe_tower_convex_hull.points.emplace_back(scale_(x), scale_(y + depth));
            wipe_tower_convex_hull.rotate(Geometry::deg2rad(a), Point(scale_(x), scale_(y)));
            convex_hulls_temp.push_back(wipe_tower_convex_hull);
        } else {
            //here, wipe_tower_polygon is not always convex.
            Polygon wipe_tower_polygon;
            if (print.wipe_tower_data().wipe_tower_mesh_data)
                wipe_tower_polygon = print.wipe_tower_data().wipe_tower_mesh_data->bottom;
            wipe_tower_polygon.rotate(Geometry::deg2rad(a));
            wipe_tower_polygon.translate(Point(scale_(x), scale_(y)));
            convex_hulls_temp.push_back(wipe_tower_polygon);
        }
    }
    if (!intersection(convex_hulls_other, convex_hulls_temp).empty()) {
        if (warning) {
            warning->string += L("Prime Tower") + L(" is too close to others, and collisions may be caused.\n");
        }
    }
    if (!intersection(exclude_polys, convex_hulls_temp).empty()) {
        /*if (warning) {
            warning->string += L("Prime Tower is too close to exclusion area, there may be collisions when printing.\n");
        }*/
        return {L("Prime Tower") + L(" is too close to an exclusion area, and collisions will be caused.\n")};
    }
    if (print_config.enable_wrapping_detection.value && !intersection({wrapping_poly}, convex_hulls_temp).empty()) {
        return {L("Prime Tower") + L(" is too close to clumping detection area, and collisions will be caused.\n")};
    }
    // Skip the containment check for towers that will never be printed (single-filament
    // prints without smooth timelapse keep the config's tower position but emit nothing).
    // Pre-generation only the body square is tested — the auto-brim estimate can overshoot
    // the generated brim by several mm and must not hard-fail a print that physically fits.
    // Post-generation the mesh bottom already includes the real brim, so the exact
    // footprint is tested.
    if (filaments_count > 1 || print.enable_timelapse_print()) {
        // The shared printable polygon is plate-local, while the tower polygons above are
        // already shifted by the plate origin.
        Polygons    printable_polys = print.get_extruder_shared_printable_polygon();
        const Point plate_shift(scale_(plate_origin.x()), scale_(plate_origin.y()));
        for (Polygon &p : printable_polys)
            p.translate(plate_shift);
        if (!diff(convex_hulls_temp, printable_polys).empty())
            return {L("Prime Tower") + L(" is partially outside the printable area, and it cannot be printed.\n")};
    }
    return {};
}

FilamentCompatibilityType Print::check_multi_filaments_compatibility(
    const std::vector<std::string>& filament_types,
    const std::vector<int>& nozzle_temperatures,
    const std::vector<int>& nozzle_temperature_range_lows,
    const std::vector<int>& nozzle_temperature_range_highs)
{
    const size_t filament_count = filament_types.size();
    if (filament_count < 2)
        return FilamentCompatibilityType::Compatible;

    std::vector<int> resolved_temperatures(filament_count, 0);
    std::vector<int> resolved_range_lows(filament_count, 0);
    std::vector<int> resolved_range_highs(filament_count, 0);
    for (size_t i = 0; i < filament_count; ++i) {
        int range_low = (i < nozzle_temperature_range_lows.size()) ? nozzle_temperature_range_lows[i] : 0;
        int range_high = (i < nozzle_temperature_range_highs.size()) ? nozzle_temperature_range_highs[i] : 0;

        if (range_low == 0 || range_high == 0) {
            int default_low = range_low;
            int default_high = range_high;
            MaterialType::get_temperature_range(filament_types[i], default_low, default_high);
            if (range_low == 0)
                range_low = default_low;
            if (range_high == 0)
                range_high = default_high;
        }

        if (range_low >= range_high)
            return FilamentCompatibilityType::InvalidTemperatureRange;

        int print_temperature = (i < nozzle_temperatures.size()) ? nozzle_temperatures[i] : 0;

        resolved_temperatures[i] = print_temperature;
        resolved_range_lows[i] = range_low;
        resolved_range_highs[i] = range_high;
    }

    for (size_t i = 0; i < filament_count; ++i) {
        for (size_t j = i + 1; j < filament_count; ++j) {
            const bool i_temp_is_compatible_with_j =
                resolved_temperatures[i] >= resolved_range_lows[j] &&
                resolved_temperatures[i] <= resolved_range_highs[j];
            const bool j_temp_is_compatible_with_i =
                resolved_temperatures[j] >= resolved_range_lows[i] &&
                resolved_temperatures[j] <= resolved_range_highs[i];

            if (i_temp_is_compatible_with_j && j_temp_is_compatible_with_i)
                continue;

            // Range-only rule: any pair outside mutual recommended ranges is incompatible.
            return FilamentCompatibilityType::HighLowMixed;
        }
    }

    return FilamentCompatibilityType::Compatible;
}

bool Print::is_filaments_compatible(const std::vector<int>& filament_types)
{
    bool has_high_temperature_filament = false;
    bool has_low_temperature_filament = false;

    for (const auto& type : filament_types) {
        if (type == FilamentTempType::HighTemp)
            has_high_temperature_filament = true;
        else if (type == FilamentTempType::LowTemp)
            has_low_temperature_filament = true;
    }

    if (has_high_temperature_filament && has_low_temperature_filament)
        return false;

    return true;
}
int Print::get_compatible_filament_type(const std::set<int>& filament_types)
{
    bool has_high_temperature_filament = false;
    bool has_low_temperature_filament = false;

    for (const auto& type : filament_types) {
        if (type == FilamentTempType::HighTemp)
            has_high_temperature_filament = true;
        else if (type == FilamentTempType::LowTemp)
            has_low_temperature_filament = true;
    }

    if (has_high_temperature_filament && has_low_temperature_filament)
        return HighLowCompatible;
    else if (has_high_temperature_filament)
        return HighTemp;
    else if (has_low_temperature_filament)
        return LowTemp;
    return HighLowCompatible;
}

//BBS: this function is used to check whether multi filament can be printed
StringObjectException Print::check_multi_filament_valid(const Print& print)
{
    auto print_config = print.config();
    const std::string incompatible_temp_msg = L("Selected nozzle temperatures are incompatible. Each filament's nozzle temperature must fall within the recommended nozzle temperature range of the other filaments. Otherwise, nozzle clogging or printer damage may occur.");
    const std::string invalid_temp_range_msg = L("Invalid recommended nozzle temperature range. The lower bound must be lower than the upper bound.");
    const std::string incompatible_temp_msg_preferences_enable = L("If you still want to print, you can enable the option in Preferences / Control / Slicing / Remove mixed temperature restriction.");
    if(print_config.print_sequence == PrintSequence::ByObject) {// use ByObject valid under ByObject print sequence
        bool has_incompatible_object = false;
        bool enable_mix_printing = !print.need_check_multi_filaments_compatibility();
        StringObjectException ret;

        for (const auto &objectID_t : print.print_object_ids()) {
            std::set<int> obj_used_extruder_ids;
            auto                     print_object = print.get_object(objectID_t);// current object
            if (print_object){
                auto object_extruders_t = print_object->object_extruders(); // object used extruder
                for (unsigned int extruder : object_extruders_t) {
                    obj_used_extruder_ids.insert(static_cast<int>(extruder));
                }
            }

            if (print_object->has_support_material()) { // extruder used by supports
                auto num_extruders                 = (unsigned int) print_config.filament_diameter.size();
                assert(print_object->config().support_filament >= 0);
                if (print_object->config().support_filament >= 1 && (unsigned int)print_object->config().support_filament < num_extruders + 1)
                    obj_used_extruder_ids.insert((unsigned int) print_object->config().support_filament - 1);//0-based extruder id
                assert(print_object->config().support_interface_filament >= 0);
                if (print_object->config().support_interface_filament >= 1 && (unsigned int)print_object->config().support_interface_filament < num_extruders + 1)
                    obj_used_extruder_ids.insert((unsigned int) print_object->config().support_interface_filament - 1);
            }
            std::vector<std::string> filament_types;
            std::vector<int> nozzle_temperatures;
            std::vector<int> nozzle_temperature_range_lows;
            std::vector<int> nozzle_temperature_range_highs;
            filament_types.reserve(obj_used_extruder_ids.size());
            nozzle_temperatures.reserve(obj_used_extruder_ids.size());
            nozzle_temperature_range_lows.reserve(obj_used_extruder_ids.size());
            nozzle_temperature_range_highs.reserve(obj_used_extruder_ids.size());

            for (const auto &extruder_idx : obj_used_extruder_ids) {
                filament_types.push_back(print_config.filament_type.get_at(extruder_idx));
                nozzle_temperatures.push_back(print_config.nozzle_temperature.get_at(extruder_idx));
                nozzle_temperature_range_lows.push_back(print_config.nozzle_temperature_range_low.get_at(extruder_idx));
                nozzle_temperature_range_highs.push_back(print_config.nozzle_temperature_range_high.get_at(extruder_idx));
            }

            auto compatibility = check_multi_filaments_compatibility(
                filament_types,
                nozzle_temperatures,
                nozzle_temperature_range_lows,
                nozzle_temperature_range_highs); // check for each object
            if (compatibility == FilamentCompatibilityType::InvalidTemperatureRange) {
                ret.string = invalid_temp_range_msg;
                return ret;
            }
            if (compatibility != FilamentCompatibilityType::Compatible) {
                has_incompatible_object = true;
                break;
            }
        }
        if (has_incompatible_object){
            if (enable_mix_printing) {
                ret.string     = incompatible_temp_msg;
                ret.is_warning = true;
            } else
                ret.string = incompatible_temp_msg + " " + incompatible_temp_msg_preferences_enable;
        }
        return ret;
    }
    std::vector<unsigned int> extruders = print.extruders();
    std::vector<std::string> filament_types;
    std::vector<int> nozzle_temperatures;
    std::vector<int> nozzle_temperature_range_lows;
    std::vector<int> nozzle_temperature_range_highs;
    filament_types.reserve(extruders.size());
    nozzle_temperatures.reserve(extruders.size());
    nozzle_temperature_range_lows.reserve(extruders.size());
    nozzle_temperature_range_highs.reserve(extruders.size());
    for (const auto& extruder_idx : extruders) {
        filament_types.push_back(print_config.filament_type.get_at(extruder_idx));
        nozzle_temperatures.push_back(print_config.nozzle_temperature.get_at(extruder_idx));
        nozzle_temperature_range_lows.push_back(print_config.nozzle_temperature_range_low.get_at(extruder_idx));
        nozzle_temperature_range_highs.push_back(print_config.nozzle_temperature_range_high.get_at(extruder_idx));
    }

    auto compatibility = check_multi_filaments_compatibility(
        filament_types,
        nozzle_temperatures,
        nozzle_temperature_range_lows,
        nozzle_temperature_range_highs);
    bool enable_mix_printing = !print.need_check_multi_filaments_compatibility();

    StringObjectException ret;

    if (compatibility == FilamentCompatibilityType::InvalidTemperatureRange) {
        ret.string = invalid_temp_range_msg;
        return ret;
    }

    if(compatibility != FilamentCompatibilityType::Compatible){
        if(enable_mix_printing){
            ret.string = incompatible_temp_msg;
            ret.is_warning = true;
        }
        else{
            ret.string = incompatible_temp_msg + " " + incompatible_temp_msg_preferences_enable;
        }
    }

    return ret;
}

// One raft layer height as deposited, with the physical nozzle that lays it and the support key
// that chose that nozzle. The tower stands beside the raft level by level and has to follow these
// steps. Raft heights are capped at the nozzle's maximum, so the only step that cannot be laid is
// one under the nozzle's minimum. Shares mixed_nozzle_deposited_level_height with Slicing.cpp so
// the two cannot drift apart.
struct MixedNozzleRaftStep
{
    double      height { 0. };
    size_t      nozzle { 0 };
    const char *filament_key { nullptr };
};

// The tower can carry a step when some whole number of equal levels fits the emitting nozzle's
// layer height range. A step over the maximum can be split; one under the minimum cannot.
static bool mixed_nozzle_raft_step_carriable(const PrintConfig &config, double height, size_t nozzle)
{
    if (! std::isfinite(height) || height <= 0.)
        return false;
    if (nozzle >= config.nozzle_diameter.values.size())
        return false;
    const double minimum = resolved_min_layer_height(config, nozzle);
    const double maximum = resolved_max_layer_height(config, nozzle);
    if (! std::isfinite(minimum) || minimum <= 0. || ! std::isfinite(maximum) || maximum <= 0.)
        return false;
    const double levels = std::max(1., std::ceil(height / maximum));
    const double level  = height / levels;
    return level + EPSILON >= minimum && level <= maximum + EPSILON;
}

// The raft steps this object deposits, in print order. Support layer heights are not included:
// the support cap follows its own nozzle's limits in SlicingParameters::create_from_config().
static std::vector<MixedNozzleRaftStep> mixed_nozzle_raft_steps(const PrintConfig &print_config,
                                                                const PrintObject &object)
{
    std::vector<MixedNozzleRaftStep> steps;
    const size_t raft_layers = size_t(std::max(0, object.config().raft_layers.value));
    if (raft_layers == 0)
        return steps;
    const int base_idx      = resolved_support_filament_nozzle_idx(print_config, object.config().support_filament.value);
    const int interface_idx = resolved_support_filament_nozzle_idx(print_config, object.config().support_interface_filament.value);
    const size_t nozzles = print_config.nozzle_diameter.values.size();
    if (base_idx <= 0 || interface_idx <= 0 || size_t(base_idx) > nozzles || size_t(interface_idx) > nozzles)
        return steps;
    const double layer_height = object.config().layer_height.value;
    // The same expressions and cap Slicing.cpp uses for the raft base, interface and contact heights.
    const double base_height      = mixed_nozzle_deposited_level_height(
        print_config, std::max(layer_height, 0.75 * print_config.nozzle_diameter.get_at(size_t(base_idx - 1))), base_idx);
    const double interface_height = mixed_nozzle_deposited_level_height(
        print_config, std::max(layer_height, 0.75 * print_config.nozzle_diameter.get_at(size_t(interface_idx - 1))), interface_idx);
    const size_t interface_raft_layers = (raft_layers + 1) / 2;
    const size_t base_raft_layers      = raft_layers - interface_raft_layers;
    // The first raft layer is the print's own first layer, so a base step is only deposited once
    // there is a second base layer above it. The contact layer is always a step of its own as soon
    // as the raft is more than that single first layer.
    if (base_raft_layers > 1)
        steps.push_back({base_height, size_t(base_idx - 1), "support_filament"});
    if (raft_layers > 1)
        steps.push_back({interface_height, size_t(interface_idx - 1), "support_interface_filament"});
    return steps;
}

// Precondition: Print::validate() requires the Print::apply() to be called its invocation.
//BBS: refine seq-print validation logic
// Why a support whose filaments are on different nozzles cannot be laid, without the code, or an empty reason when
// it can. Organic tree support (and Default, Snug or Grid, which a tree draws as Organic) on a coarser nozzle than
// its interface is laid in bands that nozzle can lay (mixed_nozzle_band_support_body()). Slim, Strong and Hybrid
// trees use their own generator, which never hands its layers to the banding, and with the interface on the base's
// nozzle too the interface itself would have to be laid at coarse rows. Whether the finer nozzle then lays some body
// (and needs a filament of the base's material for it) is known only once the support is sliced:
// PrintObject::assign_interface_nozzle_body_filament() refuses it there.
struct MixedNozzleSupportRefusal {
    std::string why;
    std::string opt_key;
    bool        tree = true;
};
static MixedNozzleSupportRefusal mixed_nozzle_support_refusal(const PrintConfig &print_config, const PrintObjectConfig &config,
                                                              double base_dmr, double interface_dmr, bool via_support)
{
    const SupportMaterialStyle style = config.support_style.value;
    const bool tree          = via_support && is_tree(config.support_type.value);
    const bool own_generator = style == smsTreeSlim || style == smsTreeStrong || style == smsTreeHybrid;
    const double finest      = *std::min_element(print_config.nozzle_diameter.values.begin(), print_config.nozzle_diameter.values.end());
    if (tree && base_dmr > finest + EPSILON) {
        if (own_generator)
            return { Slic3r::format("Slim, Strong and Hybrid tree support cannot have its base on the %1% mm nozzle. Use Organic tree "
                                    "support or Normal support, or put Support/raft base on a filament on the fine nozzle.", base_dmr),
                     "support_style" };
        if (interface_dmr > base_dmr - EPSILON)
            return { Slic3r::format("Tree support cannot have its interface on the %1% mm nozzle with its base. Put Support/raft "
                                    "interface on a filament on the fine nozzle, or use Normal support.", base_dmr),
                     "support_interface_filament" };
    }
    // Both support filaments on a nozzle that cannot lay the object's layers: Organic trees are laid on rows that
    // nozzle can lay (mixed_nozzle_coarsen_shared_support()); the other tree generators cannot be.
    if (tree && own_generator && mixed_nozzle_support_on_coarse_nozzle(print_config, config, true))
        return { Slic3r::format("Slim, Strong and Hybrid tree support cannot be laid by the %1% mm nozzle, whose thinnest layer is "
                                "thicker than the object's layers. Use Organic tree support or Normal support.", base_dmr),
                 "support_style" };
    return {};
}

StringObjectException Print::validate(std::vector<StringObjectException> *warnings, Polygons* collison_polygons, std::vector<std::pair<Polygon, float>>* height_polygons) const
{
    auto add_warning = [warnings](StringObjectException w) {
        w.is_warning = true;
        if (warnings != nullptr)
            warnings->push_back(std::move(w));
    };
    auto warn = [&](std::string msg, std::string opt_key = "", const ObjectBase* object = nullptr) {
        StringObjectException w;
        w.string  = std::move(msg);
        w.opt_key = std::move(opt_key);
        w.object  = object;
        add_warning(std::move(w));
    };

    std::vector<unsigned int> extruders = this->extruders();
    unsigned int nozzles = m_config.nozzle_diameter.size();

    if (m_objects.empty())
        return {std::string()};

    if (extruders.empty())
        return { L("No extrusions under current settings.") };

    // A filament mapped to an extruder this printer lacks cannot be sliced: nozzle-indexed lookups
    // answer an out-of-range index with the first nozzle's column, so the object would silently be
    // sliced for the wrong nozzle. Reject it here, because callers of
    // SlicingParameters::create_from_config() cannot absorb an exception. Only the static manual
    // modes are checked (only there is filament_map a user input), and only this object's filaments.
    if (is_static_manual_filament_map_mode(m_config.filament_map_mode.value)) {
        for (const PrintObject *object : m_objects) {
            std::vector<unsigned int> object_filaments = object->object_extruders();
            if (object->has_support_material())
                // Support reads the limit vectors under exactly this condition. Both ids are
                // one-based, and zero means "keep the current filament", naming no mapping.
                for (int support_filament : { object->config().support_filament.value, object->config().support_interface_filament.value })
                    if (support_filament > 0)
                        object_filaments.emplace_back(unsigned(support_filament - 1));
            for (unsigned int filament_id : object_filaments)
                if (! physical_extruder_for_filament(m_config, filament_id))
                    return { Slic3r::format(L("Filament %1% is assigned to an extruder this printer does not have. Map it to an extruder between 1 and %2%."),
                                            filament_id + 1, nozzles),
                             object->model_object() };
        }
    }

    if (is_mixed_nozzle_feature_split(m_config)) {
        for (const PrintObject *object : m_objects) {
            // has_support() covers both enable_support and enforce_support_layers. Raft reuses the same two
            // keys, so one block covers both, with `code` naming the condition the object is under.
            if (object->has_support() || object->has_raft()) {
                const bool via_support = object->has_support();
                const int base_filament = object->config().support_filament.value;
                const int interface_filament = object->config().support_interface_filament.value;
                const char *const code = via_support ? "SRL-F01" : "SRL-F02";
                if (base_filament <= 0 || interface_filament <= 0) {
                    StringObjectException error;
                    error.string  = Slic3r::format("[%1%] Set the support filament (Support/raft base) to a specific filament. At Default it prints with whichever filament is active, which can be on either nozzle.", code);
                    error.object  = object->model_object();
                    error.opt_key = base_filament <= 0 ? "support_filament" : "support_interface_filament";
                    return error;
                }
                const MixedNozzleToolResolution base_tool = resolve_mixed_nozzle_tool(
                    m_config, size_t(base_filament - 1), MixedNozzleResolveScope::PhysicalToolOnly);
                const MixedNozzleToolResolution interface_tool = resolve_mixed_nozzle_tool(
                    m_config, size_t(interface_filament - 1), MixedNozzleResolveScope::PhysicalToolOnly);
                if (!base_tool || !interface_tool) {
                    StringObjectException error;
                    const MixedNozzleDiagnostic &diagnostic = !base_tool ? *base_tool.diagnostic : *interface_tool.diagnostic;
                    error.string  = Slic3r::format("[%1%] [%2%] %3% Set Filament grouping to Custom and assign the support filament(s) to an installed nozzle.",
                                                   code, diagnostic.stable_code(), diagnostic.message());
                    error.object  = object->model_object();
                    error.opt_key = !base_tool ? "support_filament" : "support_interface_filament";
                    return error;
                }
                const double base_dmr      = m_config.nozzle_diameter.get_at(base_tool.tool->physical_extruder);
                const double interface_dmr = m_config.nozzle_diameter.get_at(interface_tool.tool->physical_extruder);
                if (interface_dmr > base_dmr) {
                    StringObjectException error;
                    error.string  = Slic3r::format("[%1%] The support interface is on a larger nozzle than the support base. The interface is what touches the model, so set Support/raft interface to Default or to a filament on the base's nozzle or a finer one.", code);
                    error.object  = object->model_object();
                    error.opt_key = "support_interface_filament";
                    return error;
                }
                // Tree styles and filaments the nozzles can lay the support with (mixed_nozzle_support_refusal()).
                if (const MixedNozzleSupportRefusal refusal = mixed_nozzle_support_refusal(m_config, object->config(), base_dmr, interface_dmr, via_support);
                    ! refusal.why.empty()) {
                    StringObjectException error;
                    error.string  = (refusal.tree ? "[SRL-F14] " : "[SRL-F15] ") + refusal.why;
                    error.object  = object->model_object();
                    error.opt_key = refusal.opt_key;
                    return error;
                }
                // With a prime tower the tower stands beside the raft and follows its steps, so a raft step
                // no whole number of tower levels can carry is refused here, on a Process setting the user
                // can change. The tower's own structure audit remains as a backstop.
                if (m_config.enable_prime_tower.value || this->has_wipe_tower()) {
                    for (const MixedNozzleRaftStep &step : mixed_nozzle_raft_steps(m_config, *object))
                        if (! mixed_nozzle_raft_step_carriable(m_config, step.height, step.nozzle)) {
                            StringObjectException error;
                            error.string  = Slic3r::format(
                                "[SRL-F11] The raft lays a %1% mm layer on nozzle %2%, which prints between %3% mm and %4% mm. "
                                "No whole number of prime-tower levels fits that step, so the tower cannot stand beside the raft. "
                                "Use fewer raft layers, or move %5% to a nozzle that can lay this height.",
                                step.height, step.nozzle + 1,
                                resolved_min_layer_height(m_config, step.nozzle),
                                resolved_max_layer_height(m_config, step.nozzle),
                                step.filament_key);
                            error.object  = object->model_object();
                            error.opt_key = "raft_layers";
                            return error;
                        }
                }
            }
        }
        // A raft whose bed layer the coarse nozzle lays starts at that nozzle's thinnest layer when the first layer
        // is thinner (mixed_nozzle_raft_first_layer_height()), and the first layer is one for the whole plate.
        {
            const PrintObject *raised = nullptr;
            for (const PrintObject *object : m_objects)
                if (mixed_nozzle_raft_first_layer_height(m_config, object->config()) > m_config.initial_layer_print_height.value + EPSILON)
                    raised = object;
            if (raised != nullptr) {
                const double height = mixed_nozzle_raft_first_layer_height(m_config, raised->config());
                for (const PrintObject *object : m_objects)
                    if (std::abs(mixed_nozzle_raft_first_layer_height(m_config, object->config()) - height) > EPSILON) {
                        StringObjectException error;
                        error.string  = Slic3r::format("[SRL-F16] The raft under \"%1%\" starts on the bed at %2% mm, the thinnest layer the nozzle "
                                                       "that lays it can print, and that makes the first layer %2% mm for the whole plate. \"%3%\" "
                                                       "would start at %4% mm. Give both objects the same raft and Support/raft base, or set First "
                                                       "layer height to %2% mm.",
                                                       raised->model_object()->name, height, object->model_object()->name,
                                                       mixed_nozzle_raft_first_layer_height(m_config, object->config()));
                        error.object  = object->model_object();
                        error.opt_key = "raft_layers";
                        return error;
                    }
                // Said only once nothing refuses the plate for it.
                const int    nozzle = resolved_support_filament_nozzle_idx(m_config, raised->config().raft_layers.value == 1 ?
                    raised->config().support_interface_filament.value : raised->config().support_filament.value);
                warn(Slic3r::format("The raft's first layer prints at %1% mm instead of the %2% mm First layer height: the %3% mm nozzle "
                                    "lays the raft's base and prints no thinner.",
                                    height, m_config.initial_layer_print_height.value, m_config.nozzle_diameter.get_at(size_t(std::max(nozzle, 1) - 1))),
                     "initial_layer_print_height", raised->model_object());
            }
        }
        const bool regular_skirt  = m_config.skirt_loops.value > 0 && this->has_skirt();
        const bool infinite_skirt = this->has_infinite_skirt();
        if (infinite_skirt) {
            StringObjectException error;
            error.string  = "[SRL-F03] Feature Split does not support a draft shield.";
            error.object  = m_objects.front()->model_object();
            error.opt_key = "draft_shield";
            return error;
        }
        // Skirt and brim are routed to the first-layer outer-wall tool
        // (Print::feature_split_adhesion_tool()). Reject rather than mis-route the loops if that tool does
        // not resolve; this should not trigger, since every filament of the object was resolved above.
        if (! this->feature_split_adhesion_tool()) {
            for (const PrintObject *object : m_objects) {
                if (object->has_brim()) {
                    StringObjectException error;
                    error.string  = "[SRL-F04] The brim and skirt have no nozzle to print on, because this plate's Feature Split fine and coarse "
                                     "materials are not set. Open Set up / Change... and choose them.";
                    error.object  = object->model_object();
                    // The link opens setup, where the fine and coarse materials are chosen.
                    error.opt_key = "mixed_nozzle_slicing_mode";
                    return error;
                }
            }
            if (regular_skirt) {
                StringObjectException error;
                error.string  = "[SRL-F04] The brim and skirt have no nozzle to print on, because this plate's Feature Split fine and coarse "
                                     "materials are not set. Open Set up / Change... and choose them.";
                error.object  = m_objects.front()->model_object();
                error.opt_key = "mixed_nozzle_slicing_mode";
                return error;
            }
        }
        for (const PrintObject *object : m_objects) {
            for (size_t region_id = 0; region_id < object->num_printing_regions(); ++region_id) {
                const PrintRegionConfig &region = object->printing_region(region_id).config();
                if (region.sparse_infill_density.value > 0. &&
                    ! feature_combine_admits_sparse_infill_pattern(region.sparse_infill_pattern.value)) {
                    StringObjectException error;
                    error.string  = "[SRL-F08] Feature Split cannot print locked zag or lightning infill at the coarse layer height; "
                                     "both need every fine layer that one coarse layer replaces.";
                    error.object  = object->model_object();
                    error.opt_key = "sparse_infill_pattern";
                    return error;
                }
                // infill_combination_max_layer_height defaults to 100%, and both 0 and 100% mean "auto: use
                // the nozzle diameter". Only another value is an explicit override the mode must refuse.
                const bool infill_combination_height_overridden =
                    ! (is_approx(region.infill_combination_max_layer_height.value, 0.) ||
                       (region.infill_combination_max_layer_height.percent &&
                        is_approx(region.infill_combination_max_layer_height.value, 100.)));
                if (region.sparse_infill_density.value > 0. &&
                    (region.infill_combination.value || infill_combination_height_overridden)) {
                    StringObjectException error;
                    error.string  = "[SRL-F09] Feature Split sets infill combination from its coarse layer height. "
                                     "Leave infill combination off and its maximum layer height at 0 (auto).";
                    error.object  = object->model_object();
                    error.opt_key = region.infill_combination.value ? "infill_combination" : "infill_combination_max_layer_height";
                    return error;
                }
                // Coarse inner walls need at least two wall loops (with fewer, every wall is the outer loop)
                // and cannot combine with InnerOuterInner, which would interrupt the outer wall pass with an
                // inner loop and cost two tool changes per island.
                const bool coarse_inner_walls = region.outer_wall_filament_id.value != region.inner_wall_filament_id.value;
                if (coarse_inner_walls && region.wall_loops.value < 2) {
                    StringObjectException error;
                    error.string  = "[SRL-F10] When the inner walls use a different filament from the outer wall, Feature Split needs at least 2 wall loops.";
                    error.object  = object->model_object();
                    error.opt_key = "wall_loops";
                    return error;
                }
                if (coarse_inner_walls && region.wall_sequence.value == WallSequence::InnerOuterInner) {
                    StringObjectException error;
                    error.string  = "[SRL-F10] When the inner walls use a different filament from the outer wall, Feature Split does not support the inner/outer/inner wall order.";
                    error.object  = object->model_object();
                    error.opt_key = "wall_sequence";
                    return error;
                }
                // Check that each role's tool can deposit at the cadence that role runs at. Every role runs
                // at the object's layer height, except a sparse infill owned by a different physical tool
                // than the walls, which runs once per band at the coarse height. The shared first layer is
                // checked against the tools of the base-cadence roles. An uncommitted sparse leaf falls back
                // to outer_wall_filament_id at the fine cadence, which is already checked here.
                const double base_cadence = object->config().layer_height.value;
                const double shared_first_layer = m_config.initial_layer_print_height.value > 0. ?
                    m_config.initial_layer_print_height.value : base_cadence;
                const int fine_owner   = region.outer_wall_filament_id.value;
                const int sparse_owner = region.sparse_infill_filament_id.value;
                double coarse_height = 0.;
                if (fine_owner > 0 && sparse_owner > 0 && region.sparse_infill_density.value > 0.) {
                    // resolve_mixed_nozzle_cadence only answers when the two owners land on
                    // different physical tools, so a nonzero coarse height is itself the proof
                    // that this region's sparse infill really is coarse-owned.
                    const MixedNozzleCadenceResolution resolved = resolve_mixed_nozzle_cadence(
                        m_config, base_cadence, m_config.mixed_nozzle_coarse_layer_height.value,
                        size_t(fine_owner - 1), size_t(sparse_owner - 1));
                    if (resolved)
                        coarse_height = resolved.cadence->coarse_height;
                }
                struct RoleOwner { const char *key; int filament; bool is_sparse_infill; };
                const RoleOwner role_owners[] = {
                    {"outer_wall_filament_id",     region.outer_wall_filament_id.value,     false},
                    {"inner_wall_filament_id",     region.inner_wall_filament_id.value,     false},
                    {"internal_solid_filament_id", region.internal_solid_filament_id.value, false},
                    {"top_surface_filament_id",    region.top_surface_filament_id.value,    false},
                    {"bottom_surface_filament_id", region.bottom_surface_filament_id.value, false},
                    {"sparse_infill_filament_id",  region.sparse_infill_filament_id.value,  true},
                };
                std::optional<size_t> fine_nozzle;
                if (fine_owner > 0)
                    fine_nozzle = physical_extruder_for_filament(m_config, unsigned(fine_owner - 1));
                for (const RoleOwner &owner : role_owners) {
                    if (owner.filament <= 0)
                        continue;
                    // A region that draws no sparse infill deposits nothing under this key, so
                    // the tool it names is not asked to lay anything.
                    if (owner.is_sparse_infill && region.sparse_infill_density.value <= 0.)
                        continue;
                    const std::optional<size_t> nozzle = physical_extruder_for_filament(m_config, unsigned(owner.filament - 1));
                    // An unresolved filament is already refused above, so read fail-open here rather than
                    // report it twice.
                    if (! nozzle || *nozzle >= m_config.nozzle_diameter.values.size())
                        continue;
                    const bool sparse_on_its_own_tool =
                        owner.is_sparse_infill && fine_nozzle && *nozzle != *fine_nozzle;
                    // A coarse-owned sparse infill with no resolved coarse height is reported by the cadence
                    // resolver, not here.
                    if (sparse_on_its_own_tool && coarse_height <= 0.)
                        continue;
                    const bool runs_at_coarse_height = sparse_on_its_own_tool;
                    const double cadence = runs_at_coarse_height ? coarse_height : base_cadence;
                    const double nozzle_minimum = resolved_min_layer_height(m_config, *nozzle);
                    const double nozzle_maximum = resolved_max_layer_height(m_config, *nozzle);
                    const bool lays_first_layer = ! runs_at_coarse_height;
                    if (nozzle_minimum > cadence + EPSILON || nozzle_maximum + EPSILON < cadence ||
                        cadence > m_config.nozzle_diameter.get_at(*nozzle) ||
                        (lays_first_layer && (nozzle_minimum > shared_first_layer + EPSILON ||
                                              nozzle_maximum + EPSILON < shared_first_layer))) {
                        StringObjectException error;
                        error.string = Slic3r::format(
                            "[SRL-F12] %1% names filament %2%, which prints on nozzle %3%. That nozzle lays "
                            "between %4% mm and %5% mm through a %6% mm opening, and this role is deposited at %7% mm%8%.",
                            owner.key, owner.filament, *nozzle + 1, nozzle_minimum, nozzle_maximum,
                            m_config.nozzle_diameter.get_at(*nozzle), cadence,
                            lays_first_layer ? Slic3r::format(", over a shared first layer of %1% mm", shared_first_layer)
                                             : std::string());
                        error.object  = object->model_object();
                        error.opt_key = owner.key;
                        return error;
                    }
                }
            }
        }
    }

    // Every synchronized refusal has a stable code. The sentence explains it to a person; the code is
    // what tests, logs and the plan export match on. Defined above both mode branches because the
    // shared prime tower checks below run for either mode.
    const auto reject = [](const PrintObject *object, const char *code, std::string opt_key, std::string message) {
        StringObjectException error;
        error.string = std::string("[") + code + "] " + std::move(message);
        error.object = object == nullptr ? nullptr : object->model_object();
        error.opt_key = std::move(opt_key);
        return error;
    };

    if (is_mixed_nozzle_feature_split(m_config) || is_mixed_nozzle_body_split(m_config)) {
        const PrintObject *const tower_check_object = m_objects.front();
        // A prime tower is admitted only as the printer's native Type 1 tower, one clause per line, each
        // naming the key a user would change. A tower is required at every nozzle change in both modes,
        // since filament parked in a heated nozzle degrades without a purge. purge_in_prime_tower,
        // single_extruder_multi_material_priming and enable_filament_ramming are inert under Type 1 but
        // fail closed so a later generator change cannot bring an unqualified path back.
        if (m_config.enable_prime_tower.value || this->has_wipe_tower()) {
            const auto reject_tower = [&reject, tower_check_object](const char *opt_key, const char *message) {
                return reject(tower_check_object, "SRL-A32", opt_key, message);
            };
            if (this->wipe_tower_type() != WipeTowerType::Type1)
                return reject_tower("wipe_tower_type",
                                    "Feature Split and Body Split work only with the printer's own prime tower type.");
            if (m_config.wipe_tower_filament.value != 0)
                return reject_tower("wipe_tower_filament",
                                    "A separate prime tower filament is not supported: it adds a third filament to every layer, and Feature Split and Body Split print only the fine and coarse materials.");
            if (! is_approx(m_config.wipe_tower_rotation_angle.value, 0.))
                return reject_tower("wipe_tower_rotation_angle",
                                    "Feature Split and Body Split need the prime tower rotation at 0 degrees.");
            if (m_config.prime_tower_flat_ironing.value)
                return reject_tower("prime_tower_flat_ironing",
                                    "Prime-tower flat ironing is not supported: it retraces the tower top stepping by the shared bead width, which is not the width the fine tool draws.");
            if (m_config.enable_tower_interface_features.value)
                return reject_tower("enable_tower_interface_features",
                                    "Prime tower interface features are not supported by Feature Split or Body Split.");
            if (! m_config.use_relative_e_distances.value)
                return reject_tower("use_relative_e_distances",
                                    "A prime tower requires relative E distances.");
            if (m_config.ooze_prevention.value)
                return reject_tower("ooze_prevention",
                                    "Ooze prevention is not supported by Feature Split or Body Split when a prime tower is used.");
            if (m_config.prime_volume_mode.value != pvmDefault)
                return reject_tower("prime_volume_mode",
                                    "Feature Split and Body Split need the default prime volume mode.");
            if (m_config.purge_in_prime_tower.value)
                return reject_tower("purge_in_prime_tower",
                                    "Purging into the prime tower is not supported by Feature Split or Body Split.");
            if (m_config.single_extruder_multi_material_priming.value)
                return reject_tower("single_extruder_multi_material_priming",
                                    "Front-bed priming is not supported by Feature Split or Body Split.");
            if (m_config.enable_filament_ramming.value)
                return reject_tower("enable_filament_ramming",
                                    "Configurable filament ramming is not supported by Feature Split or Body Split.");
        }
    }

    // An IDEX printer in copy or mirror mode prints with both heads at once, so there is no tool
    // change to hand a band from one nozzle to the other. Those modes come from the printer profile's
    // start G-code; the plain IDEX profile of the same printer is fine.
    if ((is_mixed_nozzle_feature_split(m_config) || is_mixed_nozzle_body_split(m_config)) &&
        mixed_nozzle_start_gcode_duplicates_heads(m_config.machine_start_gcode.value))
        return reject(m_objects.empty() ? nullptr : m_objects.front(), "SRL-A50", "machine_start_gcode",
                      "This printer profile prints with both heads at once (copy or mirror mode). Feature Split and Body Split "
                      "hand the print from one head to the other, so pick the printer's normal IDEX profile.");

    // The logical filaments an object prints with: its regions and parts, and its support.
    const auto printed_filaments = [](const PrintObject &object) {
        std::vector<unsigned int> filaments = object.object_extruders();
        if (object.has_support_material())
            for (const int filament : {object.config().support_filament.value, object.config().support_interface_filament.value})
                if (filament > 0)
                    filaments.push_back(unsigned(filament - 1));
        return filaments;
    };

    // The rules below are for named non-Bambu vendor profiles. A config with no printer model keeps
    // its existing admission.
    const bool other_vendor_printer = !is_BBL_printer() && !m_config.printer_model.value.empty() &&
                                      m_config.printer_model.value.rfind("Bambu Lab", 0) != 0;

    // On a non-Bambu printer Orca prints filament i on tool i (ToolOrdering) whatever filament_map
    // says, while mixed-nozzle code reads filament_map, so the two must agree.
    if ((is_mixed_nozzle_feature_split(m_config) || is_mixed_nozzle_body_split(m_config)) && other_vendor_printer)
        for (const PrintObject *object : m_objects)
            for (const unsigned int filament : printed_filaments(*object))
                if (const std::optional<size_t> tool = physical_extruder_for_filament(m_config, filament);
                    tool && *tool != filament)
                    return reject(object, "SRL-A51", "filament_map",
                                  Slic3r::format("On this printer filament %1% always prints on toolhead %1%, but the filament map "
                                                 "puts it on toolhead %2%. Set the map back to one filament per toolhead, and pick "
                                                 "the fine and coarse filaments by the toolhead that holds each nozzle.",
                                                 filament + 1, *tool + 1));

    // A toolchanger such as the Prusa XL or the Snapmaker U1 has more tools than the two a
    // mixed-nozzle print uses. The tools the plate prints on must be exactly two; a third printing
    // tool has no place on the fine and coarse grid. Two-tool printers never enter this block.
    std::vector<size_t> mixed_nozzle_used_tools;
    if ((is_mixed_nozzle_feature_split(m_config) || is_mixed_nozzle_body_split(m_config)) && other_vendor_printer &&
        m_config.nozzle_diameter.values.size() > 2 && !m_objects.empty()) {
        std::set<size_t> used;
        for (const PrintObject *object : m_objects)
            for (const unsigned int filament : printed_filaments(*object))
                if (const std::optional<size_t> tool = physical_extruder_for_filament(m_config, filament))
                    used.insert(*tool);
        if (used.size() > 2)
            return reject(m_objects.front(), "SRL-A03", "filament_map",
                          Slic3r::format("Feature Split and Body Split print on two toolheads, and this plate also prints on toolhead %1%. "
                                         "Move that filament to the fine or the coarse toolhead, or turn the mode off.",
                                         *std::next(used.begin(), 2) + 1));
        mixed_nozzle_used_tools.assign(used.begin(), used.end());
        // The tower and the fine-before-coarse order take the coarse tool to be the first tool
        // with the widest nozzle on the whole printer (mixed_nozzle_tower_coarse_extruder). A
        // parked toolhead with a wider nozzle, or an equally wide one ahead of the coarse
        // toolhead, would have them plan for a tool that never prints.
        if (mixed_nozzle_used_tools.size() == 2) {
            const double first  = m_config.nozzle_diameter.get_at(mixed_nozzle_used_tools[0]);
            const double second = m_config.nozzle_diameter.get_at(mixed_nozzle_used_tools[1]);
            const int    coarse = mixed_nozzle_tower_coarse_extruder(m_config);
            if (!is_approx(first, second) && coarse >= 0) {
                const size_t used_coarse = second > first ? mixed_nozzle_used_tools[1] : mixed_nozzle_used_tools[0];
                if (size_t(coarse) != used_coarse)
                    return reject(m_objects.front(), "SRL-A03", "nozzle_diameter",
                                  Slic3r::format("Feature Split and Body Split take the first toolhead with the widest nozzle as the "
                                                 "coarse one. On this printer that is toolhead %1% (%2% mm), which this plate does not "
                                                 "print on. Put the coarse filament on toolhead %1%, or fit it with a nozzle no wider "
                                                 "than toolhead %3% (%4% mm).",
                                                 coarse + 1, m_config.nozzle_diameter.get_at(size_t(coarse)), used_coarse + 1,
                                                 m_config.nozzle_diameter.get_at(used_coarse)));
            }
        }
    }

    if (is_mixed_nozzle_body_split(m_config)) {
        if (m_config.regional_interface_tolerance.value != 0.)
            return reject(m_objects.empty() ? nullptr : m_objects.front(), "SRL-A42", "regional_interface_tolerance",
                          "Body Split needs the interface tolerance between bodies left at its default of 0 mm.");
        // Any number of objects is admitted: each slices its own native grid and publishes its own plan
        // block, and the fine-before-coarse rotation is plate-wide.
        //
        // Print-level checks read m_config, not per-object config, so they run once. Band geometry is
        // committed in PrintObject::make_perimeters(), before filament grouping writes filament_map back,
        // so a region's binding must already be fixed. resolve_mixed_nozzle_tool() answers only where the
        // map is a static input the engine reads rather than one it rewrites.
        const auto reject_unresolved = [&reject](const PrintObject *object, const MixedNozzleDiagnostic &diagnostic) {
            return reject(object, "SRL-A02", diagnostic.option_key,
                          Slic3r::format("[%1%] %2% Set Filament grouping to Custom and assign each filament to the nozzle it must print with.",
                                         diagnostic.stable_code(), diagnostic.message()));
        };
        for (const PrintObject *object : m_objects)
            for (unsigned int logical_filament : object->object_extruders()) {
                const MixedNozzleToolResolution resolution = resolve_mixed_nozzle_tool(
                    m_config, logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
                if (!resolution)
                    return reject_unresolved(object, *resolution.diagnostic);
            }
        // Production admission is generic over the configured physical pair. Qualification of
        // particular pair/tier combinations belongs to profiles and evidence, not C++ literals.
        std::vector<double> installed_nozzles = m_config.nozzle_diameter.values;
        // The pair is the two tools this plate prints on (a third was refused above).
        if (installed_nozzles.size() > 2) {
            installed_nozzles.clear();
            for (const size_t tool : mixed_nozzle_used_tools)
                installed_nozzles.push_back(m_config.nozzle_diameter.get_at(tool));
        }
        std::sort(installed_nozzles.begin(), installed_nozzles.end());
        if (installed_nozzles.size() != 2 ||
            std::any_of(installed_nozzles.begin(), installed_nozzles.end(), [](double diameter) {
                return !std::isfinite(diameter) || diameter <= 0.;
            }) || is_approx(installed_nozzles[0], installed_nozzles[1]))
            return reject(m_objects.front(), "SRL-A03", "nozzle_diameter",
                          "Body Split needs a printer with two nozzles of different sizes.");
        if (const auto *dynamic_map = m_full_print_config.option<ConfigOptionBool>("enable_filament_dynamic_map");
            dynamic_map != nullptr && dynamic_map->value)
            return reject(m_objects.front(), "SRL-A04", "enable_filament_dynamic_map", "Dynamic filament mapping is not supported by Body Split.");

        // A plain object beside a Body Split object prints as with the mode off, but a plate with no Body
        // Split object has nothing to synchronize.
        if (std::none_of(m_objects.begin(), m_objects.end(), [this](const PrintObject *object) {
                return is_body_split_object(m_config, *object->model_object());
            }))
            return reject(m_objects.front(), "SRL-A06", "mixed_nozzle_slicing_mode",
                          "Body Split needs at least one object whose parts, painted colours or modifiers print on both nozzles. "
                          "Assign a part, a painted colour or a modifier to the other nozzle, or turn Body Split off for this plate.");

        // A plain object on a Body Split plate is sliced as with the mode off, so the native-grid checks
        // below do not apply. It still gets Feature Split's checks: support and raft must name tools that
        // exist, and every role must fit its nozzle's layer height limits at the object's layer height
        // and on the shared first layer.
        const auto refuse_plain_object = [&](const PrintObject *object) -> std::optional<StringObjectException> {
            if (object->has_support() || object->has_raft()) {
                const bool via_support = object->has_support();
                const int base_filament = object->config().support_filament.value;
                const int interface_filament = object->config().support_interface_filament.value;
                if (base_filament <= 0 || interface_filament <= 0)
                    return reject(object, "SRL-A29", base_filament <= 0 ? "support_filament" : "support_interface_filament",
                                  "Set the support filament (Support/raft base) to a specific filament. At Default it prints with whichever filament is active, which can be on either nozzle.");
                const MixedNozzleToolResolution base_tool = resolve_mixed_nozzle_tool(
                    m_config, size_t(base_filament - 1), MixedNozzleResolveScope::PhysicalToolOnly);
                if (!base_tool) return reject_unresolved(object, *base_tool.diagnostic);
                const MixedNozzleToolResolution interface_tool = resolve_mixed_nozzle_tool(
                    m_config, size_t(interface_filament - 1), MixedNozzleResolveScope::PhysicalToolOnly);
                if (!interface_tool) return reject_unresolved(object, *interface_tool.diagnostic);
                const double base_dmr      = m_config.nozzle_diameter.get_at(base_tool.tool->physical_extruder);
                const double interface_dmr = m_config.nozzle_diameter.get_at(interface_tool.tool->physical_extruder);
                if (interface_dmr > base_dmr)
                    return reject(object, "SRL-A29", "support_interface_filament",
                                  "The support interface is on a larger nozzle than the support base. The interface is what touches the model, so set Support/raft interface to Default or to a filament on the base's nozzle or a finer one.");
                // Feature Split starts such a raft on the bed at the coarse nozzle's thinnest layer; a Body Split plate
                // shares its first layer with the Body Split objects, which cannot follow.
                if (object->has_raft() && base_dmr > *std::min_element(m_config.nozzle_diameter.values.begin(), m_config.nozzle_diameter.values.end()) + EPSILON)
                    return reject(object, "SRL-A52", "support_filament", Slic3r::format(
                        "On a Body Split plate a raft cannot have its base on the %1% mm nozzle. Put Support/raft base on a filament on the fine nozzle, set Raft layers to 0, or use Feature Split for this plate.", base_dmr));
                if (const MixedNozzleSupportRefusal refusal = mixed_nozzle_support_refusal(m_config, object->config(), base_dmr, interface_dmr, via_support);
                    ! refusal.why.empty())
                    return reject(object, refusal.tree ? "SRL-A53" : "SRL-A55", refusal.opt_key, refusal.why);
                if (m_config.enable_prime_tower.value || this->has_wipe_tower())
                    for (const MixedNozzleRaftStep &step : mixed_nozzle_raft_steps(m_config, *object))
                        if (! mixed_nozzle_raft_step_carriable(m_config, step.height, step.nozzle))
                            return reject(object, "SRL-A49", "raft_layers", Slic3r::format(
                                "The raft lays a %1% mm layer on nozzle %2%, which prints between %3% mm and %4% mm. "
                                "No whole number of prime-tower levels fits that step, so the tower cannot stand beside the raft. "
                                "Use fewer raft layers, or move %5% to a nozzle that can lay this height.",
                                step.height, step.nozzle + 1,
                                resolved_min_layer_height(m_config, step.nozzle),
                                resolved_max_layer_height(m_config, step.nozzle),
                                step.filament_key));
            }
            const double layer_height = object->config().layer_height.value;
            const double shared_first_layer = m_config.initial_layer_print_height.value > 0. ?
                m_config.initial_layer_print_height.value : layer_height;
            for (size_t region_id = 0; region_id < object->num_printing_regions(); ++region_id) {
                const PrintRegionConfig &region = object->printing_region(region_id).config();
                const std::pair<const char *, int> role_owners[] = {
                    {"outer_wall_filament_id",     region.outer_wall_filament_id.value},
                    {"inner_wall_filament_id",     region.inner_wall_filament_id.value},
                    {"internal_solid_filament_id", region.internal_solid_filament_id.value},
                    {"top_surface_filament_id",    region.top_surface_filament_id.value},
                    {"bottom_surface_filament_id", region.bottom_surface_filament_id.value},
                    {"sparse_infill_filament_id",  region.sparse_infill_density.value > 0. ? region.sparse_infill_filament_id.value : 0},
                };
                for (const auto &[key, filament] : role_owners) {
                    if (filament <= 0)
                        continue;
                    // An unresolved filament was already refused above.
                    const std::optional<size_t> nozzle = physical_extruder_for_filament(m_config, unsigned(filament - 1));
                    if (! nozzle || *nozzle >= m_config.nozzle_diameter.values.size())
                        continue;
                    const double nozzle_minimum = resolved_min_layer_height(m_config, *nozzle);
                    const double nozzle_maximum = resolved_max_layer_height(m_config, *nozzle);
                    if (nozzle_minimum > layer_height + EPSILON || nozzle_maximum + EPSILON < layer_height ||
                        layer_height > m_config.nozzle_diameter.get_at(*nozzle) ||
                        nozzle_minimum > shared_first_layer + EPSILON || nozzle_maximum + EPSILON < shared_first_layer)
                        return reject(object, "SRL-A38", key, Slic3r::format(
                            "%1% names filament %2%, which prints on nozzle %3%. That nozzle lays between %4% mm and %5% mm "
                            "through a %6% mm opening, and this object prints at %7% mm over a shared first layer of %8% mm.",
                            key, filament, *nozzle + 1, nozzle_minimum, nozzle_maximum,
                            m_config.nozzle_diameter.get_at(*nozzle), layer_height, shared_first_layer));
                }
            }
            return std::nullopt;
        };

        // Per-object checks: mesh structure, region config and object config. Each admitted object
        // pushes its resolved coarse ratio here; plate-wide uniformity is checked once after the loop.
        std::vector<int> plate_coarse_cadence_ratios;
        for (const PrintObject *object : m_objects) {
        // Copies of an object share its slice, native grid and plan block. G-code prints every copy
        // within each tool's visit to a layer, so a copy adds no nozzle change or tower visit.

        const ModelObject *model_object = object->model_object();
        // A plain object has no bodies to split and skips the native-grid checks below.
        if (! is_body_split_object(m_config, *model_object)) {
            if (std::optional<StringObjectException> refusal = refuse_plain_object(object))
                return *refusal;
            continue;
        }
        // is_body_split_object() admits a lone model part only when it is painted or a modifier names a
        // filament on the other nozzle.
        const bool single_painted_body = std::count_if(model_object->volumes.begin(), model_object->volumes.end(),
            [](const ModelVolume *volume) { return volume->is_model_part(); }) == 1;
        // A support blocker or enforcer is not sliced into any region, so it only shapes support.
        // Modifiers and negative parts are admitted: ownership rows come from ordinary region slicing,
        // so a negative part is cut out of every body and colour, and a modifier's region follows the
        // body it sits in.
        if (object->num_printing_regions() < 2)
            return reject(object, "SRL-A07", "mixed_nozzle_slicing_mode", "Body Split needs at least two parts or painted regions that have something to print.");
        // Fuzzy-skin painting is admitted: the painted band roughens only its own body's external wall
        // (apply_body_split_fuzzy_skin_masks).
        for (const ModelVolume *volume : model_object->volumes) {
            if (! volume->is_mm_painted())
                continue;
            // A painted part with no filament of its own prints with the object filament, as in stock Orca;
            // ModelVolume::extruder_id() is that fallback.
            const int own_filament = volume->extruder_id();
            MixedNozzleToolResolution own_resolution;
            if (own_filament > 0) {
                own_resolution = resolve_mixed_nozzle_tool(
                    m_config, size_t(own_filament - 1), MixedNozzleResolveScope::PhysicalToolOnly);
                if (! own_resolution)
                    return reject_unresolved(object, *own_resolution.diagnostic);
            }
            for (int painted_filament : volume->get_extruders()) {
                const MixedNozzleToolResolution painted_resolution = resolve_mixed_nozzle_tool(
                    m_config, size_t(painted_filament - 1), MixedNozzleResolveScope::PhysicalToolOnly);
                if (! painted_resolution)
                    return reject_unresolved(object, *painted_resolution.diagnostic);
                // Native facet ownership is staged before the Body regional grid, so a painted child may
                // resolve to the other physical tool. Other synchronized modes keep the same-tool restriction
                // until they have an equivalent pre-grid ownership proof.
                if (own_resolution && painted_resolution.tool->physical_extruder != own_resolution.tool->physical_extruder &&
                    !is_mixed_nozzle_body_split(m_config))
                    return reject(object, "SRL-A08", "mixed_nozzle_slicing_mode",
                                  "Every painted colour on a body must print on the same nozzle as that body's own filament.");
            }
        }

        // Structural admission: a synchronized object must be closed solids a slicer can hand to two
        // nozzles. Geometry is checked per fine layer in PrintObject::make_perimeters(), settings here.
        for (const ModelVolume *volume : model_object->volumes) {
            // A support blocker or enforcer is never printed, so it need not be a closed solid.
            if (! volume->is_model_part())
                continue;
            TriangleMesh mesh = volume->mesh();
            mesh.transform(volume->get_matrix());
            if (mesh.its.indices.empty() || mesh.stats().open_edges != 0)
                return reject(object, "SRL-A09", "mixed_nozzle_slicing_mode",
                              "Every Body Split part must be a closed solid, and this one is empty or has open edges. Repair the mesh before slicing.");
            // Use signed volume, not the shell count: an inward-wound shell such as an exported internal cavity
            // is a pocket, not a second part, so only the net solid has to be positive.
            if (its_volume(mesh.its) <= EPSILON)
                return reject(object, "SRL-A10", "mixed_nozzle_slicing_mode",
                              "A Body Split part encloses no solid volume. Check that its facets face outwards.");
        }

        std::vector<unsigned int> logical_filaments;
        std::vector<size_t> physical_extruders;
        std::vector<size_t> region_physical_extruders;
        std::vector<double> regional_cadences;
        std::vector<size_t> region_logical_filaments;
        std::vector<size_t> region_skin_filaments;
        for (size_t region_id = 0; region_id < object->num_printing_regions(); ++region_id) {
            const PrintRegionConfig &region = object->printing_region(region_id).config();
            const int logical_filament = region.outer_wall_filament_id.value;
            // Without the region's fine-skins opt-in, all six roles must be on one logical filament.
            const bool fine_skins = region.mixed_nozzle_body_fine_skins.value;
            const int skin_filament = fine_skins ? region.internal_solid_filament_id.value : logical_filament;
            if (fine_skins) {
                if (logical_filament <= 0 || region.inner_wall_filament_id.value != logical_filament ||
                    region.sparse_infill_filament_id.value != logical_filament)
                    return reject(object, "SRL-A11", "outer_wall_filament_id",
                                  "In each body, the walls and sparse infill must all use one filament.");
                if (skin_filament <= 0 || region.top_surface_filament_id.value != skin_filament ||
                    region.bottom_surface_filament_id.value != skin_filament)
                    return reject(object, "SRL-A11", "internal_solid_filament_id",
                                  "In each body, internal solid infill, top surface and bottom surface must all use one filament, whether or not it matches the body's wall filament.");
            } else if (logical_filament <= 0 || region.inner_wall_filament_id.value != logical_filament ||
                       region.sparse_infill_filament_id.value != logical_filament ||
                       region.internal_solid_filament_id.value != logical_filament ||
                       region.top_surface_filament_id.value != logical_filament ||
                       region.bottom_surface_filament_id.value != logical_filament) {
                return reject(object, "SRL-A11", "outer_wall_filament_id", "Every feature of each body must use one filament.");
            }
            // Every filament was resolved above, so this cannot fail; it is still read fail-closed.
            const MixedNozzleToolResolution resolution = resolve_mixed_nozzle_tool(
                m_config, size_t(logical_filament - 1), MixedNozzleResolveScope::PhysicalToolOnly);
            if (!resolution)
                return reject_unresolved(object, *resolution.diagnostic);
            logical_filaments.emplace_back(unsigned(logical_filament - 1));
            physical_extruders.emplace_back(resolution.tool->physical_extruder);
            region_physical_extruders.emplace_back(resolution.tool->physical_extruder);
            regional_cadences.emplace_back(region.regional_layer_height.value == 0. ?
                                               object->config().layer_height.value :
                                               region.regional_layer_height.value);
            region_logical_filaments.emplace_back(size_t(logical_filament - 1));
            region_skin_filaments.emplace_back(size_t(skin_filament - 1));

            if (! regional_band_admits_sparse_infill_pattern(region.sparse_infill_pattern.value))
                return reject(object, "SRL-A13", "sparse_infill_pattern",
                              "This sparse infill pattern is not identical on both layers of a coarse band, so committing the pair would not print what the fine layers would have. Use grid, triangles, stars, or aligned rectilinear.");
            if (! regional_band_admits_sparse_infill_density(region.sparse_infill_density.value))
                return reject(object, "SRL-A14", "sparse_infill_density", "Body Split needs a sparse infill density above 0% and below 100%.");
            if (region.fill_multiline.value != 1)
                return reject(object, "SRL-A15", "fill_multiline", "Body Split needs one line per fill pass.");
            // A nonzero threshold turns small sparse islands into solid infill, and the band
            // preflight refuses any band surface that is not plain internal. Keeping it at zero is
            // what makes narrow features inside a coarse region bandable at all.
            if (! is_approx(region.minimum_sparse_infill_area.value, 0.))
                return reject(object, "SRL-A16", "minimum_sparse_infill_area",
                              "Set the minimum sparse infill threshold to 0 mm2 for Body Split: a nonzero threshold re-types small sparse islands as solid infill, which a coarse band cannot commit.");
            if (region.infill_combination.value)
                return reject(object, "SRL-A17", "infill_combination", "Turn off infill combination for Body Split.");
            if (! is_approx(region.top_shell_thickness.value, 0.) || ! is_approx(region.bottom_shell_thickness.value, 0.))
                return reject(object, "SRL-A18", "top_shell_thickness", "Turn off shell thickness overrides for Body Split.");
            // Wall options that read neighbouring layers, fuzzy skin, ironing, scarf seams and Z contouring
            // are all admitted: each cell gets its walls, neighbours, height and midplane from the native
            // generators on its own body's cadence.
            if (region.counterbore_hole_bridging.value != chbNone)
                return reject(object, "SRL-A21", "counterbore_hole_bridging", "Counterbore hole bridging is not supported by Body Split.");
            if (region.hole_to_polyhole.value)
                return reject(object, "SRL-A43", "hole_to_polyhole",
                              "Converting holes to polyholes is not supported by Body Split.");
            if (region.separated_infills.value || region.center_of_surface_pattern.value == CenterOfSurfacePattern::Each_Model)
                return reject(object, "SRL-A44", region.separated_infills.value ? "separated_infills" : "center_of_surface_pattern",
                              "Separated infill, or infill centred on each model, is not supported by Body Split.");
        }
        // A colour painted onto a body, or a modifier's area, prints at the cadence slicing gives it, not at
        // the layer height its region copies from the body: fine paint on a coarse body is laid at the fine
        // height.
        {
            const double base_height = object->config().layer_height.value;
            const std::vector<BodySplitRegionAssignment> bodies =
                collect_body_split_body_assignments(*model_object, base_height);
            const auto resolve_child = [&](const PrintObjectRegions::VolumeRegion &body, const PrintRegion *child) {
                if (child == nullptr || body.region == nullptr || body.model_volume == nullptr ||
                    !body.model_volume->is_model_part() || body.model_volume->extruder_id() <= 0)
                    return;
                const size_t region_id = size_t(child->print_object_region_id());
                if (region_id >= regional_cadences.size() || region_id == size_t(body.region->print_object_region_id()))
                    return;
                const double own_height = body.model_volume->config.has("regional_layer_height") ?
                    body.model_volume->config.opt_float("regional_layer_height") : 0.;
                if (const std::optional<double> cadence = resolve_body_split_child_cadence(
                        m_config, bodies, base_height, size_t(body.model_volume->extruder_id() - 1),
                        own_height != 0. ? own_height : base_height, region_logical_filaments[region_id]))
                    regional_cadences[region_id] = *cadence;
            };
            for (const PrintObjectRegions::LayerRangeRegions &layer_range : object->shared_regions()->layer_ranges) {
                const std::vector<PrintObjectRegions::VolumeRegion> &volume_regions = layer_range.volume_regions;
                for (const PrintObjectRegions::PaintedRegion &painted : layer_range.painted_regions)
                    if (painted.parent >= 0 && size_t(painted.parent) < volume_regions.size())
                        resolve_child(volume_regions[size_t(painted.parent)], painted.region);
                // A modifier's area belongs to the body it sits in, through any nested modifiers.
                for (const PrintObjectRegions::VolumeRegion &modifier : volume_regions) {
                    if (modifier.model_volume == nullptr || !modifier.model_volume->is_modifier())
                        continue;
                    int parent = modifier.parent;
                    while (parent >= 0 && size_t(parent) < volume_regions.size() &&
                           volume_regions[size_t(parent)].model_volume != nullptr &&
                           !volume_regions[size_t(parent)].model_volume->is_model_part())
                        parent = volume_regions[size_t(parent)].parent;
                    if (parent >= 0 && size_t(parent) < volume_regions.size())
                        resolve_child(volume_regions[size_t(parent)], modifier.region);
                }
            }
        }
        // The per-region filament sets above are inflated by PrintApply.cpp's ghost painted regions.
        // Filament, tool and cadence facts below come from each body's own ModelVolume, which reports
        // no ghosts.
        const std::vector<BodySplitRegionAssignment> body_assignments =
            collect_body_split_volume_assignments(*model_object, object->config().layer_height.value);
        std::vector<size_t> body_physical_extruders;
        for (const BodySplitRegionAssignment &assignment : body_assignments) {
            const MixedNozzleToolResolution resolution = resolve_mixed_nozzle_tool(
                m_config, assignment.logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
            if (! resolution)
                return reject_unresolved(object, *resolution.diagnostic);
            body_physical_extruders.push_back(resolution.tool->physical_extruder);
        }
        // A modifier that names a filament puts it on a tool, so it has to resolve.
        for (const ModelVolume *volume : model_object->volumes) {
            if (! volume->is_modifier() || ! volume->config.has("extruder") || volume->config.extruder() <= 0)
                continue;
            const size_t modifier_filament = size_t(volume->config.extruder() - 1);
            const MixedNozzleToolResolution resolution = resolve_mixed_nozzle_tool(
                m_config, modifier_filament, MixedNozzleResolveScope::PhysicalToolOnly);
            if (! resolution)
                return reject_unresolved(object, *resolution.diagnostic);
        }
        sort_remove_duplicates(body_physical_extruders);
        // Every synchronized body's filaments, its own and any painted colours, must resolve onto one
        // of the two configured tools, and both tools must be used. On a toolchanger these are the two
        // tools this plate prints on.
        const std::vector<size_t> body_tools = m_config.nozzle_diameter.values.size() > 2 ?
            mixed_nozzle_used_tools : std::vector<size_t>{0, 1};
        if (body_tools.size() != 2 || body_physical_extruders != body_tools)
            return reject(object, "SRL-A22", "filament_map",
                          "Every body's filament must be assigned to one of the two installed nozzles, and both nozzles must be used.");

        const double base_cadence = object->config().layer_height.value;
        // Painted colours contribute tool/material usage above, but do not inherit the
        // parent body's cadence. Cross-tool paint receives its target cadence during native
        // ownership projection; classify the bodies here from their explicit assignments.
        std::vector<BodySplitRegionAssignment> body_cadence_assignments =
            collect_body_split_body_assignments(*model_object, base_cadence);
        // The native painting projector can create a base-height fine region inside one
        // coarse body, and so can a modifier on the fine filament. Qualify only actually painted
        // colours and modifier filaments with the same resolver used by the projector and
        // PrintRegion::flow; a ghost region is not evidence of fine usage.
        if (single_painted_body && body_cadence_assignments.size() == 1) {
            const BodySplitRegionAssignment coarse = body_cadence_assignments.front();
            for (const BodySplitRegionAssignment &painted : body_assignments)
                if (resolve_mixed_nozzle_cadence(m_config, base_cadence, coarse.cadence,
                                                painted.logical_filament, coarse.logical_filament))
                    body_cadence_assignments.push_back({base_cadence, painted.logical_filament});
        }
        int coarse_cadence_ratio_seen = 0;
        size_t base_cadence_assignments = 0;
        size_t base_cadence_tool = size_t(-1);
        size_t coarse_cadence_tool = size_t(-1);
        std::optional<size_t> base_cadence_filament;
        std::optional<size_t> coarse_cadence_filament;
        double coarse_cadence_value = 0.;
        bool base_cadence_tool_mismatch = false;
        // The first layer need not equal the base cadence: both bodies print it as one shared cell 0 at
        // initial_layer_print_height and start their cadence above it. It still has to fit each nozzle's
        // layer height range, checked below.
        bool cadence_invalid = !std::isfinite(base_cadence) || base_cadence <= 0.;
        for (const BodySplitRegionAssignment &assignment : body_cadence_assignments) {
            if (cadence_invalid || !std::isfinite(assignment.cadence) || assignment.cadence <= 0.) {
                cadence_invalid = true;
                continue;
            }
            const double raw_ratio = assignment.cadence / base_cadence;
            const int integer_ratio = int(std::lround(raw_ratio));
            if (!is_approx(raw_ratio, double(integer_ratio))) {
                cadence_invalid = true;
                continue;
            }
            // Already resolved above; re-resolved here and read fail-closed.
            const MixedNozzleToolResolution resolution = resolve_mixed_nozzle_tool(
                m_config, assignment.logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
            if (! resolution)
                return reject_unresolved(object, *resolution.diagnostic);
            const size_t tool = resolution.tool->physical_extruder;
            if (integer_ratio == 1) {
                ++base_cadence_assignments;
                if (base_cadence_tool == size_t(-1)) {
                    base_cadence_tool = tool;
                    base_cadence_filament = assignment.logical_filament;
                } else if (base_cadence_tool != tool)
                    base_cadence_tool_mismatch = true;   // unreachable, kept fail-closed
            } else if (integer_ratio >= 2 && coarse_cadence_ratio_seen == 0) {
                coarse_cadence_ratio_seen = integer_ratio;
                coarse_cadence_tool = tool;
                coarse_cadence_filament = assignment.logical_filament;
                coarse_cadence_value = assignment.cadence;
            } else if (integer_ratio >= 2 && tool == coarse_cadence_tool) {
                // A further coarse-cadence entry on the same physical tool (a painted colour or a second
                // coarse body) must resolve the same layer height, so the plan's single per-tool band
                // schedule holds.
                if (! is_approx(assignment.cadence, coarse_cadence_value))
                    return reject(object, "SRL-A45", "regional_layer_height",
                                  "Every coarse body in an object must use the same layer height and the same nozzle.");
            } else {
                cadence_invalid = true;
            }
        }
        if (cadence_invalid || base_cadence_assignments < 1 || base_cadence_tool_mismatch ||
            coarse_cadence_ratio_seen < 2 || !base_cadence_filament || !coarse_cadence_filament) {
            // One part split only by a modifier on the other nozzle's filament (is_body_split_object()): say so,
            // since nothing on the plate looks like two bodies.
            const ModelVolume *lone_part = nullptr;
            size_t parts = 0;
            for (const ModelVolume *volume : model_object->volumes)
                if (volume != nullptr && volume->is_model_part()) {
                    ++parts;
                    lone_part = volume;
                }
            if (parts == 1 && !lone_part->is_mm_painted()) {
                const int part_filament = lone_part->extruder_id() > 0 ? lone_part->extruder_id() : 1;
                const std::optional<size_t> part_tool = physical_extruder_for_filament(m_config, unsigned(part_filament - 1));
                for (const ModelVolume *volume : model_object->volumes) {
                    if (volume == nullptr || !volume->is_modifier() || !volume->config.has("extruder") || volume->config.extruder() <= 0)
                        continue;
                    const int filament = volume->config.extruder();
                    const std::optional<size_t> tool = physical_extruder_for_filament(m_config, unsigned(filament - 1));
                    if (!part_tool || !tool || *tool == *part_tool || *tool >= m_config.nozzle_diameter.values.size() ||
                        *part_tool >= m_config.nozzle_diameter.values.size())
                        continue;
                    const bool part_coarse = m_config.nozzle_diameter.values[*part_tool] > m_config.nozzle_diameter.values[*tool];
                    return reject(object, "SRL-A23", "regional_layer_height", Slic3r::format(
                        "This object is one part with a modifier (\"%1%\") on filament %2%, which prints on the other nozzle, so "
                        "Body Split slices it as two bodies, and the coarse one needs a coarse layer height. %3%",
                        volume->name, filament,
                        part_coarse ?
                            "Set the part's layer height to a whole multiple of the base layer height, or set the modifier "
                            "to a filament on the part's own nozzle." :
                            "A modifier cannot be the coarse body: set the modifier to a filament on the part's own nozzle, "
                            "or make that area a separate part with a coarse layer height."));
                }
            }
            return reject(object, "SRL-A23", "regional_layer_height",
                          "Body Split needs fine bodies at the base layer height and coarse bodies at one shared whole "
                          "multiple of it.");
        }
        // A second colour on the coarse tool is admitted: it prints at the shared coarse cadence, so the
        // band schedule stays one per tool. The ratio, tool distinctness and nozzle ordering come from
        // the shared cadence resolver, whose diagnostic code is reported in this sentence.
        const MixedNozzleCadenceResolution resolved_body_cadence = resolve_mixed_nozzle_cadence(
            m_config, base_cadence, coarse_cadence_value, *base_cadence_filament, *coarse_cadence_filament);
        if (!resolved_body_cadence)
            return reject(object, "SRL-A23", "regional_layer_height",
                          Slic3r::format("[%1%] %2%", resolved_body_cadence.diagnostic->stable_code(),
                                         resolved_body_cadence.diagnostic->message()));
        const int coarse_cadence_ratio = resolved_body_cadence.cadence->ratio;
        plate_coarse_cadence_ratios.push_back(coarse_cadence_ratio);
        // Fine skins are admitted at any region count: converted cells route to the fine body
        // (PrintObjectSlice.cpp, fine_skin_partner), fine-skin planes stay inside the body that opted in,
        // and painted regions never convert. A region's skin filament may differ from its wall filament
        // only on the coarse-cadence region, and only to the fine region's filament, so the object still
        // uses exactly two logical filaments. Without mixed_nozzle_body_fine_skins the divergence was
        // already refused above, so every iteration continues.
        for (size_t region_id = 0; region_id < region_skin_filaments.size(); ++region_id) {
            if (region_skin_filaments[region_id] == region_logical_filaments[region_id])
                continue;
            if (region_logical_filaments[region_id] != *coarse_cadence_filament)
                return reject(object, "SRL-A47", "internal_solid_filament_id",
                              "Only the coarse body may print its internal solid infill, top surface and bottom surface with a different filament from its own walls.");
            if (region_skin_filaments[region_id] != *base_cadence_filament)
                return reject(object, "SRL-A47", "internal_solid_filament_id",
                              "The coarse body's internal solid infill, top surface and bottom surface must use the fine body's filament, because Body Split prints with exactly two nozzles.");
        }
        if (model_object->has_custom_layering() || object->config().precise_z_height.value ||
            object->config().slicing_mode.value != SlicingMode::Regular)
            return reject(object, "SRL-A24", "layer_height", "Custom, precise, or variable layer heights are not supported by Body Split.");
        if (object->config().wall_generator.value != PerimeterGeneratorType::Classic)
            return reject(object, "SRL-A25", "wall_generator", "Body Split needs the classic wall generator.");
        // Interface shells, thick bridges and the extra bridge layer are admitted: surface detection and
        // both bridge passes read each region's own neighbouring cell (an absent cell reads as empty),
        // and LayerRegion::bridging_flow takes the cell's own tool. Segmented region width and
        // interlocking depth are admitted too: Orca's segmentation runs on the native views row by row,
        // with the depth alternation following the coarse cadence (paint_interlocking_phases).
        // On a layer with more than one region, elephant foot compensation trims every region with an
        // outline built from region 0's external perimeter flow, which on a mixed-nozzle layer would
        // govern one tool's first layers by the other tool's bead width. Pin it off.
        if (object->config().elefant_foot_compensation.value > 0. && object->config().elefant_foot_compensation_layers.value > 0)
            return reject(object, "SRL-A27", "elefant_foot_compensation", "Elephant foot compensation is not supported by Body Split; set it to 0.");
        // Conditional admission of interlocking_beam follows the selected integer cadence ratio.
        if (object->config().interlocking_beam.value) {
            if (object->config().interlocking_beam_layer_count.value != coarse_cadence_ratio)
                return reject(object, "SRL-A28", "interlocking_beam_layer_count",
                              "The interlocking beam layer count must match how many fine layers make one coarse layer; any other value splits beam rows across coarse layers.");
        }
        if (object->has_support() || object->has_raft()) {
            // Raft reuses support_filament/support_interface_filament, so one block covers both, as in the
            // Feature Split block above.
            const bool via_support = object->has_support();
            const int base_filament = object->config().support_filament.value;
            const int interface_filament = object->config().support_interface_filament.value;
            if (base_filament <= 0 || interface_filament <= 0)
                return reject(object, "SRL-A29", base_filament <= 0 ? "support_filament" : "support_interface_filament",
                              "Set the support filament (Support/raft base) to a specific filament. At Default it prints with whichever filament is active, which can be on either nozzle.");
            // An unresolved support filament is reported by reject_unresolved: one diagnostic per failure
            // kind rather than per config key.
            const MixedNozzleToolResolution base_tool = resolve_mixed_nozzle_tool(
                m_config, size_t(base_filament - 1), MixedNozzleResolveScope::PhysicalToolOnly);
            if (!base_tool) return reject_unresolved(object, *base_tool.diagnostic);
            const MixedNozzleToolResolution interface_tool = resolve_mixed_nozzle_tool(
                m_config, size_t(interface_filament - 1), MixedNozzleResolveScope::PhysicalToolOnly);
            if (!interface_tool) return reject_unresolved(object, *interface_tool.diagnostic);
            const double base_dmr      = m_config.nozzle_diameter.get_at(base_tool.tool->physical_extruder);
            const double interface_dmr = m_config.nozzle_diameter.get_at(interface_tool.tool->physical_extruder);
            if (interface_dmr > base_dmr)
                return reject(object, "SRL-A29", "support_interface_filament",
                              "The support interface is on a larger nozzle than the support base. The interface is what touches the model, so set Support/raft interface to Default or to a filament on the base's nozzle or a finer one.");
            // Same tree support admission as the Feature Split block above.
            if (const MixedNozzleSupportRefusal refusal = mixed_nozzle_support_refusal(m_config, object->config(), base_dmr, interface_dmr, via_support);
                ! refusal.why.empty())
                return reject(object, refusal.tree ? "SRL-A53" : "SRL-A55", refusal.opt_key, refusal.why);
            // Body Split plans each body's layers from the bed. On a raft the first coarse layers, the
            // plan block and the prime tower levels do not line up with the raised object, and the slice
            // failed later with an internal message (SRL-A38, SRL-C02 code 10, SRL-TOWER-STRUCTURE or
            // "empty layers"). Refused here, naming the setting.
            if (object->has_raft())
                return reject(object, "SRL-A54", "raft_layers",
                              "Body Split cannot print on a raft yet. Set Raft layers to 0 for this object, or turn Body Split off for this plate.");
            // Same raft-step admission as the Feature Split block above, under this mode's code. One helper,
            // so the two modes cannot disagree.
            if (m_config.enable_prime_tower.value || this->has_wipe_tower()) {
                for (const MixedNozzleRaftStep &step : mixed_nozzle_raft_steps(m_config, *object))
                    if (! mixed_nozzle_raft_step_carriable(m_config, step.height, step.nozzle))
                        return reject(object, "SRL-A49", "raft_layers", Slic3r::format(
                            "The raft lays a %1% mm layer on nozzle %2%, which prints between %3% mm and %4% mm. "
                            "No whole number of prime-tower levels fits that step, so the tower cannot stand beside the raft. "
                            "Use fewer raft layers, or move %5% to a nozzle that can lay this height.",
                            step.height, step.nozzle + 1,
                            resolved_min_layer_height(m_config, step.nozzle),
                            resolved_max_layer_height(m_config, step.nozzle),
                            step.filament_key));
            }
        }
        if (object->config().flush_into_infill.value || object->config().flush_into_objects.value || object->config().flush_into_support.value)
            return reject(object, "SRL-A30", "flush_into_infill", "Wiping into the object is not supported by Body Split.");
        // The shared first layer joins this loop: every installed tool deposits on it, so it has to fit
        // each tool's layer height range, like each regional cadence. Slicing resolves a non-positive
        // initial_layer_print_height to the object's layer height, so check the height actually laid.
        const double shared_first_layer = m_config.initial_layer_print_height.value > 0. ?
            m_config.initial_layer_print_height.value : base_cadence;
        // A body whose nozzle cannot lay the base or the shared first layer starts on the bed with a
        // thicker first cell of whole base rows and holds every cell to that nozzle's minimum
        // (PrintObjectSlice.cpp, RegionalGrids.cpp). Still refused: a cadence outside the nozzle's
        // limits, a fine body that cannot lay the base or first layer, a first cell above the maximum,
        // and a prime tower on such a pair when the lagging tower is unavailable (smooth timelapse,
        // wrapping detection). One helper answers this for admission, the wizard and the cadence editor.
        const bool with_prime_tower = m_config.enable_prime_tower.value || this->has_wipe_tower();
        const bool lagging_tower_available = mixed_nozzle_tower_lagging_available(m_config);
        for (size_t region_id = 0; region_id < regional_cadences.size(); ++region_id) {
            const size_t nozzle = region_physical_extruders[region_id];
            const BodySplitToolEnvelope envelope = body_split_tool_envelope(
                m_config, nozzle, base_cadence, regional_cadences[region_id], shared_first_layer);
            if (envelope == BodySplitToolEnvelope::Unsupported)
                return reject(object, "SRL-A38", "max_layer_height",
                              "A body's layer height or the shared first layer is outside what its nozzle can print, or the base layer height is below that nozzle's minimum.");
            if (!body_split_tool_envelope_admitted(envelope, with_prime_tower, lagging_tower_available))
                return reject(object, "SRL-A38", "timelapse_type", Slic3r::format(
                    "Nozzle %1% cannot lay the %2% mm base layer or the %3% mm first layer, so its body starts "
                    "on the bed with a thicker first cell, and the prime tower has to lag behind the part to give each "
                    "nozzle a layer it can lay. Smooth timelapse and wrapping detection keep the tower level with the "
                    "part. Turn those off, or pick a base layer height of at least %4% mm.",
                    nozzle + 1, base_cadence, shared_first_layer, resolved_min_layer_height(m_config, nozzle)));
        }
        } // end per-object loop

        // Print-level post-checks: these read m_config, not per-object config, so they run once.
        const PrintObject *const any_object = m_objects.front();
        // Every object on the plate must resolve the same coarse cadence ratio so band boundaries align.
        // The list cannot be empty: the loop above rejects before falling through, and validate()
        // returns early on an empty plate.
        if (std::any_of(plate_coarse_cadence_ratios.begin(), plate_coarse_cadence_ratios.end(),
                        [&](int ratio) { return ratio != plate_coarse_cadence_ratios.front(); }))
            return reject(any_object, "SRL-A46", "regional_layer_height",
                          "Every object on a Body Split plate must use the same ratio of coarse to fine layer height, so coarse layers line up across objects.");
        if (m_config.print_sequence.value != PrintSequence::ByLayer)
            return reject(any_object, "SRL-A31", "print_sequence", "Body Split needs the print sequence set to by layer.");
        if (m_config.spiral_mode.value)
            return reject(any_object, "SRL-A33", "spiral_mode", "Spiral mode is not supported by Body Split.");
        const auto has_custom_sequence = [](const std::vector<int> &sequence) {
            return std::any_of(sequence.begin(), sequence.end(), [](int filament) { return filament != 0; });
        };
        if (m_config.toolchange_ordering.value != ToolChangeOrderingType::Default ||
            has_custom_sequence(m_config.first_layer_print_sequence.values) || has_custom_sequence(m_config.other_layers_print_sequence.values) ||
            m_config.other_layers_print_sequence_nums.value != 0)
            return reject(any_object, "SRL-A34", "toolchange_ordering", "Custom or cyclic tool ordering is not supported by Body Split.");
        if (m_config.resonance_avoidance.value)
            return reject(any_object, "SRL-A35", "resonance_avoidance", "Resonance avoidance is not supported by Body Split.");
        // Every configured filament, not only the first two: a synchronized region may sit on any
        // filament index.
        for (size_t logical_filament = 0; logical_filament < m_config.filament_adaptive_volumetric_speed.size(); ++ logical_filament)
            if (m_config.filament_adaptive_volumetric_speed.get_at(logical_filament))
                return reject(any_object, "SRL-A36", "filament_adaptive_volumetric_speed", "Adaptive volumetric speed is not supported by Body Split.");
        if (std::any_of(m_config.slicing_pipeline_plugin.values.begin(), m_config.slicing_pipeline_plugin.values.end(),
                        [](const std::string &plugin) { return ! plugin.empty(); }))
            return reject(any_object, "SRL-A37", "slicing_pipeline_plugin", "Mutating slicing-pipeline plugins are not supported by Body Split.");
        // Fully manual nozzle mapping resolves extruders from filament_map like any manual mode. The
        // engine also builds a nozzle-level grouping from filament_volume_map and filament_nozzle_map and
        // silently skips it when either is mis-sized, which stale projects can produce. Report it.
        if (m_config.filament_map_mode.value == fmmNozzleManual) {
            // Size against filament_diameter, never filament_colour: colour is optional UI metadata whose
            // stock default is one entry.
            const size_t filament_count = m_config.filament_diameter.size();
            if (m_config.filament_nozzle_map.size() != filament_count || m_config.filament_volume_map.size() != filament_count)
                return reject(any_object, "SRL-A39",
                              m_config.filament_nozzle_map.size() != filament_count ? "filament_nozzle_map" : "filament_volume_map",
                              "Filaments are assigned to nozzles by hand, but the nozzle lists do not match the number of filaments. Open Filament grouping and assign each filament to a nozzle again.");
        }
        // The flush volume matrix holds one filament_count x filament_count block per physical nozzle,
        // and its consumers (_make_wipe_tower(), ToolOrdering, GCode, WipeTower2, PresetBundle) index
        // those blocks without checking. Measure the nozzle count they use, not flush_multiplier's
        // length, or a one-head matrix on a two-nozzle machine lets the tower read past its slice.
        // Scoped to synchronized prints, although every multi-filament print has the same hazard.
        const size_t flush_filaments = m_config.filament_diameter.size();
        const size_t flush_nozzles   = m_config.nozzle_diameter.size();
        if (flush_filaments > 1 && m_config.flush_volumes_matrix.size() != flush_filaments * flush_filaments * flush_nozzles)
            return reject(any_object, "SRL-A40", "flush_volumes_matrix",
                          Slic3r::format("The flush volume matrix has %1% entries, but %2% filaments across %3% nozzles need %4%.",
                                         m_config.flush_volumes_matrix.size(), flush_filaments, flush_nozzles,
                                         flush_filaments * flush_filaments * flush_nozzles));

        // Run the export's own check early so it cannot fail after the whole slice. append_full_config()
        // rebuilds the matrix per head from filament_colour and throws unless one colour is declared or
        // colours^2 entries exist per flush_multiplier head, and it divides by flush_multiplier.size(),
        // so an empty vector is refused too. This changes no purge volume, only which configs are refused.
        const size_t export_colours = m_config.filament_colour.size();
        const size_t export_heads   = m_config.flush_multiplier.size();
        if (export_heads == 0)
            return reject(any_object, "SRL-A40", "flush_multiplier",
                          "The flush multiplier is empty. The export divides the flush volume matrix by the number of flush-multiplier heads, so at least one is required.");
        if (export_colours > 1 && m_config.flush_volumes_matrix.size() != export_colours * export_colours * export_heads)
            return reject(any_object, "SRL-A40", "flush_volumes_matrix",
                          Slic3r::format("The flush volume matrix has %1% entries, but the export rebuilds it as %2% filament colours across %3% flush-multiplier heads, needing %4%.",
                                         m_config.flush_volumes_matrix.size(), export_colours, export_heads,
                                         export_colours * export_colours * export_heads));
    }

    if (nozzles < 2 && extruders.size() > 1) {
        auto ret = check_multi_filament_valid(*this);
        if (!ret.string.empty())
        {
            ret.type = STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP;
            if (ret.is_warning) {
                add_warning(ret);
            }else
                return ret;
        }
    }

    if (m_config.print_sequence == PrintSequence::ByObject && (m_objects.size() > 1 || m_objects[0]->instances().size() > 1)) {
        if (m_config.timelapse_type == TimelapseType::tlSmooth)
            return {L("Smooth mode of timelapse is not supported when \"by object\" sequence is enabled.")};

        if (m_config.enable_wrapping_detection) {
            StringObjectException clumping_detection_setting_err;
            clumping_detection_setting_err.string = L("Clumping detection is not supported when \"by object\" sequence is enabled.");
            clumping_detection_setting_err.opt_key = "enable_wrapping_detection";
            return clumping_detection_setting_err;
        }

        //BBS: refine seq-print validation logic
        auto ret = sequential_print_clearance_valid(*this, collison_polygons, height_polygons);
        if (!ret.string.empty()) {
            ret.type = STRING_EXCEPT_OBJECT_COLLISION_IN_SEQ_PRINT;
            return ret;
        }
    }
    else {
        //BBS
        StringObjectException layer_warning;
        auto ret = layered_print_cleareance_valid(*this, &layer_warning);
        if (!ret.string.empty()) {
            ret.type = STRING_EXCEPT_OBJECT_COLLISION_IN_LAYER_PRINT;
            return ret;
        }
        if (!layer_warning.string.empty())
            add_warning(layer_warning);
    }

    if (m_config.enable_prime_tower) {
        for (const PrintObject* object : m_objects) {
            if (object->config().precise_z_height.value) {
                warn(L("Enabling both precise Z height and the prime tower may cause slicing errors."), "precise_z_height");
                break;
            }
        }
    } else {
        if (m_config.enable_wrapping_detection)
            warn(L("A prime tower is required for clumping detection; otherwise, there may be flaws on the model."), "enable_prime_tower");
    }

    if (m_config.spiral_mode) {
        size_t total_copies_count = 0;
        for (const PrintObject* object : m_objects)
            total_copies_count += object->instances().size();
        // #4043
        if (total_copies_count > 1 && m_config.print_sequence != PrintSequence::ByObject)
            return {L("Please select \"By object\" print sequence to print multiple objects in spiral vase mode."), nullptr, "spiral_mode"};
        assert(m_objects.size() == 1);
        const auto all_regions = m_objects.front()->all_regions();
        if (all_regions.size() > 1) {
            // Orca: make sure regions are not compatible
            if (std::any_of(all_regions.begin() + 1, all_regions.end(), [this, ra = all_regions.front()](const auto rb) {
                return !Layer::is_perimeter_compatible(*this, ra, rb);
            })) {
                return {L("Spiral (vase) mode does not work when an object contains more than one material."), nullptr, "spiral_mode"};
            }
        }
    }

    // Cache of layer height profiles for checking:
    // 1) Whether all layers are synchronized if printing with wipe tower and / or unsynchronized supports.
    // 2) Whether layer height is constant for Organic supports.
    // 3) Whether build volume Z is not violated.
    std::vector<std::vector<coordf_t>> layer_height_profiles;
    auto layer_height_profile = [this, &layer_height_profiles](const size_t print_object_idx) -> const std::vector<coordf_t>& {
        const PrintObject       &print_object = *m_objects[print_object_idx];
        if (layer_height_profiles.empty())
            layer_height_profiles.assign(m_objects.size(), std::vector<coordf_t>());
        std::vector<coordf_t>   &profile      = layer_height_profiles[print_object_idx];
        if (profile.empty())
            PrintObject::update_layer_height_profile(*print_object.model_object(), print_object.slicing_parameters(), profile);
        return profile;
    };

    // Checks that the print does not exceed the max print height
    for (size_t print_object_idx = 0; print_object_idx < m_objects.size(); ++ print_object_idx) {
        const PrintObject &print_object = *m_objects[print_object_idx];
        //FIXME It is quite expensive to generate object layers just to get the print height!
        if (auto layers = generate_object_layers(print_object.slicing_parameters(), layer_height_profile(print_object_idx), print_object.config().precise_z_height.value);
            !layers.empty()) {

            Vec3d test =this->shrinkage_compensation();
            const double shrinkage_compensation_z = this->shrinkage_compensation().z();
            
            if (shrinkage_compensation_z != 1. && layers.back() > (this->config().printable_height / shrinkage_compensation_z + EPSILON)) {
                // The object exceeds the maximum build volume height because of shrinkage compensation.
                return StringObjectException{
                    Slic3r::format(_u8L("While the object %1% itself fits the build volume, it exceeds the maximum build volume height because of material shrinkage compensation."), print_object.model_object()->name),
                    print_object.model_object(),
                    ""
                };
            } else if (layers.back() > this->config().printable_height + EPSILON) {
                // Test whether the last slicing plane is below or above the print volume.
                return StringObjectException{
                    0.5 * (layers[layers.size() - 2] + layers.back()) > this->config().printable_height + EPSILON ?
                    Slic3r::format(_u8L("The object %1% exceeds the maximum build volume height."), print_object.model_object()->name) :
                    Slic3r::format(_u8L("While the object %1% itself fits the build volume, its last layer exceeds the maximum build volume height."), print_object.model_object()->name) +
                    " " + _u8L("You might want to reduce the size of your model or change current print settings and retry."),
                    print_object.model_object(),
                    ""
                };
            }
        }
    }

    // Some of the objects has variable layer height applied by painting or by a table.
    bool has_custom_layering = std::find_if(m_objects.begin(), m_objects.end(), 
        [](const PrintObject *object) { return object->model_object()->has_custom_layering(); }) 
        != m_objects.end();

    // Custom layering is not allowed for tree supports as of now.
    for (size_t print_object_idx = 0; print_object_idx < m_objects.size(); ++ print_object_idx)
        if (const PrintObject &print_object = *m_objects[print_object_idx];
            print_object.has_support_material() && is_tree(print_object.config().support_type.value) && (print_object.config().support_style.value == smsTreeOrganic || 
                // Orca: use organic as default
                print_object.config().support_style.value == smsDefault) &&
            print_object.model_object()->has_custom_layering()) {
            if (const std::vector<coordf_t> &layers = layer_height_profile(print_object_idx); ! layers.empty())
                if (! check_object_layers_fixed(print_object.slicing_parameters(), layers))
                    return {_u8L("Variable layer height is not supported with Organic supports.") };
        }

    if (this->has_wipe_tower() && ! m_objects.empty()) {
        // Make sure all extruders use same diameter filament and have the same nozzle diameter
        // EPSILON comparison is used for nozzles and 10 % tolerance is used for filaments
        double first_nozzle_diam = m_config.nozzle_diameter.get_at(extruders.front());
        double first_filament_diam = m_config.filament_diameter.get_at(extruders.front());
        for (const auto& extruder_idx : extruders) {
            double nozzle_diam = m_config.nozzle_diameter.get_at(extruder_idx);
            double filament_diam = m_config.filament_diameter.get_at(extruder_idx);
            if (nozzle_diam - EPSILON > first_nozzle_diam || nozzle_diam + EPSILON < first_nozzle_diam
                || std::abs((filament_diam - first_filament_diam) / first_filament_diam) > 0.1) {
                // return { L("Different nozzle diameters and different filament diameters may not work well when prime tower is enabled. It's very experimental, please proceed with caucious.") };
                    warn(L("Different nozzle diameters and different filament diameters may not work well when the prime tower is enabled. It's very experimental, so please proceed with caution."), "nozzle_diameter");
                    break;
                }
        }

        if (! m_config.use_relative_e_distances)
            return { L("The Wipe Tower is currently only supported with the relative extruder addressing (use_relative_e_distances=1).") };

        if (m_config.ooze_prevention && m_config.single_extruder_multi_material)
            return {L("Ooze prevention is only supported with the wipe tower when 'single_extruder_multi_material' is off.")};
            
#if 0
        if (m_config.gcode_flavor != gcfRepRapSprinter && m_config.gcode_flavor != gcfRepRapFirmware &&
            m_config.gcode_flavor != gcfRepetier && m_config.gcode_flavor != gcfMarlinLegacy && m_config.gcode_flavor != gcfMarlinFirmware)
            return { L("The prime tower is currently only supported for the Marlin, RepRap/Sprinter, RepRapFirmware and Repetier G-code flavors.")};

        if ((m_config.print_sequence == PrintSequence::ByObject) && extruders.size() > 1)
            return { L("A prime tower is not supported in \u201cBy object\u201d print."), nullptr, "enable_prime_tower" };

        // BBS: When prime tower is on, object layer and support layer must be aligned. So support gap should be multiple of object layer height.
        for (size_t i = 0; i < m_objects.size(); i++) {
            const PrintObject* object = m_objects[i];
            const SlicingParameters& slicing_params = object->slicing_parameters();
            if (object->config().adaptive_layer_height) {
                return  { L("A prime tower is not supported when adaptive layer height is on. It requires that all objects have the same layer height."), object, "adaptive_layer_height" };
            }

            if (!object->config().enable_support)
                continue;

            double gap_layers = slicing_params.gap_object_support / slicing_params.layer_height;
            if (gap_layers - (int)gap_layers > EPSILON) {
                return {L("A prime tower requires any \u201csupport gap\u201d to be a multiple of layer height."), object};
            }
        }
#endif

        if (m_objects.size() > 1) {
            const SlicingParameters &slicing_params0 = m_objects.front()->slicing_parameters();
            size_t                  tallest_object_idx = 0;
            for (size_t i = 1; i < m_objects.size(); ++ i) {
                const PrintObject       *object         = m_objects[i];
                const SlicingParameters &slicing_params = object->slicing_parameters();
                if (std::abs(slicing_params.first_print_layer_height - slicing_params0.first_print_layer_height) > EPSILON ||
                    std::abs(slicing_params.layer_height             - slicing_params0.layer_height            ) > EPSILON)
                    return {L("A prime tower requires that all objects have the same layer height."), object, "initial_layer_print_height"};
                if (slicing_params.raft_layers() != slicing_params0.raft_layers())
                    return {L("A prime tower requires that all objects are printed over the same number of raft layers."), object, "raft_layers"};
                // BBS: support gap can be multiple of object layer height, remove _L()
#if 0
                if (slicing_params0.gap_object_support != slicing_params.gap_object_support ||
                    slicing_params0.gap_support_object != slicing_params.gap_support_object)
                    return {L("The prime tower is only supported for multiple objects if they are printed with the same support_top_z_distance."), object};
#endif
                // Under Feature Split and Body Split the layering is fixed, so the adaptive-layer envelope
                // min/max_layer_height is not part of it. It comes from each object's nozzles, so a plain
                // object on the coarse nozzle would otherwise differ while laying the same layers.
                SlicingParameters envelope_free = slicing_params;
                if (is_mixed_nozzle_slicing_enabled(m_config)) {
                    envelope_free.min_layer_height = slicing_params0.min_layer_height;
                    envelope_free.max_layer_height = slicing_params0.max_layer_height;
                }
                if (!equal_layering(envelope_free, slicing_params0))
                    return  { L("A prime tower requires that all objects are sliced with the same layer height."), object };
                if (has_custom_layering) {
                    auto &lh         = layer_height_profile(i);
                    auto &lh_tallest = layer_height_profile(tallest_object_idx);
                    if (*(lh.end() - 2) > *(lh_tallest.end() - 2))
                        tallest_object_idx = i;
                }
            }

            // BBS: remove obsolete logics and _L()
            if (has_custom_layering) {
                std::vector<std::vector<coordf_t>> layer_z_series;
                layer_z_series.assign(m_objects.size(), std::vector<coordf_t>());
               
                for (size_t idx_object = 0; idx_object < m_objects.size(); ++idx_object) {
                    layer_z_series[idx_object] = generate_object_layers(m_objects[idx_object]->slicing_parameters(), layer_height_profiles[idx_object], m_objects[idx_object]->config().precise_z_height.value);
                }

                for (size_t idx_object = 0; idx_object < m_objects.size(); ++idx_object) {
                    if (idx_object == tallest_object_idx) continue;
                    // Check that the layer height profiles are equal. This will happen when one object is
                    // a copy of another, or when a layer height modifier is used the same way on both objects.
                    // The latter case might create a floating point inaccuracy mismatch, so compare
                    // element-wise using an epsilon check.
                    size_t         i   = 0;
                    const coordf_t eps = 0.5 * EPSILON; // layers closer than EPSILON will be merged later. Let's make
                    // this check a bit more sensitive to make sure we never consider two different layers as one.
                    while (i < layer_height_profiles[idx_object].size() && i < layer_height_profiles[tallest_object_idx].size()) {
                        // BBS: remove the break condition, because a variable layer height object and a new object will not be checked when slicing
                        //if (i % 2 == 0 && layer_height_profiles[tallest_object_idx][i] > layer_height_profiles[idx_object][layer_height_profiles[idx_object].size() - 2])
                        //    break;
                        if (std::abs(layer_height_profiles[idx_object][i] - layer_height_profiles[tallest_object_idx][i]) > eps)
                            return {L("The prime tower is only supported if all objects have the same variable layer height.")};
                        ++i;
                    }
                }
            }
        }
    }

	{
		// Find the smallest used nozzle diameter and the number of unique nozzle diameters.
		double min_nozzle_diameter = std::numeric_limits<double>::max();
		double max_nozzle_diameter = 0;
		for (unsigned int extruder_id : extruders) {
			double dmr = m_config.nozzle_diameter.get_at(extruder_id);
			min_nozzle_diameter = std::min(min_nozzle_diameter, dmr);
			max_nozzle_diameter = std::max(max_nozzle_diameter, dmr);
		}

        // BBS: remove L()
#if 0
        // We currently allow one to assign extruders with a higher index than the number
        // of physical extruders the machine is equipped with, as the Printer::apply() clamps them.
        unsigned int total_extruders_count = m_config.nozzle_diameter.size();
        for (const auto& extruder_idx : extruders)
            if ( extruder_idx >= total_extruders_count )
                return {L("One or more object were assigned an extruder that the printer does not have.")};
#endif

        auto validate_extrusion_width = [min_nozzle_diameter, max_nozzle_diameter](const ConfigBase &config, const char *opt_key, double layer_height, std::string &err_msg) -> bool {
            double extrusion_width_min = config.get_abs_value(opt_key, min_nozzle_diameter);
            double extrusion_width_max = config.get_abs_value(opt_key, max_nozzle_diameter);
            if (extrusion_width_min == 0) {
                // Default "auto-generated" extrusion width is always valid.
            } else if (extrusion_width_min <= layer_height) {
                    err_msg = L("Line width too small");
                    return false;
                } else if (extrusion_width_max > max_nozzle_diameter * MAX_LINE_WIDTH_MULTIPLIER) {
                err_msg = L("Line width too large");
				return false;
			}
			return true;
		};
        for (PrintObject *object : m_objects) {
            if (object->has_support_material()) {
                // BBS: remove useless logics and L()
#if 0
				if ((object->config().support_filament == 0 || object->config().support_interface_filament == 0) && max_nozzle_diameter - min_nozzle_diameter > EPSILON) {
                    // The object has some form of support and either support_filament or support_interface_filament
                    // will be printed with the current tool without a forced tool change. Play safe, assert that all object nozzles
                    // are of the same diameter.
                    return {L("Printing with multiple extruders of differing nozzle diameters. "
                           "If support is to be printed with the current filament (support_filament == 0 or support_interface_filament == 0), "
                           "all nozzles have to be of the same diameter."), object, "support_filament"};
                }
#endif

                // BBS
#if 0
                if (this->has_wipe_tower() && object->config().independent_support_layer_height) {
                    return {L("A prime tower requires that support has the same layer height as the object."), object, "support_filament"};
                }
#endif

                // Prusa: Fixing crashes with invalid tip diameter or branch diameter
                // https://github.com/prusa3d/PrusaSlicer/commit/96b3ae85013ac363cd1c3e98ec6b7938aeacf46d
                if (is_tree(object->config().support_type.value)) {
                    if (object->config().support_style == smsTreeOrganic ||
                        // Orca: use organic as default
                        object->config().support_style == smsDefault) {

                        // Orca: check the support wall count and the base pattern
                        if (object->config().tree_support_wall_count > 1 &&
                            object->config().support_base_pattern != SupportMaterialPattern::smpNone &&
                            object->config().support_base_pattern != SupportMaterialPattern::smpDefault)
                            warn(L("For Organic supports, two walls are supported only with the Hollow/Default base pattern."), "support_base_pattern");

                        // Orca: check if the Lightning base pattern selected
                        if (object->config().support_base_pattern == SupportMaterialPattern::smpLightning)
                            warn(L("The Lightning base pattern is not supported by this support type; Rectilinear will be used instead."), "support_base_pattern");

                        float extrusion_width = std::min(
                            support_material_flow(object).width(),
                            support_material_interface_flow(object).width());
                        if (object->config().tree_support_tip_diameter < extrusion_width - EPSILON)
                            return { L("Organic support tree tip diameter must not be smaller than support material extrusion width."), object, "tree_support_tip_diameter" };
                        if (object->config().tree_support_branch_diameter_organic < 2. * extrusion_width - EPSILON)
                            return { L("Organic support branch diameter must not be smaller than 2x support material extrusion width."), object, "tree_support_branch_diameter_organic" };
                        if (object->config().tree_support_branch_diameter_organic < object->config().tree_support_tip_diameter)
                            return { L("Organic support branch diameter must not be smaller than support tree tip diameter."), object, "tree_support_branch_diameter_organic" };
                    }
                } else if (object->config().support_base_pattern == SupportMaterialPattern::smpLightning) {
                    // Orca: check if the Lightning base pattern selected
                    warn(L("The Lightning base pattern is not supported by this support type; Rectilinear will be used instead."), "support_base_pattern");
                } else if (object->config().support_base_pattern == SupportMaterialPattern::smpNone) {
                    // Orca: check if the Hollow base pattern selected
                    warn(L("The Hollow base pattern is not supported by this support type; Rectilinear will be used instead."), "support_base_pattern");
                }
            }

            // Do we have custom support data that would not be used?
            // Notify the user in that case.
            if (! object->has_support()) {
                for (const ModelVolume* mv : object->model_object()->volumes) {
                    bool has_enforcers = mv->is_support_enforcer() ||
                        (mv->is_model_part() && mv->supported_facets.has_facets(*mv, EnforcerBlockerType::ENFORCER));
                    if (has_enforcers) {
                        warn(L("Support enforcers are used but support is not enabled. Please enable support."), "", object);
                        break;
                    }
                }
            }

            double initial_layer_print_height = m_config.initial_layer_print_height.value;
            double first_layer_min_nozzle_diameter;
            if (object->has_raft()) {
                // if we have raft layers, only support material extruder is used on first layer
                size_t first_layer_extruder = object->config().raft_layers == 1
                    ? object->config().support_interface_filament-1
                    : object->config().support_filament-1;
                first_layer_min_nozzle_diameter = (first_layer_extruder == size_t(-1)) ?
                    min_nozzle_diameter :
                    m_config.nozzle_diameter.get_at(first_layer_extruder);
            } else {
                // if we don't have raft layers, any nozzle diameter is potentially used in first layer
                first_layer_min_nozzle_diameter = min_nozzle_diameter;
            }
            if (initial_layer_print_height > first_layer_min_nozzle_diameter)
                return {L("Layer height cannot exceed nozzle diameter."), object, "initial_layer_print_height"};

            // validate layer_height
            double layer_height = object->config().layer_height.value;
            if (layer_height > min_nozzle_diameter)
                return {L("Layer height cannot exceed nozzle diameter."), object, "layer_height"};

            // Validate extrusion widths.
            std::string err_msg;
            if (!validate_extrusion_width(object->config(), "line_width", layer_height, err_msg))
            	return {err_msg, object, "line_width"};
            if (object->has_support() || object->has_raft()) {
                if (!validate_extrusion_width(object->config(), "support_line_width", layer_height, err_msg))
                    return {err_msg, object, "support_line_width"};
            }
            for (const char *opt_key : { "inner_wall_line_width", "outer_wall_line_width", "sparse_infill_line_width", "internal_solid_infill_line_width", "top_surface_line_width","skin_infill_line_width" ,"skeleton_infill_line_width"})
				for (const PrintRegion &region : object->all_regions())
                    if (!validate_extrusion_width(region.config(), opt_key, layer_height, err_msg))
		            	return  {err_msg, object, opt_key};

            const bool allow_thin_bridge_width = object->config().thick_bridges && object->config().thick_internal_bridges;
            for (const PrintRegion &region : object->all_regions()) {
                const auto &bridge_width_opt = region.config().bridge_line_width;
                // The outer wall is included: under Feature Split its bridge road is laid by the fine nozzle
                // whatever the coarse roles use. A percentage width is a share of the nozzle drawing the road
                // and cannot trip this; an absolute width has to fit the outer wall's tool too.
                for (FlowRole bridge_role : { frPerimeter, frExternalPerimeter, frInfill, frSolidInfill, frTopSolidInfill }) {
                    // The road is flowed on the physical nozzle the filament map sends the role's filament
                    // to, so the limit is that nozzle's. It matches the logical column in Off mode and for an
                    // identity map. A filament an active mode cannot resolve is reported by that mode's
                    // admission checks; here it keeps the plain column.
                    const std::optional<double> resolved_nozzle =
                        mixed_nozzle_flow_nozzle_diameter(m_config, size_t(region.extruder(bridge_role) - 1));
                    const double nozzle_diameter = resolved_nozzle ? *resolved_nozzle :
                        m_config.nozzle_diameter.get_at(region.extruder(bridge_role) - 1);
                    const double bridge_width    = bridge_width_opt.get_abs_value(nozzle_diameter);
                    if (bridge_width <= 0.)
                        continue;
                    if (bridge_width > nozzle_diameter) {
                        err_msg = L("Bridge line width must not exceed nozzle diameter");
                        return { err_msg, object, "bridge_line_width" };
                    }
                    if (!allow_thin_bridge_width && bridge_width <= layer_height) {
                        err_msg = L("Line width too small");
                        return { err_msg, object, "bridge_line_width" };
                    }
                }
            }
        }
    }

    // Orca: G92 E0 is not supported when using absolute extruder addressing
    // This check is modified from PrusaSlicer, the original author is Vojtech Bubnik
    // Orca: case‑sensitive match for exactly "G92 E0" (uppercase G and E only) 
    // because gcode is case sensitive and G92 e0 satisfies the regex but causes a slicing error
    // https://github.com/OrcaSlicer/OrcaSlicer/issues/13927

	    // Matches any case of "G92 E0" (original pattern)
    static const boost::regex regex_g92e0 {
        "^[ \\t]*[gG]92[ \\t]*[eE](0(\\.0*)?|\\.0+)[ \\t]*(;.*)?$"
    };
    // Matches only the exact uppercase "G92 E0"
    static const boost::regex regex_g92e0_correct {
        "^[ \\t]*G92[ \\t]*E(0(\\.0*)?|\\.0+)[ \\t]*(;.*)?$"
    };

    const bool before_has_g92_any = boost::regex_search(
        m_config.before_layer_change_gcode.value, regex_g92e0);
    const bool layer_has_g92_any  = boost::regex_search(
        m_config.layer_change_gcode.value, regex_g92e0);

    if (m_config.use_relative_e_distances) {
        // Relative mode: "G92 E0" is required to reset extruder position.
        const bool before_has_g92_exact = boost::regex_search(
            m_config.before_layer_change_gcode.value, regex_g92e0_correct);
        const bool layer_has_g92_exact  = boost::regex_search(
            m_config.layer_change_gcode.value, regex_g92e0_correct);

        // Wrong case found?
        if (before_has_g92_any && !before_has_g92_exact)
            return {L("\"G92 E0\" was found in before_layer_change_gcode, but the G or E are not uppercase. "
                      "Please change them to the exact uppercase \"G92 E0\"."),
                    nullptr, "before_layer_change_gcode"};
        if (layer_has_g92_any && !layer_has_g92_exact)
            return {L("\"G92 E0\" was found in layer_change_gcode, but the G or E are not uppercase. "
                      "Please change them to the exact uppercase \"G92 E0\"."),
                    nullptr, "layer_change_gcode"};

        // Only Marlin flavours need the reset; BBL printers do not.
        if ((m_config.gcode_flavor == gcfMarlinLegacy || m_config.gcode_flavor == gcfMarlinFirmware) &&
            !is_BBL_printer() &&
            !before_has_g92_exact && !layer_has_g92_exact)
            return {L("Relative extruder addressing requires resetting the extruder position at each layer to "
                      "prevent loss of floating point accuracy. Add \"G92 E0\" to layer_gcode."),
                    nullptr, "before_layer_change_gcode"};
    } else {
        // Absolute mode: any occurrence of "G92 E0" is incompatible.
        if (before_has_g92_any)
            return {L("\"G92 E0\" was found in before_layer_change_gcode, which is incompatible with absolute extruder "
                      "addressing."),
                    nullptr, "before_layer_change_gcode"};
        if (layer_has_g92_any)
            return {L("\"G92 E0\" was found in layer_change_gcode, which is incompatible with absolute extruder "
                      "addressing."),
                    nullptr, "layer_change_gcode"};
	}

    const ConfigOptionDef* bed_type_def = print_config_def.get("curr_bed_type");
    assert(bed_type_def != nullptr);

    // ORCA: check if bed type is compatible with all selected filaments
    if (is_BBL_printer() || m_config.support_multi_bed_types.value) {
	    const t_config_enum_values* bed_type_keys_map = bed_type_def->enum_keys_map;
	    for (unsigned int extruder_id : extruders) {
	        const ConfigOptionInts* bed_temp_opt = m_config.option<ConfigOptionInts>(get_bed_temp_key(m_config.curr_bed_type));
	        for (unsigned int extruder_id : extruders) {
	            int curr_bed_temp = bed_temp_opt->get_at(extruder_id);
	            if (curr_bed_temp == 0 && bed_type_keys_map != nullptr) {
	                std::string bed_type_name;
	                for (auto item : *bed_type_keys_map) {
	                    if (item.second == m_config.curr_bed_type) {
	                        bed_type_name = item.first;
	                        break;
	                    }
	                }

	                StringObjectException except;
	                except.string = Slic3r::format(L("Plate %d: %s does not support filament %s"), this->get_plate_index() + 1, L(bed_type_name), extruder_id + 1);
	                except.string += "\n";
	                except.type   = STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE;
	                except.params.push_back(std::to_string(this->get_plate_index() + 1));
	                except.params.push_back(L(bed_type_name));
	                except.params.push_back(std::to_string(extruder_id+1));
	                except.object = nullptr;
	                return except;
	           }
            }
        }
    }

    // check if print speed/accel/jerk is higher than the maximum speed of the printer
    if (warnings) {
        // The motion-ability checks are mutually exclusive (gated on warning_key), so collect the
        // single one that fires into a local and push it once - separate from the precise-wall and
        // shrinkage warnings below.
        StringObjectException motion_warning;
        try {
            auto check_extruder = [&](const int extruder_id) {
                auto check_motion_ability_object_setting = [&](const std::vector<std::string>& keys_to_check, double limit) -> std::string {
                    std::string warning_key;
                    for (const auto& key : keys_to_check) {
                        if (m_default_object_config.get_abs_value_at(key, extruder_id) > limit) {
                            warning_key = key;
                            break;
                        }
                    }
                    return warning_key;
                };
                auto check_motion_ability_region_setting = [&](const std::vector<std::string>& keys_to_check, double limit) -> std::string {
                    std::string warning_key;
                    for (const auto& key : keys_to_check) {
                        if (m_default_region_config.get_abs_value_at(key, extruder_id) > limit) {
                            warning_key = key;
                            break;
                        }
                    }
                    return warning_key;
                };
                std::string warning_key;

                const auto max_junction_deviation = m_config.machine_max_junction_deviation.values[0]; // TODO: fix this
                const bool ignore_jerk_validation = m_config.gcode_flavor == gcfMarlinFirmware && max_junction_deviation > 0;

                // check jerk
                if (!ignore_jerk_validation) {
                    if (m_default_object_config.default_jerk.get_at(extruder_id) == 1 || m_default_object_config.outer_wall_jerk.get_at(extruder_id) == 1 ||
                        m_default_object_config.inner_wall_jerk.get_at(extruder_id) == 1) {
                       motion_warning.string = L("Setting the jerk speed too low could lead to artifacts on curved surfaces");
                       if (m_default_object_config.outer_wall_jerk.get_at(extruder_id) == 1)
                            warning_key = "outer_wall_jerk";
                       else if (m_default_object_config.inner_wall_jerk.get_at(extruder_id) == 1)
                            warning_key = "inner_wall_jerk";
                       else
                            warning_key = "default_jerk";

                       motion_warning.opt_key = warning_key;
                    }

                    if (warning_key.empty() && m_default_object_config.default_jerk.get_at(extruder_id) > 0) {
                       std::vector<std::string> jerk_to_check = {"default_jerk",     "outer_wall_jerk",    "inner_wall_jerk", "infill_jerk",
                                                                 "top_surface_jerk", "initial_layer_jerk", "travel_jerk"};
                       const auto               max_jerk = std::min(m_config.machine_max_jerk_x.values[0], m_config.machine_max_jerk_y.values[0]);
                       warning_key.clear();
                       warning_key = check_motion_ability_object_setting(jerk_to_check, max_jerk);
                       if (!warning_key.empty()) {
                            motion_warning.string = L(
                                "The jerk setting exceeds the printer's maximum jerk (machine_max_jerk_x/machine_max_jerk_y).\n"
                                "Orca will automatically cap the jerk speed to ensure it doesn't surpass the printer's capabilities.\n"
                                "You can adjust the maximum jerk setting in your printer's configuration to get higher speeds.");
                            motion_warning.opt_key = warning_key;
                       }
                    }
                }

                // Check junction deviation
                // Orca: Only marlin FW supports max junction deviation. Dont display warning if firmware is not supporting it.
                const bool support_max_junction_deviation = ( m_config.gcode_flavor == gcfMarlinFirmware);
                if (warning_key.empty() && m_default_object_config.default_junction_deviation.get_at(extruder_id) > max_junction_deviation && support_max_junction_deviation) {
                    motion_warning.string  = L( "Junction deviation setting exceeds the printer's maximum value (machine_max_junction_deviation).\n"
                                          "Orca will automatically cap the junction deviation to ensure it doesn't surpass the printer's capabilities.\n"
                                          "You can adjust the machine_max_junction_deviation value in your printer's configuration to get higher limits.");
                    motion_warning.opt_key = "default_junction_deviation";
                }
                
                // check acceleration
                const auto max_accel = m_config.machine_max_acceleration_extruding.values[0];
                if (warning_key.empty() && m_default_object_config.default_acceleration.get_at(extruder_id) > 0 && max_accel > 0) {
                   const bool support_travel_acc = (m_config.gcode_flavor == gcfRepetier || m_config.gcode_flavor == gcfMarlinFirmware ||
                                                    m_config.gcode_flavor == gcfRepRapFirmware);

                   std::vector<std::string> accel_to_check;
                   if (!support_travel_acc)
                        accel_to_check = {
                            "default_acceleration",
                            "inner_wall_acceleration",
                            "outer_wall_acceleration",
                            "bridge_acceleration",
                            "initial_layer_acceleration",
                            "sparse_infill_acceleration",
                            "internal_solid_infill_acceleration",
                            "top_surface_acceleration",
                            "travel_acceleration",
                        };
                   else
                        accel_to_check = {
                            "default_acceleration",
                            "inner_wall_acceleration",
                            "outer_wall_acceleration",
                            "bridge_acceleration",
                            "initial_layer_acceleration",
                            "sparse_infill_acceleration",
                            "internal_solid_infill_acceleration",
                            "top_surface_acceleration",
                        };
                   warning_key = check_motion_ability_object_setting(accel_to_check, max_accel);
                   if (!warning_key.empty()) {
                        motion_warning.string  = L("The acceleration setting exceeds the printer's maximum acceleration "
                                              "(machine_max_acceleration_extruding).\nOrca will "
                                              "automatically cap the acceleration speed to ensure it doesn't surpass the printer's "
                                              "capabilities.\nYou can adjust the "
                                              "machine_max_acceleration_extruding value in your printer's configuration to get higher speeds.");
                        motion_warning.opt_key = warning_key;
                   }
                   if (support_travel_acc) {
                        const auto max_travel = m_config.machine_max_acceleration_travel.values[0];
                        if (max_travel > 0) {
                            accel_to_check = {
                                "travel_acceleration",
                            };
                            warning_key = check_motion_ability_object_setting(accel_to_check, max_travel);
                            if (!warning_key.empty()) {
                                motion_warning.string = L(
                                    "The travel acceleration setting exceeds the printer's maximum travel acceleration "
                                    "(machine_max_acceleration_travel).\nOrca will "
                                    "automatically cap the travel acceleration speed to ensure it doesn't surpass the printer's "
                                    "capabilities.\nYou can adjust the "
                                    "machine_max_acceleration_travel value in your printer's configuration to get higher speeds.");
                                motion_warning.opt_key = warning_key;
                            }
                        }
                   }
                }

                // check speed
                // Orca: disable the speed check for now as we don't cap the speed
                // if (warning_key.empty()) {
                //    auto       speed_to_check = {"inner_wall_speed",  "outer_wall_speed", "sparse_infill_speed",   "internal_solid_infill_speed",
                //                                 "top_surface_speed", "bridge_speed",     "internal_bridge_speed", "gap_infill_speed"};
                //    const auto max_speed      = std::min(m_config.machine_max_speed_x.values[0], m_config.machine_max_speed_y.values[0]);
                //    warning_key.clear();
                //    warning_key = check_motion_ability_region_setting(speed_to_check, max_speed);
                //    if (warning_key.empty() && m_config.travel_speed > max_speed)
                //         warning_key = "travel_speed";
                //    if (!warning_key.empty()) {
                //         motion_warning.string = L(
                //             "The speed setting exceeds the printer's maximum speed (machine_max_speed_x/machine_max_speed_y).\nOrca will "
                //             "automatically cap the print speed to ensure it doesn't surpass the printer's capabilities.\nYou can adjust the "
                //             "maximum speed setting in your printer's configuration to get higher speeds.");
                //         motion_warning.opt_key = warning_key;
                //    }
                // }
            };
            check_extruder(0); // TODO: check used extruder variants

            // check wall sequence and precise outer wall
            if (m_default_region_config.precise_outer_wall && m_default_region_config.wall_sequence != WallSequence::InnerOuter)
                warn(L("The precise wall option will be ignored for outer-inner or inner-outer-inner wall sequences."), "precise_outer_wall");

            // check adaptive pressure advance model
            for (unsigned int extruder_id : extruders) {
                if (m_config.adaptive_pressure_advance.get_at(extruder_id) && 
                    m_config.enable_pressure_advance.get_at(extruder_id)) {
                    
                    const std::string pa_model = m_config.adaptive_pressure_advance_model.get_at(extruder_id);
                    if (!pa_model.empty()) {
                        std::string validation_error = AdaptivePAProcessor::validate_adaptive_pa_model(pa_model);
                        if (!validation_error.empty()) {
                            warn(L("The Adaptive Pressure Advance model for one or more extruders may contain invalid values."),
                                 "adaptive_pressure_advance_model");
                            break;
                        }
                    }
                }
            }

        } catch (std::exception& e) {
            BOOST_LOG_TRIVIAL(warning) << "Orca: validate motion ability failed: " << e.what() << std::endl;
        }
        if (!motion_warning.string.empty())
            add_warning(motion_warning);
    }
    if (!this->has_same_shrinkage_compensations())
        warn(L("Filament shrinkage will not be used because filament shrinkage for the used filaments does not match."));
    return {};
}

#if 0
// the bounding box of objects placed in copies position
// (without taking skirt/brim/support material into account)
BoundingBox Print::bounding_box() const
{
    BoundingBox bb;
    for (const PrintObject *object : m_objects)
        for (const PrintInstance &instance : object->instances()) {
        	BoundingBox bb2(object->bounding_box());
        	bb.merge(bb2.min + instance.shift);
        	bb.merge(bb2.max + instance.shift);
        }
    return bb;
}

// the total bounding box of extrusions, including skirt/brim/support material
// this methods needs to be called even when no steps were processed, so it should
// only use configuration values
BoundingBox Print::total_bounding_box() const
{
    // get objects bounding box
    BoundingBox bb = this->bounding_box();

    // we need to offset the objects bounding box by at least half the perimeters extrusion width
    Flow perimeter_flow = m_objects.front()->get_layer(0)->get_region(0)->flow(frPerimeter);
    double extra = perimeter_flow.width/2;

    // consider support material
    if (this->has_support_material()) {
        extra = std::max(extra, SUPPORT_MATERIAL_MARGIN);
    }

    // consider brim and skirt
    if (m_config.brim_width.value > 0) {
        Flow brim_flow = this->brim_flow();
        extra = std::max(extra, m_config.brim_width.value + brim_flow.width/2);
    }
    if (this->has_skirt()) {
        int skirts = m_config.skirt_loops.value;
        if (skirts == 0 && this->has_infinite_skirt()) skirts = 1;
        Flow skirt_flow = this->skirt_flow();
        extra = std::max(
            extra,
            m_config.brim_width.value
                + m_config.skirt_distance.value
                + skirts * skirt_flow.spacing()
                + skirt_flow.width/2
        );
    }

    if (extra > 0)
        bb.offset(scale_(extra));

    return bb;
}
#endif

double Print::skirt_first_layer_height() const
{
    // A raft the coarse nozzle starts on the bed raises the first layer (mixed_nozzle_raft_first_layer_height()).
    double height = m_config.initial_layer_print_height.value;
    for (const PrintObject *object : m_objects)
        height = std::max(height, mixed_nozzle_raft_first_layer_height(m_config, object->config()));
    return height;
}

// Feature Split owner of brim and skirt: the first-layer outer-wall (fine shell) tool. nullopt
// outside Feature Split, with no print regions, with outer_wall_filament_id unset, or when its
// filament does not resolve to a physical tool.
std::optional<MixedNozzleResolvedTool> Print::feature_split_adhesion_tool() const
{
    if (! is_mixed_nozzle_feature_split(m_config) || m_print_regions.empty())
        return std::nullopt;
    const int outer_wall_filament_id = m_print_regions.front()->config().outer_wall_filament_id.value;
    if (outer_wall_filament_id <= 0)
        return std::nullopt;
    const MixedNozzleToolResolution resolution = resolve_mixed_nozzle_tool(
        m_config, size_t(outer_wall_filament_id - 1), MixedNozzleResolveScope::PhysicalToolOnly);
    return resolution.tool;
}

Flow Print::brim_flow() const
{
    ConfigOptionFloatOrPercent width = m_config.initial_layer_line_width;
    if (width.value <= 0)
        width = m_print_regions.front()->config().inner_wall_line_width;
    if (width.value <= 0)
        width = m_objects.front()->config().line_width;

    /* We currently use a random region's perimeter extruder.
       While this works for most cases, we should probably consider all of the perimeter
       extruders and take the one with, say, the smallest index.
       The same logic should be applied to the code that selects the extruder during G-code
       generation as well. */
    float nozzle = (float)m_config.nozzle_diameter.get_at(m_print_regions.front()->config().outer_wall_filament_id-1);
    // Under Feature Split the brim belongs to the first-layer outer-wall tool, which may differ from
    // the logical-index lookup above (for example with a swapped map).
    if (const std::optional<MixedNozzleResolvedTool> tool = this->feature_split_adhesion_tool())
        nozzle = float(tool->nozzle_diameter);

    return Flow::new_from_config_width(
        frPerimeter,
        // Flow::new_from_config_width takes care of the percent to value substitution
		width,
        nozzle,
		(float)this->skirt_first_layer_height());
}

Flow Print::skirt_flow() const
{

    // Orca: fall back to m_config if no objects are present
    ConfigOptionFloatOrPercent width = m_config.initial_layer_line_width;
    if (width.value <= 0)
        width = m_objects.empty() ? m_config.initial_layer_line_width : m_objects.front()->config().line_width;

    /* We currently use a random object's support material extruder.
       While this works for most cases, we should probably consider all of the support material
       extruders and take the one with, say, the smallest index;
       The same logic should be applied to the code that selects the extruder during G-code
       generation as well. */
    float nozzle = (float) m_config.nozzle_diameter.get_at(
        m_objects.empty() ? 0 : m_objects.front()->config().support_filament - 1);
    // Under Feature Split the skirt belongs to the first-layer outer-wall tool, not the support
    // filament's tool.
    if (const std::optional<MixedNozzleResolvedTool> tool = this->feature_split_adhesion_tool()) {
        nozzle = float(tool->nozzle_diameter);
        // With every object on a raft, layer 1 is the raft's and the skirt goes with the filament that lays the
        // raft's bed layer (GCode.cpp takes the first filament of the layer when the outer-wall tool is not on
        // it). When that filament is on another nozzle the road is that nozzle's first-layer road.
        if (std::all_of(m_objects.begin(), m_objects.end(), [](const PrintObject *object) { return object->has_raft(); })) {
            const PrintObject *object   = m_objects.front();
            const int          filament = object->config().raft_layers.value == 1 ? object->config().support_interface_filament.value :
                                                                                    object->config().support_filament.value;
            const int          raft_nozzle = resolved_support_filament_nozzle_idx(m_config, filament);
            if (filament > 0 && raft_nozzle > 0 && size_t(raft_nozzle) <= m_config.nozzle_diameter.values.size() &&
                std::abs(m_config.nozzle_diameter.get_at(size_t(raft_nozzle - 1)) - tool->nozzle_diameter) > EPSILON) {
                const Flow raft = support_material_1st_layer_flow(object, float(this->skirt_first_layer_height()), filament);
                return Flow(raft.width(), raft.height(), raft.nozzle_diameter());
            }
        }
    }

    return Flow::new_from_config_width(frPerimeter,
                                       // Flow::new_from_config_width takes care of the percent to value substitution
                                       width,
                                       nozzle,
                                       (float) this->skirt_first_layer_height());
}

bool Print::has_support_material() const
{
    for (const PrintObject *object : m_objects)
        if (object->has_support_material())
            return true;
    return false;
}

/*  This method assigns extruders to the volumes having a material
    but not having extruders set in the volume config. */
void Print::auto_assign_extruders(ModelObject* model_object) const
{
    // only assign extruders if object has more than one volume
    if (model_object->volumes.size() < 2)
        return;

//    size_t extruders = m_config.nozzle_diameter.values.size();
    for (size_t volume_id = 0; volume_id < model_object->volumes.size(); ++ volume_id) {
        ModelVolume *volume = model_object->volumes[volume_id];
        //FIXME Vojtech: This assigns an extruder ID even to a modifier volume, if it has a material assigned.
        if ((volume->is_model_part() || volume->is_modifier()) && ! volume->material_id().empty() && ! volume->config.has("extruder"))
            volume->config.set("extruder", int(volume_id + 1));
    }
}

void  PrintObject::set_shared_object(PrintObject *object)
{
    m_shared_object = object;
    BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << boost::format(": this=%1%, found shared object from %2%")%this%m_shared_object;
}

void  PrintObject::clear_shared_object()
{
    if (m_shared_object) {
        BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << boost::format(": this=%1%, clear previous shared object data %2%")%this %m_shared_object;
        clear_regional_grid_state();
        m_layers.clear();
        m_support_layers.clear();

        m_shared_object = nullptr;

        invalidate_all_steps_without_cancel();
    }
}

void  PrintObject::copy_layers_from_shared_object()
{
    if (m_shared_object) {
        clear_regional_grid_state();
        m_layers.clear();
        m_support_layers.clear();

        firstLayerObjSliceByVolume.clear();
        firstLayerObjSliceByGroups.clear();

        BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << boost::format(": this=%1%, copied layers from object %2%")%this%m_shared_object;
        m_layers = m_shared_object->layers();
        m_support_layers = m_shared_object->support_layers();

        // Native planning state and physical bindings are value sidecars. Rebuild the
        // non-owning grid views through this object's shared event-layer vector instead
        // of copying pointers from the source object's side table.
        m_region_slicing = m_shared_object->m_region_slicing;
        m_rendezvous = m_shared_object->m_rendezvous;
        m_native_regional_grid_state = m_shared_object->m_native_regional_grid_state;
        m_region_grid_physical_extruders = m_shared_object->m_region_grid_physical_extruders;
        if (m_native_regional_grid_state.cells.size() == num_printing_regions()) {
            const coordf_t object_z_offset = slicing_parameters().object_print_z_min;
            m_region_grids.assign(num_printing_regions(), {});
            bool complete = true;
            for (size_t region = 0; region < m_region_grids.size() && complete; ++region) {
                auto &grid = m_region_grids[region];
                grid.reserve(m_native_regional_grid_state.cells[region].size());
                for (const NativeRegionCellState &cell : m_native_regional_grid_state.cells[region]) {
                    const auto event = std::lower_bound(m_layers.begin(), m_layers.end(), cell.cell.top_z + object_z_offset - 1e-8,
                        [](const Layer *layer, coordf_t z) { return layer->print_z < z; });
                    if (event == m_layers.end() || std::abs((*event)->print_z - cell.cell.top_z - object_z_offset) > 1e-7) {
                        complete = false;
                        break;
                    }
                    LayerRegion *host = (*event)->get_region(int(region));
                    if (host == nullptr || !host->has_cell() || host->cell_index() != cell.cell.cell_index ||
                        std::abs(host->bottom_z() - cell.cell.bottom_z - object_z_offset) > 1e-7 ||
                        std::abs(host->height() - cell.cell.height) > 1e-7 ||
                        std::abs(host->slice_z() - cell.cell.slice_z) > 1e-7) {
                        complete = false;
                        break;
                    }
                    grid.push_back(host);
                }
            }
            if (!complete)
                m_region_grids.clear();
        }
        m_pre_interlocking_slices = m_shared_object->m_pre_interlocking_slices;
        m_regional_volume_overlap_layer = m_shared_object->m_regional_volume_overlap_layer;
        m_regional_volume_overlap_mm2 = m_shared_object->m_regional_volume_overlap_mm2;

        firstLayerObjSliceByVolume = m_shared_object->firstLayerObjSlice();
        firstLayerObjSliceByGroups = m_shared_object->firstLayerObjGroups();
    }
}

void  PrintObject::copy_layers_overhang_from_shared_object()
{
    if (m_shared_object) {
        for (size_t index = 0; index <  m_layers.size() && index <  m_shared_object->m_layers.size(); index++)
        {
            Layer* layer_src = m_layers[index];
            layer_src->loverhangs = m_shared_object->m_layers[index]->loverhangs;
            layer_src->loverhangs_bbox = m_shared_object->m_layers[index]->loverhangs_bbox;
        }
        BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << boost::format(": this=%1%, copied layer overhang from object %2%")%this%m_shared_object;
    }
}


// BBS
BoundingBox PrintObject::get_first_layer_bbox(float& a, float& layer_height, std::string& name)
{
    BoundingBox bbox;
    a = 0;
    name = this->model_object()->name;
    if (layer_count() > 0) {
        auto layer = get_layer(0);
        layer_height = layer->height;
        // only work for object with single instance
        auto shift = instances()[0].shift_without_plate_offset();
        for (auto bb : layer->lslices_bboxes)
        {
            bb.translate(shift.x(), shift.y());
            bbox.merge(bb);
        }
        for (auto slice : layer->lslices) {
            a += area(slice);
        }
    }
    if (has_brim())
        bbox = firstLayerObjectBrimBoundingBox;
    return bbox;
}

// BBS: map print object with its first layer's first extruder
std::map<ObjectID, unsigned int> getObjectExtruderMap(const Print& print) {
    std::map<ObjectID, unsigned int> objectExtruderMap;
    for (const PrintObject* object : print.objects()) {
        // BBS
        if (object->object_first_layer_wall_extruders.empty()){
            unsigned int objectFirstLayerFirstExtruder = print.config().filament_diameter.size();
            auto firstLayerRegions = object->layers().front()->regions();
            if (!firstLayerRegions.empty()) {
                for (const LayerRegion* regionPtr : firstLayerRegions) {
                    if (regionPtr->has_extrusions())
                        objectFirstLayerFirstExtruder = std::min(objectFirstLayerFirstExtruder,
                          regionPtr->region().extruder(frExternalPerimeter));
                }
            }
            objectExtruderMap.insert(std::make_pair(object->id(), objectFirstLayerFirstExtruder));
        }
        else {
            objectExtruderMap.insert(std::make_pair(object->id(), object->object_first_layer_wall_extruders.front()));
        }
    }
    return objectExtruderMap;
}

// Slicing process, running at a background thread.
void Print::process(long long *time_cost_with_cache, bool use_cache)
{
    long long start_time = 0, end_time = 0;
    if (time_cost_with_cache)
        *time_cost_with_cache = 0;

    {
        const auto* sp = this->config().option<ConfigOptionStrings>("slicing_pipeline_plugin");
        m_pipeline_plugin_active = s_slicing_pipeline_hook_fn && sp && !sp->values.empty();
    }

    name_tbb_thread_pool_threads_set_locale();

    //compute the PrintObject with the same geometries
    BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << boost::format(": this=%1%, enter, use_cache=%2%, object size=%3%")%this%use_cache%m_objects.size();
    if (m_objects.empty())
        return;

    const auto throw_on_regional_binding_mismatch = [this](bool require_committed_grid) {
        if (!is_mixed_nozzle_body_split(m_config))
            return;
        // A plain object on a Body Split plate has no grid to commit, so only Body Split objects are
        // asked.
        if (require_committed_grid && !std::all_of(m_objects.begin(), m_objects.end(), [this](const PrintObject *object) {
                return ! is_body_split_object(m_config, *object->model_object()) ||
                       (object->region_grids().size() == object->num_printing_regions() &&
                        object->region_grid_physical_extruders().size() == object->num_printing_regions());
            }))
            return;
        if (const std::optional<RegionalBandToolMismatch> mismatch = this->find_regional_band_tool_mismatch(m_config))
            throw Slic3r::SlicingError(
                Slic3r::format("[SRL-C01] Body Split planned the coarse layer at layer %1% for nozzle %2%, but the "
                               "filament grouping now puts that body on nozzle %3%. The filament to nozzle assignment changed during slicing. Slice again.",
                               mismatch->first_layer_idx, mismatch->planned_physical_extruder, mismatch->resolved_physical_extruder),
                mismatch->object->model_object()->id().id);
    };

    // A repeated process pass may reach flow derivation before the end-of-process binding check.
    // Refuse a changed map here when native grids already exist, so a width-resolution error does
    // not mask the binding diagnostic.
    throw_on_regional_binding_mismatch(true);

    for (PrintObject *obj : m_objects)
        obj->clear_shared_object();

    //add the print_object share check logic
    auto is_print_object_the_same = [this](const PrintObject* object1, const PrintObject* object2) -> bool{
        if (object1->trafo().matrix() != object2->trafo().matrix())
            return false;
        const ModelObject* model_obj1 = object1->model_object();
        const ModelObject* model_obj2 = object2->model_object();
        if (model_obj1->volumes.size() != model_obj2->volumes.size())
            return false;
        bool has_extruder1 = model_obj1->config.has("extruder");
        bool has_extruder2 = model_obj2->config.has("extruder");
        if ((has_extruder1 != has_extruder2)
            || (has_extruder1 && model_obj1->config.extruder() != model_obj2->config.extruder()))
            return false;
        for (int index = 0; index < model_obj1->volumes.size(); index++) {
            const ModelVolume &model_volume1 = *model_obj1->volumes[index];
            const ModelVolume &model_volume2 = *model_obj2->volumes[index];
            if (model_volume1.type() != model_volume2.type())
                return false;
            if (model_volume1.mesh_ptr() != model_volume2.mesh_ptr())
                return false;
            if (!(model_volume1.get_transformation() == model_volume2.get_transformation()))
                return false;
            has_extruder1 = model_volume1.config.has("extruder");
            has_extruder2 = model_volume2.config.has("extruder");
            if ((has_extruder1 != has_extruder2)
                || (has_extruder1 && model_volume1.config.extruder() != model_volume2.config.extruder()))
                return false;
            if (!model_volume1.supported_facets.equals(model_volume2.supported_facets))
                return false;
            if (!model_volume1.seam_facets.equals(model_volume2.seam_facets))
                return false;
            if (!model_volume1.mmu_segmentation_facets.equals(model_volume2.mmu_segmentation_facets))
                return false;
            if (!model_volume1.fuzzy_skin_facets.equals(model_volume2.fuzzy_skin_facets))
                return false;
            if (model_volume1.config.get() != model_volume2.config.get())
                return false;
        }
        //if (!object1->config().equals(object2->config()))
        //    return false;
        if (model_obj1->layer_height_profile.get() != model_obj2->layer_height_profile.get())
            return false;
        if (model_obj1->config.get() != model_obj2->config.get())
            return false;
        return true;
    };
    int object_count = m_objects.size();
    std::set<PrintObject*> need_slicing_objects;
    std::set<PrintObject*> re_slicing_objects;
    if (!use_cache) {
        for (int index = 0; index < object_count; index++)
        {
            PrintObject *obj =  m_objects[index];
            for (PrintObject *slicing_obj : need_slicing_objects)
            {
                if (is_print_object_the_same(obj, slicing_obj)) {
                    obj->set_shared_object(slicing_obj);
                    break;
                }
            }
            if (!obj->get_shared_object())
                need_slicing_objects.insert(obj);
        }
    }
    else {
        for (int index = 0; index < object_count; index++)
        {
            PrintObject *obj =  m_objects[index];
            if (obj->layer_count() > 0)
                need_slicing_objects.insert(obj);
        }
        for (int index = 0; index < object_count; index++)
        {
            PrintObject *obj =  m_objects[index];
            bool found_shared = false;
            if (need_slicing_objects.find(obj) == need_slicing_objects.end()) {
                for (PrintObject *slicing_obj : need_slicing_objects)
                {
                    if (is_print_object_the_same(obj, slicing_obj)) {
                        obj->set_shared_object(slicing_obj);
                        found_shared = true;
                        break;
                    }
                }
                if (!found_shared) {
                    BOOST_LOG_TRIVIAL(warning) << boost::format("Also can not find the shared object, identify_id %1%, maybe shared object is skipped")%obj->model_object()->instances[0]->loaded_id;
                    //throw Slic3r::SlicingError("Cannot find the cached data.");
                    //don't report errot, set use_cache to false, and reslice these objects
                    need_slicing_objects.insert(obj);
                    re_slicing_objects.insert(obj);
                    //use_cache = false;
                }
            }
        }
    }

    BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << boost::format(": total object counts %1% in current print, need to slice %2%")%m_objects.size()%need_slicing_objects.size();
    BOOST_LOG_TRIVIAL(info) << "Starting the slicing process." << log_memory_info();
    if (!use_cache) {
        // Fire the SlicingPipeline hook for `obj` iff it just (re)computed `pstep` this pass.
        auto hook_after = [this](PrintObject* obj, bool was_done, PrintObjectStep pstep, SlicingPipelineStepPlugin sstep) {
            if (m_pipeline_plugin_active && !was_done && obj->is_step_done(pstep))
                run_pipeline_hook(sstep, obj);
        };

        // SlicingPipeline: dedicated slice loop so the Slice boundary is hookable before perimeters.
        for (PrintObject *obj : m_objects) {
            if (need_slicing_objects.count(obj) != 0) {
                const bool was_done = obj->is_step_done(posSlice);
                obj->slice();
                hook_after(obj, was_done, posSlice, SlicingPipelineStepPlugin::posSlice);
                // re-snapshot each layer's raw_slices AFTER the Slice hook ran, so the
                // plugin's mutation becomes the untyped baseline. Without this, a later
                // perimeter-only re-run (make_perimeters -> restore_untyped_slices) reverts
                // slices to the PRE-hook geometry while posSlice stays cached (the hook does
                // not re-fire), silently un-applying the mutation; raw_slices consumers
                // (sharp-tail support, ToolOrdering) also read this backup directly. Gated on
                // an active plugin AND a genuine (re)slice, so the inactive path is untouched
                // and re-backing-up an unmutated layer is a harmless identical copy.
                if (m_pipeline_plugin_active && !was_done && obj->is_step_done(posSlice))
                    for (Layer *layer : obj->layers())
                        layer->backup_untyped_slices();
            } else {
                if (obj->set_started(posSlice)) obj->set_done(posSlice);   // shared/duplicate — no hook
            }
        }
        for (PrintObject *obj : m_objects) {
            if (need_slicing_objects.count(obj) != 0) {
                const bool was_done = obj->is_step_done(posPerimeters);
                obj->make_perimeters();   // slice() inside is a no-op: posSlice already DONE
                hook_after(obj, was_done, posPerimeters, SlicingPipelineStepPlugin::posPerimeters);
            } else {
                if (obj->set_started(posPerimeters)) obj->set_done(posPerimeters);
            }
        }
        for (PrintObject *obj : m_objects) {
            if (need_slicing_objects.count(obj) != 0) {
                const bool was_done = obj->is_step_done(posEstimateCurledExtrusions);
                obj->estimate_curled_extrusions();
                hook_after(obj, was_done, posEstimateCurledExtrusions, SlicingPipelineStepPlugin::posEstimateCurledExtrusions);
            }
            else {
                if (obj->set_started(posEstimateCurledExtrusions))
                    obj->set_done(posEstimateCurledExtrusions);
            }
        }
        for (PrintObject *obj : m_objects) {
            if (need_slicing_objects.count(obj) != 0) {
                // split prepare_infill (fill-surface prep) from infill (make_fills) so a
                // plugin can mutate fill surfaces at the PrepareInfill seam and have make_fills
                // consume them (unlike the Infill seam, which fires after the fills are already
                // built). infill() re-invokes prepare_infill() as a no-op once posPrepareInfill
                // is DONE, so this is a mechanical split mirroring the slice/perimeters loop.
                const bool prepare_was_done = obj->is_step_done(posPrepareInfill);
                obj->prepare_infill();
                hook_after(obj, prepare_was_done, posPrepareInfill, SlicingPipelineStepPlugin::posPrepareInfill);
                const bool was_done = obj->is_step_done(posInfill);
                obj->infill();
                hook_after(obj, was_done, posInfill, SlicingPipelineStepPlugin::posInfill);
            }
            else {
                if (obj->set_started(posPrepareInfill))
                    obj->set_done(posPrepareInfill);
                if (obj->set_started(posInfill))
                    obj->set_done(posInfill);
            }
        }
        for (PrintObject *obj : m_objects) {
            if (need_slicing_objects.count(obj) != 0) {
                const bool was_done = obj->is_step_done(posIroning);
                obj->ironing();
                hook_after(obj, was_done, posIroning, SlicingPipelineStepPlugin::posIroning);
            }
            else {
                if (obj->set_started(posIroning))
                    obj->set_done(posIroning);
            }
        }

        // Z-Contouring
        for (PrintObject *obj : m_objects) {
            bool need_contouring = need_slicing_objects.count(obj) != 0 && obj->need_z_contouring();
            if (need_contouring) {
                const bool was_done = obj->is_step_done(posContouring);
                obj->contour_z();
                hook_after(obj, was_done, posContouring, SlicingPipelineStepPlugin::posContouring);
            } else {
                if (obj->set_started(posContouring))
                    obj->set_done(posContouring);
            }
        }

        // SlicingPipeline: support runs in the parallel block below; the hook must fire in a
        // sequential loop afterward. Snapshot per-object done-state just before the parallel_for.
        std::vector<char> sup_was_done(m_objects.size(), 1);
        if (m_pipeline_plugin_active)
            for (size_t i = 0; i < m_objects.size(); ++i)
                sup_was_done[i] = m_objects[i]->is_step_done(posSupportMaterial) ? 1 : 0;

        tbb::parallel_for(tbb::blocked_range<int>(0, int(m_objects.size())),
            [this, need_slicing_objects](const tbb::blocked_range<int>& range) {
                for (int i = range.begin(); i < range.end(); i++) {
                    PrintObject* obj = m_objects[i];
                    if (need_slicing_objects.count(obj) != 0) {
                        obj->generate_support_material();
                    }
                    else {
                        if (obj->set_started(posSupportMaterial))
                            obj->set_done(posSupportMaterial);
                    }
                }
            }
        );

        if (m_pipeline_plugin_active)
            for (size_t i = 0; i < m_objects.size(); ++i)
                if (need_slicing_objects.count(m_objects[i]) != 0 && !sup_was_done[i]
                    && m_objects[i]->is_step_done(posSupportMaterial))
                    run_pipeline_hook(SlicingPipelineStepPlugin::posSupportMaterial, m_objects[i]);

        for (PrintObject* obj : m_objects) {
            if (need_slicing_objects.count(obj) != 0) {
                const bool was_done = obj->is_step_done(posDetectOverhangsForLift);
                obj->detect_overhangs_for_lift();
                hook_after(obj, was_done, posDetectOverhangsForLift, SlicingPipelineStepPlugin::posDetectOverhangsForLift);
            }
            else {
                if (obj->set_started(posDetectOverhangsForLift))
                    obj->set_done(posDetectOverhangsForLift);
            }
        }
    }
    else {
        for (PrintObject *obj : m_objects) {
            if (re_slicing_objects.count(obj) == 0) {
                if (obj->set_started(posSlice))
                    obj->set_done(posSlice);
                if (obj->set_started(posPerimeters))
                    obj->set_done(posPerimeters);
                if (obj->set_started(posPrepareInfill))
                    obj->set_done(posPrepareInfill);
                if (obj->set_started(posInfill))
                    obj->set_done(posInfill);
                if (obj->set_started(posIroning))
                    obj->set_done(posIroning);
                if (obj->set_started(posContouring))
                    obj->set_done(posContouring);
                if (obj->set_started(posSupportMaterial))
                    obj->set_done(posSupportMaterial);
                if (obj->set_started(posDetectOverhangsForLift))
                    obj->set_done(posDetectOverhangsForLift);
            }
            else {
                obj->make_perimeters();
                obj->infill();
                obj->ironing();
                obj->generate_support_material();
                obj->detect_overhangs_for_lift();
                obj->estimate_curled_extrusions();
            }
        }
    }

    for (PrintObject *obj : m_objects)
    {
        if (need_slicing_objects.count(obj) == 0) {
            obj->copy_layers_from_shared_object();
            obj->copy_layers_overhang_from_shared_object();
        }
    }



    if (this->set_started(psWipeTower)) {
        {
            std::vector<std::set<int>> geometric_unprintables(m_config.nozzle_diameter.size());
            for (PrintObject* obj : m_objects) {
                std::vector<std::set<int>> obj_geometric_unprintables = obj->detect_extruder_geometric_unprintables();
                for (size_t idx = 0; idx < obj_geometric_unprintables.size(); ++idx) {
                    if (idx < geometric_unprintables.size()) {
                        geometric_unprintables[idx].insert(obj_geometric_unprintables[idx].begin(), obj_geometric_unprintables[idx].end());
                    }
                }
            }
            this->set_geometric_unprintable_filaments(geometric_unprintables);
        }

        m_wipe_tower_data.clear();
        m_tool_ordering.clear();
        // Only a generated tower fills it; a tower-enabled print with no tool change keeps it empty.
        m_fake_wipe_tower = FakeWipeTower();
        if (this->has_wipe_tower()) {
            this->_make_wipe_tower();
            this->select_feature_infill_by_time();
        }
        else if (this->config().print_sequence != PrintSequence::ByObject) {
            // Initialize the tool ordering, so it could be used by the G-code preview slider for planning tool changes and filament switches.
            m_tool_ordering = ToolOrdering(*this, -1, false);
            m_tool_ordering.sort_and_build_data(*this, -1, false);
            if (m_tool_ordering.empty() || m_tool_ordering.last_extruder() == unsigned(-1))
                throw Slic3r::SlicingError("The print is empty. The model is not printable with current print settings.");

        }
        this->set_done(psWipeTower);
        if (m_pipeline_plugin_active) run_pipeline_hook(SlicingPipelineStepPlugin::psWipeTower, nullptr);
    }

    if (this->has_wipe_tower()) {
        m_fake_wipe_tower.set_pos({ m_config.wipe_tower_x.get_at(m_plate_index), m_config.wipe_tower_y.get_at(m_plate_index) });
    }

    if (this->set_started(psSkirtBrim)) {
        this->set_status(70, L("Generating skirt & brim"));

        if (time_cost_with_cache)
            start_time = (long long)Slic3r::Utils::get_current_time_utc();

        m_skirt.clear();
        m_skirt_brim_groups.clear();
        m_has_shared_per_object_skirt = false;
        m_skirt_convex_hull.clear();
        m_objectBrimAreasByInstance.clear();
        m_first_layer_convex_hull.points.clear();
        for (PrintObject *object : m_objects)  object->m_skirt.clear();

        //BBS: get the objects' indices when GCodes are generated
        ToolOrdering tool_ordering;
        unsigned int initial_extruder_id = (unsigned int)-1;
        bool         has_wipe_tower = false;
        std::vector<const PrintInstance*> 					print_object_instances_ordering;
        std::vector<const PrintInstance*>::const_iterator 	print_object_instance_sequential_active;
        std::vector<std::pair<coordf_t, std::vector<GCode::LayerToPrint>>> layers_to_print = GCode::collect_layers_to_print(*this);
        std::vector<unsigned int> printExtruders;
        // Cleared on every process so a print-sequence or selector-mode change can never leave
        // stale object pointers behind; repopulated below only by the sequential selector path.
        m_sequential_dynamic_orderings.clear();
        if (this->config().print_sequence == PrintSequence::ByObject) {
            // Order object instances for sequential print.
            print_object_instances_ordering = sort_object_instances_by_model_order(*this);
            std::vector<unsigned int> first_layer_used_filaments;
            std::vector<std::vector<unsigned int>> all_filaments;
            for (print_object_instance_sequential_active = print_object_instances_ordering.begin(); print_object_instance_sequential_active != print_object_instances_ordering.end(); ++print_object_instance_sequential_active) {
                tool_ordering = ToolOrdering(*(*print_object_instance_sequential_active)->print_object, initial_extruder_id);
                for (size_t idx = 0; idx < tool_ordering.layer_tools().size(); ++idx) {
                    auto& layer_filament = tool_ordering.layer_tools()[idx].extruders;
                    all_filaments.emplace_back(layer_filament);
                    if (idx == 0)
                        first_layer_used_filaments.insert(first_layer_used_filaments.end(), layer_filament.begin(), layer_filament.end());
                }
            }
            sort_remove_duplicates(first_layer_used_filaments);
            auto used_filaments = collect_sorted_used_filaments(all_filaments);
            this->set_slice_used_filaments(first_layer_used_filaments,used_filaments);

            auto physical_unprintables = this->get_physical_unprintable_filaments(used_filaments);
            auto geometric_unprintables = this->get_geometric_unprintable_filaments();
            auto filament_unprintable_volumes = this->get_filament_unprintable_flow(used_filaments);
            // Selector (per-layer regroup) prints skip the static grouping: their print-wide result
            // is stitched from the per-object plans after the ordering loop below.
            const bool dynamic_reorder = this->is_dynamic_group_reorder();
            if (!dynamic_reorder) {
                std::vector<int>filament_maps = this->get_filament_maps();
                auto map_mode = get_filament_map_mode();
                // Grouping returns a nozzle-aware result; the 1-based extruder map for the by-object
                // path is derived from it. It is computed in every static map mode (in manual modes it
                // mirrors the user's assignment) and published print-wide: GCode's per-nozzle
                // placeholder and config-index lookups read it via get_layered_nozzle_group_result(),
                // and without it sequential exports on multi-nozzle printers see an empty nozzle table
                // (e.g. nozzle_diameter_at_nozzle_id[]) and custom g-code fails to resolve.
                auto grouping_result = ToolOrdering::get_recommended_filament_maps(all_filaments, this, map_mode, physical_unprintables, geometric_unprintables, filament_unprintable_volumes);
                this->set_nozzle_group_result(std::make_shared<MultiNozzleUtils::LayeredNozzleGroupResult>(grouping_result));
                // Orca: the sequential write-back stays gated to auto modes. In manual modes the
                // config maps already carry the user's assignment (the per-object ToolOrdering below
                // consumes them directly), so a write-back would only re-store the pre-slice values;
                // keeping the gate avoids churning the config on every sequential manual slice.
                if (map_mode < FilamentMapMode::fmmManual) {
                    auto derived_maps = grouping_result.get_extruder_map(false);
                    if (!derived_maps.empty()) {
                        filament_maps = derived_maps;
                        // Write the maps back: used filaments adopt the engine's extruder/nozzle
                        // choice, unused ones keep their config assignment.
                        // Orca: the config maps are the merge base; fall back to a synthesized base
                        // when no producer sized them to the filament count (CLI runs until the
                        // per-filament synthesis lands there), where indexing per filament would
                        // run out of bounds.
                        std::vector<int> base_filament_map = m_config.filament_map.values;
                        if (base_filament_map.size() != derived_maps.size())
                            base_filament_map.assign(derived_maps.size(), 1);
                        std::vector<int> base_volume_map = m_config.filament_volume_map.values;
                        if (base_volume_map.size() != derived_maps.size())
                            base_volume_map.assign(derived_maps.size(), (int)nvtStandard);
                        update_filament_maps_to_config(FilamentGroupUtils::update_used_filament_values(base_filament_map, derived_maps, used_filaments),
                                                       FilamentGroupUtils::update_used_filament_values(base_volume_map, grouping_result.get_volume_map(), used_filaments),
                                                       grouping_result.get_nozzle_map());
                    }
                }
                // check map valid both in auto and mannual mode
                std::transform(filament_maps.begin(), filament_maps.end(), filament_maps.begin(), [](int value) {return value - 1; });
            }

            //        print_object_instances_ordering = sort_object_instances_by_max_z(print);
            const PrintObject                     *prev_planned_object = nullptr;
            unsigned int                           seq_last_extruder   = (unsigned int)-1;
            MultiNozzleUtils::NozzleStatusRecorder nozzle_status;
            std::vector<std::vector<int>>          nozzle_map_per_layer;
            std::vector<std::vector<unsigned int>> stitched_layer_filaments;
            print_object_instance_sequential_active = print_object_instances_ordering.begin();
            for (; print_object_instance_sequential_active != print_object_instances_ordering.end(); ++print_object_instance_sequential_active) {
                const PrintObject *print_object = (*print_object_instance_sequential_active)->print_object;
                if (dynamic_reorder) {
                    if (print_object != prev_planned_object) {
                        // Plan each unique object once, threading the physical nozzle occupancy and
                        // the previous object's last filament into the next plan; repeated instances
                        // of an object reuse the plan, mirroring the export loop's reuse.
                        ToolOrdering ordering(*print_object, seq_last_extruder);
                        ordering.set_nozzle_status(nozzle_status);
                        ordering.sort_and_build_data(*print_object, seq_last_extruder);
                        nozzle_status = ordering.get_nozzle_status();
                        if (ordering.last_extruder() != static_cast<unsigned int>(-1))
                            seq_last_extruder = ordering.last_extruder();
                        const auto &object_maps = ordering.get_layered_nozzle_group_result().get_layer_filament_nozzle_maps();
                        nozzle_map_per_layer.insert(nozzle_map_per_layer.end(), object_maps.begin(), object_maps.end());
                        // Orca: the stitch input comes from the same orderings that produced the
                        // per-layer maps — the collection loop above is per-instance and seeded -1,
                        // so its layers are misaligned with these plans. layer_tools() of a sorted
                        // ordering already carries the planned per-layer filament order.
                        for (const auto &layer_tool : ordering.layer_tools())
                            stitched_layer_filaments.emplace_back(layer_tool.extruders);
                        m_sequential_dynamic_orderings[print_object] = std::move(ordering);
                        prev_planned_object = print_object;
                    }
                    tool_ordering = m_sequential_dynamic_orderings.at(print_object);
                } else {
                    tool_ordering = ToolOrdering(*print_object, initial_extruder_id);
                    tool_ordering.sort_and_build_data(*print_object, initial_extruder_id);
                }
                if ((initial_extruder_id = tool_ordering.first_extruder()) != static_cast<unsigned int>(-1)) {
                    append(printExtruders, tool_ordering.tools_for_layer(layers_to_print.front().first).extruders);
                }
            }
            if (dynamic_reorder && m_objects.size() > 1) {
                // Stitch the per-object plans into one print-wide selector result. A single-object
                // sequential print publishes (and writes back) from its own ordering instead: the
                // per-object publish gate treats one object as not sequential.
                auto stitched = ToolOrdering::build_sequential_group_result(this, std::move(nozzle_map_per_layer), stitched_layer_filaments,
                                                                            stitched_layer_filaments, used_filaments, physical_unprintables,
                                                                            geometric_unprintables, filament_unprintable_volumes);
                this->set_nozzle_group_result(std::make_shared<MultiNozzleUtils::LayeredNozzleGroupResult>(stitched));
                update_to_config_by_nozzle_group_result(stitched);
            }
        }
        else {
            tool_ordering = this->tool_ordering();
            tool_ordering.assign_custom_gcodes(*this);

            std::vector<unsigned int> first_layer_used_filaments;
            if (!tool_ordering.layer_tools().empty())
                first_layer_used_filaments = tool_ordering.layer_tools().front().extruders;
            // A Body Split coarse body with a thicker first cell prints it on a later event layer, but its
            // filament is still a first-layer filament for the bed temperature and first-layer checks.
            const bool has_first_layer_band = std::any_of(m_objects.begin(), m_objects.end(), [](const PrintObject *object) {
                return object->layers().size() > 1 && object->layers()[1]->in_first_layer_band();
            });
            if (has_first_layer_band) {
                coordf_t band_top = 0.;
                for (const PrintObject *object : m_objects)
                    for (const Layer *layer : object->layers()) {
                        if (!layer->in_first_layer_band())
                            break;
                        band_top = std::max(band_top, layer->print_z);
                    }
                for (const LayerTools &lt : tool_ordering.layer_tools()) {
                    if (&lt == &tool_ordering.layer_tools().front())
                        continue;
                    if (lt.print_z > band_top + EPSILON)
                        break;
                    first_layer_used_filaments.insert(first_layer_used_filaments.end(), lt.extruders.begin(), lt.extruders.end());
                }
                sort_remove_duplicates(first_layer_used_filaments);
            }

            this->set_slice_used_filaments(first_layer_used_filaments, tool_ordering.all_extruders());
            has_wipe_tower = this->has_wipe_tower() && tool_ordering.has_wipe_tower();
            initial_extruder_id = tool_ordering.first_extruder();
            print_object_instances_ordering = chain_print_object_instances(*this);
            append(printExtruders, tool_ordering.tools_for_layer(layers_to_print.front().first).extruders);
        }

        auto objectExtruderMap = getObjectExtruderMap(*this);
        std::vector<std::pair<ObjectID, unsigned int>> objPrintVec;
        for (const PrintInstance* instance : print_object_instances_ordering) {
            const ObjectID& print_object_ID = instance->print_object->id();
            bool existObject = false;
            for (auto& objIDPair : objPrintVec) {
                if (print_object_ID == objIDPair.first) existObject = true;
            }
            if (!existObject && objectExtruderMap.find(print_object_ID) != objectExtruderMap.end())
                objPrintVec.push_back(std::make_pair(print_object_ID, objectExtruderMap.at(print_object_ID)));
        }
        // Orca: Build both the old object-keyed brim map and the per-instance
        // maps used by skirt/brim groups.
        m_brimMap.clear();
        m_brimMapByInstance.clear();
        m_first_layer_convex_hull.points.clear();
        if (this->has_brim()) {
            Polygons islands_area;
            make_brim(*this, this->make_try_cancel(), islands_area, m_brimMap,
                m_brimMapByInstance, objPrintVec, printExtruders,
                &m_objectBrimAreasByInstance);
            for (Polygon& poly_ex : islands_area)
                poly_ex.douglas_peucker(SCALED_RESOLUTION);
            for (Polygon &poly : union_(this->first_layer_islands(), islands_area))
                append(m_first_layer_convex_hull.points, std::move(poly.points));
        }


        if (has_skirt() || has_infinite_skirt() || has_brim()) {
            // Generate skirt/brim groups after brim so per-object and draft-shield footprints
            // include brims when grouping and offsetting skirt loops.
            assert(m_skirt.empty());
            _make_skirt();
            if (m_config.print_sequence == PrintSequence::ByObject &&
                m_config.skirt_type == stPerObject &&
                this->has_shared_per_object_skirt()) {
                throw Slic3r::SlicingError(L("Per-object skirts cannot fit between the objects in By object print sequence.\n\nMove the objects farther apart, reduce brim/skirt size, switch Skirt type to Combined, or switch Print sequence to By layer."));
            }
        }

        this->finalize_first_layer_convex_hull();
        this->set_done(psSkirtBrim);
        if (m_pipeline_plugin_active) run_pipeline_hook(SlicingPipelineStepPlugin::psSkirtBrim, nullptr);

        if (time_cost_with_cache) {
            end_time = (long long)Slic3r::Utils::get_current_time_utc();
            *time_cost_with_cache = *time_cost_with_cache + end_time - start_time;
        }
    }
    //BBS
    for (PrintObject *obj : m_objects) {
        if (((!use_cache)&&(need_slicing_objects.count(obj) != 0))
            || (use_cache &&(re_slicing_objects.count(obj) != 0))){
            const bool was_done = obj->is_step_done(posSimplifyPath);
            obj->simplify_extrusion_path();
            // Unlike every other seam (all inside the `if (!use_cache)` block above), this loop is
            // shared with the use_cache path (re_slicing_objects), so `!use_cache` must be checked
            // explicitly here to keep hooks from ever firing on cache-loaded (plugin-final) objects.
            if (!use_cache && m_pipeline_plugin_active && !was_done && obj->is_step_done(posSimplifyPath))
                run_pipeline_hook(SlicingPipelineStepPlugin::posSimplifyPath, obj);
        }
        else {
            if (obj->set_started(posSimplifyPath))
                obj->set_done(posSimplifyPath);
            if (obj->set_started(posSimplifyInfill))
                obj->set_done(posSimplifyInfill);
            if (obj->set_started(posSimplifySupportPath))
                obj->set_done(posSimplifySupportPath);
        }
    }

    // BBS
    bool has_adaptive_layer_height = false;
    for (PrintObject* obj : m_objects) {
        if (obj->model_object()->layer_height_profile.empty() == false) {
            has_adaptive_layer_height = true;
            break;
        }
    }
    if(!m_no_check /*&& !has_adaptive_layer_height*/)
    {
        using Clock                 = std::chrono::high_resolution_clock;
        auto            startTime   = Clock::now();
        std::optional<const FakeWipeTower *> wipe_tower_opt = {};
        // A tower-enabled print whose tool ordering needs no tower has no tower to collide with.
        if (this->has_wipe_tower() && !m_wipe_tower_data.tool_changes.empty()) {
            m_fake_wipe_tower.set_pos({m_config.wipe_tower_x.get_at(m_plate_index), m_config.wipe_tower_y.get_at(m_plate_index)});
            wipe_tower_opt = std::make_optional<const FakeWipeTower *>(&m_fake_wipe_tower);
        }
        auto            conflictRes = ConflictChecker::find_inter_of_lines_in_diff_objs(m_objects, wipe_tower_opt);
        auto            endTime     = Clock::now();
        volatile double seconds     = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() / (double) 1000;
        BOOST_LOG_TRIVIAL(info) << "gcode path conflicts check takes " << seconds << " secs.";

        m_conflict_result = conflictRes;
        if (conflictRes.has_value()) {
            BOOST_LOG_TRIVIAL(error) << boost::format("gcode path conflicts found between %1% and %2%")%conflictRes.value()._objName1 %conflictRes.value()._objName2;
        }
    }

    // The band schedule was committed in PrintObject::make_perimeters() against the binding resolved
    // before slicing. The by-layer grouping in psWipeTower and the by-object grouping in psSkirtBrim
    // can rewrite filament_map after that, and both have run by here. Check the binding is still the
    // one the coarse roads were planned for.
    throw_on_regional_binding_mismatch(false);

    BOOST_LOG_TRIVIAL(info) << "Slicing process finished." << log_memory_info();
}

// G-code export process, running at a background thread.
// The export_gcode may die for various reasons (fails to process filename_format,
// write error into the G-code, cannot execute post-processing scripts).
// It is up to the caller to show an error message.
std::string Print::export_gcode(const std::string& path_template, GCodeProcessorResult* result, ThumbnailsGeneratorCallback thumbnail_cb)
{
    // output everything to a G-code file
    // The following call may die if the filename_format template substitution fails.
    std::string path = this->output_filepath(path_template);
    std::string message;
    if (!path.empty() && result == nullptr) {
        // Only show the path if preview_data is not set -> running from command line.
        message = L("Exporting G-code");
        message += " to ";
        message += path;
    } else
        message = L("Generating G-code");
    this->set_status(80, message);

    // The following line may die for multiple reasons.
    GCode gcode;
    //BBS: compute plate offset for gcode-generator
    const Vec3d origin = this->get_plate_origin();
    gcode.set_gcode_offset(origin(0), origin(1));
    gcode.do_export(this, path.c_str(), result, thumbnail_cb);
    gcode.export_layer_filaments(result);

    // Only the command line reports a slice with no Preview (result == nullptr). This restates the
    // block do_export() already wrote, so the two cannot disagree.
    if (result == nullptr)
        if (const std::string summary = this->regional_layer_plan_export().summary; ! summary.empty())
            boost::nowide::cout << summary << std::endl;
    //BBS
    if (result != nullptr) {
        result->conflict_result = m_conflict_result;
        // Surface the slicer's per-filament nozzle grouping onto the post-slice result
        // the device GUI reads. This is the static L/R + rack subset the multi-nozzle path computes;
        // null for single-nozzle prints where nothing computes it. It is assigned after g-code
        // generation and read by no emitter, so it does not affect the emitted g-code.
        result->nozzle_group_result = this->get_layered_nozzle_group_result();
    }
    return path.c_str();
}

void Print::_make_skirt()
{
    const bool generate_skirt = this->has_skirt() || this->has_infinite_skirt();

    // First off we need to decide how tall the skirt must be.
    // The skirt_height option from config is expressed in layers, but our
    // object might have different layer heights, so we need to find the print_z
    // of the highest layer involved.
    // Note that unless has_infinite_skirt() == true
    // the actual skirt might not reach this $skirt_height_z value since the print
    // order of objects on each layer is not guaranteed and will not generally
    // include the thickest object first. It is just guaranteed that a skirt is
    // prepended to the first 'n' layers (with 'n' = skirt_height).
    // $skirt_height_z in this case is the highest possible skirt height for safety.
    coordf_t skirt_height_z = 0.;
    if (generate_skirt) {
        for (const PrintObject *object : m_objects) {
            size_t skirt_layers = this->has_infinite_skirt() ?
                object->layer_count() :
                std::min(size_t(m_config.skirt_height.value), object->layer_count());
            skirt_height_z = std::max(skirt_height_z, object->m_layers[skirt_layers-1]->print_z);
        }
    }

    struct ObjectSkirtHull {
        PrintObject* object;
        Polygon      hull;
    };

    // Orca: Build one local occupied hull per object from object/support
    // geometry up to skirt height. Instances translate this hull later.
    std::vector<ObjectSkirtHull> object_convex_hulls;
    for (PrintObject *object : m_objects) {
        Points object_points;
        // A Body Split first cell that stands on the bed and ends above the first layer is not in the
        // first layer's slices; its slab coverage holds it (as in Brim.cpp print_object_bed_outline).
        const bool first_layer_band = object->m_layers.size() > 1 && object->m_layers[1]->in_first_layer_band();
        // Get object layers up to skirt_height_z.
        for (const Layer *layer : object->m_layers) {
            if (generate_skirt && layer->print_z > skirt_height_z)
                break;
            for (const ExPolygon &expoly : layer->lslices)
                // Collect the outer contour points only, ignore holes for the calculation of the convex hull.
                append(object_points, expoly.contour.points);
            if (first_layer_band && layer->has_slab_coverage())
                for (const ExPolygon &expoly : layer->slab_coverage())
                    append(object_points, expoly.contour.points);
        }
        // Get support layers up to skirt_height_z.
        for (const SupportLayer *layer : object->support_layers()) {
            if (generate_skirt && layer->print_z > skirt_height_z)
                break;
            layer->support_fills.collect_points(object_points);
            layer->fine_body_fills.collect_points(object_points);
        }

        object_convex_hulls.push_back({ object, Slic3r::Geometry::convex_hull(object_points) });
    }

    if (object_convex_hulls.empty())
        return;

    this->throw_if_canceled();

    // Skirt may be printed on several layers, having distinct layer heights,
    // but loops must be aligned so can't vary width/spacing
    // TODO: use each extruder's own flow
    double initial_layer_print_height = this->skirt_first_layer_height();
    Flow   flow = this->skirt_flow();
    float  spacing = flow.spacing();
    double mm3_per_mm = flow.mm3_per_mm();

    std::vector<size_t> extruders;
    std::vector<double> extruders_e_per_mm;
    {
        auto set_extruders = this->extruders();
        extruders.reserve(set_extruders.size());
        extruders_e_per_mm.reserve(set_extruders.size());
        for (auto &extruder_id : set_extruders) {
            extruders.push_back(extruder_id);
            extruders_e_per_mm.push_back(Extruder((unsigned int)extruder_id, &m_config, m_config.single_extruder_multi_material).e_per_mm(mm3_per_mm));
        }
    }

    // Initial skirt centerline offset from the occupied outline.
    // The skirt will touch the occupied outline if skirt_distance is zero.
    // Generate loops inward to outward; callers reverse them before G-code export.
    // Loop while we have less skirts than required or any extruder hasn't reached the min length if any.
    auto append_skirt_loops_for_hull = [&](const Polygon& hull, ExtrusionEntityCollection& dst, bool collect_skirt_hull) {
        float distance = float(scale_(m_config.skirt_distance.value - spacing/2.));
        std::vector<coordf_t> extruded_length(extruders.size(), 0.);
        for (size_t i = m_config.skirt_loops, extruder_idx = 0; i > 0; -- i) {
            this->throw_if_canceled();
            // Offset the skirt outside.
            distance += float(scale_(spacing));
            // Generate the skirt centerline.
            Polygon loop;
            {
                // Orca: the hull already represents the occupied outline used for this skirt.
                Polygons loops = offset(hull, distance, ClipperLib::jtRound, float(scale_(0.1)));
                Geometry::simplify_polygons(loops, scale_(0.05), &loops);
			    if (loops.empty())
				    break;
			    loop = loops.front();
            }
            // Extrude the skirt loop.
            ExtrusionLoop eloop(elrSkirt);
            eloop.paths.emplace_back(ExtrusionPath(
                ExtrusionPath(
                    erSkirt,
                    (float)mm3_per_mm,         // this will be overridden at G-code export time
                    flow.width(),
				    (float)initial_layer_print_height  // this will be overridden at G-code export time
                )));
            eloop.paths.back().polyline = Polyline3(loop.split_at_first_point());
            dst.append(eloop);
            if (m_config.min_skirt_length.value > 0) {
                // The skirt length is limited. Sum the total amount of filament length extruded, in mm.
                extruded_length[extruder_idx] += unscale<double>(loop.length()) * extruders_e_per_mm[extruder_idx];
                if (extruded_length[extruder_idx] < m_config.min_skirt_length.value) {
                    // Not extruded enough yet with the current extruder. Add another loop.
                    if (i == 1)
                        ++ i;
                } else {
                    assert(extruded_length[extruder_idx] >= m_config.min_skirt_length.value);
                    // Enough extruded with the current extruder. Extrude with the next one,
                    // until the prescribed number of skirt loops is extruded.
                    if (extruder_idx + 1 < extruders.size())
                        ++ extruder_idx;
                }
            } else {
                // The skirt length is not limited, extrude the skirt with the 1st extruder only.
            }
        }

        if (collect_skirt_hull)
            for (Polygon &poly : offset(hull, distance + 0.5f * float(scale_(spacing)), ClipperLib::jtRound, float(scale_(0.1))))
                append(m_skirt_convex_hull, std::move(poly.points));
    };

    m_skirt.clear();
    m_skirt_brim_groups.clear();
    m_has_shared_per_object_skirt = false;
    for (const ObjectSkirtHull& object_hull : object_convex_hulls)
        object_hull.object->m_skirt.clear();

    if (m_config.skirt_type == stCombined || m_config.skirt_type == stPerObject) {
        struct SkirtBrimGroupItem {
            Points       occupied_points;
            ObjectID     object_id;
            size_t       instance_id;
            bool         emits_skirt;
        };

        // Orca: Each object instance can emit skirt/brim. Wipe tower is only an
        // obstacle here; it may merge nearby items, but does not emit anything.
        std::vector<SkirtBrimGroupItem> group_items;
        const coord_t grouping_offset = scale_(m_config.skirt_distance.value + m_config.skirt_loops.value * spacing);
        for (const ObjectSkirtHull& object_hull : object_convex_hulls) {
            PrintObject* object = object_hull.object;
            std::vector<size_t> object_item_indices;
            object_item_indices.reserve(object->instances().size());
            for (size_t instance_idx = 0; instance_idx < object->instances().size(); ++instance_idx) {
                const PrintInstance &instance = object->instances()[instance_idx];
                Points copy_points = object_hull.hull.points;
                for (Point &pt : copy_points)
                    pt += instance.shift;
                if (copy_points.size() < 3)
                    continue;

                object_item_indices.push_back(group_items.size());
                group_items.push_back({ std::move(copy_points), object->id(), instance_idx, true });
            }

            for (size_t item_idx : object_item_indices) {
                const ObjectInstanceID key{ object->id(), group_items[item_idx].instance_id };
                if (auto instance_brim_it = m_objectBrimAreasByInstance.find(key); instance_brim_it != m_objectBrimAreasByInstance.end())
                    for (const ExPolygon& area : instance_brim_it->second)
                        append(group_items[item_idx].occupied_points, area.contour.points);
            }
        }

        // Orca: the wipe tower contributes occupied area, but does not emit a skirt by itself.
        Points wipe_tower_points = this->first_layer_wipe_tower_corners();
        if (wipe_tower_points.size() >= 3)
            group_items.push_back({ std::move(wipe_tower_points), ObjectID(), size_t(-1), false });

        std::vector<size_t> parent(group_items.size());
        std::iota(parent.begin(), parent.end(), 0);
        // Orca: Use union-find so touching items can be merged while scanning.
        auto find_parent = [&parent](size_t idx) {
            while (parent[idx] != idx) {
                parent[idx] = parent[parent[idx]];
                idx = parent[idx];
            }
            return idx;
        };
        auto unite = [&parent, &find_parent](size_t a, size_t b) {
            a = find_parent(a);
            b = find_parent(b);
            if (a != b)
                parent[b] = a;
        };

        // Orca: Combined skirt starts with all items in the same group.
        if (m_config.skirt_type == stCombined && !group_items.empty())
            for (size_t i = 1; i < group_items.size(); ++i)
                unite(0, i);

        auto build_skirt_brim_groups = [&]() {
            struct SkirtBrimGroupData {
                Points                                           points;
                std::vector<ObjectInstanceID>                    instances;
                bool                                             emits_skirt = false;
            };

            std::map<size_t, SkirtBrimGroupData> grouped;
            for (size_t i = 0; i < group_items.size(); ++i) {
                SkirtBrimGroupData& group = grouped[find_parent(i)];
                append(group.points, group_items[i].occupied_points);
                if (group_items[i].object_id.valid())
                    group.instances.push_back({ group_items[i].object_id, group_items[i].instance_id });
                group.emits_skirt = group.emits_skirt || group_items[i].emits_skirt;
            }
            return grouped;
        };

        bool groups_changed = m_config.skirt_type == stPerObject;
        while (groups_changed) {
            groups_changed = false;
            auto grouped_points = build_skirt_brim_groups();
            std::vector<std::pair<size_t, Polygon>> group_envelopes;
            for (const auto& [root, group] : grouped_points) {
                if (group.points.size() < 3)
                    continue;

                // Orca: Only skirt-emitting groups are expanded by skirt distance;
                // obstacle-only groups stay at their occupied outline.
                Polygon envelope = Geometry::convex_hull(group.points);
                if (group.emits_skirt) {
                    // Orca: If the expanded skirt outline touches another group
                    // or obstacle, merge them and run the pass again.
                    Polygons envelopes = offset(envelope, grouping_offset, ClipperLib::jtRound, float(scale_(0.1)));
                    if (envelopes.empty())
                        continue;
                    envelope = std::move(envelopes.front());
                }
                group_envelopes.emplace_back(root, std::move(envelope));
            }

            for (size_t i = 0; i < group_envelopes.size(); ++i) {
                for (size_t j = i + 1; j < group_envelopes.size(); ++j) {
                    const size_t root_i = find_parent(group_envelopes[i].first);
                    const size_t root_j = find_parent(group_envelopes[j].first);
                    if (root_i != root_j && !intersection(group_envelopes[i].second, group_envelopes[j].second).empty()) {
                        unite(root_i, root_j);
                        groups_changed = true;
                    }
                }
            }
        }

        auto make_brims_for_skirt_brim_group = [this](const std::vector<ObjectInstanceID>& group_instances) {
            std::vector<SkirtBrimGroup::Brim> brims;
            std::vector<ObjectInstanceID> brim_instances;
            for (const ObjectInstanceID& instance : group_instances) {
                const auto brim_it = m_brimMapByInstance.find(instance);
                if (brim_it != m_brimMapByInstance.end() && !brim_it->second.empty())
                    brim_instances.push_back(instance);
            }

            const bool combine_group_brims = m_config.combine_brims && brim_instances.size() > 1;
            if (!combine_group_brims) {
                for (const ObjectInstanceID& instance : brim_instances)
                    brims.push_back({ m_brimMapByInstance.at(instance), { instance } });
                return brims;
            }

            std::vector<size_t> brim_parent(brim_instances.size());
            std::iota(brim_parent.begin(), brim_parent.end(), 0);
            auto find_brim_parent = [&brim_parent](size_t idx) {
                while (brim_parent[idx] != idx) {
                    brim_parent[idx] = brim_parent[brim_parent[idx]];
                    idx = brim_parent[idx];
                }
                return idx;
            };
            auto unite_brims = [&brim_parent, &find_brim_parent](size_t a, size_t b) {
                a = find_brim_parent(a);
                b = find_brim_parent(b);
                if (a != b)
                    brim_parent[b] = a;
            };

            const coord_t brim_contact_distance = coord_t(brim_flow().scaled_spacing() * 2.);
            for (size_t i = 0; i < brim_instances.size(); ++i) {
                const auto area_i = m_objectBrimAreasByInstance.find(brim_instances[i]);
                if (area_i == m_objectBrimAreasByInstance.end())
                    continue;
                for (size_t j = i + 1; j < brim_instances.size(); ++j) {
                    const auto area_j = m_objectBrimAreasByInstance.find(brim_instances[j]);
                    if (area_j != m_objectBrimAreasByInstance.end() &&
                        !intersection_ex(offset_ex(area_i->second, brim_contact_distance, jtRound, SCALED_RESOLUTION), area_j->second).empty())
                        unite_brims(i, j);
                }
            }

            std::map<size_t, std::vector<ObjectInstanceID>> combined_brim_ids;
            for (size_t i = 0; i < brim_instances.size(); ++i)
                combined_brim_ids[find_brim_parent(i)].push_back(brim_instances[i]);

            for (const auto& [_, instances] : combined_brim_ids) {
                if (instances.size() == 1) {
                    const ObjectInstanceID& instance = instances.front();
                    brims.push_back({ m_brimMapByInstance.at(instance), { instance } });
                    continue;
                }

                ExPolygons combined_area;
                for (const ObjectInstanceID& instance : instances)
                    expolygons_append(combined_area, m_objectBrimAreasByInstance.at(instance));
                combined_area = union_ex(combined_area);
                const float scaled_resolution  = float(scaled(m_config.resolution.value));
                const float brim_cleanup_delta = std::max(scaled_resolution, float(SCALED_EPSILON));
                combined_area = offset2_ex(combined_area, brim_cleanup_delta, -brim_cleanup_delta, jtRound, scaled_resolution);

                Polygons islands_area;
                brims.push_back({ makeBrimInfillFromPlateCoordinates(combined_area, *this, islands_area), instances });
            }

            return brims;
        };

        auto grouped_points = build_skirt_brim_groups();
        for (auto& [_, group] : grouped_points) {
            if (!group.emits_skirt || group.points.size() < 3)
                continue;
            if (generate_skirt && m_config.skirt_type == stPerObject && group.instances.size() > 1)
                m_has_shared_per_object_skirt = true;
            // Orca: Group points already include the occupied outline, so don't
            // add skirt distance here again.
            ExtrusionEntityCollection group_skirt;
            if (generate_skirt)
                append_skirt_loops_for_hull(Geometry::convex_hull(group.points), group_skirt, true);
            std::vector<SkirtBrimGroup::Brim> group_brims = make_brims_for_skirt_brim_group(group.instances);
            if (!group_skirt.empty()) {
                group_skirt.reverse();
                // Orca: Keep m_skirt filled for code that still reads a flat
                // skirt collection.
                m_skirt.append(group_skirt.entities);
            }
            if (!group_skirt.empty() || !group_brims.empty())
                m_skirt_brim_groups.push_back({ std::move(group_skirt), std::move(group.instances), std::move(group_brims) });
        }
    }
}

Polygons Print::first_layer_islands() const
{
    Polygons islands;
    for (PrintObject *object : m_objects) {
        Polygons object_islands;
        for (ExPolygon &expoly : object->m_layers.front()->lslices)
            object_islands.push_back(expoly.contour);
        if (!object->support_layers().empty()) {
            if (object->support_layers().front()->support_type==stInnerNormal)
                object->support_layers().front()->support_fills.polygons_covered_by_spacing(object_islands, float(SCALED_EPSILON));
            else if(object->support_layers().front()->support_type==stInnerTree) {
                ExPolygons &expolys_first_layer = object->m_support_layers.front()->lslices;
                for (ExPolygon &expoly : expolys_first_layer) { object_islands.push_back(expoly.contour); }
            }
        }
        islands.reserve(islands.size() + object_islands.size() * object->instances().size());
        for (const PrintInstance &instance : object->instances())
            for (Polygon &poly : object_islands) {
                islands.push_back(poly);
                islands.back().translate(instance.shift);
            }
    }
    return islands;
}

Points Print::first_layer_wipe_tower_corners(bool check_wipe_tower_existance) const
{
    Points corners;
    if (check_wipe_tower_existance && (!has_wipe_tower() || m_wipe_tower_data.tool_changes.empty()))
        return corners;
    {
        double width = m_wipe_tower_data.bbx.max.x() - m_wipe_tower_data.bbx.min.x();
        double depth = m_wipe_tower_data.bbx.max.y() -m_wipe_tower_data.bbx.min.y();
        Vec2d  pt0   = m_wipe_tower_data.bbx.min + m_wipe_tower_data.rib_offset.cast<double>();
        
        // First the corners.
        std::vector<Vec2d> pts = { pt0,
                                   Vec2d(pt0.x()+width, pt0.y()),
                                   Vec2d(pt0.x()+width, pt0.y()+depth),
                                   Vec2d(pt0.x(),pt0.y()+depth)
                                 };

        // Now the stabilization cone.
        Vec2d center = (pts[0] + pts[2])/2.;
        const auto [cone_R, cone_x_scale] = WipeTower2::get_wipe_tower_cone_base(m_config.prime_tower_width, m_wipe_tower_data.height, m_wipe_tower_data.depth, m_config.wipe_tower_cone_angle);
        double r = cone_R + m_wipe_tower_data.brim_width;
        for (double alpha = 0.; alpha<2*M_PI; alpha += M_PI/20.)
            pts.emplace_back(center + r*Vec2d(std::cos(alpha)/cone_x_scale, std::sin(alpha)));

        for (Vec2d& pt : pts) {
            pt = Eigen::Rotation2Dd(Geometry::deg2rad(m_config.wipe_tower_rotation_angle.value)) * pt;
            //Orca: offset the wipe tower to the plate origin
            pt += Vec2d(m_config.wipe_tower_x.get_at(m_plate_index) + m_origin(0), m_config.wipe_tower_y.get_at(m_plate_index) + m_origin(1));
            corners.emplace_back(Point(scale_(pt.x()), scale_(pt.y())));
        }
    }
    return corners;
}

//SoftFever
Vec2d Print::translate_to_print_space(const Vec2d &point) const {
    //const BoundingBoxf bed_bbox(config().printable_area.values);
    return Vec2d(point(0) - m_origin(0), point(1) - m_origin(1));
}

Vec2d Print::translate_to_print_space(const Point &point) const {
    return Vec2d(unscaled(point.x()) - m_origin(0), unscaled(point.y()) - m_origin(1));
}

FilamentTempType Print::get_filament_temp_type(const std::string& filament_type)
{
    // Range-based classification only: do not use filament_info.json.
    int min_temp, max_temp;
    if (MaterialType::get_temperature_range(filament_type, min_temp, max_temp)) {
        if (max_temp <= 250)
            return FilamentTempType::LowTemp;
        else if (max_temp < 280)
            return FilamentTempType::HighLowCompatible;
        else
            return FilamentTempType::HighTemp;
    }

    return Undefine;
}

int Print::get_hrc_by_nozzle_type(const NozzleType&type)
{
    static std::map<std::string, int>nozzle_type_to_hrc;
    if (nozzle_type_to_hrc.empty()) {
        fs::path file_path = fs::path(resources_dir()) / "info" / "nozzle_info.json";
        boost::nowide::ifstream in(file_path.string());
        //std::ifstream in(file_path.string());
        json j;
        try {
            j = json::parse(in);
            in.close();
            for (const auto& elem : j["nozzle_hrc"].items())
                nozzle_type_to_hrc[elem.key()] = elem.value();
        }
        catch (const json::parse_error& err) {
            in.close();
            BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << ": parse " << file_path.string() << " got a nlohmann::detail::parse_error, reason = " << err.what();
            nozzle_type_to_hrc = {
                {"hardened_steel",55},
                {"stainless_steel",20},
                {"tungsten_carbide", 85},
                {"brass",2},
                {"undefine",0}
            };
        }
    }
    auto iter = nozzle_type_to_hrc.find(NozzleTypeEumnToStr[type]);
    if (iter != nozzle_type_to_hrc.end())
        return iter->second;
    //0 represents undefine
    return 0;
}

std::vector<std::string> Print::get_incompatible_filaments_by_nozzle(const float nozzle_diameter, const std::optional<NozzleVolumeType> nozzle_volume_type)
{
    static std::map<std::string, std::map<std::string, std::vector<std::string>>> incompatible_filaments;
    if(incompatible_filaments.empty()){
        fs::path file_path = fs::path(resources_dir()) / "info" / "nozzle_incompatibles.json";
        boost::nowide::ifstream in(file_path.string());
        json j;
        try {
            j = json::parse(in);
            for(auto& [volume_type, diameter_list] : j["incompatible_nozzles"].items()) {
                for(auto& [diameter, filaments]: diameter_list.items()){
                    incompatible_filaments[volume_type][diameter] = filaments.get<std::vector<std::string>>();
                }
            }
        }
        catch(const json::parse_error& err){
            in.close();
            BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << ": parse " << file_path.string() << " got a nlohmann::detail::parse_error, reason = " << err.what();

            incompatible_filaments[get_nozzle_volume_type_string(NozzleVolumeType::nvtHighFlow)] = {};
            incompatible_filaments[get_nozzle_volume_type_string(NozzleVolumeType::nvtStandard)] = {};
        }
    }
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << nozzle_diameter;
    std::string diameter_str = oss.str();

    if(nozzle_volume_type.has_value()){
        return incompatible_filaments[get_nozzle_volume_type_string(nozzle_volume_type.value())][diameter_str];
    }

    std::vector<std::string> incompatible_filaments_list;
    for(auto& [volume_type, diameter_list] : incompatible_filaments){
        auto iter = diameter_list.find(diameter_str);
        if(iter != diameter_list.end()){
            append(incompatible_filaments_list, iter->second);
        }
    }
    return incompatible_filaments_list;
}

void Print::finalize_first_layer_convex_hull()
{
    append(m_first_layer_convex_hull.points, m_skirt_convex_hull);
    if (m_first_layer_convex_hull.empty()) {
        // Neither skirt nor brim was extruded. Collect points of printed objects from 1st layer.
        for (Polygon &poly : this->first_layer_islands())
            append(m_first_layer_convex_hull.points, std::move(poly.points));
    }
    append(m_first_layer_convex_hull.points, this->first_layer_wipe_tower_corners());
    m_first_layer_convex_hull = Geometry::convex_hull(m_first_layer_convex_hull.points);
}

void Print::update_filament_maps_to_config(std::vector<int> f_maps, std::vector<int> f_volume_maps, std::vector<int> f_nozzle_maps)
{
    if ((m_config.filament_map.values != f_maps) || (m_config.filament_volume_map.values != f_volume_maps) || (m_config.filament_nozzle_map.values != f_nozzle_maps))
    {
        BOOST_LOG_TRIVIAL(info) << __FUNCTION__ << boost::format(": filament maps changed after pre-slicing.");
        m_ori_full_print_config.option<ConfigOptionInts>("filament_map", true)->values = f_maps;
        m_config.filament_map.values = f_maps;

        if (!f_volume_maps.empty()) {
            m_ori_full_print_config.option<ConfigOptionInts>("filament_volume_map", true)->values = f_volume_maps;
            m_config.filament_volume_map.values = f_volume_maps;
        }
        else {
            m_ori_full_print_config.option<ConfigOptionInts>("filament_volume_map", true)->values.resize(f_maps.size(), nvtStandard);
            m_config.filament_volume_map.values.resize(f_maps.size(), nvtStandard);
        }

        if (!f_nozzle_maps.empty()) {
            m_ori_full_print_config.option<ConfigOptionInts>("filament_nozzle_map", true)->values = f_nozzle_maps;
            m_config.filament_nozzle_map.values = f_nozzle_maps;
        }
    }

    {
        int extruder_count = 1, extruder_volume_type_count = 1;
        bool support_multi = m_ori_full_print_config.support_different_extruders(extruder_count);
        std::vector<std::vector<NozzleVolumeType>> nozzle_volume_types;
        extruder_volume_type_count = m_ori_full_print_config.get_extruder_nozzle_volume_count(extruder_count, nozzle_volume_types);

        //filament_map_2
        // Orca: seed with 0-based extruder indices so the override keying below degenerates to the
        // plain per-extruder slot when the rebuild loop is skipped; the loop overwrites every
        // entry when it runs.
        m_config.filament_map_2.values = f_maps;
        for (auto& v : m_config.filament_map_2.values)
            --v;
        auto opt_extruder_type = dynamic_cast<const ConfigOptionEnumsGeneric*>(m_ori_full_print_config.option("extruder_type"));
        auto opt_nozzle_volume_type = dynamic_cast<const ConfigOptionEnumsGeneric*>(m_ori_full_print_config.option("nozzle_volume_type"));
        // Orca: the loop tolerates configs without the extruder options (unit tests, degenerate
        // presets); the backfill and the override are bounds-checked because the change block above
        // is skipped when the maps are unchanged, in which case the stored map may be shorter than
        // the filament count.
        auto* ori_volume_map = m_ori_full_print_config.option<ConfigOptionInts>("filament_volume_map", true);
        for (int index = 0; opt_extruder_type && opt_nozzle_volume_type && index < f_maps.size(); index++)
        {
            ExtruderType extruder_type = (ExtruderType)(opt_extruder_type->get_at(f_maps[index] - 1));
            NozzleVolumeType nozzle_volume_type = (NozzleVolumeType)(opt_nozzle_volume_type->get_at(f_maps[index] - 1));
            if (f_volume_maps.empty()) {
                // No per-filament map supplied: backfill from the extruder's own volume type.
                if (m_config.filament_volume_map.values.size() > index)
                    m_config.filament_volume_map.values[index] = nozzle_volume_type;
                if (ori_volume_map->values.size() > index)
                    ori_volume_map->values[index] = nozzle_volume_type;
            }
            else if ((extruder_volume_type_count > extruder_count) && (m_config.filament_volume_map.values.size() > index))
                nozzle_volume_type = (NozzleVolumeType)(m_config.filament_volume_map.values[index]);
            // Orca: when the process variant columns cannot be matched (degenerate
            // print_extruder_id), key the override by plain extruder index like the seeding
            // above instead of poisoning the map with -1.
            int slot_index = m_ori_full_print_config.get_index_for_extruder(f_maps[index], "print_extruder_id", extruder_type, nozzle_volume_type, "print_extruder_variant");
            m_config.filament_map_2.values[index] = slot_index >= 0 ? slot_index : f_maps[index] - 1;
        }

        m_full_print_config = m_ori_full_print_config;
        std::set<std::string> filament_keys = filament_options_with_variant;
        filament_keys.insert("filament_self_index");
        if ((extruder_count > 1) || support_multi)
            m_full_print_config.update_values_to_printer_extruders_for_multiple_filaments(m_full_print_config, extruder_count, extruder_volume_type_count, filament_keys,  "filament_self_index", "filament_extruder_variant");

        const std::vector<std::string> &extruder_retract_keys = print_config_def.extruder_retract_keys();
        const std::string               filament_prefix       = "filament_";
        t_config_option_keys            print_diff;
        DynamicPrintConfig              filament_overrides;
        for (auto& opt_key: extruder_retract_keys)
        {
            const ConfigOption *opt_new_filament = m_full_print_config.option(filament_prefix + opt_key);
            const ConfigOption *opt_new_machine = m_full_print_config.option(opt_key);
            const ConfigOption *opt_old_machine = m_config.option(opt_key);

            if (opt_new_filament)
                compute_filament_override_value(opt_key, opt_old_machine, opt_new_machine, opt_new_filament, m_full_print_config, print_diff, filament_overrides, m_config.filament_map_2.values);
        }

        if ((extruder_count > 1) || support_multi) {
            t_config_option_keys keys(filament_options_with_variant.begin(), filament_options_with_variant.end());
            keys.push_back("filament_self_index");
            m_config.apply_only(m_full_print_config, keys, true);
        }
        if (!print_diff.empty()) {
            m_placeholder_parser.apply_config(filament_overrides);
            m_config.apply(filament_overrides);
        }
    }
    update_filament_self_index_cache();
    m_has_auto_filament_map_result = true;
}

bool Print::collect_filament_variant_uses(const MultiNozzleUtils::LayeredNozzleGroupResult& group_result,
                                          const DynamicPrintConfig& config,
                                          std::unordered_map<int, std::vector<FilamentVariantUse>>& uses) const
{
    auto opt_filament_type = config.option<ConfigOptionStrings>("filament_type");
    auto opt_extruder_type = dynamic_cast<const ConfigOptionEnumsGeneric*>(config.option("extruder_type"));
    if (!opt_filament_type || !opt_extruder_type)
        return false;

    const size_t filament_count = opt_filament_type->values.size();
    const size_t extruder_count = opt_extruder_type->values.size();
    auto add_use = [&](std::set<FilamentVariantUse> &variant_set, const MultiNozzleUtils::NozzleInfo &nozzle) {
        // Orca: a persisted result can outlive a printer swap; never index the extruder
        // arrays with a stale nozzle record.
        if (nozzle.extruder_id < 0 || static_cast<size_t>(nozzle.extruder_id) >= extruder_count)
            return;
        FilamentVariantUse use;
        use.extruder_type      = static_cast<ExtruderType>(opt_extruder_type->get_at(nozzle.extruder_id));
        use.nozzle_volume_type = nozzle.volume_type;
        use.extruder_id        = nozzle.extruder_id;
        variant_set.insert(use);
    };
    for (size_t f_index = 0; f_index < filament_count; ++f_index) {
        std::set<FilamentVariantUse> variant_set;
        for (const MultiNozzleUtils::NozzleInfo &nozzle : group_result.get_nozzles_for_filament(static_cast<int>(f_index)))
            add_use(variant_set, nozzle);
        // A filament the plan never routes (not printed) still needs a deterministic slot: take
        // its default-map assignment from the result itself, so both the slice-time write-back
        // and the apply-time reproduction resolve the same slot even when the surrounding
        // filament_map has not round-tripped through the plate config in between.
        if (variant_set.empty()) {
            if (auto default_nozzle = group_result.get_nozzle_for_filament(static_cast<int>(f_index), -1); default_nozzle.has_value())
                add_use(variant_set, *default_nozzle);
        }
        // Filaments still without a variant stay absent from the map: the slot rebuild then
        // resolves them from their static filament_map / filament_volume_map assignment.
        if (!variant_set.empty())
            uses[static_cast<int>(f_index)] = std::vector<FilamentVariantUse>(variant_set.begin(), variant_set.end());
    }
    return true;
}

void Print::update_to_config_by_nozzle_group_result(const MultiNozzleUtils::LayeredNozzleGroupResult& group_result)
{
    std::vector<int> derived_maps = group_result.get_extruder_map(false); // 1-based
    if (derived_maps.empty())
        return;

    if (!group_result.is_support_dynamic_nozzle_map()) {
        // No filament actually migrated between nozzles, so the plan reduces to a single
        // grouping: write all maps like the static paths do, and the next apply re-derives
        // identical slots from the written maps.
        std::vector<int> base_filament_map = m_config.filament_map.values;
        if (base_filament_map.size() != derived_maps.size())
            base_filament_map.assign(derived_maps.size(), 1);
        std::vector<int> base_volume_map = m_config.filament_volume_map.values;
        if (base_volume_map.size() != derived_maps.size())
            base_volume_map.assign(derived_maps.size(), (int)nvtStandard);
        const std::vector<unsigned int> used_filaments = group_result.get_used_filaments();
        update_filament_maps_to_config(FilamentGroupUtils::update_used_filament_values(base_filament_map, derived_maps, used_filaments),
                                       FilamentGroupUtils::update_used_filament_values(base_volume_map, group_result.get_volume_map(), used_filaments),
                                       group_result.get_nozzle_map());
        return;
    }

    // Orca: keep the coarse per-filament extruder map published even though the per-layer truth
    // lives in the grouping result: the pre-export consumers, the plate read-back after slicing
    // and the preview panel all key on filament_map. The write is direct — the full map
    // write-back's single-slot rebuild would undo the per-variant expansion below.
    m_ori_full_print_config.option<ConfigOptionInts>("filament_map", true)->values = derived_maps;
    m_config.filament_map.values = derived_maps;

    std::unordered_map<int, std::vector<FilamentVariantUse>> filament_variant_uses;
    if (!collect_filament_variant_uses(group_result, m_ori_full_print_config, filament_variant_uses)) {
        // Degenerate config (no filament/extruder typing): fall back to the single-slot
        // write-back so the maps and overrides stay coherent.
        update_filament_maps_to_config(derived_maps);
        return;
    }

    int extruder_count = 1, extruder_volume_type_count = 1;
    m_ori_full_print_config.support_different_extruders(extruder_count);
    std::vector<std::vector<NozzleVolumeType>> nozzle_volume_types;
    extruder_volume_type_count = m_ori_full_print_config.get_extruder_nozzle_volume_count(extruder_count, nozzle_volume_types);

    // Note: filament_map_2 keeps its apply-time (static) derivation here; the per-slot machine
    // indices below key the override merge instead, so nothing on this path reads it. Its other
    // consumers are the three-map write-back (which recomputes it) and the diagnostic copy in
    // the g-code header; the time estimator resolves per-(extruder x volume-type) machine limits
    // from the live nozzle occupancy instead (see GCodeProcessor::get_machine_config_idx).
    m_full_print_config = m_ori_full_print_config;
    std::set<std::string> filament_keys = filament_options_with_variant;
    filament_keys.insert("filament_self_index");
    std::vector<int> slot_machine_indices;
    m_full_print_config.update_filament_config_values_for_multiple_extruders(m_full_print_config, filament_variant_uses,
                                                                             extruder_count, extruder_volume_type_count,
                                                                             filament_keys, "filament_self_index", "filament_extruder_variant",
                                                                             &slot_machine_indices);

    const std::vector<std::string> &extruder_retract_keys = print_config_def.extruder_retract_keys();
    const std::string               filament_prefix       = "filament_";
    t_config_option_keys            print_diff;
    DynamicPrintConfig              filament_overrides;
    for (auto& opt_key: extruder_retract_keys)
    {
        const ConfigOption *opt_new_filament = m_full_print_config.option(filament_prefix + opt_key);
        const ConfigOption *opt_new_machine = m_full_print_config.option(opt_key);
        const ConfigOption *opt_old_machine = m_config.option(opt_key);

        if (opt_new_filament)
            compute_filament_override_value(opt_key, opt_old_machine, opt_new_machine, opt_new_filament, m_full_print_config, print_diff, filament_overrides, slot_machine_indices);
    }

    {
        t_config_option_keys keys(filament_options_with_variant.begin(), filament_options_with_variant.end());
        keys.push_back("filament_self_index");
        m_config.apply_only(m_full_print_config, keys, true);
    }
    if (!print_diff.empty()) {
        m_placeholder_parser.apply_config(filament_overrides);
        m_config.apply(filament_overrides);
    }

    update_filament_self_index_cache();
    m_has_auto_filament_map_result = true;
}

void Print::apply_config_for_render(const DynamicConfig &config)
{
    m_config.apply(config);
}

std::optional<RegionalBandToolMismatch> Print::find_regional_band_tool_mismatch(const PrintConfig &config) const
{
    for (const PrintObject *object : m_objects) {
        // A plain object on a Body Split plate has no native grid and so no committed binding to drift
        // from. An object that holds a grid is always checked, even if the map under test would now call
        // it plain.
        if (object->region_grids().empty() && ! is_body_split_object(config, *object->model_object()))
            continue;
        const auto &grids = object->region_grids();
        const auto &bindings = object->region_grid_physical_extruders();
        const size_t region_count = object->num_printing_regions();
        if (grids.size() != region_count || bindings.size() != region_count)
            return RegionalBandToolMismatch{object, 0, unsigned(-1), -1};
        for (size_t region_id = 0; region_id < region_count; ++region_id) {
            // Skip regions with no committed band. PrintApply.cpp creates a painted-sibling region, real or
            // ghost, for every parent volume and paint colour. A ghost never plans a band and its binding
            // mirrors the parent's tool, so comparing its own outer_wall_filament_id (the paint colour's
            // filament) against that fails spuriously when both filaments resolve to the same tool. Every
            // real body region is non-empty once slicing succeeds.
            if (grids[region_id].empty())
                continue;
            const int logical_filament = object->printing_region(region_id).config().outer_wall_filament_id.value;
            const std::optional<size_t> resolved = logical_filament > 0 ?
                physical_extruder_for_filament(config, unsigned(logical_filament - 1)) : std::nullopt;
            if (!resolved || *resolved != bindings[region_id])
                return RegionalBandToolMismatch{object, 0, unsigned(bindings[region_id]),
                                                resolved ? int(*resolved) : -1};
        }
    }
    return std::nullopt;
}

RegionalLayerPlanExport Print::regional_layer_plan_export() const
{
    // An ordinary print has no plan, and its stream must not carry a block that says it does.
    if (! is_mixed_nozzle_slicing_enabled(m_config))
        return {};

    // One code for every way the plan can fail to be publishable. Each carries its own sentence,
    // because what the user has to change differs; what they share is that nothing is written.
    const auto refuse = [this](const std::string &message) {
        const std::string named = "[SRL-C02] " + message;
        return m_objects.empty() ? Slic3r::SlicingError(named) :
                                   Slic3r::SlicingError(named, m_objects.front()->model_object()->id().id);
    };

    // Each plan block is scoped by its own SRL_PLAN_START...SRL_PLAN_END pair, and region ids restart
    // at 0 inside it, so Body Split publishes one complete block per object, and so does
    // Feature Split (below).
    if (is_mixed_nozzle_body_split(m_config)) {
        if (m_objects.empty())
            throw refuse("Exporting the layer plan needs at least one object on the plate.");

        const int toolchange_total    = this->tool_ordering().mixed_nozzle_toolchange_total();
        const int toolchange_per_band = this->tool_ordering().mixed_nozzle_toolchange_per_band();

        std::string block;
        size_t total_grids = 0, total_cells = 0, total_rendezvous = 0, planned_objects = 0;
        for (const PrintObject *object_ptr : m_objects) {
            // A plain object on a Body Split plate is sliced as with the mode off and publishes no block.
            if (! is_body_split_object(m_config, *object_ptr->model_object()))
                continue;
            ++ planned_objects;
            const PrintObject &object = *object_ptr;
            const auto &state = object.native_regional_grid_state();
            const auto &bindings = object.region_grid_physical_extruders();
            if (object.region_grids().size() != object.num_printing_regions() ||
                state.cells.size() != object.region_grids().size() || bindings.size() != state.cells.size())
                throw refuse("The Body Split layer plan is incomplete, so no plan block was written.");

            const coordf_t base_cadence = object.config().layer_height.value;
            std::vector<RegionalGridPlanRecord> grids;
            grids.reserve(state.cells.size());
            for (size_t region_id = 0; region_id < state.cells.size(); ++region_id) {
                // A painted-sibling region, real or ghost, never enters `precedence` in the native-grid build
                // and has no grid entry. Skip it: an empty RegionalGridPlanRecord would make
                // format_regional_layer_plan() reject the plan as GridNotContiguous.
                if (state.cells[region_id].empty())
                    continue;
                if (bindings[region_id] >= m_config.nozzle_diameter.size())
                    throw refuse("A body is not assigned to a nozzle, so no plan block was written.");
                const PrintRegionConfig &region = object.printing_region(region_id).config();
                coordf_t cadence = region.regional_layer_height.value == 0. ?
                    base_cadence : region.regional_layer_height.value;
                // A painted region copies its parent body's config, so its regional_layer_height is
                // the parent's. Its cells follow the cadence the native projection resolved for the
                // painted colour's own tool (coarse paint on a fine body prints coarse cells), so
                // state that one. Body regions keep the configured value unchanged.
                bool body_region = false;
                for (const PrintObjectRegions::LayerRangeRegions &layer_range : object.shared_regions()->layer_ranges)
                    for (const PrintObjectRegions::VolumeRegion &volume_region : layer_range.volume_regions)
                        if (volume_region.region != nullptr && volume_region.model_volume != nullptr &&
                            volume_region.model_volume->is_model_part() &&
                            size_t(volume_region.region->print_object_region_id()) == region_id)
                            body_region = true;
                if (!body_region && region_id < object.region_slicing_parameters().size() &&
                    object.region_slicing_parameters()[region_id].layer_height > 0.)
                    cadence = object.region_slicing_parameters()[region_id].layer_height;
                std::vector<RegionCell> cells;
                cells.reserve(state.cells[region_id].size());
                // The planner flags the cells the skin window converted (RegionCell::fine_skin). A height
                // mismatch is not a safe signal: the shared first layer and a body whose height is not a
                // whole multiple of its cadence both differ in height without being fine-skin cells.
                for (const NativeRegionCellState &native_cell : state.cells[region_id]) {
                    RegionCell cell = native_cell.cell;
                    cell.physical_extruder_id = unsigned(bindings[region_id]);
                    // Two regions: the other region's binding. With more regions, the tool the cell's stamped
                    // filament resolves to, which PrintObjectSlice.cpp set to the fine body's filament.
                    if (cell.fine_skin && object.num_printing_regions() == 2) {
                        const size_t other_region_id = 1 - region_id;
                        if (other_region_id < bindings.size())
                            cell.physical_extruder_id = unsigned(bindings[other_region_id]);
                    } else if (cell.fine_skin) {
                        const size_t cell_index = size_t(&native_cell - state.cells[region_id].data());
                        const auto &grid = object.region_grids()[region_id];
                        const int stamped = cell_index < grid.size() && grid[cell_index] != nullptr ?
                            grid[cell_index]->cell_filament_id() : 0;
                        const MixedNozzleToolResolution stamped_tool = stamped > 0 ?
                            resolve_mixed_nozzle_tool(m_config, size_t(stamped - 1), MixedNozzleResolveScope::PhysicalToolOnly) :
                            MixedNozzleToolResolution{};
                        if (!stamped_tool)
                            throw refuse("A fine-skin cell has no resolvable fine tool, so no plan block was written.");
                        cell.physical_extruder_id = unsigned(stamped_tool.tool->physical_extruder);
                    }
                    cells.push_back(cell);
                }
                const RegionalGridPhaseRule phase = region_id < state.region_plans.size() ?
                    state.region_plans[region_id].phase_rule : m_config.regional_grid_phase_rule.value;
                grids.push_back({region_id, unsigned(bindings[region_id]),
                                 m_config.nozzle_diameter.get_at(bindings[region_id]), cadence, phase,
                                 std::move(cells)});
            }

            // A supported or rafted object's plan block also declares the physical tools and heights support
            // base and interface resolved to, so the consumer can verify support like the object bands. Raft
            // reuses the support filament keys, so one record covers both.
            std::optional<RegionalSupportPlanRecord> support_record;
            if (object.has_support() || object.has_raft()) {
                const MixedNozzleToolResolution base_tool = resolve_mixed_nozzle_tool(
                    m_config, size_t(object.config().support_filament.value - 1), MixedNozzleResolveScope::PhysicalToolOnly);
                const MixedNozzleToolResolution interface_tool = resolve_mixed_nozzle_tool(
                    m_config, size_t(object.config().support_interface_filament.value - 1), MixedNozzleResolveScope::PhysicalToolOnly);
                if (!base_tool || !interface_tool)
                    throw refuse("A supported object's support filaments are not assigned to a nozzle, so no plan block was written.");
                support_record = RegionalSupportPlanRecord{
                    unsigned(base_tool.tool->physical_extruder), object.slicing_parameters().max_suport_layer_height,
                    unsigned(interface_tool.tool->physical_extruder), object.config().layer_height.value};
            }

            // The plate-wide tool change totals are passed to every object's call. The plan format has no
            // plate-level container, so the SRL_TOOLCHANGES line repeats identically in every block.
            const RegionalLayerPlanText object_plan = format_regional_layer_plan(
                grids, object.rendezvous_planes(), state.event_planes,
                m_config.regional_interface_tolerance.value, base_cadence, toolchange_total, toolchange_per_band,
                support_record);
            if (object_plan.error != RegionalPlanFormatError::None || object_plan.text.empty())
                throw refuse(Slic3r::format("The Body Split layer plan is not internally consistent (plan integrity code %1%), so no plan block was written.",
                                            int(object_plan.error)));

            block += object_plan.text;
            total_grids += grids.size();
            for (const RegionalGridPlanRecord &grid : grids)
                total_cells += grid.cells.size();
            total_rendezvous += object.rendezvous_planes().size();
        }

        const std::string native_summary =
            "SRL plan: objects=" + std::to_string(planned_objects) +
            " grids=" + std::to_string(total_grids) +
            " cells=" + std::to_string(total_cells) +
            " rendezvous=" + std::to_string(total_rendezvous) +
            " tolerance=" + float_to_string_decimal_point(m_config.regional_interface_tolerance.value, 8) +
            " phase=" + std::string(m_config.regional_grid_phase_rule.value == RegionalGridPhaseRule::KeepNominal ? "KEEP" : "REPHASE");
        return {block, native_summary};
    }

    // A Feature Split record names no object, so each object publishes its own
    // SRL_PLAN_START...SRL_PLAN_END block, in object order, as Body Split does.
    if (m_objects.empty())
        throw refuse("Exporting the layer plan needs at least one object on the plate.");

    std::string block;
    size_t      total_regions = 0, total_bands = 0;
    for (const PrintObject *object_ptr : m_objects) {
        const PrintObject &object = *object_ptr;

        // Feature Split: a committed band's top fine layer carries the marker combine_infill leaves in
        // fill_surfaces, an stInternal surface thicker than one fine layer with thickness_layers = N.
        // This reads what process() already committed and never mutates slicing state. Feature Split
        // refuses raft layers, so the layers() index is the plain fine-layer index.
        const coordf_t base_cadence = object.config().layer_height.value;
        std::vector<RegionalGridPlanRecord> bands;
        bands.reserve(object.num_printing_regions());
        for (size_t region_id = 0; region_id < object.num_printing_regions(); ++region_id) {
            std::vector<RegionCell> band_cells;
            for (size_t layer_idx = 0; layer_idx < object.layers().size(); ++layer_idx) {
                const Layer *layer = object.layers()[layer_idx];
                for (const Surface &surface : layer->regions()[region_id]->fill_surfaces.surfaces) {
                    if (surface.surface_type == stInternal && surface.thickness > layer->height + EPSILON) {
                        const size_t band_size = size_t(surface.thickness_layers);
                        if (band_size > layer_idx + 1)
                            throw refuse("A Feature Split coarse band ends before its window begins, so no plan block was written.");
                        const size_t first_layer_idx = layer_idx + 1 - band_size;
                        const coordf_t bottom_z = object.layers()[first_layer_idx]->bottom_z();
                        const coordf_t top_z    = layer->print_z;
                        band_cells.push_back({region_id, band_cells.size(), first_layer_idx, layer_idx,
                                              bottom_z, top_z, 0.5 * (bottom_z + top_z), surface.thickness});
                        break;
                    }
                }
            }
            if (band_cells.empty())
                continue;

            const PrintRegionConfig &region_config = object.printing_region(region_id).config();
            const int fine_owner   = region_config.outer_wall_filament_id.value;
            const int sparse_owner = region_config.sparse_infill_filament_id.value;
            if (fine_owner <= 0 || sparse_owner <= 0)
                throw refuse("A Feature Split region planned a coarse layer but has no fine and coarse filaments set, so no plan block was written.");
            const MixedNozzleCadenceResolution resolved = resolve_mixed_nozzle_cadence(
                m_config, base_cadence, m_config.mixed_nozzle_coarse_layer_height.value,
                size_t(fine_owner - 1), size_t(sparse_owner - 1));
            if (! resolved)
                throw refuse("A Feature Split region planned a coarse layer but its coarse layer height no longer fits its nozzles, so no plan block was written.");
            const MixedNozzleResolvedCadence &cadence = *resolved.cadence;

            bands.push_back({region_id, unsigned(cadence.coarse_tool.physical_extruder),
                             cadence.coarse_tool.nozzle_diameter, cadence.coarse_height,
                             RegionalGridPhaseRule::KeepNominal, std::move(band_cells),
                             RegionalPlanRecordMode::Feature});
        }
        // A Feature Split print with no sparse infill commits zero bands. That is a valid plan, and the
        // formatter renders a zero-band block.
        const RegionalLayerPlanText feature_plan = format_regional_layer_plan_feature(bands);
        if (feature_plan.error != RegionalPlanFormatError::None || feature_plan.text.empty())
            throw refuse(Slic3r::format("The Feature Split layer plan is not internally consistent (plan integrity code %1%), so no plan block was written.",
                                        int(feature_plan.error)));

        block += feature_plan.text;
        total_regions += bands.size();
        for (const RegionalGridPlanRecord &group : bands)
            total_bands += group.cells.size();
    }

    const std::string feature_summary =
        std::string("SRL plan: mode=feature") +
        (m_objects.size() > 1 ? " objects=" + std::to_string(m_objects.size()) : std::string()) +
        " regions=" + std::to_string(total_regions) +
        " bands=" + std::to_string(total_bands);
    return {block, feature_summary};
}

std::vector<int> Print::get_filament_maps() const
{
    return m_config.filament_map.values;
}

std::vector<int> Print::get_filament_nozzle_maps() const
{
    return m_config.filament_nozzle_map.values;
}

std::vector<int> Print::get_filament_volume_maps() const
{
    return m_config.filament_volume_map.values;
}

FilamentMapMode Print::get_filament_map_mode() const
{
    return m_config.filament_map_mode;
}

std::vector<std::set<int>> Print::get_physical_unprintable_filaments(const std::vector<unsigned int>& used_filaments) const
{
    int extruder_num = m_config.nozzle_diameter.size();
    std::vector<std::set<int>>physical_unprintables(extruder_num);
    if (extruder_num < 2)
        return physical_unprintables;

    auto get_unprintable_extruder_id = [&](unsigned int filament_idx) -> int {
        // filament_printable may be shorter than the filament count; get_at() clamps.
        int status = m_config.filament_printable.get_at(filament_idx);
        for (int i = 0; i < extruder_num; ++i) {
            if (!(status >> i & 1)) {
                return i;
            }
        }
        return -1;
    };


    std::set<int> tpu_filaments;
    for (auto f : used_filaments) {
        if (m_config.filament_type.get_at(f) == "TPU")
            tpu_filaments.insert(f);
    }

    for (auto f : used_filaments) {
        int extruder_id = get_unprintable_extruder_id(f);
        if (extruder_id == -1)
            continue;
        physical_unprintables[extruder_id].insert(f);
    }

    return physical_unprintables;
}

std::map<int, std::set<NozzleVolumeType>> Print::get_filament_unprintable_flow(const std::vector<unsigned int> &used_filaments) const
{
    std::map<int, std::set<NozzleVolumeType>> ret;
    std::vector<std::string> extruder_variant_list = m_config.printer_extruder_variant.values;
    // A filament that declares no extruder variants carries no flow restriction.
    const ConfigOptionStrings *filament_variant_opt = m_ori_full_print_config.option<ConfigOptionStrings>("filament_extruder_variant");
    if (filament_variant_opt == nullptr)
        return ret;
    std::vector<std::string> filament_variant_list = filament_variant_opt->values;
    std::vector<int> filament_self_index;
    if (!m_ori_full_print_config.has("filament_self_index"))
        filament_self_index.resize(filament_variant_list.size(), 1);
    else
        filament_self_index = m_ori_full_print_config.option<ConfigOptionInts>("filament_self_index")->values;
    std::unordered_set<int> used_fils_set(used_filaments.begin(), used_filaments.end());

    std::unordered_map<int, std::set<NozzleVolumeType>> filament_variant_map;
    for(int i = 0; i < filament_variant_list.size(); ++i){
        NozzleVolumeType volume = convert_to_nvt_type(filament_variant_list[i]);
        if(volume != nvtHybrid) filament_variant_map[filament_self_index[i]].insert(volume);
    }

    for (auto iter : filament_variant_map) {
        int fil_idx = iter.first - 1;
        if (used_fils_set.find(fil_idx) == used_fils_set.end()) continue;
        const std::set<NozzleVolumeType> &volumes = iter.second;
        for (int exd_idx = 0; exd_idx < extruder_variant_list.size(); ++exd_idx) {
            auto exd_volume = convert_to_nvt_type(extruder_variant_list[exd_idx]);
            assert(exd_volume != nvtHybrid);
            if (volumes.find(exd_volume) == volumes.end() && exd_volume != nvtHybrid) ret[fil_idx].insert(exd_volume);
        }
    }
    return ret;
}


std::vector<double> Print::get_extruder_printable_height() const
{
    return m_config.extruder_printable_height.values;
}

std::vector<Polygons> Print::get_extruder_printable_polygons() const
{
    std::vector<Polygons>           extruder_printable_polys;
    std::vector<std::vector<Vec2d>> extruder_printable_areas = m_config.extruder_printable_area.values;
    for (const auto &e_printable_area : extruder_printable_areas) {
        Polygons ploys = {Polygon::new_scale(e_printable_area)};
        extruder_printable_polys.emplace_back(ploys);
    }
    return std::move(extruder_printable_polys);
}

std::vector<Polygons> Print::get_extruder_unprintable_polygons() const
{
    std::vector<Vec2d>              printable_area           = m_config.printable_area.values;
    Polygon                         printable_poly           = Polygon::new_scale(printable_area);
    std::vector<std::vector<Vec2d>> extruder_printable_areas = m_config.extruder_printable_area.values;
    std::vector<Polygons>           extruder_unprintable_polys;
    for (const auto &e_printable_area : extruder_printable_areas) {
        Polygons ploys = diff(printable_poly, Polygon::new_scale(e_printable_area));
        extruder_unprintable_polys.emplace_back(ploys);
    }
    return std::move(extruder_unprintable_polys);
}

size_t Print::get_extruder_id(unsigned int filament_id) const
{
    std::vector<int> filament_map = get_filament_maps();
    size_t result = filament_id < filament_map.size() ? size_t(filament_map[filament_id] - 1) : 0;
#ifndef NDEBUG
    // Debug cross-check: resolve_mixed_nozzle_tool reads config.filament_map, and this function reads
    // the same map via get_filament_maps(). Compiles out under NDEBUG; never changes the result.
    if (is_mixed_nozzle_slicing_enabled(m_config) && filament_id < filament_map.size()) {
        const MixedNozzleToolResolution resolved = resolve_mixed_nozzle_tool(m_config, size_t(filament_id), MixedNozzleResolveScope::PhysicalToolOnly);
        assert(!resolved || resolved.tool->physical_extruder == result);
    }
#endif
    return result;
}

// Region reachable by every extruder = intersection of all per-extruder printable areas.
// For single-nozzle printers, or whenever extruder_printable_area is unpopulated / degenerate (all
// current single/dual profiles), fall back to the full printable_area so the wipe-tower-center clamp
// is identical to the previous full-bed clamp.
Polygons Print::get_extruder_shared_printable_polygon() const
{
    const std::vector<Vec2ds>& extruder_printable_areas = m_config.extruder_printable_area.values;
    if (m_config.nozzle_diameter.size() < 2 || extruder_printable_areas.empty())
        return {Polygon::new_scale(m_config.printable_area.values)};
    for (const Vec2ds& area : extruder_printable_areas)
        if (area.size() < 3)
            return {Polygon::new_scale(m_config.printable_area.values)};

    Polygons shared_printable_polys = {Polygon::new_scale(extruder_printable_areas.front())};
    for (size_t i = 1; i < extruder_printable_areas.size(); ++i)
        shared_printable_polys = intersection(shared_printable_polys, Polygons{Polygon::new_scale(extruder_printable_areas[i])});
    return shared_printable_polys;
}

// Narrow the stored grouping result to the layer-aware type the slicing pipeline uses.
std::shared_ptr<MultiNozzleUtils::LayeredNozzleGroupResult> Print::get_layered_nozzle_group_result() const
{
    return std::dynamic_pointer_cast<MultiNozzleUtils::LayeredNozzleGroupResult>(m_nozzle_group_result);
}

// Dynamic (per-layer selector) regroup predicate.
// Orca: enable_filament_dynamic_map is a project flag registered in the ConfigDef but NOT a static
// PrintConfig member, so it is read from the applied full config. No profile sets it; it is turned
// on per project by the "smart filament assign" checkbox (shown when a filament track switch is
// ready), so absent-key -> nullptr -> false keeps the static grouping path (identical output) for
// everything else. There is no mixed-colour-filament guard (mixed-colour filaments are not
// supported). The remaining gates (auto-for-flush mode, multi-extruder machine) read the static
// PrintConfig members.
bool Print::is_dynamic_group_reorder() const
{
    const auto *opt     = m_full_print_config.option<ConfigOptionBool>("enable_filament_dynamic_map");
    const bool  enabled = opt && opt->value;
    if (!enabled || m_config.filament_map_mode != FilamentMapMode::fmmAutoForFlush || m_config.nozzle_diameter.size() <= 1)
        return false;
    return true;
}

int Print::get_filament_config_indx(int filament_id, int layer_id)
{
    int index = get_config_index(filament_id, layer_id, m_config.filament_extruder_variant.values, m_filament_self_index, m_filament_index_map);
#ifndef NDEBUG
    // Debug cross-check: the layer-aware resolver and the static config-only resolver must agree for
    // every Mixed-Nozzle print, since the per-layer dynamic path is refused and every layer resolves
    // the same (extruder_id, volume_type). They are computed independently. Compiles out under
    // NDEBUG; never changes the result.
    if (is_mixed_nozzle_slicing_enabled(m_config) && filament_id >= 0 && size_t(filament_id) < m_config.filament_map.size()) {
        const MixedNozzleToolResolution resolved = resolve_mixed_nozzle_tool(m_config, size_t(filament_id), MixedNozzleResolveScope::PhysicalToolOnly);
        assert(!resolved || resolved.tool->variant_column == index);
    }
#endif
    return index;
}

void Print::update_filament_self_index_cache()
{
    m_missing_nozzle_group_logged.clear();   // reset the per-slice get_config_index log dedupe

    std::vector<int> values;
    if (m_full_print_config.has("filament_self_index")) {
        values = m_full_print_config.option<ConfigOptionInts>("filament_self_index")->values;
    } else if (m_ori_full_print_config.has("filament_self_index")) {
        values = m_ori_full_print_config.option<ConfigOptionInts>("filament_self_index")->values;
    } else {
        values = m_config.filament_self_index.values;
    }

    size_t expected_size = m_config.filament_extruder_variant.values.size();
    m_filament_self_index.clear();
    if (expected_size == 0) {
        m_filament_index_map.clear();
        m_nozzle_index_map.clear();
        return;
    }
    m_filament_self_index.resize(expected_size, 1);
    if (!values.empty()) {
        for (size_t i = 0; i < expected_size; ++i) {
            int v = i < values.size() ? values[i] : 1;
            if (v <= 0)
                v = 1;
            m_filament_self_index[i] = v;
        }
    }
    m_filament_index_map.clear();
    m_nozzle_index_map.clear();
}

int Print::get_nozzle_config_index(int filament_id, int layer_id)
{
    // Orca: print_extruder_id/print_extruder_variant are PrintRegionConfig members in this codebase;
    // the process-wide expanded values live in the default region config (regions never override them).
    return get_config_index(filament_id, layer_id, m_default_region_config.print_extruder_variant.values, m_default_region_config.print_extruder_id.values, m_nozzle_index_map);
}

int Print::get_config_index(int filament_id, int layer_id, const std::vector<std::string> &variant_list, const std::vector<int>& self_index_list, FilamentIndexMap &index_map)
{
    auto group_result = get_layered_nozzle_group_result();
    // Orca: defensive — when no grouping producer has published a result yet, fall back to the
    // static identity: one filament-variant column per filament.
    if (!group_result)
        return filament_id;
    auto nozzle_info  = group_result->get_nozzle_for_filament(filament_id, layer_id);
    if (!nozzle_info.has_value()) {
        // Orca: this fallback runs per-filament/per-layer in the g-code hot path — log once per filament
        // (reset each slice) instead of flooding thousands of identical lines that bury the real error.
        if (m_missing_nozzle_group_logged.insert(filament_id).second)
            BOOST_LOG_TRIVIAL(error) << __FUNCTION__
                                     << boost::format(", Line %1%: could not found group_nozzle_info corresponding to filament_id %2%, layer_id %3% (further occurrences for this filament suppressed)") % __LINE__ % filament_id %
                                            layer_id;
        return 0;
    }

    ExtruderType     extruder_type      = ExtruderType(m_config.extruder_type.get_at(nozzle_info->extruder_id));
    NozzleVolumeType nozzle_volume_type = nozzle_info->volume_type;

    FilamentIndexKey key{filament_id, extruder_type, nozzle_volume_type};
    auto             iter = index_map.find(key);
    if (iter == index_map.end()) {
        int index = get_config_index_base(nozzle_volume_type, extruder_type, filament_id + 1, variant_list, self_index_list);
        index_map[key] = index;
        return index;
    } else {
        return index_map[key];
    }
}

int Print::get_config_index(int filament_id, int layer_id, const std::vector<std::string> &variant_list, const std::vector<int>& self_index_list, PrintIndexMap &index_map)
{
    auto group_result = get_layered_nozzle_group_result();
    // Orca: same static fallback as the filament overload; the slot degenerates to the filament's
    // extruder column (filament_map is 1 based, get_extruder_id guards the filament id range).
    if (!group_result)
        return (int)get_extruder_id(filament_id);
    auto nozzle_info  = group_result->get_nozzle_for_filament(filament_id, layer_id);
    if (!nozzle_info.has_value()) {
        // Orca: this fallback runs per-filament/per-layer in the g-code hot path — log once per filament
        // (reset each slice) instead of flooding thousands of identical lines that bury the real error.
        if (m_missing_nozzle_group_logged.insert(filament_id).second)
            BOOST_LOG_TRIVIAL(error) << __FUNCTION__
                                     << boost::format(", Line %1%: could not found group_nozzle_info corresponding to filament_id %2%, layer_id %3% (further occurrences for this filament suppressed)") % __LINE__ % filament_id %
                                            layer_id;
        return 0;
    }

    int              extruder_id        = nozzle_info->extruder_id + 1; // to 1 based
    ExtruderType     extruder_type      = ExtruderType(m_config.extruder_type.get_at(nozzle_info->extruder_id));
    NozzleVolumeType nozzle_volume_type = nozzle_info->volume_type;

    PrintIndexKey key{filament_id, extruder_id, extruder_type, nozzle_volume_type};
    auto          iter = index_map.find(key);
    if (iter == index_map.end()) {
        int index = get_config_index_base(nozzle_volume_type, extruder_type, extruder_id, variant_list, self_index_list);
        index_map[key] = index;
        return index;
    } else {
        return index_map[key];
    }
}

// Wipe tower support.
bool Print::has_wipe_tower() const
{
    if (m_config.enable_prime_tower.value == true) {
        if (m_config.enable_wrapping_detection.value && m_config.wrapping_exclude_area.values.size() > 2)
            return true;

        if (enable_timelapse_print())
            return true;

        return !m_config.spiral_mode.value && m_config.filament_diameter.values.size() > 1;
    }
    return false;
}

const WipeTowerData &Print::wipe_tower_data(size_t filaments_cnt) const
{
    // If the wipe tower wasn't created yet, make sure the depth and brim_width members are set to default.
    double max_height = 0;
    for (size_t obj_idx = 0; obj_idx < m_objects.size(); obj_idx++) {
        double object_z = (double) m_objects[obj_idx]->size().z();
        max_height      = std::max(unscale_(object_z), max_height);
    }
    if (max_height < EPSILON) return m_wipe_tower_data;

    double layer_height                  = 0.08f; // hard code layer height
    layer_height        = m_objects.front()->config().layer_height.value;

    auto   timelapse_type  = config().option<ConfigOptionEnum<TimelapseType>>("timelapse_type");
    bool   need_wipe_tower = (timelapse_type ? (timelapse_type->value == TimelapseType::tlSmooth) : false) | (m_config.wipe_tower_wall_type.value == WipeTowerWallType::wtwRib);
    double extra_spacing = config().option("prime_tower_infill_gap")->getFloat() / 100.;
    double rib_width     = config().option("wipe_tower_rib_width")->getFloat();

    double filament_change_volume = 0.;
    {
        std::vector<double> filament_change_lengths;
        auto                filament_change_lengths_opt = config().option<ConfigOptionFloats>("filament_change_length");
        if (filament_change_lengths_opt) filament_change_lengths = filament_change_lengths_opt->values;
        double              length   = filament_change_lengths.empty() ? 0 : *std::max_element(filament_change_lengths.begin(), filament_change_lengths.end());
        double              diameter = 1.75;
        std::vector<double> diameters;
        auto                filament_diameter_opt = config().option<ConfigOptionFloats>("filament_diameter");
        if (filament_diameter_opt) diameters = filament_diameter_opt->values;
        diameter               = diameters.empty() ? diameter : *std::max_element(diameters.begin(), diameters.end());
        filament_change_volume = length * PI * diameter * diameter / 4.;
    }


    if (! is_step_done(psWipeTower) && filaments_cnt !=0) {
        double wipe_volume  = m_config.prime_volume;
        int filament_depth_count = m_config.nozzle_diameter.values.size() == 2 ? filaments_cnt : filaments_cnt - 1;
        if (filaments_cnt == 1 && enable_timelapse_print()) filament_depth_count = 1;
        double volume = wipe_volume * filament_depth_count;
        if (m_config.nozzle_diameter.values.size() == 2) volume += filament_change_volume * (int) (filaments_cnt / 2);

        // Sizing should take into account currently set wiping volumes.
        // For a long time, the initial preview would just use 900/width per toolchange (15mm on a 60mm wide tower)
        // and it worked well enough. Let's try to do slightly better by accounting for the purging volumes.
        const bool semm_flush = m_config.purge_in_prime_tower && m_config.single_extruder_multi_material;
        if (semm_flush) volume = WipeTower2::estimate_semm_flush_volume(m_config, filaments_cnt);

        if (m_config.wipe_tower_wall_type.value == WipeTowerWallType::wtwRib) {
            double depth = std::sqrt(volume / layer_height * extra_spacing);
            if (need_wipe_tower || filaments_cnt > 1) {
                float min_wipe_tower_depth = WipeTower::get_limit_depth_by_height(max_height);
                depth  = std::max((double) min_wipe_tower_depth, depth);
                depth += rib_width / std::sqrt(2) + config().wipe_tower_extra_rib_length.value;
                const_cast<Print *>(this)->m_wipe_tower_data.depth = depth;
                const_cast<Print *>(this)->m_wipe_tower_data.brim_width = m_config.prime_tower_brim_width;
            }
        }
        else {
            double width = m_config.prime_tower_width;
            double depth = volume / (layer_height * width);
            // The flush volumes already hold the spacing between wipes.
            if (!semm_flush) depth *= extra_spacing;
            if (need_wipe_tower || depth > EPSILON) {
                float min_wipe_tower_depth = WipeTower::get_limit_depth_by_height(max_height);
                depth = std::max((double) min_wipe_tower_depth, depth);
            }
            const_cast<Print *>(this)->m_wipe_tower_data.depth = depth;
            const_cast<Print *>(this)->m_wipe_tower_data.brim_width = m_config.prime_tower_brim_width;
        }
        // Auto means something different under a mixed nozzle. Orca's rule is linear in height and gives
        // a tall tower a brim too narrow to hold it, so a mixed-nozzle tower resolves Auto through the
        // height tiers instead.
        if (m_config.prime_tower_brim_width < 0)
            const_cast<Print *>(this)->m_wipe_tower_data.brim_width =
                is_mixed_nozzle_slicing_enabled(m_config)
                    ? mixed_nozzle_tower_brim_width(float(max_height))
                    : WipeTower::get_auto_brim_by_height(float(max_height));
    }
    return m_wipe_tower_data;
}

bool Print::enable_timelapse_print() const
{
    return m_config.timelapse_type.value == TimelapseType::tlSmooth;
}

// Volume of sparse infill in a fill collection, nested collections included.
static double sparse_infill_volume(const ExtrusionEntity &entity)
{
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
        double volume = 0.;
        for (const ExtrusionEntity *child : collection->entities)
            volume += sparse_infill_volume(*child);
        return volume;
    }
    return entity.role() == erInternalInfill ? entity.total_volume() : 0.;
}

void Print::select_feature_infill_by_time()
{
    m_feature_economics_trials = 0;
    m_feature_economics_refilled_layers = 0;
    m_feature_economics_prepared_trials = 0;
    m_feature_economics_committed_tower_seconds = 0.;
    m_feature_economics_committed_tower_volume_mm3 = 0.;
    m_feature_economics_committed_tower_digest.clear();
    m_feature_economics_committed_tower_plan_digest.clear();
    m_feature_economics_whole_print_digest.clear();
    if (!is_mixed_nozzle_feature_split(m_config) || m_pipeline_plugin_active ||
        wipe_tower_type() != WipeTowerType::Type1 || m_config.print_sequence == PrintSequence::ByObject)
        return;

    // Compare generated native work. Tower candidates go through the same AutoPad and structural
    // checks as export; a saved payload makes a rejected trial reversible. Hand-off time is kept in
    // two terms so it can be reported separately; total_seconds() is the scalar the selector compares.
    struct HandoffCost {
        double tower_seconds { 0. };
        double switch_seconds { 0. };
        double tower_volume_mm3 { 0. };
        bool   available { false };
        double total_seconds() const { return tower_seconds + switch_seconds; }
    };
    // The committed plan's tower is generated once, before this pass. A band's fine alternative
    // differs only by the hand-off its coarse infill asks for, so its cost is the committed cost less
    // that band's switches and the tower rows they paid for. No tower is rebuilt per trial.
    struct CommittedRow {
        double                    print_z { 0. };
        std::vector<unsigned int> filaments;
        // Generated tower work at this layer, keyed by the filament each tool change entered.
        std::vector<std::pair<unsigned int, std::pair<double, double>>> tower_by_entered_filament;
        // Tower work at this layer that no band owns: the solid base and the levels between switches.
        // It is only built up to the last switch, and not at all when nothing switches.
        double loose_seconds { 0. };
    };
    std::vector<CommittedRow> committed_rows;
    // Per committed row, what a plain tower level costs there: the nearest row's loose work, as the
    // committed tower laid it. Filled once the tower is indexed.
    std::vector<double> plain_level_seconds;
    // Priming, the final purge and every row that is not a tool change stay in the total whatever a
    // band decides, so the total is carried whole and a band's own switches are subtracted from it.
    double committed_tower_seconds { 0. };
    double committed_tower_volume_mm3 { 0. };
    const NativeFilamentChangeTiming native_timing {
        float(m_config.machine_load_filament_time.value),
        float(m_config.machine_unload_filament_time.value),
        float(m_config.machine_tool_change_time.value)};
    // Tower rows are timed by the slicer's own estimator, not by the tower writer's length over feed
    // rate, which leaves out acceleration, corners and Z moves. A row the estimator cannot time keeps
    // the writer's figure.
    const auto group_result = get_layered_nozzle_group_result();
    SnippetTimer snippet_timer(m_config, group_result);
    const auto row_seconds = [&](const WipeTower::ToolChangeResult &change) {
        const double z = double(change.tower_z_start > 0.f ? change.tower_z_start : change.print_z);
        double seconds = 0.;
        if (!change.gcode.empty())
            seconds += snippet_timer.seconds(change.gcode, change.start_pos.x(), change.start_pos.y(), z);
        // GCode lays a nozzle change's ramming before the change G-code, on a two-nozzle printer only.
        if (!change.nozzle_change_result.gcode.empty() && m_config.nozzle_diameter.size() > 1)
            seconds += snippet_timer.seconds(change.nozzle_change_result.gcode,
                change.nozzle_change_result.start_pos.x(), change.nozzle_change_result.start_pos.y(), z);
        return std::isfinite(seconds) ? seconds : double(change.elapsed_time);
    };
    // Every row laid on the tower is a visit: the head retracts and lifts, travels from the part to
    // the tower, and comes back the same way with the tool it then holds. The centre of the objects
    // stands in for where it leaves from, and the centre of the tower for where it goes.
    Vec2d tower_centre(m_config.wipe_tower_x.get_at(m_plate_index), m_config.wipe_tower_y.get_at(m_plate_index));
    if (m_wipe_tower_data.bbx.defined)
        tower_centre += m_wipe_tower_data.bbx.center();
    else
        tower_centre += 0.5 * Vec2d(m_config.prime_tower_width.value, m_wipe_tower_data.depth);
    BoundingBoxf parts_box;
    for (const PrintObject *object : m_objects) {
        const BoundingBox box = object->bounding_box();
        for (const PrintInstance &instance : object->instances()) {
            const Vec2d shift = unscale(instance.shift_without_plate_offset());
            parts_box.merge(unscale(box.min) + shift);
            parts_box.merge(unscale(box.max) + shift);
        }
    }
    const auto positive_or = [](double value, double fallback) { return value > 0. ? value : fallback; };
    // The lift GCode makes before a travel with this filament, from height z: a spiral lift for the
    // spiral, slope and auto hop types, a straight one otherwise. heading is the direction of travel.
    const auto lift_gcode = [&](int filament, double z, const Vec2d &heading) {
        const int material = get_filament_config_indx(filament, 1);
        const int nozzle = get_nozzle_config_index(filament, 1);
        const double hop = m_config.z_hop.get_at(material);
        if (!(hop > 0.))
            return std::string();
        const double lift_speed = positive_or(m_config.travel_speed_z.get_at(nozzle), m_config.travel_speed.get_at(nozzle));
        const ZHopType type = ZHopType(m_config.z_hop_types.get_at(material));
        const double slope = m_config.travel_slope.get_at(size_t(get_extruder_id((unsigned int) filament))) * M_PI / 180.;
        const double norm = heading.norm();
        if ((type == ZHopType::zhtAuto || type == ZHopType::zhtSpiral || type == ZHopType::zhtSlope) &&
            slope > 0. && norm > 0.) {
            const double radius = hop / (2. * M_PI * std::atan(slope));
            const Vec2d centre = radius * heading / norm;
            return Slic3r::format("G17\nG3 Z%1% I%2% J%3% P1 F%4%\n", z + hop, -centre.y(), centre.x(), 60. * lift_speed);
        }
        return Slic3r::format("G1 Z%1% F%2%\n", z + hop, 60. * lift_speed);
    };
    // One way between the part and the tower with this filament loaded, as GCode writes it: retract,
    // lift, travel, drop back down and deretract.
    std::map<unsigned int, double> leg_cache;
    const auto leg_seconds = [&](int filament) {
        if (filament < 0)
            return 0.;
        if (const auto found = leg_cache.find((unsigned int) filament); found != leg_cache.end())
            return found->second;
        const int material = get_filament_config_indx(filament, 1);
        const int nozzle = get_nozzle_config_index(filament, 1);
        const double z = 1.;
        const double retract = m_config.retraction_length.get_at(material);
        const double retract_speed = m_config.retraction_speed.get_at(material);
        const double deretract_speed = positive_or(m_config.deretraction_speed.get_at(material), retract_speed);
        const double travel_speed = m_config.travel_speed.get_at(nozzle);
        const double lift_speed = positive_or(m_config.travel_speed_z.get_at(nozzle), travel_speed);
        const Vec2d from = parts_box.defined ? parts_box.center() : tower_centre;
        std::string gcode;
        if (retract > 0. && retract_speed > 0.)
            gcode += Slic3r::format("G1 E%1% F%2%\n", -retract, 60. * retract_speed);
        gcode += lift_gcode(filament, z, tower_centre - from);
        gcode += Slic3r::format("G1 X%1% Y%2% F%3%\n", tower_centre.x(), tower_centre.y(), 60. * travel_speed);
        gcode += Slic3r::format("G1 Z%1% F%2%\n", z, 60. * lift_speed);
        if (retract > 0. && deretract_speed > 0.)
            gcode += Slic3r::format("G1 E%1% F%2%\n", retract, 60. * deretract_speed);
        double seconds = snippet_timer.seconds(gcode, from.x(), from.y(), z);
        if (!std::isfinite(seconds))
            seconds = 0.;
        leg_cache.emplace((unsigned int) filament, seconds);
        return seconds;
    };
    const auto visit_seconds = [&](const WipeTower::ToolChangeResult &change) {
        if (change.gcode.empty() && change.nozzle_change_result.gcode.empty())
            return 0.;
        return leg_seconds(change.initial_tool) + leg_seconds(change.new_tool);
    };
    // The printer's change G-code runs on every switch. On the H2D it lifts Z, makes the trip to the
    // cutter and back and deretracts, and none of that is tower work. It is rendered once per pair of
    // filaments with the values GCode gives it, timed from the tower centre and back to it, and charged
    // per switch by price_walk. A body that does not render costs nothing, as before.
    std::map<std::pair<int, int>, double> change_cache;
    const auto change_seconds = [&](int from, int to) {
        if (from < 0 || to < 0 || from == to || m_config.change_filament_gcode.value.empty())
            return 0.;
        if (const auto found = change_cache.find({from, to}); found != change_cache.end())
            return found->second;
        double seconds = 0.;
        try {
            // The purge the tower rows for this pair carried, which sets the change G-code's flush.
            double purge_sum = 0.;
            size_t purge_rows = 0;
            for (const auto &generated : m_wipe_tower_data.tool_changes)
                for (const auto &change : generated)
                    if (change.is_tool_change && change.initial_tool == from && change.new_tool == to) {
                        purge_sum += double(change.purge_volume);
                        ++purge_rows;
                    }
            const double purge = purge_rows == 0 || purge_sum / purge_rows < EPSILON ? 0. :
                std::max(purge_sum / purge_rows, 100.); // GCode's smallest flush
            const int old_material = get_filament_config_indx(from, 1);
            const int new_material = get_filament_config_indx(to, 1);
            const int old_extruder = int(get_extruder_id((unsigned int) from));
            const int new_extruder = int(get_extruder_id((unsigned int) to));
            const int old_nozzle = group_result ? group_result->get_nozzle_id(from, 1) : old_extruder;
            const int new_nozzle = group_result ? group_result->get_nozzle_id(to, 1) : new_extruder;
            const double area = 0.25 * M_PI * std::pow(m_config.filament_diameter.get_at(size_t(to)), 2);
            const double flush_length = area > 0. ? purge / area : 0.;
            const auto e_feedrate = [&](int material) {
                const int feedrate = area > 0. ? int(60. * m_config.filament_max_volumetric_speed.get_at(material) / area) : 0;
                return feedrate == 0 ? 100 : feedrate;
            };
            const double z = 1.;
            DynamicConfig vars;
            vars.set_key_value("previous_extruder", new ConfigOptionInt(from));
            vars.set_key_value("next_extruder", new ConfigOptionInt(to));
            vars.set_key_value("current_extruder", new ConfigOptionInt(from));
            vars.set_key_value("current_extruder_id", new ConfigOptionInt(old_extruder));
            vars.set_key_value("current_hotend", new ConfigOptionInt(old_extruder));
            vars.set_key_value("next_hotend", new ConfigOptionInt(new_extruder));
            vars.set_key_value("current_nozzle_id", new ConfigOptionInt(old_nozzle));
            vars.set_key_value("next_nozzle_id", new ConfigOptionInt(new_nozzle));
            vars.set_key_value("current_filament_id", new ConfigOptionInt(from));
            vars.set_key_value("next_filament_id", new ConfigOptionInt(to));
            const auto &variants = m_config.printer_extruder_variant.values;
            vars.set_key_value("old_extruder_variant", new ConfigOptionString(
                old_extruder >= 0 && size_t(old_extruder) < variants.size() ? variants[old_extruder] : std::string()));
            vars.set_key_value("new_extruder_variant", new ConfigOptionString(
                new_extruder >= 0 && size_t(new_extruder) < variants.size() ? variants[new_extruder] : std::string()));
            std::vector<double> nozzle_diameters;
            std::vector<std::string> nozzle_volume_types;
            if (group_result)
                for (int id = 0;; ++id) {
                    const auto nozzle = group_result->get_nozzle_from_id(id);
                    if (!nozzle)
                        break;
                    nozzle_diameters.push_back(std::stod(nozzle->diameter));
                    nozzle_volume_types.push_back(get_nozzle_volume_type_string(nozzle->volume_type));
                }
            vars.set_key_value("nozzle_diameter_at_nozzle_id", new ConfigOptionFloats(nozzle_diameters));
            vars.set_key_value("nozzle_volume_types", new ConfigOptionStrings(nozzle_volume_types));
            vars.set_key_value("layer_num", new ConfigOptionInt(1));
            vars.set_key_value("layer_z", new ConfigOptionFloat(z));
            vars.set_key_value("toolchange_z", new ConfigOptionFloat(z));
            vars.set_key_value("max_layer_z", new ConfigOptionFloat(z));
            vars.set_key_value("relative_e_axis", new ConfigOptionBool(m_config.use_relative_e_distances.value));
            vars.set_key_value("toolchange_count", new ConfigOptionInt(3));
            vars.set_key_value("fan_speed", new ConfigOptionInt(0));
            vars.set_key_value("outer_wall_volumetric_speed",
                new ConfigOptionFloat(m_config.filament_max_volumetric_speed.get_at(new_material)));
            vars.set_key_value("old_retract_length", new ConfigOptionFloat(m_config.retraction_length.get_at(old_material)));
            vars.set_key_value("new_retract_length", new ConfigOptionFloat(m_config.retraction_length.get_at(new_material)));
            vars.set_key_value("filament_retract_length_nc",
                new ConfigOptionFloat(std::max(0., m_config.filament_retract_length_nc.get_at(old_material))));
            vars.set_key_value("old_retract_length_toolchange",
                new ConfigOptionFloat(m_config.retract_length_toolchange.get_at(old_material)));
            vars.set_key_value("new_retract_length_toolchange",
                new ConfigOptionFloat(m_config.retract_length_toolchange.get_at(new_material)));
            vars.set_key_value("new_extruder_retracted_length", new ConfigOptionFloat(0.));
            vars.set_key_value("old_filament_temp", new ConfigOptionInt(m_config.nozzle_temperature.get_at(old_material)));
            vars.set_key_value("new_filament_temp", new ConfigOptionInt(m_config.nozzle_temperature.get_at(new_material)));
            vars.set_key_value("x_after_toolchange", new ConfigOptionFloat(tower_centre.x()));
            vars.set_key_value("y_after_toolchange", new ConfigOptionFloat(tower_centre.y()));
            vars.set_key_value("z_after_toolchange", new ConfigOptionFloat(z));
            vars.set_key_value("first_flush_volume", new ConfigOptionFloat(flush_length / 2.));
            vars.set_key_value("second_flush_volume", new ConfigOptionFloat(flush_length / 2.));
            vars.set_key_value("old_filament_e_feedrate", new ConfigOptionInt(e_feedrate(old_material)));
            vars.set_key_value("new_filament_e_feedrate", new ConfigOptionInt(e_feedrate(new_material)));
            for (const char *key : {"travel_point_1_x", "travel_point_1_y", "travel_point_2_x", "travel_point_2_y",
                                    "travel_point_3_x", "travel_point_3_y"})
                vars.set_key_value(key, new ConfigOptionFloat(0.));
            const size_t filament_count = m_config.filament_type.values.size();
            std::vector<double> flush_speeds(filament_count), cooling(filament_count);
            std::vector<int> flush_temperatures(filament_count);
            const bool fast_flush = m_config.prime_volume_mode == PrimeVolumeMode::pvmFast;
            for (size_t id = 0; id < filament_count; ++id) {
                const int material = get_filament_config_indx(int(id), 1);
                flush_speeds[id] = positive_or(m_config.filament_flush_volumetric_speed.get_at(material),
                                               m_config.filament_max_volumetric_speed.get_at(material));
                flush_temperatures[id] = fast_flush ? m_config.filament_flush_temp_fast.get_at(material) :
                                                      m_config.filament_flush_temp.get_at(material);
                if (flush_temperatures[id] == 0)
                    flush_temperatures[id] = m_config.nozzle_temperature_range_high.get_at(id);
                cooling[id] = m_config.filament_cooling_before_tower.get_at(material);
            }
            vars.set_key_value("flush_volumetric_speeds", new ConfigOptionFloats(flush_speeds));
            vars.set_key_value("flush_temperatures", new ConfigOptionInts(flush_temperatures));
            vars.set_key_value("filament_cooling_before_tower", new ConfigOptionFloats(cooling));
            vars.set_key_value("flush_length", new ConfigOptionFloat(flush_length));
            for (int index = 1; index <= 4; ++index)
                vars.set_key_value("flush_length_" + std::to_string(index), new ConfigOptionFloat(0.));
            vars.set_key_value("wipe_avoid_perimeter", new ConfigOptionBool(m_config.prime_tower_skip_points.value));
            vars.set_key_value("wipe_avoid_pos_x", new ConfigOptionFloat(tower_centre.x()));
            vars.set_key_value("is_prime_tower_interface", new ConfigOptionBool(false));
            vars.set_key_value("filament_tower_interface_purge_volume",
                new ConfigOptionFloat(m_config.filament_tower_interface_purge_volume.get_at(size_t(to))));
            vars.set_key_value("filament_tower_interface_print_temp",
                new ConfigOptionInt(m_config.nozzle_temperature.get_at(new_material)));
            vars.set_key_value("wipe_tower_center_pos_x", new ConfigOptionFloat(tower_centre.x()));
            vars.set_key_value("wipe_tower_center_pos_y", new ConfigOptionFloat(tower_centre.y()));
            vars.set_key_value("wipe_tower_center_pos_valid", new ConfigOptionBool(true));
            vars.set_key_value("retraction_distance_when_cut",
                new ConfigOptionFloat(m_config.retraction_distances_when_cut.get_at(old_material)));
            vars.set_key_value("long_retraction_when_cut",
                new ConfigOptionBool(m_config.long_retractions_when_cut.get_at(old_material)));
            vars.set_key_value("retraction_distance_when_ec",
                new ConfigOptionFloat(m_config.retraction_distances_when_ec.get_at(old_material)));
            vars.set_key_value("long_retraction_when_ec",
                new ConfigOptionBool(m_config.long_retractions_when_ec.get_at(old_material)));
            std::string gcode = m_placeholder_parser.process(m_config.change_filament_gcode.value, (unsigned int) to, &vars);
            // GCode travels back beside the tower after the change G-code and drops to the tower's Z,
            // then makes a short lifted move to where the block starts and deretracts.
            const int nozzle = get_nozzle_config_index(to, 1);
            const double return_speed = m_config.travel_speed.get_at(nozzle);
            const double drop_speed = positive_or(m_config.travel_speed_z.get_at(nozzle), return_speed);
            const double deretract_speed = positive_or(m_config.deretraction_speed.get_at(new_material),
                                                       m_config.retraction_speed.get_at(new_material));
            const double deretract = m_config.retract_length_toolchange.get_at(new_material);
            const Vec2d beside = tower_centre + Vec2d(10., 0.);
            gcode += Slic3r::format("\nG1 X%1% Y%2% F%3%\nG1 Z%4% F%5%\n", beside.x(), beside.y(), 60. * return_speed,
                                    z, 60. * drop_speed);
            gcode += lift_gcode(to, z, tower_centre - beside);
            gcode += Slic3r::format("G1 X%1% Y%2% F%3%\nG1 Z%4% F%5%\n", tower_centre.x(), tower_centre.y(),
                                    60. * return_speed, z, 60. * drop_speed);
            if (deretract > 0. && deretract_speed > 0.)
                gcode += Slic3r::format("G1 E%1% F%2%\n", deretract, 60. * deretract_speed);
            seconds = snippet_timer.seconds(gcode, tower_centre.x(), tower_centre.y(), z);
            if (!std::isfinite(seconds))
                seconds = 0.;
        } catch (const std::exception &error) {
            BOOST_LOG_TRIVIAL(warning) << "Feature infill time: the change G-code from filament " << from + 1
                                       << " to " << to + 1 << " could not be timed: " << error.what();
            seconds = 0.;
        }
        change_cache.emplace(std::make_pair(from, to), seconds);
        return seconds;
    };
    // One digest line per generated row, written as the totals are summed, so two slices whose totals
    // differ can be compared row by row. Each row carries its start and end positions: seconds that
    // moved while both positions held mean the same road timed differently; a moved start position
    // means the block began at a different corner and the travel changed.
    std::ostringstream tower_digest;
    tower_digest.precision(17); // ios_base member, so this needs no <iomanip> here
    const auto record_tower_row = [&tower_digest](const char *kind, const WipeTower::ToolChangeResult &change,
                                                  double priced) {
        // A row whose writer laid no road still holds the writer's unknown-position value (the final
        // purge on a Type1 tower is one). Label it instead of printing a position far off the bed. A
        // level with no writer keeps its zeroes.
        const auto unset = [](float value) {
            // Catches the sentinel, and an infinity or a NaN from any arithmetic on it.
            return !(value < std::numeric_limits<float>::max());
        };
        const auto position = [&tower_digest, &unset](const char *name, const Vec2f &pos) {
            if (unset(pos.x()) || unset(pos.y()))
                tower_digest << " " << name << "=unset";
            else
                tower_digest << " " << name << "=" << double(pos.x()) << "," << double(pos.y());
        };
        tower_digest << kind << " z=" << double(change.print_z)
                     << " tower_z=" << double(change.tower_z_start)
                     << " seconds=" << double(change.elapsed_time)
                     << " priced=" << priced
                     << " purge=" << double(change.purge_volume)
                     << " tool_change=" << (change.is_tool_change ? 1 : 0)
                     << " leaves=" << change.initial_tool
                     << " enters=" << change.new_tool;
        position("start", change.start_pos);
        position("end", change.end_pos);
        tower_digest << "\n";
    };
    const auto index_committed_tower = [&]() {
        committed_rows.clear();
        committed_tower_seconds = 0.;
        committed_tower_volume_mm3 = 0.;
        tower_digest.str(std::string());
        // What each generated row costs the print: the estimator's time for its G-code, and for a row
        // laid between the part's own roads, the trip there and back.
        std::unordered_map<const WipeTower::ToolChangeResult *, double> priced;
        if (m_wipe_tower_data.priming)
            for (const auto &change : *m_wipe_tower_data.priming)
                priced[&change] = row_seconds(change);
        for (const auto &generated : m_wipe_tower_data.tool_changes)
            for (const auto &change : generated)
                priced[&change] = row_seconds(change) + visit_seconds(change);
        if (m_wipe_tower_data.final_purge)
            priced[&*m_wipe_tower_data.final_purge] = row_seconds(*m_wipe_tower_data.final_purge);
        for (const LayerTools &row : m_tool_ordering.layer_tools()) {
            CommittedRow entry;
            entry.print_z = row.print_z;
            entry.filaments.assign(row.extruders.begin(), row.extruders.end());
            committed_rows.push_back(std::move(entry));
        }
        const auto add_total = [&](const char *kind, const WipeTower::ToolChangeResult &change) {
            record_tower_row(kind, change, priced[&change]);
            committed_tower_seconds += priced[&change];
            // Same collection, same pass: the tower's material sits in memory beside its elapsed
            // time, so the band's tower work needs no extra tower build.
            committed_tower_volume_mm3 += change.purge_volume;
        };
        if (m_wipe_tower_data.priming)
            for (const auto &change : *m_wipe_tower_data.priming) add_total("priming", change);
        for (const auto &generated : m_wipe_tower_data.tool_changes)
            for (const auto &change : generated) add_total("generated", change);
        if (m_wipe_tower_data.final_purge) add_total("final_purge", *m_wipe_tower_data.final_purge);
        // The tower's solid base is paid once whatever any band decides: a band that stops switching only
        // moves the base up a layer. Its rows stay in the committed total and are never indexed as a
        // band's own work, or the lowest band would get a discount the print never collects.
        double tower_base_top_z = 0.;
        if (is_mixed_nozzle_slicing_enabled(m_config)) {
            double lowest = std::numeric_limits<double>::infinity();
            double highest = 0.;
            for (const auto &generated : m_wipe_tower_data.tool_changes)
                for (const auto &change : generated) {
                    lowest = std::min(lowest, double(change.print_z));
                    highest = std::max(highest, double(change.print_z));
                }
            if (std::isfinite(lowest) && highest > 0.)
                tower_base_top_z = std::max(lowest,
                    double(mixed_nozzle_tower_base_height(float(highest))));
        }
        for (const auto &generated : m_wipe_tower_data.tool_changes)
            for (const auto &change : generated) {
                if (!change.is_tool_change || change.new_tool < 0)
                    continue;
                if (double(change.print_z) <= tower_base_top_z + EPSILON)
                    continue;
                CommittedRow *owner = nullptr;
                double closest = std::numeric_limits<double>::infinity();
                for (CommittedRow &candidate : committed_rows) {
                    const double distance = std::abs(candidate.print_z - double(change.print_z));
                    if (distance < closest) {
                        closest = distance;
                        owner = &candidate;
                    }
                }
                if (owner == nullptr || closest > EPSILON)
                    continue;
                owner->tower_by_entered_filament.emplace_back((unsigned int) change.new_tool,
                    std::make_pair(priced[&change], double(change.purge_volume)));
            }
        // Every other generated row, filed under the layer it was laid at, so the whole-print check can
        // tell what the tower costs up to a given switch.
        for (const auto &generated : m_wipe_tower_data.tool_changes)
            for (const auto &change : generated) {
                if (change.is_tool_change && change.new_tool >= 0 &&
                    double(change.print_z) > tower_base_top_z + EPSILON)
                    continue;
                CommittedRow *owner = nullptr;
                double closest = std::numeric_limits<double>::infinity();
                for (CommittedRow &candidate : committed_rows) {
                    const double distance = std::abs(candidate.print_z - double(change.print_z));
                    if (distance < closest) {
                        closest = distance;
                        owner = &candidate;
                    }
                }
                if (owner != nullptr && closest <= EPSILON)
                    owner->loose_seconds += priced[&change];
            }
        // A plain level is looked for above the solid base, whose rows are heavier than any level a
        // dropped switch leaves behind.
        plain_level_seconds.assign(committed_rows.size(), 0.);
        {
            std::vector<size_t> levels;
            for (size_t index = 0; index < committed_rows.size(); ++index)
                if (committed_rows[index].loose_seconds > 0. &&
                    committed_rows[index].print_z > tower_base_top_z + EPSILON)
                    levels.push_back(index);
            if (!levels.empty())
                for (size_t index = 0; index < committed_rows.size(); ++index) {
                    const auto above = std::lower_bound(levels.begin(), levels.end(), index);
                    size_t nearest = above == levels.end() ? levels.back() : *above;
                    if (above != levels.begin() && (above == levels.end() || index - *(above - 1) < *above - index))
                        nearest = *(above - 1);
                    plain_level_seconds[index] = committed_rows[nearest].loose_seconds;
                }
        }
        // The tower every band is priced against, published before any band takes anything out of it.
        // Slices that disagree here disagree about the tower itself; slices that agree here but differ in
        // band seconds disagree about the pricing walk below.
        m_feature_economics_committed_tower_seconds = committed_tower_seconds;
        m_feature_economics_committed_tower_volume_mm3 = committed_tower_volume_mm3;
        m_feature_economics_committed_tower_digest = tower_digest.str();
        // The plan those rows were timed from, copied here rather than published by the tower build, so
        // all four describe the priced tower. The rebuild at the end of this pass can return before it
        // plans anything (when every band goes fine) and would otherwise leave the plan empty.
        m_feature_economics_committed_tower_plan_digest = m_wipe_tower_data.plan_digest;
    };
    // Price one tool sequence: the committed rows, with drop_filament removed from every row
    // drop_row flags. reference holds, per row and filament, whether the committed sequence switched
    // there; a tower row leaves the total only where the committed sequence switched and this one no
    // longer does. With nothing flagged it returns the committed cost unchanged. The walk takes any
    // rule for which filament leaves which row, so the whole-print check can price several bands at
    // once; price_sequence below is the one-band form every per-band trial uses.
    //
    // A row whose switches all leave still needs a tower level if a switch above it remains, since
    // the tower has to grow up to that switch. Such a row is charged a plain level, the nearest
    // level the committed tower laid with no switch, and refunded reports it with that cost.
    const auto price_walk = [&](const auto &dropped,
                                const std::vector<std::vector<char>> *reference,
                                std::vector<std::vector<char>> *record,
                                std::vector<std::pair<size_t, unsigned int>> *subtracted,
                                std::vector<std::pair<size_t, double>> *refunded = nullptr) {
        HandoffCost cost;
        cost.tower_seconds = committed_tower_seconds;
        cost.tower_volume_mm3 = committed_tower_volume_mm3;
        if (record) {
            record->clear();
            record->resize(committed_rows.size());
        }
        if (subtracted)
            subtracted->clear();
        if (refunded)
            refunded->clear();
        NativeFilamentChangeState state;
        state.filament_by_extruder.assign(m_config.nozzle_diameter.size(), -1);
        state.last_filament_by_extruder.assign(m_config.nozzle_diameter.size(), -1);
        int active = -1;
        size_t last_switch = 0;
        std::vector<size_t> emptied;
        for (size_t index = 0; index < committed_rows.size(); ++index) {
            size_t entries_removed = 0;
            const CommittedRow &row = committed_rows[index];
            if (record)
                (*record)[index].assign(row.filaments.size(), 0);
            for (size_t slot = 0; slot < row.filaments.size(); ++slot) {
                const unsigned int filament = row.filaments[slot];
                bool switched = false;
                if (!dropped(index, filament)) {
                    NativeFilamentChangeRequest request;
                    request.logical_filament_id = int(filament);
                    request.legacy_extruder_id = int(get_extruder_id(filament));
                    const auto transition = evaluate_native_filament_change(state, request, native_timing,
                        m_nozzle_group_result ? NativeFilamentChangeModel::MultiNozzle : NativeFilamentChangeModel::Legacy,
                        m_nozzle_group_result.get(), m_config.filament_diameter.size());
                    if (!transition.applied() && !transition.no_op())
                        return HandoffCost {};
                    cost.switch_seconds += transition.total_seconds();
                    switched = !transition.no_op();
                    state = transition.next_state;
                    if (switched) {
                        cost.switch_seconds += change_seconds(active, int(filament));
                        active = int(filament);
                        last_switch = index;
                    }
                }
                if (record)
                    (*record)[index][slot] = switched ? 1 : 0;
                const bool switched_before = reference != nullptr && index < reference->size() &&
                    slot < (*reference)[index].size() && (*reference)[index][slot] != 0;
                if (switched || !switched_before)
                    continue;
                // This sequence no longer switches here, so the tower work that switch paid for
                // leaves the total with it.
                bool removed_any = false;
                for (const auto &entry : row.tower_by_entered_filament)
                    if (entry.first == filament) {
                        cost.tower_seconds -= entry.second.first;
                        cost.tower_volume_mm3 -= entry.second.second;
                        removed_any = true;
                        ++entries_removed;
                    }
                if (removed_any && subtracted)
                    subtracted->emplace_back(index, filament);
            }
            if (entries_removed > 0 && entries_removed == row.tower_by_entered_filament.size() &&
                !(row.loose_seconds > 0.))
                emptied.push_back(index);
        }
        for (size_t index : emptied)
            if (index < last_switch && index < plain_level_seconds.size() && plain_level_seconds[index] > 0.) {
                cost.tower_seconds += plain_level_seconds[index];
                if (refunded)
                    refunded->emplace_back(index, plain_level_seconds[index]);
            }
        cost.available = std::isfinite(cost.tower_seconds) && std::isfinite(cost.switch_seconds);
        return cost;
    };
    const auto price_sequence = [&](const std::vector<char> &drop_row, unsigned int drop_filament,
                                    const std::vector<std::vector<char>> *reference,
                                    std::vector<std::vector<char>> *record,
                                    std::vector<std::pair<size_t, unsigned int>> *subtracted,
                                    std::vector<std::pair<size_t, double>> *refunded = nullptr) {
        return price_walk([&drop_row, drop_filament](size_t index, unsigned int filament) {
            return index < drop_row.size() && drop_row[index] != 0 && filament == drop_filament;
        }, reference, record, subtracted, refunded);
    };
    const auto model_seconds = [&](const std::vector<Layer *> &layers) {
        double seconds = 0.;
        for (const Layer *layer : layers) {
            const LayerTools &tools = m_tool_ordering.tools_for_layer(layer->print_z);
            for (const LayerRegion *region : layer->regions()) {
                FullPrintConfig cfg;
                cfg.apply(m_config);
                cfg.apply(layer->object()->config());
                cfg.apply(region->region().config());
                for (const ExtrusionEntity *group : region->fills.entities) {
                    const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(group);
                    if (!collection)
                        return std::numeric_limits<double>::quiet_NaN();
                    const unsigned filament = tools.extruder(*collection, region->region(), region);
                    const int nozzle = get_nozzle_config_index(int(filament), layer->id());
                    const int material = get_filament_config_indx(int(filament), layer->id());
                    // The volume per second the path is laid at: its role's speed, slowed on the first
                    // layers and capped by the filament's volumetric limit.
                    const auto path_rate = [&](const ExtrusionPath &path) {
                        double speed = 0.;
                        switch (path.role()) {
                        case erInternalInfill: speed = cfg.sparse_infill_speed.get_at(nozzle); break;
                        case erSolidInfill: speed = cfg.internal_solid_infill_speed.get_at(nozzle); break;
                        case erTopSolidInfill: speed = cfg.top_surface_speed.get_at(nozzle); break;
                        case erBottomSurface: speed = cfg.initial_layer_infill_speed.get_at(nozzle); break;
                        case erBridgeInfill: speed = cfg.bridge_speed.get_at(nozzle); break;
                        case erInternalBridgeInfill: speed = cfg.get_abs_value_at("internal_bridge_speed", nozzle); break;
                        case erGapFill: speed = cfg.gap_infill_speed.get_at(nozzle); break;
                        case erIroning: speed = cfg.ironing_speed.value; break;
                        default: return std::numeric_limits<double>::quiet_NaN();
                        }
                        double cap = m_config.filament_max_volumetric_speed.get_at(material);
                        if (m_config.filament_adaptive_volumetric_speed.get_at(material))
                            cap = std::min(cap, calc_max_volumetric_speed(path.height, path.width,
                                m_config.volumetric_speed_coefficients.get_at(material)));
                        if (layer->id() == 0)
                            speed = cfg.initial_layer_infill_speed.get_at(nozzle);
                        else if (layer->id() < cfg.slow_down_layers.value) {
                            const double initial = cfg.initial_layer_infill_speed.get_at(nozzle);
                            if (speed > initial)
                                speed = initial + (speed - initial) * layer->id() /
                                    cfg.slow_down_layers.value;
                        }
                        const double flow_ratio = m_config.filament_flow_ratio.get_at(material);
                        if (!(flow_ratio > 0.)) return std::numeric_limits<double>::quiet_NaN();
                        cap /= flow_ratio;
                        double rate = speed * path.mm3_per_mm;
                        if (cap > 0.) rate = rate > 0. ? std::min(rate, cap) : cap;
                        return rate > 0. ? rate : std::numeric_limits<double>::quiet_NaN();
                    };
                    // The acceleration GCode sets for the path's role.
                    const auto path_acceleration = [&](const ExtrusionPath &path) {
                        const double fallback = cfg.default_acceleration.get_at(nozzle);
                        if (!(fallback > 0.))
                            return 0.;
                        if (layer->id() == 0 && cfg.initial_layer_acceleration.get_at(nozzle) > 0.)
                            return double(cfg.initial_layer_acceleration.get_at(nozzle));
                        double value = 0.;
                        if (is_bridge(path.role()))
                            value = cfg.get_abs_value_at("bridge_acceleration", nozzle);
                        else if (path.role() == erInternalInfill)
                            value = cfg.get_abs_value_at("sparse_infill_acceleration", nozzle);
                        else if (path.role() == erSolidInfill)
                            value = cfg.get_abs_value_at("internal_solid_infill_acceleration", nozzle);
                        else if (is_top_surface(path.role()))
                            value = cfg.top_surface_acceleration.get_at(nozzle);
                        return value > 0. ? value : fallback;
                    };
                    // The collection is written out as G-code and timed by the estimator, so
                    // acceleration and corners count, as do the short moves between its paths. A
                    // collection the estimator cannot time falls back to volume over rate.
                    const double filament_area = 0.25 * M_PI * std::pow(m_config.filament_diameter.get_at(filament), 2);
                    const double travel_speed = m_config.travel_speed.get_at(nozzle);
                    double rate_seconds = 0.;
                    std::string gcode;
                    Vec2d start = Vec2d::Zero();
                    bool first = true;
                    double last_acceleration = -1.;
                    char line[128];
                    const auto add_path = [&](const ExtrusionPath &path) {
                        const double rate = path_rate(path);
                        rate_seconds += path.total_volume() / rate;
                        const auto &points = path.polyline.points;
                        if (!(rate > 0.) || points.size() < 2)
                            return;
                        const Vec2d from = unscale(points.front()).template head<2>();
                        if (first) {
                            start = from;
                            first = false;
                        } else if (travel_speed > 0.) {
                            snprintf(line, sizeof(line), "G1 X%.3f Y%.3f F%.0f\n", from.x(), from.y(), 60. * travel_speed);
                            gcode += line;
                        }
                        const double acceleration = path_acceleration(path);
                        if (acceleration > 0. && acceleration != last_acceleration) {
                            snprintf(line, sizeof(line), "M204 S%.0f\n", acceleration);
                            gcode += line;
                            last_acceleration = acceleration;
                        }
                        const double feedrate = 60. * rate / path.mm3_per_mm;
                        const double e_per_mm = filament_area > 0. ? path.mm3_per_mm / filament_area : 0.;
                        Vec2d previous = from;
                        for (size_t at = 1; at < points.size(); ++ at) {
                            const Vec2d to = unscale(points[at]).template head<2>();
                            snprintf(line, sizeof(line), at == 1 ? "G1 X%.3f Y%.3f E%.5f F%.0f\n" : "G1 X%.3f Y%.3f E%.5f\n",
                                     to.x(), to.y(), (to - previous).norm() * e_per_mm, feedrate);
                            gcode += line;
                            previous = to;
                        }
                    };
                    const auto flat = collection->flatten();
                    for (const ExtrusionEntity *entity : flat.entities) {
                        if (const auto *path = dynamic_cast<const ExtrusionPath *>(entity)) add_path(*path);
                        else if (const auto *paths = dynamic_cast<const ExtrusionMultiPath *>(entity))
                            for (const auto &path : paths->paths) add_path(path);
                        else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(entity))
                            for (const auto &path : loop->paths) add_path(path);
                        else return std::numeric_limits<double>::quiet_NaN();
                    }
                    if (!std::isfinite(rate_seconds))
                        return std::numeric_limits<double>::quiet_NaN();
                    const double timed = gcode.empty() ? std::numeric_limits<double>::quiet_NaN() :
                        snippet_timer.seconds(gcode, start.x(), start.y(), layer->print_z);
                    seconds += std::isfinite(timed) ? timed : rate_seconds;
                }
            }
        }
        return seconds;
    };

    // The committed tower is indexed once for the whole print. A band that goes fine takes its own
    // switches out of that index, so the bands below price against the shortened tower.
    index_committed_tower();
    // Every object layer by height, so a band can ask whether anything else still wants the
    // coarse filament at that height before it takes the switch out.
    std::vector<std::pair<double, const Layer *>> layers_by_height;
    for (const PrintObject *object : m_objects)
        for (const Layer *layer : object->layers())
            layers_by_height.emplace_back(layer->print_z, layer);
    std::sort(layers_by_height.begin(), layers_by_height.end(),
        [](const std::pair<double, const Layer *> &left, const std::pair<double, const Layer *> &right) {
            return left.first < right.first;
        });
    // Filaments this pass must never take a switch away from, because something it does not
    // model prints with them: support, its interface, and a pinned tower filament.
    std::set<unsigned int> protected_filaments;
    for (const PrintObject *object : m_objects) {
        if (object->support_layers().empty())
            continue;
        for (int id : {object->config().support_filament.value,
                       object->config().support_interface_filament.value})
            if (id > 0)
                protected_filaments.insert((unsigned int)(id - 1));
    }
    if (m_config.wipe_tower_filament.value > 0)
        protected_filaments.insert((unsigned int)(m_config.wipe_tower_filament.value - 1));
    // What LayerTools::extruder() can still hand this layer, read from region roles rather than by
    // walking extrusions. A region whose sparse infill is fine owned no longer asks for the coarse
    // tool. This errs toward keeping a switch, so a band can only under-claim its saving. One region
    // at a time with its sparse ownership passed in, so the whole-print check can ask about a region
    // it is only considering sending fine.
    const auto region_may_use = [](const LayerRegion *region, unsigned int filament, bool sparse_fine_owned) {
        if (region->cell_filament_id() > 0)
            return (unsigned int)(region->cell_filament_id() - 1) == filament;
        const PrintRegionConfig &cfg = region->region().config();
        for (int id : {cfg.outer_wall_filament_id.value, cfg.inner_wall_filament_id.value,
                       cfg.internal_solid_filament_id.value, cfg.top_surface_filament_id.value,
                       cfg.bottom_surface_filament_id.value})
            if (id > 0 && (unsigned int)(id - 1) == filament)
                return true;
        return !sparse_fine_owned && cfg.sparse_infill_filament_id.value > 0 &&
               (unsigned int)(cfg.sparse_infill_filament_id.value - 1) == filament;
    };
    const auto layer_may_use = [&region_may_use](const Layer *layer, unsigned int filament) {
        for (const LayerRegion *region : layer->regions())
            if (region_may_use(region, filament, region->feature_split_sparse_fine_owned()))
                return true;
        return false;
    };
    const auto height_still_uses = [&](double print_z, unsigned int filament) {
        auto at = std::lower_bound(layers_by_height.begin(), layers_by_height.end(), print_z - EPSILON,
            [](const std::pair<double, const Layer *> &entry, double value) { return entry.first < value; });
        for (; at != layers_by_height.end() && at->first <= print_z + EPSILON; ++ at)
            if (layer_may_use(at->second, filament))
                return true;
        return false;
    };
    // The same question with the regions in going_fine read as fine owned already.
    const auto height_would_use = [&](double print_z, unsigned int filament,
                                      const std::set<const LayerRegion *> &going_fine) {
        auto at = std::lower_bound(layers_by_height.begin(), layers_by_height.end(), print_z - EPSILON,
            [](const std::pair<double, const Layer *> &entry, double value) { return entry.first < value; });
        for (; at != layers_by_height.end() && at->first <= print_z + EPSILON; ++ at)
            for (const LayerRegion *region : at->second->regions())
                if (region_may_use(region, filament,
                        region->feature_split_sparse_fine_owned() || going_fine.count(region) != 0))
                    return true;
        return false;
    };
    struct SavedRegion {
        LayerRegion *region;
        SurfaceCollection surfaces;
        ExtrusionEntityCollection fills;
        bool fine;
    };
    // A band that wins keeps its regenerated fills, and the coarse state it replaced is held
    // until the final tower is built, so a tower the agreed plan cannot build can be undone.
    struct AcceptedBand {
        AdvisorFeatureBand      *band;
        std::vector<SavedRegion> saved;
    };
    std::vector<AcceptedBand> accepted;
    // Every band the per-band pass kept coarse, with what the whole-print check needs to price it
    // again and, if it has to, send it fine.
    struct KeptBand {
        PrintObject                      *object { nullptr };
        AdvisorFeatureBand               *band { nullptr };
        std::vector<Layer *>              own;
        std::vector<size_t>               own_index;
        Layer                            *roof { nullptr };
        size_t                            roof_index { 0 };
        unsigned int                      coarse_id { 0 };
        double                            fine_model { 0. };
        double                            coarse_model { 0. };
        std::vector<std::vector<size_t>>  rows; // per own layer, the committed rows at its height
    };
    std::vector<KeptBand> kept;

    // What a band's trial re-fills is fixed before the pass starts: its own layers go back to the
    // surfaces captured before combining and are filled again, so no other band's decision can
    // change the result. The one layer that is not fixed is the roof above the band, which the
    // band above may have kept its own fine fills on, so the roof is re-filled on the pass that
    // decides and everything else can be re-filled before it.
    struct BandTrial {
        size_t               band { 0 };          // which band in the published vector
        std::vector<Layer *> own;                 // the band's own layers, without the roof
        std::vector<size_t>  own_index;           // where each of those sits in the object's layers
        Layer               *roof { nullptr };    // the layer above the band, shared with the band above
        size_t               roof_index { 0 };
        bool                 alone { false };     // no other band's trial re-fills these layers
        bool                 ready { false };     // the re-fill below was done before the deciding pass
        std::vector<SavedRegion> fine;            // what that re-fill left on the band's own layers
    };
    for (PrintObject *object : m_objects) {
        const auto published = object->m_mixed_nozzle_advisor_pending;
        if (!published || object->m_feature_uncombined_surfaces.size() != object->m_layers.size()) continue;
        object->m_mixed_nozzle_advisor_pending = std::make_shared<AdvisorSliceObservations>(*published);
        auto &bands = object->m_mixed_nozzle_advisor_pending->feature_bands;
        // Top down lets subsequent trials see a tower shortened by earlier accepted decisions.
        std::vector<BandTrial> trials;
        for (size_t at = bands.size(); at-- > 0; ) {
            const AdvisorFeatureBand &band = bands[at];
            if (band.outcome != AdvisorFeatureOutcome::Committed) continue;
            BandTrial trial;
            trial.band = at;
            for (size_t index = 0; index < object->m_layers.size(); ++ index) {
                Layer *layer = object->m_layers[index];
                if (layer->print_z > band.z_low + EPSILON && layer->print_z <= band.z_high + EPSILON) {
                    trial.own.push_back(layer);
                    trial.own_index.push_back(index);
                }
            }
            if (trial.own.empty()) continue;
            trial.roof = trial.own.back()->upper_layer;
            if (trial.roof != nullptr) {
                trial.roof_index = size_t(std::find(object->m_layers.begin(), object->m_layers.end(), trial.roof) -
                                          object->m_layers.begin());
                if (trial.roof_index >= object->m_layers.size())
                    trial.roof = nullptr;
            }
            trials.push_back(std::move(trial));
        }
        // make_fills() works on a whole layer, so a layer two bands of different regions both
        // re-fill has to stay on the deciding pass, where the two cannot be in it at once.
        std::vector<unsigned int> claims(object->m_layers.size(), 0);
        for (const BandTrial &trial : trials)
            for (size_t index : trial.own_index)
                ++ claims[index];
        for (BandTrial &trial : trials) {
            trial.alone = true;
            for (size_t index : trial.own_index)
                if (claims[index] != 1)
                    trial.alone = false;
        }
        // Re-fill one band's own layers, keep what that produced, and leave the layers as found.
        const auto prepare_trial = [&](BandTrial &trial) {
            const AdvisorFeatureBand &band = bands[trial.band];
            std::vector<SavedRegion> coarse;
            for (Layer *layer : trial.own)
                for (LayerRegion *region : layer->regions())
                    coarse.push_back({region, region->fill_surfaces, region->fills,
                                      region->feature_split_sparse_fine_owned()});
            const auto put_back = [&coarse]() {
                for (auto &entry : coarse) {
                    entry.region->fill_surfaces = std::move(entry.surfaces);
                    entry.region->fills = std::move(entry.fills);
                    entry.region->set_feature_split_sparse_fine_owned(entry.fine);
                }
            };
            try {
                for (size_t at = 0; at < trial.own.size(); ++ at) {
                    throw_if_canceled();
                    Layer *layer = trial.own[at];
                    LayerRegion *region = layer->get_region(int(band.region_id));
                    region->fill_surfaces = object->m_feature_uncombined_surfaces[trial.own_index[at]][band.region_id];
                    region->set_feature_split_sparse_fine_owned(true);
                    layer->make_fills(object->m_adaptive_fill_octrees.first.get(),
                        object->m_adaptive_fill_octrees.second.get(), object->m_lightning_generator.get());
                    layer->make_ironing();
                    layer->simplify_infill_extrusion_path();
                }
            } catch (...) {
                put_back();
                throw;
            }
            for (Layer *layer : trial.own)
                for (LayerRegion *region : layer->regions())
                    trial.fine.push_back({region, std::move(region->fill_surfaces), std::move(region->fills),
                                          region->feature_split_sparse_fine_owned()});
            put_back();
            trial.ready = true;
        };
        // Nothing these trials read or write is shared with each other, so they go out over the
        // machine's cores the way the pipeline's own fill, ironing and simplify stages do, under
        // the same cancellation check. A trial that throws puts its own layers back first.
        {
            std::vector<size_t> hand_out;
            for (size_t at = 0; at < trials.size(); ++ at)
                if (trials[at].alone)
                    hand_out.push_back(at);
            tbb::parallel_for(tbb::blocked_range<size_t>(0, hand_out.size()),
                [this, &trials, &hand_out, &prepare_trial](const tbb::blocked_range<size_t> &range) {
                    for (size_t at = range.begin(); at < range.end(); ++ at) {
                        this->throw_if_canceled();
                        prepare_trial(trials[hand_out[at]]);
                    }
                }
            );
            throw_if_canceled();
        }
        // A decision that stands changes the layers it was made on. A trial prepared against what
        // those layers held before that is no longer the trial this pass would run, so it goes
        // back on the deciding pass.
        std::vector<char> disturbed(object->m_layers.size(), 0);

        for (BandTrial &trial : trials) {
            AdvisorFeatureBand &band = bands[trial.band];
            throw_if_canceled();
            std::vector<Layer *> layers = trial.own;
            if (trial.roof != nullptr) layers.push_back(trial.roof);
            const double coarse_model = model_seconds(layers) * object->instances().size();
            std::vector<std::vector<char>> committed_record;
            const HandoffCost coarse_handoff = price_sequence({}, 0, nullptr, &committed_record, nullptr);
            if (!std::isfinite(coarse_model) || !coarse_handoff.available) {
                band.reason = !std::isfinite(coarse_model) ? "native_model_cost_unavailable" : "native_handoff_cost_unavailable";
                continue;
            }
            bool ready = trial.ready;
            for (size_t index : trial.own_index)
                if (disturbed[index] != 0)
                    ready = false;
            std::vector<SavedRegion> saved;
            for (Layer *layer : layers)
                for (LayerRegion *region : layer->regions())
                    saved.push_back({region, region->fill_surfaces, region->fills, region->feature_split_sparse_fine_owned()});
            const auto restore_regions = [&] {
                for (auto &entry : saved) {
                    entry.region->fill_surfaces = std::move(entry.surfaces);
                    entry.region->fills = std::move(entry.fills);
                    entry.region->set_feature_split_sparse_fine_owned(entry.fine);
                }
            };
            ++ m_feature_economics_trials;
            if (ready)
                ++ m_feature_economics_prepared_trials;
            try {
            if (ready) {
                for (auto &entry : trial.fine) {
                    entry.region->fill_surfaces = std::move(entry.surfaces);
                    entry.region->fills = std::move(entry.fills);
                    entry.region->set_feature_split_sparse_fine_owned(entry.fine);
                }
                m_feature_economics_refilled_layers += trial.own.size();
            } else {
                for (size_t at = 0; at < trial.own.size(); ++ at) {
                    throw_if_canceled();
                    ++ m_feature_economics_refilled_layers;
                    Layer *layer = trial.own[at];
                    LayerRegion *region = layer->get_region(int(band.region_id));
                    region->fill_surfaces = object->m_feature_uncombined_surfaces[trial.own_index[at]][band.region_id];
                    region->set_feature_split_sparse_fine_owned(true);
                    layer->make_fills(object->m_adaptive_fill_octrees.first.get(),
                        object->m_adaptive_fill_octrees.second.get(), object->m_lightning_generator.get());
                    layer->make_ironing();
                    layer->simplify_infill_extrusion_path();
                }
            }
            if (trial.roof != nullptr) {
                throw_if_canceled();
                ++ m_feature_economics_refilled_layers;
                LayerRegion *region = trial.roof->get_region(int(band.region_id));
                const auto &original = object->m_feature_uncombined_surfaces[trial.roof_index][band.region_id];
                // Undo only the extra roof protection above this rejected coarse band.
                // Preserve this layer's own sparse/void schedule for its next band.
                Surfaces surfaces;
                for (const Surface &surface : region->fill_surfaces.surfaces)
                    if (surface.surface_type == stInternal || surface.surface_type == stInternalVoid)
                        surfaces.push_back(surface);
                for (const Surface &surface : original.surfaces)
                    if (surface.surface_type != stInternal && surface.surface_type != stInternalVoid)
                        surfaces.push_back(surface);
                region->fill_surfaces.surfaces = std::move(surfaces);
                trial.roof->make_fills(object->m_adaptive_fill_octrees.first.get(),
                    object->m_adaptive_fill_octrees.second.get(), object->m_lightning_generator.get());
                trial.roof->make_ironing();
                trial.roof->simplify_infill_extrusion_path();
            }
            } catch (...) {
                restore_regions();
                throw;
            }
            // The band's coarse infill is the only thing this trial takes away. LayerTools::extruder()
            // sends a fine owned sparse region to outer_wall_filament_id and a coarse one to
            // sparse_infill_filament_id, so the switch that goes is the one into that coarse id,
            // on every band layer where nothing else at that height still wants it.
            const PrintRegionConfig &band_config = layers.front()->get_region(int(band.region_id))->region().config();
            const int coarse_id = band_config.sparse_infill_filament_id.value - 1;
            const int fine_id = band_config.outer_wall_filament_id.value - 1;
            std::vector<char> drop_row(committed_rows.size(), 0);
            if (coarse_id >= 0 && coarse_id != fine_id &&
                protected_filaments.count((unsigned int) coarse_id) == 0)
                for (const Layer *layer : layers) {
                    if (layer == trial.roof || height_still_uses(layer->print_z, (unsigned int) coarse_id))
                        continue;
                    for (size_t index = 0; index < committed_rows.size(); ++ index)
                        if (std::abs(committed_rows[index].print_z - layer->print_z) < EPSILON)
                            drop_row[index] = 1;
                }
            std::vector<std::pair<size_t, unsigned int>> subtracted;
            std::vector<std::pair<size_t, double>> refunded;
            const double fine_model = model_seconds(layers) * object->instances().size();
            const HandoffCost fine_handoff = price_sequence(drop_row, (unsigned int) std::max(coarse_id, 0),
                                                            &committed_record, nullptr, &subtracted, &refunded);
            const bool available = std::isfinite(fine_model) && fine_handoff.available;
            if (available) {
                band.fine_model_seconds = fine_model;
                band.coarse_model_seconds = coarse_model;
                // Publish the split first and define the legacy scalar as its sum, so the two
                // forms are bit-identical for any existing reader.
                band.incremental_switch_seconds = coarse_handoff.switch_seconds - fine_handoff.switch_seconds;
                band.incremental_tower_seconds = coarse_handoff.tower_seconds - fine_handoff.tower_seconds;
                band.incremental_handoff_seconds = *band.incremental_switch_seconds + *band.incremental_tower_seconds;
                band.incremental_tower_volume_mm3 = coarse_handoff.tower_volume_mm3 - fine_handoff.tower_volume_mm3;
            }
            // A coarse band that lays much less sparse infill than its fine layers would is not a
            // cheaper band, it is a hollow one: a region narrower than the coarse pattern's line pitch
            // (a 6 mm column on the 0.6 at 15 %) gets no coarse line at all and costs nothing, so the
            // time test below would keep it coarse. Such a band goes fine whatever the time.
            double coarse_sparse = 0., fine_sparse = 0.;
            std::set<const LayerRegion *> band_regions;
            for (Layer *layer : trial.own) {
                const LayerRegion *region = layer->get_region(int(band.region_id));
                band_regions.insert(region);
                fine_sparse += sparse_infill_volume(region->fills);
            }
            for (const SavedRegion &entry : saved)
                if (band_regions.count(entry.region) != 0)
                    coarse_sparse += sparse_infill_volume(entry.fills);
            const bool coarse_starved = fine_sparse > EPSILON && coarse_sparse < 0.5 * fine_sparse;
            if (available && (coarse_starved ||
                              fine_model + fine_handoff.total_seconds() < coarse_model + coarse_handoff.total_seconds())) {
                band.outcome = AdvisorFeatureOutcome::FineFallback;
                band.reason = coarse_starved ? "coarse_band_lays_too_little_infill" : "fine_faster_with_native_tower";
                // Carry the shortened tower forward, so the bands below price against it.
                for (size_t index = 0; index < drop_row.size(); ++ index)
                    if (drop_row[index] != 0) {
                        auto &filaments = committed_rows[index].filaments;
                        filaments.erase(std::remove(filaments.begin(), filaments.end(), (unsigned int) coarse_id),
                                        filaments.end());
                    }
                for (const auto &entry : subtracted) {
                    auto &tower_rows = committed_rows[entry.first].tower_by_entered_filament;
                    tower_rows.erase(std::remove_if(tower_rows.begin(), tower_rows.end(),
                        [&entry](const std::pair<unsigned int, std::pair<double, double>> &row) {
                            return row.first == entry.second;
                        }), tower_rows.end());
                }
                // A row left with no switch below the last one is a plain level now.
                for (const auto &entry : refunded)
                    committed_rows[entry.first].loose_seconds += entry.second;
                committed_tower_seconds = fine_handoff.tower_seconds;
                committed_tower_volume_mm3 = fine_handoff.tower_volume_mm3;
                // The layers this decision stands on are not the layers a later trial was
                // prepared against any more.
                for (size_t index : trial.own_index)
                    disturbed[index] = 1;
                if (trial.roof != nullptr)
                    disturbed[trial.roof_index] = 1;
                accepted.push_back(AcceptedBand{&band, std::move(saved)});
                BOOST_LOG_TRIVIAL(info) << "Feature infill time: fine at Z " << band.z_high
                    << ", model " << coarse_model << " -> " << fine_model
                    << " s, avoided handoff " << coarse_handoff.total_seconds() - fine_handoff.total_seconds()
                    << " s (switching " << coarse_handoff.switch_seconds - fine_handoff.switch_seconds
                    << " s, tower " << coarse_handoff.tower_seconds - fine_handoff.tower_seconds << " s)";
            } else {
                restore_regions();
                band.reason = available ? "coarse_faster_with_native_tower" : "native_cost_unavailable";
                // A band that stayed coarse on its own numbers still has to pay for the whole tower, which
                // only the pass after this loop can see.
                if (available && coarse_id >= 0 && coarse_id != fine_id &&
                    protected_filaments.count((unsigned int) coarse_id) == 0) {
                    KeptBand entry;
                    entry.object = object;
                    entry.band = &band;
                    entry.own = trial.own;
                    entry.own_index = trial.own_index;
                    entry.roof = trial.roof;
                    entry.roof_index = trial.roof_index;
                    entry.coarse_id = (unsigned int) coarse_id;
                    entry.fine_model = fine_model;
                    entry.coarse_model = coarse_model;
                    for (const Layer *layer : trial.own) {
                        std::vector<size_t> at_height;
                        for (size_t index = 0; index < committed_rows.size(); ++ index)
                            if (std::abs(committed_rows[index].print_z - layer->print_z) < EPSILON)
                                at_height.push_back(index);
                        entry.rows.push_back(std::move(at_height));
                    }
                    kept.push_back(std::move(entry));
                }
            }
        }
        object->m_feature_economics_applied = true;
        // The normal final simplification stage publishes these updated observations.
    }

    // Bands that went fine on their own numbers can leave a shorter tower with a smaller footprint,
    // which the committed rows do not show. When some bands still wait for the whole-print check,
    // the tower is built again for the plan as it now stands and priced afresh. The published
    // committed figures keep describing the tower the bands were first priced against.
    const auto reindex_published_kept = [&]() {
        const double published_seconds = m_feature_economics_committed_tower_seconds;
        const double published_volume = m_feature_economics_committed_tower_volume_mm3;
        const std::string published_digest = m_feature_economics_committed_tower_digest;
        const std::string published_plan = m_feature_economics_committed_tower_plan_digest;
        index_committed_tower();
        m_feature_economics_committed_tower_seconds = published_seconds;
        m_feature_economics_committed_tower_volume_mm3 = published_volume;
        m_feature_economics_committed_tower_digest = published_digest;
        m_feature_economics_committed_tower_plan_digest = published_plan;
    };
    if (!accepted.empty() && !kept.empty()) {
        bool rebuilt = false;
        try {
            _make_wipe_tower();
            rebuilt = true;
        } catch (const Slic3r::SlicingError &) {
            // No tower for this plan; the final build below decides what the print gets. The bands
            // are priced on the rows they already have.
        }
        if (rebuilt) {
            reindex_published_kept();
            for (KeptBand &entry : kept)
                for (size_t layer = 0; layer < entry.own.size(); ++ layer) {
                    entry.rows[layer].clear();
                    for (size_t index = 0; index < committed_rows.size(); ++ index)
                        if (std::abs(committed_rows[index].print_z - entry.own[layer]->print_z) < EPSILON)
                            entry.rows[layer].push_back(index);
                }
        }
    }

    // The model seconds of a plan that sent bands fine at the whole-print check, priced again on the
    // tower built for it at the end; negative when there is nothing to price again.
    double reprice_model = -1.;

    // The whole print, after every band has decided on its own numbers. A band's price holds its
    // switches and the tower rows they pay for, never the tower as a whole: the priming, the solid
    // base, the levels laid between switches and the height needed to reach the last switch. Bands
    // that each save a little can stand up a tower that costs more than they save together, so the
    // agreed plan is priced as a whole and set against all fine. All fine keeps a tower only for the
    // switches it still has; with no coarse work left, _make_wipe_tower() builds that tower as Off does.
    if (!kept.empty()) {
        std::vector<std::vector<char>> current_record;
        price_sequence({}, 0, nullptr, &current_record, nullptr);
        // A print that lays tower levels on every layer whatever switches keeps its whole tower.
        const bool tower_stands_alone = m_config.timelapse_type.value == TimelapseType::tlSmooth ||
                                        m_config.enable_wrapping_detection.value;
        // Top down, so going fine one band at a time takes the tower's top down with it.
        std::vector<size_t> order(kept.size());
        std::iota(order.begin(), order.end(), size_t(0));
        std::stable_sort(order.begin(), order.end(), [&kept](size_t left, size_t right) {
            return kept[left].band->z_high > kept[right].band->z_high;
        });
        struct PlanCost {
            double model { 0. };
            double tower { 0. };
            double switches { 0. };
            bool   available { false };
            double total() const { return model + tower + switches; }
        };
        // The plan with the top `going` bands in that order sent fine and the rest kept coarse.
        const auto price_plan = [&](size_t going) {
            std::set<const LayerRegion *> going_fine;
            for (size_t at = 0; at < going; ++ at) {
                const KeptBand &entry = kept[order[at]];
                for (const Layer *layer : entry.own)
                    going_fine.insert(layer->get_region(int(entry.band->region_id)));
            }
            std::vector<std::vector<unsigned int>> drops(committed_rows.size());
            for (size_t at = 0; at < going; ++ at) {
                const KeptBand &entry = kept[order[at]];
                for (size_t layer = 0; layer < entry.own.size(); ++ layer) {
                    if (height_would_use(entry.own[layer]->print_z, entry.coarse_id, going_fine))
                        continue;
                    for (size_t index : entry.rows[layer])
                        drops[index].push_back(entry.coarse_id);
                }
            }
            std::vector<std::vector<char>> record;
            const HandoffCost handoff = price_walk([&drops](size_t index, unsigned int filament) {
                return std::find(drops[index].begin(), drops[index].end(), filament) != drops[index].end();
            }, &current_record, &record, nullptr);
            PlanCost cost;
            cost.available = handoff.available;
            if (!cost.available)
                return cost;
            for (size_t at = 0; at < order.size(); ++ at)
                cost.model += at < going ? kept[order[at]].fine_model : kept[order[at]].coarse_model;
            cost.switches = handoff.switch_seconds;
            // The first filament's first load is not a tool change; any switch after it is, and
            // the tower is built up to the last of them.
            size_t switches = 0, last_switch = 0;
            for (size_t index = 0; index < record.size(); ++ index)
                for (char switched : record[index])
                    if (switched != 0) {
                        ++ switches;
                        last_switch = index;
                    }
            if (switches <= 1 && !tower_stands_alone)
                cost.tower = 0.;
            else {
                cost.tower = handoff.tower_seconds;
                if (!tower_stands_alone)
                    for (size_t index = last_switch + 1; index < committed_rows.size(); ++ index)
                        cost.tower -= committed_rows[index].loose_seconds;
            }
            return cost;
        };
        const PlanCost plan = price_plan(0);
        const PlanCost fine = price_plan(kept.size());
        // The plan has to win by more than the estimate can be trusted to. Its error grows with the
        // print, so the margin is a share of the plan, and never under a minute.
        const double margin_seconds = std::max(60., 0.02 * plan.total());
        // The tower only has to reach the last switch, so a band above the one below it carries the
        // tower levels between the two. The best plan is looked for among the ones that send the
        // top bands fine, one more at a time: an isolated high band, or a cluster of them, whose
        // saving does not cover the height it forces goes, and the tower's top comes down with it.
        // A plan the bands already beat keeps every band.
        size_t best = 0;
        double best_total = plan.total();
        PlanCost best_cost = plan;
        if (plan.available)
            for (size_t trimmed = 1; trimmed < kept.size(); ++ trimmed) {
                const PlanCost candidate = price_plan(trimmed);
                if (candidate.available && candidate.total() < best_total - EPSILON) {
                    best = trimmed;
                    best_total = candidate.total();
                    best_cost = candidate;
                }
            }
        size_t going = 0;
        const char *reason = nullptr;
        if (plan.available && fine.available) {
            if (!(best_total < fine.total() - margin_seconds)) {
                going = kept.size();
                reason = "whole_print_slower_than_fine";
            } else if (best > 0) {
                going = best;
                reason = "tower_height_costs_more_than_band_saves";
            }
        }
        const double chosen_seconds = going == 0 ? plan.total() : going == best ? best_total : fine.total();
        const PlanCost &chosen = going == 0 ? plan : going == best ? best_cost : fine;
        if (going > 0 && chosen.available)
            reprice_model = chosen.model;
        {
            std::ostringstream digest;
            digest.precision(12);
            const char *decision = !(plan.available && fine.available) ? "unpriced" :
                                   going == 0 ? "kept" : going == best ? "trimmed" : "all_fine";
            digest << "whole_print decision=" << decision << " plan=" << plan.total()
                   << " all_fine=" << fine.total() << " margin=" << margin_seconds
                   << " best_trim=" << best << " best=" << best_total
                   << " chosen=" << chosen_seconds
                   << " coarse_bands=" << kept.size()
                   << " plan_model=" << plan.model << " plan_tower=" << plan.tower
                   << " plan_switches=" << plan.switches
                   << " all_fine_model=" << fine.model << " all_fine_tower=" << fine.tower
                   << " all_fine_switches=" << fine.switches
                   << " chosen_model=" << chosen.model << " chosen_tower=" << chosen.tower
                   << " chosen_switches=" << chosen.switches
                   << " committed_tower=" << m_feature_economics_committed_tower_seconds << "\n";
            for (size_t at = 0; at < going; ++ at) {
                const KeptBand &entry = kept[order[at]];
                digest << "whole_print fine object=" << entry.object->model_object()->name
                       << " z=" << entry.band->z_low << ":" << entry.band->z_high
                       << " fine_model=" << entry.fine_model << " coarse_model=" << entry.coarse_model
                       << " reason=" << reason << "\n";
            }
            m_feature_economics_whole_print_digest = digest.str();
        }
        // The digest goes to the log line by line, so a command line slice shows what the check saw.
        {
            std::istringstream lines(m_feature_economics_whole_print_digest);
            for (std::string line; std::getline(lines, line); )
                BOOST_LOG_TRIVIAL(info) << "Feature infill time digest: " << line;
        }
        BOOST_LOG_TRIVIAL(info) << "Feature infill time, whole print: plan " << plan.total()
            << " s against all fine " << fine.total() << " s over " << kept.size() << " coarse bands, "
            << (going == 0 ? "plan kept" : "bands sent fine: ") << (going == 0 ? "" : reason);
        for (size_t at = 0; at < going; ++ at) {
            KeptBand &entry = kept[order[at]];
            PrintObject *object = entry.object;
            AdvisorFeatureBand &band = *entry.band;
            std::vector<Layer *> layers = entry.own;
            if (entry.roof != nullptr)
                layers.push_back(entry.roof);
            std::vector<SavedRegion> saved;
            for (Layer *layer : layers)
                for (LayerRegion *region : layer->regions())
                    saved.push_back({region, region->fill_surfaces, region->fills, region->feature_split_sparse_fine_owned()});
            // The same re-fill a band's own trial makes on the deciding pass: its layers from the
            // surfaces captured before combining, fine owned, and the roof's extra protection
            // taken off again.
            try {
                for (size_t layer_at = 0; layer_at < entry.own.size(); ++ layer_at) {
                    throw_if_canceled();
                    ++ m_feature_economics_refilled_layers;
                    Layer *layer = entry.own[layer_at];
                    LayerRegion *region = layer->get_region(int(band.region_id));
                    region->fill_surfaces = object->m_feature_uncombined_surfaces[entry.own_index[layer_at]][band.region_id];
                    region->set_feature_split_sparse_fine_owned(true);
                    layer->make_fills(object->m_adaptive_fill_octrees.first.get(),
                        object->m_adaptive_fill_octrees.second.get(), object->m_lightning_generator.get());
                    layer->make_ironing();
                    layer->simplify_infill_extrusion_path();
                }
                if (entry.roof != nullptr) {
                    throw_if_canceled();
                    ++ m_feature_economics_refilled_layers;
                    LayerRegion *region = entry.roof->get_region(int(band.region_id));
                    const auto &original = object->m_feature_uncombined_surfaces[entry.roof_index][band.region_id];
                    Surfaces surfaces;
                    for (const Surface &surface : region->fill_surfaces.surfaces)
                        if (surface.surface_type == stInternal || surface.surface_type == stInternalVoid)
                            surfaces.push_back(surface);
                    for (const Surface &surface : original.surfaces)
                        if (surface.surface_type != stInternal && surface.surface_type != stInternalVoid)
                            surfaces.push_back(surface);
                    region->fill_surfaces.surfaces = std::move(surfaces);
                    entry.roof->make_fills(object->m_adaptive_fill_octrees.first.get(),
                        object->m_adaptive_fill_octrees.second.get(), object->m_lightning_generator.get());
                    entry.roof->make_ironing();
                    entry.roof->simplify_infill_extrusion_path();
                }
            } catch (...) {
                for (auto &region : saved) {
                    region.region->fill_surfaces = std::move(region.surfaces);
                    region.region->fills = std::move(region.fills);
                    region.region->set_feature_split_sparse_fine_owned(region.fine);
                }
                throw;
            }
            band.outcome = AdvisorFeatureOutcome::FineFallback;
            band.reason = reason;
            accepted.push_back(AcceptedBand{&band, std::move(saved)});
        }
    }

    if (accepted.empty())
        return;
    // One tower build for the plan the bands agreed on, not one per trial.
    try {
        _make_wipe_tower();
    } catch (const Slic3r::SlicingError &error) {
        // That plan has no buildable tower. Put every band that went fine back on the coarse
        // tool and rebuild the tower the committed plan already had, so the print still slices.
        for (auto entry = accepted.rbegin(); entry != accepted.rend(); ++ entry) {
            for (auto &region : entry->saved) {
                region.region->fill_surfaces = std::move(region.surfaces);
                region.region->fills = std::move(region.fills);
                region.region->set_feature_split_sparse_fine_owned(region.fine);
            }
            entry->band->outcome = AdvisorFeatureOutcome::Committed;
            entry->band->reason = std::string("fine_tower_unavailable: ") + error.what();
        }
        _make_wipe_tower();
        return;
    }
    // A plan that sent bands fine at the whole-print check now has its own, shorter tower. Its
    // price on that tower is what the print should take, and goes to the digest and the log.
    if (reprice_model >= 0.) {
        reindex_published_kept();
        const HandoffCost rebuilt = price_sequence({}, 0, nullptr, nullptr, nullptr);
        if (rebuilt.available) {
            std::ostringstream line;
            line.precision(12);
            line << "whole_print rebuilt chosen=" << reprice_model + rebuilt.total_seconds()
                 << " chosen_model=" << reprice_model << " chosen_tower=" << rebuilt.tower_seconds
                 << " chosen_switches=" << rebuilt.switch_seconds;
            m_feature_economics_whole_print_digest += line.str() + "\n";
            BOOST_LOG_TRIVIAL(info) << "Feature infill time digest: " << line.str();
        }
    }
}

void Print::_make_wipe_tower()
{
    m_wipe_tower_data.clear();

    // BBS
    // The logical filament domain (filament_diameter), never the optional colour vector: the flush
    // slices below index by filament id, and an unset colour list would narrow the tower to the first
    // filament.
    const unsigned int number_of_extruders = (unsigned int)(m_config.filament_diameter.values.size());
    // Both slices below are blind. A matrix that cannot supply one full
    // number_of_extruders x number_of_extruders block per nozzle is a malformed *physical* matrix
    // and is refused; a longer one is fine (the stock 4x4 default sits on smaller domains).
    const auto require_flush_block = [number_of_extruders](size_t available, size_t nozzle_id) {
        const size_t needed = size_t(number_of_extruders) * number_of_extruders;
        if (available < needed)
            throw Slic3r::SlicingError(Slic3r::format(
                "The purging volume matrix supplies %1% entries for nozzle %2%, but %3% filaments need %4%. "
                "Check the printer's purging volumes.",
                available, nozzle_id + 1, number_of_extruders, needed));
    };

    const bool is_wipe_tower_type2 = this->wipe_tower_type() == WipeTowerType::Type2;
    // Let the ToolOrdering class know there will be initial priming extrusions at the start of the print.
    m_wipe_tower_data.tool_ordering = ToolOrdering(*this, (unsigned int) -1, is_wipe_tower_type2);
    m_wipe_tower_data.tool_ordering.sort_and_build_data(*this, (unsigned int)-1, is_wipe_tower_type2);

    // A Feature Split plate that never reaches the coarse nozzle gets Off's tower, so everything
    // below that asks the mode reads tower_config.
    m_wipe_tower_config_as_off.reset();
    if (m_wipe_tower_data.tool_ordering.tower_planned_as_off())
        m_wipe_tower_config_as_off = std::make_unique<PrintConfig>(mixed_nozzle_tower_config_as_off(m_config));
    const PrintConfig &tower_config = this->wipe_tower_config();

    if (!m_wipe_tower_data.tool_ordering.has_wipe_tower())
        // Don't generate any wipe tower.
        return;

    // Preserve tower-only rows as typed scheduled events. They have no fabricated support
    // geometry; GCode export merges them into its global level stream and uses the dedicated
    // tower-support processor.
    {
        size_t event_index = 0;
        for (LayerTools &lt : m_wipe_tower_data.tool_ordering.layer_tools()) {
            if (lt.has_wipe_tower)
                lt.wipe_tower_event_index = event_index++;
            if (lt.has_wipe_tower && !lt.has_object && !lt.has_support)
                lt.tower_support_event = true;
        }
    }
    this->throw_if_canceled();

    if (!is_wipe_tower_type2) {
        // in BBL machine, wipe tower is only use to prime extruder. So just use a global wipe volume.
        // Auto-pad is deliberately limited to the admitted two-tool mixed-nozzle Type1 path.
        const ConfigOptionBool *auto_pad_option = m_config.option<ConfigOptionBool>("mixed_nozzle_auto_pad");
        const bool valid_auto_pad_mapping = [&] {
            if (m_config.filament_diameter.values.empty())
                return false;
            for (size_t index = 0; index < m_config.filament_diameter.values.size(); ++index)
                if (!resolve_mixed_nozzle_tool(m_config, index, MixedNozzleResolveScope::PhysicalToolOnly))
                    return false;
            return true;
        }();
        const bool auto_pad = auto_pad_option != nullptr && auto_pad_option->value &&
                              (is_mixed_nozzle_body_split(tower_config) || is_mixed_nozzle_feature_split(tower_config)) &&
                              m_config.nozzle_diameter.values.size() == 2 &&
                              valid_auto_pad_mapping &&
                              std::none_of(m_config.extruder_max_nozzle_count.values.begin(),
                                           m_config.extruder_max_nozzle_count.values.end(),
                                           [](double count) { return count > 1.; });
        WipeTower wipe_tower(tower_config, m_plate_index, m_origin, m_wipe_tower_data.tool_ordering.first_extruder(),
                             m_wipe_tower_data.tool_ordering.empty() ? 0.f : m_wipe_tower_data.tool_ordering.back().print_z, m_wipe_tower_data.tool_ordering.all_extruders());
        // Orca: the tower's first-layer flow follows the user's first-layer flow ratio (BBS reads
        // its initial_layer_flow_ratio here — STUDIO-14254; first_layer_flow_ratio is Orca's analog,
        // default 1.0 in both). Honor the set_other_flow_ratios gate that governs the option
        // everywhere else.
        // Per-layer filament->nozzle grouping. sort_and_build_data() above publishes it on the Print
        // for by-layer prints; by-object prints publish only later (psSkirtBrim), so fall back to the
        // ToolOrdering's own copy there. set_extruder() below dereferences it, so it must be set first.
        auto print_group_result = get_layered_nozzle_group_result();
        const MultiNozzleUtils::LayeredNozzleGroupResult &nozzle_group_result =
            print_group_result ? *print_group_result : m_wipe_tower_data.tool_ordering.get_layered_nozzle_group_result();
        auto configure_wipe_tower = [&](WipeTower &tower) {
            // Orca: the tower's first-layer flow follows first_layer_flow_ratio (Orca's analogue of BBS
            // initial_layer_flow_ratio), under the set_other_flow_ratios switch that governs it elsewhere.
            tower.set_first_layer_flow_ratio(m_default_object_config.set_other_flow_ratios
                                                 ? float(m_default_region_config.first_layer_flow_ratio)
                                                 : 1.f);
            tower.set_has_tpu_filament(this->has_tpu_filament());
            tower.set_nozzle_group_result(nozzle_group_result);
            // A Body plate on the lagging tower needs the compact tower it rests on, whatever the project
            // saved, as Feature Split gets it.
            if (m_wipe_tower_data.tool_ordering.mixed_nozzle_body_tower_lags())
                tower.set_no_sparse_layers(true);
            {
                // Orca: acceleration options are object-scope (PrintConfig members in BBS), so
                // resolve the per-variant columns here; initial_layer_travel_acceleration is
                // FloatOrPercent over travel_acceleration and needs the full config to resolve.
                std::vector<double> first_layer_travel_accels;
                for (size_t i = 0; i < m_config.initial_layer_travel_acceleration.values.size(); ++i)
                    first_layer_travel_accels.emplace_back(m_full_print_config.get_abs_value_at("initial_layer_travel_acceleration", i));
                tower.set_accelerations(m_default_object_config.default_acceleration.values,
                                        m_default_object_config.initial_layer_acceleration.values,
                                        m_default_object_config.travel_acceleration.values,
                                        first_layer_travel_accels);
            }
            // Feed the has_filament_switcher device flag (a develop-only dynamic key no shipping profile
            // sets, read defensively from the full config) and the shared printable bed used by the PETG
            // pre-extrusion offset clamp. Both are inert unless has_filament_switcher is set.
            {
                const ConfigOptionBool* hfs = m_full_print_config.option<ConfigOptionBool>("has_filament_switcher");
                tower.set_has_filament_switcher(hfs && hfs->value);
            }
            tower.set_shared_print_bed(this->get_extruder_shared_printable_polygon());
            // Set the extruder & material properties at the wipe tower object.
            for (size_t i = 0; i < number_of_extruders; ++i)
                tower.set_extruder(i, tower_config);
        };
        configure_wipe_tower(wipe_tower);

        // BBS: remove priming logic
        // m_wipe_tower_data.priming = Slic3r::make_unique<std::vector<WipeTower::ToolChangeResult>>(
        //    wipe_tower.prime((float)this->skirt_first_layer_height(), m_wipe_tower_data.tool_ordering.all_extruders(), false));

        // The recorder/flush pass below is the only mutable event capture. Auto-pad never reruns
        // it while trying widths; it replays these immutable records into independent Type1
        // planners instead.
        std::vector<WipeTowerPlanEvent> plan_events;
        std::set<int> used_filament_ids;

        // Lets go through the wipe tower layers and determine pairs of extruder changes for each
        // to pass to wipe_tower (so that it can use it for planning the layout of the tower)
        {
        // Get wiping matrix to get number of extruders and convert vector<double> to vector<float>:
        bool               is_mutli_extruder = m_config.nozzle_diameter.values.size() > 1;
        size_t nozzle_nums = m_config.nozzle_diameter.values.size();
        using FlushMatrix = std::vector<std::vector<float>>;
        std::vector<FlushMatrix> multi_extruder_flush;
        for (size_t nozzle_id = 0; nozzle_id < nozzle_nums; ++nozzle_id) {
            std::vector<float> flush_matrix(cast<float>(get_flush_volumes_matrix(m_config.flush_volumes_matrix.values, nozzle_id, nozzle_nums)));
            require_flush_block(flush_matrix.size(), nozzle_id);
            std::vector<std::vector<float>> wipe_volumes;
            for (unsigned int i = 0; i < number_of_extruders; ++i)
                wipe_volumes.push_back(std::vector<float>(flush_matrix.begin() + i * number_of_extruders, flush_matrix.begin() + (i + 1) * number_of_extruders));

            multi_extruder_flush.emplace_back(wipe_volumes);
        }

        // Per-carousel-slot purge tracking via NozzleStatusRecorder (BBS pattern); the layered
        // group result set on the tower above resolves each filament to its nozzle slot per layer.
        MultiNozzleUtils::NozzleStatusRecorder nozzle_recorder;

        std::vector<int>filament_maps = get_filament_maps();
        int layer_idx = -1;

        unsigned int current_filament_id = m_wipe_tower_data.tool_ordering.first_extruder();
        // Initialize NozzleStatusRecorder with the first filament's carousel slot
        {
            auto nozzle = nozzle_group_result.get_nozzle_for_filament(current_filament_id, layer_idx);
            if (nozzle)
                nozzle_recorder.set_nozzle_status(nozzle->group_id, current_filament_id, nozzle->extruder_id);
        }

        // A running estimate of print time, so a nozzle that returns to the tower holding its own filament
        // can be re-primed for how long it sat parked. Each layer's model work is priced per filament from
        // its own paths: the role's speed from the region, capped by the filament's flow limit, as the
        // Feature Split economics price fills. Travel, acceleration and cooling slowdown are left out, so
        // the wait is a low estimate and the re-prime errs small. Only mixed-nozzle slicing pays for it.
        const bool reprime_estimate = is_mixed_nozzle_slicing_enabled(tower_config);
        std::vector<std::pair<double, const Layer *>> reprime_layers;
        if (reprime_estimate) {
            for (const PrintObject *object : m_objects)
                for (const Layer *layer : object->layers())
                    reprime_layers.emplace_back(layer->print_z, layer);
            std::sort(reprime_layers.begin(), reprime_layers.end(),
                [](const std::pair<double, const Layer *> &lhs, const std::pair<double, const Layer *> &rhs) { return lhs.first < rhs.first; });
        }
        const auto reprime_layer_work = [&](const LayerTools &tools) {
            std::map<unsigned int, double> seconds;
            auto at = std::lower_bound(reprime_layers.begin(), reprime_layers.end(), tools.print_z - EPSILON,
                [](const std::pair<double, const Layer *> &entry, double value) { return entry.first < value; });
            for (; at != reprime_layers.end() && at->first <= tools.print_z + EPSILON; ++at) {
                const Layer *layer = at->second;
                const double copies = double(std::max<size_t>(1, layer->object()->instances().size()));
                for (const LayerRegion *layerm : layer->regions()) {
                    const PrintRegionConfig &rc = layerm->region().config();
                    for (const ExtrusionEntityCollection *owner : {&layerm->perimeters, &layerm->fills})
                        for (const ExtrusionEntity *group : owner->entities) {
                            const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(group);
                            if (collection == nullptr)
                                continue;
                            const unsigned int filament = tools.extruder(*collection, layerm->region(), layerm);
                            const int nozzle   = get_nozzle_config_index(int(filament), layer->id());
                            const int material = get_filament_config_indx(int(filament), layer->id());
                            double cap = m_config.filament_max_volumetric_speed.get_at(material);
                            const double flow_ratio = m_config.filament_flow_ratio.get_at(material);
                            if (flow_ratio > 0.)
                                cap /= flow_ratio;
                            const auto path_seconds = [&](const ExtrusionPath &path) {
                                double speed = 0.;
                                switch (path.role()) {
                                case erExternalPerimeter:
                                case erOverhangPerimeter:    speed = rc.outer_wall_speed.get_at(nozzle); break;
                                case erPerimeter:            speed = rc.inner_wall_speed.get_at(nozzle); break;
                                case erInternalInfill:       speed = rc.sparse_infill_speed.get_at(nozzle); break;
                                case erSolidInfill:
                                case erBottomSurface:        speed = rc.internal_solid_infill_speed.get_at(nozzle); break;
                                case erTopSolidInfill:       speed = rc.top_surface_speed.get_at(nozzle); break;
                                case erBridgeInfill:
                                case erInternalBridgeInfill: speed = rc.bridge_speed.get_at(nozzle); break;
                                case erGapFill:              speed = rc.gap_infill_speed.get_at(nozzle); break;
                                default: break;
                                }
                                double rate = (std::isfinite(speed) && speed > 0.) ? speed * path.mm3_per_mm : 0.;
                                if (std::isfinite(cap) && cap > 0.)
                                    rate = rate > 0. ? std::min(rate, cap) : cap;
                                return rate > 0. ? path.total_volume() / rate : 0.;
                            };
                            double work = 0.;
                            const ExtrusionEntityCollection flat = collection->flatten();
                            for (const ExtrusionEntity *entity : flat.entities) {
                                if (const auto *path = dynamic_cast<const ExtrusionPath *>(entity))
                                    work += path_seconds(*path);
                                else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(entity))
                                    for (const ExtrusionPath &part : multi->paths) work += path_seconds(part);
                                else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(entity))
                                    for (const ExtrusionPath &part : loop->paths) work += path_seconds(part);
                            }
                            if (std::isfinite(work))
                                seconds[filament] += work * copies;
                        }
                }
            }
            return seconds;
        };
        double reprime_clock = 0.;
        // Physical tool -> the clock when it last handed over to another tool.
        std::map<int, double> reprime_parked_since;
        const double reprime_switch_seconds = std::max(0., double(m_config.machine_tool_change_time.value));

        for (auto& layer_tools : m_wipe_tower_data.tool_ordering.layer_tools()) { // for all layers
            ++layer_idx;

            const std::map<unsigned int, double> reprime_work =
                reprime_estimate ? reprime_layer_work(layer_tools) : std::map<unsigned int, double>{};
            const auto reprime_work_of = [&reprime_work](unsigned int filament) {
                const auto it = reprime_work.find(filament);
                return it == reprime_work.end() ? 0. : it->second;
            };
            if (!layer_tools.has_wipe_tower) {
                for (const auto &work : reprime_work)
                    reprime_clock += work.second;
                continue;
            }
            bool first_layer = &layer_tools == &m_wipe_tower_data.tool_ordering.front();
            // force_emit marks a compact-mode filler layer (LayerTools::wipe_tower_emit,
            // ToolOrdering::fill_wipe_tower_partitions) that must still deposit its wall and structure
            // although no real switch happens here.
            const bool disable_last_layer_fill =
                (m_config.enable_wrapping_detection || enable_timelapse_print()) &&
                layer_tools.wipe_tower_partitions == 0;
            plan_events.push_back(WipeTowerPlanEvent{
                float(layer_tools.print_z), float(layer_tools.wipe_tower_layer_height),
                current_filament_id, current_filament_id, 0.f, 0.f, 0.f,
                layer_tools.wipe_tower_emit, disable_last_layer_fill,
                current_filament_id, current_filament_id, -1, -1, false, false, -1.f});
            if (!auto_pad)
                wipe_tower.plan_toolchange((float)layer_tools.print_z, (float)layer_tools.wipe_tower_layer_height, current_filament_id, current_filament_id,
                    0.f, 0.f, 0.f, layer_tools.wipe_tower_emit);

            used_filament_ids.insert(layer_tools.extruders.begin(), layer_tools.extruders.end());

            for (const auto filament_id : layer_tools.extruders) {
                if (filament_id == current_filament_id) {
                    reprime_clock += reprime_work_of(filament_id);
                    continue;
                }

                float volume_to_purge = 0;
                // How long the incoming nozzle has been parked, through the end of this switch.
                double reprime_idle = -1.;
                // Destination-tool state captured before set_nozzle_status() below, so conditioning sees what
                // the destination held when the switch begins.
                std::optional<float> conditioned_extruder_change_prime;
                // When conditioning selects its automatic Saving candidate, replace it with the destination
                // nozzle's own handoff prime. Kept separate so an explicit per-tool conditioning value still
                // wins.
                std::optional<float> handoff_extruder_change_prime;
                bool automatic_handoff_extruder_change_prime = false;
                // The outgoing tool does the ramming, so plan_toolchange needs its own resolved deposit, not
                // the destination's.
                std::optional<float> ram_length_override;
                bool automatic_ram_length = false;
                int outgoing_extruder = -1;
                int incoming_extruder = -1;

                // Per-carousel-slot purge tracking via NozzleStatusRecorder
                {
                    auto nozzle_info = nozzle_group_result.get_nozzle_for_filament(filament_id, layer_idx);
                    if (nozzle_info) {
                        int extruder_id = nozzle_info->extruder_id;
                        incoming_extruder = extruder_id;
                        if (const auto parked = reprime_parked_since.find(extruder_id); parked != reprime_parked_since.end())
                            reprime_idle = reprime_clock + reprime_switch_seconds - parked->second;
                        int nozzle_id   = nozzle_info->group_id;
                        int prev_nozzle_filament = nozzle_recorder.get_filament_in_nozzle(nozzle_id);
                        const bool destination_holds_same_filament =
                            !nozzle_recorder.is_nozzle_empty(nozzle_id) && prev_nozzle_filament == static_cast<int>(filament_id);
                        const auto current_nozzle_info = nozzle_group_result.get_nozzle_for_filament(current_filament_id, layer_idx);
                        const bool physical_tool_switch = current_nozzle_info && current_nozzle_info->extruder_id != extruder_id;
                        // On the H2C's rack, whether this switch lands on the hotend already mounted
                        // on the incoming tool, and whether it leaves the outgoing tool's hotend in
                        // place. A tool with one hotend always does both.
                        const bool incoming_hotend_stays = nozzle_recorder.get_nozzle_in_extruder(extruder_id) == nozzle_id;
                        const bool outgoing_hotend_stays = !current_nozzle_info ||
                            current_nozzle_info->extruder_id != extruder_id || current_nozzle_info->group_id == nozzle_id;
                        if (extruder_id >= 0)
                            conditioned_extruder_change_prime = mixed_nozzle_conditioned_extruder_change_prime(
                                tower_config, filament_id, size_t(extruder_id), physical_tool_switch, destination_holds_same_filament,
                                incoming_hotend_stays);
                        // The conditioned helper carries the qualification, warm-return, mode,
                        // provenance, and ordinary-filament-prime guards. Only its unset
                        // per-tool value selects the automatic Saving candidate; an explicit
                        // per-tool value must continue to win over the physical resolver.
                        if (conditioned_extruder_change_prime &&
                            (size_t(extruder_id) >= m_config.mixed_nozzle_extruder_change_prime_volume.values.size() ||
                             m_config.mixed_nozzle_extruder_change_prime_volume.is_nil(size_t(extruder_id)))) {
                            const auto handoff = mixed_nozzle_handoff_deposits(
                                tower_config, filament_id, size_t(extruder_id), -1.,
                                auto_pad ? double(layer_tools.wipe_tower_layer_height) : -1., incoming_hotend_stays);
                            if (handoff) {
                                if (auto_pad)
                                    automatic_handoff_extruder_change_prime = true;
                                else
                                    handoff_extruder_change_prime = mixed_nozzle_idle_reprime_volume(
                                        tower_config, filament_id, handoff->prime_volume_mm3, reprime_idle);
                            }
                        }
                        if (current_nozzle_info && current_nozzle_info->extruder_id >= 0) {
                            outgoing_extruder = current_nozzle_info->extruder_id;
                            const auto handoff = mixed_nozzle_handoff_deposits(
                                tower_config, current_filament_id, size_t(current_nozzle_info->extruder_id), -1.,
                                auto_pad ? double(layer_tools.wipe_tower_layer_height) : -1., outgoing_hotend_stays);
                            if (handoff) {
                                if (auto_pad)
                                    automatic_ram_length = true;
                                else
                                    ram_length_override = handoff->ram_length_mm;
                            }
                        }

                        if (!nozzle_recorder.is_nozzle_empty(nozzle_id) &&
                            static_cast<int>(filament_id) != prev_nozzle_filament) {
                            volume_to_purge = multi_extruder_flush[extruder_id][prev_nozzle_filament][filament_id];
                            // Fast purge mode uses flush_multiplier_fast; Default is inert.
                            float flush_multiplier = (m_config.prime_volume_mode == PrimeVolumeMode::pvmFast)
                                ? m_config.flush_multiplier_fast.get_at(extruder_id)
                                : m_config.flush_multiplier.get_at(extruder_id);
                            volume_to_purge *= flush_multiplier;
                            volume_to_purge = layer_tools.wiping_extrusions().mark_wiping_extrusions(
                                *this, current_filament_id, filament_id, volume_to_purge);
                        }
                        nozzle_recorder.set_nozzle_status(nozzle_id, filament_id, extruder_id);
                    }
                }

                //During the filament change, the extruder will extrude an extra length of grab_length for the corresponding detection, so the purge can reduce this length.
                int grab_extruder_id = filament_maps[filament_id] - 1;
                float grab_purge_volume = m_config.grab_length.get_at(grab_extruder_id) * 2.4; //(diameter/2)^2*PI=2.4
                volume_to_purge = std::max(0.f, volume_to_purge - grab_purge_volume);

                // Prime volume per-filament: the tower now picks extruder-change vs nozzle-change
                // (carousel) internally per plan layer, so pass both candidates (BBS pattern).
                auto prime_volumes = mixed_nozzle_vendor_prime_volumes(m_config, filament_id);
                if (handoff_extruder_change_prime)
                    prime_volumes.extruder_change = *handoff_extruder_change_prime;
                else if (conditioned_extruder_change_prime)
                    prime_volumes.extruder_change = *conditioned_extruder_change_prime;

                plan_events.push_back(WipeTowerPlanEvent{
                    float(layer_tools.print_z), float(layer_tools.wipe_tower_layer_height),
                    current_filament_id, filament_id,
                    prime_volumes.extruder_change, prime_volumes.nozzle_change, volume_to_purge,
                    false, false, current_filament_id, filament_id, outgoing_extruder, incoming_extruder,
                    automatic_ram_length, automatic_handoff_extruder_change_prime,
                    ram_length_override ? *ram_length_override : -1.f});
                plan_events.back().idle_seconds = float(reprime_idle);
                // Clock: the outgoing tool parks when this switch starts; the switch, the incoming tool's own
                // tower work and then its model work on this layer follow.
                if (outgoing_extruder >= 0 && outgoing_extruder != incoming_extruder)
                    reprime_parked_since[outgoing_extruder] = reprime_clock;
                reprime_clock += reprime_switch_seconds;
                {
                    const double cap = m_config.filament_max_volumetric_speed.get_at(filament_id);
                    if (std::isfinite(cap) && cap > 0.)
                        reprime_clock += double(std::max(0.f, prime_volumes.extruder_change + volume_to_purge)) / cap;
                }
                reprime_clock += reprime_work_of(filament_id);
                if (!auto_pad)
                    wipe_tower.plan_toolchange((float)layer_tools.print_z, (float)layer_tools.wipe_tower_layer_height, current_filament_id, filament_id,
                        prime_volumes.extruder_change, prime_volumes.nozzle_change, volume_to_purge, false,
                        ram_length_override ? *ram_length_override : -1.f);
                current_filament_id = filament_id;
            }
            layer_tools.wiping_extrusions().ensure_perimeters_infills_order(*this);

            // if enable timelapse, slice all layer
            if (m_config.enable_wrapping_detection || enable_timelapse_print()) {
                if (layer_tools.wipe_tower_partitions == 0 && !auto_pad) wipe_tower.set_last_layer_extruder_fill(false);
                continue;
            }

            if (&layer_tools == &m_wipe_tower_data.tool_ordering.back() || (&layer_tools + 1)->wipe_tower_partitions == 0)
                break;
        }
        }

        // The lagging prime tower. How far it may sit below the part is the printer's headroom under the
        // gantry rod; whether it may lag at all depends on this plate, because a lagged visit brings the
        // nozzle well down while the toolhead body stays where it is. Both come from the printer config
        // and the saved project, never hard-coded, so the engine and the preview get the same number.
        m_mixed_nozzle_tower_lag_max = 0.f;
        m_mixed_nozzle_tower_lag_notice.clear();
        int   lag_coarse_tool   = -1;
        float lag_coarse_height = 0.f;
        float lag_fine_height   = 0.f;
        // A Body Split plate on the lagging tower (ToolOrdering decides it from the objects' heights) gets
        // the same lag and the same notice as Feature Split.
        const ToolOrdering &lag_ordering = m_wipe_tower_data.tool_ordering;
        // The lagging schedule asks which nozzle the arriving tool is on, so it gets the map rather than
        // one filament id, resolved through the engine's shared nozzle resolver.
        int              lag_coarse_extruder = -1;
        std::vector<int> lag_filament_extruders;
        if (is_mixed_nozzle_feature_split(tower_config) || lag_ordering.mixed_nozzle_body_tower_lags()) {
            float plan_height = 0.f;
            for (const WipeTowerPlanEvent &event : plan_events)
                if (std::isfinite(event.z))
                    plan_height = std::max(plan_height, event.z);
            // Qualified: Print has a member of the same name that answers the resolved value.
            const float headroom = Slic3r::mixed_nozzle_tower_lag_max(float(m_config.extruder_clearance_height_to_rod.value));
            // The footprint the toolhead has to clear. The generated rectangle is not known until
            // the planner has run, so this takes the largest one the planner could return, plus
            // the brim: too generous only ever costs the lag, never safety.
            const float span = std::max({float(m_config.prime_tower_width.value),
                                         WipeTower::get_limit_depth_by_height(plan_height),
                                         mixed_nozzle_tower_footprint_floor(plan_height)});
            const float brim = m_config.prime_tower_brim_width.value < 0.
                ? mixed_nozzle_tower_brim_width(plan_height)
                : float(m_config.prime_tower_brim_width.value);
            const Vec2d corner(m_config.wipe_tower_x.get_at(m_plate_index),
                               m_config.wipe_tower_y.get_at(m_plate_index));
            const BoundingBoxf tower_footprint(corner - Vec2d(brim, brim),
                                               corner + Vec2d(span + brim, span + brim));
            std::vector<BoundingBoxf> object_footprints;
            for (const PrintObject *object : m_objects) {
                const BoundingBox box = object->bounding_box();
                for (const PrintInstance &instance : object->instances()) {
                    const Vec2d shift = unscale(instance.shift_without_plate_offset());
                    object_footprints.emplace_back(unscale(box.min) + shift, unscale(box.max) + shift);
                }
            }
            // The H2D machine JSON uses the legacy key "extruder_clearance_max_radius", which
            // PrintConfigDef::handle_legacy renames on load, so this is the member to read.
            const float radius = float(m_config.extruder_clearance_radius.value);
            // The schedule never needs more than the two coarse layers one visit can lay, so a
            // plate that cannot give the toolhead its clearance still lags; it just gets the
            // smaller ceiling. There is no keep-pace fallback.
            lag_coarse_height = float(lag_ordering.mixed_nozzle_body_tower_lags() ?
                lag_ordering.mixed_nozzle_lag_coarse_height() : m_config.mixed_nozzle_coarse_layer_height.value);
            lag_fine_height   = float(m_default_object_config.layer_height.value);
            const float crowded_ceiling = 2.f * lag_coarse_height;
            const bool  clear = mixed_nozzle_tower_lag_placement_clear(tower_footprint, object_footprints, radius);
            m_mixed_nozzle_tower_lag_max = clear ? std::max(headroom, crowded_ceiling) : crowded_ceiling;
            m_mixed_nozzle_tower_lag_notice = clear
                ? Slic3r::format("; MIXED_NOZZLE_TOWER_LAG: %1% mm below the part",
                                 m_mixed_nozzle_tower_lag_max)
                : Slic3r::format("; MIXED_NOZZLE_TOWER_LAG: %1% mm below the part, "
                                 "an object stands within %2% mm of the tower",
                                 m_mixed_nozzle_tower_lag_max, radius);
            // A project saved without this setting carries the compact tower switched off, and Feature Split
            // supplies it anyway because a tower visited only at switches has no sparse layers to allow. Say
            // so, so the project and the slice do not appear to disagree.
            if (mixed_nozzle_compact_tower_applied_by_mode(m_config))
                m_mixed_nozzle_tower_lag_notice += ", compact tower applied by Feature Split";
            // The coarse tool builds the tower; every other tool only primes on it. The scheduler
            // asks the same question through the same helper, so the two cannot disagree about
            // which prints get this tower.
            if ((mixed_nozzle_tower_lagging(m_config) || lag_ordering.mixed_nozzle_body_tower_lags()) &&
                lag_fine_height > 0.f) {
                lag_coarse_tool     = mixed_nozzle_tower_coarse_filament(m_config);
                lag_coarse_extruder = mixed_nozzle_tower_coarse_extruder(m_config);
                // Every slot the map carries, whether the schedule uses it or not, so the
                // schedule can ask about whichever filament arrives.
                lag_filament_extruders.assign(m_config.filament_map.values.size(), -1);
                for (size_t filament = 0; filament < lag_filament_extruders.size(); ++ filament)
                    if (const std::optional<size_t> extruder =
                            physical_extruder_for_filament(m_config, unsigned(filament)))
                        lag_filament_extruders[filament] = int(*extruder);
            }
        }
        // Only the tower reads these. With no coarse tool it keeps pace, like every tower that is not
        // mixed nozzle.
        const auto apply_tower_lag = [&](WipeTower &tower) {
            tower.set_lagging(m_mixed_nozzle_tower_lag_max, lag_coarse_tool, lag_filament_extruders,
                              lag_coarse_extruder, lag_coarse_height, lag_fine_height);
        };

        std::vector<int> categories;
        for (size_t i = 0; i < m_config.filament_adhesiveness_category.values.size(); ++i) {
            categories.push_back(m_config.filament_adhesiveness_category.get_at(i));
        }

        std::unique_ptr<WipeTower> auto_wipe_tower;
        WipeTower *final_wipe_tower = &wipe_tower;
        bool structural_gap_rejected = false;
        std::string structural_rejection;
        // The rejection above is written for the tower's own tests. When a Body Split plate has
        // support on the coarser nozzle (the case the rc4 sweep hit), say what the user can change.
        const auto tower_rejection_for_user = [this](const std::string &rejection) {
            if (m_config.nozzle_diameter.values.size() != 2)
                return rejection;
            // The tower keeps materials that do not stick to each other (PLA and PETG, say) in
            // separate blocks. When the two nozzles share no material, the fine nozzle's block is
            // not laid on the levels the coarse nozzle takes, and the next fine level has no
            // support under it. Say which materials and what to change.
            if (is_mixed_nozzle_feature_split(m_config) || is_mixed_nozzle_body_split(m_config)) {
                std::array<std::set<int>, 2> categories;
                std::array<std::vector<std::string>, 2> types;
                for (const unsigned int filament : this->extruders()) {
                    const std::optional<size_t> extruder = physical_extruder_for_filament(m_config, filament);
                    if (!extruder || *extruder > 1 || filament >= m_config.filament_adhesiveness_category.values.size())
                        continue;
                    categories[*extruder].insert(m_config.filament_adhesiveness_category.values[filament]);
                    const std::string type = filament < m_config.filament_type.values.size() ? m_config.filament_type.values[filament] : std::string();
                    if (!type.empty() && std::find(types[*extruder].begin(), types[*extruder].end(), type) == types[*extruder].end())
                        types[*extruder].push_back(type);
                }
                const bool shared = std::any_of(categories[0].begin(), categories[0].end(),
                                                [&categories](int category) { return categories[1].count(category) > 0; });
                if (!categories[0].empty() && !categories[1].empty() && !shared) {
                    const auto side = [this, &types](size_t extruder) {
                        std::string names;
                        for (const std::string &type : types[extruder])
                            names += (names.empty() ? "" : " and ") + type;
                        return (names.empty() ? std::string("one material") : names) + " on the " +
                               Slic3r::float_to_string_decimal_point(m_config.nozzle_diameter.values[extruder], 1) + " mm nozzle";
                    };
                    return "The prime tower cannot be built with " + side(0) + " and " + side(1) +
                           ": these materials do not stick to each other, so the tower keeps them apart and "
                           "misses levels. Use the same kind of material on both nozzles (for example PLA for the "
                           "walls and the sparse infill) and slice again.\n" + rejection;
                }
            }
            if (!is_mixed_nozzle_body_split(m_config))
                return rejection;
            const double coarse = std::max(m_config.nozzle_diameter.values[0], m_config.nozzle_diameter.values[1]);
            for (const PrintObject *object : m_objects) {
                if (!object->has_support())
                    continue;
                const int base = object->config().support_filament.value;
                const std::optional<size_t> extruder = base > 0 ?
                    physical_extruder_for_filament(m_config, unsigned(base - 1)) : std::nullopt;
                if (extruder && *extruder < m_config.nozzle_diameter.values.size() &&
                    std::abs(m_config.nozzle_diameter.values[*extruder] - coarse) < EPSILON)
                    return "The prime tower cannot be planned with the support base on the " +
                           Slic3r::float_to_string_decimal_point(coarse, 1) +
                           " mm nozzle on this Body Split plate. Set Support/raft base to a filament on the "
                           "finer nozzle and slice again.\n" + rejection;
            }
            return rejection;
        };
        // Validate the writer's typed structural records against the actual physical nozzle that
        // emitted each road. The bed is the first predecessor. Block-local purge/ramming roads
        // provide interior continuity only; they cannot support the perimeter/rib domain. A false
        // result rejects an auto-pad candidate, while an unresolved/nonfinite binding throws a
        // named slicing diagnostic before any candidate or manual tower is accepted.
        auto audit_structural_emissions = [&](const std::vector<std::vector<WipeTower::ToolChangeResult>> &changes) {
            if (!is_mixed_nozzle_feature_split(tower_config) && !is_mixed_nozzle_body_split(tower_config))
                return true;
            std::map<unsigned int, float> previous_z_by_domain;
            std::map<unsigned int, std::vector<const WipeTower::StructuralEmission *>> earlier_by_domain;
            // Held roads above their block's support chain that no road has reached yet.
            std::map<unsigned int, std::vector<const WipeTower::StructuralEmission *>> held_by_domain;
            double largest_nozzle_diameter = 0.;
            for (double diameter : m_config.nozzle_diameter.values)
                if (std::isfinite(diameter) && diameter > 0.)
                    largest_nozzle_diameter = std::max(largest_nozzle_diameter, diameter);
            for (const auto &layer : changes) {
                std::map<unsigned int, float> current_z_by_domain;
                for (const WipeTower::ToolChangeResult &change : layer)
                    for (const WipeTower::StructuralEmission &emission : change.structural_emissions) {
                        if (emission.role == WipeTower::StructuralRole::None)
                            continue;
                        // Interior purge/ramming roads are support evidence only for their own
                        // block domain. They must never satisfy the shared perimeter/rib chain,
                        // and a missing block identity is an unsupported writer contract.
                        if (emission.role == WipeTower::StructuralRole::InteriorDeposit &&
                            emission.support_domain < 2)
                            throw Slic3r::SlicingError("[SRL-TOWER-STRUCTURE] Interior deposit has no block support domain.");
                        if (emission.role == WipeTower::StructuralRole::TowerPerimeter &&
                            emission.support_domain != 1)
                            throw Slic3r::SlicingError("[SRL-TOWER-STRUCTURE] Perimeter has an invalid support domain.");
                        if (emission.role == WipeTower::StructuralRole::TowerBlock &&
                            emission.support_domain < 2)
                            throw Slic3r::SlicingError("[SRL-TOWER-STRUCTURE] Structural block has no support domain.");
                        const bool ordinary_structure = emission.role != WipeTower::StructuralRole::InteriorDeposit;
                        if (emission.logical_tool < 0 ||
                            size_t(emission.logical_tool) >= m_config.filament_map.values.size())
                            throw Slic3r::SlicingError("[SRL-TOWER-STRUCTURE] No effective nozzle mapping exists for an emitted structural tower path.");
                        const int nozzle = m_config.filament_map.values[size_t(emission.logical_tool)] - 1;
                        if (nozzle < 0 || size_t(nozzle) >= m_config.nozzle_diameter.values.size())
                            throw Slic3r::SlicingError(Slic3r::format(
                                "[SRL-TOWER-STRUCTURE] Filament %1% is not assigned to a nozzle, so the prime tower cannot print it.",
                                emission.logical_tool + 1));
                        const double maximum = resolved_max_layer_height(m_config, size_t(nozzle));
                        if (!std::isfinite(maximum) || maximum <= 0.)
                            throw Slic3r::SlicingError(Slic3r::format(
                                "[SRL-TOWER-STRUCTURE] Nozzle %1% has no valid maximum layer height, so the prime tower cannot be planned.",
                                nozzle + 1));
                        // Catch-up levels are appended to the same visit in emission order.
                        // Credit support already deposited in this visit as well as earlier
                        // visits; otherwise two legal steps are audited as one unsupported jump.
                        const float predecessor = std::max(previous_z_by_domain[emission.support_domain],
                                                           current_z_by_domain[emission.support_domain]);
                        // The minimum is checked as well: every road the tower lays, ramming and purge
                        // included, has to be a height its own nozzle can lay.
                        const double minimum = resolved_min_layer_height(m_config, size_t(nozzle));
                        const auto reject = [&](const char *reason) {
                            structural_rejection = Slic3r::format(
                                "[SRL-TOWER-STRUCTURE] The prime tower has no level that can carry this path: %1%. "
                                "Filament %2%, nozzle %3%, support domain %4%, Z %5% mm, "
                                "previous support Z %6% mm, deposited height %7% mm, minimum %9% mm, maximum %8% mm.",
                                reason, emission.logical_tool + 1, nozzle + 1, emission.support_domain,
                                emission.z, predecessor, emission.height, maximum, minimum);
                            return false;
                        };
                        // The tower's first layer sits on the bed at the plate's first layer height, which
                        // every tool on that layer also lays on the part, even below its nozzle's minimum.
                        // Upstream bounds initial_layer_print_height by the nozzle diameter only, never by
                        // min_layer_height, so the first layer is not held to the minimum here either. Every
                        // road above it is. Interior purge and ramming retain their existing special-flow
                        // maximum exemption; ordinary structural roads are always checked against maximum.
                        const bool on_first_layer = std::isfinite(emission.z) &&
                            double(emission.z) <= m_config.initial_layer_print_height.value + EPSILON;
                        if (!on_first_layer && std::isfinite(minimum) && minimum > 0. &&
                            std::isfinite(emission.height) && double(emission.height) < minimum - EPSILON)
                            return reject("the deposited height is under the emitting nozzle's minimum");
                        if (!std::isfinite(emission.z) || !std::isfinite(emission.height) ||
                            emission.z < 0.f || emission.height <= 0.f ||
                            (ordinary_structure && emission.height > maximum + EPSILON) ||
                            !std::isfinite(emission.footprint_min.x()) ||
                            !std::isfinite(emission.footprint_min.y()) ||
                            !std::isfinite(emission.footprint_max.x()) ||
                            !std::isfinite(emission.footprint_max.y()) ||
                            emission.footprint_min.x() > emission.footprint_max.x() ||
                            emission.footprint_min.y() > emission.footprint_max.y())
                            return reject("invalid structural geometry or deposited height");
                        // Purge/ramming rows use their own flow and intentional bridge rules.
                        // Their actual material supports this continuous interior block, but
                        // only ordinary structure is subject to this vertical support-step cap.
                        // A ramming road may start a fresh stripe directly on the bed below a
                        // higher stripe in the same block. The block-wide top is not its local
                        // predecessor. This exception requires bed contact and clear XY separation
                        // from every earlier higher road, with a nozzle-width clearance. Other
                        // backwards roads, including ordinary structure, remain unsupported.
                        bool separate_bed_stripe = false;
                        if (emission.role == WipeTower::StructuralRole::InteriorDeposit &&
                            emission.z < predecessor - EPSILON &&
                            std::abs(double(emission.z) - double(emission.height)) <= EPSILON) {
                            separate_bed_stripe = true;
                            for (const WipeTower::StructuralEmission *earlier : earlier_by_domain[emission.support_domain]) {
                                if (earlier->z <= emission.z + EPSILON)
                                    continue;
                                const double clearance = largest_nozzle_diameter;
                                const bool apart =
                                    double(emission.footprint_min.x()) > double(earlier->footprint_max.x()) + clearance ||
                                    double(earlier->footprint_min.x()) > double(emission.footprint_max.x()) + clearance ||
                                    double(emission.footprint_min.y()) > double(earlier->footprint_max.y()) + clearance ||
                                    double(earlier->footprint_min.y()) > double(emission.footprint_max.y()) + clearance;
                                if (!apart) {
                                    separate_bed_stripe = false;
                                    break;
                                }
                            }
                        }
                        if ((emission.z < predecessor - EPSILON && !separate_bed_stripe) ||
                            (ordinary_structure && emission.z - predecessor > maximum + EPSILON))
                            return reject("structural support gap exceeds the emitting nozzle limit");
                        // A lagging tower may sit below the part by as much as this plate gives the toolhead
                        // and no more. change.print_z is the part layer the block belongs to; emission.z is
                        // where the tower actually is.
                        if (m_mixed_nozzle_tower_lag_max > 0.f &&
                            double(change.print_z) - double(emission.z) >
                                double(m_mixed_nozzle_tower_lag_max) + EPSILON)
                            return reject("the tower sits further below the part than this plate allows");
                        // A held coarse prime road (or a road lifted to one) stands above its block's support chain
                        // until a road reaches its height. A road crossing it lower down (by more than the G-code's
                        // rounding) would be printed into it.
                        {
                            std::vector<const WipeTower::StructuralEmission *> &held = held_by_domain[emission.support_domain];
                            held.erase(std::remove_if(held.begin(), held.end(), [&](const WipeTower::StructuralEmission *road) {
                                return road->z <= predecessor + EPSILON; }), held.end());
                            constexpr float apart = 0.05f;
                            for (const WipeTower::StructuralEmission *road : held)
                                if (emission.z < road->z - 0.02f &&
                                    emission.footprint_min.x() < road->footprint_max.x() - apart &&
                                    road->footprint_min.x() < emission.footprint_max.x() - apart &&
                                    emission.footprint_min.y() <= road->footprint_max.y() + apart &&
                                    road->footprint_min.y() <= emission.footprint_max.y() + apart)
                                    return reject("a road would be printed under a higher road laid before it");
                            if (emission.held_level)
                                held.push_back(&emission);
                        }
                        // A ramming held above its level to reach the outgoing tool's own minimum stands on
                        // the same tower top as the level beside it, and the next coarse layer covers it. It
                        // is checked above like any road, but it is not the top of its block's support chain;
                        // otherwise the arriving tool's purge beside it, one fine layer on the same top,
                        // reads as the tower going backwards.
                        if (!emission.held_above_level)
                            current_z_by_domain[emission.support_domain] =
                                std::max(current_z_by_domain[emission.support_domain], emission.z);
                        earlier_by_domain[emission.support_domain].push_back(&emission);
                    }
                for (const auto &domain_z : current_z_by_domain)
                    previous_z_by_domain[domain_z.first] = std::max(previous_z_by_domain[domain_z.first], domain_z.second);
            }
            return true;
        };
        if (!auto_pad) {
            wipe_tower.set_used_filament_ids(std::vector<int>(used_filament_ids.begin(), used_filament_ids.end()));
            wipe_tower.set_filament_categories(categories);
            apply_tower_lag(wipe_tower);

            // Generate the wipe tower layers.
            m_wipe_tower_data.tool_changes.reserve(m_wipe_tower_data.tool_ordering.layer_tools().size());
            wipe_tower.generate_new(m_wipe_tower_data.tool_changes);
            if (!audit_structural_emissions(m_wipe_tower_data.tool_changes))
                throw Slic3r::SlicingError(tower_rejection_for_user(structural_rejection));
        } else {
            struct AutoPadCandidate {
                std::unique_ptr<WipeTower> tower;
                std::vector<std::vector<WipeTower::ToolChangeResult>> tool_changes;
                std::vector<float> automatic_ram_lengths;
                std::vector<float> automatic_prime_volumes;
                float requested_width { 0.f };
                size_t expected_switches { 0 };
                bool replay_ok { false };
                double footprint { std::numeric_limits<double>::infinity() };
                // Candidate-owned tower timing is derived only after the native event replay has
                // generated this candidate's actual rows.  It is retained as evidence only; the
                // uncertainty is carried with the candidate and is never converted into a
                // per-switch selector score.
                WipeTowerReplayTiming tower_timing;
                // Residuals are measured from the generated outer-wall bounds after all Type1
                // replanning.  A positive value means an emitted/nominal span escaped that
                // authoritative geometry; ribs and brim are allowed to make the actual bounds
                // larger than the nominal rectangle.
                double depth_residual { std::numeric_limits<double>::infinity() };
                double bbox_residual { std::numeric_limits<double>::infinity() };
            };

            // The pad decision has to be a pure function of the saved project. The height it asks about is
            // the last level the tower plan reaches, the number WipeTower::generate_new() adopts before it
            // plans; the model top differs whenever the print finishes on one tool. The tool ordering is the
            // fallback for a plan that carried no finite level.
            float plan_tower_height = 0.f;
            for (const WipeTowerPlanEvent &event : plan_events)
                if (std::isfinite(event.z))
                    plan_tower_height = std::max(plan_tower_height, event.z);
            const float tower_height = plan_tower_height > 0.f
                ? plan_tower_height
                : (m_wipe_tower_data.tool_ordering.empty()
                       ? 0.f : float(m_wipe_tower_data.tool_ordering.back().print_z));
            // The smallest base this tower may stand on. It is a safety minimum, so a floor above the
            // configured width lifts the search's cap to the floor and no further. Same answer as
            // WipeTower::tower_footprint_floor(): the larger of the stock height/depth rule and the height
            // tiers. The tier is a floor on the printed pad, and the planner floors the rectangle to the
            // smaller of the two, so the tier is the safe cap: a cap has to sit at or above what the planner
            // hands back, and the planner aligns its floor up to the extrusion grid.
            //
            // The two floors are kept apart below: the stock rule is about the nominal block's depth, the
            // tier about the printed pad, rib wall included.
            const float stock_depth_floor = tower_height > 0.f
                ? WipeTower::get_limit_depth_by_height(tower_height) : 0.f;
            const float pad_extent_floor = tower_height > 0.f
                ? mixed_nozzle_tower_footprint_floor(tower_height) : 0.f;
            const float footprint_floor = std::max(stock_depth_floor, pad_extent_floor);
            const float width_cap = std::max(float(m_config.prime_tower_width.value), footprint_floor);
            const float minimum_nozzle_width = m_config.nozzle_diameter.values.empty() ? 0.f :
                float(1.25 * *std::min_element(m_config.nozzle_diameter.values.begin(),
                                               m_config.nozzle_diameter.values.end()));
            // Leave a positive cleaning box and respect the existing option's legal range. The stability
            // floor is carried by the nominal rectangle (WipeTower::tower_footprint_floor), and the ribs
            // still extend beyond it.
            const float line_clearance_floor = 2.f * minimum_nozzle_width + 2.f * float(EPSILON);
            const float minimum_width = std::max(float(print_config_def.get("prime_tower_width")->min),
                                                 line_clearance_floor);
            if (!std::isfinite(width_cap) || width_cap + EPSILON < minimum_width)
                throw Slic3r::SlicingError(Slic3r::format(
                    "Automatic mixed-nozzle prime-tower padding cannot fit within the prime_tower_width cap of %1% mm; "
                    "the configured width range and line clearance require at least %2% mm. "
                    "Raise Prime tower > Width in the process settings, or choose a different cadence in Mixed-Nozzle setup.",
                    width_cap, minimum_width));

            // The share of the idle re-prime the search is trying to fit. It starts at the full re-prime and
            // steps down only when no tower inside the cap carries it.
            float reprime_share = 1.f;
            auto build_auto_pad_candidate = [&](float requested_width, bool preserve_width = false) {
                AutoPadCandidate candidate;
                candidate.requested_width = requested_width;
                candidate.tower = Slic3r::make_unique<WipeTower>(
                    m_config, m_plate_index, m_origin,
                    m_wipe_tower_data.tool_ordering.first_extruder(), tower_height,
                    m_wipe_tower_data.tool_ordering.all_extruders(), requested_width);
                candidate.tower->set_preserve_planned_width(preserve_width);
                configure_wipe_tower(*candidate.tower);
                apply_tower_lag(*candidate.tower);
                WipeTowerReplayContext replay_context{
                    *candidate.tower, m_config, categories, used_filament_ids, requested_width,
                    print_group_result,
                    [&](const std::vector<std::vector<WipeTower::ToolChangeResult>> &changes) {
                        const bool accepted = audit_structural_emissions(changes);
                        if (!accepted)
                            structural_gap_rejected = true;
                        return accepted;
                    },
                    width_cap,
                    {},
                    false,
                    false};
                replay_context.reprime_share = reprime_share;
                WipeTowerReplayResult replay = replay_wipe_tower_plan(plan_events, replay_context);
                candidate.tool_changes = std::move(replay.tool_changes);
                candidate.automatic_ram_lengths = std::move(replay.automatic_ram_lengths);
                candidate.automatic_prime_volumes = std::move(replay.automatic_prime_volumes);
                candidate.expected_switches = replay.expected_switches;
                candidate.replay_ok = replay.replay_ok;
                candidate.tower_timing = replay.timing;
                candidate.footprint = replay.footprint;
                candidate.depth_residual = replay.depth_residual;
                candidate.bbox_residual = replay.bbox_residual;

                return candidate;
            };

            auto automatic_residuals_converged = [&](const AutoPadCandidate &candidate) {
                if (!candidate.replay_ok || !candidate.tower)
                    return false;
                const double final_width = candidate.tower->width();
                if (std::abs(final_width - candidate.requested_width) > 0.001)
                    return false;
                if (!std::isfinite(candidate.depth_residual) || candidate.depth_residual > 0.5 ||
                    !std::isfinite(candidate.bbox_residual) || candidate.bbox_residual > 0.5)
                    return false;
                size_t prime_index = 0;
                size_t ram_index = 0;
                for (const WipeTowerPlanEvent &event : plan_events) {
                    if (event.automatic_prime) {
                        if (prime_index >= candidate.automatic_prime_volumes.size() || event.incoming_extruder < 0)
                            return false;
                        const auto handoff = mixed_nozzle_handoff_deposits(
                            m_config, event.incoming_filament, size_t(event.incoming_extruder), final_width,
                            event.layer_height);
                        if (!handoff || std::abs(double(candidate.automatic_prime_volumes[prime_index++]) -
                                                  double(mixed_nozzle_shared_reprime_volume(
                                                      m_config, event.incoming_filament,
                                                      handoff->prime_volume_mm3, event.idle_seconds,
                                                      reprime_share))) > 0.02)
                            return false;
                    }
                    if (event.automatic_ram) {
                        if (ram_index >= candidate.automatic_ram_lengths.size() || event.outgoing_extruder < 0)
                            return false;
                        const auto handoff = mixed_nozzle_handoff_deposits(
                            m_config, event.outgoing_filament, size_t(event.outgoing_extruder), final_width,
                            event.layer_height);
                        if (!handoff || std::abs(double(candidate.automatic_ram_lengths[ram_index++]) -
                                                  double(handoff->ram_length_mm)) > 0.02)
                            return false;
                    }
                }
                return prime_index == candidate.automatic_prime_volumes.size() &&
                       ram_index == candidate.automatic_ram_lengths.size();
            };

            std::vector<float> seeds;
            // With no tower height in hand the pad has nothing to judge stability against, so it does not
            // shrink. A state not filled in yet must never buy a smaller footprint than the project asked
            // for.
            if (tower_height > 0.f) {
                seeds.emplace_back(minimum_width);
                if (width_cap > minimum_width + 0.01f)
                    seeds.emplace_back(0.5f * (minimum_width + width_cap));
            }
            if (seeds.empty() || width_cap > minimum_width + 0.01f)
                seeds.emplace_back(width_cap);

            // The sliced tower's own stability check: read the geometry that came out and refuse a candidate
            // under the floor, whatever route got it there. The stock rule is read on the nominal block and
            // the tier on the pad the candidate would print, measured off the same rib section the preview
            // and the wall use. The half millimetre matches the replay's residual tolerance for this
            // geometry.
            const bool rib_wall_pad = m_config.wipe_tower_wall_type.value == WipeTowerWallType::wtwRib;
            auto candidate_stands = [&](const AutoPadCandidate &candidate) {
                if (footprint_floor <= 0.f)
                    return true;
                if (!candidate.tower)
                    return false;
                const double nominal_width = double(candidate.tower->width());
                const double nominal_depth = double(candidate.tower->get_depth());
                if (!std::isfinite(nominal_width) || !std::isfinite(nominal_depth))
                    return false;
                if (std::min(nominal_width, nominal_depth) + 0.5 < double(stock_depth_floor))
                    return false;
                if (pad_extent_floor <= 0.f)
                    return true;
                double pad_width = nominal_width;
                double pad_depth = nominal_depth;
                if (rib_wall_pad) {
                    const Polygon section = WipeTower::rib_section(
                        float(nominal_width), float(nominal_depth),
                        candidate.tower->get_rib_length(), candidate.tower->get_rib_width(),
                        m_config.wipe_tower_fillet_wall.value);
                    if (section.points.size() < 3)
                        return false;
                    const BoundingBox pad = section.bounding_box();
                    pad_width = unscale<double>(pad.max.x() - pad.min.x());
                    pad_depth = unscale<double>(pad.max.y() - pad.min.y());
                }
                return std::min(pad_width, pad_depth) + 0.5 >= double(pad_extent_floor);
            };

            std::optional<AutoPadCandidate> best_candidate;
            constexpr size_t max_width_iterations = 8;
            // The seeds walk toward the planner's own fixed point and usually meet on the way. A width this
            // search already replayed leads to a candidate already compared, so it is skipped instead of
            // generating a full Type1 tower again. The walk's rules live in walk_auto_pad_width(); this keeps
            // the candidate each build made, so a walk that settles hands over the tower it settled on.
            std::set<long> replayed_widths;
            // Every walk of every re-prime share, for the refusal below.
            std::vector<AutoPadWidthWalk> walks;
            // A tower that carries the full idle re-prime can be wider than the cap, since a rib tower
            // squares its largest layer. The cap and every structural check stay; the re-prime above the
            // floor steps down until a tower fits, and at share 0 this is the plain search. A plan with no
            // re-prime to give up searches once. Each share runs walk_auto_pad_width from a clean slate.
            bool reprime_acts = false;
            for (const WipeTowerPlanEvent &event : plan_events)
                if (event.automatic_prime &&
                    mixed_nozzle_idle_reprime_volume(m_config, event.incoming_filament, 0.f, event.idle_seconds) > 0.f)
                    reprime_acts = true;
            const std::vector<float> reprime_shares = reprime_acts
                ? std::vector<float>{1.f, .5f, .25f, 0.f} : std::vector<float>{1.f};
            for (const float share : reprime_shares) {
                reprime_share = share;
                replayed_widths.clear();
                std::vector<AutoPadWidthWalk> share_walks;
                for (float seed : seeds) {
                    AutoPadCandidate candidate;
                    AutoPadWidthWalk walk = walk_auto_pad_width(
                        seed, minimum_width, width_cap, max_width_iterations, replayed_widths,
                        [&](float requested_width) {
                            candidate = build_auto_pad_candidate(requested_width);
                            return AutoPadWidthBuild{candidate.replay_ok,
                                                     candidate.tower ? candidate.tower->width() : 0.f};
                        });
                    const bool settled = walk.end == AutoPadWalkEnd::Settled;
                    if (settled)
                        candidate.requested_width = walk.width;
                    walks.push_back(walk);
                    share_walks.push_back(std::move(walk));
                    if (!settled || !automatic_residuals_converged(candidate) || !candidate_stands(candidate))
                        continue;
                    if (!best_candidate || candidate.footprint < best_candidate->footprint)
                        best_candidate = std::move(candidate);
                }

                // Row rounding can make the rib planner alternate between two widths. Rebuild
                // at a visited width without squaring it again, so automatic prime is resolved
                // against the exact width that prints. Keep all geometry and stability checks.
                if (!best_candidate) {
                    for (float width : auto_pad_fallback_widths(share_walks)) {
                        if (width < minimum_width || width > width_cap)
                            continue;
                        AutoPadCandidate candidate = build_auto_pad_candidate(width, true);
                        if (!automatic_residuals_converged(candidate) || !candidate_stands(candidate))
                            continue;
                        if (!best_candidate || candidate.footprint < best_candidate->footprint)
                            best_candidate = std::move(candidate);
                    }
                }
                if (best_candidate)
                    break;
            }

            // More tools purge more per layer, and a square rib tower that holds it can be wider than the
            // cap. Keep the width at the cap and let the tower grow deeper instead, the way a stock tower
            // grows. Every residual and stability check still applies.
            if (!best_candidate &&
                std::any_of(walks.begin(), walks.end(), [](const AutoPadWidthWalk &walk) { return walk.above_cap_width > 0.f; })) {
                for (const float share : reprime_shares) {
                    reprime_share = share;
                    AutoPadCandidate candidate = build_auto_pad_candidate(width_cap, true);
                    if (automatic_residuals_converged(candidate) && candidate_stands(candidate)) {
                        best_candidate = std::move(candidate);
                        break;
                    }
                }
            }
            if (!best_candidate) {
                if (structural_gap_rejected)
                    throw Slic3r::SlicingError(tower_rejection_for_user(structural_rejection));
            }
            if (!best_candidate) {
                // The refusal has to be true about this plate. Raising the cap helps only when the planner
                // asked for a tower wider than the cap; otherwise the refusal says the cap was not the limit.
                // Neither suggests turning Automatic padding off, since the GUI locks it while the automatic
                // tower setup manages it.
                float above_cap_width = 0.f;
                for (const AutoPadWidthWalk &walk : walks)
                    if (walk.above_cap_width > 0.f &&
                        (above_cap_width <= 0.f || walk.above_cap_width < above_cap_width))
                        above_cap_width = walk.above_cap_width;
                const size_t tool_count = used_filament_ids.size();
                if (above_cap_width > 0.f)
                    throw Slic3r::SlicingError(Slic3r::format(
                        "Automatic mixed-nozzle prime-tower padding did not converge within the prime_tower_width cap of %1% mm "
                        "with %2% tools. The planner asked for a tower of %3% mm, wider than the cap. "
                        "Raise Prime tower > Width in the process settings (with Auto-size mixed-nozzle pad on, it is the "
                        "cap and stays editable for rib walls too), or choose a different cadence in Mixed-Nozzle setup.",
                        width_cap, tool_count, above_cap_width));
                throw Slic3r::SlicingError(Slic3r::format(
                    "Automatic mixed-nozzle prime-tower padding did not converge within the prime_tower_width cap of %1% mm "
                    "with %2% tools. Every tower the planner drew for this plate fit inside the cap, so the cap is not what "
                    "stopped it. A different cadence in Mixed-Nozzle setup changes the schedule the tower is sized for.",
                    width_cap, tool_count));
            }

            auto_wipe_tower = std::move(best_candidate->tower);
            final_wipe_tower = auto_wipe_tower.get();
            m_wipe_tower_data.tool_changes = std::move(best_candidate->tool_changes);
        }

        m_wipe_tower_data.height    = final_wipe_tower->get_height();
        m_wipe_tower_data.width     = final_wipe_tower->width();
        m_wipe_tower_data.depth     = final_wipe_tower->get_depth();
        m_wipe_tower_data.brim_width = final_wipe_tower->get_brim_width();
        m_wipe_tower_data.bbx       = final_wipe_tower->get_bbx();
        m_wipe_tower_data.rib_offset = final_wipe_tower->get_rib_offset();
        // Taken from whichever tower was committed (the plain one or the pad search's winner) and kept
        // with the tower data, so it lives as long as the rows it describes. The pricing pass copies it
        // out in index_committed_tower(); a later rebuild clears it with the rest of the tower data.
        m_wipe_tower_data.plan_digest = final_wipe_tower->plan_digest();

        // Unload the current filament over the purge tower.
        coordf_t layer_height = m_objects.front()->config().layer_height.value;
        bool generate_final_purge = true;
        if (m_wipe_tower_data.tool_ordering.back().wipe_tower_partitions > 0) {
            // The wipe tower goes up to the last layer of the print.
            if (final_wipe_tower->layer_finished()) {
                // The wipe tower is printed to the top of the print and it has no space left for the final extruder purge.
                // Lift Z to the next layer.
                final_wipe_tower->set_layer(float(m_wipe_tower_data.tool_ordering.back().print_z + layer_height), float(layer_height), 0, false,
                                            true);
            } else {
                // There is yet enough space at this layer of the wipe tower for the final purge.
            }
        } else {
            // The wipe tower does not reach the last print layer.
            // Skip final purge to avoid generating purge lines in mid-air.
            assert(m_wipe_tower_data.tool_ordering.back().wipe_tower_partitions == 0);
            generate_final_purge = false;
        }
        m_wipe_tower_data.final_purge = generate_final_purge
            ? Slic3r::make_unique<WipeTower::ToolChangeResult>(final_wipe_tower->tool_change((unsigned int)(-1)))
            : Slic3r::make_unique<WipeTower::ToolChangeResult>();

        m_wipe_tower_data.used_filament         = final_wipe_tower->get_used_filament();
        m_wipe_tower_data.number_of_toolchanges = final_wipe_tower->get_number_of_toolchanges();
        m_wipe_tower_data.construct_mesh(final_wipe_tower->width(), final_wipe_tower->get_depth(), final_wipe_tower->get_height(), final_wipe_tower->get_brim_width(), config().wipe_tower_wall_type.value == WipeTowerWallType::wtwRib,
                                          final_wipe_tower->get_rib_width(), final_wipe_tower->get_rib_length(), config().wipe_tower_fillet_wall.value);
        const Vec3d origin                      = this->get_plate_origin();
        m_fake_wipe_tower.rib_offset = final_wipe_tower->get_rib_offset();
        m_fake_wipe_tower.set_fake_extrusion_data(final_wipe_tower->position() + m_fake_wipe_tower.rib_offset, final_wipe_tower->width(), final_wipe_tower->get_height(), final_wipe_tower->get_layer_height(),
                                                  m_wipe_tower_data.depth,
                                                  m_wipe_tower_data.brim_width, {scale_(origin.x()), scale_(origin.y())});
        m_fake_wipe_tower.outer_wall = final_wipe_tower->get_outer_wall();
    } else {
        // Get wiping matrix to get number of extruders and convert vector<double> to vector<float>:
        std::vector<float> flush_matrix(cast<float>(m_config.flush_volumes_matrix.values));
        require_flush_block(flush_matrix.size(), 0);
        // Extract purging volumes for each extruder pair:
        std::vector<std::vector<float>> wipe_volumes;
        for (unsigned int i = 0; i<number_of_extruders; ++i)
            wipe_volumes.push_back(std::vector<float>(flush_matrix.begin()+i*number_of_extruders, flush_matrix.begin()+(i+1)*number_of_extruders));

        // Orca: itertate over wipe_volumes and change the non-zero values to the prime_volume
        if ((!m_config.purge_in_prime_tower || !m_config.single_extruder_multi_material) && is_wipe_tower_type2) {
            for (unsigned int i = 0; i < number_of_extruders; ++i) {
                for (unsigned int j = 0; j < number_of_extruders; ++j) {
                    if (wipe_volumes[i][j] > 0) {
                        wipe_volumes[i][j] = m_config.prime_volume;
                    }
                }
            }
        }
        // Initialize the wipe tower.
        WipeTower2 wipe_tower(m_config, m_default_region_config, m_plate_index, m_origin, wipe_volumes,
                              m_wipe_tower_data.tool_ordering.first_extruder());

        // wipe_tower.set_retract();
        // wipe_tower.set_zhop();

        // Set the extruder & material properties at the wipe tower object.
        for (size_t i = 0; i < number_of_extruders; ++i)
            wipe_tower.set_extruder(i, m_config);

        m_wipe_tower_data.priming = Slic3r::make_unique<std::vector<WipeTower::ToolChangeResult>>(
            wipe_tower.prime((float)this->skirt_first_layer_height(), m_wipe_tower_data.tool_ordering.all_extruders(), false));

        // Lets go through the wipe tower layers and determine pairs of extruder changes for each
        // to pass to wipe_tower (so that it can use it for planning the layout of the tower)
        {
            unsigned int current_extruder_id = m_wipe_tower_data.tool_ordering.all_extruders().back();
            for (auto &layer_tools : m_wipe_tower_data.tool_ordering.layer_tools()) { // for all layers
                if (!layer_tools.has_wipe_tower)
                    continue;
                bool first_layer = &layer_tools == &m_wipe_tower_data.tool_ordering.front();
                wipe_tower.plan_toolchange((float) layer_tools.print_z, (float) layer_tools.wipe_tower_layer_height, current_extruder_id,
                                           current_extruder_id, false);
                for (const auto extruder_id : layer_tools.extruders) {
                    if ((first_layer && extruder_id == m_wipe_tower_data.tool_ordering.all_extruders().back()) || extruder_id !=
                        current_extruder_id) {
                        float volume_to_wipe = m_config.prime_volume;
                        if (m_config.purge_in_prime_tower && m_config.single_extruder_multi_material) {
                            volume_to_wipe = wipe_volumes[current_extruder_id][extruder_id]; // total volume to wipe after this toolchange
                            volume_to_wipe *= m_config.flush_multiplier.get_at(0);
                            // Not all of that can be used for infill purging:
                            volume_to_wipe -= (float) m_config.filament_minimal_purge_on_wipe_tower.get_at(extruder_id);

                            // try to assign some infills/objects for the wiping:
                            volume_to_wipe = layer_tools.wiping_extrusions().mark_wiping_extrusions(*this, current_extruder_id, extruder_id,
                                                                                                    volume_to_wipe);

                            // add back the minimal amount toforce on the wipe tower:
                            volume_to_wipe += (float) m_config.filament_minimal_purge_on_wipe_tower.get_at(extruder_id);
                        }

                        // request a toolchange at the wipe tower with at least volume_to_wipe purging amount
                        wipe_tower.plan_toolchange((float) layer_tools.print_z, (float) layer_tools.wipe_tower_layer_height,
                                                   current_extruder_id, extruder_id, volume_to_wipe);
                        current_extruder_id = extruder_id;
                    }
                }
                layer_tools.wiping_extrusions().ensure_perimeters_infills_order(*this);
                if (&layer_tools == &m_wipe_tower_data.tool_ordering.back() || (&layer_tools + 1)->wipe_tower_partitions == 0)
                    break;
            }
        }

        // Generate the wipe tower layers.
        m_wipe_tower_data.tool_changes.reserve(m_wipe_tower_data.tool_ordering.layer_tools().size());
        wipe_tower.generate(m_wipe_tower_data.tool_changes);
        m_wipe_tower_data.depth             = wipe_tower.get_depth();
        m_wipe_tower_data.z_and_depth_pairs = wipe_tower.get_z_and_depth_pairs();
        m_wipe_tower_data.brim_width        = wipe_tower.get_brim_width();
        m_wipe_tower_data.height            = wipe_tower.get_wipe_tower_height();
        m_wipe_tower_data.bbx               = wipe_tower.get_bbx();
        m_wipe_tower_data.rib_offset        = wipe_tower.get_rib_offset();

        // Unload the current filament over the purge tower.
        coordf_t layer_height = m_objects.front()->config().layer_height.value;
        bool generate_final_purge = true;
        if (m_wipe_tower_data.tool_ordering.back().wipe_tower_partitions > 0) {
            // The wipe tower goes up to the last layer of the print.
            if (wipe_tower.layer_finished()) {
                // The wipe tower is printed to the top of the print and it has no space left for the final extruder purge.
                // Lift Z to the next layer.
                wipe_tower.set_layer(float(m_wipe_tower_data.tool_ordering.back().print_z + layer_height), float(layer_height), 0, false,
                                     true);
            } else {
                // There is yet enough space at this layer of the wipe tower for the final purge.
            }
        } else {
            // The wipe tower does not reach the last print layer.
            // Skip final purge to avoid generating purge lines in mid-air.
            assert(m_wipe_tower_data.tool_ordering.back().wipe_tower_partitions == 0);
            generate_final_purge = false;
        }
        m_wipe_tower_data.final_purge = generate_final_purge
            ? Slic3r::make_unique<WipeTower::ToolChangeResult>(wipe_tower.tool_change((unsigned int)(-1)))
            : Slic3r::make_unique<WipeTower::ToolChangeResult>();

        m_wipe_tower_data.used_filament         = wipe_tower.get_used_filament();
        m_wipe_tower_data.number_of_toolchanges = wipe_tower.get_number_of_toolchanges();
        m_wipe_tower_data.width                  = wipe_tower.width();
        m_wipe_tower_data.construct_mesh(wipe_tower.width(), wipe_tower.get_depth(),
                                         wipe_tower.get_wipe_tower_height(), wipe_tower.get_brim_width(),
                                         config().wipe_tower_wall_type.value == WipeTowerWallType::wtwRib,
                                         wipe_tower.get_rib_width(), wipe_tower.get_rib_length(),
                                         config().wipe_tower_fillet_wall.value);
        const Vec3d origin                      = Vec3d::Zero();
        // FakeWipeTower::pos is a bed-frame translation applied after rotation
        // (getFakeExtrusionPathsFromWipeTower2 rotates about the local origin), so the
        // tower-local rib offset must be rotated into the bed frame first.
        m_fake_wipe_tower.rib_offset = Eigen::Rotation2Df(Geometry::deg2rad((float)config().wipe_tower_rotation_angle.value)) *
                                       wipe_tower.get_rib_offset();
        m_fake_wipe_tower.set_fake_extrusion_data(wipe_tower.position() + m_fake_wipe_tower.rib_offset, wipe_tower.width(), wipe_tower.get_wipe_tower_height(),
                                                  config().initial_layer_print_height, m_wipe_tower_data.depth,
                                                  m_wipe_tower_data.z_and_depth_pairs, m_wipe_tower_data.brim_width,
                                                  config().wipe_tower_rotation_angle, config().wipe_tower_cone_angle,
                                                  {scale_(origin.x()), scale_(origin.y())});
    }
    // The generator stops at the last required tower partition. ToolOrdering can still have
    // trailing native levels marked has_wipe_tower; those are not exported tower events.
    // Bind identities to the actual generated level stream and reject any interior mismatch.
    size_t exported_event = 0;
    for (LayerTools& level : m_wipe_tower_data.tool_ordering.layer_tools()) {
        level.wipe_tower_event_index = size_t(-1);
        if (!level.has_wipe_tower) continue;
        if (exported_event == m_wipe_tower_data.tool_changes.size()) {
            level.has_wipe_tower = false;
            level.wipe_tower_emit = false;
            level.tower_support_event = false;
            continue;
        }
        const auto& changes = m_wipe_tower_data.tool_changes[exported_event];
        if (changes.empty() || std::abs(double(changes.front().print_z) - level.print_z) > EPSILON)
            throw Slic3r::SlicingError("[SRL-TOWER-STRUCTURE] Generated tower level does not match the scheduled support stream.");
        level.wipe_tower_event_index = exported_event++;
    }
    if (exported_event != m_wipe_tower_data.tool_changes.size())
        throw Slic3r::SlicingError("[SRL-TOWER-STRUCTURE] Generated tower levels remain outside the scheduled support stream.");

}

// Generate a recommended G-code output file name based on the format template, default extension, and template parameters
// (timestamps, object placeholders derived from the model, current placeholder prameters and print statistics.
// Use the final print statistics if available, or just keep the print statistics placeholders if not available yet (before G-code is finalized).
std::string Print::output_filename(const std::string &filename_base) const
{
    // Set the placeholders for the data know first after the G-code export is finished.
    // These values will be just propagated into the output file name.
    DynamicConfig config = this->finished() ? this->print_statistics().config() : this->print_statistics().placeholders();
    config.set_key_value("num_filaments", new ConfigOptionInt((int)m_config.nozzle_diameter.size()));
    config.set_key_value("num_extruders", new ConfigOptionInt((int) m_config.nozzle_diameter.size()));
    config.set_key_value("plate_name", new ConfigOptionString(get_plate_name()));
    config.set_key_value("plate_number", new ConfigOptionString(get_plate_number_formatted()));
    config.set_key_value("model_name", new ConfigOptionString(get_model_name()));

    // the same type of filament contains multiple names, support exporting according to the filament name
    auto full_print_config = this->full_print_config();
    const ConfigOptionStrings* filament_settings_id = full_print_config.option<ConfigOptionStrings>("filament_settings_id");
    std::string filament_name = "";
    auto extruders = this->extruders(true);
    if(!extruders.empty()) {
        // first extruder is the default extruder
        int extruder_id = extruders.front();
        if(filament_settings_id->values.size() > extruder_id) {
            filament_name = filament_settings_id->values[extruder_id];
        }
    }
    size_t end_pos = filament_name.find_first_of("@");
    if (end_pos != std::string::npos) {
        filament_name = filament_name.substr(0, end_pos);
    }
    config.set_key_value("filament_name", new ConfigOptionString(filament_name));

    return this->PrintBase::output_filename(m_config.filename_format.value, ".gcode", filename_base, &config);
}

std::string Print::get_model_name() const
{
    if (model().model_info != nullptr)
    {
        return model().model_info->model_name;
    } else {
        return "";
    }
}

std::string Print::get_plate_number_formatted() const
{
    std::string plate_number = std::to_string(get_plate_index() + 1);
    static const size_t n_zero = 2;

    return std::string(n_zero - std::min(n_zero, plate_number.length()), '0') + plate_number;
}

//BBS: add gcode file preload logic
void Print::set_gcode_file_ready()
{
    this->set_started(psGCodeExport);
	this->set_done(psGCodeExport);
    BOOST_LOG_TRIVIAL(info) << __FUNCTION__ <<  boost::format(": done");
}
//BBS: add gcode file preload logic
void Print::set_gcode_file_invalidated()
{
    this->invalidate_step(psGCodeExport);
    BOOST_LOG_TRIVIAL(info) << __FUNCTION__ <<  boost::format(": done");
}

//BBS: add gcode file preload logic
void Print::export_gcode_from_previous_file(const std::string& file, GCodeProcessorResult* result, ThumbnailsGeneratorCallback thumbnail_cb)
{
    try {
        GCodeProcessor processor;
        GCodeProcessor::s_IsBBLPrinter = is_BBL_printer();
        const Vec3d origin = this->get_plate_origin();
        processor.set_xy_offset(origin(0), origin(1));
        // Reloaded sliced projects re-estimate with the same nozzle-grouping slot context as the
        // original export; process_file re-derives the device-side nozzle grouping onto the result
        // (via ensure_nozzle_group_result), so the multi-nozzle send/monitor mapping survives here.
        if (result != nullptr && result->nozzle_group_result)
            processor.initialize_from_context(result->nozzle_group_result);
        //processor.enable_producers(true);
        processor.process_file(file);

        // filament seq is loaded from file, processor result will override the value
        auto filament_seq_loaded = result->filament_change_sequence;
        auto nozzle_seq_loaded   = result->nozzle_change_sequence;
        *result = std::move(processor.extract_result());
        result->filament_change_sequence = filament_seq_loaded;
        result->nozzle_change_sequence   = nozzle_seq_loaded;
    } catch (std::exception & /* ex */) {
        BOOST_LOG_TRIVIAL(error) << __FUNCTION__ <<  boost::format(": found errors when process gcode file %1%") %file.c_str();
        throw Slic3r::RuntimeError(
            std::string("Failed to process the G-code file ") + file + " from previous 3mf\n");
    }

    BOOST_LOG_TRIVIAL(info) << __FUNCTION__ <<  boost::format(":  process the G-code file %1% successfully")%file.c_str();
}

std::tuple<float, float> Print::object_skirt_offset(double margin_height) const
{
    if (config().skirt_loops == 0 || config().skirt_type != stPerObject || m_objects.empty())
        return std::make_tuple(0, 0);
    
    float max_nozzle_diameter = *std::max_element(m_config.nozzle_diameter.values.begin(), m_config.nozzle_diameter.values.end());
    float max_layer_height    = *std::max_element(config().max_layer_height.values.begin(), config().max_layer_height.values.end());
    float line_width = m_config.initial_layer_line_width.get_abs_value(max_nozzle_diameter);
    float object_skirt_witdh  = skirt_flow().width() + (config().skirt_loops - 1) * skirt_flow().spacing();
    float object_skirt_offset = 0;

    if (is_all_objects_are_short())
        object_skirt_offset = config().skirt_distance + object_skirt_witdh;
    else if (config().draft_shield == dsEnabled || config().skirt_height * max_layer_height > config().nozzle_height - margin_height)
        object_skirt_offset = config().skirt_distance + line_width;
    else if (config().skirt_distance + object_skirt_witdh > config().extruder_clearance_radius/2)
        object_skirt_offset = (config().skirt_distance + object_skirt_witdh - config().extruder_clearance_radius/2);
    else
        return std::make_tuple(0, 0);

    return std::make_tuple(object_skirt_offset, object_skirt_witdh);
}

DynamicConfig PrintStatistics::config() const
{
    DynamicConfig config;
    std::string normal_print_time = short_time(this->estimated_normal_print_time);
    std::string silent_print_time = short_time(this->estimated_silent_print_time);
    config.set_key_value("print_time", new ConfigOptionString(normal_print_time));
    config.set_key_value("normal_print_time", new ConfigOptionString(normal_print_time));
    config.set_key_value("silent_print_time", new ConfigOptionString(silent_print_time));
    config.set_key_value("used_filament",             new ConfigOptionFloat(this->total_used_filament / 1000.));
    config.set_key_value("extruded_volume",           new ConfigOptionFloat(this->total_extruded_volume));
    config.set_key_value("total_cost",                new ConfigOptionFloat(this->total_cost));
    config.set_key_value("total_toolchanges",         new ConfigOptionInt(this->total_toolchanges));
    config.set_key_value("total_weight",              new ConfigOptionFloat(this->total_weight));
    config.set_key_value("extruded_weight_total",     new ConfigOptionFloat(this->total_weight));
    config.set_key_value("extruded_volume_total",     new ConfigOptionFloat(this->total_extruded_volume));
    config.set_key_value("total_wipe_tower_cost",     new ConfigOptionFloat(this->total_wipe_tower_cost));
    config.set_key_value("total_wipe_tower_filament", new ConfigOptionFloat(this->total_wipe_tower_filament));
    config.set_key_value("initial_tool",              new ConfigOptionInt(static_cast<int>(this->initial_tool)));
    config.set_key_value("initial_extruder",          new ConfigOptionInt(static_cast<int>(this->initial_tool)));
    return config;
}

DynamicConfig PrintStatistics::placeholders()
{
    DynamicConfig config;
    for (const std::string key : {
        "print_time", "normal_print_time", "silent_print_time",
        "used_filament", "extruded_volume", "extruded_volume_total", "total_cost", "total_weight", "extruded_weight_total",
        "initial_tool", "initial_extruder", "total_toolchanges", "total_wipe_tower_cost", "total_wipe_tower_filament"})
        config.set_key_value(key, new ConfigOptionString(std::string("{") + key + "}"));
    return config;
}

std::string PrintStatistics::finalize_output_path(const std::string &path_in) const
{
    std::string final_path;
    try {
        boost::filesystem::path path(path_in);
        DynamicConfig cfg = this->config();
        PlaceholderParser pp;
        std::string new_stem = pp.process(path.stem().string(), 0, &cfg);
        final_path = (path.parent_path() / (new_stem + path.extension().string())).string();
    } catch (const std::exception &ex) {
        BOOST_LOG_TRIVIAL(error) << "Failed to apply the print statistics to the export file name: " << ex.what();
        final_path = path_in;
    }
    return final_path;
}

// Orca: Implement prusa's filament shrink compensation approach
// Returns if all used filaments have same shrinkage compensations.
 bool Print::has_same_shrinkage_compensations() const {
     const std::vector<unsigned int> extruders = this->extruders();
     if (extruders.empty())
         return false;

     const double filament_shrinkage_compensation_xy = m_config.filament_shrink.get_at(extruders.front());
     const double filament_shrinkage_compensation_z  = m_config.filament_shrinkage_compensation_z.get_at(extruders.front());

     for (unsigned int extruder : extruders) {
         if (filament_shrinkage_compensation_xy != m_config.filament_shrink.get_at(extruder) ||
             filament_shrinkage_compensation_z  != m_config.filament_shrinkage_compensation_z.get_at(extruder)) {
             return false;
         }
     }

     return true;
 }

// Orca: Implement prusa's filament shrink compensation approach, but amended so 100% from the user is the equivalent to 0 in orca.
 // Returns scaling for each axis representing shrinkage compensations in each axis.
Vec3d Print::shrinkage_compensation() const
{
    if (!this->has_same_shrinkage_compensations())
        return Vec3d::Ones();

    const unsigned int first_extruder = this->extruders().front();

    const double xy_shrinkage_percent = m_config.filament_shrink.get_at(first_extruder);
    const double z_shrinkage_percent  = m_config.filament_shrinkage_compensation_z.get_at(first_extruder);

    const double xy_compensation = 100.0 / xy_shrinkage_percent;
    const double z_compensation  = 100.0 / z_shrinkage_percent;

    return { xy_compensation, xy_compensation, z_compensation };
}

const std::string PrintStatistics::FilamentUsedG     = "filament used [g]";
const std::string PrintStatistics::FilamentUsedGMask = "; filament used [g] =";

const std::string PrintStatistics::TotalFilamentUsedG          = "total filament used [g]";
const std::string PrintStatistics::TotalFilamentUsedGMask      = "; total filament used [g] =";
const std::string PrintStatistics::TotalFilamentUsedGValueMask = "; total filament used [g] = %.2lf\n";

const std::string PrintStatistics::FilamentUsedCm3     = "filament used [cm3]";
const std::string PrintStatistics::FilamentUsedCm3Mask = "; filament used [cm3] =";

const std::string PrintStatistics::FilamentUsedMm     = "filament used [mm]";
const std::string PrintStatistics::FilamentUsedMmMask = "; filament used [mm] =";

const std::string PrintStatistics::FilamentCost     = "filament cost";
const std::string PrintStatistics::FilamentCostMask = "; filament cost =";

const std::string PrintStatistics::TotalFilamentCost          = "total filament cost";
const std::string PrintStatistics::TotalFilamentCostMask      = "; total filament cost =";
const std::string PrintStatistics::TotalFilamentCostValueMask = "; total filament cost = %.2lf\n";

const std::string PrintStatistics::TotalFilamentUsedWipeTower     = "total filament used for wipe tower [g]";
const std::string PrintStatistics::TotalFilamentUsedWipeTowerValueMask = "; total filament used for wipe tower [g] = %.2lf\n";


/*add json export/import related functions */
#define JSON_POLYGON_CONTOUR                "contour"
#define JSON_POLYGON_HOLES                  "holes"
#define JSON_POINTS                 "points"
#define JSON_EXPOLYGON              "expolygon"
#define JSON_ARC_FITTING            "arc_fitting"
#define JSON_OBJECT_NAME            "name"
#define JSON_IDENTIFY_ID          "identify_id"


#define JSON_LAYERS                  "layers"
#define JSON_SUPPORT_LAYERS                  "support_layers"
#define JSON_TREE_SUPPORT_LAYERS                  "tree_support_layers"
#define JSON_LAYER_REGIONS                  "layer_regions"
#define JSON_FIRSTLAYER_GROUPS                  "first_layer_groups"

#define JSON_FIRSTLAYER_GROUP_ID                  "group_id"
#define JSON_FIRSTLAYER_GROUP_VOLUME_IDS          "volume_ids"
#define JSON_FIRSTLAYER_GROUP_SLICES               "slices"

#define JSON_LAYER_PRINT_Z            "print_z"
#define JSON_LAYER_SLICE_Z            "slice_z"
#define JSON_LAYER_HEIGHT             "height"
#define JSON_LAYER_ID                  "layer_id"
#define JSON_LAYER_SLICED_POLYGONS    "sliced_polygons"
#define JSON_LAYER_SLLICED_BBOXES      "sliced_bboxes"
#define JSON_LAYER_OVERHANG_POLYGONS    "overhang_polygons"
#define JSON_LAYER_OVERHANG_BBOX       "overhang_bbox"

#define JSON_SUPPORT_LAYER_ISLANDS                  "support_islands"
#define JSON_SUPPORT_LAYER_FILLS                    "support_fills"
#define JSON_SUPPORT_LAYER_INTERFACE_ID             "interface_id"
#define JSON_SUPPORT_LAYER_TYPE                     "support_type"
#define JSON_SUPPORT_LAYER_FINE_BODY_FILLS          "fine_body_fills"
#define JSON_SUPPORT_LAYER_BASE_ON_INTERFACE_NOZZLE "base_on_interface_nozzle"
#define JSON_SUPPORT_LAYER_FINE_BODY_FILAMENT       "interface_nozzle_body_filament"

#define JSON_LAYER_REGION_CONFIG_HASH             "config_hash"
#define JSON_LAYER_REGION_SLICES                  "slices"
#define JSON_LAYER_REGION_RAW_SLICES              "raw_slices"
//#define JSON_LAYER_REGION_ENTITIES                "entities"
#define JSON_LAYER_REGION_THIN_FILLS                  "thin_fills"
#define JSON_LAYER_REGION_FILL_EXPOLYGONS             "fill_expolygons"
#define JSON_LAYER_REGION_FILL_SURFACES               "fill_surfaces"
#define JSON_LAYER_REGION_FILL_NO_OVERLAP             "fill_no_overlap_expolygons"
#define JSON_LAYER_REGION_UNSUPPORTED_BRIDGE_EDGES    "unsupported_bridge_edges"
#define JSON_LAYER_REGION_PERIMETERS                  "perimeters"
#define JSON_LAYER_REGION_FILLS                  "fills"



#define JSON_SURF_TYPE              "surface_type"
#define JSON_SURF_THICKNESS         "thickness"
#define JSON_SURF_THICKNESS_LAYER   "thickness_layers"
#define JSON_SURF_BRIDGE_ANGLE       "bridge_angle"
#define JSON_SURF_EXTRA_PERIMETERS   "extra_perimeters"

#define JSON_ARC_DATA                "arc_data"
#define JSON_ARC_START_INDEX         "start_index"
#define JSON_ARC_END_INDEX           "end_index"
#define JSON_ARC_PATH_TYPE           "path_type"

#define JSON_IS_ARC                  "is_arc"
#define JSON_ARC_LENGTH              "length"
#define JSON_ARC_ANGLE_RADIUS        "angle_radians"
#define JSON_ARC_POLAY_START_THETA   "polar_start_theta"
#define JSON_ARC_POLAY_END_THETA     "polar_end_theta"
#define JSON_ARC_START_POINT          "start_point"
#define JSON_ARC_END_POINT            "end_point"
#define JSON_ARC_DIRECTION            "direction"
#define JSON_ARC_RADIUS               "radius"
#define JSON_ARC_CENTER               "center"

//extrusions
#define JSON_EXTRUSION_ENTITY_TYPE             "entity_type"
#define JSON_EXTRUSION_NO_SORT                 "no_sort"
#define JSON_EXTRUSION_PATHS                   "paths"
#define JSON_EXTRUSION_ENTITIES                "entities"
#define JSON_EXTRUSION_TYPE_PATH               "path"
#define JSON_EXTRUSION_TYPE_MULTIPATH          "multipath"
#define JSON_EXTRUSION_TYPE_LOOP               "loop"
#define JSON_EXTRUSION_TYPE_COLLECTION         "collection"
#define JSON_EXTRUSION_POLYLINE                "polyline"
#define JSON_EXTRUSION_MM3_PER_MM              "mm3_per_mm"
#define JSON_EXTRUSION_WIDTH                   "width"
#define JSON_EXTRUSION_HEIGHT                  "height"
#define JSON_EXTRUSION_ROLE                    "role"
#define JSON_EXTRUSION_NO_EXTRUSION            "no_extrusion"
#define JSON_EXTRUSION_LOOP_ROLE               "loop_role"


static void to_json(json& j, const Points& p_s) {
    for (const Point& p : p_s)
    {
        j.push_back(p.x());
        j.push_back(p.y());
    }
}

static void to_json(json& j, const BoundingBox& bb) {
    j.push_back(bb.min.x());
    j.push_back(bb.min.y());
    j.push_back(bb.max.x());
    j.push_back(bb.max.y());
}

static void to_json(json& j, const ExPolygon& polygon) {
    json contour_json = json::array(), holes_json = json::array();

    //contour
    const Polygon& slice_contour =   polygon.contour;
    contour_json = slice_contour.points;
    j[JSON_POLYGON_CONTOUR] = std::move(contour_json);

    //holes
    const Polygons& slice_holes =   polygon.holes;
    for (const Polygon& hole_polyon : slice_holes)
    {
        json hole_json = json::array();
        hole_json =  hole_polyon.points;
        holes_json.push_back(std::move(hole_json));
    }
    j[JSON_POLYGON_HOLES] = std::move(holes_json);
}

static void to_json(json& j, const Surface& surf) {
    j[JSON_EXPOLYGON] = surf.expolygon;
    j[JSON_SURF_TYPE] = surf.surface_type;
    j[JSON_SURF_THICKNESS] = surf.thickness;
    j[JSON_SURF_THICKNESS_LAYER] = surf.thickness_layers;
    j[JSON_SURF_BRIDGE_ANGLE] = surf.bridge_angle;
    j[JSON_SURF_EXTRA_PERIMETERS] = surf.extra_perimeters;
}

static void to_json(json& j, const ArcSegment& arc_seg) {
    json start_point_json = json::array(), end_point_json = json::array(), center_point_json = json::array();
    j[JSON_IS_ARC] = arc_seg.is_arc;
    j[JSON_ARC_LENGTH] = arc_seg.length;
    j[JSON_ARC_ANGLE_RADIUS] = arc_seg.angle_radians;
    j[JSON_ARC_POLAY_START_THETA] = arc_seg.polar_start_theta;
    j[JSON_ARC_POLAY_END_THETA] = arc_seg.polar_end_theta;
    start_point_json.push_back(arc_seg.start_point.x());
    start_point_json.push_back(arc_seg.start_point.y());
    j[JSON_ARC_START_POINT] = std::move(start_point_json);
    end_point_json.push_back(arc_seg.end_point.x());
    end_point_json.push_back(arc_seg.end_point.y());
    j[JSON_ARC_END_POINT] = std::move(end_point_json);
    j[JSON_ARC_DIRECTION] = arc_seg.direction;
    j[JSON_ARC_RADIUS] = arc_seg.radius;
    center_point_json.push_back(arc_seg.center.x());
    center_point_json.push_back(arc_seg.center.y());
    j[JSON_ARC_CENTER] = std::move(center_point_json);
}


static void to_json(json& j, const Polyline& poly_line) {
    json points_json = json::array(), fittings_json = json::array();
    points_json = poly_line.points;

    j[JSON_POINTS] = std::move(points_json);
    for (const PathFittingData& path_fitting : poly_line.fitting_result)
    {
        json fitting_json;
        fitting_json[JSON_ARC_START_INDEX] = path_fitting.start_point_index;
        fitting_json[JSON_ARC_END_INDEX] = path_fitting.end_point_index;
        fitting_json[JSON_ARC_PATH_TYPE] = path_fitting.path_type;
        if (path_fitting.arc_data.is_arc)
            fitting_json[JSON_ARC_DATA] = path_fitting.arc_data;

        fittings_json.push_back(std::move(fitting_json));
    }
    j[JSON_ARC_FITTING] = fittings_json;
}

static void to_json(json& j, const ExtrusionPath& extrusion_path) {
    j[JSON_EXTRUSION_POLYLINE] = extrusion_path.polyline;
    j[JSON_EXTRUSION_MM3_PER_MM] = extrusion_path.mm3_per_mm;
    j[JSON_EXTRUSION_WIDTH] = extrusion_path.width;
    j[JSON_EXTRUSION_HEIGHT] = extrusion_path.height;
    j[JSON_EXTRUSION_ROLE] = extrusion_path.role();
    j[JSON_EXTRUSION_NO_EXTRUSION] = extrusion_path.is_force_no_extrusion();
}

static bool convert_extrusion_to_json(json& entity_json, json& entity_paths_json, const ExtrusionEntity* extrusion_entity) {
    std::string path_type;
    const ExtrusionPath* path = NULL;
    const ExtrusionMultiPath* multipath = NULL;
    const ExtrusionLoop* loop = NULL;
    const ExtrusionEntityCollection* collection = dynamic_cast<const ExtrusionEntityCollection*>(extrusion_entity);

    if (!collection)
        path = dynamic_cast<const ExtrusionPath*>(extrusion_entity);

    if (!collection && !path)
        multipath = dynamic_cast<const ExtrusionMultiPath*>(extrusion_entity);

    if (!collection && !path && !multipath)
        loop = dynamic_cast<const ExtrusionLoop*>(extrusion_entity);

    path_type = path?JSON_EXTRUSION_TYPE_PATH:(multipath?JSON_EXTRUSION_TYPE_MULTIPATH:(loop?JSON_EXTRUSION_TYPE_LOOP:JSON_EXTRUSION_TYPE_COLLECTION));
    if (path_type.empty()) {
        BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << boost::format(":invalid extrusion path type Found");
        return false;
    }

    entity_json[JSON_EXTRUSION_ENTITY_TYPE] = path_type;

    if (path) {
        json entity_path_json = *path;
        entity_paths_json.push_back(std::move(entity_path_json));
    }
    else if (multipath) {
        for (const ExtrusionPath& extrusion_path : multipath->paths)
        {
            json entity_path_json = extrusion_path;
            entity_paths_json.push_back(std::move(entity_path_json));
        }
    }
    else if (loop) {
        entity_json[JSON_EXTRUSION_LOOP_ROLE] = loop->loop_role();
        for (const ExtrusionPath& extrusion_path : loop->paths)
        {
            json entity_path_json = extrusion_path;
            entity_paths_json.push_back(std::move(entity_path_json));
        }
    }
    else {
        //recursive collections
        entity_json[JSON_EXTRUSION_NO_SORT] = collection->no_sort;
        for (const ExtrusionEntity* recursive_extrusion_entity : collection->entities) {
            json recursive_entity_json, recursive_entity_paths_json = json::array();
            bool ret = convert_extrusion_to_json(recursive_entity_json, recursive_entity_paths_json, recursive_extrusion_entity);
            if (!ret) {
                continue;
            }
            entity_paths_json.push_back(std::move(recursive_entity_json));
        }
    }

    if (collection)
        entity_json[JSON_EXTRUSION_ENTITIES] = std::move(entity_paths_json);
    else
        entity_json[JSON_EXTRUSION_PATHS] = std::move(entity_paths_json);
    return true;
}

static void to_json(json& j, const LayerRegion& layer_region) {
    json unsupported_bridge_edges_json = json::array(), slices_surfaces_json = json::array(), raw_slices_json = json::array(), thin_fills_json, thin_fill_entities_json = json::array();
    json fill_expolygons_json = json::array(), fill_no_overlap_expolygons_json = json::array(), fill_surfaces_json = json::array(), perimeters_json, perimeter_entities_json = json::array(), fills_json, fill_entities_json = json::array();

    j[JSON_LAYER_REGION_CONFIG_HASH] = layer_region.region().config_hash();
    //slices
    for (const Surface& slice_surface : layer_region.slices.surfaces) {
        json surface_json = slice_surface;
        slices_surfaces_json.push_back(std::move(surface_json));
    }
    j.push_back({JSON_LAYER_REGION_SLICES, std::move(slices_surfaces_json)});

    //raw_slices
    for (const ExPolygon& raw_slice_explogyon : layer_region.raw_slices) {
        json raw_polygon_json = raw_slice_explogyon;

        raw_slices_json.push_back(std::move(raw_polygon_json));
    }
    j.push_back({JSON_LAYER_REGION_RAW_SLICES, std::move(raw_slices_json)});

    //thin fills
    thin_fills_json[JSON_EXTRUSION_NO_SORT] = layer_region.thin_fills.no_sort;
    thin_fills_json[JSON_EXTRUSION_ENTITY_TYPE] = JSON_EXTRUSION_TYPE_COLLECTION;
    for (const ExtrusionEntity* extrusion_entity : layer_region.thin_fills.entities) {
        json thinfills_entity_json, thinfill_entity_paths_json = json::array();
        bool ret = convert_extrusion_to_json(thinfills_entity_json, thinfill_entity_paths_json, extrusion_entity);
        if (!ret) {
            BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << boost::format(":error found at print_z %1%") % layer_region.layer()->print_z;
            continue;
        }

        thin_fill_entities_json.push_back(std::move(thinfills_entity_json));
    }
    thin_fills_json[JSON_EXTRUSION_ENTITIES] = std::move(thin_fill_entities_json);
    j.push_back({JSON_LAYER_REGION_THIN_FILLS, std::move(thin_fills_json)});

    //fill_expolygons
    for (const ExPolygon& fill_expolygon : layer_region.fill_expolygons) {
        json fill_expolygon_json = fill_expolygon;

        fill_expolygons_json.push_back(std::move(fill_expolygon_json));
    }
    j.push_back({JSON_LAYER_REGION_FILL_EXPOLYGONS, std::move(fill_expolygons_json)});

    //fill_surfaces
    for (const Surface& fill_surface : layer_region.fill_surfaces.surfaces) {
        json surface_json = fill_surface;
        fill_surfaces_json.push_back(std::move(surface_json));
    }
    j.push_back({JSON_LAYER_REGION_FILL_SURFACES, std::move(fill_surfaces_json)});

    //fill_no_overlap_expolygons
    for (const ExPolygon& fill_no_overlap_expolygon : layer_region.fill_no_overlap_expolygons) {
        json fill_no_overlap_expolygon_json = fill_no_overlap_expolygon;

        fill_no_overlap_expolygons_json.push_back(std::move(fill_no_overlap_expolygon_json));
    }
    j.push_back({JSON_LAYER_REGION_FILL_NO_OVERLAP, std::move(fill_no_overlap_expolygons_json)});

    //unsupported_bridge_edges
    for (const Polyline& poly_line : layer_region.unsupported_bridge_edges)
    {
        json polyline_json = poly_line;

        unsupported_bridge_edges_json.push_back(std::move(polyline_json));
    }
    j.push_back({JSON_LAYER_REGION_UNSUPPORTED_BRIDGE_EDGES, std::move(unsupported_bridge_edges_json)});

    //perimeters
    perimeters_json[JSON_EXTRUSION_NO_SORT] = layer_region.perimeters.no_sort;
    perimeters_json[JSON_EXTRUSION_ENTITY_TYPE] = JSON_EXTRUSION_TYPE_COLLECTION;
    for (const ExtrusionEntity* extrusion_entity : layer_region.perimeters.entities) {
        json perimeters_entity_json, perimeters_entity_paths_json = json::array();
        bool ret = convert_extrusion_to_json(perimeters_entity_json, perimeters_entity_paths_json, extrusion_entity);
        if (!ret)
            continue;

        perimeter_entities_json.push_back(std::move(perimeters_entity_json));
    }
    perimeters_json[JSON_EXTRUSION_ENTITIES] = std::move(perimeter_entities_json);
    j.push_back({JSON_LAYER_REGION_PERIMETERS, std::move(perimeters_json)});

    //fills
    fills_json[JSON_EXTRUSION_NO_SORT] = layer_region.fills.no_sort;
    fills_json[JSON_EXTRUSION_ENTITY_TYPE] = JSON_EXTRUSION_TYPE_COLLECTION;
    for (const ExtrusionEntity* extrusion_entity : layer_region.fills.entities) {
        json fill_entity_json, fill_entity_paths_json = json::array();
        bool ret = convert_extrusion_to_json(fill_entity_json, fill_entity_paths_json, extrusion_entity);
        if (!ret)
            continue;

        fill_entities_json.push_back(std::move(fill_entity_json));
    }
    fills_json[JSON_EXTRUSION_ENTITIES] = std::move(fill_entities_json);
    j.push_back({JSON_LAYER_REGION_FILLS, std::move(fills_json)});

    return;
}

static void to_json(json& j, const groupedVolumeSlices& first_layer_group) {
    json volumes_json = json::array(), slices_json = json::array();
    j[JSON_FIRSTLAYER_GROUP_ID] = first_layer_group.groupId;

    for (const ObjectID& obj_id : first_layer_group.volume_ids)
    {
        volumes_json.push_back(obj_id.id);
    }
    j[JSON_FIRSTLAYER_GROUP_VOLUME_IDS] = std::move(volumes_json);

    for (const ExPolygon& slice_expolygon : first_layer_group.slices) {
        json slice_expolygon_json = slice_expolygon;

        slices_json.push_back(std::move(slice_expolygon_json));
    }
    j[JSON_FIRSTLAYER_GROUP_SLICES] = std::move(slices_json);
}

//load apis from json
static void from_json(const json& j, Points& p_s) {
    int array_size = j.size();
    for (int index = 0; index < array_size/2; index++)
    {
        coord_t x = j[2*index], y = j[2*index+1];
        Point p(x, y);
        p_s.push_back(std::move(p));
    }
    return;
}

static void from_json(const json& j, BoundingBox& bbox) {
    bbox.min[0] = j[0];
    bbox.min[1] = j[1];
    bbox.max[0] = j[2];
    bbox.max[1] = j[3];
    bbox.defined = true;

    return;
}

static void from_json(const json& j, ExPolygon& polygon) {
    polygon.contour.points = j[JSON_POLYGON_CONTOUR];

    int holes_count = j[JSON_POLYGON_HOLES].size();
    for (int holes_index = 0; holes_index < holes_count; holes_index++)
    {
        Polygon poly;

        poly.points = j[JSON_POLYGON_HOLES][holes_index];
        polygon.holes.push_back(std::move(poly));
    }
    return;
}

static void from_json(const json& j, Surface& surf) {
    surf.expolygon = j[JSON_EXPOLYGON];
    surf.surface_type = j[JSON_SURF_TYPE];
    surf.thickness = j[JSON_SURF_THICKNESS];
    surf.thickness_layers = j[JSON_SURF_THICKNESS_LAYER];
    surf.bridge_angle = j[JSON_SURF_BRIDGE_ANGLE];
    surf.extra_perimeters = j[JSON_SURF_EXTRA_PERIMETERS];

    return;
}

static void from_json(const json& j, ArcSegment& arc_seg) {
    arc_seg.is_arc = j[JSON_IS_ARC];
    arc_seg.length = j[JSON_ARC_LENGTH];
    arc_seg.angle_radians = j[JSON_ARC_ANGLE_RADIUS];
    arc_seg.polar_start_theta = j[JSON_ARC_POLAY_START_THETA];
    arc_seg.polar_end_theta = j[JSON_ARC_POLAY_END_THETA];
    arc_seg.start_point.x() = j[JSON_ARC_START_POINT][0];
    arc_seg.start_point.y() = j[JSON_ARC_START_POINT][1];
    arc_seg.end_point.x() = j[JSON_ARC_END_POINT][0];
    arc_seg.end_point.y() = j[JSON_ARC_END_POINT][1];
    arc_seg.direction = j[JSON_ARC_DIRECTION];
    arc_seg.radius    = j[JSON_ARC_RADIUS];
    arc_seg.center.x() = j[JSON_ARC_CENTER][0];
    arc_seg.center.y() = j[JSON_ARC_CENTER][1];

    return;
}


static void from_json(const json& j, Polyline& poly_line) {
    poly_line.points = j[JSON_POINTS];

    int arc_fitting_count = j[JSON_ARC_FITTING].size();
    for (int arc_fitting_index = 0; arc_fitting_index < arc_fitting_count; arc_fitting_index++)
    {
        const json& fitting_json = j[JSON_ARC_FITTING][arc_fitting_index];
        PathFittingData path_fitting;
        path_fitting.start_point_index = fitting_json[JSON_ARC_START_INDEX];
        path_fitting.end_point_index = fitting_json[JSON_ARC_END_INDEX];
        path_fitting.path_type = fitting_json[JSON_ARC_PATH_TYPE];

        if (fitting_json.contains(JSON_ARC_DATA)) {
            path_fitting.arc_data = fitting_json[JSON_ARC_DATA];
        }

        poly_line.fitting_result.push_back(std::move(path_fitting));
    }
    return;
}

static void from_json(const json& j, ExtrusionPath& extrusion_path) {
    Polyline temp_polyline = j[JSON_EXTRUSION_POLYLINE];
    extrusion_path.polyline = Polyline3(temp_polyline);
    extrusion_path.mm3_per_mm             =    j[JSON_EXTRUSION_MM3_PER_MM];
    extrusion_path.width                  =    j[JSON_EXTRUSION_WIDTH];
    extrusion_path.height                 =    j[JSON_EXTRUSION_HEIGHT];
    extrusion_path.set_extrusion_role(j[JSON_EXTRUSION_ROLE]);
    extrusion_path.set_force_no_extrusion(j[JSON_EXTRUSION_NO_EXTRUSION]);
}

static bool convert_extrusion_from_json(const json& entity_json, ExtrusionEntityCollection& entity_collection) {
    std::string path_type = entity_json[JSON_EXTRUSION_ENTITY_TYPE];
    bool ret = false;

    if (path_type == JSON_EXTRUSION_TYPE_PATH) {
        ExtrusionPath* path = new ExtrusionPath();
        if (!path) {
            BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << boost::format(": oom when new ExtrusionPath");
            return false;
        }
        *path = entity_json[JSON_EXTRUSION_PATHS][0];
        entity_collection.entities.push_back(path);
    }
    else if (path_type == JSON_EXTRUSION_TYPE_MULTIPATH) {
        ExtrusionMultiPath* multipath = new ExtrusionMultiPath();
        if (!multipath) {
            BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << boost::format(": oom when new ExtrusionMultiPath");
            return false;
        }
        int paths_count = entity_json[JSON_EXTRUSION_PATHS].size();
        for (int path_index = 0; path_index < paths_count; path_index++)
        {
            ExtrusionPath path;
            path = entity_json[JSON_EXTRUSION_PATHS][path_index];
            multipath->paths.push_back(std::move(path));
        }
        entity_collection.entities.push_back(multipath);
    }
    else if (path_type == JSON_EXTRUSION_TYPE_LOOP) {
        ExtrusionLoop* loop = new ExtrusionLoop();
        if (!loop) {
            BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << boost::format(": oom when new ExtrusionLoop");
            return false;
        }
        loop->set_loop_role(entity_json[JSON_EXTRUSION_LOOP_ROLE]);
        int paths_count = entity_json[JSON_EXTRUSION_PATHS].size();
        for (int path_index = 0; path_index < paths_count; path_index++)
        {
            ExtrusionPath path;
            path = entity_json[JSON_EXTRUSION_PATHS][path_index];
            loop->paths.push_back(std::move(path));
        }
        entity_collection.entities.push_back(loop);
    }
    else if (path_type == JSON_EXTRUSION_TYPE_COLLECTION) {
        ExtrusionEntityCollection* collection = new ExtrusionEntityCollection();
        if (!collection) {
            BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << boost::format(": oom when new ExtrusionEntityCollection");
            return false;
        }
        collection->no_sort = entity_json[JSON_EXTRUSION_NO_SORT];
        int entities_count = entity_json[JSON_EXTRUSION_ENTITIES].size();
        for (int entity_index = 0; entity_index < entities_count; entity_index++)
        {
            const json& entity_item_json = entity_json[JSON_EXTRUSION_ENTITIES][entity_index];
            ret = convert_extrusion_from_json(entity_item_json, *collection);
            if (!ret) {
                BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << boost::format(": convert_extrusion_from_json failed");
                return false;
            }
        }
        entity_collection.entities.push_back(collection);
    }
    else {
        BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << boost::format(": unknown path type %1%")%path_type;
        return false;
    }

    return true;
}

static void convert_layer_region_from_json(const json& j, LayerRegion& layer_region) {
    //slices
    int slices_count = j[JSON_LAYER_REGION_SLICES].size();
    for (int slices_index = 0; slices_index < slices_count; slices_index++)
    {
        Surface surface;

        surface = j[JSON_LAYER_REGION_SLICES][slices_index];
        layer_region.slices.surfaces.push_back(std::move(surface));
    }

    //raw_slices
    int raw_slices_count = j[JSON_LAYER_REGION_RAW_SLICES].size();
    for (int raw_slices_index = 0; raw_slices_index < raw_slices_count; raw_slices_index++)
    {
        ExPolygon polygon;

        polygon = j[JSON_LAYER_REGION_RAW_SLICES][raw_slices_index];
        layer_region.raw_slices.push_back(std::move(polygon));
    }

    //thin fills
    layer_region.thin_fills.no_sort = j[JSON_LAYER_REGION_THIN_FILLS][JSON_EXTRUSION_NO_SORT];
    int thinfills_entities_count = j[JSON_LAYER_REGION_THIN_FILLS][JSON_EXTRUSION_ENTITIES].size();
    for (int thinfills_entities_index = 0; thinfills_entities_index < thinfills_entities_count; thinfills_entities_index++)
    {
        const json& extrusion_entity_json =  j[JSON_LAYER_REGION_THIN_FILLS][JSON_EXTRUSION_ENTITIES][thinfills_entities_index];
        bool ret = convert_extrusion_from_json(extrusion_entity_json, layer_region.thin_fills);
        if (!ret) {
            BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << boost::format(":error parsing thin_fills found at layer %1%, print_z %2%") %layer_region.layer()->id() %layer_region.layer()->print_z;
            char error_buf[1024];
            ::sprintf(error_buf, "Error while parsing thin_fills at layer %zu, print_z %f", layer_region.layer()->id(), layer_region.layer()->print_z);
            throw Slic3r::FileIOError(error_buf);
        }
    }

    //fill_expolygons
    int fill_expolygons_count = j[JSON_LAYER_REGION_FILL_EXPOLYGONS].size();
    for (int fill_expolygons_index = 0; fill_expolygons_index < fill_expolygons_count; fill_expolygons_index++)
    {
        ExPolygon polygon;

        polygon = j[JSON_LAYER_REGION_FILL_EXPOLYGONS][fill_expolygons_index];
        layer_region.fill_expolygons.push_back(std::move(polygon));
    }

    //fill_surfaces
    int fill_surfaces_count = j[JSON_LAYER_REGION_FILL_SURFACES].size();
    for (int fill_surfaces_index = 0; fill_surfaces_index < fill_surfaces_count; fill_surfaces_index++)
    {
        Surface surface;

        surface = j[JSON_LAYER_REGION_FILL_SURFACES][fill_surfaces_index];
        layer_region.fill_surfaces.surfaces.push_back(std::move(surface));
    }

    //fill_no_overlap_expolygons
    int fill_no_overlap_expolygons_count = j[JSON_LAYER_REGION_FILL_NO_OVERLAP].size();
    for (int fill_no_overlap_expolygons_index = 0; fill_no_overlap_expolygons_index < fill_no_overlap_expolygons_count; fill_no_overlap_expolygons_index++)
    {
        ExPolygon polygon;

        polygon = j[JSON_LAYER_REGION_FILL_NO_OVERLAP][fill_no_overlap_expolygons_index];
        layer_region.fill_no_overlap_expolygons.push_back(std::move(polygon));
    }

    //unsupported_bridge_edges
    int unsupported_bridge_edges_count = j[JSON_LAYER_REGION_UNSUPPORTED_BRIDGE_EDGES].size();
    for (int unsupported_bridge_edges_index = 0; unsupported_bridge_edges_index < unsupported_bridge_edges_count; unsupported_bridge_edges_index++)
    {
        Polyline polyline;

        polyline = j[JSON_LAYER_REGION_UNSUPPORTED_BRIDGE_EDGES][unsupported_bridge_edges_index];
        layer_region.unsupported_bridge_edges.push_back(std::move(polyline));
    }

    //perimeters
    layer_region.perimeters.no_sort = j[JSON_LAYER_REGION_PERIMETERS][JSON_EXTRUSION_NO_SORT];
    int perimeters_entities_count = j[JSON_LAYER_REGION_PERIMETERS][JSON_EXTRUSION_ENTITIES].size();
    for (int perimeters_entities_index = 0; perimeters_entities_index < perimeters_entities_count; perimeters_entities_index++)
    {
        const json& extrusion_entity_json =  j[JSON_LAYER_REGION_PERIMETERS][JSON_EXTRUSION_ENTITIES][perimeters_entities_index];
        bool ret = convert_extrusion_from_json(extrusion_entity_json, layer_region.perimeters);
        if (!ret) {
            BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << boost::format(": error parsing perimeters found at layer %1%, print_z %2%") %layer_region.layer()->id() %layer_region.layer()->print_z;
            char error_buf[1024];
            ::sprintf(error_buf, "Error while parsing perimeters at layer %zu, print_z %f", layer_region.layer()->id(), layer_region.layer()->print_z);
            throw Slic3r::FileIOError(error_buf);
        }
    }

    //fills
    layer_region.fills.no_sort = j[JSON_LAYER_REGION_FILLS][JSON_EXTRUSION_NO_SORT];
    int fills_entities_count = j[JSON_LAYER_REGION_FILLS][JSON_EXTRUSION_ENTITIES].size();
    for (int fills_entities_index = 0; fills_entities_index < fills_entities_count; fills_entities_index++)
    {
        const json& extrusion_entity_json =  j[JSON_LAYER_REGION_FILLS][JSON_EXTRUSION_ENTITIES][fills_entities_index];
        bool ret = convert_extrusion_from_json(extrusion_entity_json, layer_region.fills);
        if (!ret) {
            BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << boost::format(": error parsing fills found at layer %1%, print_z %2%") %layer_region.layer()->id() %layer_region.layer()->print_z;
            char error_buf[1024];
            ::sprintf(error_buf, "Error while parsing fills at layer %zu, print_z %f", layer_region.layer()->id(), layer_region.layer()->print_z);
            throw Slic3r::FileIOError(error_buf);
        }
    }

    return;
}


void extract_layer(const json& layer_json, Layer& layer) {
    //slice_polygons
    int slice_polygons_count = layer_json[JSON_LAYER_SLICED_POLYGONS].size();
    for (int polygon_index = 0; polygon_index < slice_polygons_count; polygon_index++)
    {
        ExPolygon polygon;

        polygon = layer_json[JSON_LAYER_SLICED_POLYGONS][polygon_index];
        layer.lslices.push_back(std::move(polygon));
    }

    //slice_bboxes
    int sliced_bboxes_count = layer_json[JSON_LAYER_SLLICED_BBOXES].size();
    for (int bbox_index = 0; bbox_index < sliced_bboxes_count; bbox_index++)
    {
        BoundingBox bbox;

        bbox = layer_json[JSON_LAYER_SLLICED_BBOXES][bbox_index];
        layer.lslices_bboxes.push_back(std::move(bbox));
    }

    //overhang_polygons
    int overhang_polygons_count = layer_json[JSON_LAYER_OVERHANG_POLYGONS].size();
    for (int polygon_index = 0; polygon_index < overhang_polygons_count; polygon_index++)
    {
        ExPolygon polygon;

        polygon = layer_json[JSON_LAYER_OVERHANG_POLYGONS][polygon_index];
        layer.loverhangs.push_back(std::move(polygon));
    }

    //overhang_box
    layer.loverhangs_bbox = layer_json[JSON_LAYER_OVERHANG_BBOX];

    //layer_regions
    int layer_region_count = layer.region_count();
    for (int layer_region_index = 0; layer_region_index < layer_region_count; layer_region_index++)
    {
        LayerRegion* layer_region = layer.get_region(layer_region_index);
        const json& layer_region_json = layer_json[JSON_LAYER_REGIONS][layer_region_index];
        convert_layer_region_from_json(layer_region_json, *layer_region);

        //LayerRegion layer_region = layer_json[JSON_LAYER_REGIONS][layer_region_index];
    }

    return;
}

void extract_support_layer(const json& support_layer_json, SupportLayer& support_layer) {
    extract_layer(support_layer_json, support_layer);

    support_layer.support_type = support_layer_json[JSON_SUPPORT_LAYER_TYPE];
    //support_islands
    int islands_count = support_layer_json[JSON_SUPPORT_LAYER_ISLANDS].size();
    for (int islands_index = 0; islands_index < islands_count; islands_index++)
    {
        ExPolygon polygon;

        polygon = support_layer_json[JSON_SUPPORT_LAYER_ISLANDS][islands_index];
        support_layer.support_islands.push_back(std::move(polygon));
    }

    //support_fills
    support_layer.support_fills.no_sort = support_layer_json[JSON_SUPPORT_LAYER_FILLS][JSON_EXTRUSION_NO_SORT];
    int support_fills_entities_count = support_layer_json[JSON_SUPPORT_LAYER_FILLS][JSON_EXTRUSION_ENTITIES].size();
    for (int support_fills_entities_index = 0; support_fills_entities_index < support_fills_entities_count; support_fills_entities_index++)
    {
        const json& extrusion_entity_json =  support_layer_json[JSON_SUPPORT_LAYER_FILLS][JSON_EXTRUSION_ENTITIES][support_fills_entities_index];
        bool ret = convert_extrusion_from_json(extrusion_entity_json, support_layer.support_fills);
        if (!ret) {
            BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << boost::format(": error parsing fills found at support_layer %1%, print_z %2%")%support_layer.id() %support_layer.print_z;
            char error_buf[1024];
            ::sprintf(error_buf, "Error while parsing fills at support_layer %zu, print_z %f", support_layer.id(), support_layer.print_z);
            throw Slic3r::FileIOError(error_buf);
        }
    }

    // The body the fine nozzle lays and the filament it uses (mixed-nozzle support).
    if (support_layer_json.contains(JSON_SUPPORT_LAYER_BASE_ON_INTERFACE_NOZZLE))
        support_layer.base_on_interface_nozzle = support_layer_json[JSON_SUPPORT_LAYER_BASE_ON_INTERFACE_NOZZLE];
    if (support_layer_json.contains(JSON_SUPPORT_LAYER_FINE_BODY_FILAMENT))
        support_layer.interface_nozzle_body_filament = support_layer_json[JSON_SUPPORT_LAYER_FINE_BODY_FILAMENT];
    if (support_layer_json.contains(JSON_SUPPORT_LAYER_FINE_BODY_FILLS))
        for (const json &extrusion_entity_json : support_layer_json[JSON_SUPPORT_LAYER_FINE_BODY_FILLS])
            if (! convert_extrusion_from_json(extrusion_entity_json, support_layer.fine_body_fills))
                throw Slic3r::FileIOError("Error while parsing the fine support body at a support layer");

    return;
}

static void from_json(const json& j, groupedVolumeSlices& firstlayer_group)
{
    firstlayer_group.groupId               =   j[JSON_FIRSTLAYER_GROUP_ID];

    int volume_count = j[JSON_FIRSTLAYER_GROUP_VOLUME_IDS].size();
    for (int volume_index = 0; volume_index < volume_count; volume_index++)
    {
        ObjectID obj_id;

        obj_id.id = j[JSON_FIRSTLAYER_GROUP_VOLUME_IDS][volume_index];
        firstlayer_group.volume_ids.push_back(std::move(obj_id));
    }

    int slices_count = j[JSON_FIRSTLAYER_GROUP_SLICES].size();
    for (int slice_index = 0; slice_index < slices_count; slice_index++)
    {
        ExPolygon polygon;

        polygon = j[JSON_FIRSTLAYER_GROUP_SLICES][slice_index];
        firstlayer_group.slices.push_back(std::move(polygon));
    }
}

int Print::export_cached_data(const std::string& directory, bool with_space)
{
    if (is_mixed_nozzle_body_split(m_config))
        throw Slic3r::SlicingError("[SRL-C03] Saving slice data for reuse is not available with Body Split.");
    int ret = 0;
    boost::filesystem::path directory_path(directory);

    auto convert_layer_to_json = [](json& layer_json, const Layer* layer) {
        json slice_polygons_json = json::array(), slice_bboxs_json = json::array(), overhang_polygons_json = json::array(), layer_regions_json = json::array();
        layer_json[JSON_LAYER_PRINT_Z] = layer->print_z;
        layer_json[JSON_LAYER_HEIGHT] = layer->height;
        layer_json[JSON_LAYER_SLICE_Z] = layer->slice_z;
        layer_json[JSON_LAYER_ID] = layer->id();
        //layer_json["slicing_errors"] = layer->slicing_errors;

        //sliced_polygons
        for (const ExPolygon& slice_polygon : layer->lslices) {
            json slice_polygon_json = slice_polygon;
            slice_polygons_json.push_back(std::move(slice_polygon_json));
        }
        layer_json[JSON_LAYER_SLICED_POLYGONS] = std::move(slice_polygons_json);

        //sliced_bbox
        for (const BoundingBox& slice_bbox : layer->lslices_bboxes) {
            json bbox_json = json::array();

            bbox_json = slice_bbox;
            slice_bboxs_json.push_back(std::move(bbox_json));
        }
        layer_json[JSON_LAYER_SLLICED_BBOXES] = std::move(slice_bboxs_json);

        //overhang_polygons
        for (const ExPolygon& overhang_polygon : layer->loverhangs) {
            json overhang_polygon_json = overhang_polygon;
            overhang_polygons_json.push_back(std::move(overhang_polygon_json));
        }
        layer_json[JSON_LAYER_OVERHANG_POLYGONS] = std::move(overhang_polygons_json);

        //overhang_box
        layer_json[JSON_LAYER_OVERHANG_BBOX] = layer->loverhangs_bbox;

        for (const LayerRegion *layer_region : layer->regions()) {
            json region_json = *layer_region;

            layer_regions_json.push_back(std::move(region_json));
        }
        layer_json[JSON_LAYER_REGIONS] = std::move(layer_regions_json);

        return;
    };

    //firstly clear this directory
    if (fs::exists(directory_path)) {
        fs::remove_all(directory_path);
    }
    try {
        if (!fs::create_directory(directory_path)) {
            BOOST_LOG_TRIVIAL(error) << boost::format("create directory %1% failed")%directory;
            return CLI_EXPORT_CACHE_DIRECTORY_CREATE_FAILED;
        }
    }
    catch (...)
    {
        BOOST_LOG_TRIVIAL(error) << boost::format("create directory %1% failed")%directory;
        return CLI_EXPORT_CACHE_DIRECTORY_CREATE_FAILED;
    }

    int count = 0;
    std::vector<std::string> filename_vector;
    std::vector<json> json_vector;
    for (PrintObject *obj : m_objects) {
        const ModelObject* model_obj = obj->model_object();
        if (obj->get_shared_object()) {
            BOOST_LOG_TRIVIAL(info) << boost::format("shared object %1%, skip directly")%model_obj->name;
            continue;
        }

        const PrintInstance &print_instance = obj->instances()[0];
        const ModelInstance *model_instance = print_instance.model_instance;
        size_t identify_id = (model_instance->loaded_id > 0)?model_instance->loaded_id: model_instance->id().id;
        std::string file_name = directory +"/obj_"+std::to_string(identify_id)+".json";

        BOOST_LOG_TRIVIAL(info) << boost::format("begin to dump object %1%, identify_id %2% to %3%")%model_obj->name %identify_id %file_name;

        try {
            json root_json, layers_json = json::array(), support_layers_json = json::array(), first_layer_groups = json::array();

            root_json[JSON_OBJECT_NAME] = model_obj->name;
            root_json[JSON_IDENTIFY_ID] = identify_id;

            //export the layers
            std::vector<json> layers_json_vector(obj->layer_count());
            tbb::parallel_for(
                tbb::blocked_range<size_t>(0, obj->layer_count()),
                [&layers_json_vector, obj, convert_layer_to_json](const tbb::blocked_range<size_t>& layer_range) {
                    for (size_t layer_index = layer_range.begin(); layer_index < layer_range.end(); ++ layer_index) {
                        const Layer *layer = obj->get_layer(layer_index);
                        json layer_json;
                        convert_layer_to_json(layer_json, layer);
                        layers_json_vector[layer_index] = std::move(layer_json);
                    }
                }
            );
            for (int l_index = 0; l_index < layers_json_vector.size(); l_index++) {
                layers_json.push_back(std::move(layers_json_vector[l_index]));
            }
            layers_json_vector.clear();
            /*for (const Layer *layer : obj->layers()) {
                // for each layer
                json layer_json;

                convert_layer_to_json(layer_json, layer);

                layers_json.push_back(std::move(layer_json));
            }*/

            root_json[JSON_LAYERS] = std::move(layers_json);

            //export the support layers
            std::vector<json> support_layers_json_vector(obj->support_layer_count());
            tbb::parallel_for(
                tbb::blocked_range<size_t>(0, obj->support_layer_count()),
                [&support_layers_json_vector, obj, convert_layer_to_json](const tbb::blocked_range<size_t>& support_layer_range) {
                    for (size_t s_layer_index = support_layer_range.begin(); s_layer_index < support_layer_range.end(); ++ s_layer_index) {
                        const SupportLayer *support_layer = obj->get_support_layer(s_layer_index);
                        json support_layer_json, support_islands_json = json::array(), support_fills_json, supportfills_entities_json = json::array();

                        convert_layer_to_json(support_layer_json, support_layer);

                        support_layer_json[JSON_SUPPORT_LAYER_INTERFACE_ID] = support_layer->interface_id();
                        support_layer_json[JSON_SUPPORT_LAYER_TYPE] = support_layer->support_type;

                        //support_islands
                        for (const ExPolygon& support_island : support_layer->support_islands) {
                            json support_island_json = support_island;
                            support_islands_json.push_back(std::move(support_island_json));
                        }
                        support_layer_json[JSON_SUPPORT_LAYER_ISLANDS] = std::move(support_islands_json);

                        //support_fills
                        support_fills_json[JSON_EXTRUSION_NO_SORT] = support_layer->support_fills.no_sort;
                        support_fills_json[JSON_EXTRUSION_ENTITY_TYPE] = JSON_EXTRUSION_TYPE_COLLECTION;
                        for (const ExtrusionEntity* extrusion_entity : support_layer->support_fills.entities) {
                            json supportfill_entity_json, supportfill_entity_paths_json = json::array();
                            bool ret = convert_extrusion_to_json(supportfill_entity_json, supportfill_entity_paths_json, extrusion_entity);
                            if (!ret)
                                continue;

                            supportfills_entities_json.push_back(std::move(supportfill_entity_json));
                        }
                        support_fills_json[JSON_EXTRUSION_ENTITIES] = std::move(supportfills_entities_json);
                        support_layer_json[JSON_SUPPORT_LAYER_FILLS] = std::move(support_fills_json);

                        // The body the fine nozzle lays and the filament it uses (mixed-nozzle support).
                        support_layer_json[JSON_SUPPORT_LAYER_BASE_ON_INTERFACE_NOZZLE] = support_layer->base_on_interface_nozzle;
                        support_layer_json[JSON_SUPPORT_LAYER_FINE_BODY_FILAMENT] = support_layer->interface_nozzle_body_filament;
                        json fine_body_json = json::array();
                        for (const ExtrusionEntity *extrusion_entity : support_layer->fine_body_fills.entities) {
                            json entity_json, entity_paths_json = json::array();
                            if (convert_extrusion_to_json(entity_json, entity_paths_json, extrusion_entity))
                                fine_body_json.push_back(std::move(entity_json));
                        }
                        support_layer_json[JSON_SUPPORT_LAYER_FINE_BODY_FILLS] = std::move(fine_body_json);

                        support_layers_json_vector[s_layer_index] = std::move(support_layer_json);
                    }
                }
            );
            for (int s_index = 0; s_index < support_layers_json_vector.size(); s_index++) {
                support_layers_json.push_back(std::move(support_layers_json_vector[s_index]));
            }
            support_layers_json_vector.clear();

            /*for (const SupportLayer *support_layer : obj->support_layers()) {
                json support_layer_json, support_islands_json = json::array(), support_fills_json, supportfills_entities_json = json::array();

                convert_layer_to_json(support_layer_json, support_layer);

                support_layer_json[JSON_SUPPORT_LAYER_INTERFACE_ID] = support_layer->interface_id();

                //support_islands
                for (const ExPolygon& support_island : support_layer->support_islands.expolygons) {
                    json support_island_json = support_island;
                    support_islands_json.push_back(std::move(support_island_json));
                }
                support_layer_json[JSON_SUPPORT_LAYER_ISLANDS] = std::move(support_islands_json);

                //support_fills
                support_fills_json[JSON_EXTRUSION_NO_SORT] = support_layer->support_fills.no_sort;
                support_fills_json[JSON_EXTRUSION_ENTITY_TYPE] = JSON_EXTRUSION_TYPE_COLLECTION;
                for (const ExtrusionEntity* extrusion_entity : support_layer->support_fills.entities) {
                    json supportfill_entity_json, supportfill_entity_paths_json = json::array();
                    bool ret = convert_extrusion_to_json(supportfill_entity_json, supportfill_entity_paths_json, extrusion_entity);
                    if (!ret)
                        continue;

                    supportfills_entities_json.push_back(std::move(supportfill_entity_json));
                }
                support_fills_json[JSON_EXTRUSION_ENTITIES] = std::move(supportfills_entities_json);
                support_layer_json[JSON_SUPPORT_LAYER_FILLS] = std::move(support_fills_json);

                support_layers_json.push_back(std::move(support_layer_json));
            } // for each layer*/
            root_json[JSON_SUPPORT_LAYERS] = std::move(support_layers_json);

            const std::vector<groupedVolumeSlices> &first_layer_obj_groups =  obj->firstLayerObjGroups();
            for (size_t s_group_index = 0; s_group_index < first_layer_obj_groups.size(); ++ s_group_index) {
                groupedVolumeSlices group = first_layer_obj_groups[s_group_index];

                //convert the id
                for (ObjectID& obj_id : group.volume_ids)
                {
                    const ModelVolume* currentModelVolumePtr = nullptr;
                    //BBS: support shared object logic
                    const PrintObject* shared_object = obj->get_shared_object();
                    if (!shared_object)
                        shared_object = obj;
                    const ModelVolumePtrs& volumes_ptr = shared_object->model_object()->volumes;
                    size_t volume_count = volumes_ptr.size();
                    for (size_t index = 0; index < volume_count; index ++) {
                        currentModelVolumePtr = volumes_ptr[index];
                        if (currentModelVolumePtr->id() == obj_id) {
                            obj_id.id = index;
                            break;
                        }
                    }
                }

                json first_layer_group_json;

                first_layer_group_json = group;
                first_layer_groups.push_back(std::move(first_layer_group_json));
            }
            root_json[JSON_FIRSTLAYER_GROUPS] = std::move(first_layer_groups);

            filename_vector.push_back(file_name);
            json_vector.push_back(std::move(root_json));
            /*boost::nowide::ofstream c;
            c.open(file_name, std::ios::out | std::ios::trunc);
            if (with_space)
                c << root_json.dump(1, '\t') << std::endl;
            else
                c << root_json.dump(0) << std::endl;
            c.close();*/
            count ++;
            BOOST_LOG_TRIVIAL(info) << boost::format("will dump object %1%'s json to %2%.")%model_obj->name%file_name;
        }
        catch(std::exception &err) {
            BOOST_LOG_TRIVIAL(error) << __FUNCTION__<< ": save to "<<file_name<<" got a generic exception, reason = " << err.what();
            ret = CLI_EXPORT_CACHE_WRITE_FAILED;
        }
    }

    boost::mutex mutex;
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, filename_vector.size()),
        [filename_vector, &json_vector, with_space, &ret, &mutex](const tbb::blocked_range<size_t>& output_range) {
            for (size_t object_index = output_range.begin(); object_index < output_range.end(); ++ object_index) {
                try {
                    boost::nowide::ofstream c;
                    c.open(filename_vector[object_index], std::ios::out | std::ios::trunc);
                    if (with_space)
                        c << json_vector[object_index].dump(1, '\t') << std::endl;
                    else
                        c << json_vector[object_index].dump(0) << std::endl;
                    c.close();
                }
                catch(std::exception &err) {
                    BOOST_LOG_TRIVIAL(error) << __FUNCTION__<< ": save to "<<filename_vector[object_index]<<" got a generic exception, reason = " << err.what();
                    boost::unique_lock l(mutex);
                    ret = CLI_EXPORT_CACHE_WRITE_FAILED;
                }
            }
        }
    );

    BOOST_LOG_TRIVIAL(info) << __FUNCTION__<< boost::format(": total printobject count %1%, saved %2%, ret=%3%")%m_objects.size() %count %ret;
    return ret;
}


int Print::load_cached_data(const std::string& directory)
{
    if (is_mixed_nozzle_body_split(m_config))
        throw Slic3r::SlicingError("[SRL-C03] Saving slice data for reuse is not available with Body Split.");
    int ret = 0;
    boost::filesystem::path directory_path(directory);

    if (!fs::exists(directory_path)) {
        BOOST_LOG_TRIVIAL(info) << boost::format("directory %1% not exist.")%directory;
        return CLI_IMPORT_CACHE_NOT_FOUND;
    }

    auto find_region = [this](PrintObject* object, size_t config_hash) -> const PrintRegion* {
        int regions_count = object->num_printing_regions();
        for (int index = 0; index < regions_count; index++ )
        {
            const PrintRegion&  print_region = object->printing_region(index);
            if (print_region.config_hash() == config_hash ) {
                return &print_region;
            }
        }
        return NULL;
    };

    int count = 0;
    std::vector<std::pair<std::string, PrintObject*>> object_filenames;
    for (PrintObject *obj : m_objects) {
        const ModelObject* model_obj = obj->model_object();
        const PrintInstance &print_instance = obj->instances()[0];
        const ModelInstance *model_instance = print_instance.model_instance;

        obj->clear_layers();
        obj->clear_support_layers();

        int identify_id = model_instance->loaded_id;
        if (identify_id <= 0) {
            //for old 3mf
            identify_id = model_instance->id().id;
            BOOST_LOG_TRIVIAL(info) << __FUNCTION__<< boost::format(": object %1%'s loaded_id is 0, need to use the instance_id %2%")%model_obj->name %identify_id;
            //continue;
        }
        std::string file_name = directory +"/obj_"+std::to_string(identify_id)+".json";

        if (!fs::exists(file_name)) {
            BOOST_LOG_TRIVIAL(info) << __FUNCTION__<<boost::format(": file %1% not exist, maybe a shared object, skip it")%file_name;
            continue;
        }
        object_filenames.push_back({file_name, obj});
    }

    boost::mutex mutex;
    std::vector<json> object_jsons(object_filenames.size());
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, object_filenames.size()),
        [object_filenames, &ret, &object_jsons, &mutex](const tbb::blocked_range<size_t>& filename_range) {
            for (size_t filename_index = filename_range.begin(); filename_index < filename_range.end(); ++ filename_index) {
                try {
                    json root_json;
                    boost::nowide::ifstream ifs(object_filenames[filename_index].first);
                    ifs >> root_json;
                    object_jsons[filename_index] = std::move(root_json);
                }
                catch(std::exception &err) {
                    BOOST_LOG_TRIVIAL(error) << __FUNCTION__<< ": load from "<<object_filenames[filename_index].first<<" got a generic exception, reason = " << err.what();
                    boost::unique_lock l(mutex);
                    ret = CLI_IMPORT_CACHE_LOAD_FAILED;
                }
            }
        }
    );

    if (ret) {
        BOOST_LOG_TRIVIAL(error) << __FUNCTION__<< boost::format(": load json failed.");
        return ret;
    }

    for (int obj_index = 0; obj_index < object_jsons.size(); obj_index++) {
        json& root_json = object_jsons[obj_index];
        PrintObject *obj = object_filenames[obj_index].second;

        try {
            //boost::nowide::ifstream ifs(file_name);
            //ifs >> root_json;

            std::string name = root_json.at(JSON_OBJECT_NAME);
            int identify_id = root_json.at(JSON_IDENTIFY_ID);
            int layer_count = 0, support_layer_count = 0, firstlayer_group_count = 0;

            layer_count = root_json[JSON_LAYERS].size();
            support_layer_count = root_json[JSON_SUPPORT_LAYERS].size();
            firstlayer_group_count = root_json[JSON_FIRSTLAYER_GROUPS].size();

            BOOST_LOG_TRIVIAL(info) << __FUNCTION__<<boost::format(":will load %1%, identify_id %2%, layer_count %3%, support_layer_count %4%, firstlayer_group_count %5%")
                %name %identify_id %layer_count %support_layer_count %firstlayer_group_count;

            Layer* previous_layer = NULL;
            //create layer and layer regions
            for (int index = 0; index < layer_count; index++)
            {
                json& layer_json = root_json[JSON_LAYERS][index];
                Layer* new_layer = obj->add_layer(layer_json[JSON_LAYER_ID], layer_json[JSON_LAYER_HEIGHT], layer_json[JSON_LAYER_PRINT_Z], layer_json[JSON_LAYER_SLICE_Z]);
                if (!new_layer) {
                    BOOST_LOG_TRIVIAL(error) <<__FUNCTION__<< boost::format(":create_layer failed, out of memory");
                    return CLI_OUT_OF_MEMORY;
                }
                if (previous_layer) {
                    previous_layer->upper_layer = new_layer;
                    new_layer->lower_layer = previous_layer;
                }
                previous_layer = new_layer;

                //layer regions
                int layer_regions_count = layer_json[JSON_LAYER_REGIONS].size();
                for (int region_index = 0; region_index < layer_regions_count; region_index++)
                {
                    json& region_json = layer_json[JSON_LAYER_REGIONS][region_index];
                    size_t config_hash = region_json[JSON_LAYER_REGION_CONFIG_HASH];
                    const PrintRegion *print_region = find_region(obj, config_hash);

                    if (!print_region){
                        BOOST_LOG_TRIVIAL(error) <<__FUNCTION__<< boost::format(":can not find print region of object %1%, layer %2%, print_z %3%, layer_region %4%")
                            %name % index %new_layer->print_z %region_index;
                        //delete new_layer;
                        return CLI_IMPORT_CACHE_DATA_CAN_NOT_USE;
                    }

                    new_layer->add_region(print_region);
                }

            }

            //load the layer data parallel
            BOOST_LOG_TRIVIAL(info) << __FUNCTION__<<boost::format(": load the layers in parallel");
            tbb::parallel_for(
                tbb::blocked_range<size_t>(0, obj->layer_count()),
                [&root_json, &obj](const tbb::blocked_range<size_t>& layer_range) {
                    for (size_t layer_index = layer_range.begin(); layer_index < layer_range.end(); ++ layer_index) {
                        const json& layer_json = root_json[JSON_LAYERS][layer_index];
                        Layer* layer = obj->get_layer(layer_index);
                        extract_layer(layer_json, *layer);
                    }
                }
            );

            //support layers
            Layer* previous_support_layer = NULL;
            //create support_layers
            for (int index = 0; index < support_layer_count; index++)
            {
                json& layer_json = root_json[JSON_SUPPORT_LAYERS][index];
                SupportLayer* new_support_layer = obj->add_support_layer(layer_json[JSON_LAYER_ID], layer_json[JSON_SUPPORT_LAYER_INTERFACE_ID], layer_json[JSON_LAYER_HEIGHT], layer_json[JSON_LAYER_PRINT_Z]);
                if (!new_support_layer) {
                    BOOST_LOG_TRIVIAL(error) <<__FUNCTION__<< boost::format(":add_support_layer failed, out of memory");
                    return CLI_OUT_OF_MEMORY;
                }
                if (previous_support_layer) {
                    previous_support_layer->upper_layer = new_support_layer;
                    new_support_layer->lower_layer = previous_support_layer;
                }
                previous_support_layer = new_support_layer;
            }

            BOOST_LOG_TRIVIAL(info) << __FUNCTION__<< boost::format(": finished load layers, start to load support_layers.");
            tbb::parallel_for(
                tbb::blocked_range<size_t>(0, obj->support_layer_count()),
                [&root_json, &obj](const tbb::blocked_range<size_t>& support_layer_range) {
                    for (size_t layer_index = support_layer_range.begin(); layer_index < support_layer_range.end(); ++ layer_index) {
                        const json& layer_json = root_json[JSON_SUPPORT_LAYERS][layer_index];
                        SupportLayer* support_layer = obj->get_support_layer(layer_index);
                        extract_support_layer(layer_json, *support_layer);
                    }
                }
            );

            //load first group volumes
            std::vector<groupedVolumeSlices>& firstlayer_objgroups = obj->firstLayerObjGroupsMod();
            for (int index = 0; index < firstlayer_group_count; index++)
            {
                json& firstlayer_group_json = root_json[JSON_FIRSTLAYER_GROUPS][index];
                groupedVolumeSlices firstlayer_group = firstlayer_group_json;
                //convert the id
                for (ObjectID& obj_id : firstlayer_group.volume_ids)
                {
                    ModelVolume* currentModelVolumePtr = nullptr;
                    ModelVolumePtrs& volumes_ptr = obj->model_object()->volumes;
                    size_t volume_count = volumes_ptr.size();
                    if (obj_id.id < volume_count) {
                        currentModelVolumePtr = volumes_ptr[obj_id.id];
                        obj_id = currentModelVolumePtr->id();
                    }
                    else {
                        BOOST_LOG_TRIVIAL(error) << __FUNCTION__<< boost::format(": can not find volume_id %1% from object file %2% in firstlayer groups, volume_count %3%!")
                            %obj_id.id %object_filenames[obj_index].first %volume_count;
                        return CLI_IMPORT_CACHE_LOAD_FAILED;
                    }
                }
                firstlayer_objgroups.push_back(std::move(firstlayer_group));
            }

            count ++;
            BOOST_LOG_TRIVIAL(info) << __FUNCTION__<< boost::format(": load object %1% from %2% successfully.")%count%object_filenames[obj_index].first;
        }
        catch(nlohmann::detail::parse_error &err) {
            BOOST_LOG_TRIVIAL(error) << __FUNCTION__<< ": parse "<<object_filenames[obj_index].first<<" got a nlohmann::detail::parse_error, reason = " << err.what();
            return CLI_IMPORT_CACHE_LOAD_FAILED;
        }
        catch(std::exception &err) {
            BOOST_LOG_TRIVIAL(error) << __FUNCTION__<< ": load from "<<object_filenames[obj_index].first<<" got a generic exception, reason = " << err.what();
            ret = CLI_IMPORT_CACHE_LOAD_FAILED;
        }
    }

    object_jsons.clear();
    object_filenames.clear();
    BOOST_LOG_TRIVIAL(info) << __FUNCTION__<< boost::format(": total printobject count %1%, loaded %2%, ret=%3%")%m_objects.size() %count %ret;
    return ret;
}

BoundingBoxf3 PrintInstance::get_bounding_box() const {
    return print_object->model_object()->instance_bounding_box(*model_instance, false);
}

Polygon PrintInstance::get_convex_hull_2d() {
    Polygon poly = print_object->model_object()->convex_hull_2d(model_instance->get_matrix());
    poly.douglas_peucker(0.1);
    return poly;
}

//BBS: instance_shift is too large because of multi-plate, apply without plate offset.
Point PrintInstance::shift_without_plate_offset() const
{
    const Print* print = print_object->print();
    const Vec3d plate_offset = print->get_plate_origin();
    return shift - Point(scaled(plate_offset.x()), scaled(plate_offset.y()));
}

PrintRegion *PrintObjectRegions::FuzzySkinPaintedRegion::parent_print_object_region(const LayerRangeRegions &layer_range) const
{
    using FuzzySkinParentType = PrintObjectRegions::FuzzySkinPaintedRegion::ParentType;

    if (this->parent_type == FuzzySkinParentType::PaintedRegion) {
        return layer_range.painted_regions[this->parent].region;
    }

    assert(this->parent_type == FuzzySkinParentType::VolumeRegion);
    return layer_range.volume_regions[this->parent].region;
}

int PrintObjectRegions::FuzzySkinPaintedRegion::parent_print_object_region_id(const LayerRangeRegions &layer_range) const
{
    return this->parent_print_object_region(layer_range)->print_object_region_id();
}

bool FakeWipeTower::fake_extrusion_data_valid() const
{
    const auto usable = [](float v) { return std::isfinite(v) && v > 0.f && v < 1e4f; };
    std::string problem;
    if (!usable(width) || !usable(height) || !usable(depth) || !usable(layer_height))
        problem = "size or layer height out of range";
    else if (height / layer_height > float(max_fake_layers))
        problem = "too many layers";
    else if (!std::isfinite(brim_width) || brim_width < 0.f || brim_width > 1e3f || !std::isfinite(rotation_angle) ||
             !std::isfinite(cone_angle) || !pos.allFinite())
        problem = "brim, rotation or cone out of range";
    else if (z_and_depth_pairs.empty())
        problem = "no depth steps";
    else
        for (const auto &[z, d] : z_and_depth_pairs)
            if (!std::isfinite(z) || !std::isfinite(d) || d < 0.f || d >= 1e4f) {
                problem = "depth step out of range";
                break;
            }
    if (problem.empty())
        return true;
    BOOST_LOG_TRIVIAL(error) << "Wipe tower conflict check skipped, fake tower data unusable: " << problem << " (width " << width
                             << ", height " << height << ", depth " << depth << ", layer height " << layer_height << ", "
                             << z_and_depth_pairs.size() << " depth steps)";
    return false;
}

ExtrusionLayers FakeWipeTower::getTrueExtrusionLayersFromWipeTower() const
{
    ExtrusionLayers wtels;
    wtels.type = ExtrusionLayersType::WIPE_TOWER;

    //ORCA: Fallback for WipeTower2 if outer_wall is empty
    if (outer_wall.empty()) {
        auto fake_paths = getFakeExtrusionPathsFromWipeTower2();
        float current_z = 0.f;
        for (auto& layer_paths : fake_paths) {
            if (layer_paths.empty()) continue;
            ExtrusionLayer el;
            float lh = layer_paths.front().height;
            el.height = lh;
            el.bottom_z = current_z;
            el.layer = nullptr;
            el.paths = std::move(layer_paths);
            wtels.push_back(std::move(el));
            current_z += lh;
        }
        return wtels;
    }

    std::vector<float> layer_heights;
    layer_heights.reserve(outer_wall.size());
    auto pre = outer_wall.begin();
    for (auto it = outer_wall.begin(); it != outer_wall.end(); ++it) {
        if (it == outer_wall.begin())
            layer_heights.push_back(it->first);
        else {
            layer_heights.push_back(it->first - pre->first);
            ++pre;
        }
    }
    Point trans = {scale_(pos.x()), scale_(pos.y())};
    for (auto it = outer_wall.begin(); it != outer_wall.end(); ++it) {
        int            index = std::distance(outer_wall.begin(), it);
        ExtrusionLayer el;
        ExtrusionPaths paths;
        paths.reserve(it->second.size());
        for (auto &polyline : it->second) {
            ExtrusionPath path(ExtrusionRole::erWipeTower, 0.0, 0.0, layer_heights[index]);
            path.polyline = Polyline3(polyline);
            Point3 trans3(trans, 0);
            for (auto &p : path.polyline.points) p += trans3;
            paths.push_back(path);
        }
        el.paths    = std::move(paths);
        el.bottom_z = it->first - layer_heights[index];
        el.layer    = nullptr;
        wtels.push_back(el);
    }
    return wtels;
}
void WipeTowerData::construct_mesh(float width, float depth, float height, float brim_width, bool is_rib_wipe_tower, float rib_width, float rib_length,bool fillet_wall)
{
    wipe_tower_mesh_data = WipeTowerMeshData{};
    float first_layer_height=0.08; //brim height
    if (width < EPSILON || depth < EPSILON || height < EPSILON) return;
    if (!is_rib_wipe_tower || rib_length < EPSILON) {
        wipe_tower_mesh_data->real_wipe_tower_mesh = make_cube(width, depth, height);
        wipe_tower_mesh_data->real_brim_mesh       = make_cube(width + 2 * brim_width, depth + 2 * brim_width, first_layer_height);
        wipe_tower_mesh_data->real_brim_mesh.translate({-brim_width, -brim_width, 0});
        wipe_tower_mesh_data->bottom = {scaled(Vec2f{-brim_width, -brim_width}), scaled(Vec2f{width + brim_width, 0}), scaled(Vec2f{width + brim_width, depth + brim_width}),
                                        scaled(Vec2f{0, depth})};
    } else {
        wipe_tower_mesh_data->real_wipe_tower_mesh = WipeTower::its_make_rib_tower(width, depth, height, rib_length, rib_width, fillet_wall);
        wipe_tower_mesh_data->bottom               = WipeTower::rib_section(width, depth, rib_length, rib_width, fillet_wall);
        auto brim_bottom                           = offset(wipe_tower_mesh_data->bottom, scaled(brim_width));
        if (!brim_bottom.empty())
            wipe_tower_mesh_data->bottom               = brim_bottom.front();
        wipe_tower_mesh_data->real_brim_mesh       = WipeTower::its_make_rib_brim(wipe_tower_mesh_data->bottom, first_layer_height);
        wipe_tower_mesh_data->real_wipe_tower_mesh.translate(Vec3f(rib_offset[0], rib_offset[1],0));
        wipe_tower_mesh_data->real_brim_mesh.translate(Vec3f(rib_offset[0], rib_offset[1], 0));
        wipe_tower_mesh_data->bottom.translate(scaled(Vec2f(rib_offset[0], rib_offset[1])));
    }
    //wipe_tower_mesh_data->real_wipe_tower_mesh.write_ascii("../wipe_tower_mesh.obj");
   //wipe_tower_mesh_data->real_brim_mesh.write_ascii("../wipe_tower_brim_mesh.obj");
}

} // namespace Slic3r
