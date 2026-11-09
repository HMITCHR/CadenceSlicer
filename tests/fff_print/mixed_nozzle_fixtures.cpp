// Shared fixtures for mixed-nozzle end-to-end tests. Declarations and default arguments live in
// mixed_nozzle_fixtures.hpp.
#include "mixed_nozzle_fixtures.hpp"

#include <catch2/catch_all.hpp>

#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/MultiNozzleUtils.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/TriangleSelector.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <numeric>
#include <regex>
#include <sstream>

using namespace Slic3r;

namespace mixed_nozzle_fixtures {

namespace {

// One part of a Body Split object on `filament` at its own layer height (0 leaves it unset).
ModelVolume *add_body(ModelObject *object, TriangleMesh mesh, int filament, double height,
                      const Vec3d &offset = Vec3d::Zero())
{
    ModelVolume *volume = object->add_volume(std::move(mesh), ModelVolumeType::MODEL_PART, false);
    volume->set_offset(offset);
    volume->config.set_key_value("extruder", new ConfigOptionInt(filament));
    if (height > 0.)
        volume->config.set_key_value("regional_layer_height", new ConfigOptionFloat(height));
    return volume;
}

// Places the object once on the bed and applies the print.
void place(Model &model, ModelObject *object, Print &print, const DynamicPrintConfig &config)
{
    object->add_instance();
    object->ensure_on_bed();
    print.apply(model, config);
    print.set_status_silent();
}

} // namespace

DynamicPrintConfig coupon_config(bool enabled)
{
    // Starts from the defaults; only keys that differ from them are set.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    // Print::apply counts filaments from filament_diameter (set_num_filaments() is inert here).
    config.set_key_value("filament_diameter", new ConfigOptionFloats{1.75, 1.75});
    config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(
        enabled ? MixedNozzleSlicingMode::BodySplit : MixedNozzleSlicingMode::Off));
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.4});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.08});
    // Both tools allow 0.20 mm layers; the fine body still prints at 0.10 mm.
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.20, 0.20});
    config.set_key_value("printer_extruder_id", new ConfigOptionInts{1, 2});
    config.set_key_value("printer_extruder_variant", new ConfigOptionStrings{"Direct Drive Standard", "Direct Drive Standard"});
    config.set_key_value("extruder_printable_height", new ConfigOptionFloatsNullable{0., 0.});
    config.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
    config.set_key_value("filament_map", new ConfigOptionInts{1, 2});
    // One variant column per filament, as the presets give them.
    config.set_key_value("filament_self_index", new ConfigOptionInts{1, 2});
    config.set_key_value("filament_extruder_variant", new ConfigOptionStrings{"Direct Drive Standard", "Direct Drive Standard"});
    config.set_key_value("filament_volume_map", new ConfigOptionInts{0, 0});
    config.set_key_value("filament_colour", new ConfigOptionStrings{"#FF0000", "#0000FF"});
    // filament_count^2 entries per nozzle; flush_multiplier has one value per nozzle.
    config.set_key_value("flush_volumes_matrix", new ConfigOptionFloats{0., 0., 0., 0., 0., 0., 0., 0.});
    config.set_key_value("flush_multiplier", new ConfigOptionFloats{1., 1.});
    for (const char *role_filament_id : {"outer_wall_filament_id", "inner_wall_filament_id",
                                         "sparse_infill_filament_id", "internal_solid_filament_id",
                                         "top_surface_filament_id", "bottom_surface_filament_id"})
        config.set_key_value(role_filament_id, new ConfigOptionInt(0));

    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(0.10));
    config.set_key_value("layer_height", new ConfigOptionFloat(0.10));
    config.set_key_value("use_relative_e_distances", new ConfigOptionBool(false));
    config.set_key_value("wall_generator", new ConfigOptionEnum<PerimeterGeneratorType>(PerimeterGeneratorType::Classic));
    config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipGrid));
    config.set_key_value("sparse_infill_density", new ConfigOptionPercent(15.));
    config.set_key_value("top_shell_layers", new ConfigOptionInt(3));
    config.set_key_value("top_shell_thickness", new ConfigOptionFloat(0.));
    config.set_key_value("ensure_vertical_shell_thickness", new ConfigOptionEnum<EnsureVerticalShellThickness>(evstNone));
    // Admission pin: a nonzero threshold re-types small sparse islands to stInternalSolid, which
    // the band preflight refuses. Every synchronized case therefore carries it.
    config.set_key_value("minimum_sparse_infill_area", new ConfigOptionFloat(0.));
    config.set_key_value("detect_overhang_wall", new ConfigOptionBool(false));
    config.set_key_value("thick_internal_bridges", new ConfigOptionBool(false));
    config.set_key_value("flush_into_support", new ConfigOptionBool(false));
    config.set_key_value("filament_adaptive_volumetric_speed", new ConfigOptionBoolsNullable{false, false});
    config.set_key_value("skirt_loops", new ConfigOptionInt(0));
    config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btNoBrim));
    return config;
}

// An inverted inner shell makes a cavity rather than another printable body.
TriangleMesh box_with_internal_cavity(const Vec3d &outer, const Vec3d &cavity_min, const Vec3d &cavity)
{
    TriangleMesh mesh = make_cube(outer.x(), outer.y(), outer.z());
    TriangleMesh hollow = make_cube(cavity.x(), cavity.y(), cavity.z());
    hollow.translate(float(cavity_min.x()), float(cavity_min.y()), float(cavity_min.z()));
    indexed_triangle_set inverted = hollow.its;
    for (stl_triangle_vertex_indices &triangle : inverted.indices)
        std::swap(triangle(1), triangle(2));
    its_merge(mesh.its, inverted);
    return TriangleMesh(mesh.its);
}

void init_coupon(Model &model, Print &print, const DynamicPrintConfig &config, TriangleMesh coarse_mesh)
{
    ModelObject *object = model.add_object();
    object->name = "synchronized-regional-layer-coupon";
    add_body(object, make_cube(10., 20., 4.), 1, 0.10);
    add_body(object, std::move(coarse_mesh), 2, 0.20, Vec3d(10., 0., 0.));
    place(model, object, print, config);
}

void init_feature_flow_fixture(Model &model, Print &print, const DynamicPrintConfig &config, double height,
                               double footprint)
{
    ModelObject *object = model.add_object();
    object->name = "feature-flow-fixture";
    object->add_volume(make_cube(footprint, footprint, height), ModelVolumeType::MODEL_PART, false);
    place(model, object, print, config);
}

// Gives each part the layer height of the nozzle its filament resolves to (the widest nozzle gets
// coarse_cadence, the others fine_cadence) and applies the print again.
void declare_native_cadences(Model &model, Print &print, const DynamicPrintConfig &config,
                             double fine_cadence, double coarse_cadence)
{
    const std::vector<double> &installed = print.config().nozzle_diameter.values;
    REQUIRE(installed.size() >= 2);
    const double widest = *std::max_element(installed.begin(), installed.end());
    for (ModelObject *object : model.objects)
        for (ModelVolume *volume : object->volumes) {
            if (! volume->is_model_part() || ! volume->config.has("extruder"))
                continue;
            const int filament = volume->config.extruder();
            if (filament <= 0)
                continue;
            const std::optional<size_t> nozzle = physical_extruder_for_filament(print.config(), unsigned(filament - 1));
            if (! nozzle)
                continue;
            volume->config.set_key_value("regional_layer_height",
                new ConfigOptionFloat(is_approx(installed[*nozzle], widest) ? coarse_cadence : fine_cadence));
        }
    print.apply(model, config);
    print.set_status_silent();
}

// The standard pair with each body's filament named, and no layer heights set.
void init_mapped_coupon(Model &model, Print &print, const DynamicPrintConfig &config,
                        int fine_filament_1based, int coarse_filament_1based,
                        double height)
{
    ModelObject *object = model.add_object();
    object->name = "resolved-mapping-coupon";
    add_body(object, make_cube(10., 20., height), fine_filament_1based, 0.);
    add_body(object, make_cube(20., 20., height), coarse_filament_1based, 0., Vec3d(10., 0., 0.));
    place(model, object, print, config);
}

// coupon_config(true) with its per-filament vectors widened to `count` filaments.
DynamicPrintConfig coupon_config_with_filaments(size_t count)
{
    DynamicPrintConfig config = coupon_config(true);
    config.set_key_value("filament_diameter", new ConfigOptionFloats(count, 1.75));
    config.set_key_value("filament_self_index", new ConfigOptionInts([count] {
        std::vector<int> ids(count);
        std::iota(ids.begin(), ids.end(), 1);
        return ids;
    }()));
    config.set_key_value("filament_extruder_variant", new ConfigOptionStrings(count, "Direct Drive Standard"));
    config.set_key_value("filament_volume_map", new ConfigOptionInts(count, 0));
    config.set_key_value("filament_colour", new ConfigOptionStrings(count, "#FF0000"));
    config.set_key_value("flush_volumes_matrix", new ConfigOptionFloats(count * count * 2, 0.));
    config.set_key_value("filament_adaptive_volumetric_speed", new ConfigOptionBoolsNullable(count, false));
    return config;
}

// A Bambu printer, so the tower is Type 1.
void init_tower_coupon(Model &model, Print &print, const DynamicPrintConfig &config,
                       TriangleMesh coarse_mesh)
{
    print.is_BBL_printer() = true;
    init_coupon(model, print, config, std::move(coarse_mesh));
}

// coupon_config(true) plus a Type 1 prime tower at (140, 140), 60 mm wide.
static DynamicPrintConfig tower_config()
{
    DynamicPrintConfig config = coupon_config(true);
    config.set_key_value("enable_prime_tower", new ConfigOptionBool(true));
    config.set_key_value("wipe_tower_x", new ConfigOptionFloats{140.});
    config.set_key_value("wipe_tower_y", new ConfigOptionFloats{140.});
    config.set_key_value("purge_in_prime_tower", new ConfigOptionBool(false));
    config.set_key_value("enable_filament_ramming", new ConfigOptionBool(false));
    // The tower requires relative E; the base coupon config runs absolute.
    config.set_key_value("use_relative_e_distances", new ConfigOptionBool(true));
    // Stock per-tool caps, so the volumetric assertion has something real to bind against.
    config.set_key_value("filament_max_volumetric_speed", new ConfigOptionFloatsNullable{1., 4.});
    // Per-filament ramming lengths: a one-element default would turn ramming off for tool 1.
    config.set_key_value("filament_change_length", new ConfigOptionFloats{10., 12.});
    config.set_key_value("filament_change_length_nc", new ConfigOptionFloats{10., 12.});
    config.set_key_value("filament_ramming_travel_time", new ConfigOptionFloatsNullable{1., 1.});
    config.set_key_value("filament_ramming_travel_time_nc", new ConfigOptionFloatsNullable{1., 1.});
    return config;
}

DynamicPrintConfig fine_skin_ratio3_config()
{
    DynamicPrintConfig config = coupon_config(true);
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{2, 3});
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.08});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.30});
    return config;
}

// Fine body and, 20 mm on, a coarse body with fine skins on.
void init_native_skin_coupon(Model &model, Print &print, const DynamicPrintConfig &config,
                             TriangleMesh fine_mesh, TriangleMesh coarse_mesh,
                             double fine_cadence, double coarse_cadence)
{
    ModelObject *object = model.add_object();
    object->name = "native-regional-layer-skin-coupon";
    add_body(object, std::move(fine_mesh), 1, fine_cadence);
    add_body(object, std::move(coarse_mesh), 2, coarse_cadence, Vec3d(20., 0., 0.))
        ->config.set_key_value("mixed_nozzle_body_fine_skins", new ConfigOptionBool(true));
    place(model, object, print, config);
}

DynamicPrintConfig feature_split_tower_config(double fine_d, double coarse_d, bool swapped)
{
    DynamicPrintConfig config = tower_config();
    const std::vector<double> nozzle_diameters = swapped ?
        std::vector<double>{coarse_d, fine_d} : std::vector<double>{fine_d, coarse_d};
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats(nozzle_diameters));
    config.set_key_value("filament_map", new ConfigOptionInts(swapped ?
        std::vector<int>{2, 1} : std::vector<int>{1, 2}));
    config.set_key_value("mixed_nozzle_slicing_mode",
                         new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::FeatureSplit));
    config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(.20));
    // Walls and skins on filament 1, sparse infill on filament 2, whatever each volume's extruder.
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id",
                            "internal_solid_filament_id", "top_surface_filament_id",
                            "bottom_surface_filament_id"})
        config.set_key_value(key, new ConfigOptionInt(1));
    config.set_key_value("sparse_infill_filament_id", new ConfigOptionInt(2));
    // Each nozzle's layer height range is 0.2 to 0.7 of its diameter.
    config.set_key_value("min_layer_height", new ConfigOptionFloats{
        .20 * nozzle_diameters[0], .20 * nozzle_diameters[1]});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{
        .70 * nozzle_diameters[0], .70 * nozzle_diameters[1]});
    // Narrow enough that one purge row stays under the vendor prime at the coarse tower height.
    config.set_key_value("prime_tower_width", new ConfigOptionFloat(40.));
    config.set_key_value("layer_height", new ConfigOptionFloat(.10));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(.10));
    config.set_key_value("prime_volume_mode",
                         new ConfigOptionEnum<PrimeVolumeMode>(PrimeVolumeMode::pvmDefault));
    // No gap wall lead-in moves inside the measured purge.
    config.set_key_value("prime_tower_skip_points", new ConfigOptionBool(false));
    config.set_key_value("mixed_nozzle_filament_provenance", new ConfigOptionStrings{"bound", "bound"});
    config.set_key_value("mixed_nozzle_filament_explicit_keys", new ConfigOptionStrings{"", ""});
    config.set_key_value("filament_prime_volume", new ConfigOptionFloats{45., 30.});
    config.set_key_value("mixed_nozzle_extruder_change_prime_volume",
                         new ConfigOptionFloatsNullable{ConfigOptionFloatsNullable::nil_value(),
                                                         ConfigOptionFloatsNullable::nil_value()});
    config.set_key_value("gcode_comments", new ConfigOptionBool(true));
    return config;
}

DynamicPrintConfig body_split_pad_config(double fine_d, double coarse_d,
                                              double fine_cadence, double coarse_cadence,
                                              bool swapped, bool auto_pad, double width_cap,
                                              WipeTowerWallType wall_type)
{
    DynamicPrintConfig config = feature_split_tower_config(fine_d, coarse_d, swapped);
    config.set_key_value("mixed_nozzle_slicing_mode",
                         new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::BodySplit));
    // Body Split assigns every role through each volume's extruder fallback.
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id",
                           "internal_solid_filament_id", "top_surface_filament_id",
                           "bottom_surface_filament_id", "sparse_infill_filament_id"})
        config.set_key_value(key, new ConfigOptionInt(0));
    config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(coarse_cadence));
    config.set_key_value("layer_height", new ConfigOptionFloat(fine_cadence));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(fine_cadence));
    config.set_key_value("prime_tower_width", new ConfigOptionFloat(width_cap));
    config.set_key_value("wipe_tower_wall_type",
                         new ConfigOptionEnum<WipeTowerWallType>(wall_type));
    config.set_key_value("mixed_nozzle_auto_pad", new ConfigOptionBool(auto_pad));
    return config;
}

void init_body_split_pad_coupon(Model &model, Print &print,
                                    const DynamicPrintConfig &config, bool swapped,
                                    double height, double fine_cadence, double coarse_cadence)
{
    print.is_BBL_printer() = true;
    init_mapped_coupon(model, print, config, 1, 2, height);
    declare_native_cadences(model, print, config, fine_cadence, coarse_cadence);
}

// The H2D change_filament_gcode from the shipped profile (SKIPs if the profile is missing).
static std::string h2d_change_filament_gcode()
{
    const std::filesystem::path path = std::filesystem::path(PROFILES_DIR) / "BBL" / "machine" /
                                       "Bambu Lab H2D 0.4 nozzle.json";
    if (!std::filesystem::exists(path))
        SKIP("shipped profile not present in this checkout: " << path.string());

    DynamicPrintConfig                 profile;
    std::map<std::string, std::string> key_values;
    std::string                        reason;
    profile.load_from_json(path.string(), ForwardCompatibilitySubstitutionRule::Enable, key_values, reason);
    INFO("profile: " << path.string() << (reason.empty() ? "" : ("  load reason: " + reason)));
    if (!profile.has("change_filament_gcode")) {
        // OrcaSlicer 2.5 profiles keep the G-code in a template the machine profile names in its
        // `include` list (the loader layers it under the preset); read it from there.
        const std::filesystem::path included = path.parent_path() / "Bambu Lab H2D 0.4 nozzle template change_filament_gcode.json";
        INFO("included template: " << included.string());
        REQUIRE(std::filesystem::exists(included));
        DynamicPrintConfig template_config;
        key_values.clear();
        reason.clear();
        template_config.load_from_json(included.string(), ForwardCompatibilitySubstitutionRule::Enable, key_values, reason);
        REQUIRE(template_config.has("change_filament_gcode"));
        return template_config.opt_string("change_filament_gcode");
    }
    return profile.opt_string("change_filament_gcode");
}

// Eight filament slots on an H2D-like 0.2/0.6 pair (slot 7 on the 0.6, the rest on the 0.2), Feature Split with
// sparse infill on slot 7.
DynamicPrintConfig feature_economics_baseline_config()
{
    DynamicPrintConfig config = coupon_config_with_filaments(8);

    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.12});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.42});
    // The 0.6 mm nozzle's minimum layer height.
    config.set_key_value("layer_height", new ConfigOptionFloat(0.12));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(0.12));
    config.set_key_value("filament_map", new ConfigOptionInts{1, 1, 1, 1, 1, 1, 2, 1});
    config.set_key_value("filament_type", new ConfigOptionStrings(8, "PLA"));
    config.set_key_value("filament_colour", new ConfigOptionStrings(8, "#FF0000"));
    config.set_key_value("default_filament_colour", new ConfigOptionStrings(8, "#FF0000"));
    config.set_key_value("filament_max_volumetric_speed", new ConfigOptionFloats(8, 30.));
    config.set_key_value("filament_prime_volume", new ConfigOptionFloats(8, 15.));
    config.set_key_value("filament_prime_volume_nc", new ConfigOptionFloats(8, 15.));
    config.set_key_value("filament_start_gcode", new ConfigOptionStrings(8, ""));
    config.set_key_value("filament_end_gcode", new ConfigOptionStrings(8, ""));
    config.set_key_value("filament_change_length", new ConfigOptionFloats(8, 10.));
    config.set_key_value("filament_change_length_nc", new ConfigOptionFloats(8, 12.));
    config.set_key_value("filament_cooling_before_tower", new ConfigOptionFloatsNullable(8, 10.));
    config.set_key_value("filament_flush_volumetric_speed", new ConfigOptionFloatsNullable(8, 0.));
    config.set_key_value("filament_flush_temp", new ConfigOptionIntsNullable(8, 220));
    config.set_key_value("filament_ramming_travel_time", new ConfigOptionFloatsNullable(8, 1.));
    config.set_key_value("filament_ramming_travel_time_nc", new ConfigOptionFloatsNullable(8, 1.));
    config.set_key_value("filament_ramming_volumetric_speed", new ConfigOptionFloatsNullable(8, -1.));
    config.set_key_value("filament_ramming_volumetric_speed_nc", new ConfigOptionFloatsNullable(8, -1.));
    config.set_key_value("filament_pre_cooling_temperature", new ConfigOptionIntsNullable(8, 0));
    config.set_key_value("filament_pre_cooling_temperature_nc", new ConfigOptionIntsNullable(8, 0));
    config.set_key_value("filament_retract_length_nc", new ConfigOptionFloatsNullable(8, 10.));
    config.set_key_value("retraction_distances_when_cut", new ConfigOptionFloats(8, 10.));
    config.set_key_value("retraction_distances_when_ec", new ConfigOptionFloatsNullable(8, 10.));
    config.set_key_value("long_retractions_when_ec", new ConfigOptionBoolsNullable(8, false));
    config.set_key_value("filament_flow_ratio", new ConfigOptionFloatsNullable(8, 1.));
    config.set_key_value("nozzle_temperature", new ConfigOptionInts(8, 220));
    config.set_key_value("nozzle_temperature_initial_layer", new ConfigOptionInts(8, 220));
    config.set_key_value("nozzle_temperature_range_low", new ConfigOptionInts{190, 190});
    config.set_key_value("nozzle_temperature_range_high", new ConfigOptionInts{240, 240});
    config.set_key_value("gcode_comments", new ConfigOptionBool(true));
    config.set_key_value("change_filament_gcode",
                         new ConfigOptionString(h2d_change_filament_gcode()));
    config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::FeatureSplit));
    config.set_key_value("outer_wall_filament_id", new ConfigOptionInt(1));
    config.set_key_value("inner_wall_filament_id", new ConfigOptionInt(1));
    for (const char *key : {"internal_solid_filament_id", "top_surface_filament_id", "bottom_surface_filament_id"})
        config.set_key_value(key, new ConfigOptionInt(1));
    config.set_key_value("sparse_infill_filament_id", new ConfigOptionInt(7));
    config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(.36));
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{2, 3});
    const auto tower = tower_config();
    for (const char *key : {"enable_prime_tower", "prime_tower_width", "wipe_tower_x", "wipe_tower_y",
                           "wipe_tower_filament", "wipe_tower_rotation_angle", "prime_tower_flat_ironing",
                           "enable_tower_interface_features", "ooze_prevention", "purge_in_prime_tower",
                           "single_extruder_multi_material_priming", "enable_filament_ramming", "use_relative_e_distances"})
        config.set_key_value(key, tower.option(key)->clone());
    config.set_key_value("wipe_tower_type", new ConfigOptionEnum<WipeTowerType>(WipeTowerType::Type1));
    config.set_key_value("mixed_nozzle_auto_pad", new ConfigOptionBool(true));
    // Large enough that the switching half of the hand-off cannot be read as rounding.
    config.set_key_value("machine_tool_change_time", new ConfigOptionFloat(10000.));
    config.set_key_value("top_shell_layers", new ConfigOptionInt(0));
    config.set_key_value("top_shell_thickness", new ConfigOptionFloat(0.));
    config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipRectilinear));
    return config;
}

namespace {

// One G-code line with the modal position before and after it, and tower/model classification.
struct TowerExitLine
{
    std::string raw;
    bool   motion   { false };
    bool   has_xy   { false };
    bool   has_z    { false };
    bool   in_tower { false };
    bool   model    { false };
    double x0 { 0. }, y0 { 0. }, z0 { 0. };
    double x  { 0. }, y  { 0. }, z  { 0. };
    double e  { 0. };
    double f  { 0. };
};

std::vector<TowerExitLine> tower_exit_lines(const std::string &gcode)
{
    std::vector<TowerExitLine> out;
    double x = 0., y = 0., z = 0., f = 0.;
    bool   in_tower = false;
    std::string feature;
    std::istringstream stream(gcode);
    std::string raw;
    while (std::getline(stream, raw)) {
        TowerExitLine line;
        line.raw = raw;
        if (raw.rfind("; WIPE_TOWER_START", 0) == 0)
            in_tower = true;
        else if (raw.rfind("; WIPE_TOWER_END", 0) == 0)
            in_tower = false;
        else if (raw.rfind("; FEATURE:", 0) == 0)
            feature = raw.substr(10);
        const std::string code = raw.substr(0, raw.find(';'));
        std::istringstream words(code);
        std::string word;
        words >> word;
        if (word == "G0" || word == "G1" || word == "G2" || word == "G3") {
            line.motion = true;
            line.x0 = x; line.y0 = y; line.z0 = z;
            while (words >> word) {
                if (word.size() < 2)
                    continue;
                const double value = std::stod(word.substr(1));
                switch (word.front()) {
                case 'X': x = value; line.has_xy = true; break;
                case 'Y': y = value; line.has_xy = true; break;
                case 'Z': z = value; line.has_z = true; break;
                case 'E': line.e = value; break;
                case 'F': f = value; break;
                default: break;
                }
            }
            line.x = x; line.y = y; line.z = z;
        }
        line.f        = f;
        line.in_tower = in_tower;
        line.model    = line.motion && line.has_xy && line.e > 0. && ! in_tower && ! feature.empty() &&
                        feature.find("Prime tower") == std::string::npos &&
                        feature.find("Skirt") == std::string::npos &&
                        feature.find("Brim") == std::string::npos;
        out.push_back(std::move(line));
    }
    return out;
}

// True when any part of segment a-b lies strictly inside the box.
bool tower_exit_segment_enters(double ax, double ay, double bx, double by, const BoundingBoxf &box)
{
    if (box.min.x() >= box.max.x() || box.min.y() >= box.max.y())
        return false;
    double t0 = 0., t1 = 1.;
    const double dx = bx - ax, dy = by - ay;
    const double p[4] = { -dx, dx, -dy, dy };
    const double q[4] = { ax - box.min.x(), box.max.x() - ax, ay - box.min.y(), box.max.y() - ay };
    for (int i = 0; i < 4; ++i) {
        if (std::abs(p[i]) < 1e-12) {
            if (q[i] <= 0.)
                return false;
            continue;
        }
        const double t = q[i] / p[i];
        if (p[i] < 0.) t0 = std::max(t0, t);
        else           t1 = std::min(t1, t);
        if (t0 >= t1)
            return false;
    }
    return t1 - t0 > 1e-9;
}

} // namespace

void apply_tower_exit_retraction(DynamicPrintConfig &config)
{
    config.set_key_value("retraction_distances_when_cut", new ConfigOptionFloats{10., 10.});
    config.set_key_value("retraction_length", new ConfigOptionFloats{0.4, 0.4});
    config.set_key_value("wipe_distance", new ConfigOptionFloats{2., 2.});
    config.set_key_value("wipe", new ConfigOptionBools{true, true});
    config.set_key_value("retract_before_wipe", new ConfigOptionPercents{0., 0.});
    config.set_key_value("z_hop", new ConfigOptionFloats{.4, .4});
}

void paint_cadence_facets(ModelVolume &volume, const std::vector<int> &facets, EnforcerBlockerType state)
{
    TriangleSelector selector(volume.mesh());
    for (int facet : facets)
        selector.set_facet(facet, state);
    volume.mmu_segmentation_facets.set(selector);
}

// A fine body on filament 1 at 0.10 mm and a coarse body at 0.20 mm, 2 mm apart so painting and
// body contact stay separate questions. An optional second coarse body sits 2 mm further on.
void init_paint_cadence_coupon(Model &model, Print &print, const DynamicPrintConfig &config,
                               int coarse_extruder,
                               std::optional<EnforcerBlockerType> fine_paint,
                               std::optional<EnforcerBlockerType> coarse_paint,
                               const std::vector<int> &facets,
                               int second_coarse_extruder,
                               bool bbl_printer)
{
    print.is_BBL_printer() = bbl_printer;
    ModelObject *object = model.add_object();
    object->name = "paint-cadence-coupon";
    ModelVolume *fine = add_body(object, make_cube(10., 20., 4.), 1, 0.10);
    if (fine_paint)
        paint_cadence_facets(*fine, facets, *fine_paint);
    ModelVolume *coarse = add_body(object, make_cube(10., 20., 4.), coarse_extruder, 0.20, Vec3d(12., 0., 0.));
    if (coarse_paint)
        paint_cadence_facets(*coarse, facets, *coarse_paint);
    if (second_coarse_extruder > 0)
        add_body(object, make_cube(10., 20., 4.), second_coarse_extruder, 0.20, Vec3d(24., 0., 0.));
    place(model, object, print, config);
}

// The shared tower fixture widened to three filaments: filament 1 on the 0.2 mm nozzle, filaments
// 2 and 3 both on the 0.4 mm nozzle.
DynamicPrintConfig paint_cadence_tower_config()
{
    DynamicPrintConfig config = tower_config();
    config.set_key_value("filament_map", new ConfigOptionInts{1, 2, 2});
    config.set_key_value("filament_diameter", new ConfigOptionFloats{1.75, 1.75, 1.75});
    config.set_key_value("filament_self_index", new ConfigOptionInts{1, 2, 3});
    config.set_key_value("filament_extruder_variant", new ConfigOptionStrings(3, "Direct Drive Standard"));
    config.set_key_value("filament_volume_map", new ConfigOptionInts{0, 0, 0});
    config.set_key_value("filament_colour", new ConfigOptionStrings{"#FF0000", "#0000FF", "#00FF00"});
    config.set_key_value("flush_volumes_matrix", new ConfigOptionFloats(std::vector<double>(3 * 3 * 2, 0.)));
    config.set_key_value("filament_adaptive_volumetric_speed", new ConfigOptionBoolsNullable{false, false, false});
    config.set_key_value("filament_max_volumetric_speed", new ConfigOptionFloatsNullable{1., 4., 4.});
    config.set_key_value("filament_change_length", new ConfigOptionFloats{10., 12., 12.});
    config.set_key_value("filament_change_length_nc", new ConfigOptionFloats{10., 12., 12.});
    config.set_key_value("filament_ramming_travel_time", new ConfigOptionFloatsNullable{1., 1., 1.});
    config.set_key_value("filament_ramming_travel_time_nc", new ConfigOptionFloatsNullable{1., 1., 1.});
    return config;
}

// The economics baseline setup with free switches, so every second a plan pays for beyond its
// infill is tower time, and the arithmetic below needs no switching term.
DynamicPrintConfig free_switch_economics_config()
{
    DynamicPrintConfig config = feature_economics_baseline_config();
    for (const char *key : {"machine_tool_change_time", "machine_load_filament_time", "machine_unload_filament_time"})
        config.set_key_value(key, new ConfigOptionFloat(0.));
    return config;
}

TowerExitAudit audit_tower_exit(const std::string &gcode, double expected_pull)
{
    const std::vector<TowerExitLine> lines = tower_exit_lines(gcode);
    TowerExitAudit audit;

    BoundingBoxf model_box;
    for (const TowerExitLine &line : lines)
        if (line.model) {
            model_box.merge(Vec2d(line.x0, line.y0));
            model_box.merge(Vec2d(line.x, line.y));
        }
    REQUIRE(model_box.defined);
    BoundingBoxf model_core = model_box;
    model_core.min += Vec2d(1., 1.);
    model_core.max -= Vec2d(1., 1.);

    for (size_t r = 0; r < lines.size(); ++r) {
        if (lines[r].raw != "; FINE_TOWER_RETURN_RELIEF")
            continue;
        ++audit.returns;
        // The last road the fine nozzle laid on the tower before it leaves.
        size_t last = r;
        while (last > 0 && ! (lines[last].in_tower && lines[last].motion && lines[last].has_xy && lines[last].e > 0.))
            --last;
        REQUIRE(last > 0);
        // The purge rows of this visit.
        size_t purge = last;
        while (purge > 0 && lines[purge].raw.rfind("; CP_TOOLCHANGE_WIPE", 0) != 0)
            --purge;
        REQUIRE(purge > 0);
        BoundingBoxf rows;
        for (size_t i = purge; i < lines.size() && lines[i].raw.rfind("; WIPE_TOWER_END", 0) != 0; ++i)
            if (lines[i].motion && lines[i].has_xy && lines[i].e > 0.) {
                rows.merge(Vec2d(lines[i].x0, lines[i].y0));
                rows.merge(Vec2d(lines[i].x, lines[i].y));
            }
        REQUIRE(rows.defined);
        rows.min -= Vec2d(.3, .3);
        rows.max += Vec2d(.3, .3);

        double pulled = 0.;
        size_t wipes  = 0;
        for (size_t i = last + 1; i < r; ++i) {
            const TowerExitLine &line = lines[i];
            if (! line.motion)
                continue;
            if (line.has_z && std::abs(line.z - lines[last].z) > 1e-6)
                ++audit.lifted_before_pull;
            if (line.has_xy) {
                if (! rows.contains(Vec2d(line.x, line.y)))
                    ++audit.moves_off_rows;
                if (line.e < 0.)
                    ++wipes;
            }
            if (line.e < 0.)
                pulled -= line.e;
        }
        if (wipes == 0)
            ++audit.returns_without_wipe;

        size_t i = r + 1;
        while (i < lines.size() && ! lines[i].motion)
            ++i;
        REQUIRE(i < lines.size());
        pulled -= std::min(0., lines[i].e);
        if (std::abs(pulled - expected_pull) > 1e-4)
            ++audit.unbalanced;
        const double z_at_pull = lines[i].z;
        const double x_at_pull = lines[i].x, y_at_pull = lines[i].y;
        ++i;
        // A line that only sets the feedrate ("G1 F7200 ; spiral lift Z") moves nothing.
        while (i < lines.size() && (! lines[i].motion || (! lines[i].has_xy && ! lines[i].has_z && lines[i].e == 0.)))
            ++i;
        REQUIRE(i < lines.size());
        // The lift is the printer's own z-hop type. A spiral lift (the default z_hop_types)
        // climbs on a small circle around the point the pull ended on, so it carries X and Y; it is
        // still a lift on the tower and not a travel. Its first move has to climb and stay within
        // a couple of millimetres of the pull.
        const bool lifts_in_place = lines[i].has_z && lines[i].z > z_at_pull + 1e-6 &&
            (! lines[i].has_xy || std::hypot(lines[i].x - x_at_pull, lines[i].y - y_at_pull) <= 2.);
        if (! lifts_in_place)
            ++audit.not_lifted_next;

        // Tower to part. The final approach onto the first bead is allowed to reach the part.
        for (; i < lines.size() && ! lines[i].model; ++i) {
            const TowerExitLine &line = lines[i];
            if (! line.motion || ! line.has_xy)
                continue;
            size_t next = i + 1;
            while (next < lines.size() && ! lines[next].model && ! (lines[next].motion && lines[next].has_xy))
                ++next;
            double bx = line.x, by = line.y;
            if (next < lines.size() && lines[next].model) {
                const double len = std::hypot(line.x - line.x0, line.y - line.y0);
                if (len <= 2.)
                    continue;
                bx = line.x - (line.x - line.x0) * 2. / len;
                by = line.y - (line.y - line.y0) * 2. / len;
            }
            if (tower_exit_segment_enters(line.x0, line.y0, bx, by, model_core))
                ++audit.crossings;
        }
    }
    return audit;
}

TowerGridSpeedAudit audit_tower_grid_speed(const std::string &gcode)
{
    TowerGridSpeedAudit audit;
    bool   in_grid = false;
    double first_feed = 0.;
    for (const TowerExitLine &line : tower_exit_lines(gcode)) {
        if (line.raw.rfind("; CP EMPTY GRID START", 0) == 0) {
            in_grid = true;
            first_feed = 0.;
            continue;
        }
        if (line.raw.rfind("; CP EMPTY GRID END", 0) == 0) {
            in_grid = false;
            continue;
        }
        if (! in_grid || ! line.motion || ! line.has_xy || line.e <= 0.)
            continue;
        ++audit.grid_roads;
        if (first_feed <= 0.) {
            first_feed = line.f;
            continue;
        }
        if (line.f > first_feed + 1.)
            ++audit.faster_roads;
    }
    return audit;
}

TowerFlowAudit audit_coarse_tower_flow(const std::string &gcode, const PrintConfig &resolved)
{
    TowerFlowAudit audit;
    const double filament_area    = M_PI * 1.75 * 1.75 / 4.;
    const double first_layer_top  = resolved.initial_layer_print_height.value + 1e-3;
    auto nozzle_of = [&resolved](int tool) -> double {
        if (tool < 0 || size_t(tool) >= resolved.filament_map.values.size())
            return 0.;
        const int nozzle = resolved.filament_map.values[size_t(tool)] - 1;
        if (nozzle < 0 || size_t(nozzle) >= resolved.nozzle_diameter.values.size())
            return 0.;
        return resolved.nozzle_diameter.values[size_t(nozzle)];
    };
    const double fine_d = *std::min_element(resolved.nozzle_diameter.values.begin(), resolved.nozzle_diameter.values.end());

    int    active_tool = -1;
    bool   in_tower = false, in_grid = false, in_purge = false, in_nozzle_change = false;
    double layer_height = 0., x = 0., y = 0., z = 0.;
    // A sparse grid is lines with a travel between them. The solid block fill and the solid
    // first-layer grid share the markers but lay touching lines at the shared pitch, which is a
    // dense layer and not what this audit is about, so a section counts only once it has travelled.
    bool   grid_travelled = false;
    std::vector<double> grid_ratios;
    std::istringstream stream(gcode);
    std::string raw;
    while (std::getline(stream, raw)) {
        if (raw.rfind("; NOZZLE_CHANGE_START", 0) == 0) {
            in_nozzle_change = true;
            const size_t at = raw.find(" NF");
            if (at != std::string::npos)
                active_tool = std::stoi(raw.substr(at + 3));
            continue;
        }
        if (raw.rfind("; NOZZLE_CHANGE_END", 0) == 0) { in_nozzle_change = false; continue; }
        if (raw.rfind("M1020 S", 0) == 0) { active_tool = std::stoi(raw.substr(7)); continue; }
        if (raw.rfind("; WIPE_TOWER_START", 0) == 0) { in_tower = true; continue; }
        if (raw.rfind("; WIPE_TOWER_END", 0) == 0) { in_tower = false; in_purge = false; continue; }
        if (raw.rfind("; CP_TOOLCHANGE_WIPE", 0) == 0) { in_purge = true; continue; }
        if (raw.rfind("; CP EMPTY GRID START", 0) == 0) {
            in_grid = true;
            grid_travelled = false;
            grid_ratios.clear();
            continue;
        }
        if (raw.rfind("; CP EMPTY GRID END", 0) == 0) {
            in_grid = false;
            if (grid_travelled)
                for (double ratio : grid_ratios) {
                    ++audit.coarse_grid_roads;
                    audit.thinnest_ratio = std::min(audit.thinnest_ratio, ratio);
                    if (ratio < 0.97) ++audit.thin_grid;
                }
            continue;
        }
        if (raw.rfind("; LAYER_HEIGHT:", 0) == 0) { layer_height = std::stod(raw.substr(15)); continue; }
        const std::string code = raw.substr(0, raw.find(';'));
        if (code.size() > 1 && code.front() == 'T' && std::isdigit(static_cast<unsigned char>(code[1]))) {
            active_tool = std::stoi(code.substr(1));
            continue;
        }
        std::istringstream words(code);
        std::string word;
        words >> word;
        if (word != "G0" && word != "G1" && word != "G2" && word != "G3")
            continue;
        const bool arc = word == "G2" || word == "G3";
        const bool ccw = word == "G3";
        const double x0 = x, y0 = y;
        double e = 0., i = 0., j = 0.;
        bool   xy = false;
        while (words >> word) {
            if (word.size() < 2)
                continue;
            const double value = std::stod(word.substr(1));
            switch (word.front()) {
            case 'X': x = value; xy = true; break;
            case 'Y': y = value; xy = true; break;
            case 'Z': z = value; break;
            case 'E': e = value; break;
            case 'I': i = value; break;
            case 'J': j = value; break;
            default: break;
            }
        }
        if (in_grid && xy && e == 0. && ! grid_ratios.empty())
            grid_travelled = true;
        if (! in_tower || in_purge || in_nozzle_change || ! xy || e <= 0. || z <= first_layer_top)
            continue;
        // The brim chamfer is at most 3 mm of loops outside the wall on the lowest levels.
        const bool wall = arc && ! in_grid && z > first_layer_top + 4.;
        if (! in_grid && ! wall)
            continue;
        const double d = nozzle_of(active_tool);
        if (d <= fine_d + 1e-6 || layer_height <= 0.)
            continue;
        double length = std::hypot(x - x0, y - y0);
        if (arc) {
            const double cx = x0 + i, cy = y0 + j;
            const double r  = std::hypot(i, j);
            double sweep = std::atan2(y - cy, x - cx) - std::atan2(y0 - cy, x0 - cx);
            if (ccw) { while (sweep <= 0.) sweep += 2. * M_PI; }
            else     { while (sweep >= 0.) sweep -= 2. * M_PI; }
            length = r * std::abs(sweep);
        }
        if (length < 0.2)
            continue;
        const double mm3_per_mm = e * filament_area / length;
        const double solid      = layer_height * (1.25 * d - layer_height * (1. - M_PI / 4.));
        const double ratio      = mm3_per_mm / solid;
        if (in_grid) {
            grid_ratios.push_back(ratio);
        } else {
            audit.thinnest_ratio = std::min(audit.thinnest_ratio, ratio);
            ++audit.coarse_wall_arcs;
            if (ratio < 0.97) ++audit.thin_wall;
        }
    }
    return audit;
}

TowerFootprintAudit audit_tower_footprint(const std::string &gcode, const Print &print)
{
    TowerFootprintAudit audit;
    const WipeTowerData &tower = print.wipe_tower_data();
    BoundingBoxf planned = tower.bbx;
    REQUIRE(planned.defined);
    // The planned box is the first layer's outer loop in tower coordinates; place it on the plate
    // with half a road width of clearance.
    planned.translate(Vec2d(print.config().wipe_tower_x.get_at(0), print.config().wipe_tower_y.get_at(0)) +
                      tower.rib_offset.cast<double>());
    planned.min -= Vec2d(.5, .5);
    planned.max += Vec2d(.5, .5);
    for (const TowerExitLine &line : tower_exit_lines(gcode))
        if (line.in_tower && line.motion && line.has_xy && line.e > 0.) {
            ++audit.tower_roads;
            if (! planned.contains(Vec2d(line.x, line.y)))
                ++audit.outside;
        }
    return audit;
}

void fill_per_filament_values(DynamicPrintConfig &config)
{
    const size_t filaments = config.option<ConfigOptionFloats>("filament_diameter")->values.size();
    for (const std::string &key : Preset::filament_options()) {
        ConfigOption *option = config.option(key);
        if (option == nullptr || ! option->is_vector())
            continue;
        auto *vector = static_cast<ConfigOptionVectorBase *>(option);
        if (vector->size() >= 1 && vector->size() < filaments)
            vector->resize(filaments);
    }
}

// ---- Regional-layer prism and support fixtures ------------------------------------------------

TriangleMesh extruded_xz_outline(const std::vector<Vec2d> &outline, double depth)
{
    const size_t n = outline.size();
    indexed_triangle_set its;
    its.vertices.reserve(2 * n);
    for (const Vec2d &corner : outline) {
        its.vertices.emplace_back(stl_vertex(float(corner.x()), 0.f, float(corner.y())));
        its.vertices.emplace_back(stl_vertex(float(corner.x()), float(depth), float(corner.y())));
    }

    its.indices.reserve(2 * n + 2 * (n - 2));
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        its.indices.emplace_back(stl_triangle_vertex_indices(int(2 * i), int(2 * j + 1), int(2 * j)));
        its.indices.emplace_back(stl_triangle_vertex_indices(int(2 * i), int(2 * i + 1), int(2 * j + 1)));
    }
    // Fans from outline corner 0, reversed on the far cap so the two caps face opposite ways.
    for (size_t i = 1; i + 1 < n; ++i) {
        its.indices.emplace_back(stl_triangle_vertex_indices(0, int(2 * i), int(2 * (i + 1))));
        its.indices.emplace_back(stl_triangle_vertex_indices(1, int(2 * (i + 1) + 1), int(2 * i + 1)));
    }

    return TriangleMesh(its);
}

TriangleMesh overhang_shelf_mesh(double base_width, double shelf_width, double base_height, double total_height)
{
    return extruded_xz_outline(
        {{0., 0.}, {base_width, 0.}, {base_width, base_height}, {shelf_width, base_height},
         {shelf_width, total_height}, {0., total_height}}, 20.);
}

void init_native_coupon(Model &model, Print &print, const DynamicPrintConfig &config,
                        TriangleMesh fine_mesh, TriangleMesh coarse_mesh,
                        double fine_cadence, double coarse_cadence)
{
    ModelObject *object = model.add_object();
    object->name = "native-regional-layer-coupon";
    add_body(object, std::move(fine_mesh), 1, fine_cadence);
    add_body(object, std::move(coarse_mesh), 2, coarse_cadence, Vec3d(20., 0., 0.));
    place(model, object, print, config);
}

void init_feature_overhang_fixture(Model &model, Print &print, const DynamicPrintConfig &config)
{
    ModelObject *object = model.add_object();
    object->name = "feature-overhang-fixture";
    // The upper shelf projects 8 mm beyond the lower support, giving the perimeter detector a
    // generous, unambiguous overhang while keeping one watertight model volume.
    object->add_volume(overhang_shelf_mesh(12., 20., 2., 4.), ModelVolumeType::MODEL_PART, false);
    place(model, object, print, config);
}

std::vector<ArrivingPurge> scan_arriving_purges(const std::string &gcode,
                                                     const std::vector<double> &filament_diameters)
{
    std::vector<ArrivingPurge> result;
    // One entry per physical nozzle; a tool changer has more than two.
    std::array<int, 16> held_filament;
    held_filament.fill(-1);
    std::optional<ArrivingPurge> pending;
    bool   in_arriving_purge = false;
    double filament_length   = 0.;
    double emitted_height = 0.;
    size_t positive_moves    = 0;
    const std::regex start_marker(
        R"(^;\s*NOZZLE_CHANGE_START OF(-?\d+) NF(-?\d+) ON(-?\d+) NN(-?\d+))");

    double current_z = 0.;
    std::istringstream stream(gcode);
    std::string raw;
    while (std::getline(stream, raw)) {
        if (raw.rfind("; Z_HEIGHT:", 0) == 0)
            current_z = std::stod(raw.substr(11));
        if (raw.rfind("; LAYER_HEIGHT:", 0) == 0)
            emitted_height = std::stod(raw.substr(15));
        std::smatch marker;
        if (std::regex_search(raw, marker, start_marker)) {
            // The first marker tells which filament each nozzle starts with.
            const int old_filament = std::stoi(marker[1].str());
            const int new_filament = std::stoi(marker[2].str());
            const int old_nozzle   = std::stoi(marker[3].str());
            const int new_nozzle   = std::stoi(marker[4].str());
            if (old_nozzle >= 0 && old_nozzle < int(held_filament.size()) && held_filament[old_nozzle] < 0)
                held_filament[old_nozzle] = old_filament;
            const bool warm_return = new_nozzle >= 0 && new_nozzle < int(held_filament.size()) &&
                                     held_filament[new_nozzle] == new_filament;
            pending = ArrivingPurge{new_filament, new_nozzle, warm_return};
            pending->print_z = current_z;
            if (new_nozzle >= 0 && new_nozzle < int(held_filament.size()))
                held_filament[new_nozzle] = new_filament;
            in_arriving_purge = false;
            filament_length = 0.;
            positive_moves = 0;
            continue;
        }

        if (raw.rfind("; CP_TOOLCHANGE_WIPE", 0) == 0) {
            if (pending) {
                in_arriving_purge = true;
                filament_length = 0.;
                positive_moves = 0;
            }
            continue;
        }

        // The purge ends at the first grid, tower end or tool change end.
        const bool purge_boundary = raw.rfind("; CP EMPTY GRID START", 0) == 0 ||
                                     raw.rfind("; WIPE_TOWER_END", 0) == 0 ||
                                     raw.rfind("; CP TOOLCHANGE END", 0) == 0;
        if (in_arriving_purge && purge_boundary) {
            ArrivingPurge section = *pending;
            const int filament = section.new_filament;
            if (filament >= 0 && filament < int(filament_diameters.size())) {
                const double area = M_PI * filament_diameters[size_t(filament)] * filament_diameters[size_t(filament)] / 4.;
                section.volume_mm3 = filament_length * area;
            }
            section.positive_moves = positive_moves;
            result.emplace_back(section);
            pending.reset();
            in_arriving_purge = false;
            filament_length = 0.;
            positive_moves = 0;
            continue;
        }

        if (!in_arriving_purge)
            continue;

        const size_t comment = raw.find(';');
        const std::string code = raw.substr(0, comment);
        std::istringstream words(code);
        std::string command;
        words >> command;
        if (command != "G1")
            continue;
        std::string word;
        while (words >> word) {
            if (word.size() < 2 || word.front() != 'E')
                continue;
            double e = 0.;
            try {
                e = std::stod(word.substr(1));
            } catch (...) {
                continue;
            }
            if (e > 0.) {
                pending->max_emitted_height = std::max(pending->max_emitted_height, emitted_height);
                filament_length += e;
                ++positive_moves;
            }
        }
    }
    return result;
}

double prime_row_allowance_mm3(const PrintConfig &config, int physical_nozzle, double layer_height)
{
    const double d = config.nozzle_diameter.get_at(size_t(physical_nozzle));
    const double min_d = *std::min_element(config.nozzle_diameter.values.begin(), config.nozzle_diameter.values.end());
    // One purge row across the tower (less its two wall lines), with the oval cross-section the tower uses.
    const double h = layer_height;
    const double purge_width = 1.25 * d;
    const double row_cross_section = h * (purge_width - h * (1. - M_PI / 4.));
    const double row_length = config.prime_tower_width.value - 2. * (1.25 * min_d);
    return std::max(0., row_cross_section * row_length);
}

WarmReturns check_warm_returns(const std::string &gcode, const PrintConfig &resolved, double tower_width, bool upper_bound)
{
    constexpr double rounding_mm3 = .02;
    WarmReturns out;
    for (const ArrivingPurge &purge : scan_arriving_purges(gcode, resolved.filament_diameter.values)) {
        if (!purge.warm_return)
            continue;
        ++out.count;
        REQUIRE(purge.new_filament >= 0);
        REQUIRE(purge.new_nozzle >= 0);
        const auto floor = mixed_nozzle_handoff_deposits(resolved, size_t(purge.new_filament), size_t(purge.new_nozzle), tower_width);
        REQUIRE(floor.has_value());
        const double vendor = double(mixed_nozzle_vendor_prime_volumes(resolved, size_t(purge.new_filament)).extruder_change);
        CAPTURE(purge.print_z, purge.new_filament, purge.new_nozzle, purge.volume_mm3, floor->prime_volume_mm3, vendor);
        CHECK(purge.volume_mm3 + rounding_mm3 >= floor->prime_volume_mm3);
        if (upper_bound) {
            // The tower lays whole purge rows, so a prime can run up to one row past its volume.
            const double row = prime_row_allowance_mm3(resolved, purge.new_nozzle,
                                                       resolved.max_layer_height.get_at(size_t(purge.new_nozzle)));
            CHECK(purge.volume_mm3 <= std::max(vendor, double(floor->prime_volume_mm3)) + row + rounding_mm3);
        }
        out.below_vendor += purge.volume_mm3 + 1. < vendor ? 1 : 0;
    }
    return out;
}

std::vector<size_t> bracketed_lines(const std::string &gcode, const std::vector<std::string> &starts,
                                    const std::vector<std::string> &ends)
{
    const auto starts_with_any = [](const std::string &line, const std::vector<std::string> &prefixes) {
        return std::any_of(prefixes.begin(), prefixes.end(), [&line](const std::string &prefix) { return line.rfind(prefix, 0) == 0; });
    };
    std::vector<size_t> out(1, 0);
    std::istringstream lines(gcode);
    std::string line;
    size_t count = 0, current = 0;
    while (std::getline(lines, line)) {
        if (current == 0 && starts_with_any(line, starts))
            current = ++count;
        out.push_back(current);
        if (current != 0 && starts_with_any(line, ends))
            current = 0;
    }
    return out;
}

DynamicPrintConfig coarse_base_support_config(MixedNozzleSlicingMode mode)
{
    DynamicPrintConfig config = tower_config();
    config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(mode));
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{3, 4});
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.12});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.40});
    config.set_key_value("filament_map", new ConfigOptionInts{1, 2});
    config.set_key_value("filament_flow_ratio", new ConfigOptionFloats{1., 1.});
    // The raft is laid by the support
    // filament, so this print can start on the second filament and the BBL export reads the
    // starting filament's own gcode column by hard index.
    config.set_key_value("filament_start_gcode", new ConfigOptionStrings(2, ""));
    config.set_key_value("filament_end_gcode", new ConfigOptionStrings(2, ""));
    config.set_key_value("layer_height", new ConfigOptionFloat(0.10));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(0.10));
    config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(0.40));
    config.set_key_value("wipe_tower_no_sparse_layers", new ConfigOptionBool(true));
    config.set_key_value("gcode_comments", new ConfigOptionBool(true));
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id",
                            "internal_solid_filament_id", "top_surface_filament_id",
                            "bottom_surface_filament_id"})
        config.set_key_value(key, new ConfigOptionInt(1));
    config.set_key_value("sparse_infill_filament_id", new ConfigOptionInt(2));
    config.set_key_value("enable_support", new ConfigOptionBool(true));
    config.set_key_value("support_filament", new ConfigOptionInt(2));            // -> physical 1, .6 mm
    config.set_key_value("support_interface_filament", new ConfigOptionInt(1));  // -> physical 0, .2 mm
    config.set_key_value("independent_support_layer_height", new ConfigOptionBool(false));
    return config;
}

} // namespace mixed_nozzle_fixtures
