// Off against the emitted stream, the tower plan, and stock OrcaSlicer G-code (stock_off_scenes.hpp).

#include <catch2/catch_all.hpp>
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCode/WipeTower.hpp"
#include "libslic3r/MultiNozzleUtils.hpp"
#include "libslic3r/GCodeWriter.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/MixedNozzleConfig.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Slicing.hpp"
#include "test_helpers.hpp"
#include "mixed_nozzle_facts.hpp"
#include "mixed_nozzle_harness.hpp"
#include "stock_off_scenes.hpp"
#include <miniz.h>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;

namespace {
DynamicPrintConfig off_parity_base_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_key_value("mixed_nozzle_slicing_mode",
                         new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::Off));
    config.set_key_value("filament_diameter", new ConfigOptionFloats{1.75, 1.75});
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.4, 0.4});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.08, 0.08});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.20, 0.20});
    config.set_key_value("printer_extruder_id", new ConfigOptionInts{1, 2});
    config.set_key_value("extruder_printable_height", new ConfigOptionFloatsNullable{0., 0.});
    config.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
    config.set_key_value("filament_map", new ConfigOptionInts{1, 2});
    config.set_key_value("filament_self_index", new ConfigOptionInts{1, 2});
    config.set_key_value("filament_extruder_variant", new ConfigOptionStrings{"Direct Drive Standard", "Direct Drive Standard"});
    config.set_key_value("printer_extruder_variant", new ConfigOptionStrings{"Direct Drive Standard", "Direct Drive Standard"});
    config.set_key_value("filament_volume_map", new ConfigOptionInts{0, 0});
    config.set_key_value("enable_filament_dynamic_map", new ConfigOptionBool(false));
    config.set_key_value("filament_colour", new ConfigOptionStrings{"#FF0000", "#0000FF"});
    config.set_key_value("flush_volumes_matrix", new ConfigOptionFloats{0., 140., 140., 0., 0., 140., 140., 0.});
    config.set_key_value("flush_multiplier", new ConfigOptionFloats{1., 1.});
    for (const char *role_filament_id : {"outer_wall_filament_id", "inner_wall_filament_id",
                                         "sparse_infill_filament_id", "internal_solid_filament_id",
                                         "top_surface_filament_id", "bottom_surface_filament_id"})
        config.set_key_value(role_filament_id, new ConfigOptionInt(0));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(0.20));
    config.set_key_value("layer_height", new ConfigOptionFloat(0.20));
    config.set_key_value("precise_z_height", new ConfigOptionBool(false));
    config.set_key_value("slicing_mode", new ConfigOptionEnum<SlicingMode>(SlicingMode::Regular));
    config.set_key_value("print_sequence", new ConfigOptionEnum<PrintSequence>(PrintSequence::ByLayer));
    config.set_key_value("wall_generator", new ConfigOptionEnum<PerimeterGeneratorType>(PerimeterGeneratorType::Classic));
    config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipGrid));
    config.set_key_value("sparse_infill_density", new ConfigOptionPercent(15.));
    config.set_key_value("top_shell_layers", new ConfigOptionInt(2));
    config.set_key_value("bottom_shell_layers", new ConfigOptionInt(2));
    config.set_key_value("top_shell_thickness", new ConfigOptionFloat(0.));
    config.set_key_value("bottom_shell_thickness", new ConfigOptionFloat(0.));
    config.set_key_value("ensure_vertical_shell_thickness", new ConfigOptionEnum<EnsureVerticalShellThickness>(evstNone));
    config.set_key_value("interface_shells", new ConfigOptionBool(false));
    config.set_key_value("infill_combination", new ConfigOptionBool(false));
    config.set_key_value("extra_solid_infills", new ConfigOptionString(""));
    config.set_key_value("fill_multiline", new ConfigOptionInt(1));
    config.set_key_value("minimum_sparse_infill_area", new ConfigOptionFloat(0.));
    config.set_key_value("detect_thin_wall", new ConfigOptionBool(false));
    config.set_key_value("detect_overhang_wall", new ConfigOptionBool(false));
    config.set_key_value("gap_fill_target", new ConfigOptionEnum<GapFillTarget>(gftNowhere));
    config.set_key_value("alternate_extra_wall", new ConfigOptionBool(false));
    config.set_key_value("extra_perimeters_on_overhangs", new ConfigOptionBool(false));
    config.set_key_value("fuzzy_skin", new ConfigOptionEnum<FuzzySkinType>(FuzzySkinType::Disabled_fuzzy));
    config.set_key_value("seam_slope_type", new ConfigOptionEnum<SeamScarfType>(SeamScarfType::None));
    config.set_key_value("ironing_type", new ConfigOptionEnum<IroningType>(IroningType::NoIroning));
    config.set_key_value("enable_extra_bridge_layer", new ConfigOptionEnum<EnableExtraBridgeLayer>(eblDisabled));
    config.set_key_value("counterbore_hole_bridging", new ConfigOptionEnum<CounterboreHoleBridgingOption>(chbNone));
    config.set_key_value("thick_bridges", new ConfigOptionBool(false));
    config.set_key_value("thick_internal_bridges", new ConfigOptionBool(false));
    config.set_key_value("zaa_enabled", new ConfigOptionBool(false));
    config.set_key_value("interlocking_beam", new ConfigOptionBool(false));
    config.set_key_value("spiral_mode", new ConfigOptionBool(false));
    config.set_key_value("enable_support", new ConfigOptionBool(false));
    config.set_key_value("enforce_support_layers", new ConfigOptionInt(0));
    config.set_key_value("raft_layers", new ConfigOptionInt(0));
    config.set_key_value("enable_prime_tower", new ConfigOptionBool(false));
    config.set_key_value("flush_into_infill", new ConfigOptionBool(false));
    config.set_key_value("flush_into_objects", new ConfigOptionBool(false));
    config.set_key_value("flush_into_support", new ConfigOptionBool(false));
    config.set_key_value("resonance_avoidance", new ConfigOptionBool(false));
    config.set_key_value("filament_adaptive_volumetric_speed", new ConfigOptionBoolsNullable{false, false});
    config.set_key_value("skirt_loops", new ConfigOptionInt(0));
    config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btNoBrim));
    config.set_key_value("use_relative_e_distances", new ConfigOptionBool(false));
    const auto &fp_defaults = FullPrintConfig::defaults();
    for (const char *key : {"filament_type", "filament_vendor", "filament_start_gcode"})
        static_cast<ConfigOptionVectorBase *>(config.option(key, true))->resize(2, fp_defaults.option(key));
    mixed_nozzle_fixtures::fill_per_filament_values(config);
    return config;
}
DynamicPrintConfig off_parity_tower_config()
{
    DynamicPrintConfig config = off_parity_base_config();
    config.set_key_value("enable_prime_tower", new ConfigOptionBool(true));
    config.set_key_value("prime_tower_width", new ConfigOptionFloat(60.));
    config.set_key_value("wipe_tower_x", new ConfigOptionFloats{140.});
    config.set_key_value("wipe_tower_y", new ConfigOptionFloats{140.});
    config.set_key_value("wipe_tower_filament", new ConfigOptionInt(0));
    config.set_key_value("wipe_tower_rotation_angle", new ConfigOptionFloat(0.));
    config.set_key_value("wipe_tower_wall_type", new ConfigOptionEnum<WipeTowerWallType>(WipeTowerWallType::wtwRib));
    config.set_key_value("prime_tower_flat_ironing", new ConfigOptionBool(false));
    config.set_key_value("enable_tower_interface_features", new ConfigOptionBool(false));
    config.set_key_value("ooze_prevention", new ConfigOptionBool(false));
    config.set_key_value("purge_in_prime_tower", new ConfigOptionBool(false));
    config.set_key_value("single_extruder_multi_material_priming", new ConfigOptionBool(false));
    config.set_key_value("enable_filament_ramming", new ConfigOptionBool(false));
    config.set_key_value("use_relative_e_distances", new ConfigOptionBool(true));
    config.set_key_value("filament_max_volumetric_speed", new ConfigOptionFloatsNullable{4., 4.});
    config.set_key_value("filament_change_length", new ConfigOptionFloats{10., 10.});
    config.set_key_value("filament_change_length_nc", new ConfigOptionFloats{10., 10.});
    config.set_key_value("filament_ramming_travel_time", new ConfigOptionFloatsNullable{1., 1.});
    config.set_key_value("filament_ramming_travel_time_nc", new ConfigOptionFloatsNullable{1., 1.});
    return config;
}
// full_print_config() sizes these enum lists without their name table, so they write to G-code as
// "," and cannot be read back. Give both entries the definition default with names.
void name_enum_lists(DynamicPrintConfig &config)
{
    for (const char *key : {"extruder_type", "nozzle_type", "retract_lift_enforce", "z_hop_types", "nozzle_volume_type",
                            "overhang_fan_threshold"}) {
        const ConfigOptionDef *def = print_config_def.get(key);
        const int value = static_cast<const ConfigOptionInts *>(def->default_value.get())->values.front();
        if (def->nullable)
            config.set_key_value(key, new ConfigOptionEnumsGenericNullable(def->enum_keys_map, 2, value));
        else
            config.set_key_value(key, new ConfigOptionEnumsGeneric(def->enum_keys_map, 2, value));
    }
}
// One box per part, each on its own filament; a zero offset leaves the part where it is added.
struct BoxPart { Vec3d size, offset; int filament; ModelVolumeType type = ModelVolumeType::MODEL_PART; };

void add_parts(Model &model, const char *name, const std::vector<BoxPart> &parts)
{
    ModelObject *object = model.add_object();
    object->name = name;
    for (const BoxPart &part : parts) {
        ModelVolume *volume = object->add_volume(make_cube(part.size.x(), part.size.y(), part.size.z()), part.type, false);
        if (!part.offset.isZero())
            volume->set_offset(part.offset);
        volume->config.set_key_value("extruder", new ConfigOptionInt(part.filament));
    }
    object->add_instance();
    object->ensure_on_bed();
}

void apply_silent(Model &model, Print &print, const DynamicPrintConfig &config)
{
    print.apply(model, config);
    print.set_status_silent();
}

void add_two_tool_object(Model &model, Print &print, const DynamicPrintConfig &config, double height = 3.0)
{
    add_parts(model, "off-parity-two-tool", {{Vec3d(10., 20., height), Vec3d::Zero(), 1}, {Vec3d(20., 20., height), Vec3d(10., 0., 0.), 2}});
    apply_silent(model, print, config);
}

// The second part has a 1 mm gap: layers between 1 and 2 mm print on one tool.
void add_gapped_two_tool_object(Model &model, Print &print, const DynamicPrintConfig &config)
{
    add_parts(model, "off-parity-gapped-two-tool", {{Vec3d(10., 20., 3.), Vec3d::Zero(), 1}, {Vec3d(20., 20., 1.), Vec3d(10., 0., 0.), 2},
                                                    {Vec3d(20., 20., 1.), Vec3d(10., 0., 2.), 2}});
    apply_silent(model, print, config);
}

std::string tower_start_tag()
{
    return ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Tower_Start);
}
std::string tower_end_tag()
{
    return ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Tower_End);
}
double stated_height(const std::string &line)
{
    if (line.empty() || line[0] != ';')
        return std::nan("");
    if (line.find("HEIGHT:") == std::string::npos)
        return std::nan("");
    const size_t colon = line.rfind(':');
    if (colon == std::string::npos)
        return std::nan("");
    try {
        return std::stod(line.substr(colon + 1));
    } catch (...) {
        return std::nan("");
    }
}
std::vector<std::string> lines_of(const std::string &gcode)
{
    std::vector<std::string> lines;
    std::istringstream stream(gcode);
    for (std::string line; std::getline(stream, line);)
        lines.emplace_back(std::move(line));
    return lines;
}
bool starts_with(const std::string &line, const std::string &prefix)
{
    return line.rfind(prefix, 0) == 0;
}
std::string code_of(const std::string &line)
{
    const size_t comment = line.find(';');
    return comment == std::string::npos ? line : line.substr(0, comment);
}
double word_value(const std::string &code, char word)
{
    for (size_t i = 0; i < code.size(); ++i) {
        if (code[i] != word)
            continue;
        if (i > 0 && ! std::isspace(static_cast<unsigned char>(code[i - 1])))
            continue;
        try {
            return std::stod(code.substr(i + 1));
        } catch (...) {
            return std::nan("");
        }
    }
    return std::nan("");
}
bool is_g1(const std::string &code) { return starts_with(code, "G1 ") || code == "G1"; }

struct Move {
    size_t line_index   = 0;
    double length       = 0.;   // mm of XY travel
    double e            = 0.;   // this move's own E (the fixtures run relative E)
    bool   has_x = false, has_y = false;
    bool   has_feedrate = false;
    double feedrate     = 0.;
    bool   carries_xy() const { return has_x || has_y; }
    double e_per_mm() const { return length > 0. ? e / length : 0.; }
};
std::vector<Move> moves_of(const std::vector<std::string> &lines)
{
    std::vector<Move> moves;
    double x = std::nan("");
    double y = std::nan("");
    for (size_t i = 0; i < lines.size(); ++i) {
        const std::string code = code_of(lines[i]);
        if (! is_g1(code))
            continue;
        const double nx = word_value(code, 'X');
        const double ny = word_value(code, 'Y');
        const double e  = word_value(code, 'E');
        const double f  = word_value(code, 'F');
        Move move;
        move.line_index   = i;
        move.has_x        = ! std::isnan(nx);
        move.has_y        = ! std::isnan(ny);
        move.has_feedrate = ! std::isnan(f);
        move.feedrate     = std::isnan(f) ? 0. : f;
        move.e            = std::isnan(e) ? 0. : e;
        const double to_x = std::isnan(nx) ? x : nx;
        const double to_y = std::isnan(ny) ? y : ny;
        if (move.carries_xy() && ! std::isnan(x) && ! std::isnan(y) && ! std::isnan(to_x) && ! std::isnan(to_y))
            move.length = std::hypot(to_x - x, to_y - y);
        x = to_x;
        y = to_y;
        moves.emplace_back(move);
    }
    return moves;
}

struct Bracket {
    size_t start_line = 0;
    size_t end_line   = 0;
    bool   closed     = false;
};
std::vector<Bracket> brackets_of(const std::vector<std::string> &lines)
{
    std::vector<Bracket> brackets;
    for (size_t i = 0; i < lines.size(); ++i) {
        if (starts_with(lines[i], tower_start_tag())) {
            Bracket bracket;
            bracket.start_line = i;
            bracket.end_line   = lines.size();
            brackets.emplace_back(bracket);
        } else if (starts_with(lines[i], tower_end_tag()) && ! brackets.empty() && ! brackets.back().closed) {
            brackets.back().end_line = i;
            brackets.back().closed   = true;
        }
    }
    return brackets;
}
using Slic3r::Test::strip_nondeterministic_lines;
void check_no_mixed_nozzle_trace(const std::string &gcode)
{
    CHECK(gcode.find("MIXED_NOZZLE_TOWER_LAG") == std::string::npos);
    CHECK(gcode.find("SRL_PLAN_START") == std::string::npos);
    CHECK(gcode.find("SRL_GRID") == std::string::npos);
    CHECK(gcode.find("SRL_CELL") == std::string::npos);
}
void check_no_tower_return_lead_in(const std::vector<std::string> &lines, const std::vector<Bracket> &brackets)
{
    const std::vector<Move> moves = moves_of(lines);
    size_t checked_returns = 0;
    for (const Bracket &bracket : brackets) {
        if (! bracket.closed)
            continue;
        const Move *first  = nullptr;
        const Move *second = nullptr;
        for (const Move &move : moves) {
            if (move.line_index <= bracket.end_line)
                continue;
            if (move.e <= 0. || move.length <= 0.)
                continue;
            if (first == nullptr)
                first = &move;
            else {
                second = &move;
                break;
            }
        }
        if (first == nullptr || second == nullptr)
            continue;
        ++ checked_returns;
        const double ratio = second->e_per_mm() > 0. ? first->e_per_mm() / second->e_per_mm() : 1.;
        const bool   looks_like_lead_in =
            std::abs(first->length - mixed_nozzle_tower_return_lead_in_mm) < 0.05 &&
            std::abs(ratio - mixed_nozzle_tower_return_lead_in_flow) < 0.01;
        INFO("first bead after a tower return: line " << first->line_index + 1 << ", length "
             << first->length << " mm, flow ratio to the next bead " << ratio);
        CHECK_FALSE(looks_like_lead_in);
    }
    INFO("tower returns examined: " << checked_returns);
    CHECK(checked_returns > 0);
}
void check_brackets_match_plan(const Print &print, const std::vector<Bracket> &brackets)
{
    size_t              planned_brackets = 0;
    std::vector<double> planned_z;
    for (const std::vector<WipeTower::ToolChangeResult> &level : print.wipe_tower_data().tool_changes)
        for (const WipeTower::ToolChangeResult &block : level) {
            const size_t opens = [&]() {
                size_t count = 0;
                for (const std::string &line : lines_of(block.gcode))
                    if (starts_with(line, tower_start_tag()))
                        ++ count;
                return count;
            }();
            if (opens == 0)
                continue;
            planned_brackets += opens;
            planned_z.emplace_back(double(block.print_z));
            CHECK(block.print_z > 0.f);
        }
    size_t closed = 0;
    for (const Bracket &bracket : brackets)
        if (bracket.closed)
            ++ closed;
    INFO("planned tower brackets: " << planned_brackets << ", emitted: " << brackets.size()
         << ", closed: " << closed);
    CHECK(planned_brackets > 0);
    CHECK(brackets.size() == planned_brackets);
    CHECK(closed == brackets.size());
    for (size_t i = 1; i < planned_z.size(); ++i) {
        INFO("plan block " << i << " at Z " << planned_z[i] << " follows Z " << planned_z[i - 1]);
        CHECK(planned_z[i] >= planned_z[i - 1] - EPSILON);
    }
}
void check_tool_change_count_matches_plan(const Print &print, const std::string &gcode)
{
    size_t planned     = 0;
    size_t planned_all = 0;
    for (const std::vector<WipeTower::ToolChangeResult> &level : print.wipe_tower_data().tool_changes)
        for (const WipeTower::ToolChangeResult &block : level)
            if (block.is_tool_change && ! block.priming && ! block.gcode.empty()) {
                ++ planned_all;
                if (block.initial_tool != block.new_tool)
                    ++ planned;
            }
    const size_t emitted = Emitted::extract_emitted_facts(gcode).tool_changes.size();
    INFO("planned tool-change blocks: " << planned_all << ", of those that move the tool: " << planned
         << ", emitted tool moves: " << emitted
         << ", tower's own count: " << print.wipe_tower_data().number_of_toolchanges);
    CHECK(planned > 0);
    CHECK(emitted == planned);
}
void require_validates(Print &print)
{
    const std::string reason = print.validate().string;
    INFO("print.validate() said: '" << reason << "'");
    REQUIRE(reason.empty());
}
// The G-code of an Off plate on a Bambu printer with the two-tool object `height` tall, or with `gapped` the
// object whose second part has a gap, validated and exported.
std::string off_gcode(Model &model, Print &print, const DynamicPrintConfig &config, bool gapped = false, double height = 3.)
{
    print.is_BBL_printer() = true;
    if (gapped)
        add_gapped_two_tool_object(model, print, config);
    else
        add_two_tool_object(model, print, config, height);
    require_validates(print);
    std::string gcode = Slic3r::Test::gcode(print);
    REQUIRE_FALSE(gcode.empty());
    return gcode;
}

// The two-tool object on a Bambu printer, sliced through the harness; requires G-code.
CadenceTest::Facts off_facts(DynamicPrintConfig config)
{
    mixed_nozzle_fixtures::fill_per_filament_values(config);
    Model model;
    Print print;
    print.is_BBL_printer() = true;
    add_two_tool_object(model, print, config);
    CadenceTest::Facts facts = CadenceTest::slice(print);
    INFO("validation key=" << facts.refusal.opt_key << "; " << facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    return facts;
}

// The tower stream's deliberate differences from upstream are stated, not hidden: every bracket states its
// height, every feedrate in it is real, every block's first move states its feedrate, and E keeps the stream's
// precision.
void check_stated_tower_stream(const Print &print, const std::vector<std::string> &lines, const std::vector<Bracket> &brackets)
{
    REQUIRE_FALSE(brackets.empty());
    for (const Bracket &bracket : brackets) {
        REQUIRE(bracket.start_line + 1 < lines.size());
        const std::string &next   = lines[bracket.start_line + 1];
        const double       stated = stated_height(next);
        INFO("line after the tower bracket opened: '" << next << "', height read: " << stated);
        CHECK_FALSE(std::isnan(stated));
        CHECK(stated > 0.);
    }
    const std::vector<Move> moves = moves_of(lines);
    size_t checked_feedrates   = 0;
    for (const Bracket &bracket : brackets) {
        if (! bracket.closed)
            continue;
        for (const Move &move : moves) {
            if (move.line_index <= bracket.start_line || move.line_index >= bracket.end_line)
                continue;
            if (move.has_feedrate) {
                INFO("feedrate inside a tower bracket at line " << move.line_index + 1 << ": F" << move.feedrate);
                CHECK(move.feedrate >= 1.);
                ++ checked_feedrates;
            }
        }
    }
    CHECK(checked_feedrates > 0);
    size_t checked_first_moves = 0;
    for (const std::vector<WipeTower::ToolChangeResult> &level : print.wipe_tower_data().tool_changes)
        for (const WipeTower::ToolChangeResult &block : level) {
            if (block.gcode.empty())
                continue;
            const std::vector<std::string> block_lines = lines_of(block.gcode);
            for (const Move &move : moves_of(block_lines)) {
                if (! move.carries_xy())
                    continue;
                INFO("first XY move of a tower block: '" << block_lines[move.line_index] << "'");
                CHECK(move.has_feedrate);
                ++ checked_first_moves;
                break;
            }
        }
    CHECK(checked_first_moves > 0);
    const int digits = Slic3r::GCodeFormatter::e_digits(false);
    size_t    at_full_precision = 0;
    for (const Bracket &bracket : brackets) {
        if (! bracket.closed)
            continue;
        for (size_t i = bracket.start_line + 1; i < bracket.end_line; ++i) {
            const std::string code = code_of(lines[i]);
            if (! is_g1(code))
                continue;
            const size_t at = code.find(" E");
            if (at == std::string::npos)
                continue;
            const size_t end = code.find(' ', at + 1);
            const std::string token = code.substr(at + 2, end == std::string::npos ? std::string::npos : end - at - 2);
            const size_t dot = token.find('.');
            if (dot == std::string::npos)
                continue;
            const size_t fraction = token.size() - dot - 1;
            INFO("tower E token at line " << i + 1 << ": '" << token << "'");
            CHECK(fraction <= size_t(digits));
            if (fraction == size_t(digits))
                ++ at_full_precision;
        }
    }
    CHECK(at_full_precision > 0);
}
} // namespace

TEST_CASE("An Off two-filament tower keeps planned brackets and tool changes with either timelapse mode",
          "[TestRebuild][OffParityRebuild]")
{
    const bool smooth = GENERATE(false, true);
    DynamicPrintConfig config = off_parity_tower_config();
    if (smooth)
        config.set_key_value("timelapse_type", new ConfigOptionEnum<TimelapseType>(TimelapseType::tlSmooth));
    std::string gcodes[2];
    for (std::string &gcode : gcodes) {
        Model model;
        Print print;
        gcode = off_gcode(model, print, config);
        REQUIRE(print.has_wipe_tower());
        const auto lines = lines_of(gcode);
        const auto brackets = brackets_of(lines);
        check_no_mixed_nozzle_trace(gcode);
        check_brackets_match_plan(print, brackets);
        check_tool_change_count_matches_plan(print, gcode);
        check_no_tower_return_lead_in(lines, brackets);
        if (!smooth && &gcode == &gcodes[0])
            check_stated_tower_stream(print, lines, brackets);
    }
    CHECK(strip_nondeterministic_lines(gcodes[0]) == strip_nondeterministic_lines(gcodes[1]));
}

TEST_CASE("Off parity: no sparse layers deposits on the levels upstream deposits on", "[TestRebuild][OffParityRebuild]")
{
    DynamicPrintConfig config = off_parity_tower_config();
    config.set_key_value("wipe_tower_no_sparse_layers", new ConfigOptionBool(true));
    Model model;
    Print print;
    const std::string gcode = off_gcode(model, print, config, true);
    REQUIRE(print.has_wipe_tower());
    size_t expected_brackets = 0;
    size_t skipped_levels    = 0;
    for (const std::vector<WipeTower::ToolChangeResult> &level : print.wipe_tower_data().tool_changes) {
        if (level.empty())
            continue;
        if (level.size() == 1 && level.front().initial_tool == level.front().new_tool) {
            ++ skipped_levels;
            continue;
        }
        for (const WipeTower::ToolChangeResult &block : level)
            for (const std::string &line : lines_of(block.gcode))
                if (starts_with(line, tower_start_tag()))
                    ++ expected_brackets;
    }
    const std::vector<Bracket> brackets = brackets_of(lines_of(gcode));
    INFO("levels upstream skips: " << skipped_levels << ", brackets upstream emits: "
         << expected_brackets << ", brackets in the file: " << brackets.size());
    REQUIRE(skipped_levels > 0);
    CHECK(expected_brackets > 0);
    CHECK(brackets.size() == expected_brackets);
}

TEST_CASE("Off parity: purge rows keep the tower's shared width on unequal nozzles", "[TestRebuild][OffParityRebuild]")
{
    DynamicPrintConfig config = off_parity_tower_config();
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    // Absolute widths of 100% of the 0.2 nozzle equal the 0.2 mm layer and fail validation; auto widths do not.
    for (const char *key : {"bridge_line_width", "skin_infill_line_width", "skeleton_infill_line_width"})
        config.set_key_value(key, new ConfigOptionFloatOrPercent(0., false));
    const CadenceTest::Facts facts = off_facts(config);
    // Both tools build the tower at the shared last-nozzle width.
    std::array<size_t, 2> shared_roads{};
    for (const auto &move : facts.moves)
        if (move.type == EMoveType::Extrude && move.extrusion_role == erWipeTower &&
            move.physical_tool_id < 2 && move.mm3_per_mm > 0. &&
            std::abs(move.width - .75f) < .01f)
            ++shared_roads[move.physical_tool_id];
    CAPTURE(shared_roads);
    CHECK(shared_roads[0] > 0);
    CHECK(shared_roads[1] > 0);
}

TEST_CASE("Off parity: a first-layer flow ratio above one reaches the tower whole", "[TestRebuild][OffParityRebuild]")
{
    const float ratio = 1.3f;
    DynamicPrintConfig config = off_parity_tower_config();
    config.set_key_value("set_other_flow_ratios", new ConfigOptionBool(true));
    config.set_key_value("first_layer_flow_ratio", new ConfigOptionFloat(double(ratio)));
    const CadenceTest::Facts facts = off_facts(config);
    // The first-layer ratio widens the shared 0.4 nozzle's 0.5 mm tower road to 0.65 mm; later rows stay at 0.5 mm.
    float first_print_z = std::numeric_limits<float>::max();
    for (const auto &move : facts.moves)
        if (move.type == EMoveType::Extrude && move.extrusion_role == erWipeTower &&
            move.mm3_per_mm > 0. && move.print_z > 0.)
            first_print_z = std::min(first_print_z, move.print_z);
    REQUIRE(first_print_z < std::numeric_limits<float>::max());
    size_t first_boosted = 0, later_shared = 0, later_boosted = 0;
    for (const auto &move : facts.moves) {
        if (move.type != EMoveType::Extrude || move.extrusion_role != erWipeTower ||
            move.mm3_per_mm <= 0.)
            continue;
        const bool boosted = std::abs(move.width - ratio * .5f) < .01f;
        // By declared tower level: travel and Z-hop can put a vertex above its print layer.
        if (std::abs(move.print_z - first_print_z) < .01f)
            first_boosted += boosted;
        else {
            later_shared += std::abs(move.width - .5f) < .01f;
            later_boosted += boosted;
        }
    }
    CAPTURE(first_print_z, first_boosted, later_shared, later_boosted);
    CHECK(first_boosted > 0);
    CHECK(later_shared > 0);
    CHECK(later_boosted == 0);
}

namespace {
struct PurgeRow {
    size_t block = 0;      // the tool-change block it is in
    float  print_z = 0.f;  // that block's print Z
    double effective_f = 0.;
    double width = 0.;
    double height = 0.;
    bool states_f = false;
};
double stated_width(const std::string &line)
{
    const std::string tag = ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Width);
    if (! starts_with(line, tag))
        return std::nan("");
    try {
        return std::stod(line.substr(tag.size()));
    } catch (...) {
        return std::nan("");
    }
}
std::vector<PurgeRow> purge_rows(const DynamicPrintConfig &config, size_t &stated_feedrates)
{
    Model model;
    Print print;
    off_gcode(model, print, config);
    REQUIRE(print.has_wipe_tower());
    std::vector<PurgeRow> rows;
    stated_feedrates = 0;
    size_t blocks = 0;
    for (const std::vector<WipeTower::ToolChangeResult> &level : print.wipe_tower_data().tool_changes)
        for (const WipeTower::ToolChangeResult &block : level) {
            if (! block.is_tool_change || block.gcode.empty())
                continue;
            ++ blocks;
            const std::vector<std::string> lines = lines_of(block.gcode);
            double current_f = 0., width = 0., height = 0.;
            bool   in_nozzle_change = false;
            size_t next = 0;
            for (const Move &move : moves_of(lines)) {
                // The nozzle-change tags and the stated cross-section on the lines before this move.
                for (; next < move.line_index; ++ next) {
                    const std::string &line = lines[next];
                    if (line.find(GCodeProcessor::Nozzle_Change_Start_Tag) != std::string::npos)
                        in_nozzle_change = true;
                    else if (line.find(GCodeProcessor::Nozzle_Change_End_Tag) != std::string::npos)
                        in_nozzle_change = false;
                    else {
                        if (const double stated = stated_width(line); ! std::isnan(stated))
                            width = stated;
                        if (const double stated = stated_height(line); ! std::isnan(stated))
                            height = stated;
                    }
                }
                next = move.line_index + 1;
                if (move.has_feedrate) {
                    current_f = move.feedrate;
                    stated_feedrates += in_nozzle_change ? 0 : 1;
                }
                // A purge row: a positive-E move along X only.
                if (! in_nozzle_change && move.has_x && ! move.has_y && move.e > 0. && move.length > 0.)
                    rows.push_back({blocks, block.print_z, current_f, width, height, move.has_feedrate});
            }
        }
    return rows;
}
} // namespace

TEST_CASE("Off parity: the volumetric ceiling limits feedrates, it does not state new ones", "[TestRebuild][OffParityRebuild]")
{
    const double loose_ceiling = 200.;   // mm3/s, far above anything the tower asks for
    const double bound_ceiling = 0.5;    // mm3/s, well under it
    const double filament_area = M_PI / 4. * 1.75 * 1.75;
    DynamicPrintConfig loose = off_parity_tower_config();
    loose.set_key_value("filament_max_volumetric_speed", new ConfigOptionFloatsNullable{loose_ceiling, loose_ceiling});
    DynamicPrintConfig bound = off_parity_tower_config();
    bound.set_key_value("filament_max_volumetric_speed", new ConfigOptionFloatsNullable{bound_ceiling, bound_ceiling});
    size_t loose_statements = 0;
    size_t bound_statements = 0;
    const std::vector<PurgeRow> loose_rows = purge_rows(loose, loose_statements);
    const std::vector<PurgeRow> bound_rows = purge_rows(bound, bound_statements);
    INFO("purge rows: " << loose_rows.size() << " at a ceiling of " << loose_ceiling << " mm3/s, "
         << bound_rows.size() << " at " << bound_ceiling << " mm3/s; feedrate statements: "
         << loose_statements << " and " << bound_statements);
    REQUIRE(loose_rows.size() > 0);
    REQUIRE(bound_rows.size() == loose_rows.size());
    size_t      limited             = 0;
    size_t      faster_when_bound   = 0;
    size_t      stated_over_ceiling = 0;
    size_t      unreadable          = 0;
    for (size_t i = 0; i < bound_rows.size(); ++ i) {
        const PurgeRow &row = bound_rows[i];
        if (row.effective_f < loose_rows[i].effective_f - 1.)
            ++ limited;
        if (row.effective_f > loose_rows[i].effective_f + 1.)
            ++ faster_when_bound;
        if (! row.states_f)
            continue;
        if (! (row.width > 0.) || ! (row.height > 0.)) {
            ++ unreadable;
            continue;
        }
        const double flow    = row.height * (row.width - row.height * (1. - M_PI / 4.)) / filament_area;
        const double allowed = 60. * (bound_ceiling / filament_area) / flow;
        if (row.effective_f > allowed * 1.02 + 1.)
            ++stated_over_ceiling;
    }
    INFO("rows limited by the tight ceiling: " << limited << ", rows faster under it: "
         << faster_when_bound << ", rows whose block declared no cross-section: " << unreadable
         << ", stated rates over their own ceiling: " << stated_over_ceiling);
    REQUIRE(limited > 0);
    CHECK(faster_when_bound == 0);
    CHECK(stated_over_ceiling == 0);
    CHECK(bound_statements <= loose_statements);
}

TEST_CASE("Off parity: the first-layer purge lays the rows the plan reserves", "[TestRebuild][OffParityRebuild]")
{
    DynamicPrintConfig config = off_parity_tower_config();
    config.set_key_value("prime_tower_infill_gap", new ConfigOptionPercent(400));
    size_t statements = 0;
    std::map<size_t, std::pair<float, size_t>> blocks; // per tool-change block: its print Z and its purge rows
    for (const PurgeRow &row : purge_rows(config, statements)) {
        blocks[row.block].first = row.print_z;
        ++ blocks[row.block].second;
    }
    size_t first_layer_rows = 0;
    size_t later_rows       = 0;
    for (const auto &[block, rows] : blocks) {
        size_t &most = rows.first <= 0.25f ? first_layer_rows : later_rows;
        most = std::max(most, rows.second);
    }
    INFO("purge rows, first-layer block: " << first_layer_rows << ", ordinary block: " << later_rows);
    REQUIRE(later_rows > 0);
    REQUIRE(first_layer_rows > 0);
    CHECK(first_layer_rows > 2 * later_rows);
}

TEST_CASE("Off parity: a support filament resolves at its own column", "[TestRebuild][OffParityRebuild]")
{
    DynamicPrintConfig dynamic = off_parity_base_config();
    dynamic.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    dynamic.set_key_value("filament_map", new ConfigOptionInts{2, 1});
    dynamic.set_key_value("enable_support", new ConfigOptionBool(true));
    dynamic.set_key_value("support_filament", new ConfigOptionInt(1));
    dynamic.set_key_value("support_interface_filament", new ConfigOptionInt(2));
    dynamic.set_key_value("support_type", new ConfigOptionEnum<SupportType>(stNormalAuto));
    for (const char *key : {"support_line_width", "line_width", "initial_layer_line_width", "bridge_line_width",
                            "skin_infill_line_width", "skeleton_infill_line_width"})
        dynamic.set_key_value(key, new ConfigOptionFloatOrPercent(0., false));
    dynamic.set_key_value("independent_support_layer_height", new ConfigOptionBool(false));
    mixed_nozzle_fixtures::fill_per_filament_values(dynamic);
    Model model;
    Print print;
    mixed_nozzle_fixtures::init_feature_overhang_fixture(model, print, dynamic);
    const CadenceTest::Facts facts = CadenceTest::slice(print);
    INFO("validation key=" << facts.refusal.opt_key << "; " << facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    size_t base = 0, interface_roads = 0, wrong = 0;
    for (const auto &move : facts.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        // Off reads the filament's own column: filament 1 on the 0.6 tool keeps column 1's 0.2 mm width, and vice versa.
        if (move.extrusion_role == erSupportMaterial) {
            ++base;
            wrong += move.extruder_id != 0 || move.physical_tool_id != 1 ||
                     std::abs(move.width - .2f) > 1e-3f || std::abs(move.height - .2f) > 1e-3f;
        } else if (move.extrusion_role == erSupportMaterialInterface) {
            ++interface_roads;
            wrong += move.extruder_id != 1 || move.physical_tool_id != 0 ||
                     std::abs(move.width - .6f) > 1e-3f || std::abs(move.height - .2f) > 1e-3f;
        }
    }
    CAPTURE(base, interface_roads, wrong);
    CHECK(base > 0);
    CHECK(interface_roads > 0);
    CHECK(wrong == 0);
}

TEST_CASE("Off parity: a wipe plans its retraction at the extruder's column", "[TestRebuild][OffParityRebuild]")
{
    DynamicPrintConfig config = off_parity_tower_config();
    config.set_key_value("filament_map", new ConfigOptionInts{1, 1});
    config.set_key_value("wipe", new ConfigOptionBools{true, true});
    config.set_key_value("wipe_distance", new ConfigOptionFloats{0., 20.});
    config.set_key_value("retraction_length", new ConfigOptionFloats{1., 1.});
    config.set_key_value("retraction_speed", new ConfigOptionFloats{30., 30.});
    config.set_key_value("retract_before_wipe", new ConfigOptionPercents{0., 0.});
    config.set_key_value("retract_after_wipe", new ConfigOptionPercents{0., 0.});
    Model model;
    Print print;
    print.is_BBL_printer() = true;
    add_two_tool_object(model, print, config);
    require_validates(print);
    const std::string gcode = Slic3r::Test::gcode(print);
    REQUIRE_FALSE(gcode.empty());
    const std::string wipe_start = ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Start);
    const std::string wipe_end   = ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_End);
    size_t wipe_moves     = 0;
    size_t moves_carrying_e = 0;
    bool   in_wipe        = false;
    for (const std::string &line : lines_of(gcode)) {
        if (starts_with(line, wipe_start)) { in_wipe = true;  continue; }
        if (starts_with(line, wipe_end))   { in_wipe = false; continue; }
        if (! in_wipe)
            continue;
        const std::string code = code_of(line);
        if (! is_g1(code))
            continue;
        if (std::isnan(word_value(code, 'X')) && std::isnan(word_value(code, 'Y')))
            continue;
        ++ wipe_moves;
        if (! std::isnan(word_value(code, 'E')))
            ++ moves_carrying_e;
    }
    INFO("wipe moves: " << wipe_moves << ", of those carrying an E: " << moves_carrying_e);
    REQUIRE(wipe_moves > 0);
    CHECK(moves_carrying_e == 0);
}

TEST_CASE("Off parity: a legacy change_filament_gcode expands as it was written", "[TestRebuild][OffParityRebuild]")
{
    DynamicPrintConfig config = off_parity_base_config();
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.6, 0.2});
    config.set_key_value("layer_height", new ConfigOptionFloat(0.10));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(0.10));
    config.set_key_value("filament_map", new ConfigOptionInts{1, 1});
    config.set_key_value("change_filament_gcode", new ConfigOptionString(
        ";======== H2D ========\n"
        "{if (nozzle_diameter[current_extruder] == 0.2)}{endif}\n"
        "{if (nozzle_diameter[next_extruder] == 0.2)}{endif}\n"
        "M620.10 A0 H{nozzle_diameter[current_extruder]}\n"
        "M620.10 A0 H{nozzle_diameter[current_extruder]}\n"
        "M620.10 A1 H{nozzle_diameter[next_extruder]}\n"
        "M620.10 A1 H{nozzle_diameter[next_extruder]}\n"));
    Model model;
    Print print;
    const std::string gcode = off_gcode(model, print, config);
    size_t arriving_lines = 0;
    size_t arriving_at_the_fine_filament = 0;
    for (const std::string &line : lines_of(gcode)) {
        if (! starts_with(line, "M620.10 A1"))
            continue;
        ++ arriving_lines;
        if (line.find("H0.2") != std::string::npos)
            ++ arriving_at_the_fine_filament;
    }
    INFO("legacy M620.10 A1 lines: " << arriving_lines << ", of those stating the 0.2 filament entry: "
         << arriving_at_the_fine_filament);
    REQUIRE(arriving_lines > 0);
    CHECK(arriving_at_the_fine_filament > 0);
}

namespace {
void add_off_trim_object(Model &model, bool with_modifier)
{
    if (with_modifier)
        add_parts(model, "off-trim-modifier", {{Vec3d(30., 20., 3.), Vec3d::Zero(), 1},
                                               {Vec3d(20., 20., 3.), Vec3d(10., 0., 0.), 2, ModelVolumeType::PARAMETER_MODIFIER}});
    else
        add_parts(model, "off-trim-two-parts", {{Vec3d(10., 20., 3.), Vec3d::Zero(), 1}, {Vec3d(20., 20., 3.), Vec3d(10., 0., 0.), 2}});
}
} // namespace

TEST_CASE("Off parity: a multi-region object keeps XY compensation and elephant-foot trimming, whatever its fine-skin counts",
          "[TestRebuild][OffParityRebuild][SynchronizedRegionalLayers]")
{
    const bool with_modifier = GENERATE(false, true);
    INFO((with_modifier ? "one part and a filament modifier" : "two parts on two filaments"));
    struct Trim { const char *name; double contour; double elephant_foot; };
    const Trim trim = GENERATE(Trim{"contour shrink", -0.2, 0.}, Trim{"contour growth", 0.1, 0.},
                               Trim{"elephant foot", 0., 0.2});
    INFO(trim.name);
    DynamicPrintConfig config = off_parity_base_config();
    config.set_key_value("xy_contour_compensation", new ConfigOptionFloat(trim.contour));
    config.set_key_value("xy_hole_compensation", new ConfigOptionFloat(0.));
    config.set_key_value("elefant_foot_compensation", new ConfigOptionFloat(trim.elephant_foot));
    config.set_key_value("elefant_foot_compensation_layers", new ConfigOptionInt(1));
    const auto sliced = [&config, with_modifier](bool fine_skin_counts) {
        Model model;
        add_off_trim_object(model, with_modifier);
        if (fine_skin_counts)
            for (const auto &[volume, count] : {std::pair<size_t, int>{0, 2}, {1, 5}})
                model.objects.front()->volumes[volume]->config.set_key_value("mixed_nozzle_body_fine_skin_layers", new ConfigOptionInt(count));
        Print print;
        apply_silent(model, print, config);
        REQUIRE(print.validate().string.empty());
        return Slic3r::Test::gcode(print);
    };
    const std::string gcode = sliced(false);
    // Parts saved with different Body Split fine-skin layer counts print as if neither had one: the counts must not
    // split the regions' perimeters into separate passes.
    if (!with_modifier)
        CHECK(strip_nondeterministic_lines(sliced(true)) == strip_nondeterministic_lines(gcode));
    const auto moves = Slic3r::Emitted::emitted_moves(gcode);
    std::map<int, BoundingBoxf> outlines;
    for (const auto &move : moves) {
        if (move.type != EMoveType::Extrude || move.extrusion_role != erExternalPerimeter)
            continue;
        REQUIRE(move.width > 0.);
        auto &box = outlines[int(std::lround(move.position.z() * 1000.))];
        const Vec2d point(move.position.x(), move.position.y());
        const Vec2d radius(.5 * move.width, .5 * move.width);
        box.merge(point - radius);
        box.merge(point + radius);
    }
    REQUIRE(outlines.size() > 10);
    const Vec2d first = outlines.begin()->second.size();
    const Vec2d middle = std::next(outlines.begin(), 7)->second.size();
    if (trim.elephant_foot > 0.) {
        CHECK_THAT(middle.x(), Catch::Matchers::WithinAbs(30., .05));
        CHECK_THAT(middle.y(), Catch::Matchers::WithinAbs(20., .05));
        CHECK(first.x() < middle.x() - .1);
        CHECK(first.y() < middle.y() - .1);
    } else {
        const Vec2d expected(30. + 2. * trim.contour, 20. + 2. * trim.contour);
        CHECK_THAT(middle.x(), Catch::Matchers::WithinAbs(expected.x(), .05));
        CHECK_THAT(middle.y(), Catch::Matchers::WithinAbs(expected.y(), .05));
        CHECK_THAT(first.x(), Catch::Matchers::WithinAbs(expected.x(), .05));
        CHECK_THAT(first.y(), Catch::Matchers::WithinAbs(expected.y(), .05));
    }
}

TEST_CASE("Off material overrides inherit disabled wiping until a filament explicitly enables it",
          "[TestRebuild][OffParityRebuild][NullableMaterial]")
{
    std::array<size_t, 2> wipe_counts{};
    for (size_t explicit_wipe = 0; explicit_wipe < wipe_counts.size(); ++explicit_wipe) {
        auto config = off_parity_base_config();
        config.set_key_value("machine_start_gcode", new ConfigOptionString("G28\nM1020 S[initial_extruder]\n"));
        config.set_key_value("wipe", new ConfigOptionBools{false, false});
        config.set_key_value("wipe_distance", new ConfigOptionFloats{3., 3.});
        config.set_key_value("retraction_length", new ConfigOptionFloats{1., 1.});
        config.set_key_value("retraction_minimum_travel", new ConfigOptionFloats{0., 0.});
        config.set_key_value("filament_wipe", new ConfigOptionBoolsNullable{
            ConfigOptionBoolsNullable::nil_value(),
            explicit_wipe ? static_cast<unsigned char>(1) : ConfigOptionBoolsNullable::nil_value()});
        name_enum_lists(config);
        Model model;
        Print print;
        print.is_BBL_printer() = true;
        add_two_tool_object(model, print, config);
        const auto facts = CadenceTest::slice(print);
        INFO(facts.refusal.string);
        REQUIRE(facts.refusal.string.empty());
        REQUIRE_FALSE(facts.gcode.empty());
        std::array<size_t, 2> model_roads{};
        for (const auto &move : facts.moves)
            if (CadenceTest::model_road(move)) {
                REQUIRE(move.physical_tool_id < model_roads.size());
                ++model_roads[move.physical_tool_id];
                CHECK(move.mm3_per_mm > 0.);
            }
        CHECK(model_roads[0] > 0);
        CHECK(model_roads[1] > 0);
        const std::string marker = ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Start);
        for (const auto &line : lines_of(facts.gcode))
            wipe_counts[explicit_wipe] += starts_with(line, marker);
        ScopedTemporaryFile saved(".gcode");
        { std::ofstream output(saved.string(), std::ios::binary); output << facts.gcode; }
        DynamicPrintConfig restored;
        REQUIRE_NOTHROW(restored.load_from_gcode_file(saved.string(),
                                                     ForwardCompatibilitySubstitutionRule::Disable));
        Print reopened;
        reopened.is_BBL_printer() = true;
        reopened.apply(model, restored);
        const auto repeated = CadenceTest::slice(reopened);
        REQUIRE(repeated.refusal.string.empty());
        REQUIRE_FALSE(repeated.gcode.empty());
        size_t restored_wipes = 0;
        for (const auto &line : lines_of(repeated.gcode))
            restored_wipes += starts_with(line, marker);
        CHECK(restored_wipes == wipe_counts[explicit_wipe]);
    }
    CAPTURE(wipe_counts[0], wipe_counts[1]);
    CHECK(wipe_counts[0] == 0);
    CHECK(wipe_counts[1] > 0);
}

namespace {
// Reads a gzip file written without a file name (gzip -n), inflating with miniz.
std::string read_gzip(const std::string &path)
{
    std::ifstream in(path, std::ios::binary);
    const std::string raw((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    REQUIRE(raw.size() > 18);
    REQUIRE(static_cast<unsigned char>(raw[0]) == 0x1f);
    REQUIRE(static_cast<unsigned char>(raw[1]) == 0x8b);
    REQUIRE(raw[3] == 0); // no optional header fields
    size_t out_len = 0;
    void *out = tinfl_decompress_mem_to_heap(raw.data() + 10, raw.size() - 18, &out_len, 0);
    REQUIRE(out != nullptr);
    std::string text(static_cast<const char *>(out), out_len);
    mz_free(out);
    return text;
}

// Drops the program name, version and time line, and the per-run object ids.
std::vector<std::string> comparable_lines(const std::string &gcode)
{
    std::vector<std::string> lines;
    std::istringstream stream(strip_nondeterministic_lines(gcode));
    for (std::string line; std::getline(stream, line);)
        lines.push_back(line);
    return lines;
}
// Splits G-code into the lines before the config block and the block's "key = value" pairs.
struct SplitGcode
{
    std::vector<std::string>           body;
    std::map<std::string, std::string> config;
};

SplitGcode split_config_block(const std::vector<std::string> &lines)
{
    SplitGcode out;
    bool in_config = false;
    for (const std::string &line : lines) {
        if (line == "; CONFIG_BLOCK_START") {
            in_config = true;
            continue;
        }
        if (line == "; CONFIG_BLOCK_END") {
            in_config = false;
            continue;
        }
        if (!in_config) {
            out.body.push_back(line);
            continue;
        }
        const size_t eq = line.find(" = ");
        if (line.rfind("; ", 0) == 0 && eq != std::string::npos)
            out.config[line.substr(2, eq - 2)] = line.substr(eq + 3);
        else
            out.body.push_back(line);
    }
    return out;
}

// Takes out the time estimate's own lines (M73 progress, estimated-time comments), which a whole-suite run can move,
// keeping the normal-mode estimate in seconds.
struct ProgressFree
{
    std::vector<std::string> lines;
    size_t                   progress_lines { 0 };
    double                   normal_seconds { -1. };
};

ProgressFree without_progress_lines(const std::vector<std::string> &body)
{
    ProgressFree out;
    for (const std::string &line : body) {
        if (line.rfind("M73 ", 0) == 0) {
            ++out.progress_lines;
            continue;
        }
        if (line.rfind("; estimated ", 0) == 0) {
            const std::string normal = "; estimated printing time (normal mode) = ";
            if (line.rfind(normal, 0) == 0) {
                // "1d 2h 3m 4s", any of the parts left out.
                double seconds = 0.;
                std::istringstream parts(line.substr(normal.size()));
                for (std::string part; parts >> part;) {
                    const char unit = part.back();
                    const double value = std::atof(part.substr(0, part.size() - 1).c_str());
                    seconds += value * (unit == 'd' ? 86400. : unit == 'h' ? 3600. : unit == 'm' ? 60. : 1.);
                }
                out.normal_seconds = seconds;
            }
            continue;
        }
        out.lines.push_back(line);
    }
    return out;
}

// Settings stock does not list: the mixed-nozzle ones Off ignores, and the differs-from-system mask the G-code handoff reads.
bool is_documented_added_setting(const std::string &key)
{
    return key.rfind("mixed_nozzle_", 0) == 0 || key.rfind("regional_", 0) == 0 || key.rfind("tool_changer_tower_", 0) == 0 ||
           key == "tool_changer_idle_cooldown" || key == "synchronized_multi_nozzle_layering" ||
           key == "different_settings_to_system";
}
} // namespace

TEST_CASE("Off matches stock OrcaSlicer G-code on the shared scenes", "[TestRebuild][OffParityRebuild][OffVsStock]")
{
    REQUIRE(DynamicPrintConfig::full_print_config().opt_enum<MixedNozzleSlicingMode>("mixed_nozzle_slicing_mode") ==
            MixedNozzleSlicingMode::Off);
    for (const std::string &name : StockOffScenes::names()) {
        INFO("scene " << name);
        const SplitGcode stock = split_config_block(
            comparable_lines(read_gzip(std::string(TEST_DATA_DIR) + "/stock-off/" + name + ".gcode.gz")));
        const SplitGcode ours = split_config_block(comparable_lines(StockOffScenes::slice(name)));

        // Every line outside the config block is the same, apart from the time estimate's own lines.
        const ProgressFree stock_body = without_progress_lines(stock.body);
        const ProgressFree ours_body = without_progress_lines(ours.body);
        size_t first_diff = 0;
        while (first_diff < stock_body.lines.size() && first_diff < ours_body.lines.size() &&
               stock_body.lines[first_diff] == ours_body.lines[first_diff])
            ++first_diff;
        {
            INFO("first differing line " << first_diff + 1 << " without progress lines (stock "
                 << stock_body.lines.size() << " lines, ours " << ours_body.lines.size() << ")");
            INFO("stock: " << (first_diff < stock_body.lines.size() ? stock_body.lines[first_diff] : std::string("<end>")));
            INFO("ours:  " << (first_diff < ours_body.lines.size() ? ours_body.lines[first_diff] : std::string("<end>")));
            CHECK(first_diff == stock_body.lines.size());
            CHECK(first_diff == ours_body.lines.size());
        }
        // Both carry progress lines and an estimated time, and the estimates agree within 1%.
        CHECK(ours_body.progress_lines > 0);
        CHECK((stock_body.progress_lines > 0) == (ours_body.progress_lines > 0));
        REQUIRE(stock_body.normal_seconds > 0.);
        CHECK_THAT(ours_body.normal_seconds, Catch::Matchers::WithinRel(stock_body.normal_seconds, 0.01));

        // Every stock setting is listed with the same value; additions are only the documented ones.
        for (const auto &[key, value] : stock.config) {
            INFO("setting " << key);
            const auto it = ours.config.find(key);
            REQUIRE(it != ours.config.end());
            CHECK(it->second == value);
        }
        for (const auto &[key, value] : ours.config)
            if (stock.config.count(key) == 0) {
                INFO("added setting " << key << " = " << value);
                CHECK(is_documented_added_setting(key));
            }
        CHECK(ours.config.at("mixed_nozzle_slicing_mode") == "off");
    }
}

// An in-tower travel that names the tower's travel speed left the next road, which names
// no feedrate, running at that travel speed, far over the filament's volumetric limit.
TEST_CASE("Off: a tower road that names no feedrate does not run at the tower's travel speed",
          "[TestRebuild][OffParityRebuild]")
{
    Model model;
    Print print;
    print.is_BBL_printer() = true;
    // Layers between 1 and 2 mm have no tool change, so the tower lays its sparse grid there: grid
    // roads name no feedrate and follow an in-tower travel.
    DynamicPrintConfig config = off_parity_tower_config();
    config.set_key_value("wipe_tower_no_sparse_layers", new ConfigOptionBool(false));
    config.set_key_value("wipe_tower_wall_type", new ConfigOptionEnum<WipeTowerWallType>(WipeTowerWallType::wtwRectangle));
    add_gapped_two_tool_object(model, print, config);
    REQUIRE(print.validate().string.empty());
    const std::string gcode = Slic3r::Test::gcode(print);
    REQUIRE_FALSE(gcode.empty());
    REQUIRE(print.has_wipe_tower());
    CHECK(gcode.find("; CP EMPTY GRID START") != std::string::npos);
    const double travel = print.config().travel_speed.get_at(0);
    const double limit  = 4.;
    // Line numbers (one-based, as gcode_id) inside a nozzle-change bracket: ramming has its own ceiling.
    std::vector<bool> ramming_line(1, false);
    {
        bool ramming = false;
        for (const std::string &line : lines_of(gcode)) {
            if (line.rfind("; NOZZLE_CHANGE_START", 0) == 0)
                ramming = true;
            ramming_line.push_back(ramming);
            if (line.rfind("; NOZZLE_CHANGE_END", 0) == 0)
                ramming = false;
        }
    }
    size_t roads = 0, over_limit = 0, at_travel_speed = 0;
    double worst = 0.;
    for (const auto &move : Emitted::emitted_moves(gcode)) {
        if (move.type != EMoveType::Extrude || move.extrusion_role != erWipeTower)
            continue;
        if (move.gcode_id < ramming_line.size() && ramming_line[move.gcode_id])
            continue;
        ++ roads;
        const double rate = move.volumetric_rate();
        worst = std::max(worst, rate);
        over_limit      += rate > limit * 1.02 ? 1 : 0;
        at_travel_speed += std::abs(move.feedrate - travel) < 0.01 ? 1 : 0;
    }
    CAPTURE(roads, over_limit, at_travel_speed, worst, travel);
    CHECK(roads > 0);
    CHECK(over_limit == 0);
    CHECK(at_travel_speed == 0);
}

// Upstream's tower continuity: every tower row with neither object nor support gets a placeholder support layer
// (id -1), above a raft and under a part floating above another.
TEST_CASE("Off: tower rows with neither object nor support get placeholder support layers",
          "[TestRebuild][OffParityRebuild][Rebase25]")
{
    DynamicPrintConfig config = off_parity_tower_config();
    Model model;
    Print print;
    print.is_BBL_printer() = true;

    SECTION("between the raft and the first object layer") {
        config.set_key_value("raft_layers", new ConfigOptionInt(3));
        add_two_tool_object(model, print, config);
    }
    SECTION("a part floating above another, held by support") {
        config.set_key_value("enable_support", new ConfigOptionBool(true));
        config.set_key_value("support_type", new ConfigOptionEnum<SupportType>(stNormalAuto));
        config.set_key_value("support_filament", new ConfigOptionInt(0));
        config.set_key_value("support_interface_filament", new ConfigOptionInt(0));
        config.set_key_value("independent_support_layer_height", new ConfigOptionBool(false));
        add_parts(model, "off-floating-part", {{Vec3d(20., 20., 1.), Vec3d::Zero(), 1}, {Vec3d(20., 20., 1.), Vec3d(0., 0., 3.), 2}});
        apply_silent(model, print, config);
    }

    const StringObjectException refusal = print.validate();
    INFO(refusal.string);
    REQUIRE(refusal.string.empty());
    print.process();
    REQUIRE(print.has_wipe_tower());

    const ToolOrdering &ordering = print.wipe_tower_data().tool_ordering;
    REQUIRE_FALSE(ordering.cadence_tower_rows());
    size_t tower_rows = 0, bare_rows = 0, typed_events = 0;
    for (const LayerTools &lt : ordering.layer_tools()) {
        if (!lt.has_wipe_tower)
            continue;
        ++tower_rows;
        bare_rows += !lt.has_object && !lt.has_support;
        typed_events += lt.tower_support_event;
    }
    size_t placeholders = 0;
    for (const SupportLayer *layer : print.objects().front()->support_layers())
        if (layer->id() == size_t(-1)) {
            ++placeholders;
            CHECK(layer->support_fills.entities.empty());
        }
    CAPTURE(tower_rows, bare_rows, typed_events, placeholders);
    CHECK(tower_rows > 0);
    // Every tower row is printed through an object or a support layer, none as a Cadence event.
    CHECK(bare_rows == 0);
    CHECK(typed_events == 0);
    CHECK(placeholders > 0);

    // The plate still exports, with the tower on its layers.
    const std::string gcode = Slic3r::Test::gcode(print);
    CHECK(gcode.find(tower_start_tag()) != std::string::npos);
}

TEST_CASE("Off parity: an object's layer height limits come from its filament's own column", "[TestRebuild][OffParityRebuild]")
{
    // Filament 1 prints on the 0.6 nozzle. Off reads the limits as upstream does, from the filament's
    // own column (the 0.2 nozzle's 0.04 mm minimum); a mode reads the nozzle that prints it (0.16 mm).
    const bool mode = GENERATE(false, true);
    CAPTURE(mode);
    DynamicPrintConfig dynamic = off_parity_base_config();
    dynamic.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    dynamic.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.16});
    dynamic.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.42});
    dynamic.set_key_value("filament_map", new ConfigOptionInts{2, 1});
    if (mode)
        dynamic.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::BodySplit));
    PrintConfig print_config;
    print_config.apply(dynamic, true);
    PrintObjectConfig object_config;
    object_config.apply(dynamic, true);
    object_config.layer_height.value = 0.3;
    const SlicingParameters params = SlicingParameters::create_from_config(print_config, object_config, 10., {0u}, Vec3d::Ones());
    CHECK_THAT(params.min_layer_height, Catch::Matchers::WithinAbs(mode ? 0.16 : 0.04, 1e-9));
}

TEST_CASE("Off parity: sparse infill keeps the object's line spacing on thinner layers", "[TestRebuild][OffParityRebuild]")
{
    // Upstream spaces sparse infill at the object's layer height, also across a range printed at 0.1 mm.
    DynamicPrintConfig config = off_parity_base_config();
    config.set_key_value("sparse_infill_density", new ConfigOptionPercent(40.));
    config.set_key_value("sparse_infill_line_width", new ConfigOptionFloatOrPercent(0.45, false));
    config.set_key_value("top_shell_layers", new ConfigOptionInt(0));
    config.set_key_value("bottom_shell_layers", new ConfigOptionInt(0));
    Model model;
    add_parts(model, "off-sparse-spacing", {{Vec3d(50., 50., 6.), Vec3d::Zero(), 1}});
    DynamicPrintConfig range;
    range.set_key_value("layer_height", new ConfigOptionFloat(0.1));
    model.objects.front()->layer_config_ranges[{2.0, 4.0}].assign_config(std::move(range));
    Print print;
    apply_silent(model, print, config);
    REQUIRE(print.validate().string.empty());
    REQUIRE_NOTHROW(print.process());
    const auto moves = Slic3r::Emitted::emitted_moves(Slic3r::Test::gcode(print));
    std::map<int, double> length_by_layer;
    Vec3f previous = Vec3f::Zero();
    for (const auto &move : moves) {
        if (move.type == EMoveType::Extrude && move.extrusion_role == erInternalInfill)
            length_by_layer[int(std::lround(move.position.z() * 1000.))] += (move.position - previous).head<2>().norm();
        previous = move.position;
    }
    double thick = 0., thin = 0.;
    size_t thick_layers = 0, thin_layers = 0;
    for (const auto &[z, length] : length_by_layer) {
        if (z > 600 && z <= 1800) { thick += length; ++thick_layers; }
        else if (z > 2400 && z <= 3600) { thin += length; ++thin_layers; }
    }
    REQUIRE(thick_layers >= 4);
    REQUIRE(thin_layers >= 8);
    const double ratio = (thin / thin_layers) / (thick / thick_layers);
    CAPTURE(thick_layers, thin_layers, ratio);
    CHECK_THAT(ratio, Catch::Matchers::WithinAbs(1., 0.02));
}

TEST_CASE("Off parity: a tool on a fine nozzle may lay the tower's 0.2 mm bridge road", "[TestRebuild][OffParityRebuild]")
{
    // Upstream lets every tool thicken a floating tower area with a 0.2 mm road. A mode holds back a tool
    // whose nozzle cannot lay 0.2 mm layers (here the 0.2 mm nozzle, at most 0.14 mm).
    const bool mode = GENERATE(false, true);
    CAPTURE(mode);
    DynamicPrintConfig dynamic = off_parity_tower_config();
    dynamic.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    dynamic.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.12});
    dynamic.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.42});
    if (mode)
        dynamic.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::BodySplit));
    mixed_nozzle_fixtures::fill_per_filament_values(dynamic);
    PrintConfig config;
    config.apply(dynamic, true);
    const std::vector<unsigned int> used{0, 1};
    std::vector<MultiNozzleUtils::NozzleInfo> nozzle_list(2);
    for (int e = 0; e < 2; ++e) {
        nozzle_list[size_t(e)].diameter    = e == 0 ? "0.2" : "0.6";
        nozzle_list[size_t(e)].volume_type = nvtStandard;
        nozzle_list[size_t(e)].extruder_id = e;
        nozzle_list[size_t(e)].group_id    = e;
    }
    const auto group = MultiNozzleUtils::LayeredNozzleGroupResult::create(std::vector<int>{0, 1}, nozzle_list, used);
    REQUIRE(group.has_value());
    WipeTower tower(config, 0, Vec3d::Zero(), 0, 3.f, used);
    tower.set_nozzle_group_result(*group);
    for (size_t idx = 0; idx < used.size(); ++idx)
        tower.set_extruder(idx, config);
    CHECK(tower.tool_can_thicken(0) == !mode);
    CHECK(tower.tool_can_thicken(1));
}

TEST_CASE("Off parity: after a nozzle change the tool unretracts at its first road", "[TestRebuild][OffParityRebuild]")
{
    // Upstream unretracts where the arriving tool starts printing. A mode first travels into the tool's
    // sparse infill, clear of the walls, and unretracts there ("Travel to first infill point").
    DynamicPrintConfig config = off_parity_tower_config();
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.4, 0.6});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.08, 0.12});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.28, 0.42});
    config.set_key_value("gcode_comments", new ConfigOptionBool(true));
    Model model;
    Print print;
    const std::string gcode = off_gcode(model, print, config, false, 6.);
    REQUIRE(print.has_wipe_tower());
    REQUIRE(gcode.find("NOZZLE_CHANGE_START") != std::string::npos);
    CHECK(gcode.find("Travel to first infill point") == std::string::npos);
}
