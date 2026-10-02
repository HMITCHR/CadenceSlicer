#include <catch2/catch_all.hpp>

#include "mixed_nozzle_harness.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/GCode/ConflictChecker.hpp"
#include "libslic3r/GCode/WipeTower.hpp"
#include "libslic3r/MultiNozzleUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Slicing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <new>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace mixed_nozzle_fixtures;

namespace {
using Move = GCodeProcessorResult::MoveVertex;

enum class Mode { Off, Feature, Body };

const char *mode_name(Mode mode)
{
    return mode == Mode::Off ? "Off" : mode == Mode::Feature ? "Feature Split" : "Body Split";
}

struct SupportCase {
    const char          *name;
    SupportType          type;
    SupportMaterialStyle style;
};

// Filaments 1 and 3 sit on the left 0.2 nozzle, 2 and 4 on the right 0.8 nozzle.
struct FilamentCase {
    const char *name;
    int         base;
    int         interface_filament; // 0 = Default
    const char *interface_type = "PLA";
};

const std::vector<int> k_filament_map{1, 2, 1, 2};

// A column with a low arm on the right and a high arm on the left. The left arm's support
// rises past the right arm's contact, so its body bands span another island's top contact.
TriangleMesh two_arm_mesh()
{
    return extruded_xz_outline({{0., 0.}, {8., 0.}, {8., 3.}, {16., 3.}, {16., 4.2}, {8., 4.2},
                                {8., 8.}, {-8., 8.}, {-8., 6.6}, {0., 6.6}},
                               12.);
}

DynamicPrintConfig h2d_support_config(Mode mode, double fine, double coarse, int ratio, double top_z,
                                      const SupportCase &support, const FilamentCase &filaments)
{
    DynamicPrintConfig config = coarse_base_support_config(MixedNozzleSlicingMode::FeatureSplit);
    config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(
        mode == Mode::Off ? MixedNozzleSlicingMode::Off :
        mode == Mode::Feature ? MixedNozzleSlicingMode::FeatureSplit : MixedNozzleSlicingMode::BodySplit));
    config.set_key_value("printable_area", new ConfigOptionPoints{Vec2d(0., 0.), Vec2d(320., 0.), Vec2d(320., 320.), Vec2d(0., 320.)});
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.8});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.16});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.56});
    config.set_key_value("layer_height", new ConfigOptionFloat(fine));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(fine));
    config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(coarse));
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{ratio});
    config.set_key_value("mixed_nozzle_process_nozzle_diameters", new ConfigOptionFloats{0.2, 0.8});

    config.set_key_value("filament_diameter", new ConfigOptionFloats(4, 1.75));
    config.set_key_value("filament_map", new ConfigOptionInts(k_filament_map));
    config.set_key_value("flush_volumes_matrix", new ConfigOptionFloats(32, 0.));
    config.set_key_value("filament_colour", new ConfigOptionStrings{"#FF0000", "#0000FF", "#00FF00", "#FFFF00"});
    config.set_key_value("filament_self_index", new ConfigOptionInts{1, 2, 3, 4});
    config.set_key_value("filament_volume_map", new ConfigOptionInts(4, 0));
    config.set_key_value("filament_start_gcode", new ConfigOptionStrings(4, ""));
    config.set_key_value("filament_end_gcode", new ConfigOptionStrings(4, ""));
    config.set_key_value("filament_type", new ConfigOptionStrings(4, "PLA"));
    if (filaments.interface_filament > 0)
        config.option<ConfigOptionStrings>("filament_type")->values[size_t(filaments.interface_filament - 1)] = filaments.interface_type;
    config.set_key_value("filament_adhesiveness_category", new ConfigOptionInts(4, 100));
    config.set_key_value("filament_soluble", new ConfigOptionBools(4, false));
    config.set_key_value("filament_is_support", new ConfigOptionBools(4, false));

    // Model roles: fine filament 1; Feature Split sends sparse infill to the coarse nozzle, Body
    // Split takes each part's own filament.
    const int role_filament = mode == Mode::Body ? 0 : 1;
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id", "internal_solid_filament_id",
                            "top_surface_filament_id", "bottom_surface_filament_id"})
        config.set_key_value(key, new ConfigOptionInt(role_filament));
    config.set_key_value("sparse_infill_filament_id", new ConfigOptionInt(mode == Mode::Feature ? 2 : role_filament));
    config.set_key_value("sparse_infill_density", new ConfigOptionPercent(15.));

    config.set_key_value("enable_support", new ConfigOptionBool(true));
    config.set_key_value("support_type", new ConfigOptionEnum<SupportType>(support.type));
    config.set_key_value("support_style", new ConfigOptionEnum<SupportMaterialStyle>(support.style));
    config.set_key_value("support_filament", new ConfigOptionInt(filaments.base));
    config.set_key_value("support_interface_filament", new ConfigOptionInt(filaments.interface_filament));
    config.set_key_value("support_top_z_distance", new ConfigOptionFloat(top_z));
    config.set_key_value("support_bottom_z_distance", new ConfigOptionFloat(top_z));
    config.set_key_value("support_interface_top_layers", new ConfigOptionInt(2));
    config.set_key_value("support_interface_bottom_layers", new ConfigOptionInt(0));
    // Two walls on every tree branch, as tall or thick trees get, so tree support writes collections.
    config.set_key_value("tree_support_wall_count", new ConfigOptionInt(2));
    fill_per_filament_values(config);
    return config;
}

void load_two_arm_model(Model &model, Print &print, const DynamicPrintConfig &config, Mode mode, double fine, double coarse)
{
    ModelObject *object = model.add_object();
    object->name = "two-arm-overhang";
    ModelVolume *part = object->add_volume(two_arm_mesh(), ModelVolumeType::MODEL_PART, false);
    if (mode == Mode::Body) {
        part->config.set_key_value("extruder", new ConfigOptionInt(1));
        part->config.set_key_value("regional_layer_height", new ConfigOptionFloat(fine));
        ModelVolume *body = object->add_volume(make_cube(8., 12., 6.), ModelVolumeType::MODEL_PART, false);
        body->set_offset(Vec3d(24., 0., 0.));
        body->config.set_key_value("extruder", new ConfigOptionInt(2));
        body->config.set_key_value("regional_layer_height", new ConfigOptionFloat(coarse));
    }
    object->add_instance();
    object->instances.front()->set_offset(Vec3d(60., 60., 0.));
    object->ensure_on_bed();
    print.apply(model, config);
    print.set_status_silent();
}

// Top of the support under the model inside [x_lo, x_hi], the model's underside there, and the
// role of the topmost support road.
struct Gap {
    double       support_top   = 0.;
    double       object_bottom = std::numeric_limits<double>::max();
    ExtrusionRole top_role     = erNone;
};

Gap support_gap(const std::vector<Move> &moves, double x_lo, double x_hi)
{
    Gap gap;
    for (const Move &move : moves)
        if (CadenceTest::model_road(move) && move.position.x() > x_lo && move.position.x() < x_hi)
            gap.object_bottom = std::min(gap.object_bottom, double(move.position.z()) - double(move.height));
    for (const Move &move : moves)
        if (move.type == EMoveType::Extrude &&
            (move.extrusion_role == erSupportMaterial || move.extrusion_role == erSupportMaterialInterface) &&
            move.position.x() > x_lo && move.position.x() < x_hi &&
            double(move.position.z()) < gap.object_bottom + 1e-4 &&
            double(move.position.z()) > gap.support_top + 1e-4) {
            gap.support_top = double(move.position.z());
            gap.top_role    = move.extrusion_role;
        }
    return gap;
}

// Slices one case to G-code and returns what is wrong with it, or an empty string.
std::string run_case(Mode mode, double fine, double coarse, int ratio, double top_z,
                     const SupportCase &support, const FilamentCase &filaments)
{
    std::ostringstream what;
    what << mode_name(mode) << " / " << support.name << " / " << filaments.name << ": ";
    DynamicPrintConfig config = h2d_support_config(mode, fine, coarse, ratio, top_z, support, filaments);
    CadenceTest::Scene scene;
    scene.config   = config;
    double left    = 0.;
    scene.populate = [mode, fine, coarse, &left](Model &model, Print &print, const DynamicPrintConfig &cfg) {
        load_two_arm_model(model, print, cfg, mode, fine, coarse);
        left = model.objects.front()->instance_bounding_box(0).min.x();
    };
    CadenceTest::Facts facts;
    try {
        facts = CadenceTest::slice(scene);
    } catch (const std::exception &error) {
        return what.str() + "threw " + error.what();
    }
    if (!facts.refusal.string.empty())
        return what.str() + "refused: " + facts.refusal.string;
    if (facts.gcode.empty())
        return what.str() + "no G-code";

    // Under a mode a Default interface prints with the base filament.
    const int interface_filament = filaments.interface_filament > 0 ? filaments.interface_filament :
                                   mode != Mode::Off ? filaments.base : 0;
    // Each nozzle's layer height range, as h2d_support_config sets it.
    const double nozzle_min[2] = {0.04, 0.16};
    const double nozzle_max[2] = {0.14, 0.56};
    size_t base = 0, interface_roads = 0, base_wrong = 0, interface_wrong = 0, out_of_range = 0;
    for (const Move &move : facts.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        if (move.extrusion_role == erSupportMaterial) {
            ++base;
            // Under a mode, body the coarse base nozzle cannot lay is laid by the interface nozzle in a
            // filament of the base's material there, never an interface filament of another material.
            const bool fine_body = mode != Mode::Off && filaments.interface_filament > 0 &&
                k_filament_map[filaments.base - 1] != k_filament_map[filaments.interface_filament - 1] &&
                int(move.physical_tool_id) == k_filament_map[filaments.interface_filament - 1] - 1 &&
                (int(move.extruder_id) != filaments.interface_filament - 1 || std::string(filaments.interface_type) == "PLA");
            base_wrong += ! fine_body && (int(move.extruder_id) != filaments.base - 1 ||
                          int(move.physical_tool_id) != k_filament_map[filaments.base - 1] - 1);
        } else if (move.extrusion_role == erSupportMaterialInterface) {
            ++interface_roads;
            if (interface_filament > 0)
                interface_wrong += int(move.extruder_id) != interface_filament - 1 ||
                                   int(move.physical_tool_id) != k_filament_map[interface_filament - 1] - 1;
        } else
            continue;
        // Under a mode every support road above the first layer is a height its nozzle can lay.
        const int nozzle = int(move.physical_tool_id);
        if (mode != Mode::Off && nozzle >= 0 && nozzle < 2 && double(move.position.z()) > fine + 1e-3 &&
            (double(move.height) < nozzle_min[nozzle] - 1e-3 || double(move.height) > nozzle_max[nozzle] + 1e-3)) {
            if (out_of_range < 3)
                what << "[Z " << move.position.z() << ", height " << move.height << ", role " << int(move.extrusion_role)
                     << ", nozzle " << nozzle + 1 << "] ";
            ++out_of_range;
        }
    }
    bool ok = true;
    if (base == 0 || interface_roads == 0 || base_wrong != 0 || interface_wrong != 0) {
        what << "base " << base << " (" << base_wrong << " on the wrong filament), interface " << interface_roads
             << " (" << interface_wrong << " on the wrong filament); ";
        ok = false;
    }
    if (out_of_range != 0) {
        what << out_of_range << " support roads outside their nozzle's layer height range; ";
        ok = false;
    }
    // Under each arm the topmost support road is interface and sits the top Z distance below the
    // model, rounded up to at most one fine layer.
    for (const auto &[name, lo, hi] : {std::tuple<const char *, double, double>{"left arm", left + 1., left + 7.},
                                        {"right arm", left + 18., left + 23.}}) {
        const Gap gap = support_gap(facts.moves, lo, hi);
        const double distance = gap.object_bottom - gap.support_top;
        if (gap.support_top <= 0. || gap.top_role != erSupportMaterialInterface ||
            distance < top_z - 1e-3 || distance > top_z + fine + 1e-3) {
            what << name << " support top " << gap.support_top << " role " << int(gap.top_role) << " under model at "
                 << gap.object_bottom << "; ";
            ok = false;
        }
    }
    return ok ? std::string() : what.str();
}

std::string label_of(Mode mode, const SupportCase &support, const FilamentCase &filaments)
{
    return std::string(mode_name(mode)) + " / " + support.name + " / " + filaments.name;
}

void count_support_roles(const ExtrusionEntity *entity, size_t &base, size_t &interface_roads)
{
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection*>(entity)) {
        for (const ExtrusionEntity *child : collection->entities)
            count_support_roles(child, base, interface_roads);
        return;
    }
    base            += entity->role() == erSupportMaterial;
    interface_roads += entity->role() == erSupportMaterialInterface;
}

// Generates support without exporting G-code; returns what is wrong, or an empty string.
std::string process_case(Mode mode, double fine, double coarse, int ratio, double top_z,
                         const SupportCase &support, const FilamentCase &filaments)
{
    const std::string what = label_of(mode, support, filaments) + ": ";
    DynamicPrintConfig config = h2d_support_config(mode, fine, coarse, ratio, top_z, support, filaments);
    Model model;
    Print print;
    print.is_BBL_printer() = true;
    load_two_arm_model(model, print, config, mode, fine, coarse);
    if (const StringObjectException refusal = print.validate(); !refusal.string.empty())
        return what + "refused: " + refusal.string;
    try {
        print.process();
    } catch (const std::exception &error) {
        return what + "threw " + error.what();
    }
    size_t base = 0, interface_roads = 0;
    for (const SupportLayer *layer : print.objects().front()->support_layers())
        count_support_roles(&layer->support_fills, base, interface_roads);
    if (base == 0 || interface_roads == 0)
        return what + "base " + std::to_string(base) + ", interface " + std::to_string(interface_roads);
    return {};
}

const SupportCase k_organic{"tree organic", stTreeAuto, smsTreeOrganic};
const SupportCase k_hybrid{"tree hybrid", stTreeAuto, smsTreeHybrid};
const SupportCase k_slim{"tree slim", stTreeAuto, smsTreeSlim};
const SupportCase k_normal{"normal auto", stNormalAuto, smsDefault};

// Tree support whose base is on the coarse nozzle is refused (SRL-F14 / SRL-A53) until trees are cut per
// band. The tree cases below count that refusal as their expected outcome; the refusal case further
// down asserts it.
bool tree_refused(const std::string &outcome)
{
    return outcome.find("refused: [SRL-F14]") != std::string::npos || outcome.find("refused: [SRL-A53]") != std::string::npos;
}

// What a case is expected to do. Every case prints except an interface on a larger nozzle than the
// base under a mode, which is refused: the interface is what touches the model.
enum class Expect { Slice, Refusal };

Expect expected(Mode mode, const FilamentCase &filaments)
{
    if (mode == Mode::Off || filaments.interface_filament == 0)
        return Expect::Slice;
    const bool base_coarse      = k_filament_map[filaments.base - 1] == 2;
    const bool interface_coarse = k_filament_map[filaments.interface_filament - 1] == 2;
    return !base_coarse && interface_coarse ? Expect::Refusal : Expect::Slice;
}

std::string check_expected(Mode mode, const FilamentCase &filaments, const std::string &outcome)
{
    if (tree_refused(outcome))
        return {};
    if (expected(mode, filaments) == Expect::Slice)
        return outcome;
    return outcome.find(mode == Mode::Body ? "refused: [SRL-A29]" : "refused: [SRL-F01]") != std::string::npos ?
        std::string() : "expected a refusal, got: " + (outcome.empty() ? std::string("a slice") : outcome);
}
} // namespace

TEST_CASE("Feature Split tree support with a coarse base and fine interface is refused for now; mode Off slices it",
          "[TestRebuild][Support]")
{
    // The layout: fine 0.08 on the 0.2, coarse 0.56 on the 0.8, base on the 0.8 and
    // interface on the 0.2, trees under two arms at different heights.
    const FilamentCase base_coarse{"base coarse, interface fine", 2, 3};
    for (const SupportCase &support : {k_organic, k_hybrid, k_slim}) {
        const std::string failure = process_case(Mode::Feature, 0.08, 0.56, 7, 0.24, support, base_coarse);
        INFO(failure);
        CHECK((failure.empty() || tree_refused(failure)));
    }
    // The same filaments with the mode off, both ways round.
    for (const FilamentCase &filaments : {base_coarse, FilamentCase{"base fine, interface coarse", 3, 4}}) {
        const std::string failure = process_case(Mode::Off, 0.08, 0.56, 7, 0.24, k_organic, filaments);
        INFO(failure);
        CHECK((failure.empty() || tree_refused(failure)));
    }
}

TEST_CASE("Feature Split at a 0.08 mm fine layer prints normal support to G-code; tree support on a coarse base is refused for now",
          "[TestRebuild][Support]")
{
    // Coarse base PLA on the right 0.8, fine interface PETG on the left 0.2, at 0.56 and 0.24 coarse.
    const FilamentCase layout{"base coarse PLA, interface fine PETG", 2, 3, "PETG"};
    for (const auto &[coarse, ratio] : {std::pair<double, int>{0.56, 7}, {0.24, 3}})
        for (const SupportCase &support : {k_organic, k_hybrid, k_slim, k_normal}) {
            const std::string failure = run_case(Mode::Feature, 0.08, coarse, ratio, 0.24, support, layout);
            INFO("coarse " << coarse << ": " << failure);
            CHECK((failure.empty() || tree_refused(failure)));
        }
}

TEST_CASE("Feature Split tree support with the interface on the coarse base filament is refused for now; mode Off slices it",
          "[TestRebuild][Support]")
{
    const std::string failure = process_case(Mode::Feature, 0.08, 0.56, 7, 0.24, k_organic, FilamentCase{"interface = base", 2, 2});
    INFO(failure);
    CHECK((failure.empty() || tree_refused(failure)));
    const std::string off = process_case(Mode::Off, 0.08, 0.56, 7, 0.24, k_organic, FilamentCase{"interface Default", 2, 0});
    INFO(off);
    CHECK(off.empty());
}

TEST_CASE("Support matrix on an H2D 0.2/0.8 slices to G-code; tree support on a coarse base is refused for now", "[TestRebuild][SupportMatrix]")
{
    const std::vector<FilamentCase> filament_cases{
        {"interface Default", 2, 0},
        {"interface = base", 2, 2},
        {"interface on the base nozzle", 2, 4},
        {"base coarse, interface fine", 2, 3},
        {"base fine, interface coarse", 3, 4},
    };
    std::vector<std::string> failures;
    size_t cases = 0;
    for (Mode mode : {Mode::Off, Mode::Feature, Mode::Body})
        for (const SupportCase &support : {k_normal, k_slim, k_hybrid, k_organic})
            for (const FilamentCase &filaments : filament_cases) {
                ++cases;
                const std::string outcome = run_case(mode, 0.10, 0.30, 3, 0.20, support, filaments);
                if (std::string failure = check_expected(mode, filaments, outcome); !failure.empty() && !tree_refused(failure))
                    failures.push_back(std::string(mode_name(mode)) + " / " + support.name + " / " + filaments.name +
                                       ": " + failure);
            }
    std::ostringstream report;
    for (const std::string &failure : failures)
        report << failure << '\n';
    INFO(report.str());
    CAPTURE(cases, failures.size());
    CHECK(failures.empty());
}


namespace {
// The prime tower as the automatic tower setup leaves it: auto-sized pad under a 35 mm width cap.
void set_automatic_tower(DynamicPrintConfig &config)
{
    config.set_key_value("enable_prime_tower", new ConfigOptionBool(true));
    config.set_key_value("wipe_tower_no_sparse_layers", new ConfigOptionBool(true));
    config.set_key_value("mixed_nozzle_auto_pad", new ConfigOptionBool(true));
    config.set_key_value("prime_tower_width", new ConfigOptionFloat(35.));
    config.set_key_value("wipe_tower_wall_type", new ConfigOptionEnum<WipeTowerWallType>(wtwRib));
    config.set_key_value("wipe_tower_rib_width", new ConfigOptionFloat(8.));
    config.set_key_value("prime_tower_brim_width", new ConfigOptionFloat(-1.));
    config.set_key_value("prime_volume_mode", new ConfigOptionEnum<PrimeVolumeMode>(pvmDefault));
    config.set_key_value("wipe_tower_x", new ConfigOptionFloats{200.});
    config.set_key_value("wipe_tower_y", new ConfigOptionFloats{200.});
}

// The L test part: a 20 mm column, 50 mm tall, with a 40 mm arm whose flat underside is at 40 mm.
// Support under the arm stands on the bed. Fine PLA (1) prints the part on the 0.2, sparse infill
// and the support base use coarse PLA (2) on the 0.8, and the interface is PETG (3) on the 0.2.
// The project as the app saved it, with the automatic tower put back.
DynamicPrintConfig app_three_filament_config(Mode mode, double fine, double coarse, int ratio, const SupportCase &support)
{
    DynamicPrintConfig config;
    config.load_from_ini(std::string(TEST_DATA_DIR) + "/h2d-three-filament.ini", ForwardCompatibilitySubstitutionRule::Enable);
    REQUIRE(config.has("printable_area"));
    config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(
        mode == Mode::Off ? MixedNozzleSlicingMode::Off : MixedNozzleSlicingMode::FeatureSplit));
    config.set_key_value("layer_height", new ConfigOptionFloat(fine));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(fine));
    config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(coarse));
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{ratio});
    config.set_key_value("support_type", new ConfigOptionEnum<SupportType>(support.type));
    config.set_key_value("support_style", new ConfigOptionEnum<SupportMaterialStyle>(support.style));
    config.set_key_value("wipe_tower_wall_type", new ConfigOptionEnum<WipeTowerWallType>(wtwRib));
    config.set_key_value("mixed_nozzle_auto_pad", new ConfigOptionBool(true));
    config.set_key_value("prime_tower_width", new ConfigOptionFloat(35.));
    return config;
}

// The L test part: a 20 mm column, 50 mm tall, with a 40 mm arm whose flat underside is at 40 mm.
// Support under the arm stands on the bed. Fine PLA prints the part on the 0.2, sparse infill and
// the support base use coarse PLA on the 0.8, and the interface is PETG on the 0.2. With
// app_settings the project is the one the app saved (filament 2 PETG, 3 coarse PLA), otherwise
// the synthetic H2D (filament 2 coarse PLA, 3 PETG).
std::string run_l_part_case(Mode mode, double fine, double coarse, int ratio, const SupportCase &support,
                            bool app_settings = false)
{
    const FilamentCase filaments = app_settings ?
        FilamentCase{"app project", 3, 2, "PETG"} : FilamentCase{"fine PLA, coarse PLA base, PETG interface on fine", 2, 3, "PETG"};
    std::ostringstream what;
    what << mode_name(mode) << " / " << support.name << " / " << filaments.name << " / " << fine << "/" << coarse << ": ";
    const double top_z = 0.;
    DynamicPrintConfig config;
    if (app_settings)
        config = app_three_filament_config(mode, fine, coarse, ratio, support);
    else {
        config = h2d_support_config(mode, fine, coarse, ratio, top_z, support, filaments);
        config.set_key_value("support_interface_bottom_layers", new ConfigOptionInt(2));
        set_automatic_tower(config);
        // Flush volumes and prime volumes as the H2D profile gives these filaments.
        {
            const double gui[32] = {0, 101, 66, 157, 145, 0, 149, 178, 57, 104, 0, 167, 90, 99, 101, 0,
                                    0, 354, 237, 540, 500, 0, 515, 611, 208, 363, 0, 573, 317, 347, 353, 0};
            const int profile_index[4] = {0, 2, 1, 3}; // fine PLA, coarse PLA, PETG, spare PETG
            std::vector<double> flush(32, 0.);
            for (int nozzle = 0; nozzle < 2; ++nozzle)
                for (int from = 0; from < 4; ++from)
                    for (int to = 0; to < 4; ++to)
                        flush[size_t(nozzle * 16 + from * 4 + to)] = gui[nozzle * 16 + profile_index[from] * 4 + profile_index[to]];
            config.set_key_value("flush_volumes_matrix", new ConfigOptionFloats(flush));
            config.set_key_value("flush_multiplier", new ConfigOptionFloats{0.3, 1.});
            config.set_key_value("filament_prime_volume", new ConfigOptionFloats{45., 30., 45., 30.});
            config.set_key_value("prime_volume", new ConfigOptionFloat(45.));
            config.set_key_value("raft_first_layer_expansion", new ConfigOptionFloat(2.));
            config.set_key_value("support_line_width", new ConfigOptionFloatOrPercent(0.22, false));
        }
    }
    const BoundingBoxf bed(config.option<ConfigOptionPoints>("printable_area")->values);
    CadenceTest::Scene scene;
    scene.config = config;
    double arm_lo = 0., arm_hi = 0.;
    scene.populate = [&arm_lo, &arm_hi](Model &model, Print &print, const DynamicPrintConfig &cfg) {
        ModelObject *object = model.add_object();
        object->name = "l-part";
        object->add_volume(overhang_shelf_mesh(20., 60., 40., 50.), ModelVolumeType::MODEL_PART, false);
        object->add_instance();
        object->instances.front()->set_offset(Vec3d(60., 60., 0.));
        object->ensure_on_bed();
        const BoundingBoxf3 box = object->instance_bounding_box(0);
        arm_lo = box.min.x() + 20.;
        arm_hi = box.max.x();
        print.apply(model, cfg);
        print.set_status_silent();
    };
    CadenceTest::Facts facts;
    try {
        facts = CadenceTest::slice(scene);
    } catch (const std::exception &error) {
        return what.str() + "threw " + error.what();
    }
    if (!facts.refusal.string.empty())
        return what.str() + "refused: " + facts.refusal.string;
    if (facts.gcode.empty())
        return what.str() + "no G-code";

    const int petg = filaments.interface_filament - 1;
    const double arm_bottom = 40.;
    size_t first_layer_petg = 0, petg_outside_band = 0, petg_interface = 0, first_layer_base = 0, tower_roads = 0,
           tower_off_bed = 0;
    double lowest_petg = std::numeric_limits<double>::max();
    for (const Move &move : facts.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        const double z = double(move.position.z());
        if (move.extrusion_role == erWipeTower) {
            ++tower_roads;
            tower_off_bed += move.position.x() < bed.min.x() || move.position.x() > bed.max.x() ||
                             move.position.y() < bed.min.y() || move.position.y() > bed.max.y();
            continue;
        }
        // On the bed: the first layer, or under a mode the first band the coarse nozzle lays there.
        const bool first_layer = z < (mode == Mode::Off ? fine : coarse) + 1e-3;
        // The support's own first layer is at the first layer's height. Under a mode the fine nozzle
        // may lay it, in a filament of the base's material, never the interface filament.
        if (z < fine + 1e-3 && move.extrusion_role == erSupportMaterial && int(move.extruder_id) != petg)
            ++first_layer_base;
        if (int(move.extruder_id) != petg)
            continue;
        if (first_layer)
            ++first_layer_petg;
        // The interface filament belongs in the top contact band right under the arm.
        const bool in_band = move.extrusion_role == erSupportMaterialInterface && z > arm_bottom - 1. &&
                             z < arm_bottom + 1e-3 && move.position.x() > arm_lo - 3. && move.position.x() < arm_hi + 3.;
        if (in_band)
            ++petg_interface;
        else {
            if (petg_outside_band < 3)
                what << "[PETG role " << int(move.extrusion_role) << " at " << move.position.x() << ", "
                     << move.position.y() << ", " << z << "] ";
            ++petg_outside_band;
            lowest_petg = std::min(lowest_petg, z);
        }
    }
    bool ok = true;
    if (first_layer_petg != 0) {
        what << first_layer_petg << " interface-filament roads on the bed; ";
        ok = false;
    }
    if (first_layer_base == 0) {
        what << "no support base on the bed; ";
        ok = false;
    }
    if (petg_outside_band != 0) {
        what << petg_outside_band << " interface-filament roads outside the top contact band (lowest at Z "
             << lowest_petg << "); ";
        ok = false;
    }
    if (petg_interface == 0) {
        what << "no interface under the arm; ";
        ok = false;
    }
    if (tower_roads == 0 || tower_off_bed != 0) {
        what << "tower roads " << tower_roads << ", off the bed " << tower_off_bed << "; ";
        ok = false;
    }
    return ok ? std::string() : what.str();
}
} // namespace

TEST_CASE("Three filaments with the automatic tower: PETG interface on the fine nozzle under a coarse PLA base slices; tree support is refused for now",
          "[TestRebuild][SupportMatrix]")
{
    std::vector<std::string> failures;
    for (Mode mode : {Mode::Off, Mode::Feature})
        for (const auto &[fine, coarse, ratio] : {std::tuple<double, double, int>{0.10, 0.30, 3}, {0.08, 0.56, 7}})
            for (const SupportCase &support : {k_normal, k_organic}) {
                // Mode Off runs once per support type, as a reference.
                if (mode == Mode::Off && fine < 0.1 - 1e-6)
                    continue;
                for (bool app_settings : {false, true})
                    if (std::string failure = run_l_part_case(mode, fine, coarse, ratio, support, app_settings); !failure.empty() && !tree_refused(failure))
                        failures.push_back(failure);
            }
    std::ostringstream report;
    for (const std::string &failure : failures)
        report << failure << '\n';
    INFO(report.str());
    CHECK(failures.empty());
}

namespace {
// Shapes whose support stands on the part or on the bed under a curved underside.
TriangleMesh c_shape_mesh()
{
    // A plate on the bed, a column, and an arm over the plate.
    return extruded_xz_outline({{0., 0.}, {40., 0.}, {40., 6.}, {15., 6.}, {15., 16.}, {40., 16.}, {40., 20.}, {0., 20.}}, 20.);
}

TriangleMesh hull_mesh()
{
    // A ball on the bed, like the underside of a hull or a bunny: a small first layer and
    // support rising from the bed under a curve.
    TriangleMesh mesh = make_sphere(15., PI / 24.);
    mesh.scale(Vec3f(1.6f, 1.f, 1.f));
    return mesh;
}

std::string slice_shape(const char *shape_name, TriangleMesh mesh, double fine, double coarse, int ratio,
                        const SupportCase &support, const FilamentCase &filaments)
{
    std::ostringstream what;
    what << shape_name << " / " << support.name << " / " << filaments.name << " / " << fine << "/" << coarse << ": ";
    DynamicPrintConfig config = h2d_support_config(Mode::Feature, fine, coarse, ratio, 0.2, support, filaments);
    set_automatic_tower(config);
    CadenceTest::Scene scene;
    scene.config   = config;
    scene.populate = [&mesh, shape_name](Model &model, Print &print, const DynamicPrintConfig &cfg) {
        ModelObject *object = model.add_object();
        object->name = shape_name;
        object->add_volume(std::move(mesh), ModelVolumeType::MODEL_PART, false);
        object->add_instance();
        object->instances.front()->set_offset(Vec3d(80., 80., 0.));
        object->ensure_on_bed();
        print.apply(model, cfg);
        print.set_status_silent();
    };
    CadenceTest::Facts facts;
    try {
        facts = CadenceTest::slice(scene);
    } catch (const std::exception &error) {
        return what.str() + "threw " + error.what();
    }
    if (!facts.refusal.string.empty())
        return what.str() + "refused: " + facts.refusal.string;
    size_t support_roads = 0;
    for (const Move &move : facts.moves)
        support_roads += move.type == EMoveType::Extrude &&
                         (move.extrusion_role == erSupportMaterial || move.extrusion_role == erSupportMaterialInterface);
    return support_roads == 0 ? what.str() + "no support" : std::string();
}
} // namespace

TEST_CASE("Feature Split supports slice on shapes that stand on the part or under a curve; tree support on a coarse base is refused for now",
          "[TestRebuild][SupportMatrix]")
{
    std::vector<std::string> failures;
    for (const auto &[fine, coarse, ratio] : {std::tuple<double, double, int>{0.10, 0.30, 3}, {0.08, 0.56, 7}})
        for (const SupportCase &support : {k_organic, k_hybrid, k_normal})
            for (const FilamentCase &filaments : {FilamentCase{"interface = base", 2, 2},
                                                  FilamentCase{"PETG interface on fine", 2, 3, "PETG"}}) {
                if (std::string failure = slice_shape("C shape", c_shape_mesh(), fine, coarse, ratio, support, filaments);
                    !failure.empty() && !tree_refused(failure))
                    failures.push_back(failure);
                if (std::string failure = slice_shape("hull", hull_mesh(), fine, coarse, ratio, support, filaments);
                    !failure.empty() && !tree_refused(failure))
                    failures.push_back(failure);
            }
    // The handy Benchy and bunny, whose trees stand on the deck and rise under curves.
    for (const char *file : {"3DBenchy.drc", "Stanford_Bunny.drc"}) {
        DynamicPrintConfig config = h2d_support_config(Mode::Feature, 0.10, 0.30, 3, 0.2, k_organic,
                                                       FilamentCase{"interface = base", 2, 2});
        set_automatic_tower(config);
        config.set_key_value("prime_tower_width", new ConfigOptionFloat(60.));
        config.set_key_value("printable_height", new ConfigOptionFloat(325.));
        CadenceTest::Scene scene;
        scene.config   = config;
        scene.populate = [file](Model &model, Print &print, const DynamicPrintConfig &cfg) {
            model = Model::read_from_file(std::string(PROFILES_DIR) + "/../handy_models/" + file);
            for (ModelObject *object : model.objects) {
                if (object->instances.empty())
                    object->add_instance();
                // The bunny, at the size it imports into the app.
                if (object->instance_bounding_box(0).size().z() > 118.)
                    object->scale_to_fit(Vec3d(120., 120., 118.));
                object->instances.front()->set_offset(Vec3d(40., 40., 0.));
                object->ensure_on_bed();
            }
            print.apply(model, cfg);
            print.set_status_silent();
        };
        try {
            const CadenceTest::Facts facts = CadenceTest::slice(scene);
            if (!facts.refusal.string.empty() && !tree_refused("refused: " + facts.refusal.string))
                failures.push_back(std::string(file) + " refused: " + facts.refusal.string);
        } catch (const std::exception &error) {
            failures.push_back(std::string(file) + " threw " + error.what());
        }
    }
    std::ostringstream report;
    for (const std::string &failure : failures)
        report << failure << '\n';
    INFO(report.str());
    CHECK(failures.empty());
}

namespace {
// A bunny-like part: a ball on a foot to one side, its underside curving down to 0.32 mm over the
// bed, so trees stand on the bed and meet the curve a few layers up.
std::vector<TriangleMesh> bunny_like_parts()
{
    TriangleMesh ball = make_sphere(12., PI / 24.);
    ball.scale(Vec3f(1.4f, 1.f, 1.f));
    ball.translate(0.f, 0.f, 12.32f);
    TriangleMesh foot = make_cylinder(3., 3., PI / 24.);
    foot.translate(10.f, 0.f, 0.f);
    return {std::move(foot), std::move(ball)};
}

// A Benchy-like hull bottom: a block whose underside has a 7 x 5 mm pocket 0.3 mm deep, with a deeper
// 3 x 2 mm recess in its middle up to 0.8 mm, so trees stand on the bed inside the pocket and meet a
// contact one layer above the first band.
std::vector<TriangleMesh> benchy_like_parts()
{
    std::vector<TriangleMesh> parts;
    const auto box = [&parts](double x, double y, double z, double dx, double dy, double dz) {
        TriangleMesh mesh = make_cube(dx, dy, dz);
        mesh.translate(float(x), float(y), float(z));
        parts.push_back(std::move(mesh));
    };
    // A 30 x 20 block with a frame around the pocket, then a frame around the recess.
    const auto frame = [&box](double x0, double y0, double w, double h, double outer_w, double outer_h, double z, double dz) {
        box(x0 - (outer_w - w) / 2., y0 - (outer_h - h) / 2., z, outer_w, (outer_h - h) / 2., dz);
        box(x0 - (outer_w - w) / 2., y0 + h, z, outer_w, (outer_h - h) / 2., dz);
        box(x0 - (outer_w - w) / 2., y0, z, (outer_w - w) / 2., h, dz);
        box(x0 + w, y0, z, (outer_w - w) / 2., h, dz);
    };
    frame(11.5, 7.5, 7., 5., 30., 20., 0., 0.8);
    frame(13.5, 9., 3., 2., 7., 5., 0.3, 0.5);
    box(0., 0., 0.8, 30., 20., 8.);
    return parts;
}

// Slices a shape with organic trees in the project the app saved (fine PLA 1
// and PETG interface 2 on the 0.2, coarse PLA base 3 on the 0.8), and returns what the interface
// filament lays on the first layer, tower included, or an empty string.
std::string interface_filament_on_bed(const char *shape_name, const std::function<void(ModelObject&)> &add_parts,
                                      Mode mode, double fine, double coarse, int ratio)
{
    std::ostringstream what;
    what << shape_name << " / " << mode_name(mode) << " / " << fine << "/" << coarse << ": ";
    DynamicPrintConfig config = app_three_filament_config(mode, fine, coarse, ratio, k_organic);
    const int interface_filament = config.opt_int("support_interface_filament");
    REQUIRE(interface_filament != config.opt_int("support_filament"));
    CadenceTest::Scene scene;
    scene.config   = config;
    scene.populate = [&add_parts, shape_name](Model &model, Print &print, const DynamicPrintConfig &cfg) {
        ModelObject *object = model.add_object();
        object->name = shape_name;
        add_parts(*object);
        if (object->instances.empty())
            object->add_instance();
        object->center_around_origin(false);
        object->instances.front()->set_offset(Vec3d(100., 120., 0.));
        object->ensure_on_bed();
        print.apply(model, cfg);
        print.set_status_silent();
    };
    CadenceTest::Facts facts;
    try {
        facts = CadenceTest::slice(scene);
    } catch (const std::exception &error) {
        return what.str() + "threw " + error.what();
    }
    if (!facts.refusal.string.empty())
        return what.str() + "refused: " + facts.refusal.string;
    size_t support = 0, interface_roads = 0, on_bed = 0, tower_on_bed = 0;
    BoundingBoxf box;
    for (const Move &move : facts.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        support += move.extrusion_role == erSupportMaterial;
        const bool in_interface_filament = int(move.extruder_id) == interface_filament - 1;
        interface_roads += in_interface_filament && move.extrusion_role == erSupportMaterialInterface;
        if (!in_interface_filament || double(move.position.z()) > fine + 1e-3)
            continue;
        if (move.extrusion_role == erWipeTower)
            ++tower_on_bed;
        else {
            ++on_bed;
            box.merge(Vec2d(move.position.x(), move.position.y()));
        }
    }
    if (support == 0 || interface_roads == 0)
        return what.str() + "support " + std::to_string(support) + ", interface " + std::to_string(interface_roads);
    if (on_bed == 0 && tower_on_bed == 0)
        return {};
    what << on_bed << " interface-filament roads on the first layer";
    if (on_bed != 0)
        what << " (X " << box.min.x() << ".." << box.max.x() << ", Y " << box.min.y() << ".." << box.max.y() << ")";
    what << ", " << tower_on_bed << " tower roads in it on the first layer";
    return what.str();
}

std::function<void(ModelObject&)> parts_of(std::vector<TriangleMesh> (*make)())
{
    return [make](ModelObject &object) {
        for (TriangleMesh &mesh : make())
            object.add_volume(std::move(mesh), ModelVolumeType::MODEL_PART, false);
    };
}

std::function<void(ModelObject&)> handy_model(const char *file)
{
    return [file](ModelObject &object) {
        Model loaded = Model::read_from_file(std::string(PROFILES_DIR) + "/../handy_models/" + file);
        for (const ModelObject *source : loaded.objects)
            for (const ModelVolume *volume : source->volumes)
                object.add_volume(*volume);
        object.add_instance();
        // The bunny, at the size it imports into the app.
        if (object.instance_bounding_box(0).size().z() > 118.)
            object.scale_to_fit(Vec3d(120., 120., 118.));
    };
}
} // namespace

TEST_CASE("Organic trees under curves: stock keeps the interface filament off the bed; Feature Split refuses the coarse base for now",
          "[TestRebuild][SupportMatrix]")
{
    std::vector<std::string> failures;
    const std::vector<std::pair<const char *, std::function<void(ModelObject&)>>> shapes{
        {"bunny-like ball on a foot", parts_of(bunny_like_parts)},
        {"Benchy-like pocket", parts_of(benchy_like_parts)},
    };
    for (const auto &[name, parts] : shapes) {
        // Stock first: it keeps the interface filament off the bed on these shapes.
        if (std::string failure = interface_filament_on_bed(name, parts, Mode::Off, 0.10, 0.30, 3); !failure.empty() && !tree_refused(failure))
            failures.push_back(failure);
        for (const auto &[fine, coarse, ratio] : {std::tuple<double, double, int>{0.10, 0.30, 3}, {0.08, 0.56, 7}})
            if (std::string failure = interface_filament_on_bed(name, parts, Mode::Feature, fine, coarse, ratio); !failure.empty() && !tree_refused(failure))
                failures.push_back(failure);
    }
    std::ostringstream report;
    for (const std::string &failure : failures)
        report << failure << '\n';
    INFO(report.str());
    CHECK(failures.empty());
}

TEST_CASE("Benchy and bunny organic trees on a coarse support base are refused for now",
          "[TestRebuild][SupportMatrix]")
{
    std::vector<std::string> failures;
    for (const char *file : {"3DBenchy.drc", "Stanford_Bunny.drc"})
        if (std::string failure = interface_filament_on_bed(file, handy_model(file), Mode::Feature, 0.10, 0.30, 3); !failure.empty() && !tree_refused(failure))
            failures.push_back(failure);
    std::ostringstream report;
    for (const std::string &failure : failures)
        report << failure << '\n';
    INFO(report.str());
    CHECK(failures.empty());
}

namespace {
// A 20 x 20 x 40 column with a 40 x 20 x 10 arm at its top, under Body Split: one part on the fine
// 0.2 (filament 1, 0.1 mm), the other on the coarse 0.8 (filament 3, 0.3 mm), in the app's project
// with supports off and the automatic tower.
std::string body_split_two_part(bool column_coarse)
{
    std::ostringstream what;
    what << "Body Split, column on the " << (column_coarse ? "coarse" : "fine") << " nozzle: ";
    DynamicPrintConfig config = app_three_filament_config(Mode::Feature, 0.10, 0.30, 3, k_normal);
    config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::BodySplit));
    config.set_key_value("enable_support", new ConfigOptionBool(false));
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id", "internal_solid_filament_id",
                            "top_surface_filament_id", "bottom_surface_filament_id", "sparse_infill_filament_id"})
        config.set_key_value(key, new ConfigOptionInt(0));
    CadenceTest::Scene scene;
    scene.config   = config;
    scene.populate = [column_coarse](Model &model, Print &print, const DynamicPrintConfig &cfg) {
        ModelObject *object = model.add_object();
        object->name = "two-part column and arm";
        const auto add = [object](TriangleMesh mesh, bool coarse) {
            ModelVolume *volume = object->add_volume(std::move(mesh), ModelVolumeType::MODEL_PART, false);
            volume->config.set_key_value("extruder", new ConfigOptionInt(coarse ? 3 : 1));
            volume->config.set_key_value("regional_layer_height", new ConfigOptionFloat(coarse ? 0.3 : 0.1));
        };
        add(make_cube(20., 20., 40.), column_coarse);
        TriangleMesh arm = make_cube(40., 20., 10.);
        arm.translate(20.f, 0.f, 30.f);
        add(std::move(arm), !column_coarse);
        object->add_instance();
        object->instances.front()->set_offset(Vec3d(100., 120., 0.));
        object->ensure_on_bed();
        print.apply(model, cfg);
        print.set_status_silent();
    };
    CadenceTest::Facts facts;
    try {
        facts = CadenceTest::slice(scene);
    } catch (const std::exception &error) {
        return what.str() + "threw " + error.what();
    }
    if (!facts.refusal.string.empty())
        return what.str() + "refused: " + facts.refusal.string;
    return facts.gcode.empty() ? what.str() + "no G-code" : std::string();
}
} // namespace

TEST_CASE("Body Split slices with the part on the bed on the coarse nozzle", "[TestRebuild][Support]")
{
    for (bool column_coarse : {false, true}) {
        const std::string failure = body_split_two_part(column_coarse);
        INFO(failure);
        CHECK((failure.empty() || tree_refused(failure)));
    }
}

namespace {
// A column with a shelf on the right whose top is 0.1 mm above the underside of an arm on the left,
// and a second arm over the shelf. The left arm's support top and the bottom of the support standing
// on the shelf are closer together than the coarse nozzle's thinnest layer.
TriangleMesh shelf_and_arms_mesh()
{
    return extruded_xz_outline({{0., 0.}, {20., 0.}, {20., 10.1}, {8., 10.1}, {8., 13.}, {20., 13.}, {20., 14.},
                                {0., 14.}, {0., 11.}, {-12., 11.}, {-12., 10.}, {0., 10.}},
                               12.);
}

void collect_paths(const ExtrusionEntity *entity, std::vector<const ExtrusionPath*> &out)
{
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection*>(entity)) {
        for (const ExtrusionEntity *child : collection->entities)
            collect_paths(child, out);
    } else if (const auto *path = dynamic_cast<const ExtrusionPath*>(entity))
        out.push_back(path);
    else if (const auto *multipath = dynamic_cast<const ExtrusionMultiPath*>(entity)) {
        for (const ExtrusionPath &p : multipath->paths)
            out.push_back(&p);
    } else if (const auto *loop = dynamic_cast<const ExtrusionLoop*>(entity)) {
        for (const ExtrusionPath &p : loop->paths)
            out.push_back(&p);
    }
}

// Support roads whose height reaches down (or up) into a layer of the part at the same place.
std::string support_into_part(const PrintObject &object)
{
    std::ostringstream what;
    size_t count = 0;
    for (const SupportLayer *support : object.support_layers()) {
        std::vector<const ExtrusionPath*> paths;
        collect_paths(&support->support_fills, paths);
        for (const ExtrusionPath *path : paths) {
            const double top = support->print_z, bottom = top - double(path->height);
            for (const Layer *layer : object.layers()) {
                if (layer->print_z < bottom + 1e-3 || layer->print_z - layer->height > top - 1e-3)
                    continue;
                const ExPolygons inside = offset_ex(layer->lslices, -float(scale_(0.1)));
                const bool hits = std::any_of(path->polyline.points.begin(), path->polyline.points.end(), [&inside](const auto &road_point) {
                    const Point pt(road_point.x(), road_point.y());
                    return std::any_of(inside.begin(), inside.end(), [&pt](const ExPolygon &island) { return island.contains(pt); });
                });
                if (hits && count++ < 3)
                    what << "[road at Z " << top << ", height " << path->height << ", into the part layer "
                         << layer->print_z - layer->height << ".." << layer->print_z << "] ";
            }
        }
    }
    return count == 0 ? std::string() : std::to_string(count) + " support roads reach into the part " + what.str();
}

std::string coarse_support_on_shelf(double coarse, int ratio, const SupportCase &support)
{
    std::ostringstream what;
    what << support.name << " / coarse " << coarse << ": ";
    DynamicPrintConfig config = h2d_support_config(Mode::Feature, 0.10, coarse, ratio, 0., support,
                                                   FilamentCase{"body and interface on the coarse nozzle", 2, 4});
    config.set_key_value("independent_support_layer_height", new ConfigOptionBool(false));
    Model model;
    Print print;
    print.is_BBL_printer() = true;
    ModelObject *object = model.add_object();
    object->name = "shelf and two arms";
    object->add_volume(shelf_and_arms_mesh(), ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    object->instances.front()->set_offset(Vec3d(60., 60., 0.));
    object->ensure_on_bed();
    print.apply(model, config);
    print.set_status_silent();
    if (const StringObjectException refusal = print.validate(); !refusal.string.empty())
        return what.str() + "refused: " + refusal.string;
    try {
        print.process();
    } catch (const std::exception &error) {
        return what.str() + "threw " + error.what();
    }
    if (print.objects().front()->support_layers().empty())
        return what.str() + "no support";
    const std::string failure = support_into_part(*print.objects().front());
    return failure.empty() ? failure : what.str() + failure;
}
} // namespace

TEST_CASE("Support laid on coarse rows never reaches down into the part it stands on", "[TestRebuild][Support]")
{
    std::vector<std::string> failures;
    for (const auto &[coarse, ratio] : {std::pair<double, int>{0.30, 3}, {0.20, 2}})
        for (const SupportCase &support : {k_normal, k_organic})
            if (std::string failure = coarse_support_on_shelf(coarse, ratio, support); !failure.empty() && !tree_refused(failure))
                failures.push_back(failure);
    std::ostringstream report;
    for (const std::string &failure : failures)
        report << failure << '\n';
    INFO(report.str());
    CHECK(failures.empty());
}

namespace {
// Body Split: a coarse column on the bed and a fine arm on top of it, with support under the arm
// standing on the bed. On the layers under the arm only the coarse body prints, so a fine body
// filament has to come from the project, never the interface filament.
std::string body_split_support_body(const SupportCase &support)
{
    std::ostringstream what;
    what << "Body Split / " << support.name << ": ";
    DynamicPrintConfig config = app_three_filament_config(Mode::Feature, 0.10, 0.30, 3, support);
    config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::BodySplit));
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id", "internal_solid_filament_id",
                            "top_surface_filament_id", "bottom_surface_filament_id", "sparse_infill_filament_id"})
        config.set_key_value(key, new ConfigOptionInt(0));
    const int base = config.opt_int("support_filament"), interface_filament = config.opt_int("support_interface_filament");
    REQUIRE(base != interface_filament);
    CadenceTest::Scene scene;
    scene.config   = config;
    scene.populate = [](Model &model, Print &print, const DynamicPrintConfig &cfg) {
        ModelObject *object = model.add_object();
        object->name = "coarse column, fine arm";
        const auto add = [object](TriangleMesh mesh, int filament, double height) {
            ModelVolume *volume = object->add_volume(std::move(mesh), ModelVolumeType::MODEL_PART, false);
            volume->config.set_key_value("extruder", new ConfigOptionInt(filament));
            volume->config.set_key_value("regional_layer_height", new ConfigOptionFloat(height));
        };
        add(make_cube(20., 20., 40.), 3, 0.3);
        TriangleMesh arm = make_cube(40., 20., 10.);
        arm.translate(20.f, 0.f, 30.f);
        add(std::move(arm), 1, 0.1);
        object->add_instance();
        object->instances.front()->set_offset(Vec3d(100., 120., 0.));
        object->ensure_on_bed();
        print.apply(model, cfg);
        print.set_status_silent();
    };
    CadenceTest::Facts facts;
    try {
        facts = CadenceTest::slice(scene);
    } catch (const std::exception &error) {
        return what.str() + "threw " + error.what();
    }
    if (!facts.refusal.string.empty())
        return what.str() + "refused: " + facts.refusal.string;
    size_t body = 0, body_in_interface = 0, interface_on_bed = 0;
    for (const Move &move : facts.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        const bool in_interface = int(move.extruder_id) == interface_filament - 1;
        if (move.extrusion_role == erSupportMaterial) {
            ++body;
            body_in_interface += in_interface;
        }
        interface_on_bed += in_interface && double(move.position.z()) < 0.3 + 1e-3;
    }
    if (body == 0)
        return what.str() + "no support body";
    if (body_in_interface == 0 && interface_on_bed == 0)
        return {};
    what << body_in_interface << " support body roads in the interface filament, " << interface_on_bed
         << " interface-filament roads in the first coarse layer";
    return what.str();
}
} // namespace

TEST_CASE("Support body the fine nozzle lays never takes the interface filament", "[TestRebuild][Support]")
{
    std::vector<std::string> failures;
    for (const SupportCase &support : {k_normal, k_organic})
        if (std::string failure = body_split_support_body(support); !failure.empty() && !tree_refused(failure))
            failures.push_back(failure);
    std::ostringstream report;
    for (const std::string &failure : failures)
        report << failure << '\n';
    INFO(report.str());
    CHECK(failures.empty());
}

TEST_CASE("The fine nozzle's support body filament comes from the project, never the interface filament", "[TestRebuild][Support]")
{
    // Filaments: 1 PLA and 2 PETG (the interface) and 4 PETG on the fine nozzle, 3 PLA (the base) on the coarse one.
    DynamicPrintConfig config = app_three_filament_config(Mode::Feature, 0.10, 0.30, 3, k_normal);
    REQUIRE(config.opt_int("support_filament") == 3);
    REQUIRE(config.opt_int("support_interface_filament") == 2);
    const auto pick = [&config](const std::vector<unsigned int> &object_filaments) {
        PrintConfig print_config;
        print_config.apply(config, true);
        PrintObjectConfig object_config;
        object_config.apply(config, true);
        return mixed_nozzle_interface_nozzle_body_filament(print_config, object_config, object_filaments);
    };
    // The base's material on the fine nozzle, whether or not this layer or the object prints it.
    CHECK(pick({0}) == std::optional<unsigned int>(0));
    CHECK(pick({}) == std::optional<unsigned int>(0));
    CHECK(pick({2}) == std::optional<unsigned int>(0));
    // Without it, another ordinary filament on that nozzle, the object's own first.
    config.option<ConfigOptionBools>("filament_soluble")->values[0] = true;
    CHECK(pick({}) == std::optional<unsigned int>(3));
    // With none left, there is none: the interface filament is never the answer.
    config.option<ConfigOptionInts>("filament_map")->values[3] = 2;
    CHECK_FALSE(pick({}).has_value());
    // An interface filament of the base's own material prints the body.
    config.option<ConfigOptionStrings>("filament_type")->values[1] = "PLA";
    CHECK(pick({}) == std::optional<unsigned int>(1));
}

namespace {
struct NozzlePair {
    const char *name;
    double      fine_nozzle, coarse_nozzle;
    double      fine_min, fine_max, coarse_min, coarse_max;
    double      fine, first_layer, coarse;
    int         ratio;
};

// A low arm whose interface prints near the bed, beside a taller arm whose support base covers the
// first layer. Returns how the first tower visit went, and sets qualifies when that visit had both
// a coarse arrival and the interface filament.
std::string interface_on_tower_bed(const NozzlePair &nozzles, int base, double arm_z, bool &qualifies, std::string &seen)
{
    qualifies = false;
    std::ostringstream what;
    what << nozzles.name << " / base " << base << " / arm at " << arm_z << ": ";
    const FilamentCase filaments{"PETG interface on fine", base, 3, "PETG"};
    DynamicPrintConfig config = h2d_support_config(Mode::Feature, nozzles.fine, nozzles.coarse, nozzles.ratio, 0., k_normal, filaments);
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{nozzles.fine_nozzle, nozzles.coarse_nozzle});
    config.set_key_value("mixed_nozzle_process_nozzle_diameters", new ConfigOptionFloats{nozzles.fine_nozzle, nozzles.coarse_nozzle});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{nozzles.fine_min, nozzles.coarse_min});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{nozzles.fine_max, nozzles.coarse_max});
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(nozzles.first_layer));
    config.set_key_value("bottom_shell_layers", new ConfigOptionInt(1));
    config.set_key_value("bottom_shell_thickness", new ConfigOptionFloat(0.));
    set_automatic_tower(config);
    fill_per_filament_values(config);
    CadenceTest::Scene scene;
    scene.config   = config;
    scene.populate = [arm_z](Model &model, Print &print, const DynamicPrintConfig &cfg) {
        // One object: the layer plan export takes one object per plate.
        ModelObject *object = model.add_object();
        object->name = "low and tall arms";
        object->add_volume(overhang_shelf_mesh(10., 30., arm_z, arm_z + 3.), ModelVolumeType::MODEL_PART, false);
        TriangleMesh tall = overhang_shelf_mesh(10., 30., 4., 7.);
        tall.translate(0.f, 50.f, 0.f);
        object->add_volume(std::move(tall), ModelVolumeType::MODEL_PART, false);
        object->add_instance();
        object->instances.front()->set_offset(Vec3d(80., 80., 0.));
        object->ensure_on_bed();
        print.apply(model, cfg);
        print.set_status_silent();
    };
    CadenceTest::Facts facts;
    try {
        facts = CadenceTest::slice(scene);
    } catch (const std::exception &error) {
        return what.str() + "threw " + error.what();
    }
    if (!facts.refusal.string.empty())
        return what.str() + "refused: " + facts.refusal.string;
    double tower_bottom = std::numeric_limits<double>::max();
    for (const Move &move : facts.moves)
        if (move.type == EMoveType::Extrude && move.extrusion_role == erWipeTower)
            tower_bottom = std::min(tower_bottom, double(move.position.z()));
    // The first visit: tower roads from the first one to the next road off the tower.
    bool in_visit = false, coarse_arrival = false, interface_arrival = false;
    size_t interface_at_bottom = 0;
    std::string sequence;
    int last = -1;
    for (const Move &move : facts.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        if (move.extrusion_role != erWipeTower) {
            if (in_visit)
                break;
            continue;
        }
        in_visit = true;
        const int filament = int(move.extruder_id);
        if (filament != last) {
            sequence += " F" + std::to_string(filament + 1) + "@" + std::to_string(move.position.z());
            last = filament;
        }
        coarse_arrival    |= k_filament_map[size_t(filament) % k_filament_map.size()] == 2;
        interface_arrival |= filament == filaments.interface_filament - 1;
        interface_at_bottom += filament == filaments.interface_filament - 1 && double(move.position.z()) < tower_bottom + 1e-3;
    }
    qualifies = coarse_arrival && interface_arrival;
    what << "first visit" << sequence;
    seen = what.str();
    if (interface_at_bottom != 0)
        what << "; " << interface_at_bottom << " interface-filament roads on the tower's bed level";
    return interface_at_bottom != 0 ? what.str() : std::string();
}
} // namespace

TEST_CASE("The interface filament primes above the tower's first level", "[TestRebuild][Support]")
{
    const NozzlePair pairs[] = {
        {"0.4/0.6", 0.4, 0.6, 0.08, 0.28, 0.12, 0.42, 0.20, 0.20, 0.40, 2},
        // The 0.2 nozzle cannot lay a 0.2 mm first layer, so this pair starts at its fine height.
        {"0.2/0.8", 0.2, 0.8, 0.04, 0.14, 0.16, 0.56, 0.10, 0.10, 0.30, 3},
    };
    std::vector<std::string> failures;
    std::ostringstream seen;
    for (const NozzlePair &nozzles : pairs) {
        size_t qualifying = 0;
        for (int base : {1, 2})
            for (double arm_z : {0.2, 0.4, 0.6, 1.0}) {
                bool qualifies = false;
                std::string scene;
                const std::string failure = interface_on_tower_bed(nozzles, base, arm_z, qualifies, scene);
                qualifying += qualifies;
                seen << scene << (qualifies ? " (interface in the first visit)" : "") << '\n';
                if (!failure.empty() && !tree_refused(failure))
                    failures.push_back(failure);
            }
        // No slice here brings the interface to the first visit; the tower plan test below covers
        // that case directly. This one only guards the sliced scenes.
        (void) qualifying;
    }
    std::ostringstream report;
    report << seen.str();
    for (const std::string &failure : failures)
        report << failure << '\n';
    INFO(report.str());
    CHECK(failures.empty());
}

namespace {
// The lagging tower's plan for the first visit: PLA on the fine nozzle, then the PLA base
// on the coarse one, then the PETG interface on the fine one. Returns the Z of the level holding
// the interface arrival and of the tower's bed level, from the generated plan.
struct FirstVisit { float interface_z = -1.f, bed_z = -1.f; std::string plan; };
FirstVisit plan_first_visit(const NozzlePair &nozzles)
{
    const FilamentCase filaments{"PETG interface on fine", 2, 3, "PETG"};
    DynamicPrintConfig dynamic = h2d_support_config(Mode::Feature, nozzles.fine, nozzles.coarse, nozzles.ratio, 0., k_normal, filaments);
    dynamic.set_key_value("nozzle_diameter", new ConfigOptionFloats{nozzles.fine_nozzle, nozzles.coarse_nozzle});
    dynamic.set_key_value("min_layer_height", new ConfigOptionFloats{nozzles.fine_min, nozzles.coarse_min});
    dynamic.set_key_value("max_layer_height", new ConfigOptionFloats{nozzles.fine_max, nozzles.coarse_max});
    dynamic.set_key_value("initial_layer_print_height", new ConfigOptionFloat(nozzles.first_layer));
    set_automatic_tower(dynamic);
    fill_per_filament_values(dynamic);
    PrintConfig config;
    config.apply(dynamic, true);

    std::vector<MultiNozzleUtils::NozzleInfo> nozzle_list(2);
    for (int e = 0; e < 2; ++e) {
        std::ostringstream diameter;
        diameter << (e == 0 ? nozzles.fine_nozzle : nozzles.coarse_nozzle);
        nozzle_list[size_t(e)].diameter    = diameter.str();
        nozzle_list[size_t(e)].volume_type = nvtStandard;
        nozzle_list[size_t(e)].extruder_id = e;
        nozzle_list[size_t(e)].group_id    = e;
    }
    const std::vector<int>          filament_extruders{0, 1, 0, 1};
    const std::vector<unsigned int> used{0, 1, 2, 3};
    const auto group = MultiNozzleUtils::LayeredNozzleGroupResult::create(filament_extruders, nozzle_list, used);
    REQUIRE(group.has_value());

    const float first = float(nozzles.first_layer), coarse = float(nozzles.coarse), fine = float(nozzles.fine);
    const float top   = first + 10.f * coarse;
    WipeTower tower(config, 0, Vec3d::Zero(), 0, top, used);
    tower.set_nozzle_group_result(*group);
    for (size_t idx = 0; idx < used.size(); ++idx)
        tower.set_extruder(idx, config);
    tower.set_lagging(2.f * coarse, 1, filament_extruders, 1, coarse, fine);
    // The first part layer, a legal first level with nothing held back: fine PLA, coarse PLA base,
    // then the fine interface.
    tower.plan_toolchange(first, first, 0, 1, 50.f, 0.f, 15.f);
    tower.plan_toolchange(first, first, 1, 2, 50.f, 0.f, 15.f);
    // Later coarse layers keep the same pattern, so the tower has a base to stand on.
    for (int layer = 1; layer <= 10; ++layer) {
        const float z = first + float(layer) * coarse;
        tower.plan_toolchange(z, coarse, 2, 1, 50.f, 0.f, 15.f);
        tower.plan_toolchange(z, coarse, 1, 2, 50.f, 0.f, 15.f);
    }
    tower.plan_levels();

    FirstVisit visit;
    visit.plan = tower.plan_digest();
    const size_t generated = visit.plan.find("plan as generated\n");
    REQUIRE(generated != std::string::npos);
    std::istringstream lines(visit.plan.substr(generated));
    std::map<int, float> level_z;
    std::map<int, bool>  level_idle;
    std::string line;
    while (std::getline(lines, line)) {
        int level = -1, change = -1;
        if (std::sscanf(line.c_str(), "level %d change %d", &level, &change) == 2) {
            // The first arrival of the interface filament (index 2).
            if (visit.interface_z < 0.f && line.find(" enters=2 ") != std::string::npos)
                visit.interface_z = level_z[level];
        } else if (float z = 0.f; std::sscanf(line.c_str(), "level %d z=%f", &level, &z) == 2) {
            level_z[level]    = z;
            level_idle[level] = line.find(" idle=1") != std::string::npos;
            if (visit.bed_z < 0.f && !level_idle[level])
                visit.bed_z = z;
        }
    }
    return visit;
}
} // namespace

TEST_CASE("The tower plan primes the interface above its bed level on the first visit", "[TestRebuild][Support]")
{
    const NozzlePair pairs[] = {
        {"0.4/0.6", 0.4, 0.6, 0.08, 0.28, 0.12, 0.42, 0.20, 0.20, 0.40, 2},
        {"0.2/0.8", 0.2, 0.8, 0.04, 0.14, 0.16, 0.56, 0.10, 0.10, 0.30, 3},
    };
    for (const NozzlePair &nozzles : pairs) {
        const FirstVisit visit = plan_first_visit(nozzles);
        INFO(nozzles.name << "\n" << visit.plan);
        REQUIRE(visit.bed_z > 0.f);
        REQUIRE(visit.interface_z > 0.f);
        CHECK(visit.interface_z > visit.bed_z + 1e-4f);
    }
}

TEST_CASE("The fake wipe tower stops on data that could never give a finite stack", "[TestRebuild][Support]")
{
    // A tower drawn with no outer wall falls back to the generic fake paths; the BBL setter
    // gives no depth steps, cone or rotation of its own.
    FakeWipeTower tower;
    tower.set_fake_extrusion_data(Vec2f(10.f, 20.f), 30.f, 10.f, 0.2f, 15.f, 2.f, Vec2d::Zero());
    const ExtrusionLayers layers = tower.getTrueExtrusionLayersFromWipeTower();
    CHECK(layers.size() == 50);
    for (const ExtrusionLayer &layer : layers) {
        REQUIRE(std::isfinite(layer.bottom_z));
        CHECK(layer.bottom_z < 10.f);
    }
    // A zero layer height, a step too small to advance the Z sum, or no depth steps: no layers,
    // never an endless loop.
    const std::vector<std::pair<float, float>> steps{{0.f, 15.f}};
    FakeWipeTower flat;
    flat.set_fake_extrusion_data(Vec2f(10.f, 20.f), 30.f, 10.f, 0.f, 15.f, steps, 2.f, 0.f, 0.f, Vec2d::Zero());
    CHECK(flat.getTrueExtrusionLayersFromWipeTower().empty());
    FakeWipeTower stalled;
    stalled.set_fake_extrusion_data(Vec2f(10.f, 20.f), 30.f, 300.f, 1e-5f, 15.f, steps, 2.f, 0.f, 0.f, Vec2d::Zero());
    CHECK(stalled.getTrueExtrusionLayersFromWipeTower().empty());
    FakeWipeTower stepless;
    stepless.set_fake_extrusion_data(Vec2f(10.f, 20.f), 30.f, 10.f, 0.2f, 15.f, {}, 2.f, 0.f, 0.f, Vec2d::Zero());
    CHECK(stepless.getTrueExtrusionLayersFromWipeTower().empty());
}

TEST_CASE("A tower-enabled print that needs no tower never checks conflicts against one", "[TestRebuild][Support]")
{
    // The Body Split scene of the support body test: the tower is on, but no layer switches tools.
    DynamicPrintConfig config = app_three_filament_config(Mode::Feature, 0.10, 0.30, 3, k_normal);
    config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::BodySplit));
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id", "internal_solid_filament_id",
                            "top_surface_filament_id", "bottom_surface_filament_id", "sparse_infill_filament_id"})
        config.set_key_value(key, new ConfigOptionInt(0));
    mixed_nozzle_fixtures::fill_per_filament_values(config);
    Model model;
    // Fill the Print's memory first, so a member nothing sets reads as garbage rather than as zero.
    void *memory = ::operator new(sizeof(Print));
    std::memset(memory, 0x7f, sizeof(Print));
    const std::unique_ptr<Print, void (*)(Print *)> owner(new (memory) Print, [](Print *p) {
        p->~Print();
        ::operator delete(p);
    });
    Print &print = *owner;
    print.is_BBL_printer() = CadenceTest::Scene{}.bambu;
    ModelObject *object = model.add_object();
    const auto add = [object](TriangleMesh mesh, int filament, double height) {
        ModelVolume *volume = object->add_volume(std::move(mesh), ModelVolumeType::MODEL_PART, false);
        volume->config.set_key_value("extruder", new ConfigOptionInt(filament));
        volume->config.set_key_value("regional_layer_height", new ConfigOptionFloat(height));
    };
    add(make_cube(20., 20., 40.), 3, 0.3);
    TriangleMesh arm = make_cube(40., 20., 10.);
    arm.translate(20.f, 0.f, 30.f);
    add(std::move(arm), 1, 0.1);
    object->add_instance();
    object->instances.front()->set_offset(Vec3d(100., 120., 0.));
    object->ensure_on_bed();
    print.apply(model, config);
    print.set_status_silent();
    // Without the conflict check, which read the never-filled fake tower and could loop without end.
    print.set_no_check_flag(true);
    const std::string refusal = print.validate().string;
    INFO(refusal);
    REQUIRE(refusal.empty());
    print.process();
    REQUIRE(print.has_wipe_tower());
    REQUIRE(print.wipe_tower_data().tool_changes.empty());
    // No tower was generated, so the fake tower holds nothing, never data left from elsewhere.
    const FakeWipeTower &fake = print.get_fake_wipe_tower();
    CHECK(fake.outer_wall.empty());
    CHECK(fake.width == 0.f);
    CHECK(fake.height == 0.f);
    CHECK(fake.layer_height == 0.f);
    CHECK(fake.z_and_depth_pairs.empty());
}

TEST_CASE("The tower lays its first layer on the bed with its brim when layer 1 has no switch", "[TestRebuild][TowerFirstLayer]")
{
    // Support under a ball, its body on the coarse nozzle. Layer 1 prints on the fine nozzle alone,
    // and the coarse nozzle first arrives one fine layer up, closer than its own minimum. The L part
    // prints three filaments, with the PETG interface under its arm.
    const double fine = 0.10;
    std::vector<std::string> failures;
    for (const auto &[filaments, ball] : {std::pair<FilamentCase, bool>{{"ball, interface on the part filament", 2, 1}, true},
                                          {{"L part, PETG interface on fine", 2, 3, "PETG"}, false}}) {
        std::ostringstream what;
        what << filaments.name << ": ";
        DynamicPrintConfig config = h2d_support_config(Mode::Feature, fine, 0.30, 3, 0.2, k_normal, filaments);
        set_automatic_tower(config);
        CadenceTest::Scene scene;
        scene.config   = config;
        scene.populate = [ball = ball](Model &model, Print &print, const DynamicPrintConfig &cfg) {
            ModelObject *object = model.add_object();
            object->name = ball ? "ball" : "l-part";
            object->add_volume(ball ? hull_mesh() : overhang_shelf_mesh(20., 60., 40., 50.), ModelVolumeType::MODEL_PART, false);
            object->add_instance();
            object->instances.front()->set_offset(Vec3d(80., 80., 0.));
            object->ensure_on_bed();
            print.apply(model, cfg);
            print.set_status_silent();
        };
        CadenceTest::Facts facts;
        try {
            facts = CadenceTest::slice(scene);
        } catch (const std::exception &error) {
            failures.push_back(what.str() + "threw " + error.what());
            continue;
        }
        if (!facts.refusal.string.empty()) {
            failures.push_back(what.str() + "refused: " + facts.refusal.string);
            continue;
        }
        // Layer 1 has no tool switch, so nothing brings the tower there but the first layer itself.
        std::set<int> first_layer_filaments;
        for (const Move &move : facts.moves)
            if (move.type == EMoveType::Extrude && move.extrusion_role != erWipeTower && double(move.position.z()) < fine + 1e-3)
                first_layer_filaments.insert(int(move.extruder_id));
        // The brim widens the first layer past the fullest of the next few tower levels.
        BoundingBoxf first;
        std::map<double, std::pair<size_t, BoundingBoxf>> above;
        size_t first_roads = 0, first_interface = 0;
        double lowest = std::numeric_limits<double>::max();
        for (const Move &move : facts.moves) {
            if (move.type != EMoveType::Extrude || move.extrusion_role != erWipeTower)
                continue;
            const double z = double(move.position.z());
            lowest = std::min(lowest, z);
            const Vec2d xy(move.position.x(), move.position.y());
            if (z < fine + 1e-3) {
                ++first_roads;
                // Here the interface filament is not the part's, so it must stay off the first layer.
                first_interface += filaments.interface_filament != 1 && int(move.extruder_id) == filaments.interface_filament - 1;
                first.merge(xy);
            } else if (z < fine + 1.) {
                auto &level = above[std::round(z * 1000.) / 1000.];
                ++level.first;
                level.second.merge(xy);
            }
        }
        const std::pair<size_t, BoundingBoxf> *fullest = nullptr;
        for (const auto &level : above)
            if (fullest == nullptr || level.second.first > fullest->first)
                fullest = &level.second;
        const double brim = first.defined && fullest != nullptr ?
            std::min({fullest->second.min.x() - first.min.x(), fullest->second.min.y() - first.min.y(),
                      first.max.x() - fullest->second.max.x(), first.max.y() - fullest->second.max.y()}) : 0.;
        what << "layer 1 filaments";
        for (int filament : first_layer_filaments)
            what << ' ' << filament + 1;
        what << ", lowest tower road at Z " << lowest
             << ", first-layer tower roads " << first_roads << ", brim " << brim;
        if ((ball && first_layer_filaments.size() != 1) || first_roads == 0 || brim < 1. || first_interface != 0)
            failures.push_back(what.str() + (first_interface != 0 ? ", interface filament on the tower's first layer" : ""));
    }
    std::ostringstream report;
    for (const std::string &failure : failures)
        report << failure << '\n';
    INFO(report.str());
    CHECK(failures.empty());
}

struct CoarseNozzle { double coarse, coarse_min, coarse_max; };

// Where the coarse-base cases put the support filaments. Filaments 1 and 3 are on the 0.2, 2 and 4 on the coarse nozzle.
enum class SupportPlacement { PetgOnFine, AllCoarse, PetgOnCoarse };

// The L part (a column with an arm, support from the bed) or a step part: a box, a column on it and a slab
// wider than the box, so the support under the slab stands both on the box (bottom contacts) and on the bed.
enum class PartShape { L, Step };

// Slices a fine PLA part on the 0.2 with its support base in PLA on the coarse nozzle, 0.10 / 0.40 with a
// 0.10 first layer and widths set for the 0.2 (0.25 first layer, 0.22 support), and returns what is
// wrong, or an empty string.
std::string coarse_base_case(Mode mode, const CoarseNozzle &nozzles, const SupportCase &support, SupportPlacement where,
                     int raft_layers = 0, PartShape part_shape = PartShape::L)
{
    const double fine = 0.10;
    const FilamentCase filaments = where == SupportPlacement::AllCoarse ? FilamentCase{"PLA base and interface on coarse", 2, 2} :
        where == SupportPlacement::PetgOnCoarse ? FilamentCase{"PLA base, PETG interface, both on coarse", 2, 4, "PETG"} :
                                            FilamentCase{"coarse PLA base, PETG interface on fine", 2, 3, "PETG"};
    const bool coarse_interface = where != SupportPlacement::PetgOnFine;
    std::ostringstream what;
    what << mode_name(mode) << " / " << nozzles.coarse << " / " << support.name << " / " << filaments.name
         << (part_shape == PartShape::Step ? " / step part" : "") << ": ";
    DynamicPrintConfig config = h2d_support_config(mode, fine, 0.40, 4, 0., support, filaments);
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, nozzles.coarse});
    config.set_key_value("mixed_nozzle_process_nozzle_diameters", new ConfigOptionFloats{0.2, nozzles.coarse});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, nozzles.coarse_min});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, nozzles.coarse_max});
    config.set_key_value("support_interface_bottom_layers", new ConfigOptionInt(2));
    config.set_key_value("support_bottom_z_distance", new ConfigOptionFloat(0.1));
    config.set_key_value("line_width", new ConfigOptionFloatOrPercent(0.22, false));
    config.set_key_value("initial_layer_line_width", new ConfigOptionFloatOrPercent(0.25, false));
    config.set_key_value("support_line_width", new ConfigOptionFloatOrPercent(0.22, false));
    config.set_key_value("raft_layers", new ConfigOptionInt(raft_layers));
    set_automatic_tower(config);
    CadenceTest::Scene scene;
    scene.config   = config;
    scene.populate = [mode, part_shape](Model &model, Print &print, const DynamicPrintConfig &cfg) {
        ModelObject *object = model.add_object();
        object->name = "coarse-base-part";
        if (part_shape == PartShape::L)
            object->add_volume(overhang_shelf_mesh(20., 60., 40., 50.), ModelVolumeType::MODEL_PART, false);
        else {
            TriangleMesh mesh = make_cube(30., 20., 10.);
            TriangleMesh column = make_cube(8., 8., 10.);
            column.translate(11.f, 6.f, 10.f);
            mesh.merge(column);
            TriangleMesh slab = make_cube(44., 20., 2.);
            slab.translate(-7.f, 0.f, 20.f);
            mesh.merge(slab);
            object->add_volume(std::move(mesh), ModelVolumeType::MODEL_PART, false);
        }
        // On a Body Split plate a one-part object on one filament is a plain object. The plate needs a
        // Body Split object too: a block with a coarse half, which needs no support.
        if (mode == Mode::Body) {
            object->volumes.front()->config.set_key_value("extruder", new ConfigOptionInt(1));
            ModelObject *body = model.add_object();
            body->name = "body-block";
            ModelVolume *fine_half = body->add_volume(make_cube(10., 10., 10.), ModelVolumeType::MODEL_PART, false);
            fine_half->config.set_key_value("extruder", new ConfigOptionInt(1));
            fine_half->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.1));
            ModelVolume *coarse_half = body->add_volume(make_cube(10., 10., 10.), ModelVolumeType::MODEL_PART, false);
            coarse_half->set_offset(Vec3d(10., 0., 0.));
            coarse_half->config.set_key_value("extruder", new ConfigOptionInt(2));
            coarse_half->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.4));
            body->add_instance();
            body->instances.front()->set_offset(Vec3d(200., 200., 0.));
            body->ensure_on_bed();
        }
        object->add_instance();
        object->instances.front()->set_offset(Vec3d(80., 80., 0.));
        object->ensure_on_bed();
        print.apply(model, cfg);
        print.set_status_silent();
    };
    CadenceTest::Facts facts;
    try {
        facts = CadenceTest::slice(scene);
    } catch (const std::exception &error) {
        return what.str() + "threw " + error.what();
    }
    if (!facts.refusal.string.empty())
        return what.str() + "refused: " + facts.refusal.string;
    const double nozzle[2] = {0.2, nozzles.coarse};
    const int    petg      = filaments.interface_filament != filaments.base ? filaments.interface_filament - 1 : -1;
    // The first row: layer 1, or on the coarse nozzle the first whole layers it can lay.
    const double bed_z     = coarse_interface ? fine * std::ceil(nozzles.coarse_min / fine - 1e-6) : fine;
    const double bed_speed = config.option<ConfigOptionFloatsNullable>("initial_layer_infill_speed")->get_at(coarse_interface ? 1 : 0);
    double lowest_support = std::numeric_limits<double>::max();
    size_t narrow = 0, first_layer_petg = 0, bed_roads = 0, bed_thin = 0, bed_fast = 0, roads[2] = {0, 0};
    double thinnest[2] = {1e9, 1e9}, widest[2] = {0., 0.};
    // Support Z levels.
    std::set<long long> support_z;
    const auto key = [](double z) { return (long long)std::llround(z * 1e4); };
    for (size_t i = 0; i < facts.moves.size(); ++ i) {
        const Move &move = facts.moves[i];
        if (move.type != EMoveType::Extrude)
            continue;
        const double z = double(move.position.z());
        if (z < fine + 1e-3 && int(move.extruder_id) == petg)
            ++first_layer_petg;
        if (move.extrusion_role != erSupportMaterial && move.extrusion_role != erSupportMaterialInterface)
            continue;
        support_z.insert(key(z));
        lowest_support = std::min(lowest_support, z);
        if (z < bed_z + 1e-3) {
            ++bed_roads;
            bed_thin += coarse_interface && double(move.width) < 1.25 * nozzles.coarse - 1e-3;
            bed_fast += double(move.feedrate) > bed_speed + 1e-3;
        }
        const int tool = k_filament_map[move.extruder_id] - 1;
        ++roads[tool];
        thinnest[tool] = std::min(thinnest[tool], double(move.width));
        widest[tool]   = std::max(widest[tool], double(move.width));
        if (double(move.width) < 0.75 * nozzle[tool] - 1e-3 || double(move.height) > double(move.width) + 1e-3) {
            if (narrow < 3)
                what << "[width " << move.width << " on the " << nozzle[tool] << " at Z " << z
                     << ", height " << move.height << ", role " << int(move.extrusion_role) << "] ";
            ++narrow;
        }
    }
    // Stock lays one dense layer in the base's material between a dissimilar interface and the body: under
    // a top contact's interface, over a bottom contact's. The nozzle that lays the interface lays it too.
    // At each interface-filament layer, the next support level up and down that has support over the
    // interface's footprint (more than part there) and no interface filament must be covered densely, in
    // base material on the interface's nozzle; sparse body covers a quarter or less.
    std::map<long long, BoundingBoxf> petg_box;
    std::map<long long, double> petg_area;
    if (petg >= 0)
        for (size_t i = 1; i < facts.moves.size(); ++ i) {
            const Move &move = facts.moves[i];
            if (move.type == EMoveType::Extrude && int(move.extruder_id) == petg && move.extrusion_role == erSupportMaterialInterface) {
                const Vec3f &from = facts.moves[i - 1].position;
                BoundingBoxf &box = petg_box[key(move.position.z())];
                box.merge(Vec2d(from.x(), from.y()));
                box.merge(Vec2d(move.position.x(), move.position.y()));
                petg_area[key(move.position.z())] += double((move.position - from).norm()) * double(move.width);
            }
        }
    // Area of roads at level z over box: support, base material on the interface's nozzle, and part.
    struct Areas { double support = 0., base = 0., part = 0.; };
    const auto areas_at = [&](long long z, const BoundingBoxf &box) {
        Areas out;
        for (size_t i = 1; i < facts.moves.size(); ++ i) {
            const Move &move = facts.moves[i];
            if (move.type != EMoveType::Extrude || key(move.position.z()) != z)
                continue;
            const Vec3f &from = facts.moves[i - 1].position;
            if (!box.contains(Vec2d(0.5 * (from.x() + move.position.x()), 0.5 * (from.y() + move.position.y()))))
                continue;
            const double area = double((move.position - from).norm()) * double(move.width);
            if (CadenceTest::model_road(move))
                out.part += area;
            if (move.extrusion_role != erSupportMaterial && move.extrusion_role != erSupportMaterialInterface)
                continue;
            out.support += area;
            if (move.extrusion_role == erSupportMaterial && int(move.extruder_id) != petg &&
                k_filament_map[move.extruder_id] == k_filament_map[petg])
                out.base += area;
        }
        return out;
    };
    size_t floors_checked = 0, floors_missing = 0;
    for (const auto &[z, box] : petg_box) {
        const auto it = support_z.find(z);
        for (const long long neighbour : {it == support_z.begin() ? -1LL : *std::prev(it),
                                          std::next(it) == support_z.end() ? -1LL : *std::next(it)}) {
            if (neighbour < 0 || petg_box.count(neighbour) != 0)
                continue;
            const Areas areas = areas_at(neighbour, box);
            if (areas.support < 0.05 * petg_area.at(z) || areas.part > areas.support)
                continue;
            ++floors_checked;
            if (areas.base < 0.6 * petg_area.at(z)) {
                if (floors_missing < 3)
                    what << "[interface at Z " << z * 1e-4 << ": base material at Z " << neighbour * 1e-4 << " covers "
                         << areas.base << " mm2 against the interface's " << petg_area.at(z) << "] ";
                ++floors_missing;
            }
        }
    }
    what << "lowest support road at Z " << lowest_support << ", bed-layer roads " << bed_roads;
    for (int tool : {0, 1})
        what << ", " << roads[tool] << " roads on the " << nozzle[tool] << " " << thinnest[tool] << "-" << widest[tool];
    bool ok = true;
    if (std::abs(lowest_support - bed_z) > 1e-3 || bed_roads == 0) {
        what << ", support's first row is not at Z " << bed_z;
        ok = false;
    }
    if (bed_thin != 0 || bed_fast != 0) {
        what << ", " << bed_thin << " first-row roads under the coarse first-layer width, " << bed_fast
             << " faster than the first-layer speed " << bed_speed;
        ok = false;
    }
    if (narrow != 0) {
        what << ", " << narrow << " support roads narrower than 0.75 x their nozzle or taller than wide";
        ok = false;
    }
    if (first_layer_petg != 0) {
        what << ", " << first_layer_petg << " interface-filament roads on layer 1";
        ok = false;
    }
    if (petg >= 0 && (floors_checked == 0 || floors_missing != 0)) {
        what << ", " << floors_missing << " of " << floors_checked << " interface layers without a dense base-material layer beside them";
        ok = false;
    }
    return ok ? std::string() : what.str();
}

std::string failure_report(const std::vector<std::string> &failures)
{
    std::ostringstream report;
    for (const std::string &failure : failures)
        if (!failure.empty())
            report << failure << '\n';
    return report.str();
}

TEST_CASE("The support's bed layer prints on layer 1 and every support road is at least its nozzle's width",
          "[TestRebuild][Support]")
{
    // The coarse nozzle's minimum layer is above 0.10, so it cannot lay the support's first layer; that
    // layer must still print on layer 1, and no nozzle may lay a support road much narrower than its own
    // face. With base and interface both on the coarse nozzle there is no fine support filament: the
    // first row stays the coarse nozzle's first legal height from the bed, laid at its own first-layer
    // width and speed.
    std::vector<std::string> failures;
    for (const CoarseNozzle &nozzles : {CoarseNozzle{0.6, 0.12, 0.42}, CoarseNozzle{0.8, 0.16, 0.56}})
        for (const SupportPlacement where : {SupportPlacement::PetgOnFine, SupportPlacement::AllCoarse, SupportPlacement::PetgOnCoarse})
            for (const PartShape part_shape : {PartShape::L, PartShape::Step})
                failures.push_back(coarse_base_case(Mode::Feature, nozzles, k_normal, where, 0, part_shape));
    const std::string report = failure_report(failures);
    INFO(report);
    CHECK(report.empty());
}

TEST_CASE("A plain object on a Body Split plate lays its coarse support at the coarse nozzle's width", "[TestRebuild][Support]")
{
    // With the interface on the 0.2 the plain object's support body has no coarse cadence to band to, so
    // the 0.2 lays it; with both support filaments on the 0.6 the 0.6 lays all of it.
    std::vector<std::string> failures;
    for (const SupportPlacement where : {SupportPlacement::PetgOnFine, SupportPlacement::AllCoarse, SupportPlacement::PetgOnCoarse})
        for (const PartShape part_shape : {PartShape::L, PartShape::Step})
            failures.push_back(coarse_base_case(Mode::Body, CoarseNozzle{0.6, 0.12, 0.42}, k_normal, where, 0, part_shape));
    const std::string report = failure_report(failures);
    INFO(report);
    CHECK(report.empty());
}

TEST_CASE("A raft with the support base on the coarse nozzle is refused", "[TestRebuild][Support]")
{
    // The raft's first layer is the print's first layer, which the coarse nozzle must not lay.
    for (const Mode mode : {Mode::Feature, Mode::Body})
        for (const SupportPlacement where : {SupportPlacement::PetgOnFine, SupportPlacement::AllCoarse}) {
            const std::string outcome = coarse_base_case(mode, CoarseNozzle{0.6, 0.12, 0.42}, k_normal, where, 2);
            INFO(outcome);
            CHECK(outcome.find(mode == Mode::Feature ? "refused: [SRL-F13]" : "refused: [SRL-A52]") != std::string::npos);
            CHECK(outcome.find("Support/raft base") != std::string::npos);
        }
}

TEST_CASE("Tree support with the support base on the coarse nozzle is refused", "[TestRebuild][Support]")
{
    for (const Mode mode : {Mode::Feature, Mode::Body})
        for (const SupportCase &support : {k_organic, k_slim, k_hybrid}) {
            const std::string outcome = coarse_base_case(mode, CoarseNozzle{0.6, 0.12, 0.42}, support, SupportPlacement::PetgOnFine);
            INFO(outcome);
            CHECK(outcome.find(mode == Mode::Feature ? "refused: [SRL-F14]" : "refused: [SRL-A53]") != std::string::npos);
            CHECK(outcome.find("Normal support") != std::string::npos);
        }
}

namespace {
// Length of the support interface roads laid in the interface filament: on the first layer and in all.
struct InterfaceLength {
    double      first_layer = 0.;
    double      total       = 0.;
    size_t      layers      = 0;
    std::string refusal;
    // "Z: mm" for every layer that has the interface filament, and what else the support lays there.
    std::string by_layer;
};

// A part whose overhang starts on layer 2 (a 20 mm foot one layer tall under a 60 mm slab) and a
// second overhang 10 mm up, with the support base in PLA on the coarse nozzle and a PETG interface on
// the fine one at zero gap. Mode Off is the stock print of the same settings.
InterfaceLength interface_length_case(Mode mode, const CoarseNozzle &nozzles)
{
    const double fine = 0.10;
    const FilamentCase filaments{"coarse PLA base, PETG interface on fine", 2, 3, "PETG"};
    DynamicPrintConfig config = h2d_support_config(mode, fine, 0.40, 4, 0., k_normal, filaments);
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, nozzles.coarse});
    config.set_key_value("mixed_nozzle_process_nozzle_diameters", new ConfigOptionFloats{0.2, nozzles.coarse});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, nozzles.coarse_min});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, nozzles.coarse_max});
    config.set_key_value("line_width", new ConfigOptionFloatOrPercent(0.22, false));
    config.set_key_value("initial_layer_line_width", new ConfigOptionFloatOrPercent(0.25, false));
    config.set_key_value("support_line_width", new ConfigOptionFloatOrPercent(0.22, false));
    set_automatic_tower(config);
    CadenceTest::Scene scene;
    scene.config   = config;
    scene.populate = [](Model &model, Print &print, const DynamicPrintConfig &cfg) {
        ModelObject *object = model.add_object();
        object->name = "overhang-from-layer-two";
        TriangleMesh mesh = extruded_xz_outline({{0., 0.}, {20., 0.}, {20., 0.1}, {60., 0.1}, {60., 3.}, {40., 3.},
                                                 {40., 10.}, {70., 10.}, {70., 13.}, {0., 13.}}, 20.);
        object->add_volume(std::move(mesh), ModelVolumeType::MODEL_PART, false);
        object->add_instance();
        object->instances.front()->set_offset(Vec3d(80., 80., 0.));
        object->ensure_on_bed();
        print.apply(model, cfg);
        print.set_status_silent();
    };
    InterfaceLength out;
    CadenceTest::Facts facts;
    try {
        facts = CadenceTest::slice(scene);
    } catch (const std::exception &error) {
        out.refusal = std::string("threw ") + error.what();
        return out;
    }
    out.refusal = facts.refusal.string;
    std::map<int, double> layers;
    std::map<int, std::map<std::string, double>> others;
    Vec3f previous = Vec3f::Zero();
    for (const Move &move : facts.moves) {
        const Vec3f from = previous;
        previous = move.position;
        if (move.type != EMoveType::Extrude ||
            (move.extrusion_role != erSupportMaterialInterface && move.extrusion_role != erSupportMaterial))
            continue;
        const double length = (move.position.head<2>() - from.head<2>()).norm();
        const int    z      = int(std::lround(double(move.position.z()) * 1000.));
        if (move.extrusion_role != erSupportMaterialInterface || int(move.extruder_id) != filaments.interface_filament - 1) {
            others[z][std::string(move.extrusion_role == erSupportMaterial ? "base" : "interface") + " filament " +
                      std::to_string(int(move.extruder_id) + 1) + " h " + std::to_string(move.height).substr(0, 4)] += length;
            continue;
        }
        out.total += length;
        if (double(move.position.z()) < fine + 1e-3)
            out.first_layer += length;
        layers[z] += length;
    }
    out.layers = layers.size();
    std::ostringstream text;
    for (const auto &[z, length] : layers) {
        text << "Z " << z / 1000. << ": " << length << " mm";
        for (const auto &[what, other] : others[z])
            text << ", " << what << " " << other << " mm";
        text << "; ";
    }
    out.by_layer = text.str();
    return out;
}
} // namespace

// The contact and interface areas were once grown with the support base's road width, which is
// scaled to the coarse nozzle (0.22 to 0.66 or 0.88). The interface filament
// then covered about twice stock's area on every contact, the first layer included, and a sliver of
// it appeared on layers where stock lays none, each costing a filament swap on the fine nozzle.
TEST_CASE("A coarse support base lays no more interface filament than the stock print of the same settings",
          "[TestRebuild][Support]")
{
    const CoarseNozzle nozzles = GENERATE(CoarseNozzle{0.6, 0.12, 0.42}, CoarseNozzle{0.8, 0.16, 0.56});
    CAPTURE(nozzles.coarse);
    const InterfaceLength stock = interface_length_case(Mode::Off, nozzles);
    INFO(stock.refusal);
    REQUIRE(stock.refusal.empty());
    const InterfaceLength split = interface_length_case(Mode::Feature, nozzles);
    INFO(split.refusal);
    REQUIRE(split.refusal.empty());
    CAPTURE(stock.first_layer, split.first_layer, stock.total, split.total, stock.layers, split.layers);
    INFO("stock: " << stock.by_layer);
    INFO("split: " << split.by_layer);
    // The scene: stock lays interface filament on the first layer too.
    REQUIRE(stock.first_layer > 100.);
    REQUIRE(stock.total > stock.first_layer);
    CHECK(split.first_layer <= 1.05 * stock.first_layer);
    // The support that stands on the part starts with a layer the interface nozzle lays in the body
    // filament. Only the one layer of it that shares its height with a coarse band top still goes to
    // the interface filament, because a layer has one base filament.
    CHECK(split.total <= 1.3 * stock.total);
    CHECK(split.total >= 0.9 * stock.total);
    CHECK(split.layers <= stock.layers + 1);
}
