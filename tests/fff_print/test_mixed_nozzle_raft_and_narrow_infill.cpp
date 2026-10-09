// A narrow region keeps its sparse infill under Feature Split, a raft's rounded gap is not
// reported as an empty layer, and Body Split on a raft is refused with a message the user can act on.

#include <catch2/catch_all.hpp>

#include "mixed_nozzle_harness.hpp"
#include "mixed_nozzle_fixtures.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace mixed_nozzle_fixtures;

namespace {
using Move = GCodeProcessorResult::MoveVertex;

bool has_warning(const PrintStateBase::StateWithWarnings &state, PrintStateBase::SlicingNotificationType code)
{
    return std::any_of(state.warnings.begin(), state.warnings.end(), [code](const PrintStateBase::Warning &warning) {
        return warning.message_id == code;
    });
}

// With CADENCE_TEST_DUMP_DIR set, keeps a case's G-code there for inspection.
void dump_gcode(const std::string &name, const std::string &gcode)
{
    if (const char *dir = std::getenv("CADENCE_TEST_DUMP_DIR")) {
        std::ofstream out(std::string(dir) + "/" + name + ".gcode");
        out << gcode;
    }
}

// Sparse infill laid inside an XY window, in mm3 of filament.
double sparse_infill_volume(const std::vector<Move> &moves, const BoundingBoxf &window)
{
    double length = 0.;
    for (const Move &move : moves)
        if (move.type == EMoveType::Extrude && move.extrusion_role == erInternalInfill &&
            window.contains(Vec2d(move.position.x(), move.position.y())))
            length += move.delta_extruder;
    return length * M_PI * 1.75 * 1.75 / 4.;
}

// A 6 x 6 mm column, 20 mm tall, four walls, 15 % grid: inside the walls it is about 4.3 mm square,
// narrower than the line pitch of the 0.6 nozzle's grid. A 14 x 14 x 3 mm tab on its side, 10 mm up,
// stands on support whose base is on the coarse nozzle, so the coarse nozzle is in the print anyway.
// The column's lower left corner is at (60 + shift, 60).
CadenceTest::Facts slice_column(bool feature_split, double shift)
{
    CadenceTest::Scene scene;
    scene.config = feature_split_tower_config(.20, .60, false);
    scene.config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(.30));
    scene.config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{3});
    scene.config.set_key_value("wall_loops", new ConfigOptionInt(4));
    scene.config.set_key_value("sparse_infill_density", new ConfigOptionPercent(15.));
    scene.config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipGrid));
    scene.config.set_key_value("top_shell_layers", new ConfigOptionInt(3));
    scene.config.set_key_value("bottom_shell_layers", new ConfigOptionInt(3));
    scene.config.set_key_value("enable_support", new ConfigOptionBool(true));
    scene.config.set_key_value("support_type", new ConfigOptionEnum<SupportType>(stNormalAuto));
    scene.config.set_key_value("support_filament", new ConfigOptionInt(2));
    scene.config.set_key_value("support_interface_filament", new ConfigOptionInt(1));
    if (! feature_split) {
        // The stock print of the same plate: mode Off, everything on the fine nozzle.
        scene.config.set_key_value("mixed_nozzle_slicing_mode",
                                   new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::Off));
        scene.config.set_key_value("sparse_infill_filament_id", new ConfigOptionInt(1));
        scene.config.set_key_value("support_filament", new ConfigOptionInt(1));
        scene.config.set_key_value("enable_prime_tower", new ConfigOptionBool(false));
    }
    fill_per_filament_values(scene.config);
    scene.populate = [shift](Model &model, Print &print, const DynamicPrintConfig &config) {
        ModelObject *column = model.add_object();
        column->name = "narrow-column";
        column->add_volume(make_cube(6., 6., 20.), ModelVolumeType::MODEL_PART, false);
        ModelVolume *tab = column->add_volume(make_cube(14., 14., 3.), ModelVolumeType::MODEL_PART, false);
        tab->set_offset(Vec3d(6., -4., 10.));
        column->add_instance();
        column->instances.front()->set_offset(Vec3d(60. + shift, 60., 0.));
        print.apply(model, config);
        print.set_status_silent();
    };
    return CadenceTest::slice(scene);
}
} // namespace

// Every coarse band of the column had no room for a coarse grid line, printed nothing, and
// was kept because a band that prints nothing is the cheapest. The column came out hollow. Whether
// a coarse line crosses the column depends on where it stands, so several positions are sliced.
TEST_CASE("Feature Split keeps the sparse infill of a region narrower than the coarse pattern", "[TestRebuild][SparseInfill]")
{
    std::string report;
    size_t      checked = 0;
    for (int step = 0; step < 6; ++step) {
        const double shift = 1.5 * step;
        const BoundingBoxf window(Vec2d(59.5 + shift, 59.5), Vec2d(66.5 + shift, 66.5));
        const CadenceTest::Facts stock = slice_column(false, shift);
        INFO(stock.refusal.string);
        REQUIRE(stock.refusal.string.empty());
        const CadenceTest::Facts split = slice_column(true, shift);
        INFO(split.refusal.string);
        REQUIRE(split.refusal.string.empty());
        if (step == 0) {
            dump_gcode("column-stock", stock.gcode);
            dump_gcode("column-split", split.gcode);
        }
        const double stock_volume = sparse_infill_volume(stock.moves, window);
        const double split_volume = sparse_infill_volume(split.moves, window);
        REQUIRE(stock_volume > 10.);
        ++ checked;
        if (split_volume < 0.8 * stock_volume)
            report += "column at x " + std::to_string(60. + shift) + ": " + std::to_string(split_volume) + " mm3 of sparse infill against " +
                      std::to_string(stock_volume) + " in the stock print; ";
    }
    INFO(report);
    CHECK(checked == 6);
    CHECK(report.empty());
}

// Under a mode the raft gap is rounded up to a whole fine layer (0.1 becomes 0.16 on 0.08
// layers). The empty-layer check allowed only the configured 0.1 and stopped the export.
TEST_CASE("A raft whose gap is rounded up to the fine layer exports without an empty-layer error", "[TestRebuild][Raft]")
{
    DynamicPrintConfig config = feature_split_tower_config(.20, .40, false);
    config.set_key_value("layer_height", new ConfigOptionFloat(.08));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(.10));
    config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(.16));
    config.set_key_value("raft_layers", new ConfigOptionInt(2));
    config.set_key_value("raft_contact_distance", new ConfigOptionFloat(.10));
    config.set_key_value("support_filament", new ConfigOptionInt(1));
    config.set_key_value("support_interface_filament", new ConfigOptionInt(1));
    // A support gap under the raft's: the check takes the larger of the two.
    config.set_key_value("support_top_z_distance", new ConfigOptionFloat(0.08));
    config.set_key_value("support_bottom_z_distance", new ConfigOptionFloat(0.08));
    config.set_key_value("machine_start_gcode", new ConfigOptionString("G28\nM1020 S[initial_extruder]\n"));
    fill_per_filament_values(config);
    Model model;
    Print print;
    print.is_BBL_printer() = true;
    init_feature_flow_fixture(model, print, config, 3., 20.);
    model.objects.front()->instances.front()->set_offset(Vec3d(60., 60., 0.));
    print.apply(model, config);
    print.set_status_silent();
    const CadenceTest::Facts facts = CadenceTest::slice(print);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    dump_gcode("raft-gap", facts.gcode);
    const SlicingParameters &slicing = print.objects().front()->slicing_parameters();
    CAPTURE(slicing.gap_raft_object, slicing.raft_contact_top_z);
    // The scene: the gap the slicer uses is larger than the configured one.
    CHECK(slicing.gap_raft_object > .10 + 1e-4);
    CHECK_FALSE(has_warning(print.step_state_with_warnings(psGCodeExport), PrintStateBase::SlicingEmptyGcodeLayers));
    // The first object layer stands exactly that gap above the raft.
    double first_model_bottom = 1e9;
    for (const Move &move : facts.moves)
        if (CadenceTest::model_road(move))
            first_model_bottom = std::min(first_model_bottom, double(move.position.z()) - double(move.height));
    CHECK_THAT(first_model_bottom - slicing.raft_contact_top_z, Catch::Matchers::WithinAbs(slicing.gap_raft_object, 1e-3));
}

// Body Split on a raft failed late with internal messages (SRL-A38, SRL-C02 code 10, the
// tower's structure check, empty layers). It is refused at validation instead, naming the setting.
TEST_CASE("Body Split on a raft is refused, naming Raft layers", "[TestRebuild][Raft]")
{
    CadenceTest::Scene scene = CadenceTest::body_coupon();
    const auto validate = [](CadenceTest::Scene checked) {
        fill_per_filament_values(checked.config);
        Model model;
        Print print;
        print.is_BBL_printer() = checked.bambu;
        checked.populate(model, print, checked.config);
        return print.validate();
    };
    {
        const StringObjectException plain = validate(scene);
        INFO(plain.string);
        CHECK(plain.string.empty());
    }
    scene.config.set_key_value("raft_layers", new ConfigOptionInt(2));
    // The raft's base on the fine nozzle.
    scene.config.set_key_value("support_filament", new ConfigOptionInt(1));
    scene.config.set_key_value("support_interface_filament", new ConfigOptionInt(1));
    const StringObjectException refused = validate(scene);
    INFO(refused.string);
    REQUIRE_FALSE(refused.string.empty());
    CHECK(refused.string.find("[SRL-A54]") != std::string::npos);
    CHECK(refused.opt_key == "raft_layers");
    CHECK(refused.string.find("Raft layers") != std::string::npos);
}
