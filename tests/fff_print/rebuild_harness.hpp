#pragma once

#include "mixed_nozzle_facts.hpp"
#include "srl_fixtures.hpp"
#include "test_helpers.hpp"
#include "test_utils.hpp"

#include <fstream>
#include <functional>
#include <iterator>
#include <stdexcept>

namespace CadenceTest {
using namespace Slic3r;

struct Scene {
    DynamicPrintConfig config;
    std::function<void(Model &, Print &, const DynamicPrintConfig &)> populate;
    bool bambu = true;
};

struct Facts {
    std::string gcode;
    Emitted::EmittedFacts emitted;
    std::vector<GCodeProcessorResult::MoveVertex> moves;
    std::vector<GCodeProcessorResult::SliceWarning> warnings;
    std::vector<StringObjectException> validation_warnings;
    StringObjectException refusal;
    double seconds = 0.;
};

inline Facts slice(Print &print)
{
    Facts facts;
    facts.refusal = print.validate(&facts.validation_warnings);
    if (!facts.refusal.string.empty())
        return facts;
    print.process();
    ScopedTemporaryFile file(".gcode");
    GCodeProcessorResult processed;
    print.export_gcode(file.string(), &processed, nullptr);
    std::ifstream input(file.string(), std::ios::binary);
    if (!input)
        throw std::runtime_error("Cannot read exported scene");
    facts.gcode.assign(std::istreambuf_iterator<char>(input), {});
    facts.emitted = Emitted::extract_emitted_facts(facts.gcode);
    facts.seconds = processed.print_statistics.modes[size_t(PrintEstimatedStatistics::ETimeMode::Normal)].time;
    facts.moves = Emitted::emitted_moves(facts.gcode);
    facts.warnings = std::move(processed.warnings);
    return facts;
}

inline Facts slice(Scene scene)
{
    srl_fixtures::fill_per_filament_values(scene.config);
    if (scene.bambu && scene.config.opt_string("machine_start_gcode").find("M1020") == std::string::npos)
        scene.config.set_key_value("machine_start_gcode", new ConfigOptionString(scene.config.opt_string("machine_start_gcode") + "\nM1020 S[initial_extruder]\n"));
    Model model;
    Print print;
    print.is_BBL_printer() = scene.bambu;
    scene.populate(model, print, scene.config);
    print.set_status_silent();
    return slice(print);
}

inline Scene feature_cube(double height = 2.16, double footprint = 80.)
{
    Scene scene;
    scene.config = srl_fixtures::econ_whole_config();
    scene.config.set_key_value("machine_start_gcode", new ConfigOptionString("G28\nM1020 S[initial_extruder]\n"));
    srl_fixtures::fill_per_filament_values(scene.config);
    scene.populate = [height, footprint](Model &model, Print &print, const DynamicPrintConfig &config) {
        srl_fixtures::init_feature_flow_fixture(model, print, config, height, footprint);
        model.objects.front()->instances.front()->set_offset(Vec3d(40., 40., 0.));
        print.apply(model, config);
    };
    return scene;
}

inline Scene body_coupon(double height = 4.)
{
    Scene scene;
    scene.config = srl_fixtures::v24_body_split_pad_config(.2, .4, .1, .2, false, true, 60., wtwRib);
    srl_fixtures::apply_tower_exit_retraction(scene.config);
    srl_fixtures::fill_per_filament_values(scene.config);
    scene.populate = [height](Model &model, Print &print, const DynamicPrintConfig &config) {
        srl_fixtures::init_v24_body_split_pad_coupon(model, print, config, false, height, .1, .2);
    };
    return scene;
}

inline bool model_road(const GCodeProcessorResult::MoveVertex &move)
{
    return move.type == EMoveType::Extrude &&
        move.extrusion_role >= erPerimeter && move.extrusion_role <= erGapFill;
}
} // namespace CadenceTest
