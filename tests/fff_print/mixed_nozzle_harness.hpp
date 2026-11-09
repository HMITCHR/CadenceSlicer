#pragma once

#include "mixed_nozzle_facts.hpp"
#include "mixed_nozzle_fixtures.hpp"
#include "test_helpers.hpp"
#include "test_utils.hpp"

#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "libslic3r/Utils.hpp"

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
    PrintEstimatedStatistics statistics;
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
    facts.statistics = processed.print_statistics;
    facts.moves = Emitted::emitted_moves(facts.gcode);
    facts.warnings = std::move(processed.warnings);
    return facts;
}

// resolved, when given, receives the config the print sliced with.
inline Facts slice(Scene scene, PrintConfig *resolved = nullptr)
{
    mixed_nozzle_fixtures::fill_per_filament_values(scene.config);
    if (scene.bambu && scene.config.opt_string("machine_start_gcode").find("M1020") == std::string::npos)
        scene.config.set_key_value("machine_start_gcode", new ConfigOptionString(scene.config.opt_string("machine_start_gcode") + "\nM1020 S[initial_extruder]\n"));
    Model model;
    Print print;
    print.is_BBL_printer() = scene.bambu;
    scene.populate(model, print, scene.config);
    print.set_status_silent();
    Facts facts = slice(print);
    if (resolved != nullptr)
        *resolved = print.config();
    return facts;
}

inline Scene feature_cube(double height = 2.16, double footprint = 80.)
{
    Scene scene;
    scene.config = mixed_nozzle_fixtures::free_switch_economics_config();
    scene.config.set_key_value("machine_start_gcode", new ConfigOptionString("G28\nM1020 S[initial_extruder]\n"));
    mixed_nozzle_fixtures::fill_per_filament_values(scene.config);
    scene.populate = [height, footprint](Model &model, Print &print, const DynamicPrintConfig &config) {
        mixed_nozzle_fixtures::init_feature_flow_fixture(model, print, config, height, footprint);
        model.objects.front()->instances.front()->set_offset(Vec3d(40., 40., 0.));
        print.apply(model, config);
    };
    return scene;
}

inline Scene body_coupon(double height = 4.)
{
    Scene scene;
    scene.config = mixed_nozzle_fixtures::body_split_pad_config(.2, .4, .1, .2, false, true, 60., wtwRib);
    mixed_nozzle_fixtures::apply_tower_exit_retraction(scene.config);
    mixed_nozzle_fixtures::fill_per_filament_values(scene.config);
    scene.populate = [height](Model &model, Print &print, const DynamicPrintConfig &config) {
        mixed_nozzle_fixtures::init_body_split_pad_coupon(model, print, config, false, height, .1, .2);
    };
    return scene;
}

// A project the app saved, from tests/data: its model, with its settings in `config`.
inline Model load_project(const std::string &file, DynamicPrintConfig &config)
{
    // The 3MF reader unpacks into the temporary directory.
    ScopedTemporaryDir unpack("cadence-3mf");
    const std::string previous = temporary_dir();
    set_temporary_dir(unpack.string());
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Enable);
    Model model = Model::read_from_file(std::string(TEST_DATA_DIR) + "/" + file, &config, &substitutions,
                                        LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
    set_temporary_dir(previous);
    return model;
}

inline Scene project_scene(const std::string &file)
{
    Scene scene;
    const auto model = std::make_shared<Model>(load_project(file, scene.config));
    scene.populate = [model](Model &out, Print &print, const DynamicPrintConfig &config) {
        out = *model;
        print.apply(out, config);
    };
    return scene;
}

// One volume of a Body Split object: a box of `size` at `offset` on `filament`, at its own layer height
// (0 leaves it unset), optionally with facets painted with `paint_filament`.
struct BodyPart {
    std::string      name;
    Vec3d            size;
    Vec3d            offset = Vec3d::Zero();
    int              filament = 1;
    double           layer_height = 0.;
    ModelVolumeType  type = ModelVolumeType::MODEL_PART;
    std::vector<int> painted_facets = {};
    int              paint_filament = 0;
};

// Body Split on a 0.2/0.6 pair with two filaments (1 on the 0.2, 2 on the 0.6), coarse layers three fine
// layers tall, and one object made of `parts` in that order.
inline Scene body_parts_scene(std::vector<BodyPart> parts, double fine_max = 0.20)
{
    Scene scene;
    scene.bambu = false;
    scene.config = mixed_nozzle_fixtures::coupon_config_with_filaments(2);
    scene.config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    scene.config.set_key_value("max_layer_height", new ConfigOptionFloats{fine_max, 0.42});
    scene.config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{3});
    scene.populate = [parts](Model &model, Print &print, const DynamicPrintConfig &config) {
        ModelObject *object = model.add_object();
        object->name = "body-parts";
        for (const BodyPart &part : parts) {
            ModelVolume *volume = object->add_volume(make_cube(part.size.x(), part.size.y(), part.size.z()), part.type, false);
            volume->name = part.name;
            volume->set_offset(part.offset);
            if (part.filament > 0)
                volume->config.set_key_value("extruder", new ConfigOptionInt(part.filament));
            if (part.layer_height > 0.)
                volume->config.set_key_value("regional_layer_height", new ConfigOptionFloat(part.layer_height));
            if (!part.painted_facets.empty())
                mixed_nozzle_fixtures::paint_cadence_facets(*volume, part.painted_facets,
                                                            EnforcerBlockerType(part.paint_filament));
        }
        object->add_instance();
        object->ensure_on_bed();
        print.apply(model, config);
        print.set_status_silent();
    };
    return scene;
}

inline bool model_road(const GCodeProcessorResult::MoveVertex &move)
{
    return move.type == EMoveType::Extrude &&
        move.extrusion_role >= erPerimeter && move.extrusion_role <= erGapFill;
}
} // namespace CadenceTest
