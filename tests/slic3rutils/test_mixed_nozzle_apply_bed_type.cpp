// Setup Apply keeps the bed type the user chose.

#include <catch2/catch_all.hpp>

#include "libslic3r/MixedNozzleConfig.hpp"
#include "libslic3r/PresetBundle.hpp"

#include "wizard_apply_support.hpp"

#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::GUI::WizardTest;

TEST_CASE("Setup Apply that moves to another printer variant keeps the project's bed type",
          "[TestRebuild][WizardApply]")
{
    // A 0.2 / 0.2 H2D on the Engineering Plate; setup is opened for a 0.4 / 0.6 pair, so Apply
    // moves the printer preset to the 0.4 variant.
    ProfileSpec spec = h2d_02_06_spec();
    spec.nozzles = {0.2, 0.2};
    spec.min_heights = {0.04, 0.04};
    spec.max_heights = {0.14, 0.14};
    spec.also_visible_materials = {"Bambu PLA Basic @BBL H2D", "Bambu PLA Basic @BBL H2D 0.6 nozzle"};
    WizardProject project(spec);
    // The 0.4 variant is installed, as the H2D's variants are in the app.
    Preset *variant = project.bundle.printers.find_preset("Bambu Lab H2D 0.4 nozzle", false, true);
    REQUIRE(variant != nullptr);
    variant->is_visible = true;
    project.add_object("box", {{"box", Vec3d(60., 60., 6.), Vec3d::Zero(), 1}}, Vec2d(60., 60.));
    project.bundle.project_config.set_key_value("curr_bed_type", new ConfigOptionEnum<BedType>(btEP));
    const std::string printer_before = project.bundle.printers.get_edited_preset().name;

    const auto pair = project.bundle.nozzle_config_for_diameters({0.4, 0.6});
    REQUIRE(pair.has_value());
    SetupChoice choice;
    choice.mode = MixedNozzleSlicingMode::FeatureSplit;
    choice.fine_height = 0.12;
    choice.ratio = 3;
    choice.requested_nozzles = std::vector<double>{0.4, 0.6};
    choice.requested_min_heights = pair->option<ConfigOptionFloats>("min_layer_height")->values;
    choice.requested_max_heights = pair->option<ConfigOptionFloats>("max_layer_height")->values;
    run_wizard_setup(project, choice);

    INFO("printer after Apply: " << project.bundle.printers.get_edited_preset().name);
    REQUIRE(project.bundle.printers.get_edited_preset().name != printer_before);
    CHECK(project.bundle.project_config.opt_enum<BedType>("curr_bed_type") == btEP);
    CHECK(project.plate.get_bed_type(false) == btDefault);

    PlateSlice slice = slice_plate(project);
    INFO("Slice refused: " << slice.refusal);
    REQUIRE(slice.refusal.empty());
    CHECK(slice.config.opt_enum<BedType>("curr_bed_type") == btEP);
    CHECK(slice.gcode.find("; curr_bed_type = Engineering Plate") != std::string::npos);
}
