// Setup Apply keeps the project settings the user chose: bed type, support settings and support
// filaments, also when Apply moves the printer to another variant.

#include <catch2/catch_all.hpp>

#include "libslic3r/MixedNozzleConfig.hpp"
#include "libslic3r/PresetBundle.hpp"

#include "wizard_apply_support.hpp"

#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::GUI::WizardTest;

TEST_CASE("Setup Apply that moves to another printer variant keeps the bed type and support settings",
          "[TestRebuild][WizardApply]")
{
    // A 0.2 / 0.2 H2D on the Engineering Plate; setup is opened for a 0.4 / 0.6 pair, so Apply
    // moves the printer preset to the 0.4 variant. A third slot holds the support interface.
    ProfileSpec spec = h2d_02_06_spec();
    spec.nozzles = {0.2, 0.2};
    spec.min_heights = {0.04, 0.04};
    spec.max_heights = {0.14, 0.14};
    spec.materials.push_back(spec.materials.front());
    spec.also_visible_materials = {"Bambu PLA Basic @BBL H2D", "Bambu PLA Basic @BBL H2D 0.6 nozzle"};
    WizardProject project(spec);
    // The 0.4 variant is installed, as the H2D's variants are in the app.
    Preset *variant = project.bundle.printers.find_preset("Bambu Lab H2D 0.4 nozzle", false, true);
    REQUIRE(variant != nullptr);
    variant->is_visible = true;
    project.add_object("box", {{"box", Vec3d(60., 60., 6.), Vec3d::Zero(), 1}}, Vec2d(60., 60.));
    project.bundle.project_config.set_key_value("curr_bed_type", new ConfigOptionEnum<BedType>(btEP));
    DynamicPrintConfig &process = project.bundle.prints.get_edited_preset().config;
    process.set_key_value("enable_support", new ConfigOptionBool(true));
    process.set_key_value("support_type", new ConfigOptionEnum<SupportType>(stTreeAuto));
    process.set_key_value("support_style", new ConfigOptionEnum<SupportMaterialStyle>(smsTreeOrganic));
    process.set_key_value("support_filament", new ConfigOptionInt(2));
    process.set_key_value("support_interface_filament", new ConfigOptionInt(3));
    process.set_key_value("support_threshold_angle", new ConfigOptionInt(40));
    process.set_key_value("support_on_build_plate_only", new ConfigOptionBool(true));
    process.set_key_value("support_interface_top_layers", new ConfigOptionInt(3));
    process.set_key_value("support_base_pattern", new ConfigOptionEnum<SupportMaterialPattern>(smpHoneycomb));
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

    INFO("printer after Apply: " << project.bundle.printers.get_edited_preset().name
         << ", process " << project.bundle.prints.get_edited_preset().name);
    REQUIRE(project.bundle.printers.get_edited_preset().name != printer_before);
    CHECK(project.bundle.project_config.opt_enum<BedType>("curr_bed_type") == btEP);
    CHECK(project.plate.get_bed_type(false) == btDefault);
    const DynamicPrintConfig full = project.bundle.full_config();
    CHECK(full.opt_bool("enable_support"));
    CHECK(full.opt_enum<SupportType>("support_type") == stTreeAuto);
    CHECK(full.opt_enum<SupportMaterialStyle>("support_style") == smsTreeOrganic);
    CHECK(full.opt_int("support_filament") == 2);
    CHECK(full.opt_int("support_interface_filament") == 3);
    CHECK(full.opt_int("support_threshold_angle") == 40);
    CHECK(full.opt_bool("support_on_build_plate_only"));
    CHECK(full.opt_int("support_interface_top_layers") == 3);
    CHECK(full.opt_enum<SupportMaterialPattern>("support_base_pattern") == smpHoneycomb);

    // The bed type reaches the slice. Tree support with its interface on the coarse nozzle is a refusal of its
    // own, so this slice runs without support.
    project.bundle.prints.get_edited_preset().config.set_key_value("enable_support", new ConfigOptionBool(false));
    const PlateSlice slice = admitted_slice(project);
    CHECK(slice.config.opt_enum<BedType>("curr_bed_type") == btEP);
    CHECK(slice.gcode.find("; curr_bed_type = Engineering Plate") != std::string::npos);
}
