// Guards for the build 40 findings: setup Apply keeps the user's support settings when it switches
// from a system process preset to an MN preset.

#include <catch2/catch_all.hpp>

#include "libslic3r/PresetBundle.hpp"

#include "wizard_apply_support.hpp"

#include <string>

using namespace Slic3r;
using namespace Slic3r::GUI::WizardTest;

TEST_CASE("B40-1: setup Apply from a system preset to an MN preset keeps the support settings", "[TestRebuild][B40]")
{
    // Fine PLA in slot 1, coarse PLA in slot 2, and a third filament for the support interface.
    ProfileSpec spec = h2d_02_06_spec();
    spec.materials.push_back(spec.materials.front());
    WizardProject project(spec);
    project.add_object("box", {{"box", Vec3d(40., 20., 10.), Vec3d::Zero(), 1}}, Vec2d(80., 80.));
    const std::string system_process = project.bundle.prints.get_edited_preset().name;
    REQUIRE(system_process.rfind("MN ", 0) != 0);
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

    SetupChoice choice;
    choice.mode = MixedNozzleSlicingMode::FeatureSplit;
    choice.fine_height = 0.10;
    choice.ratio = 3;
    run_wizard_setup(project, choice);

    const std::string after = project.bundle.prints.get_edited_preset().name;
    INFO("process " << system_process << " -> " << after);
    CHECK(after.rfind("MN ", 0) == 0);
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
}
