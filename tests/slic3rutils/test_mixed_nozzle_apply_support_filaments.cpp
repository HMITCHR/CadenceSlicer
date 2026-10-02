// Setup Apply keeps the support filaments the user chose.

#include <catch2/catch_all.hpp>

#include "libslic3r/PresetBundle.hpp"

#include "wizard_apply_support.hpp"

#include <string>

using namespace Slic3r;
using namespace Slic3r::GUI::WizardTest;

TEST_CASE("Setup Apply keeps the project's support base and interface filaments", "[TestRebuild][WizardApply]")
{
    // Fine PLA in slot 1, coarse PLA in slot 2, and a third filament for the support interface.
    ProfileSpec spec = h2d_02_06_spec();
    spec.materials.push_back(spec.materials.front());
    WizardProject project(spec);
    project.add_object("box", {{"box", Vec3d(40., 20., 10.), Vec3d::Zero(), 1}}, Vec2d(80., 80.));
    DynamicPrintConfig &process = project.bundle.prints.get_edited_preset().config;
    process.set_key_value("enable_support", new ConfigOptionBool(true));
    process.set_key_value("support_filament", new ConfigOptionInt(2));
    process.set_key_value("support_interface_filament", new ConfigOptionInt(3));

    SetupChoice choice;
    choice.mode = MixedNozzleSlicingMode::FeatureSplit;
    choice.fine_height = 0.10;
    choice.ratio = 3;
    run_wizard_setup(project, choice);

    const DynamicPrintConfig full = project.bundle.full_config();
    INFO("process after Apply: " << project.bundle.prints.get_edited_preset().name);
    CHECK(full.opt_int("support_filament") == 2);
    CHECK(full.opt_int("support_interface_filament") == 3);
}
