// Guards for the rc5 finding: a 0.2 Standard / 0.6 High Flow H2D sliced its right nozzle as Standard
// after the nozzle pair changed from 0.2 / 0.8.

#include <catch2/catch_all.hpp>

#include "libslic3r/PresetBundle.hpp"

#include "slic3r/GUI/ProjectNozzleFlowState.hpp"

#include "wizard_apply_support.hpp"

#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::GUI;
using namespace Slic3r::GUI::WizardTest;

namespace {

const std::vector<int> kStandardHighFlow{int(nvtStandard), int(nvtHighFlow)};

// The owner's project before the swap: the H2D 0.2 preset, a 0.2 Standard / 0.8 High Flow pair, slot 1
// on the left and slot 2 on the right, the plate's volume map following the flows as sync writes it.
ProfileSpec h2d_02_08_spec()
{
    ProfileSpec spec = h2d_02_06_spec();
    spec.nozzles = {0.2, 0.8};
    spec.min_heights = {0.04, 0.16};
    spec.max_heights = {0.14, 0.56};
    spec.also_visible_materials = {"Bambu PLA Basic @BBL H2D 0.6 nozzle", "Bambu PLA Basic @BBL H2D 0.8 nozzle"};
    return spec;
}

void make_high_flow_right(WizardProject &project)
{
    project.bundle.project_config.set_key_value("nozzle_volume_type", new ConfigOptionEnumsGeneric(kStandardHighFlow));
    project.bundle.project_config.set_key_value("filament_volume_map", new ConfigOptionInts(kStandardHighFlow));
    project.plate.set_filament_volume_maps(kStandardHighFlow);
    project.add_object("box", {{"box", Vec3d(20., 20., 4.), Vec3d::Zero(), 1}}, Vec2d(100., 100.));
}

SetupChoice pair_choice(const PresetBundle &bundle, const std::vector<double> &pair)
{
    const auto physical = bundle.nozzle_config_for_diameters(pair);
    REQUIRE(physical.has_value());
    SetupChoice choice;
    choice.fine_height = 0.10;
    choice.ratio = 3;
    choice.requested_nozzles = pair;
    choice.requested_min_heights = physical->option<ConfigOptionFloats>("min_layer_height")->values;
    choice.requested_max_heights = physical->option<ConfigOptionFloats>("max_layer_height")->values;
    return choice;
}

std::string gcode_value(const std::string &gcode, const std::string &key)
{
    const std::string head = "; " + key + " = ";
    const std::size_t at = gcode.find(head);
    if (at == std::string::npos)
        return "absent";
    const std::size_t from = at + head.size();
    return gcode.substr(from, gcode.find('\n', from) - from);
}

void require_high_flow_right(WizardProject &project)
{
    CHECK(project.bundle.project_config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type")->values ==
          kStandardHighFlow);
    const PlateSlice slice = slice_plate(project);
    INFO(slice.refusal);
    REQUIRE(slice.refusal.empty());
    CHECK(gcode_value(slice.gcode, "nozzle_volume_type") == "Standard,High Flow");
    CHECK(gcode_value(slice.gcode, "filament_volume_map") == "0,1");
    // PLA Basic on a 0.6 or 0.8 nozzle: Standard 30, High Flow 40.
    CHECK(gcode_value(slice.gcode, "filament_max_volumetric_speed") == "2,40");
}

} // namespace

TEST_CASE("RC5-1: the 0.2 / 0.8 High Flow project slices High Flow before the swap", "[TestRebuild][RC5]")
{
    WizardProject project(h2d_02_08_spec());
    make_high_flow_right(project);
    run_wizard_setup(project, pair_choice(project.bundle, {0.2, 0.8}));
    require_high_flow_right(project);
}

TEST_CASE("RC5-2: setup for a 0.2 / 0.6 High Flow pair keeps High Flow on the right through to Slice",
          "[TestRebuild][RC5]")
{
    WizardProject project(h2d_02_08_spec());
    make_high_flow_right(project);
    // Sync's Adopt, or the sidebar's Keep both then setup: the new pair enters setup.
    run_wizard_setup(project, pair_choice(project.bundle, {0.2, 0.6}));
    CHECK(project.bundle.project_config.option<ConfigOptionFloats>("nozzle_diameter")->values ==
          std::vector<double>{0.2, 0.6});
    require_high_flow_right(project);

    // And again when the owner re-runs setup on the pair the project now has.
    SetupChoice again;
    again.fine_height = 0.10;
    again.ratio = 3;
    run_wizard_setup(project, again);
    require_high_flow_right(project);
}

TEST_CASE("RC5-3: adopting the printer's nozzles never turns High Flow into Standard unasked",
          "[TestRebuild][RC5]")
{
    const int standard = int(nvtStandard);
    const int high_flow = int(nvtHighFlow);
    // The printer reads Standard for a hotend it has not identified (DevNozzle's default flow), so a
    // Standard reading over the project's High Flow is confirmed first.
    CHECK(adopted_flows_need_confirmation({standard, high_flow}, {nvtStandard, nvtStandard}));
    CHECK(adopted_flows_need_confirmation({high_flow, standard}, {nvtStandard, nvtStandard}));
    // A High Flow reading is what the owner said yes to by adopting: no second prompt.
    CHECK_FALSE(adopted_flows_need_confirmation({standard, standard}, {nvtStandard, nvtHighFlow}));
    CHECK_FALSE(adopted_flows_need_confirmation({standard, high_flow}, {nvtStandard, nvtHighFlow}));
    CHECK_FALSE(adopted_flows_need_confirmation({standard, standard}, {nvtStandard, nvtStandard}));
}

// RC5 R1: the selected printer preset is one nozzle size's profile, so both extruders took that
// size's retraction (0.4 mm on the H2D 0.2 preset). Each nozzle of a project-owned pair takes the
// retraction family of its own nozzle size's profile: H2D 0.2 -> 0.4, 0.6 -> 1.4, 0.8 -> 3 mm.
TEST_CASE("RC5-5: each nozzle of a dissimilar pair retracts as its own nozzle size's printer profile",
          "[TestRebuild][RC5][RC5I]")
{
    const bool coarse_08 = GENERATE(false, true);
    CAPTURE(coarse_08);
    WizardProject project(coarse_08 ? h2d_02_08_spec() : h2d_02_06_spec());
    project.add_object("box", {{"box", Vec3d(20., 20., 4.), Vec3d::Zero(), 1}}, Vec2d(100., 100.));
    run_wizard_setup(project, pair_choice(project.bundle, coarse_08 ? std::vector<double>{0.2, 0.8} : std::vector<double>{0.2, 0.6}));
    {
        const PlateSlice slice = slice_plate(project);
        INFO(slice.refusal);
        REQUIRE(slice.refusal.empty());
        CHECK(gcode_value(slice.gcode, "retraction_length") == (coarse_08 ? "0.4,3" : "0.4,1.4"));
    }
    // A value the user set on the selected preset for the right extruder is kept.
    {
        auto *lengths = project.bundle.printers.get_edited_preset().config.option<ConfigOptionFloats>("retraction_length");
        const auto *ids = project.bundle.printers.get_edited_preset().config.option<ConfigOptionInts>("printer_extruder_id");
        REQUIRE(lengths != nullptr);
        REQUIRE(ids != nullptr);
        REQUIRE(lengths->values.size() == ids->values.size());
        for (size_t column = 0; column < ids->values.size(); ++column)
            if (ids->values[column] == 2)
                lengths->values[column] = 2.;
        const PlateSlice slice = slice_plate(project);
        INFO(slice.refusal);
        REQUIRE(slice.refusal.empty());
        CHECK(gcode_value(slice.gcode, "retraction_length") == "0.4,2");
    }
}

TEST_CASE("RC5-6: a pair of equal nozzles keeps the selected preset's retraction", "[TestRebuild][RC5][RC5I]")
{
    ProfileSpec spec = h2d_02_06_spec();
    spec.nozzles = {0.2, 0.2};
    spec.min_heights = {0.04, 0.04};
    spec.max_heights = {0.14, 0.14};
    WizardProject project(spec);
    const DynamicPrintConfig full = project.bundle.full_config();
    const ConfigOption *lengths = full.option("retraction_length");
    REQUIRE(lengths != nullptr);
    INFO(lengths->serialize());
    CHECK(lengths->serialize().find("1.4") == std::string::npos);
    CHECK(lengths->serialize().find("0.4") != std::string::npos);
}
