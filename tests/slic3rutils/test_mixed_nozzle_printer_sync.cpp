// Syncing a connected H2D's nozzles on a fresh install.

#include <catch2/catch_all.hpp>

#include "libslic3r/MixedNozzleConfig.hpp"
#include "libslic3r/PresetBundle.hpp"

#include "slic3r/GUI/MixedNozzleNativeEntry.hpp"
#include "slic3r/GUI/ProjectNozzleFlowState.hpp"

#include "wizard_apply_support.hpp"

#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::GUI;
using namespace Slic3r::GUI::WizardTest;

namespace {

// The H2D reports its nozzles in device order (main, the right one, first) as floats. The printer
// preset's physical_extruder_map puts each into its config slot, as Printer sync does.
std::vector<double> synced_machine_pair(const PresetBundle &bundle, const std::vector<float> &device)
{
    const auto *map = bundle.printers.get_edited_preset().config.option<ConfigOptionInts>("physical_extruder_map");
    const std::vector<int> mapping = map ? map->values : std::vector<int>{0, 1};
    std::vector<double> pair(2);
    for (size_t i = 0; i < 2; ++i)
        pair[mapping[i]] = device[i];
    return pair;
}

const std::vector<float> kDevice02Left08Right{0.8f, 0.2f};

std::vector<double> limits_of(const std::string &printer, const char *key)
{
    PresetBundle bundle = installed_bbl_profiles();
    const Preset *preset = bundle.printers.find_preset(printer, false, true);
    REQUIRE(preset != nullptr);
    return preset->config.option<ConfigOptionFloats>(key)->values;
}

} // namespace

TEST_CASE("Sync adopts the printer's 0.2 left and 0.8 right nozzles over an older project pair or a fresh install",
          "[TestRebuild][PrinterSync]")
{
    // A project after a first sync (the H2D 0.2 preset, a kept 0.6 / 0.8 pair), or a fresh install
    // with only the 0.4 variant installed, as a new user may have it. Mode Off.
    const bool fresh_install = GENERATE(false, true);
    CAPTURE(fresh_install);
    ProfileSpec spec = h2d_02_06_spec();
    spec.nozzles = {0.6, 0.8};
    spec.min_heights = {0.12, 0.16};
    spec.max_heights = {0.42, 0.56};
    spec.also_visible_materials = {"Bambu PLA Basic @BBL H2D 0.6 nozzle", "Bambu PLA Basic @BBL H2D 0.8 nozzle"};
    if (fresh_install) {
        spec.printer = "Bambu Lab H2D 0.4 nozzle";
        spec.process = "0.20mm Standard @BBL H2D";
        spec.materials = {"Bambu PLA Basic @BBL H2D", "Bambu PLA Basic @BBL H2D"};
        spec.also_visible_materials = {"Bambu PLA Basic @BBL H2D 0.2 nozzle", "Bambu PLA Basic @BBL H2D 0.8 nozzle"};
        spec.nozzles = {0.4, 0.4};
        spec.min_heights = limits_of(spec.printer, "min_layer_height");
        spec.max_heights = limits_of(spec.printer, "max_layer_height");
    }
    WizardProject project(spec);
    if (fresh_install)
        REQUIRE_FALSE(project.bundle.printers.find_preset("Bambu Lab H2D 0.2 nozzle", false, true)->is_visible);
    project.add_object("box", {{"box", Vec3d(20., 20., 4.), Vec3d::Zero(), 1}}, Vec2d(100., 100.));

    // "Adopt printer nozzles..." opens setup on the pair the printer reported. No profile covers a
    // pair no H2D variant holds.
    CHECK_FALSE(project.bundle.nozzle_config_for_diameters({0.3, 0.8}).has_value());
    const std::vector<double> machine_pair = synced_machine_pair(project.bundle, kDevice02Left08Right);
    const auto physical = project.bundle.nozzle_config_for_diameters(machine_pair);
    REQUIRE(physical.has_value());
    CHECK(physical->option<ConfigOptionFloats>("nozzle_diameter")->values == std::vector<double>{0.2, 0.8});
    CHECK(physical->option<ConfigOptionFloats>("min_layer_height")->values == std::vector<double>{0.04, 0.16});
    CHECK(physical->option<ConfigOptionFloats>("max_layer_height")->values == std::vector<double>{0.14, 0.56});

    SetupChoice choice;
    choice.fine_height = 0.10;
    choice.ratio = 3;
    // Setup opens on the profile's own sizes, not the device's float readings.
    choice.requested_nozzles = physical->option<ConfigOptionFloats>("nozzle_diameter")->values;
    choice.requested_min_heights = physical->option<ConfigOptionFloats>("min_layer_height")->values;
    choice.requested_max_heights = physical->option<ConfigOptionFloats>("max_layer_height")->values;
    REQUIRE(run_wizard_setup(project, choice).applied);

    // The sidebar reads the project's pair: exactly the printer's nozzles, on the 0.2 variant, which is
    // shown even when it was not installed.
    const auto *nozzles = project.bundle.project_config.option<ConfigOptionFloats>("nozzle_diameter");
    REQUIRE(nozzles != nullptr);
    CHECK(nozzles->values == std::vector<double>{0.2, 0.8});
    CHECK(project.bundle.printers.get_edited_preset().name == "Bambu Lab H2D 0.2 nozzle");
    CHECK(project.bundle.printers.get_selected_preset().is_visible);

    // And each side's flow as the printer reports it, config order: 0.2 Standard, 0.8 High Flow.
    const auto *map = project.bundle.printers.get_edited_preset().config.option<ConfigOptionInts>("physical_extruder_map");
    REQUIRE(map != nullptr);
    const std::vector<NozzleVolumeType> flows = device_sync_nozzle_volume_types(
        map->values, {0.8, 0.2}, {nvtHighFlow, nvtStandard}, {std::nullopt, std::nullopt});
    CHECK(flows == std::vector<NozzleVolumeType>{nvtStandard, nvtHighFlow});

    // A later printer refresh keeps the project's pair and the flow types the project owns with it.
    DynamicPrintConfig &project_config = project.bundle.project_config;
    project_config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true)->values = {int(nvtStandard),
                                                                                          int(nvtHighFlow)};
    REQUIRE(mixed_nozzle_sync_preserves_project(project_config, project.bundle.printers.get_edited_preset().config));
    REQUIRE(project_owns_nozzle_flows(project_config, 2));
    with_project_nozzle_flows_preserved(project_config, true, [&] { project.bundle.on_extruders_count_changed(2); });
    CHECK(project_config.option<ConfigOptionFloats>("nozzle_diameter")->values == std::vector<double>{0.2, 0.8});
    CHECK(project_config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type")->values ==
          std::vector<int>{int(nvtStandard), int(nvtHighFlow)});
}
