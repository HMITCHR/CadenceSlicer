// Guards for the build 33 plater findings: project state that must survive preset reloads and
// Undo on a mixed-nozzle project.

#include <catch2/catch_all.hpp>

#include "libslic3r/AppConfig.hpp"
#include "libslic3r/MixedNozzleConfig.hpp"
#include "libslic3r/PresetBundle.hpp"

#include "plugin_test_utils.hpp"
#include "wizard_apply_support.hpp"

#include <boost/filesystem.hpp>

#include <map>
#include <string>
#include <vector>

using namespace Slic3r;

namespace {

const std::string H2D_02 = "Bambu Lab H2D 0.2 nozzle";
const std::string FEATURE_02_08 = "MN Feature 0.2-0.8 0.08-0.32 @BBL";

// A data dir whose system profiles are the shipped BBL and Orca filament library ones, as the app
// installs them.
void install_system_profiles(const boost::filesystem::path &data)
{
    const boost::filesystem::path system = data / PRESET_SYSTEM_DIR;
    boost::filesystem::create_directories(system);
    for (const std::string vendor : {std::string("BBL"), std::string(PresetBundle::ORCA_FILAMENT_LIBRARY)}) {
        boost::filesystem::create_symlink(boost::filesystem::path(PROFILES_DIR) / (vendor + ".json"), system / (vendor + ".json"));
        boost::filesystem::create_directory_symlink(boost::filesystem::path(PROFILES_DIR) / vendor, system / vendor);
    }
}

// An app config with the H2D installed and the given materials ticked, printing on the 0.2 preset.
AppConfig h2d_app_config(const std::map<std::string, std::string> &materials)
{
    AppConfig config;
    config.set_vendors(AppConfig::VendorMap{{"BBL", {{"Bambu Lab H2D", {"0.2", "0.4", "0.6", "0.8"}}}}});
    config.set_section(AppConfig::SECTION_FILAMENTS, materials);
    config.set("presets", PRESET_PRINTER_NAME, H2D_02);
    return config;
}

} // namespace

TEST_CASE("P1: Add/Remove filament Confirm keeps the project's nozzle pair, process and materials",
          "[TestRebuild][B33][Plater]")
{
    ScopedDataDir data("b33-p1");
    install_system_profiles(data.dir);
    const std::map<std::string, std::string> ticked {{"Bambu PLA Basic @BBL H2D 0.2 nozzle", "true"},
                                                     {"Bambu PETG HF @BBL H2D 0.8 nozzle", "true"}};
    AppConfig app_config = h2d_app_config(ticked);

    PresetBundle bundle;
    bundle.load_presets(app_config, ForwardCompatibilitySubstitutionRule::EnableSilent);
    REQUIRE(bundle.printers.get_selected_preset_name() == H2D_02);

    // Keep both nozzles on 0.2 / 0.8, then pick the pair's Feature Split process.
    const auto pair = bundle.nozzle_config_for_diameters({0.2, 0.8});
    REQUIRE(pair.has_value());
    for (const char *key : {"nozzle_diameter", "min_layer_height", "max_layer_height"})
        bundle.project_config.set_key_value(key, pair->option(key)->clone());
    bundle.project_config.set_key_value("mixed_nozzle_slicing_mode",
        new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::FeatureSplit));
    bundle.update_compatible(PresetSelectCompatibleType::Never);
    REQUIRE(bundle.prints.select_preset_by_name(FEATURE_02_08, true));
    REQUIRE(bundle.prints.get_selected_preset().is_compatible);
    const DynamicPrintConfig project_before = bundle.project_config;
    const std::vector<std::string> materials_before = bundle.filament_presets;

    // What Confirm on the Add/Remove filament page does: the selections are exported when the page
    // opens, then the ticked materials are applied and every preset is reloaded.
    bundle.export_selections(app_config);
    std::map<std::string, std::string> added = ticked;
    added.emplace("Bambu PETG Basic @BBL H2D 0.2 nozzle", "true");
    REQUIRE(bundle.apply_vendor_config(app_config.vendors(), added, &app_config, true, "", ""));

    CHECK(bundle.printers.get_selected_preset_name() == H2D_02);
    for (const char *key : {"nozzle_diameter", "min_layer_height", "max_layer_height"}) {
        INFO(key);
        REQUIRE(bundle.project_config.has(key));
        CHECK(bundle.project_config.opt_serialize(key) == project_before.opt_serialize(key));
    }
    CHECK(bundle.prints.get_selected_preset_name() == FEATURE_02_08);
    CHECK(bundle.filament_presets == materials_before);
    CHECK(bundle.project_config.opt_enum<MixedNozzleSlicingMode>("mixed_nozzle_slicing_mode") == MixedNozzleSlicingMode::FeatureSplit);

    // A reload that comes back on another printer (the guide selecting a newly added one) does not
    // carry the pair over.
    bundle.load_presets(app_config, ForwardCompatibilitySubstitutionRule::EnableSilent,
                        {"Bambu Lab H2D", "0.4", "", ""});
    CHECK(bundle.printers.get_selected_preset_name() == "Bambu Lab H2D 0.4 nozzle");
    CHECK_FALSE(bundle.project_config.has("nozzle_diameter"));
}

TEST_CASE("P2: parts moved onto new slots and those slots' nozzle change in one apply reach the print",
          "[TestRebuild][B33][Plater]")
{
    using namespace Slic3r::GUI::WizardTest;
    ProfileSpec spec = h2d_02_06_spec();
    spec.materials.assign(4, "Bambu PLA Basic @BBL H2D 0.2 nozzle");
    WizardProject project(spec);
    DynamicPrintConfig &plate_config = *project.plate.config();
    plate_config.set_key_value("filament_map", new ConfigOptionInts{1, 2, 1, 1});
    ModelObject *object = project.add_object("Cube", {{"Cube", {20., 20., 20.}, {0., 0., 0.}, 1},
                                                      {"Cylinder", {20., 20., 20.}, {20., 0., 0.}, 1}},
                                             Vec2d(150., 150.));

    Print print;
    print.set_status_silent();
    print.is_BBL_printer() = project.bundle.is_bbl_vendor();
    print.set_plate_origin(project.plate.get_origin());
    print.apply(project.model, plate_slice_config(project));
    REQUIRE(print.config().filament_map.values == std::vector<int>{1, 2, 1, 1});

    // What Body Split Apply commits at once: the parts on slots 3 and 4, and slot 4 on the right nozzle.
    object->volumes[0]->config.set_key_value("extruder", new ConfigOptionInt(3));
    object->volumes[1]->config.set_key_value("extruder", new ConfigOptionInt(4));
    plate_config.set_key_value("filament_map", new ConfigOptionInts{1, 2, 1, 2});
    print.apply(project.model, plate_slice_config(project));
    CHECK(print.config().filament_map.values == std::vector<int>{1, 2, 1, 2});
    // A second apply of the same state, as the next Slice does, keeps it.
    print.apply(project.model, plate_slice_config(project));
    CHECK(print.config().filament_map.values == std::vector<int>{1, 2, 1, 2});
}
