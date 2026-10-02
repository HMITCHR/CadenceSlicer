#pragma once

#include "mixed_nozzle_harness.hpp"
#include "libslic3r/PresetBundle.hpp"
#include <catch2/catch_all.hpp>
#include <regex>
#include <sstream>

namespace CadenceTest {
struct OtherPrinterProfile
{
    std::string              vendor;
    std::string              printer;
    std::string              process;
    std::vector<std::string> filaments;
};

inline DynamicPrintConfig compose_profile(const OtherPrinterProfile &profile)
{
    PresetBundle library, bundle;
    library.load_vendor_configs_from_json(PROFILES_DIR, PresetBundle::ORCA_FILAMENT_LIBRARY, PresetBundle::LoadSystem,
                                          ForwardCompatibilitySubstitutionRule::Disable);
    bundle.load_vendor_configs_from_json(PROFILES_DIR, profile.vendor, PresetBundle::LoadSystem,
                                         ForwardCompatibilitySubstitutionRule::Disable, &library);
    REQUIRE(bundle.printers.select_preset_by_name(profile.printer, true));
    REQUIRE(bundle.prints.select_preset_by_name(profile.process, true));
    bundle.set_num_filaments(unsigned(profile.filaments.size()), std::string("#FFFFFF"));
    bundle.filament_presets = profile.filaments;
    REQUIRE(bundle.filaments.select_preset_by_name(profile.filaments.front(), true));
    DynamicPrintConfig config = bundle.full_config();
    const size_t nozzles = config.option<ConfigOptionFloats>("nozzle_diameter")->values.size();
    const size_t filaments = profile.filaments.size();
    auto *matrix = config.option<ConfigOptionFloats>("flush_volumes_matrix", true);
    if (matrix->values.size() != nozzles * filaments * filaments) {
        matrix->values.assign(nozzles * filaments * filaments, 140.);
        for (size_t nozzle = 0; nozzle < nozzles; ++nozzle)
            for (size_t filament = 0; filament < filaments; ++filament)
                matrix->values[nozzle * filaments * filaments + filament * filaments + filament] = 0.;
        config.option<ConfigOptionFloats>("flush_multiplier", true)->values.assign(nozzles, 1.);
    }
    return config;
}

inline void apply_other_printer_setup(DynamicPrintConfig &config, const Vec2d &tower_xy)
{
    config.set_key_value("wipe_tower_type", new ConfigOptionEnum<WipeTowerType>(WipeTowerType::Type1));
    config.set_key_value("wipe_tower_rotation_angle", new ConfigOptionFloat(0.));
    config.set_key_value("ooze_prevention", new ConfigOptionBool(false));
    config.set_key_value("enable_prime_tower", new ConfigOptionBool(true));
    config.set_key_value("wipe_tower_x", new ConfigOptionFloats{tower_xy.x()});
    config.set_key_value("wipe_tower_y", new ConfigOptionFloats{tower_xy.y()});
    config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipGrid));
    config.set_key_value("minimum_sparse_infill_area", new ConfigOptionFloat(0.));
    config.set_key_value("wall_generator", new ConfigOptionEnum<PerimeterGeneratorType>(PerimeterGeneratorType::Classic));
    config.set_key_value("top_shell_thickness", new ConfigOptionFloat(0.));
    config.set_key_value("bottom_shell_thickness", new ConfigOptionFloat(0.));
    config.set_key_value("ensure_vertical_shell_thickness", new ConfigOptionEnum<EnsureVerticalShellThickness>(evstNone));
    config.set_key_value("elefant_foot_compensation", new ConfigOptionFloat(0.));
    config.set_key_value("detect_overhang_wall", new ConfigOptionBool(false));
    config.set_key_value("only_one_wall_top", new ConfigOptionBool(false));
    config.set_key_value("thick_internal_bridges", new ConfigOptionBool(false));
    config.set_key_value("flush_into_support", new ConfigOptionBool(false));
    config.set_key_value("prime_tower_flat_ironing", new ConfigOptionBool(false));
    config.set_key_value("wipe_tower_no_sparse_layers", new ConfigOptionBool(true));
    config.set_key_value("purge_in_prime_tower", new ConfigOptionBool(false));
    config.set_key_value("enable_filament_ramming", new ConfigOptionBool(false));
    config.set_key_value("single_extruder_multi_material_priming", new ConfigOptionBool(false));
    config.set_key_value("infill_combination", new ConfigOptionBool(false));
    config.set_key_value("infill_combination_max_layer_height", new ConfigOptionFloatOrPercent(0., false));
}

inline void install_nozzles(DynamicPrintConfig &config, const std::map<size_t, double> &installed,
                     const std::map<size_t, std::pair<double, double>> &limits)
{
    auto nozzles = config.option<ConfigOptionFloats>("nozzle_diameter")->values;
    const size_t tools = nozzles.size();
    std::vector<double> minimum(tools), maximum(tools);
    const auto *min_opt = config.option<ConfigOptionFloats>("min_layer_height");
    const auto *max_opt = config.option<ConfigOptionFloats>("max_layer_height");
    for (size_t tool = 0; tool < tools; ++tool) {
        minimum[tool] = min_opt->get_at(tool);
        maximum[tool] = max_opt->get_at(tool);
    }
    for (const auto &[tool, diameter] : installed) {
        REQUIRE(tool < tools);
        nozzles[tool] = diameter;
    }
    for (const auto &[tool, range] : limits) {
        minimum[tool] = range.first;
        maximum[tool] = range.second;
    }
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats(nozzles));
    config.set_key_value("min_layer_height", new ConfigOptionFloats(minimum));
    config.set_key_value("max_layer_height", new ConfigOptionFloats(maximum));
}

inline void identity_filament_map(DynamicPrintConfig &config)
{
    const size_t filaments = config.option<ConfigOptionFloats>("filament_diameter")->values.size();
    std::vector<int> map(filaments);
    for (size_t index = 0; index < filaments; ++index)
        map[index] = int(index + 1);
    config.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
    config.set_key_value("filament_map", new ConfigOptionInts(map));
}

inline void feature_split_heights(DynamicPrintConfig &config, double fine, double coarse, int ratio, int fine_filament,
                           int coarse_filament, const std::vector<double> &pair)
{
    config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::FeatureSplit));
    config.set_key_value("layer_height", new ConfigOptionFloat(fine));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(fine));
    config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(coarse));
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{ratio});
    config.set_key_value("mixed_nozzle_process_nozzle_diameters", new ConfigOptionFloats(pair));
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id", "internal_solid_filament_id",
                            "top_surface_filament_id", "bottom_surface_filament_id"})
        config.set_key_value(key, new ConfigOptionInt(fine_filament));
    config.set_key_value("sparse_infill_filament_id", new ConfigOptionInt(coarse_filament));
}

inline void body_split_heights(DynamicPrintConfig &config, double fine, double coarse, int ratio, const std::vector<double> &pair)
{
    config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::BodySplit));
    config.set_key_value("layer_height", new ConfigOptionFloat(fine));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(fine));
    config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(coarse));
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{ratio});
    config.set_key_value("mixed_nozzle_process_nozzle_diameters", new ConfigOptionFloats(pair));
}

inline void load_feature_box(Model &model, Print &print, const DynamicPrintConfig &config, const Vec2d &centre)
{
    ModelObject *object = model.add_object();
    object->name = "other-printer-feature-box";
    object->add_volume(make_cube(80., 80., 6.), ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    object->instances.front()->set_offset(Vec3d(centre.x() - 40., centre.y() - 40., 0.));
    object->ensure_on_bed();
    print.apply(model, config);
    print.set_status_silent();
}

inline void load_body_pair(Model &model, Print &print, const DynamicPrintConfig &config, const Vec2d &centre,
                    int fine_filament, int coarse_filament, double coarse_height)
{
    ModelObject *object = model.add_object();
    object->name = "other-printer-body-pair";
    ModelVolume *fine = object->add_volume(make_cube(22., 50., 30.), ModelVolumeType::MODEL_PART, false);
    fine->config.set_key_value("extruder", new ConfigOptionInt(fine_filament));
    ModelVolume *coarse = object->add_volume(make_cube(22., 50., 30.), ModelVolumeType::MODEL_PART, false);
    coarse->set_offset(Vec3d(28., 0., 0.));
    coarse->config.set_key_value("extruder", new ConfigOptionInt(coarse_filament));
    coarse->config.set_key_value("regional_layer_height", new ConfigOptionFloat(coarse_height));
    object->add_instance();
    object->instances.front()->set_offset(Vec3d(centre.x() - 25., centre.y() - 25., 0.));
    object->ensure_on_bed();
    print.apply(model, config);
    print.set_status_silent();
}

const OtherPrinterProfile snapmaker_j1 {"Snapmaker", "Snapmaker J1 (0.2 nozzle)", "0.10 Standard @Snapmaker J1 (0.2 nozzle)",
                                       {"Snapmaker J1 PLA", "Snapmaker J1 PLA"}};
const OtherPrinterProfile prusa_xl_5t {"Prusa", "Prusa XL 5T 0.25 nozzle", "0.07mm Detail @Prusa XL 5T 0.25",
                                      {"Prusa Generic PLA @XL 5T", "Prusa Generic PLA @XL 5T", "Prusa Generic PLA @XL 5T",
                                       "Prusa Generic PLA @XL 5T", "Prusa Generic PLA @XL 5T"}};
const OtherPrinterProfile snapmaker_u1 {"Snapmaker", "Snapmaker U1 (0.4+0.6 nozzle)", "0.20 Standard @Snapmaker U1 (0.4+0.6 nozzle)",
                                       {"Snapmaker PLA SnapSpeed @U1", "Snapmaker PLA SnapSpeed @U1",
                                        "Snapmaker PLA SnapSpeed @U1", "Snapmaker PLA SnapSpeed @U1"}};

inline DynamicPrintConfig j1_config()
{
    DynamicPrintConfig config = compose_profile(snapmaker_j1);
    install_nozzles(config, {{0, 0.2}, {1, 0.6}}, {{0, {0.06, 0.14}}, {1, {0.15, 0.42}}});
    identity_filament_map(config);
    apply_other_printer_setup(config, Vec2d(230., 120.));
    return config;
}

inline DynamicPrintConfig xl_config()
{
    DynamicPrintConfig config = compose_profile(prusa_xl_5t);
    install_nozzles(config, {{0, 0.25}, {1, 0.6}}, {{0, {0.05, 0.15}}, {1, {0.15, 0.45}}});
    identity_filament_map(config);
    apply_other_printer_setup(config, Vec2d(300., 300.));
    return config;
}


struct ToolChange
{
    double z { 0. };
    int    from { -1 };
    int    to { -1 };
    bool   inside_tower_block { false };
    double purged_mm { 0. };
};

struct GcodeFacts
{
    std::set<long long>       coarse_tops_um;   // band and cell tops on the coarse tool, in micrometres
    std::vector<ToolChange>   changes;
    std::set<int>             tools_selected;
    std::map<std::string, int> bambu_only;
    std::vector<std::string>  temperature_lines; // tower "M104 T<n> S<s> N0" lines
};

inline long long to_um(double z) { return std::llround(z * 1000.); }

inline GcodeFacts scan_gcode(const std::string &gcode, int coarse_tool)
{
    static const std::regex band(R"(^; SRL_(?:BAND|CELL) .*tool=(\d+) .*z_lo=([0-9.]+) z_hi=([0-9.]+))");
    static const std::regex z_line(R"(^;Z:([0-9.]+))");
    static const std::regex tool_line(R"(^T(\d+)(?:[\s;].*)?$)");
    static const std::regex extrude(R"(^G1 .*E(-?[0-9.]+))");
    static const std::regex tower_temperature(R"(^M104 T(\d+) S(\d+) N0)");
    static const char *const bambu_tokens[] = {"M620", "M621", "M622", "M623", "M1002", "M991", "M993", "M9833", "M632", "M400 U1"};

    GcodeFacts facts;
    std::istringstream in(gcode);
    std::string line;
    bool   in_body = false, in_tower_block = false;
    double z = 0.;
    int    tool = -1;
    ToolChange *open = nullptr;
    std::optional<size_t> changes_before_end;
    std::smatch match;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (std::regex_search(line, match, band) && std::stoi(match[1]) == coarse_tool)
            facts.coarse_tops_um.insert(to_um(std::stod(match[3])));
        if (line.rfind(";LAYER_CHANGE", 0) == 0)
            in_body = true;
        if (line.rfind("; filament end gcode", 0) == 0)
            changes_before_end = facts.changes.size();
        if (std::regex_search(line, match, z_line))
            z = std::stod(match[1]);
        for (const char *token : bambu_tokens)
            if (line.rfind(token, 0) == 0)
                ++facts.bambu_only[token];
        if (std::regex_search(line, match, tower_temperature))
            facts.temperature_lines.push_back(line);
        if (line.rfind("; CP TOOLCHANGE START", 0) == 0)
            in_tower_block = true;
        if (line.rfind("; CP TOOLCHANGE END", 0) == 0) {
            in_tower_block = false;
            open = nullptr;
        }
        if (std::regex_search(line, match, tool_line)) {
            const int next = std::stoi(match[1]);
            if (in_body) {
                facts.tools_selected.insert(next);
                if (tool >= 0 && next != tool) {
                    facts.changes.push_back({z, tool, next, in_tower_block, 0.});
                    open = &facts.changes.back();
                }
            }
            tool = next;
            continue;
        }
        if (open != nullptr && std::regex_search(line, match, extrude)) {
            const double e = std::stod(match[1]);
            if (e > 0.)
                open->purged_mm += e;
        }
    }
    if (changes_before_end && *changes_before_end < facts.changes.size())
        facts.changes.resize(*changes_before_end);
    return facts;
}

} // namespace CadenceTest
