// The stock printer dialogs in Feature Split and Body Split, replayed from a real H2D with two AMS:
// AMS "A" (id 0, info 10001101) feeds the left 0.2 nozzle, AMS "B" (id 1, info 10001003) the right
// 0.6. A2 black, A3 gray, A4 yellow, B3 magenta. Every material has a fixed nozzle there, so it may
// only take a tray of the AMS that feeds that nozzle.

// Include order as in test_dev_mapping.cpp (Windows.h first, wx/timer.h before DeviceManager.hpp).
#ifdef WIN32
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #include <Windows.h>
#endif

#include <catch2/catch_all.hpp>

#include <wx/timer.h>

#include "slic3r/GUI/DeviceManager.hpp"
#include "slic3r/GUI/DeviceCore/DevFilaSystem.h"
#include "slic3r/GUI/DeviceCore/DevMapping.h"
#include "slic3r/GUI/MixedNozzlePrinterDialogs.hpp"

#include "wizard_apply_support.hpp"

#include <nlohmann/json.hpp>

#include <map>
#include <string>
#include <vector>

using json = nlohmann::json;
using namespace Slic3r;
using namespace Slic3r::GUI;

namespace {

// The two AMS units as the printer reports them. tray_exist_bits: bit ams_id * 4 + tray_id.
void load_two_ams_h2d(MachineObject &obj)
{
    const json push = json::parse(R"({
        "ams": {
            "tray_exist_bits": "4E",
            "ams": [
                { "id": "0", "info": "10001101", "tray": [
                    { "id": "1", "tray_color": "000000FF" },
                    { "id": "2", "tray_color": "808080FF" },
                    { "id": "3", "tray_color": "F4EE2AFF" } ] },
                { "id": "1", "info": "10001003", "tray": [
                    { "id": "2", "tray_color": "FF00FFFF" } ] }
            ]
        }
    })");
    DevFilaSystemParser::ParseV1_0(push, &obj, obj.GetFilaSystem().get(), false);
    // The tray type comes from the GUI preset bundle in the app; set it directly (test_dev_mapping.cpp).
    for (const auto &[ams, slot] : std::vector<std::pair<const char *, const char *>>{{"0", "1"}, {"0", "2"}, {"0", "3"}, {"1", "2"}}) {
        DevAmsTray *tray = obj.GetFilaSystem()->GetAmsTray(ams, slot);
        REQUIRE(tray != nullptr);
        tray->m_fila_type = "PLA";
    }
}

FilamentInfo pla(int id, const std::string &color)
{
    FilamentInfo info;
    info.id = id;
    info.type = "PLA";
    info.color = color;
    return info;
}

const FilamentInfo &result_for(const std::vector<FilamentInfo> &result, int id)
{
    for (const FilamentInfo &info : result)
        if (info.id == id)
            return info;
    FAIL("no mapping result for filament " << id);
    return result.front();
}

} // namespace

TEST_CASE("Synchronize matches each material only against the AMS that feeds its nozzle", "[SyncAms][MixedNozzle]")
{
    MachineObject obj(nullptr, nullptr, "test", "test_dev", "127.0.0.1");
    load_two_ams_h2d(obj);
    const auto &ams = obj.GetFilaSystem()->GetAmsList();
    REQUIRE(ams.at("0")->GetExtruderId() == DEPUTY_EXTRUDER_ID); // left
    REQUIRE(ams.at("1")->GetExtruderId() == MAIN_EXTRUDER_ID);   // right

    // paint-walls-block: black, white and green on the left nozzle, yellow (slot 7) on the right.
    const std::vector<FilamentInfo> filaments{pla(0, "#000000FF"), pla(1, "#FFFFFFFF"), pla(2, "#057748FF"),
                                              pla(6, "#F4EE2AFF")};
    const std::vector<int> filament_map{1, 1, 1, 1, 1, 1, 2, 1};

    // Stock matching uses both AMS units: the right nozzle's yellow lands on A4, which feeds the left
    // nozzle. This is what mode Off still does.
    std::vector<FilamentInfo> stock;
    REQUIRE(DevMappingUtil::ams_filament_mapping(&obj, filaments, stock, {true, true, true, true}, {}, true) == 0);
    CHECK(result_for(stock, 6).ams_id == "0");

    std::vector<FilamentInfo> result;
    REQUIRE(mixed_nozzle_ams_mapping_by_nozzle(&obj, filaments, filament_map, result, true, true) == 0);
    REQUIRE(result.size() == filaments.size());
    for (std::size_t i = 1; i < result.size(); ++i)
        CHECK(result[i - 1].id < result[i].id);
    for (int id : {0, 1, 2}) {
        CAPTURE(id);
        const FilamentInfo &left = result_for(result, id);
        CHECK(left.ams_id == "0");
        CHECK(left.tray_id >= 0);
    }
    CHECK(result_for(result, 0).slot_id == "1"); // black on A2
    const FilamentInfo &right = result_for(result, 6);
    CHECK(right.ams_id == "1");
    CHECK(right.slot_id == "2"); // B3, the only tray on the right
    // Synchronize lists only these materials in use; mode Off lists every project material, as stock.
    const std::vector<int> used{1, 2, 3, 7};
    CHECK(mixed_nozzle_sync_lists_filament(MixedNozzleSlicingMode::FeatureSplit, used, 6));
    CHECK_FALSE(mixed_nozzle_sync_lists_filament(MixedNozzleSlicingMode::BodySplit, used, 3));
    CHECK(mixed_nozzle_sync_lists_filament(MixedNozzleSlicingMode::Off, used, 3));

    SECTION("a material on a nozzle without a usable tray stays unmapped") {
        // A material on neither nozzle (no map entry) is not matched at all.
        std::vector<FilamentInfo> unplaced;
        REQUIRE(mixed_nozzle_ams_mapping_by_nozzle(&obj, {pla(0, "#000000FF"), pla(9, "#FF00FFFF")}, filament_map,
                                                   unplaced, true, true) == 0);
        CHECK(result_for(unplaced, 0).ams_id == "0");
        CHECK(result_for(unplaced, 9).tray_id == -1);
        // A PETG on the right nozzle: the right AMS holds only PLA and the left AMS is not offered.
        FilamentInfo petg = pla(6, "#F4EE2AFF");
        petg.type = "PETG";
        std::vector<FilamentInfo> typed;
        mixed_nozzle_ams_mapping_by_nozzle(&obj, {petg}, filament_map, typed, false, true);
        REQUIRE(typed.size() == 1);
        CHECK(typed[0].ams_id != "0");
    }
}

TEST_CASE("Filament sync reports the tray each material came from", "[SyncAms][MixedNozzle]")
{
    using namespace Slic3r::GUI::WizardTest;
    WizardProject project(h2d_02_06_spec());
    PresetBundle &bundle = project.bundle;
    const Preset *pla_basic = bundle.filaments.find_preset("Bambu PLA Basic @BBL H2D 0.2 nozzle", false, true);
    REQUIRE(pla_basic != nullptr);
    const std::string pla_id = pla_basic->filament_id;
    REQUIRE_FALSE(pla_id.empty());

    auto tray = [&](const std::string &ams_id, const std::string &slot_id, const std::string &color, bool loaded) {
        DynamicPrintConfig config;
        config.set_key_value("filament_id", new ConfigOptionStrings{loaded ? pla_id : std::string()});
        config.set_key_value("tag_uid", new ConfigOptionStrings{std::string()});
        config.set_key_value("ams_id", new ConfigOptionStrings{ams_id});
        config.set_key_value("slot_id", new ConfigOptionStrings{slot_id});
        config.set_key_value("filament_type", new ConfigOptionStrings{loaded ? std::string("PLA") : std::string()});
        config.set_key_value("tray_name", new ConfigOptionStrings{std::string(1, char('A' + std::stoi(ams_id))) + std::to_string(std::stoi(slot_id) + 1)});
        config.set_key_value("filament_colour", new ConfigOptionStrings{color});
        config.set_key_value("filament_multi_colour", new ConfigOptionStrings{});
        config.set_key_value("filament_colour_type", new ConfigOptionStrings{std::string("1")});
        config.set_key_value("filament_exist", new ConfigOptionBools{loaded});
        config.set_key_value("filament_slot_placeholder", new ConfigOptionBools{false});
        return config;
    };
    bundle.filament_ams_list = {
        {0, tray("0", "0", "#00000000", false)},
        {1, tray("0", "1", "#000000", true)},
        {2, tray("0", "2", "#808080", true)},
        {3, tray("0", "3", "#F4EE2A", true)},
        {0x10000 + 6, tray("1", "2", "#FF00FF", true)},
    };

    std::vector<std::pair<DynamicPrintConfig *, std::string>> unknowns;
    MergeFilamentInfo merge;
    std::vector<int> keys;

    SECTION("Overwriting") {
        std::map<int, AMSMapInfo> maps;
        const unsigned int count = bundle.sync_ams_list(unknowns, false, maps, true, merge, false, &keys);
        REQUIRE(count == 4);
        CHECK(keys == std::vector<int>{1, 2, 3, 0x10000 + 6});
        // Sidebar::build_filament_ams_list keys: main extruder trays at 0x10000 + ams_id * 4 + slot, the
        // deputy's at ams_id * 4 + slot. H2D physical_extruder_map {1, 0}: the main (device 0) is the right.
        const std::vector<int> nozzles = mixed_nozzle_synced_filament_nozzles(keys, {1, 0});
        CHECK(nozzles == std::vector<int>{1, 1, 1, 2});
        // A printer whose main extruder is config slot 0, and one that is not a two-nozzle printer.
        CHECK(mixed_nozzle_synced_filament_nozzles(keys, {0, 1}) == std::vector<int>{2, 2, 2, 1});
        CHECK(mixed_nozzle_synced_filament_nozzles(keys, {0}) == std::vector<int>(keys.size(), 0));
        // Each synced material goes on the nozzle its AMS feeds, once.
        std::vector<int> map{1, 1, 1, 1};
        CHECK(mixed_nozzle_apply_synced_nozzles(map, nozzles));
        CHECK(map == std::vector<int>{1, 1, 1, 2});
        CHECK_FALSE(mixed_nozzle_apply_synced_nozzles(map, nozzles));
    }
    SECTION("Mapping, with the unused trays added") {
        // Two project materials: the first mapped to A2, the second left unmapped.
        std::map<int, AMSMapInfo> maps{{0, AMSMapInfo{"0", "1"}}};
        const unsigned int count = bundle.sync_ams_list(unknowns, true, maps, true, merge, false, &keys);
        REQUIRE(count == 5);
        CHECK(keys == std::vector<int>{1, -1, 2, 3, 0x10000 + 6});
        CHECK(mixed_nozzle_synced_filament_nozzles(keys, {1, 0}) == std::vector<int>{1, 0, 1, 1, 2});
    }
    SECTION("Color only reports nothing") {
        std::map<int, AMSMapInfo> maps{{0, AMSMapInfo{"0", "1"}}};
        bundle.sync_ams_list(unknowns, true, maps, true, merge, true, &keys);
        CHECK(keys.empty());
    }
}
