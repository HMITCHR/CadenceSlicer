// The High Flow case replayed from the printer's own report: an H2D whose right nozzle is a
// 0.6 it reports as type "HS01" (Standard) and whose left is a 0.2 "HS00", both with serial "N/A",
// synced into a project that holds 0.2 Standard / 0.6 High Flow.

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

#include "fff_print/mixed_nozzle_harness.hpp"
#include "slic3r/GUI/DeviceManager.hpp"
#include "slic3r/GUI/DeviceCore/DevNozzleRack.h"
#include "slic3r/GUI/DeviceCore/DevNozzleSystem.h"
#include "slic3r/GUI/ProjectNozzleFlowState.hpp"
#include "slic3r/GUI/SelectMachine.hpp"

#include <nlohmann/json.hpp>

#include <cctype>
#include <cmath>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using json = nlohmann::json;
using namespace Slic3r;
using namespace Slic3r::GUI;

TEST_CASE("The H2D's report of a 0.6 \"HS01\" right nozzle never replaces the project's High Flow unasked",
          "[TestRebuild][HighFlow][DeviceSync]")
{
    MachineObject obj(nullptr, nullptr, "test", "test_dev", "127.0.0.1");
    // The printer's nozzle block as an H2D sent it: device extruder 0 is the right nozzle.
    const json report = json::parse(R"({
        "nozzle": {
            "info": [
                { "id": 0, "diameter": 0.6, "type": "HS01", "sn": "N/A" },
                { "id": 1, "diameter": 0.2, "type": "HS00", "sn": "N/A" }
            ]
        }
    })");
    DevNozzleSystemParser::ParseV2_0(report, obj.GetNozzleSystem());
    REQUIRE(obj.GetNozzleSystem()->GetExtNozzleCount() == 2);

    // Config order is left, right; the H2D's physical_extruder_map is {1, 0}.
    const std::vector<int> mapping{1, 0};
    std::vector<double>           machine_pair(2);
    std::vector<NozzleVolumeType> machine_flows(2);
    for (size_t slot = 0; slot < 2; ++slot) {
        const DevNozzle nozzle = obj.GetNozzleSystem()->GetExtNozzle(mapping[slot]);
        machine_pair[slot]  = std::round(double(nozzle.m_diameter) * 10.) / 10.;
        // The reading the sync path takes: "S" in the type string, so Standard.
        REQUIRE(nozzle.GetNozzleFlowType() == NozzleFlowType::S_FLOW);
        machine_flows[slot] = DevNozzle::ToNozzleVolumeType(nozzle.GetNozzleFlowType());
    }
    CHECK(machine_pair == std::vector<double>{0.2, 0.6});
    CHECK(machine_flows == std::vector<NozzleVolumeType>{nvtStandard, nvtStandard});

    // The project: left 0.2 Standard, right 0.6 High Flow.
    const std::vector<int> project{int(nvtStandard), int(nvtHighFlow)};

    // After "Adopt printer nozzles": the Standard reading over High Flow is asked about, not written.
    const AdoptedFlowPlan adopted = plan_adopted_flows(project, machine_flows, true);
    CHECK(adopted.ask);
    REQUIRE(adopted.writes.size() == 1);
    CHECK(adopted.writes.front().first == 1);
    CHECK(adopted.writes.front().second == nvtStandard);
    // "Keep project settings" (the default button): nothing is written, High Flow stays.
    CHECK_FALSE(adopted.writes_apply(false));
    CHECK_FALSE(adopted.redraw_cards(false));
    // "Use printer flow types": Standard is written and the nozzle cards are redrawn to show it.
    CHECK(adopted.writes_apply(true));
    CHECK(adopted.redraw_cards(true));

    // A plain sync of the same pair (no adopt) asks as well.
    const AdoptedFlowPlan synced = plan_adopted_flows(project, machine_flows, false);
    CHECK(synced.ask);
    CHECK_FALSE(synced.writes_apply(false));

    // A printer that does identify the hotend ("HH01") reports High Flow: nothing to write or ask.
    json identified = report;
    identified["nozzle"]["info"][0]["type"] = "HH01";
    DevNozzleSystemParser::ParseV2_0(identified, obj.GetNozzleSystem());
    const NozzleFlowType right = obj.GetNozzleSystem()->GetExtNozzle(0).GetNozzleFlowType();
    CHECK(right == NozzleFlowType::H_FLOW);
    const AdoptedFlowPlan same = plan_adopted_flows(project, {nvtStandard, DevNozzle::ToNozzleVolumeType(right)}, true);
    CHECK(same.writes.empty());
    CHECK_FALSE(same.ask);

    // What a sync writes in config order (left, right) from the device order (right first). A 0.2 mm
    // nozzle stays Standard whatever it reports; a selection from the printer's stats wins; without
    // flow support every nozzle is Standard, as upstream.
    const std::vector<double> device_pair{0.6, 0.2};
    const std::vector<std::optional<NozzleVolumeType>> device_flows{nvtHighFlow, nvtStandard};
    CHECK(device_sync_nozzle_volume_types(mapping, device_pair, device_flows, {{}, {}}) ==
          std::vector<NozzleVolumeType>{nvtStandard, nvtHighFlow});
    CHECK(device_sync_nozzle_volume_types(mapping, device_pair, {nvtHighFlow, nvtHighFlow}, {{}, {}}) ==
          std::vector<NozzleVolumeType>{nvtStandard, nvtHighFlow});
    CHECK(device_sync_nozzle_volume_types(mapping, device_pair, device_flows, {std::nullopt, nvtHybrid}) ==
          std::vector<NozzleVolumeType>{nvtHybrid, nvtHighFlow});
    CHECK(device_sync_nozzle_volume_types(mapping, device_pair, {{}, {}}, {{}, {}}) ==
          std::vector<NozzleVolumeType>{nvtStandard, nvtStandard});
}

// The send check for a hotend rack printer: before a print starts, the right extruder's sliced size and
// flow must be held by the mounted hotend or a rack hotend, or Send is blocked with "The printer has no
// nozzle matching the slicing file" (SelectMachineDialog::CheckErrorExtruderNozzleWithSlicing). Stock
// code, kept as it is. This slices an H2C plan (left 0.4, rack 0.2, fine on the rack side) in both modes
// and runs the sizes the dialog reads through the inventory lookup it uses.
TEST_CASE("An H2C plan whose right size is not in the hotend rack is held back by the send check",
          "[TestRebuild][H2C][RackCheck][SendDialog]")
{
    using namespace mixed_nozzle_fixtures;
    const bool body_split = GENERATE(false, true);
    CAPTURE(body_split);
    DynamicPrintConfig config = body_split ? body_split_pad_config(.20, .40, .10, .20, true, true, 60., wtwRib)
                                           : feature_split_tower_config(.20, .40, true);
    config.set_key_value("extruder_max_nozzle_count", new ConfigOptionIntsNullable({1, 6}));
    config.set_key_value("physical_extruder_map", new ConfigOptionInts({1, 0}));
    config.set_key_value("machine_start_gcode", new ConfigOptionString("G28\nM1020 S[initial_extruder]\n"));
    Model model;
    Print print;
    print.is_BBL_printer() = true;
    if (body_split)
        init_body_split_pad_coupon(model, print, config, true, 4.0, .10, .20);
    else
        init_tower_coupon(model, print, config);
    const CadenceTest::Facts facts = CadenceTest::slice(print);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());

    // Body Split prints a body on each side and changes tools. The Feature Split coupon is priced all fine.
    std::set<unsigned> printing_tools;
    for (const auto &move : facts.moves)
        if (CadenceTest::model_road(move))
            printing_tools.insert(move.physical_tool_id);
    if (body_split) {
        CHECK_FALSE(facts.emitted.tool_changes.empty());
        CHECK(printing_tools == std::set<unsigned>{0, 1});
    } else
        CHECK(printing_tools.size() == 1);

    // The sizes the dialog reads for the right extruder (s_get_slicing_extuder_nozzles): the grouping's
    // used nozzles when the map is dynamic, otherwise each used filament's extruder in the sliced config.
    const auto group = print.get_nozzle_group_result();
    REQUIRE(group);
    std::vector<float> right_sizes;
    if (group->is_support_dynamic_nozzle_map()) {
        for (const auto &nozzle : group->get_used_nozzles_in_extruder(1))
            right_sizes.push_back(std::stof(nozzle.diameter));
    } else {
        for (unsigned int filament : group->get_used_filaments()) {
            const auto nozzle = GUI::slicing_extruder_nozzle(config, print.config().filament_map.values, int(filament) + 1);
            REQUIRE(nozzle.has_value());
            if (nozzle->physical_extruder == MAIN_EXTRUDER_ID)
                right_sizes.push_back(nozzle->diameter);
        }
    }
    REQUIRE_FALSE(right_sizes.empty());
    for (float size : right_sizes)
        CHECK_THAT(size, Catch::Matchers::WithinAbs(0.2, 1e-6));

    // An H2C with 0.4 hotends mounted on both sides and a 0.4 and a 0.6 in the rack: no 0.2 anywhere.
    MachineObject obj(nullptr, nullptr, "test", "test_dev", "");
    obj.GetNozzleSystem()->SetSupportNozzleRack(true);
    json report = json::parse(R"({
        "nozzle": {
            "info": [
                { "id": 0,  "diameter": 0.4, "type": "HS00" },
                { "id": 1,  "diameter": 0.4, "type": "HS00" },
                { "id": 16, "diameter": 0.4, "type": "HS00" },
                { "id": 17, "diameter": 0.6, "type": "HS00" }
            ]
        }
    })");
    DevNozzleSystemParser::ParseV2_0(report, obj.GetNozzleSystem());
    REQUIRE(obj.GetNozzleSystem()->GetNozzleRack()->IsSupported());
    REQUIRE(obj.GetNozzleSystem()->GetRackNozzles().size() == 2);
    for (float size : right_sizes)
        CHECK(obj.GetNozzleSystem()->CollectNozzles(MAIN_EXTRUDER_ID, NozzleFlowType::S_FLOW, size).empty());

    // A 0.2 in the rack passes. On a printer that reports flow types, a 0.2 High Flow does not stand in
    // for a Standard one.
    report["nozzle"]["info"].push_back(json::parse(R"({ "id": 18, "diameter": 0.2, "type": "HH00" })"));
    DevNozzleSystemParser::ParseV2_0(report, obj.GetNozzleSystem());
    if (obj.is_nozzle_flow_type_supported())
        for (float size : right_sizes)
            CHECK(obj.GetNozzleSystem()->CollectNozzles(MAIN_EXTRUDER_ID, NozzleFlowType::S_FLOW, size).empty());
    report["nozzle"]["info"].push_back(json::parse(R"({ "id": 19, "diameter": 0.2, "type": "HS00" })"));
    DevNozzleSystemParser::ParseV2_0(report, obj.GetNozzleSystem());
    for (float size : right_sizes)
        CHECK_FALSE(obj.GetNozzleSystem()->CollectNozzles(MAIN_EXTRUDER_ID, NozzleFlowType::S_FLOW, size).empty());
}
