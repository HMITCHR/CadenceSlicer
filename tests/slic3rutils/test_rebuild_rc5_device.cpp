// The rc5 High Flow finding replayed from the printer's own report: an H2D whose right nozzle is a
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

#include "slic3r/GUI/DeviceManager.hpp"
#include "slic3r/GUI/DeviceCore/DevNozzleSystem.h"
#include "slic3r/GUI/ProjectNozzleFlowState.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <vector>

using json = nlohmann::json;
using namespace Slic3r;
using namespace Slic3r::GUI;

TEST_CASE("RC5-4: the H2D's report of a 0.6 \"HS01\" right nozzle never replaces the project's High Flow unasked",
          "[TestRebuild][RC5][RC5I]")
{
    MachineObject obj(nullptr, nullptr, "test", "test_dev", "127.0.0.1");
    // The printer's nozzle block as the owner's H2D sent it: device extruder 0 is the right nozzle.
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
}

TEST_CASE("A printer sync offers the filament sync before setup opens", "[TestRebuild][SyncOrder]")
{
    // Sync button or auto sync, printer with an AMS: the filament offer comes before setup.
    CHECK(sync_filaments_before_setup(true, true, false));
    // A nozzle picked by hand in the sidebar: setup opens at once, with or without a printer online.
    CHECK_FALSE(sync_filaments_before_setup(false, true, false));
    CHECK_FALSE(sync_filaments_before_setup(false, false, false));
    // No AMS: setup opens as before and the stock offer follows the sync.
    CHECK_FALSE(sync_filaments_before_setup(true, false, false));
    // One sync asks once, also when the project pair and the nozzle prompt both lead to setup.
    CHECK_FALSE(sync_filaments_before_setup(true, true, true));
}
