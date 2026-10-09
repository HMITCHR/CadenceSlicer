#include "TestModePrinter.hpp"

#include "DeviceManager.hpp"
#include "DeviceCore/DevExtruderSystem.h"
#include "DeviceCore/DevFilaSystem.h"
#include "DeviceCore/DevNozzleSystem.h"
#include "libslic3r/PrintConfig.hpp"

#include <cmath>
#include <cstdio>

namespace Slic3r::GUI::TestMode {

using nlohmann::json;

namespace {

// The H2D lists its right nozzle as extruder 0 (physical_extruder_map 1, 0).
constexpr int RIGHT_DEVICE_ID = 0;
constexpr int LEFT_DEVICE_ID  = 1;

std::string hex(unsigned long long value)
{
    char text[24];
    std::snprintf(text, sizeof(text), "%llX", value);
    return text;
}

bool nozzle_entry(const json &spec, const char *side, int device_id, json &out, std::string &error)
{
    const json side_spec = spec.value(side, json::object());
    if (!side_spec.is_object()) {
        error = std::string("\"") + side + "\" must be an object";
        return false;
    }
    const double diameter = side_spec.value("diameter", 0.4);
    if (!std::isfinite(diameter) || diameter <= 0.) {
        error = std::string("bad ") + side + " nozzle diameter";
        return false;
    }
    std::string type = side_spec.value("type", std::string());
    if (type.empty()) {
        const std::string flow = side_spec.value("flow", std::string("Standard"));
        type = fake_nozzle_type(flow);
        if (type.empty()) {
            error = std::string("unknown ") + side + " nozzle flow \"" + flow + "\"";
            return false;
        }
    }
    out = {{"id", device_id}, {"diameter", diameter}, {"type", type}, {"sn", "N/A"}, {"wear", 0}};
    return true;
}

} // namespace

std::string fake_nozzle_type(const std::string &flow)
{
    // H = hardened steel nozzle, then the flow letter, then the material code the H2D uses.
    if (flow == "Standard")
        return "HS01";
    if (flow == "High Flow")
        return "HH01";
    if (flow == "TPU High Flow")
        return "HU01";
    return {};
}

json fake_printer_reports(const json &spec, std::string &error)
{
    if (!spec.is_object()) {
        error = "printer description must be an object";
        return json::array();
    }
    json right, left;
    if (!nozzle_entry(spec, "right", RIGHT_DEVICE_ID, right, error) || !nozzle_entry(spec, "left", LEFT_DEVICE_ID, left, error))
        return json::array();

    // Each extruder holds the nozzle with its own id; bit 3 of "info" says a nozzle is fitted.
    json extruders = json::array();
    for (int id : {RIGHT_DEVICE_ID, LEFT_DEVICE_ID})
        extruders.push_back({{"id", id}, {"info", 8}, {"temp", 0}, {"spre", 0xFFFF}, {"snow", 0xFFFF},
                             {"star", 0xFFFF}, {"stat", 0}, {"hnow", id}, {"filam_bak", json::array()}});

    json ams_units = json::array();
    unsigned long long ams_exist = 0, tray_exist = 0;
    int next_id = 0, next_ht_id = 128;
    for (const json &unit : spec.value("ams", json::array())) {
        const std::string side = unit.value("extruder", std::string("left"));
        if (side != "left" && side != "right") {
            error = "AMS extruder must be \"left\" or \"right\"";
            return json::array();
        }
        const std::string kind = unit.value("type", std::string("AMS 2 Pro"));
        int type_id = 0;
        if (kind == "AMS")
            type_id = 1;
        else if (kind == "AMS 2 Pro")
            type_id = 3;
        else if (kind == "AMS HT")
            type_id = 4;
        else {
            error = "unknown AMS type \"" + kind + "\"";
            return json::array();
        }
        const bool ht = type_id == 4;
        const int id = ht ? next_ht_id++ : next_id++;
        const json trays = unit.value("trays", json::array());
        if (trays.size() > (ht ? 1u : 4u)) {
            error = kind + " holds " + (ht ? "one tray" : "four trays");
            return json::array();
        }
        const int device_extruder = side == "right" ? RIGHT_DEVICE_ID : LEFT_DEVICE_ID;
        ams_exist |= 1ull << (ht ? 4 + (id - 128) : id);
        json tray_reports = json::array();
        for (std::size_t slot = 0; slot < trays.size(); ++slot) {
            const json &tray = trays[slot];
            json report {{"id", std::to_string(slot)}};
            // A tray with no "type" is an empty slot.
            if (tray.contains("type")) {
                tray_exist |= 1ull << (ht ? 16 + (id - 128) : id * 4 + int(slot));
                report["tray_type"] = tray.at("type").get<std::string>();
                report["tray_info_idx"] = tray.value("filament_id", std::string());
                report["tray_color"] = tray.value("color", std::string("FFFFFFFF"));
                report["tray_sub_brands"] = tray.value("name", std::string());
                report["tag_uid"] = "0000000000000000";
                report["remain"] = 100;
                report["cols"] = json::array({report["tray_color"]});
                report["ctype"] = 0;
            }
            tray_reports.push_back(report);
        }
        // "info": unit type in bits 0-3, the extruder it feeds in bits 8-11.
        ams_units.push_back({{"id", std::to_string(id)}, {"info", hex(type_id | (device_extruder << 8))},
                             {"humidity", "5"}, {"temp", "25.0"}, {"tray", tray_reports}});
    }

    json version {{"info", {{"command", "get_version"}, {"sequence_id", "0"}, {"module", json::array({
        {{"name", "ota"}, {"product_name", "Test mode printer"}, {"sw_ver", "01.01.00.00"}, {"hw_ver", "OTA"}, {"sn", "TESTMODE"}},
    })}}}};

    // No external spool report, so the AMS trays are the only filaments a sync can map.
    json print {{"command", "push_status"}, {"msg", 0}, {"sequence_id", "0"},
                // Present in every new-style report; parse_new_info() reads "device" only then.
                {"cfg", "0"}, {"fun", "0"}, {"aux", "0"}, {"stat", "0"},
                {"device", {{"type", 1},
                            {"extruder", {{"state", 2}, {"info", extruders}}},
                            {"nozzle", {{"exist", 3}, {"state", 0}, {"info", json::array({right, left})}}}}},
                {"ams", {{"ams", ams_units}, {"ams_exist_bits", hex(ams_exist)}, {"tray_exist_bits", hex(tray_exist)},
                         {"tray_is_bbl_bits", hex(tray_exist)}, {"tray_read_done_bits", hex(tray_exist)},
                         {"tray_reading_bits", "0"}, {"version", 1}}}};
    return json::array({version, json{{"print", print}}});
}

bool update_fake_printer(MachineObject &obj, const json &spec, std::string &error)
{
    const json reports = fake_printer_reports(spec, error);
    if (reports.empty())
        return false;
    obj.printer_type = spec.value("model", std::string("O1D"));
    obj.dev_connection_type = "lan";
    for (const json &report : reports)
        if (obj.parse_json("lan", report.dump()) < 0) {
            error = "the printer object refused a report";
            return false;
        }
    // The report must come through as written, or a scenario would test something else.
    for (const auto &[side, device_id] : {std::pair<const char *, int>{"right", RIGHT_DEVICE_ID}, {"left", LEFT_DEVICE_ID}}) {
        const double wanted = spec.value(side, json::object()).value("diameter", 0.4);
        const double got = obj.GetExtderSystem()->GetNozzleDiameter(device_id);
        if (std::abs(got - wanted) > 1e-4) {
            error = std::string("the ") + side + " nozzle reads " + std::to_string(got) + " mm after the report";
            return false;
        }
    }
    std::size_t wanted_trays = 0, loaded_trays = 0;
    for (const json &unit : spec.value("ams", json::array()))
        for (const json &tray : unit.value("trays", json::array()))
            wanted_trays += tray.contains("type");
    for (const auto &[id, ams] : obj.GetFilaSystem()->GetAmsList())
        for (const auto &[slot, tray] : ams->GetTrays())
            loaded_trays += tray->is_exists;
    if (loaded_trays != wanted_trays) {
        error = "the printer reads " + std::to_string(loaded_trays) + " loaded AMS trays after the report, not " +
                std::to_string(wanted_trays);
        return false;
    }
    return true;
}

std::unique_ptr<MachineObject> make_fake_printer(DeviceManager *manager, const json &spec, std::string &error)
{
    // No agent and no address: nothing it is asked to do leaves the app. The status parser reads
    // the manager's settings, so it needs one.
    if (manager == nullptr) {
        error = "no device manager";
        return nullptr;
    }
    auto obj = std::make_unique<MachineObject>(manager, nullptr, "Test mode printer", "TESTMODE-PRINTER", "");
    if (!update_fake_printer(*obj, spec, error))
        return nullptr;
    return obj;
}

json describe_printer(MachineObject &obj)
{
    json out {{"model", obj.printer_type}, {"info_ready", obj.is_info_ready()},
              {"flow_type_supported", obj.is_nozzle_flow_type_supported()},
              {"has_ams", obj.GetFilaSystem()->HasAms()}};
    for (const auto &[side, device_id] : {std::pair<const char *, int>{"left", LEFT_DEVICE_ID}, {"right", RIGHT_DEVICE_ID}}) {
        const NozzleFlowType flow = obj.GetExtderSystem()->GetNozzleFlowType(device_id);
        out[side] = {{"diameter", std::round(obj.GetExtderSystem()->GetNozzleDiameter(device_id) * 100.) / 100.},
                     {"flow", flow == NozzleFlowType::NONE_FLOWTYPE ? std::string("none")
                                                                    : get_nozzle_volume_type_string(DevNozzle::ToNozzleVolumeType(flow))}};
    }
    json units = json::array();
    for (const auto &[id, ams] : obj.GetFilaSystem()->GetAmsList()) {
        json trays = json::array();
        for (const auto &[slot, tray] : ams->GetTrays())
            trays.push_back({{"slot", slot}, {"exists", tray->is_exists}, {"type", tray->m_fila_type},
                             {"color", tray->color}, {"filament_id", tray->setting_id}});
        units.push_back({{"id", id}, {"extruder", ams->GetExtruderId() == RIGHT_DEVICE_ID ? "right" : "left"},
                         {"trays", trays}});
    }
    out["ams"] = units;
    return out;
}

} // namespace Slic3r::GUI::TestMode
