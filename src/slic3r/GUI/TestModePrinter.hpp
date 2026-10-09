#ifndef slic3r_GUI_TestModePrinter_hpp_
#define slic3r_GUI_TestModePrinter_hpp_

// A pretend printer for the scripted test mode, so the printer sync (nozzles, flow types, AMS
// filaments) can be run without a real printer. It is a MachineObject with no network agent and
// no address, fed the status reports a real H2D sends, built from a scenario's description. Only
// the test mode creates one; nothing here talks to a printer.

#include <nlohmann/json.hpp>

#include <memory>
#include <string>

namespace Slic3r {
class DeviceManager;
class MachineObject;
}

namespace Slic3r::GUI::TestMode {

// The messages a printer described by SPEC sends after connecting, in order: its firmware
// versions, then one full status report. SPEC, every key optional:
//   "model":  printer model id, default "O1D" (H2D)
//   "left", "right": {"diameter": 0.4, "flow": "Standard" | "High Flow" | "TPU High Flow"}
//             or {"diameter": 0.6, "type": "HS01"}, the nozzle type string exactly as a printer
//             reports it ("type" wins over "flow"). Default 0.4 Standard.
//   "ams": [{"extruder": "left" | "right", "type": "AMS" | "AMS 2 Pro" | "AMS HT",
//            "trays": [{"type": "PLA", "color": "FFFFFFFF", "filament_id": "GFA00"}, ...]}]
// Like the H2D, the report lists the right nozzle as extruder 0 and the left as extruder 1.
// Returns an empty array and sets ERROR when SPEC cannot be reported.
nlohmann::json fake_printer_reports(const nlohmann::json &spec, std::string &error);

// The nozzle type string a printer reports for FLOW ("Standard" gives "HS01"), empty if unknown.
std::string fake_nozzle_type(const std::string &flow);

// A printer object of MANAGER that has received the reports for SPEC. It has no agent and no
// address, so any command it is given goes nowhere, and MANAGER does not list it. Null with ERROR
// set when SPEC cannot be reported.
std::unique_ptr<MachineObject> make_fake_printer(DeviceManager *manager, const nlohmann::json &spec, std::string &error);

// Feeds OBJ the reports for SPEC, as a printer does after a nozzle or a spool is changed.
bool update_fake_printer(MachineObject &obj, const nlohmann::json &spec, std::string &error);

// What OBJ reports, for the test mode's state dumps: nozzles in config order (left, right) and
// the AMS trays.
nlohmann::json describe_printer(MachineObject &obj);

} // namespace Slic3r::GUI::TestMode

#endif
