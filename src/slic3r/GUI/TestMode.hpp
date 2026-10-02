#ifndef slic3r_GUI_TestMode_hpp_
#define slic3r_GUI_TestMode_hpp_

// Scripted GUI test mode. Only CADENCE_TEST_SCRIPT turns it on: the app then skips network,
// cloud, keychain and update start-up, runs the scenario steps through the same handlers the
// user's clicks reach, writes a state dump and window captures per step to CADENCE_TEST_OUT, and
// exits with a status code. Without the variable nothing here runs.

#include <nlohmann/json_fwd.hpp>
#include <string>

class wxMenu;
class wxPoint;
class wxWindow;

namespace Slic3r::GUI {

class Sidebar;
class MixedNozzleWizardDialog;

namespace TestMode {
bool active();
// Called once the main window is up. Starts the scenario on a timer.
void start();
// A popup menu's choice. A scripted click that names a menu item answers it; otherwise, and
// always outside test mode, the menu pops up for the user.
int popup_menu(wxWindow &owner, wxMenu &menu, const wxPoint &at);
// Each plate validation's result, in order, for the state dumps. Nothing outside test mode.
void note_validation(const std::string &error);
} // namespace TestMode

// Sidebar controls live in Sidebar::priv, so this is implemented in Plater.cpp.
struct TestModeSidebarAccess
{
    // which: "printer", "bed_type", "filament:N" (1-based), "left_diameter", "right_diameter", "left_flow",
    // "right_flow", "single_diameter", "single_flow". Selects the item as a click would and fires
    // the combo's own event; from_list opens the list and picks from it, in the list's own order.
    static bool pick(Sidebar &sidebar, const std::string &which, const std::string &value, std::string &error,
                     bool from_list = false);
    // which: "mixed_setup", "add_filament", "delete_filament".
    static bool click(Sidebar &sidebar, const std::string &which, std::string &error);
    static void dump(Sidebar &sidebar, nlohmann::json &out);
};

// Wizard controls whose labels repeat (per-part roles, material pickers, exact slots).
struct TestModeWizardAccess
{
    static bool set_role(MixedNozzleWizardDialog &dialog, std::size_t part, bool fine, std::string &error);
    static bool set_material(MixedNozzleWizardDialog &dialog, bool fine, int slot, std::string &error);
    static bool set_exact_slot(MixedNozzleWizardDialog &dialog, std::size_t part, int slot, std::string &error);
    static bool pick_row(MixedNozzleWizardDialog &dialog, std::size_t row, std::string &error);
    static void dump(MixedNozzleWizardDialog &dialog, nlohmann::json &out);
};

} // namespace Slic3r::GUI

#endif
