#include "TestMode.hpp"
#include "TestModePrinter.hpp"

#include "DeviceManager.hpp"
#include "GUI_App.hpp"
#include "GLToolbar.hpp"
#include "GUI_ObjectList.hpp"
#include "MainFrame.hpp"
#include "MixedNozzleSetupController.hpp"
#include "MixedNozzleSetupDialog.hpp"
#include "MixedNozzleSidebarPanel.hpp"
#include "MsgDialog.hpp"
#include "ObjectDataViewModel.hpp"
#include "PartPlate.hpp"
#include "Selection.hpp"
#include "SyncAmsInfoDialog.hpp"
#include "Tab.hpp"
#include "3DScene.hpp"
#include "Plater.hpp"
#include "PresetComboBoxes.hpp"
#include "UnsavedChangesDialog.hpp"
#include "Widgets/Button.hpp"
#include "Widgets/ComboBox.hpp"
#include "Widgets/SpinInput.hpp"
#include "Widgets/TextInput.hpp"
#include "Widgets/StateColor.hpp"
#include "libslic3r/AppConfig.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PresetBundle.hpp"

#include <nlohmann/json.hpp>

#include <wx/arrstr.h>
#include <wx/bookctrl.h>
#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/menu.h>
#include <wx/choice.h>
#include <wx/msgdlg.h>
#include <wx/radiobut.h>
#include <wx/scrolbar.h>
#include <wx/scrolwin.h>
#include <wx/simplebook.h>
#include <wx/stattext.h>
#include <wx/timer.h>
#include <wx/tokenzr.h>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <set>

#ifdef __APPLE__
#include <objc/message.h>
#include <objc/runtime.h>
#endif

namespace Slic3r::GUI {

using nlohmann::json;
using Clock = std::chrono::steady_clock;

namespace {

std::string env(const char *name)
{
    const char *value = std::getenv(name);
    return value == nullptr ? std::string() : std::string(value);
}

std::string mode_name(MixedNozzleSlicingMode mode)
{
    switch (mode) {
    case MixedNozzleSlicingMode::FeatureSplit: return "FeatureSplit";
    case MixedNozzleSlicingMode::BodySplit: return "BodySplit";
    default: return "Off";
    }
}

// The menu item the current scripted click answers, and why it could not.
std::string g_menu_answer;
std::string g_menu_error;
// What the last scripted menu answer found: the item, and whether it was enabled and checked.
json g_menu_last;
// Plate validation results since the last step's dump, oldest first.
json g_validations = json::array();
// The pretend printer a scenario connected; see connect_printer.
std::unique_ptr<MachineObject> g_fake_printer;

std::string bed_name(BedType type)
{
    return type >= btDefault && type < btCount ? ConfigOptionEnum<BedType>(type).serialize() : std::string("none");
}

std::string plain(const wxString &label) { return into_u8(wxControl::RemoveMnemonics(label).Strip(wxString::both)); }

// Shown on screen as far as wx knows: the window and every parent up to its top-level window.
bool visible(wxWindow *window)
{
    for (wxWindow *w = window; w != nullptr; w = w->GetParent()) {
        if (!w->IsShown())
            return false;
        if (w->IsTopLevel())
            return true;
    }
    return true;
}

template<class Fn> void walk(wxWindow *root, Fn &&fn)
{
    for (wxWindow *child : root->GetChildren()) {
        if (child->IsTopLevel())
            continue;
        fn(child);
        walk(child, fn);
    }
}

std::vector<wxDialog *> shown_modal_dialogs()
{
    std::vector<wxDialog *> out;
    for (wxWindow *w : wxTopLevelWindows)
        if (auto *dialog = dynamic_cast<wxDialog *>(w); dialog != nullptr && dialog->IsShown() && dialog->IsModal())
            out.push_back(dialog);
    return out;
}

// The class a scenario names a dialog by.
std::string dialog_class(wxTopLevelWindow *top)
{
    if (dynamic_cast<MixedNozzleWizardDialog *>(top) != nullptr)
        return "MixedNozzleWizardDialog";
    if (dynamic_cast<DissimilarNozzleDialog *>(top) != nullptr)
        return "DissimilarNozzleDialog";
    if (dynamic_cast<MessageDialog *>(top) != nullptr)
        return "MessageDialog";
    if (dynamic_cast<SyncAmsInfoDialog *>(top) != nullptr)
        return "SyncAmsInfoDialog";
    if (dynamic_cast<SyncNozzleAndAmsDialog *>(top) != nullptr)
        return "SyncNozzleAndAmsDialog";
    return wxString(top->GetClassInfo()->GetClassName()).ToStdString();
}

MixedNozzleWizardDialog *shown_wizard()
{
    for (wxWindow *w : wxTopLevelWindows)
        if (auto *wizard = dynamic_cast<MixedNozzleWizardDialog *>(w); wizard != nullptr && wizard->IsShown())
            return wizard;
    return nullptr;
}

wxWindow *find_descendant(wxWindow *root, const std::function<bool(wxWindow *)> &match)
{
    wxWindow *found = nullptr;
    walk(root, [&](wxWindow *w) {
        if (found == nullptr && match(w))
            found = w;
    });
    return found;
}

// The page a control sits on: the child of a book control, else the top-level window.
wxWindow *page_of(wxWindow *w)
{
    for (wxWindow *x = w; x != nullptr; x = x->GetParent()) {
        if (x->GetParent() != nullptr && dynamic_cast<wxBookCtrlBase *>(x->GetParent()) != nullptr)
            return x;
        if (x->IsTopLevel())
            return x;
    }
    return w;
}

// Text that does not fit: wider than its page, clipped inside its own control, or cut by the
// page or window edge.
void check_overflow(wxWindow *top, const std::string &where, json &out)
{
    const int top_width = top->GetClientSize().x;
    const wxPoint top_origin = top->ClientToScreen(wxPoint(0, 0));
    walk(top, [&](wxWindow *w) {
        if (!visible(w))
            return;
        std::string kind;
        std::string text;
        int needed = 0;
        bool check_inside = true;
        if (auto *st = dynamic_cast<wxStaticText *>(w)) {
            kind = "text";
            const wxString label = st->GetLabel();
            text = into_u8(label);
            wxStringTokenizer lines(label, "\n", wxTOKEN_RET_EMPTY_ALL);
            while (lines.HasMoreTokens())
                needed = std::max(needed, st->GetTextExtent(wxControl::RemoveMnemonics(lines.GetNextToken())).x);
            check_inside = (st->GetWindowStyle() & wxST_ELLIPSIZE_MASK) == 0;
        } else if (auto *cb = dynamic_cast<wxCheckBox *>(w)) {
            kind = "checkbox";
            text = plain(cb->GetLabel());
            needed = cb->GetBestSize().x;
        } else if (auto *rb = dynamic_cast<wxRadioButton *>(w)) {
            kind = "radio";
            text = plain(rb->GetLabel());
            needed = rb->GetBestSize().x;
        } else if (auto *ch = dynamic_cast<wxChoice *>(w)) {
            kind = "choice";
            text = into_u8(ch->GetStringSelection());
            needed = ch->GetTextExtent(ch->GetStringSelection()).x + ch->FromDIP(28);
        } else if (auto *combo = dynamic_cast<::ComboBox *>(w)) {
            kind = "combo";
            text = into_u8(combo->GetValue());
            needed = combo->GetTextExtent(combo->GetValue()).x;
            check_inside = false;
        } else if (dynamic_cast<wxButton *>(w) != nullptr) {
            kind = "button";
            text = plain(w->GetLabel());
            needed = w->GetBestSize().x;
        } else
            return;
        if (text.empty() || needed <= 0)
            return;
        wxWindow *page = page_of(w);
        const int page_width = page->GetClientSize().x;
        const wxPoint page_origin = page->ClientToScreen(wxPoint(0, 0));
        const wxPoint at = w->GetScreenPosition();
        const int width = w->GetSize().x;
        const int x_in_page = at.x - page_origin.x;
        const int x_in_top = at.x - top_origin.x;
        const int shown_right = std::min(needed, width);
        std::vector<std::string> problems;
        if (needed > page_width + 1)
            problems.push_back("wider_than_page");
        if (check_inside && needed > width + 2)
            problems.push_back("clipped_in_control");
        if (x_in_page + shown_right > page_width + 1)
            problems.push_back("past_page_edge");
        if (x_in_top + shown_right > top_width + 1)
            problems.push_back("past_window_edge");
        if (problems.empty())
            return;
        out.push_back({{"window", where}, {"kind", kind}, {"text", text}, {"problems", problems},
                       {"needed", needed}, {"width", width}, {"x_in_page", x_in_page},
                       {"page_width", page_width}, {"x_in_window", x_in_top}, {"window_width", top_width}});
    });
}

// unwrap joins the lines a wrapped label was split into, so a scenario can match its text.
std::string label_text(wxWindow *w, bool unwrap)
{
    wxString label = w->GetLabel();
    if (unwrap)
        label.Replace("\n", " ");
    return into_u8(label);
}

std::vector<std::string> visible_texts(wxWindow *root, bool unwrap = false)
{
    std::vector<std::string> out;
    walk(root, [&](wxWindow *w) {
        if (dynamic_cast<wxStaticText *>(w) != nullptr && visible(w)) {
            const std::string text = label_text(w, unwrap);
            if (!text.empty())
                out.push_back(text);
        }
    });
    return out;
}

// Visible buttons and their state, for dialogs whose buttons carry the wording.
json visible_buttons(wxWindow *root)
{
    json out = json::array();
    walk(root, [&](wxWindow *w) {
        if ((dynamic_cast<wxButton *>(w) != nullptr || dynamic_cast<::Button *>(w) != nullptr) && visible(w)) {
            const std::string text = plain(w->GetLabel());
            if (!text.empty())
                out.push_back({{"label", text}, {"enabled", w->IsEnabled()}});
        }
    });
    return out;
}

// Visible buttons on the same row with less than 4 px between them, as "left | right".
json touching_buttons(wxWindow *root)
{
    std::vector<wxWindow *> buttons;
    walk(root, [&](wxWindow *w) {
        if ((dynamic_cast<wxButton *>(w) != nullptr || dynamic_cast<::Button *>(w) != nullptr) && visible(w) &&
            !plain(w->GetLabel()).empty())
            buttons.push_back(w);
    });
    json out = json::array();
    for (wxWindow *a : buttons)
        for (wxWindow *b : buttons) {
            const wxRect ra = a->GetScreenRect(), rb = b->GetScreenRect();
            const bool same_row = ra.GetTop() < rb.GetBottom() && rb.GetTop() < ra.GetBottom();
            if (a == b || !same_row || ra.GetLeft() > rb.GetLeft())
                continue;
            if (rb.GetLeft() - ra.GetRight() - 1 < 4)
                out.push_back(plain(a->GetLabel()) + " | " + plain(b->GetLabel()));
        }
    return out;
}

// Visible dropdowns: the value shown and the items offered.
json visible_choices(wxWindow *root)
{
    json out = json::array();
    walk(root, [&](wxWindow *w) {
        if (!visible(w))
            return;
        json items = json::array();
        std::string value;
        if (auto *ch = dynamic_cast<wxChoice *>(w)) {
            for (unsigned int i = 0; i < ch->GetCount(); ++i)
                items.push_back(into_u8(ch->GetString(i)));
            value = into_u8(ch->GetStringSelection());
        } else if (auto *combo = dynamic_cast<::ComboBox *>(w)) {
            for (unsigned int i = 0; i < combo->GetCount(); ++i)
                items.push_back(into_u8(combo->GetString(i)));
            value = into_u8(combo->GetValue());
        } else
            return;
        out.push_back({{"value", value}, {"items", items}, {"enabled", w->IsEnabled()}});
    });
    return out;
}

// Visible checkboxes: label, ticked, enabled.
json visible_checkboxes(wxWindow *root)
{
    json out = json::array();
    walk(root, [&](wxWindow *w) {
        if (auto *cb = dynamic_cast<wxCheckBox *>(w); cb != nullptr && visible(w))
            out.push_back({{"label", plain(cb->GetLabel())}, {"value", cb->GetValue()}, {"enabled", cb->IsEnabled()}});
    });
    return out;
}

// Tooltips on visible text, keyed by the text.
json visible_tooltips(wxWindow *root, bool unwrap = false)
{
    json out = json::object();
    walk(root, [&](wxWindow *w) {
        if (dynamic_cast<wxStaticText *>(w) != nullptr && visible(w) && !w->GetToolTipText().IsEmpty())
            out[label_text(w, unwrap)] = into_u8(w->GetToolTipText());
    });
    return out;
}

std::string colour_text(const wxColour &c) { return into_u8(c.GetAsString(wxC2S_HTML_SYNTAX)); }

// Visible text drawn in nearly its own background colour, such as white on white.
json low_contrast_texts(wxWindow *root)
{
    json out = json::array();
    walk(root, [&](wxWindow *w) {
        if (dynamic_cast<wxStaticText *>(w) == nullptr || !visible(w) || w->GetLabel().IsEmpty())
            return;
        wxWindow *painted = w;
        while (painted->GetParent() != nullptr && !painted->UseBgCol() && !painted->IsTopLevel())
            painted = painted->GetParent();
        const wxColour bg = painted->GetBackgroundColour();
        const wxColour fg = w->GetForegroundColour();
        if (StateColor::GetColorDifference(bg, fg) < 20.)
            out.push_back({{"text", into_u8(w->GetLabel())}, {"fg", colour_text(fg)}, {"bg", colour_text(bg)}});
    });
    return out;
}

// Visible text wider than its control, so it is cut off or ellipsized.
json cut_texts(wxWindow *root)
{
    json out = json::array();
    walk(root, [&](wxWindow *w) {
        auto *st = dynamic_cast<wxStaticText *>(w);
        if (st == nullptr || !visible(w))
            return;
        // An ellipsized label shows the cut text, so measure what it was given.
        const wxString label = st->GetLabel();
        int needed = 0;
        for (const wxString &line : wxSplit(label, '\n', '\0'))
            needed = std::max(needed, st->GetTextExtent(wxControl::RemoveMnemonics(line)).x);
        if (needed > st->GetSize().x + 2)
            out.push_back({{"text", into_u8(label)}, {"needed", needed}, {"width", st->GetSize().x}});
    });
    return out;
}

// A name for a sidebar section: its class and the first text it shows.
std::string section_name(wxWindow *section)
{
    std::string name = into_u8(wxString(section->GetClassInfo()->GetClassName()));
    if (dynamic_cast<MixedNozzleSidebarPanel *>(section) != nullptr)
        return "MixedNozzleSidebarPanel";
    if (auto *st = dynamic_cast<wxStaticText *>(section); st != nullptr && !st->GetLabel().empty())
        return name + " '" + plain(st->GetLabel()) + "'";
    if (wxWindow *text = find_descendant(section, [](wxWindow *w) {
            return dynamic_cast<wxStaticText *>(w) != nullptr && visible(w) && !w->GetLabel().empty();
        }))
        name += " '" + plain(text->GetLabel()) + "'";
    return name;
}

// Visible sidebar controls the user cannot fully see: cut by a parent or the window, or covered by
// another sidebar section, and containers squeezed below the height their content needs.
// `sections_parent` holds the sections. A section that scrolls only has to hold its content in
// its scrolled area; content of a scrolling list inside a section is left alone.
json hidden_sidebar_controls(wxWindow *sections_parent)
{
    json out = json::array();
    const auto screen_rect = [](wxWindow *w) { return wxRect(w->GetScreenPosition(), w->GetSize()); };
    const auto client_rect = [](wxWindow *w) { return wxRect(w->ClientToScreen(wxPoint(0, 0)), w->GetClientSize()); };
    // What a parent lets its children use: the scrolled content of a scrolling area, else its client area.
    const auto usable_rect = [&](wxWindow *w) {
        auto *scroll = dynamic_cast<wxScrollHelperBase *>(w);
        if (scroll == nullptr)
            return client_rect(w);
        const wxPoint origin = w->ClientToScreen(scroll->CalcScrolledPosition(wxPoint(0, 0)));
        return wxRect(origin, w->GetVirtualSize());
    };
    std::vector<wxWindow *> sections;
    for (wxWindow *child : sections_parent->GetChildren())
        if (!child->IsTopLevel() && child->IsShown() && !screen_rect(child).IsEmpty())
            sections.push_back(child);
    walk(sections_parent, [&](wxWindow *w) {
        if (!visible(w) || dynamic_cast<wxScrollBar *>(w) != nullptr)
            return;
        wxWindow *section = w;
        while (section->GetParent() != sections_parent) {
            if (dynamic_cast<wxScrollHelperBase *>(section->GetParent()) != nullptr &&
                section->GetParent()->GetParent() != sections_parent)
                return;
            section = section->GetParent();
        }
        const wxRect rect = screen_rect(w);
        std::string label = plain(w->GetLabel());
        if (label.empty())
            label = into_u8(w->GetToolTipText());
        const auto report = [&](const std::string &problem, const std::string &by, const wxRect &part) {
            out.push_back({{"control", label}, {"class", into_u8(wxString(w->GetClassInfo()->GetClassName()))},
                           {"section", section_name(section)}, {"problem", problem}, {"by", by},
                           {"rect", {rect.x, rect.y, rect.width, rect.height}},
                           {"hidden_part", {part.x, part.y, part.width, part.height}}});
        };
        // A sizer that ran out of room squeezes its window below what the content needs. A window
        // given a fixed minimum height, such as the compact Nozzle box, is sized on purpose.
        if (w->GetSizer() != nullptr && dynamic_cast<wxScrollHelperBase *>(w) == nullptr && w->GetMinSize().y <= 0) {
            const int needed = w->GetSizer()->GetMinSize().y;
            if (w->GetClientSize().y < needed - 1) {
                label = section_name(w);
                report("squashed", std::to_string(needed - w->GetClientSize().y) + " px short", rect);
                return;
            }
        }
        // A section that scrolls must itself fit the sidebar, like a control.
        const bool control = dynamic_cast<wxControl *>(w) != nullptr || dynamic_cast<::Button *>(w) != nullptr ||
                             dynamic_cast<TextInput *>(w) != nullptr || dynamic_cast<SpinInput *>(w) != nullptr ||
                             (w == section && dynamic_cast<wxScrollHelperBase *>(w) != nullptr);
        if (!control || rect.IsEmpty())
            return;
        wxRect reach = rect;
        wxRect shown = rect;
        std::string clipped_by;
        for (wxWindow *a = w->GetParent(); a != nullptr; a = a->GetParent()) {
            const wxRect before = reach;
            reach.Intersect(a->IsTopLevel() ? client_rect(a) : usable_rect(a));
            shown.Intersect(client_rect(a));
            if (clipped_by.empty() && (before.width - reach.width > 1 || before.height - reach.height > 1))
                clipped_by = a->IsTopLevel() ? std::string("window") : section_name(a);
            // Past a scrolling area only its own frame counts, and that is checked on its own.
            if (a->IsTopLevel() || dynamic_cast<wxScrollHelperBase *>(a) != nullptr)
                break;
        }
        if (!clipped_by.empty()) {
            report("clipped", clipped_by, reach);
            return;
        }
        for (wxWindow *other : sections) {
            if (other == section)
                continue;
            wxRect covered = shown;
            covered.Intersect(screen_rect(other));
            if (covered.width > 1 && covered.height > 1) {
                report("covered", section_name(other), covered);
                return;
            }
        }
    });
    return out;
}

// The sidebar check, skipped while a modal dialog is open: the sidebar cannot be used then, and
// it may be mid-change.
json sidebar_layout_hidden()
{
    wxWindow *panel = find_descendant(wxGetApp().mainframe, [](wxWindow *w) {
        return dynamic_cast<MixedNozzleSidebarPanel *>(w) != nullptr;
    });
    if (panel == nullptr || !shown_modal_dialogs().empty())
        return json::array();
    json out = hidden_sidebar_controls(panel->GetParent());
    // The Process settings page scrolls, so the check above leaves it alone, but squeezed to a
    // sliver it shows no row a user can reach.
    ParamsPanel *params = wxGetApp().mainframe->m_param_panel;
    wxWindow *page = params != nullptr ? params->get_paged_view() : nullptr;
    if (page != nullptr && visible(page)) {
        const int short_by = page->FromDIP(SidebarProps::SettingsPageMinHeight()) - page->GetClientSize().y;
        if (short_by > 1) {
            const wxRect rect(page->GetScreenPosition(), page->GetSize());
            out.push_back({{"control", "Process settings page"}, {"class", "ParamsPanel"}, {"section", "Process"},
                           {"problem", "squashed"}, {"by", std::to_string(short_by) + " px short"},
                           {"rect", {rect.x, rect.y, rect.width, rect.height}},
                           {"hidden_part", {rect.x, rect.y, rect.width, rect.height}}});
        }
    }
    return out;
}

std::string capture_window(wxTopLevelWindow *top, const boost::filesystem::path &file, std::string &error)
{
#ifdef __APPLE__
    void *ns_window = top->GetWXWindow();
    if (ns_window == nullptr) {
        error = "no native window";
        return {};
    }
    using WindowNumber = long (*)(id, SEL);
    const long number = reinterpret_cast<WindowNumber>(objc_msgSend)(static_cast<id>(ns_window), sel_registerName("windowNumber"));
    if (number <= 0) {
        error = "window has no number (not on screen)";
        return {};
    }
    const std::string command = "/usr/sbin/screencapture -x -o -l" + std::to_string(number) + " '" + file.string() + "' 2>/dev/null";
    const int rc = std::system(command.c_str());
    if (rc != 0 || !boost::filesystem::exists(file)) {
        error = "screencapture exit " + std::to_string(rc);
        return {};
    }
    return file.filename().string();
#else
    error = "capture is macOS only";
    return {};
#endif
}

class Runner : public wxEvtHandler
{
public:
    Runner(json script, boost::filesystem::path out) : m_script(std::move(script)), m_out(std::move(out))
    {
        m_steps = m_script.value("steps", json::array());
        m_started = Clock::now();
        m_timeout = std::chrono::seconds(m_script.value("timeout_s", 900));
        m_timer.SetOwner(this);
        Bind(wxEVT_TIMER, [this](wxTimerEvent &) { tick(); });
        m_timer.Start(100);
    }

private:
    struct Action
    {
        bool started {false};
        bool returned {false};
        bool ok {true};
        std::string error;
    };
    enum class Phase { Ready, Posted, Settling };

    json m_script;
    json m_steps;
    boost::filesystem::path m_out;
    wxTimer m_timer;
    std::size_t m_index {0};
    Phase m_phase {Phase::Ready};
    std::shared_ptr<Action> m_action;
    std::set<wxDialog *> m_modal_before;
    Clock::time_point m_started;
    Clock::time_point m_phase_start;
    Clock::time_point m_quiet_until;
    Clock::duration m_timeout;
    bool m_busy {false};
    bool m_blocked_in_modal {false};
    json m_results = json::array();
    json m_dismissed = json::array();
    int m_failures {0};
    std::optional<Clock::time_point> m_layout_wait;
    // Every dialog that opened, in order, so a scenario can check which came first.
    json m_dialog_history = json::array();
    std::set<wxTopLevelWindow *> m_dialogs_shown;

    const json &step() const { return m_steps[m_index]; }
    long long elapsed_ms() const
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - m_phase_start).count();
    }

    void tick()
    {
        note_dialogs();
        if (m_busy)
            return;
        m_busy = true;
        try {
            do_tick();
        } catch (const std::exception &ex) {
            fail_step(std::string("exception: ") + ex.what());
        }
        m_busy = false;
    }

    void do_tick()
    {
        if (Clock::now() - m_started > m_timeout) {
            m_failures++;
            m_results.push_back({{"index", m_index}, {"ok", false}, {"error", "scenario timeout"}});
            finish(3);
            return;
        }
        if (m_phase == Phase::Ready) {
            if (m_index >= m_steps.size()) {
                finish(m_failures == 0 ? 0 : 1);
                return;
            }
            if (Clock::now() < m_quiet_until)
                return;
            if (dismiss_message_dialog()) {
                m_quiet_until = Clock::now() + std::chrono::milliseconds(600);
                return;
            }
            post_action();
            return;
        }
        if (m_phase == Phase::Posted) {
            const int timeout = step().value("timeout_s", 60) * 1000;
            if (m_action->returned) {
                enter_settling(false);
            } else if (m_action->started && elapsed_ms() > 300) {
                for (wxDialog *dialog : shown_modal_dialogs())
                    if (m_modal_before.count(dialog) == 0) {
                        enter_settling(true);
                        return;
                    }
            }
            if (m_phase == Phase::Posted && elapsed_ms() > timeout)
                fail_step("action did not return or open a dialog in time");
            return;
        }
        // Settling.
        if (elapsed_ms() < step().value("settle_ms", 700))
            return;
        // A dialog the action is waiting on and nobody scripted, such as "Save changes?" on New.
        if (m_blocked_in_modal && step().value("wait", std::string()) == "no_dialog" && Clock::now() >= m_quiet_until &&
            dismiss_message_dialog()) {
            m_quiet_until = Clock::now() + std::chrono::milliseconds(600);
            return;
        }
        // A failed action has nothing to wait for.
        const std::string wait = m_action->ok ? step().value("wait", std::string()) : std::string();
        if (!wait.empty() && !wait_satisfied(wait)) {
            if (elapsed_ms() <= step().value("timeout_s", 60) * 1000)
                return;
            // A soft wait records the state it reached and goes on.
            if (!step().value("soft", false)) {
                fail_step("wait for " + wait + " timed out");
                return;
            }
            m_action->error = "wait for " + wait + " timed out (soft)";
        }
        // The sidebar lays out again a moment after a dialog closes, so a hidden control only
        // counts once it stays hidden for half a second.
        if (!sidebar_layout_hidden().empty()) {
            if (!m_layout_wait)
                m_layout_wait = Clock::now();
            if (Clock::now() - *m_layout_wait < std::chrono::milliseconds(500))
                return;
        }
        m_layout_wait.reset();
        record_step();
        ++m_index;
        m_phase = Phase::Ready;
    }

    // One line per dialog when it first shows: "class | title | its texts". The stock offer to
    // sync filaments after a printer sync is a frame, not a dialog, and is noted too.
    void note_dialogs()
    {
        std::set<wxTopLevelWindow *> shown;
        for (wxWindow *w : wxTopLevelWindows) {
            auto *dialog = dynamic_cast<wxTopLevelWindow *>(w);
            if (dialog == nullptr || !dialog->IsShown() ||
                (dynamic_cast<wxDialog *>(w) == nullptr && dynamic_cast<SyncNozzleAndAmsDialog *>(w) == nullptr))
                continue;
            shown.insert(dialog);
            if (m_dialogs_shown.count(dialog) != 0)
                continue;
            std::string texts;
            for (const std::string &text : visible_texts(dialog))
                texts += (texts.empty() ? "" : " / ") + text;
            // Cut on a character boundary so the line stays valid UTF-8.
            std::size_t cut = std::min<std::size_t>(texts.size(), 200);
            while (cut > 0 && cut < texts.size() && (static_cast<unsigned char>(texts[cut]) & 0xC0) == 0x80)
                --cut;
            m_dialog_history.push_back(dialog_class(dialog) + " | " + into_u8(dialog->GetTitle()) + " | " +
                                       texts.substr(0, cut));
        }
        m_dialogs_shown = std::move(shown);
    }

    void enter_settling(bool blocked)
    {
        m_blocked_in_modal = blocked;
        m_phase = Phase::Settling;
        m_phase_start = Clock::now();
    }

    bool wait_satisfied(const std::string &wait)
    {
        Plater *plater = wxGetApp().plater();
        if (wait == "slice_done") {
            if (plater->is_background_process_slicing())
                return false;
            const PartPlate *plate = plater->get_partplate_list().get_curr_plate();
            return plate != nullptr && plate->is_slice_result_valid();
        }
        if (wait == "no_dialog")
            return shown_modal_dialogs().empty();
        if (wait == "wizard")
            return shown_wizard() != nullptr;
        if (wait == "ranking_done") {
            MixedNozzleWizardDialog *wizard = shown_wizard();
            if (wizard == nullptr)
                return true;
            json state;
            TestModeWizardAccess::dump(*wizard, state);
            return !state.value("ranking_busy", false);
        }
        return true;
    }

    // A dialog nobody scripted (a message box, unsaved changes): answer it from the scenario's
    // list, then the usual safe answers, and note it. The wizard is left to the steps.
    bool dismiss_message_dialog()
    {
        if (step().value("in", std::string()) == "dialog")
            return false;
        for (wxDialog *dialog : shown_modal_dialogs()) {
            if (dynamic_cast<MixedNozzleWizardDialog *>(dialog) != nullptr)
                continue;
            std::vector<std::string> answers = m_script.value("auto_answers", std::vector<std::string>{});
            for (const char *fallback : {"Don't Save", "Don't save", "Discard", "No", "OK", "Close", "Cancel"})
                answers.emplace_back(fallback);
            for (const std::string &answer : answers) {
                wxWindow *button = find_descendant(dialog, [&](wxWindow *w) {
                    return (dynamic_cast<wxButton *>(w) != nullptr || dynamic_cast<::Button *>(w) != nullptr) &&
                           visible(w) && w->IsEnabled() && plain(w->GetLabel()) == answer;
                });
                if (button == nullptr)
                    continue;
                m_dismissed.push_back({{"before_step", m_index}, {"title", into_u8(dialog->GetTitle())},
                                       {"texts", visible_texts(dialog)}, {"answer", answer}});
                wxGetApp().CallAfter([button] { fire_button(button); });
                return true;
            }
            m_dismissed.push_back({{"before_step", m_index}, {"title", into_u8(dialog->GetTitle())},
                                   {"texts", visible_texts(dialog)}, {"answer", "EndModal"}});
            dialog->EndModal(wxID_CANCEL);
            return true;
        }
        return false;
    }

    static void fire_button(wxWindow *button)
    {
        wxCommandEvent event(wxEVT_BUTTON, button->GetId());
        event.SetEventObject(button);
        button->GetEventHandler()->ProcessEvent(event);
    }

    void post_action()
    {
        m_modal_before.clear();
        for (wxDialog *dialog : shown_modal_dialogs())
            m_modal_before.insert(dialog);
        m_action = std::make_shared<Action>();
        m_phase = Phase::Posted;
        m_phase_start = Clock::now();
        std::shared_ptr<Action> action = m_action;
        const json current = step();
        wxGetApp().CallAfter([this, action, current] {
            action->started = true;
            try {
                action->ok = run(current, action->error);
            } catch (const std::exception &ex) {
                action->ok = false;
                action->error = std::string("exception: ") + ex.what();
            }
            action->returned = true;
        });
    }

    wxWindow *target(const std::string &in, std::string &error)
    {
        wxWindow *found = nullptr;
        if (in == "wizard")
            found = shown_wizard();
        else if (in == "dialog") {
            const auto dialogs = shown_modal_dialogs();
            if (!dialogs.empty())
                found = dialogs.back();
        } else if (in == "sidebar_panel")
            found = find_descendant(wxGetApp().mainframe, [](wxWindow *w) {
                return dynamic_cast<MixedNozzleSidebarPanel *>(w) != nullptr && visible(w);
            });
        else if (in == "main" || in.empty())
            found = wxGetApp().mainframe;
        if (found == nullptr)
            error = "no window for \"" + in + "\"";
        return found;
    }

    // Click, choose or tick a control by its label, the way the user would.
    bool click(const json &s, std::string &error)
    {
        wxWindow *root = target(s.value("in", std::string()), error);
        if (root == nullptr)
            return false;
        const std::string label = s.value("label", std::string());
        const std::string contains = s.value("label_contains", std::string());
        const auto matches = [&](const std::string &text) {
            return label.empty() ? (!contains.empty() && text.find(contains) != std::string::npos) : text == label;
        };
        // "nth" picks among controls with the same label, such as two Cancel buttons.
        std::size_t skip = s.value("nth", 0);
        wxWindow *control = find_descendant(root, [&](wxWindow *w) {
            const bool kind = dynamic_cast<wxButton *>(w) != nullptr || dynamic_cast<::Button *>(w) != nullptr ||
                              dynamic_cast<wxRadioButton *>(w) != nullptr || dynamic_cast<wxCheckBox *>(w) != nullptr;
            if (!(kind && visible(w) && matches(plain(w->GetLabel()))))
                return false;
            return skip-- == 0;
        });
        if (control == nullptr) {
            error = "no visible control labelled \"" + (label.empty() ? contains : label) + "\"";
            return false;
        }
        if (!control->IsEnabled()) {
            error = "control \"" + plain(control->GetLabel()) + "\" is disabled";
            return false;
        }
        if (auto *radio = dynamic_cast<wxRadioButton *>(control)) {
            radio->SetValue(true);
            wxCommandEvent event(wxEVT_RADIOBUTTON, radio->GetId());
            event.SetEventObject(radio);
            event.SetInt(1);
            radio->GetEventHandler()->ProcessEvent(event);
        } else if (auto *check = dynamic_cast<wxCheckBox *>(control)) {
            const bool value = s.value("value", !check->GetValue());
            check->SetValue(value);
            wxCommandEvent event(wxEVT_CHECKBOX, check->GetId());
            event.SetEventObject(check);
            event.SetInt(value ? 1 : 0);
            check->GetEventHandler()->ProcessEvent(event);
        } else {
            const std::string menu = s.value("menu", std::string());
            g_menu_answer = menu;
            g_menu_error.clear();
            fire_button(control);
            if (!menu.empty()) {
                const bool unused = !g_menu_answer.empty();
                g_menu_answer.clear();
                if (unused) {
                    error = "no menu opened for \"" + menu + "\"";
                    return false;
                }
                if (!g_menu_error.empty()) {
                    error = g_menu_error;
                    return false;
                }
            }
        }
        return true;
    }

    // Choose an item in a dropdown (wxChoice or the Widgets ComboBox) by the item's text, the nth
    // visible dropdown that offers it.
    bool choose(const json &s, std::string &error)
    {
        wxWindow *root = target(s.value("in", std::string()), error);
        if (root == nullptr)
            return false;
        const std::string item = s.value("item", std::string());
        const std::string contains = s.value("item_contains", std::string());
        // With no item text, "index" picks that item of the nth visible dropdown.
        const int by_index = s.value("index", -1);
        const auto matches = [&](const std::string &text) {
            return item.empty() ? (!contains.empty() && text.find(contains) != std::string::npos) : text == item;
        };
        const std::size_t nth = s.value("nth", 0);
        std::size_t seen = 0;
        std::vector<std::string> offered;
        wxWindow *found = nullptr;
        int index = -1;
        walk(root, [&](wxWindow *w) {
            if (found != nullptr || !visible(w))
                return;
            std::vector<std::string> items;
            if (auto *ch = dynamic_cast<wxChoice *>(w))
                for (unsigned int i = 0; i < ch->GetCount(); ++i) items.push_back(into_u8(ch->GetString(i)));
            else if (auto *combo = dynamic_cast<::ComboBox *>(w))
                for (unsigned int i = 0; i < combo->GetCount(); ++i) items.push_back(into_u8(combo->GetString(i)));
            else
                return;
            if (by_index >= 0 && item.empty() && contains.empty()) {
                if (seen++ == nth && by_index < int(items.size())) {
                    found = w;
                    index = by_index;
                }
                return;
            }
            for (std::size_t i = 0; i < items.size(); ++i) {
                if (!matches(items[i]))
                    continue;
                if (seen++ == nth) {
                    found = w;
                    index = int(i);
                }
                break;
            }
            if (found == nullptr && offered.size() < 40)
                offered.insert(offered.end(), items.begin(), items.end());
        });
        if (found == nullptr) {
            std::string list;
            for (const std::string &o : offered) list += (list.empty() ? "" : " | ") + o;
            error = "no visible dropdown " + std::to_string(nth) + " offers \"" + (item.empty() ? contains : item) +
                    "\" (offered: " + list.substr(0, 600) + ")";
            return false;
        }
        if (!found->IsEnabled()) {
            error = "dropdown offering \"" + (item.empty() ? contains : item) + "\" is disabled";
            return false;
        }
        if (auto *ch = dynamic_cast<wxChoice *>(found)) {
            ch->SetSelection(index);
            wxCommandEvent event(wxEVT_CHOICE, ch->GetId());
            event.SetEventObject(ch);
            event.SetInt(index);
            event.SetString(ch->GetString(index));
            ch->GetEventHandler()->ProcessEvent(event);
        } else
            static_cast<::ComboBox *>(found)->SelectAndNotify(index);
        return true;
    }

    // Move the pointer onto a text, as hovering a row does.
    bool hover(const json &s, std::string &error)
    {
        wxWindow *root = target(s.value("in", std::string()), error);
        if (root == nullptr)
            return false;
        const std::string contains = s.value("label_contains", std::string());
        wxWindow *text = find_descendant(root, [&](wxWindow *w) {
            return dynamic_cast<wxStaticText *>(w) != nullptr && visible(w) &&
                   into_u8(w->GetLabel()).find(contains) != std::string::npos;
        });
        if (text == nullptr) {
            error = "no visible text containing \"" + contains + "\"";
            return false;
        }
        wxMouseEvent event(wxEVT_ENTER_WINDOW);
        event.SetEventObject(text);
        text->GetEventHandler()->ProcessEvent(event);
        return true;
    }

    // Resize the target's window, as dragging its edge does.
    bool resize(const json &s, std::string &error)
    {
        wxWindow *root = target(s.value("in", std::string()), error);
        if (root == nullptr)
            return false;
        wxWindow *top = wxGetTopLevelParent(root);
        const wxSize now = top->GetSize();
        top->SetSize(wxSize(s.value("width", now.x), s.value("height", now.y)));
        top->Layout();
        return true;
    }

    // Confirm on the Add/Remove filament page, a web page a script cannot tick, with the materials
    // the step adds and removes: what GuideFrame::run() and GUI_App::run_wizard() do after OK.
    bool confirm_filament_selection(const json &s, std::string &error)
    {
        GUI_App &app = wxGetApp();
        app.preset_bundle->export_selections(*app.app_config);
        std::map<std::string, std::string> filaments;
        if (app.app_config->has_section(AppConfig::SECTION_FILAMENTS))
            filaments = app.app_config->get_section(AppConfig::SECTION_FILAMENTS);
        const std::map<std::string, std::string> before = filaments;
        for (const std::string &name : s.value("add", std::vector<std::string>{}))
            filaments[name] = "true";
        for (const std::string &name : s.value("remove", std::vector<std::string>{}))
            filaments.erase(name);
        bool keep = false;
        if (filaments != before &&
            !app.check_and_keep_current_preset_changes(_L("Configuration package changed"),
                                                       _L("The configuration package is changed in previous Config Guide"),
                                                       ActionButtons::KEEP | ActionButtons::SAVE, &keep))
            return true;
        if (!app.preset_bundle->apply_vendor_config(app.app_config->vendors(), filaments, app.app_config, true, "", "")) {
            error = "apply_vendor_config failed";
            return false;
        }
        if (keep)
            app.apply_keeped_preset_modifications();
        app.app_config->set_legacy_datadir(false);
        app.update_mode();
        app.load_current_presets();
        app.update_publish_status();
        app.mainframe->refresh_plugin_tips();
        return true;
    }

    MixedNozzleWizardDialog *wizard(std::string &error)
    {
        MixedNozzleWizardDialog *dialog = shown_wizard();
        if (dialog == nullptr)
            error = "setup wizard is not open";
        return dialog;
    }

    bool run(const json &s, std::string &error)
    {
        const std::string what = s.value("do", std::string());
        Plater *plater = wxGetApp().plater();
        Sidebar &sidebar = plater->sidebar();
        if (what == "new_project") {
            plater->new_project();
            return true;
        }
        if (what == "select_printer")
            return TestModeSidebarAccess::pick(sidebar, "printer", s.at("name").get<std::string>(), error);
        if (what == "set_bed_type")
            return TestModeSidebarAccess::pick(sidebar, "bed_type", s.at("name").get<std::string>(), error);
        if (what == "set_nozzle" || what == "set_flow") {
            const std::string side = s.value("side", std::string("left"));
            const std::string which = side + (what == "set_nozzle" ? "_diameter" : "_flow");
            return TestModeSidebarAccess::pick(sidebar, which,
                s.at(what == "set_nozzle" ? "diameter" : "flow").get<std::string>(), error,
                s.value("from_list", false));
        }
        if (what == "set_filament_count") {
            const std::size_t count = s.at("count").get<std::size_t>();
            for (int guard = 0; guard < 32 && wxGetApp().preset_bundle->filament_presets.size() != count; ++guard) {
                const bool add = wxGetApp().preset_bundle->filament_presets.size() < count;
                if (!TestModeSidebarAccess::click(sidebar, add ? "add_filament" : "delete_filament", error))
                    return false;
            }
            return wxGetApp().preset_bundle->filament_presets.size() == count;
        }
        if (what == "filament_selection")
            return confirm_filament_selection(s, error);
        if (what == "set_filament")
            return TestModeSidebarAccess::pick(sidebar, "filament:" + std::to_string(s.at("slot").get<int>()),
                                               s.at("name").get<std::string>(), error);
        if (what == "add_shape") {
            wxGetApp().obj_list()->load_generic_subobject(s.value("shape", std::string("Cube")), ModelVolumeType::INVALID);
            return true;
        }
        if (what == "add_part") {
            ObjectList *list = wxGetApp().obj_list();
            list->select_item([list, &s] { return list->GetModel()->GetItemById(s.value("object", 0)); });
            list->load_generic_subobject(s.value("shape", std::string("Cube")), ModelVolumeType::MODEL_PART);
            return true;
        }
        // The material an object prints with, as choosing it in the object list does.
        if (what == "set_object_filament") {
            ObjectList *list = wxGetApp().obj_list();
            list->select_item([list, &s] { return list->GetModel()->GetItemById(s.value("object", 0)); });
            list->set_extruder_for_selected_items(s.at("slot").get<int>());
            return true;
        }
        // The current plate's material-to-nozzle map, set by hand ("maps": nozzle 1 or 2 per material).
        if (what == "set_filament_map") {
            PartPlate *plate = plater->get_partplate_list().get_curr_plate();
            if (plate == nullptr) {
                error = "no current plate";
                return false;
            }
            plate->set_filament_map_mode(fmmManual);
            plate->set_filament_maps(s.at("maps").get<std::vector<int>>());
            plater->update();
            return true;
        }
        if (what == "load_file") {
            plater->load_files(std::vector<std::string>{s.at("path").get<std::string>()});
            return true;
        }
        if (what == "open_wizard")
            return TestModeSidebarAccess::click(sidebar, "mixed_setup", error);
        // Folds or unfolds a sidebar section ("printer_title" or "filament_title"), as clicking its title does.
        if (what == "fold_section")
            return TestModeSidebarAccess::click(sidebar, s.at("section").get<std::string>(), error);
        if (what == "click")
            return click(s, error);
        if (what == "choose")
            return choose(s, error);
        if (what == "hover")
            return hover(s, error);
        if (what == "resize")
            return resize(s, error);
        if (what == "wizard_role") {
            MixedNozzleWizardDialog *dialog = wizard(error);
            return dialog != nullptr && TestModeWizardAccess::set_role(*dialog, s.at("part").get<std::size_t>(),
                                                                       s.at("role").get<std::string>() == "fine", error);
        }
        if (what == "wizard_material") {
            MixedNozzleWizardDialog *dialog = wizard(error);
            return dialog != nullptr && TestModeWizardAccess::set_material(*dialog, s.at("role").get<std::string>() == "fine",
                                                                           s.at("slot").get<int>(), error);
        }
        if (what == "wizard_slot") {
            MixedNozzleWizardDialog *dialog = wizard(error);
            return dialog != nullptr && TestModeWizardAccess::set_exact_slot(*dialog, s.at("part").get<std::size_t>(),
                                                                             s.at("slot").get<int>(), error);
        }
        if (what == "wizard_row") {
            MixedNozzleWizardDialog *dialog = wizard(error);
            return dialog != nullptr && TestModeWizardAccess::pick_row(*dialog, s.at("row").get<std::size_t>(), error);
        }
        if (what == "undo") {
            plater->undo();
            return true;
        }
        if (what == "redo") {
            plater->redo();
            return true;
        }
        if (what == "slice") {
            wxPostEvent(plater, SimpleEvent(EVT_GLTOOLBAR_SLICE_PLATE));
            return true;
        }
        if (what == "plate_settings") {
            wxCommandEvent event;
            plater->open_platesettings_dialog(event);
            return true;
        }
        if (what == "save_3mf") {
            const boost::filesystem::path path = m_out / s.at("name").get<std::string>();
            return plater->export_3mf(path) == 0 || (error = "export_3mf failed", false);
        }
        if (what == "open_3mf") {
            plater->load_project(from_u8((m_out / s.at("name").get<std::string>()).string()));
            return true;
        }
        // A pretend printer (TestModePrinter.hpp) for the printer sync. Connecting again with other
        // nozzles or spools is the printer reporting a change.
        if (what == "connect_printer") {
            const json spec = s.value("printer", json::object());
            if (g_fake_printer)
                return TestMode::update_fake_printer(*g_fake_printer, spec, error);
            g_fake_printer = TestMode::make_fake_printer(wxGetApp().getDeviceManager(), spec, error);
            return g_fake_printer != nullptr;
        }
        if (what == "disconnect_printer") {
            g_fake_printer.reset();
            return true;
        }
        // The sidebar's printer Sync button.
        if (what == "sync_printer") {
            sidebar.deal_btn_sync();
            return true;
        }
        if (what == "wait" || what == "dump")
            return true;
        error = "unknown step \"" + what + "\"";
        return false;
    }

    json state()
    {
        json out;
        auto &bundle = *wxGetApp().preset_bundle;
        const DynamicPrintConfig &project = bundle.project_config;
        Plater *plater = wxGetApp().plater();
        json &p = out["project"];
        p["mode"] = mode_name(mixed_nozzle_project_default(project));
        p["printer_preset"] = bundle.printers.get_edited_preset().name;
        if (const auto *d = bundle.printers.get_edited_preset().config.option<ConfigOptionFloats>("nozzle_diameter"))
            p["printer_nozzle_diameter"] = d->values;
        if (const auto *d = project.option<ConfigOptionFloats>("nozzle_diameter"))
            p["nozzle_diameter_override"] = d->values;
        json flows = json::array();
        if (const auto *types = project.option<ConfigOptionEnumsGeneric>("nozzle_volume_type"))
            for (int type : types->values)
                flows.push_back(get_nozzle_volume_type_string(NozzleVolumeType(type)));
        p["nozzle_volume_type"] = flows;
        if (const auto *stats = bundle.printers.get_edited_preset().config.option<ConfigOptionStrings>("extruder_nozzle_stats"))
            p["printer_nozzle_stats"] = stats->values;
        p["filament_presets"] = bundle.filament_presets;
        if (const auto *colours = project.option<ConfigOptionStrings>("filament_colour"))
            p["filament_colour"] = colours->values;
        for (const std::string &key : project.keys())
            if (key.rfind("mixed_nozzle", 0) == 0 || key == "filament_map" || key == "filament_nozzle_map" ||
                key.find("prime_tower") != std::string::npos)
                p["keys"][key] = project.opt_serialize(key);
        json plates = json::array();
        PartPlateList &list = plater->get_partplate_list();
        for (int i = 0; i < list.get_plate_count(); ++i) {
            PartPlate *plate = list.get_plate(i);
            const MixedNozzleModeSource mode = effective_mixed_nozzle_mode(project, *plate);
            json entry {{"index", i}, {"mode", mode_name(mode.effective)}, {"inherits_project", mode.inherits_project},
                        {"filament_maps", plate->get_real_filament_maps(project)},
                        {"slice_valid", plate->is_slice_result_valid()}};
            for (const std::string &key : plate->config()->keys())
                entry["overrides"][key] = plate->config()->opt_serialize(key);
            plates.push_back(entry);
        }
        out["plates"] = plates;
        out["current_plate"] = list.get_curr_plate_index();
        // The bed type as each place sees it: project, current plate's own, the canvas label,
        // the Plate Settings tab and the printer's remembered choice.
        {
            json &bed = out["bed"];
            if (const auto *opt = project.option<ConfigOptionEnum<BedType>>("curr_bed_type"))
                bed["project"] = bed_name(opt->value);
            if (PartPlate *plate = list.get_curr_plate()) {
                bed["plate"] = bed_name(plate->get_bed_type(false));
                bed["canvas"] = bed_name(plate->get_bed_type(true));
            }
            if (Tab *tab = wxGetApp().get_plate_tab(); tab != nullptr && tab->get_config() != nullptr)
                if (const auto *opt = tab->get_config()->option<ConfigOptionEnum<BedType>>("curr_bed_type"))
                    bed["plate_tab"] = bed_name(opt->value);
            bed["printer_setting"] = wxGetApp().app_config->get_printer_setting(bundle.printers.get_selected_preset_name(),
                                                                                "curr_bed_type");
        }
        out["undo"] = {{"can_undo", plater->can_undo()}, {"can_redo", plater->can_redo()}};
        out["slicing"] = plater->is_background_process_slicing();

        json objects = json::array();
        ObjectDataViewModel *view = wxGetApp().obj_list()->GetModel();
        const ModelObjectPtrs &model_objects = plater->model().objects;
        for (std::size_t oi = 0; oi < model_objects.size(); ++oi) {
            const ModelObject &object = *model_objects[oi];
            const int object_slot = object.config.has("extruder") ? object.config.extruder() : 0;
            const wxDataViewItem item = view->GetItemById(int(oi));
            json entry {{"name", object.name}, {"shown", into_u8(view->GetExtruder(item))}, {"slot", object_slot}};
            json volumes = json::array();
            for (std::size_t vi = 0; vi < object.volumes.size(); ++vi) {
                const ModelVolume &volume = *object.volumes[vi];
                const int own = volume.config.has("extruder") ? volume.config.extruder() : 0;
                const wxDataViewItem vitem = view->GetItemByVolumeId(int(oi), int(vi));
                volumes.push_back({{"name", volume.name}, {"part", volume.is_model_part()},
                                   {"shown", vitem.IsOk() ? into_u8(view->GetExtruder(vitem)) : std::string()},
                                   {"own_slot", own},
                                   {"slot", own > 0 ? own : object_slot > 0 ? object_slot : 1}});
            }
            entry["volumes"] = volumes;
            objects.push_back(entry);
        }
        out["objects"] = objects;

        TestModeSidebarAccess::dump(plater->sidebar(), out["sidebar"]);
        if (wxWindow *panel = find_descendant(wxGetApp().mainframe, [](wxWindow *w) {
                return dynamic_cast<MixedNozzleSidebarPanel *>(w) != nullptr && visible(w);
            }))
        {
            out["sidebar"]["panel_lines"] = visible_texts(panel, true);
            out["sidebar"]["panel_buttons"] = visible_buttons(panel);
            out["sidebar"]["panel_choices"] = visible_choices(panel);
            out["sidebar"]["panel_tooltips"] = visible_tooltips(panel, true);
            out["sidebar"]["panel_low_contrast"] = low_contrast_texts(panel);
            out["sidebar"]["panel_cut"] = cut_texts(panel);
            // Zero once the section has the height its lines ask for, after growing or shrinking.
            // How far the section's height is from the height it asks for, and how much of its
            // content sits outside its view. Both are zero when it shows everything.
            out["sidebar"]["panel_extra_height"] = panel->GetSize().y - panel->GetBestSize().y;
            if (panel->GetSizer() != nullptr)
                out["sidebar"]["panel_scrolled_out"] = panel->GetSizer()->GetMinSize().y - panel->GetClientSize().y;
            // Visible lines and buttons of the section that are not wholly in view, scrolled away or cut.
            json out_of_view = json::array();
            wxRect view(panel->ClientToScreen(wxPoint(0, 0)), panel->GetClientSize());
            for (wxWindow *a = panel->GetParent(); a != nullptr; a = a->GetParent()) {
                view.Intersect(wxRect(a->ClientToScreen(wxPoint(0, 0)), a->GetClientSize()));
                if (a->IsTopLevel())
                    break;
            }
            walk(panel, [&](wxWindow *w) {
                const bool shown_item = dynamic_cast<wxStaticText *>(w) != nullptr || dynamic_cast<::Button *>(w) != nullptr;
                const wxRect rect(w->GetScreenPosition(), w->GetSize());
                if (shown_item && visible(w) && !view.Contains(rect))
                    out_of_view.push_back(label_text(w, true));
            });
            out["sidebar"]["panel_out_of_view"] = out_of_view;
        }
        // Every section of the sidebar, not only this one: a section that does not grow pushes
        // its controls under the next section.
        out["sidebar"]["layout_hidden"] = sidebar_layout_hidden();
        if (ParamsPanel *params = wxGetApp().mainframe->m_param_panel; params != nullptr && params->get_paged_view() != nullptr)
            out["sidebar"]["process_page_height"] = params->ToDIP(params->get_paged_view()->GetClientSize().y);
        json selected = json::array();
        const Selection &selection = plater->get_selection();
        for (unsigned int idx : selection.get_volume_idxs())
            if (const GLVolume *volume = selection.get_volume(idx))
                selected.push_back({{"object", volume->object_idx()}, {"volume", volume->volume_idx()}});
        out["selection"] = selected;
        out["last_menu"] = g_menu_last;
        // What the current plate's slice used: the material profile names the Preview legend shows,
        // and the tallest extrusion per role, which sets the top of the Layer Height legend.
        if (PartPlate *plate = list.get_curr_plate(); plate != nullptr && plate->is_slice_result_valid()) {
            json &sliced = out["sliced"];
            if (Print *print = plate->fff_print()) {
                if (const auto *names = print->full_print_config().option<ConfigOptionStrings>("filament_settings_id"))
                    sliced["filament_settings_id"] = names->values;
                sliced["bed_type"] = bed_name(print->config().curr_bed_type.value);
            }
            if (const GCodeProcessorResult *result = plate->get_slice_result()) {
                sliced["result_filament_ids"] = result->settings_ids.filament;
                // The bed type the G-code's config block names.
                std::string gcode_path = result->filename;
                if (gcode_path.empty() || !boost::filesystem::exists(gcode_path))
                    gcode_path = plate->get_tmp_gcode_path();
                // And the nozzle flow keys it names, as gcode.<key>.
                std::ifstream gcode(gcode_path);
                const std::string key = "; curr_bed_type = ";
                for (std::string line; std::getline(gcode, line);) {
                    if (line.rfind(key, 0) == 0)
                        sliced["gcode_bed_type"] = line.substr(key.size());
                    for (const char *flow_key : {"nozzle_volume_type", "filament_volume_map", "extruder_nozzle_stats",
                                                 "filament_max_volumetric_speed", "nozzle_diameter", "filament_map",
                                                 "filament_settings_id"})
                        if (const std::string head = std::string("; ") + flow_key + " = "; line.rfind(head, 0) == 0)
                            sliced["gcode"][flow_key] = line.substr(head.size());
                }
                json tallest;
                for (const auto &move : result->moves) {
                    if (move.type != EMoveType::Extrude)
                        continue;
                    const std::string role = ExtrusionEntity::role_to_string(move.extrusion_role);
                    if (!tallest.contains(role) || tallest[role]["height"].get<float>() < move.height)
                        tallest[role] = {{"height", move.height}, {"z", move.position.z()}, {"tool", int(move.extruder_id)}};
                }
                sliced["tallest_extrusion"] = tallest;
            }
        }
        out["printer"] = g_fake_printer ? TestMode::describe_printer(*g_fake_printer) : json {{"connected", false}};
        if (g_fake_printer)
            out["printer"]["connected"] = true;
        out["dialog_history"] = m_dialog_history;
        out["validations"] = g_validations;
        g_validations = json::array();
        if (const PartPlate *plate = list.get_curr_plate())
            out["apply_invalid"] = plate->is_apply_result_invalid();
        if (MixedNozzleWizardDialog *dialog = shown_wizard())
            TestModeWizardAccess::dump(*dialog, out["wizard"]);
        return out;
    }

    void record_step()
    {
        const json &s = step();
        char prefix[8];
        std::snprintf(prefix, sizeof(prefix), "%02zu", m_index);
        const std::string base = std::string(prefix) + "-" + s.value("do", std::string("step"));
        json result {{"index", m_index}, {"step", s}, {"ok", m_action->ok}, {"blocked_in_modal", m_blocked_in_modal}};
        if (!m_action->error.empty())
            result["error"] = m_action->error;
        if (!m_action->ok)
            m_failures++;
        json st = state();
        json dialogs = json::array();
        json overflow = json::array();
        int count = 0;
        for (wxWindow *w : wxTopLevelWindows) {
            auto *top = dynamic_cast<wxTopLevelWindow *>(w);
            if (top == nullptr || !top->IsShown() || top == wxGetApp().mainframe)
                continue;
            if (dynamic_cast<wxDialog *>(top) == nullptr)
                continue;
            const std::string title = into_u8(top->GetTitle());
            json entry {{"title", title}, {"class", wxString(top->GetClassInfo()->GetClassName()).ToStdString()},
                        {"size", {top->GetSize().x, top->GetSize().y}}, {"texts", visible_texts(top)},
                        {"buttons", visible_buttons(top)}, {"choices", visible_choices(top)},
                        {"checkboxes", visible_checkboxes(top)}, {"buttons_touching", touching_buttons(top)},
                        {"tooltips", visible_tooltips(top)}};
            entry["class"] = dialog_class(top);
            std::string capture_error;
            const std::string png = capture_window(top, m_out / (base + "-" + std::to_string(count++) + ".png"), capture_error);
            if (!png.empty())
                entry["png"] = png;
            else
                entry["capture_error"] = capture_error;
            check_overflow(top, title, overflow);
            dialogs.push_back(entry);
        }
        // The sidebar process preset list with each item's tooltip, as hovering the open list shows.
        if (s.value("process_combo", false)) {
            json items = json::array();
            if (wxWindow *w = find_descendant(wxGetApp().mainframe, [](wxWindow *w) {
                    auto *combo = dynamic_cast<PresetComboBox *>(w);
                    return combo != nullptr && combo->get_type() == Preset::TYPE_PRINT && visible(w);
                })) {
                auto *combo = static_cast<PresetComboBox *>(w);
                st["process_combo_value"] = into_u8(combo->GetValue());
                for (unsigned int i = 0; i < combo->GetCount(); ++i)
                    items.push_back({{"label", into_u8(combo->GetString(i))}, {"tooltip", into_u8(combo->GetItemTooltip(i))}});
            }
            st["process_combo"] = items;
        }
        if (s.value("capture_main", false)) {
            std::string capture_error;
            const std::string png = capture_window(wxGetApp().mainframe, m_out / (base + "-main.png"), capture_error);
            st["main_png"] = png.empty() ? capture_error : png;
        }
        json sidebar_overflow = json::array();
        if (wxWindow *panel = find_descendant(wxGetApp().mainframe, [](wxWindow *w) {
                return dynamic_cast<MixedNozzleSidebarPanel *>(w) != nullptr && visible(w);
            }))
            check_overflow(panel, "sidebar_panel", sidebar_overflow);
        st["dialogs"] = dialogs;
        st["overflow"] = overflow;
        st["overflow_sidebar"] = sidebar_overflow;
        st["step"] = result;
        std::ofstream((m_out / (base + ".json")).string()) << st.dump(2);
        result["state_file"] = base + ".json";
        result["overflow"] = overflow.size();
        m_results.push_back(result);
        write_summary(-1);
    }

    void fail_step(const std::string &error)
    {
        m_failures++;
        json st;
        try {
            st = state();
        } catch (...) {
        }
        m_results.push_back({{"index", m_index}, {"step", m_index < m_steps.size() ? m_steps[m_index] : json()},
                             {"ok", false}, {"error", error}, {"state", st}});
        // A step that cannot finish leaves the app in an unknown state; stop here.
        finish(2);
    }

    void write_summary(int exit_code)
    {
        json summary {{"scenario", m_script.value("name", std::string())}, {"steps", m_results},
                      {"auto_dismissed", m_dismissed}, {"failures", m_failures}, {"exit_code", exit_code},
                      {"finished", exit_code >= 0}};
        std::ofstream((m_out / "result.json").string()) << summary.dump(2);
    }

    void finish(int code)
    {
        m_timer.Stop();
        write_summary(code);
        std::fflush(nullptr);
        std::_Exit(code);
    }
};

Runner *g_runner = nullptr;

} // namespace

namespace TestMode {

bool active()
{
    static const bool on = !env("CADENCE_TEST_SCRIPT").empty();
    return on;
}

void start()
{
    if (!active() || g_runner != nullptr)
        return;
    const std::string script_path = env("CADENCE_TEST_SCRIPT");
    std::string out = env("CADENCE_TEST_OUT");
    if (out.empty())
        out = (boost::filesystem::path(script_path).parent_path() / "out").string();
    boost::filesystem::create_directories(out);
    json script;
    try {
        std::ifstream in(script_path);
        in >> script;
    } catch (const std::exception &ex) {
        std::ofstream((boost::filesystem::path(out) / "result.json").string())
            << json({{"error", std::string("cannot read scenario: ") + ex.what()}, {"exit_code", 4}}).dump(2);
        std::_Exit(4);
    }
    g_runner = new Runner(std::move(script), out);
}

void note_validation(const std::string &error)
{
    if (active())
        g_validations.push_back(error);
}

MachineObject *fake_printer()
{
    return active() ? g_fake_printer.get() : nullptr;
}

int popup_menu(wxWindow &owner, wxMenu &menu, const wxPoint &at)
{
    if (!active() || g_menu_answer.empty())
        return owner.GetPopupMenuSelectionFromUser(menu, at);
    const std::string answer = g_menu_answer;
    g_menu_answer.clear();
    for (wxMenuItem *item : menu.GetMenuItems()) {
        if (item->IsSeparator() || plain(item->GetItemLabelText()) != answer)
            continue;
        g_menu_last = {{"item", answer}, {"enabled", item->IsEnabled()},
                       {"checkable", item->IsCheckable()}, {"checked", item->IsCheckable() && item->IsChecked()}};
        if (!item->IsEnabled()) {
            g_menu_error = "menu item \"" + answer + "\" is disabled";
            return wxID_NONE;
        }
        return item->GetId();
    }
    std::string offered;
    for (wxMenuItem *item : menu.GetMenuItems())
        if (!item->IsSeparator())
            offered += (offered.empty() ? "" : " | ") + plain(item->GetItemLabelText());
    g_menu_error = "menu has no item \"" + answer + "\" (offered: " + offered + ")";
    return wxID_NONE;
}

} // namespace TestMode

// ---- wizard controls whose labels repeat -------------------------------------------------------

bool TestModeWizardAccess::set_role(MixedNozzleWizardDialog &dialog, std::size_t part, bool fine, std::string &error)
{
    if (part >= dialog.m_body_roles.size()) {
        error = "no part row " + std::to_string(part);
        return false;
    }
    wxRadioButton *radio = fine ? dialog.m_body_roles[part].first : dialog.m_body_roles[part].second;
    if (!visible(radio) || !radio->IsEnabled()) {
        error = "part role button is hidden or disabled";
        return false;
    }
    radio->SetValue(true);
    wxCommandEvent event(wxEVT_RADIOBUTTON, radio->GetId());
    event.SetEventObject(radio);
    event.SetInt(1);
    radio->GetEventHandler()->ProcessEvent(event);
    return true;
}

bool TestModeWizardAccess::set_material(MixedNozzleWizardDialog &dialog, bool fine, int slot, std::string &error)
{
    ::ComboBox *combo = fine ? dialog.m_fine_filament : dialog.m_coarse_filament;
    if (slot < 1 || unsigned(slot) > combo->GetCount()) {
        error = "no material slot " + std::to_string(slot);
        return false;
    }
    if (!visible(combo) || !combo->IsEnabled()) {
        error = "material picker is hidden or disabled";
        return false;
    }
    combo->SelectAndNotify(slot - 1);
    return true;
}

bool TestModeWizardAccess::set_exact_slot(MixedNozzleWizardDialog &dialog, std::size_t part, int slot, std::string &error)
{
    if (part >= dialog.m_body_slot_overrides.size()) {
        error = "no exact-slot row " + std::to_string(part);
        return false;
    }
    wxChoice *choice = dialog.m_body_slot_overrides[part];
    if (slot < 0 || unsigned(slot) >= choice->GetCount() || !visible(choice)) {
        error = "exact slot " + std::to_string(slot) + " not offered or hidden";
        return false;
    }
    choice->SetSelection(slot);
    wxCommandEvent event(wxEVT_CHOICE, choice->GetId());
    event.SetEventObject(choice);
    event.SetInt(slot);
    choice->GetEventHandler()->ProcessEvent(event);
    return true;
}

bool TestModeWizardAccess::pick_row(MixedNozzleWizardDialog &dialog, std::size_t row, std::string &error)
{
    if (row >= dialog.m_speed_buttons.size()) {
        error = "no coarse layer row " + std::to_string(row);
        return false;
    }
    wxRadioButton *radio = dialog.m_speed_buttons[row];
    radio->SetValue(true);
    wxCommandEvent event(wxEVT_RADIOBUTTON, radio->GetId());
    event.SetEventObject(radio);
    event.SetInt(1);
    radio->GetEventHandler()->ProcessEvent(event);
    return true;
}

void TestModeWizardAccess::dump(MixedNozzleWizardDialog &dialog, json &out)
{
    const int page = dialog.m_pages->GetSelection();
    out["page"] = page;
    out["page_title"] = page >= 0 ? into_u8(dialog.m_pages->GetPageText(std::size_t(page))) : std::string();
    out["mode"] = mode_name(dialog.selected_mode());
    out["draft_mode"] = mode_name(dialog.m_input.draft.mode);
    json roles = json::array();
    for (std::size_t i = 0; i < dialog.m_body_roles.size(); ++i) {
        const auto &row = dialog.m_input.body_rows[i];
        const bool fine = dialog.m_body_roles[i].first->GetValue();
        const bool coarse = dialog.m_body_roles[i].second->GetValue();
        roles.push_back({{"part", row.volume_name}, {"role", fine ? "fine" : coarse ? "coarse" : "none"},
                         {"note", i < dialog.m_body_role_notes.size() ? into_u8(dialog.m_body_role_notes[i]->GetLabel()) : std::string()},
                         {"exact_slot", i < dialog.m_body_slot_overrides.size() ? dialog.m_body_slot_overrides[i]->GetSelection() : -1}});
    }
    out["roles"] = roles;
    out["fine_material"] = into_u8(dialog.m_fine_filament->GetValue());
    out["fine_slot"] = dialog.m_fine_filament->GetSelection() + 1;
    out["coarse_material"] = into_u8(dialog.m_coarse_filament->GetValue());
    out["coarse_slot"] = dialog.m_coarse_filament->GetSelection() + 1;
    out["fine_resolved"] = into_u8(dialog.m_fine_resolved->GetLabel());
    out["coarse_resolved"] = into_u8(dialog.m_coarse_resolved->GetLabel());
    out["status"] = into_u8(dialog.m_status->GetLabel());
    out["tower"] = dialog.m_tower != nullptr ? into_u8(dialog.m_tower->GetStringSelection()) : std::string();
    out["scope"] = dialog.m_scope != nullptr ? into_u8(dialog.m_scope->GetStringSelection()) : std::string();
    json rows = json::array();
    for (std::size_t i = 0; i < dialog.m_speed_buttons.size(); ++i)
        rows.push_back({{"label", into_u8(dialog.m_speed_buttons[i]->GetLabel())},
                        {"selected", dialog.m_speed_buttons[i]->GetValue()}});
    out["coarse_rows"] = rows;
    out["cadence_time"] = dialog.m_cadence_time != nullptr ? into_u8(dialog.m_cadence_time->GetLabel()) : std::string();
    // Busy while any row is slicing or has no time yet, not only during a slice.
    bool busy = false;
    for (const auto &row : dialog.m_cadence_page.rows)
        busy = busy || row.slicing ||
               (dialog.m_ranking && (!row.estimate || row.estimate->status == WizardEstimateStatus::Pending));
    out["ranking_busy"] = busy;
    out["ready_line"] = dialog.m_ready_line != nullptr ? into_u8(dialog.m_ready_line->GetLabel()) : std::string();
    out["can_apply"] = dialog.m_review.can_apply;
    json buttons;
    for (wxButton *button : {dialog.m_more, dialog.m_done, dialog.m_back, dialog.m_next, dialog.m_apply, dialog.m_apply_slice})
        if (button != nullptr)
            buttons[plain(button->GetLabel())] = {{"shown", button->IsShown()}, {"enabled", button->IsEnabled()}};
    out["buttons"] = buttons;
    // What the wizard's own wrap would use for the step 3 lines, to tell a missed wrap from a
    // wrong width.
    json probe = json::array();
    for (wxStaticText *text : {dialog.m_fine_help, dialog.m_coarse_help, dialog.m_speed_summary, dialog.m_cadence_time,
                               dialog.m_times_note, dialog.m_basis_line})
        if (text != nullptr)
            probe.push_back({{"text", into_u8(text->GetLabel()).substr(0, 40)}, {"shown", text->IsShown()},
                             {"wrapped", text->GetLabel().Find('\n') != wxNOT_FOUND},
                             {"wrap_width", dialog.wrap_width(text, 0)}, {"x", text->GetPosition().x},
                             {"width", text->GetSize().x},
                             {"page_width", dialog.m_pages->GetPage(2)->GetSize().x}});
    out["wrap_probe"] = probe;
    // Where Joining sits in the More options page's visible area, and what has the focus.
    if (auto *more = dialog.m_pages->GetPage(std::size_t(MixedNozzleWizardPage::MoreOptions)); more != nullptr &&
        dialog.m_joining_panel != nullptr) {
        // In view: the Joining heading and the first choice under it are inside the visible area.
        const int top = dialog.m_joining_panel->GetPosition().y;
        int needed = 0;
        if (!dialog.m_body_joining.empty()) {
            wxWindow *first = dialog.m_body_joining.front().second;
            needed = first->GetScreenRect().GetBottom() - dialog.m_joining_panel->GetScreenPosition().y;
        }
        out["joining"] = {{"shown", dialog.m_joining_panel->IsShown()}, {"top", top}, {"needed", needed},
                          {"page_height", more->GetClientSize().y},
                          {"in_view", dialog.m_joining_panel->IsShown() && top >= 0 &&
                                          top + needed <= more->GetClientSize().y}};
    }
    bool joining_focus = false;
    for (const auto &entry : dialog.m_body_joining)
        joining_focus = joining_focus || wxWindow::FindFocus() == entry.second;
    out["joining_focused"] = joining_focus;
}

} // namespace Slic3r::GUI
