#include "MixedNozzleSetupDialog.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <set>

#include <wx/display.h>
#include <wx/stattext.h>
#include <wx/simplebook.h>
#include <wx/scrolwin.h>
#include <wx/choice.h>
#include <wx/checkbox.h>
#include <wx/radiobut.h>
#include <wx/textctrl.h>
#include <wx/button.h>
#include <wx/timer.h>
#include <wx/collpane.h>
#include <wx/panel.h>
#include <wx/settings.h>
#include <wx/activityindicator.h>
#include <wx/gauge.h>
#include <wx/statbmp.h>

#include "libslic3r/MixedNozzleRanking.hpp"
#include "libslic3r/Slicing.hpp"

#include "GUI_App.hpp"
#include "MsgDialog.hpp"
#include "wxExtensions.hpp"
#include "Widgets/ComboBox.hpp"
#include "Widgets/DialogButtons.hpp"

namespace Slic3r::GUI {

namespace {
// Page order of the wizard's simplebook.
constexpr int kModePage = int(MixedNozzleWizardPage::ModeAndScope);
constexpr int kMaterialsPage = int(MixedNozzleWizardPage::Materials);
constexpr int kSpeedPage = int(MixedNozzleWizardPage::Speed);
constexpr int kReviewPage = int(MixedNozzleWizardPage::Review);
constexpr int kMorePage = int(MixedNozzleWizardPage::MoreOptions);

// The reasons and their wording live in the model.
wxString wizard_review_message(const std::string &reason)
{
    return from_u8(wizard_review_reason_text(reason));
}

// What a finished ranking slice means for its row.
WizardEstimate ranking_estimate(const MixedNozzleSliceTime &time)
{
    WizardEstimate estimate;
    estimate.status = WizardEstimateStatus::Unavailable;
    switch (time.status) {
    case MixedNozzleSliceTimeStatus::Estimated:
        return wizard_estimate(time.seconds, time.switches, time.tower_mm3);
    case MixedNozzleSliceTimeStatus::Refused:
        estimate.note = "slicing would refuse this choice";
        estimate.refused = true;
        estimate.refusal = time.diagnostic;
        break;
    case MixedNozzleSliceTimeStatus::Cancelled:
        estimate.note = "not estimated";
        break;
    case MixedNozzleSliceTimeStatus::Failed:
        // A row that fails to slice says so on its own row.
        estimate.status = WizardEstimateStatus::Failed;
        break;
    }
    return estimate;
}

double ranking_clock_seconds()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

MixedNozzleIntroDialog::MixedNozzleIntroDialog(wxWindow *parent)
    : DPIDialog(parent, wxID_ANY, _L("About mixed-nozzle slicing"), wxDefaultPosition,
                wxDefaultSize, wxDEFAULT_DIALOG_STYLE)
{
    // The one page "?" opens. No picture panels until there is artwork for them.
    auto *root = new wxBoxSizer(wxVERTICAL);
    const auto paragraph = [this, root](const wxString &text, bool bold) {
        auto *control = new wxStaticText(this, wxID_ANY, text);
        if (bold)
            control->SetFont(control->GetFont().Bold());
        control->Wrap(FromDIP(460));
        root->Add(control, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, FromDIP(12));
    };
    paragraph(from_u8(wizard_intro_lead_line()), false);
    paragraph(_L("Feature Split"), true);
    paragraph(_L("Split by feature. By default the fine nozzle prints walls, top and bottom surfaces and solid "
                 "infill, and the coarse nozzle prints sparse infill. You can change which nozzle prints each "
                 "feature, supports included. Works on any model."), false);
    paragraph(_L("Body Split"), true);
    paragraph(_L("Split by part. Each part prints entirely on the nozzle you pick for it, outside included."), false);
    paragraph(_L("It needs two different nozzles, a material on each, and a prime tower. Setup sets the rest."), false);
    auto *buttons = new wxBoxSizer(wxHORIZONTAL);
    buttons->AddStretchSpacer();
    auto *close = new wxButton(this, wxID_OK, _L("Close"));
    close->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { EndModal(wxID_OK); });
    buttons->Add(close, 0, wxALL, FromDIP(4));
    root->Add(buttons, 0, wxEXPAND | wxALL, FromDIP(8));
    Bind(wxEVT_CLOSE_WINDOW, [this](wxCloseEvent &) { EndModal(wxID_OK); });
    SetSizerAndFit(root);
    wxGetApp().UpdateDlgDarkUI(this);
}

MixedNozzleWizardDialog::MixedNozzleWizardDialog(wxWindow *parent, MixedNozzleWizardDialogInput input)
    : DPIDialog(parent, wxID_ANY, _L("Mixed-nozzle setup"), wxDefaultPosition,
                wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER), m_input(std::move(input)),
      m_original_effective(m_input.effective_config)
{
    // Seed the ratio "before the fine height was last edited" with the draft's selection, so
    // updating cadences without editing the fine height still prefers the incoming ratio.
    m_previous_cadence_ratio = m_input.draft.chosen_cadence_ratio;
    // A project already set up has its own fine and coarse layer.
    if (m_input.configured_mode != MixedNozzleSlicingMode::Off && m_input.draft.chosen_coarse_height)
        m_existing_layers = std::make_pair(m_input.draft.chosen_fine_height, *m_input.draft.chosen_coarse_height);
    auto *root = new wxBoxSizer(wxVERTICAL);
    m_pages = new wxSimplebook(this, wxID_ANY);
    // A page's content never sets the dialog's width. Labels wrap to their page, so a page asking
    // for its widest line would hold the dialog at the widest width it ever wrapped to, and a wide
    // row on one page would push every page past the window's right edge, where their lines were
    // wrapped and then cut off.
    m_pages->SetMinSize(wxSize(FromDIP(320), -1));
    // Four steps and one More options page. Each step says where it is and what it asks.
    auto page = [this](const wxString &title, int step) {
        auto *panel = new wxScrolledWindow(m_pages, wxID_ANY);
        panel->SetScrollRate(0, FromDIP(12));
        auto *sizer = new wxBoxSizer(wxVERTICAL);
        panel->SetSizer(sizer);
        if (step > 0) {
            auto *where = new wxStaticText(panel, wxID_ANY, wxString::Format(_L("Step %d of 4"), step));
            sizer->Add(where, 0, wxLEFT | wxRIGHT | wxTOP, FromDIP(8));
        }
        auto *heading = new wxStaticText(panel, wxID_ANY, title);
        heading->SetFont(heading->GetFont().Bold().Larger());
        sizer->Add(heading, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(8));
        m_pages->AddPage(panel, title);
        // Rewrap to the page's current width whenever it is resized.
        panel->Bind(wxEVT_SIZE, [this, panel](wxSizeEvent &event) {
            if (rewrap_labels(panel))
                panel->FitInside();
            event.Skip();
        });
        return panel;
    };
    auto label = [this](wxWindow *parent, wxSizer *sizer, const wxString &text, int right_reserve = 0) {
        auto *control = new wxStaticText(parent, wxID_ANY, text);
        sizer->Add(control, 0, wxEXPAND | wxALL, FromDIP(8));
        if (right_reserve > 0)
            reserve_wrap(control, right_reserve);
        return control;
    };
    auto choice = [this](wxWindow *parent, wxSizer *sizer, const std::vector<wxString> &items, int selected) {
        auto *control = new wxChoice(parent, wxID_ANY);
        for (const auto &item : items) control->Append(item);
        if (selected >= 0 && selected < int(items.size())) control->SetSelection(selected);
        sizer->Add(control, 0, wxEXPAND | wxALL, FromDIP(8));
        return control;
    };

    // One wizard with named steps, the one place that writes scope, mode and tower decisions.
    auto card = [this, &label](wxWindow *parent, wxSizer *sizer, const wxString &title,
                               const wxString &description, bool first) {
        auto *panel = new wxPanel(parent, wxID_ANY);
        auto *inner = new wxBoxSizer(wxVERTICAL);
        auto *button = new wxRadioButton(panel, wxID_ANY, title, wxDefaultPosition, wxDefaultSize,
                                         first ? wxRB_GROUP : 0);
        inner->Add(button, 0, wxALL, FromDIP(8));
        label(panel, inner, description, FromDIP(4));
        panel->SetSizer(inner);
        sizer->Add(panel, 0, wxEXPAND | wxALL, FromDIP(4));
        return button;
    };

    auto *mode_page = page(_L("What prints fine"), 1);
    // The introduction is the lead line of step 1; "?" opens the explanation. Neither writes
    // anything.
    {
        auto *lead_row = new wxBoxSizer(wxHORIZONTAL);
        auto *lead = new wxStaticText(mode_page, wxID_ANY, from_u8(wizard_intro_lead_line()));
        lead_row->Add(lead, 1, wxALIGN_CENTER_VERTICAL);
        auto *about = new wxButton(mode_page, wxID_ANY, "?", wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
        reserve_wrap(lead, about->GetBestSize().x + FromDIP(8));
        about->SetToolTip(_L("About mixed-nozzle slicing"));
        about->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
            MixedNozzleIntroDialog intro(this);
            intro.ShowModal();
        });
        lead_row->Add(about, 0, wxLEFT | wxALIGN_CENTER_VERTICAL, FromDIP(8));
        mode_page->GetSizer()->Add(lead_row, 0, wxEXPAND | wxALL, FromDIP(8));
    }
    // The card text names the project's own nozzles.
    const WizardModeCardText card_text = wizard_mode_card_text(m_input.effective_config.nozzle_diameter.values);
    m_mode_feature = card(mode_page, mode_page->GetSizer(), _L("Feature Split"),
        from_u8(card_text.feature), true);
    m_mode_body = card(mode_page, mode_page->GetSizer(), _L("Body Split"),
        from_u8(card_text.body), false);
    // A part by its name and size, with its material on More options only. The object's name only
    // when several objects are listed.
    std::set<std::size_t> listed_objects;
    for (const auto &row : m_input.body_rows)
        listed_objects.insert(row.object_id.id);
    const auto part_row_text = [this, &listed_objects](std::size_t index, const std::string &material) {
        const auto &row = m_input.body_rows[index];
        const std::array<double, 3> size = index < m_input.body_sizes.size() ? m_input.body_sizes[index]
                                                                               : std::array<double, 3>{0., 0., 0.};
        return from_u8(wizard_part_row_text(material, row.object_name, row.volume_name,
            listed_objects.size() > 1, size[0], size[1], size[2]));
    };
    // The Body Split parts sit under their card. Each prints fine or coarse; its material follows
    // from the Materials step, and a part on a third material keeps its slot.
    {
        const std::vector<double> &pair = m_input.effective_config.nozzle_diameter.values;
        const auto nozzle = [&pair](bool coarse) {
            if (pair.size() != 2)
                return coarse ? _L("Coarse") : _L("Fine");
            const double diameter = coarse ? std::max(pair[0], pair[1]) : std::min(pair[0], pair[1]);
            return (coarse ? _L("Coarse") : _L("Fine")) + wxString::Format(" %g mm", diameter);
        };
        m_body_assignments = new wxPanel(mode_page, wxID_ANY);
        auto *body_sizer = new wxBoxSizer(wxVERTICAL);
        m_body_assignments->SetSizer(body_sizer);
        auto *grid = new wxFlexGridSizer(4, FromDIP(4), FromDIP(12));
        // Each part by its name and size. Materials are chosen on a later step.
        for (std::size_t index = 0; index < m_input.body_rows.size(); ++index) {
            const auto &row = m_input.body_rows[index];
            auto *name = new wxStaticText(m_body_assignments, wxID_ANY, part_row_text(index, std::string()));
            grid->Add(name, 0, wxALIGN_CENTER_VERTICAL);
            auto *fine = new wxRadioButton(m_body_assignments, wxID_ANY, nozzle(false), wxDefaultPosition,
                                           wxDefaultSize, wxRB_GROUP);
            auto *coarse = new wxRadioButton(m_body_assignments, wxID_ANY, nozzle(true));
            grid->Add(fine, 0, wxALIGN_CENTER_VERTICAL);
            grid->Add(coarse, 0, wxALIGN_CENTER_VERTICAL);
            auto *note = new wxStaticText(m_body_assignments, wxID_ANY, wxEmptyString);
            grid->Add(note, 0, wxALIGN_CENTER_VERTICAL);
            // Pointing at a part's row, or choosing its nozzle, selects that part in the 3D view.
            const std::size_t object_id = row.object_id.id;
            const std::size_t volume_id = row.volume_id.id;
            const auto highlight = [this, object_id, volume_id] {
                if (m_input.highlight_part)
                    m_input.highlight_part(object_id, volume_id);
            };
            name->Bind(wxEVT_ENTER_WINDOW, [highlight](wxMouseEvent &event) {
                highlight();
                event.Skip();
            });
            for (wxRadioButton *button : {fine, coarse})
                button->Bind(wxEVT_RADIOBUTTON, [this, highlight, index](wxCommandEvent &event) {
                    highlight();
                    // Choosing Fine or Coarse replaces the slot the part opened on; a slot picked in
                    // More options in this session stays.
                    if (index < m_body_slot_overrides.size() && m_body_slot_from_project[index]) {
                        m_body_slot_overrides[index]->SetSelection(0);
                        m_body_slot_from_project[index] = false;
                    }
                    refresh_body_roles_view();
                    refresh_navigation();
                    event.Skip();
                });
            m_body_roles.emplace_back(fine, coarse);
            m_body_role_notes.push_back(note);
        }
        body_sizer->Add(grid, 0, wxEXPAND | wxLEFT | wxRIGHT, FromDIP(24));
        auto *joining_row = new wxBoxSizer(wxHORIZONTAL);
        m_joining_summary = new wxStaticText(m_body_assignments, wxID_ANY, wxEmptyString);
        joining_row->Add(m_joining_summary, 1, wxALIGN_CENTER_VERTICAL);
        auto *change_joining = new wxButton(m_body_assignments, wxID_ANY, _L("Change joining..."),
                                            wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
        change_joining->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { open_more_options(true); });
        joining_row->Add(change_joining, 0, wxLEFT, FromDIP(8));
        reserve_wrap(m_joining_summary, change_joining->GetBestSize().x + FromDIP(8));
        body_sizer->Add(joining_row, 0, wxEXPAND | wxALL, FromDIP(8));
        // A single-part object is not a Body object yet. Say how to make it one.
        std::map<std::size_t, std::pair<std::string, std::size_t>> parts;
        std::set<std::size_t> painted;
        for (const auto &row : m_input.body_rows) {
            auto &entry = parts[row.object_id.id];
            entry.first = row.object_name;
            ++entry.second;
            if (!row.painted_logical_filaments.empty())
                painted.insert(row.object_id.id);
        }
        std::vector<std::string> single;
        for (const auto &[id, entry] : parts)
            if (entry.second == 1 && painted.count(id) == 0)
                single.push_back(entry.first);
        const std::string hint = wizard_single_part_hint(single);
        if (!hint.empty())
            label(m_body_assignments, body_sizer, from_u8(hint));
        mode_page->GetSizer()->Add(m_body_assignments, 0, wxEXPAND);
    }
    // A project that has been set up can go back to one nozzle here. The link takes the Off path,
    // which Review states and Apply writes as the mode alone.
    m_off_link = new wxButton(mode_page, wxID_ANY, _L("Print with one nozzle instead"), wxDefaultPosition,
                              wxDefaultSize, wxBU_EXACTFIT);
    m_off_link->Show(m_input.configured_mode != MixedNozzleSlicingMode::Off);
    mode_page->GetSizer()->Add(m_off_link, 0, wxALL, FromDIP(8));
    m_off_link->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { choose_one_nozzle(); });
    // Each card is its own panel, so wxRB_GROUP groups nothing here. Set every value on every
    // click.
    auto apply_mode = [this](MixedNozzleSlicingMode mode) {
        const WizardModeSelection selection = wizard_select_mode(mode);
        m_mode_feature->SetValue(selection.feature);
        m_mode_body->SetValue(selection.body);
        m_off_chosen = selection.off;
        m_input.draft.mode = mode;
    };
    apply_mode(m_input.draft.mode == MixedNozzleSlicingMode::BodySplit
        ? MixedNozzleSlicingMode::BodySplit : MixedNozzleSlicingMode::FeatureSplit);
    // The Materials page changes shape with the mode, so re-derive it on the click.
    auto choose_mode = [this, apply_mode](MixedNozzleSlicingMode mode) {
        apply_mode(mode);
        refresh_resolved_tools();
        refresh_navigation();
    };
    m_mode_feature->Bind(wxEVT_RADIOBUTTON, [choose_mode](wxCommandEvent &event) {
        choose_mode(MixedNozzleSlicingMode::FeatureSplit);
        event.Skip();
    });
    m_mode_body->Bind(wxEVT_RADIOBUTTON, [choose_mode](wxCommandEvent &event) {
        choose_mode(MixedNozzleSlicingMode::BodySplit);
        event.Skip();
    });

    auto *assign_page = page(_L("Materials"), 2);
    // One row per slot: a square in the slot's filament colour, as in the sidebar combo, then the
    // material name. The index is the slot.
    const std::vector<WizardMaterialRow> material_rows =
        wizard_material_rows(m_input.filament_labels, m_input.filament_colours);
    const int swatch = int(std::lround(2. * wxGetApp().em_unit()));
    auto material_picker = [this, &material_rows, swatch](wxWindow *parent, wxSizer *sizer, int selected) {
        auto *control = new ::ComboBox(parent, wxID_ANY, wxEmptyString, wxDefaultPosition,
                                       wxDefaultSize, 0, nullptr, wxCB_READONLY);
        for (const WizardMaterialRow &row : material_rows) {
            wxBitmap *square = get_extruder_color_icon(row.colour, std::to_string(row.slot + 1),
                                                       swatch, swatch);
            control->Append(from_u8(row.label), square == nullptr ? wxNullBitmap : *square);
        }
        if (selected >= 0 && std::size_t(selected) < material_rows.size())
            control->SetSelection(selected);
        sizer->Add(control, 0, wxEXPAND | wxALL, FromDIP(8));
        return control;
    };
    const auto resolved_line = [this, assign_page] {
        auto *line = new wxStaticText(assign_page, wxID_ANY, wxEmptyString);
        assign_page->GetSizer()->Add(line, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(8));
        return line;
    };
    label(assign_page, assign_page->GetSizer(), _L("Fine layers"));
    m_fine_filament = material_picker(assign_page, assign_page->GetSizer(),
        m_input.draft.fine_logical_filament ? int(*m_input.draft.fine_logical_filament) : wxNOT_FOUND);
    m_fine_resolved = resolved_line();
    label(assign_page, assign_page->GetSizer(), _L("Coarse layers"));
    m_coarse_filament = material_picker(assign_page, assign_page->GetSizer(),
        m_input.draft.coarse_logical_filament ? int(*m_input.draft.coarse_logical_filament) : wxNOT_FOUND);
    m_coarse_resolved = resolved_line();
    m_materials_note = label(assign_page, assign_page->GetSizer(), m_input.materials_note.empty()
        ? _L("Pre-selected from the materials detected on each nozzle.") : from_u8(m_input.materials_note));
    // Show the resolved nozzle, diameter and flow variant so the choice can be checked here.
    for (auto *picker : {m_fine_filament, m_coarse_filament})
        picker->Bind(wxEVT_COMBOBOX, [this](wxCommandEvent &) {
            // A material picked by hand is never replaced by the parts' own.
            m_materials_picked_by_hand = true;
            m_role_default_note.clear();
            refresh_derived_map();
            refresh_resolved_tools();
            refresh_navigation();
        });
    label(assign_page, assign_page->GetSizer(),
        _L("Your wall, shell and painting settings are kept."));

    // Fine and coarse layer are one decision, detail against time, so they share a page and the
    // coarse list follows the fine layer.
    auto *finish_page = page(_L("Detail and speed"), 3);
    // Which process preset every time on this page uses, and why.
    m_basis_line = label(finish_page, finish_page->GetSizer(), wxEmptyString);
    auto *fine_title = label(finish_page, finish_page->GetSizer(), _L("Fine layer"));
    fine_title->SetFont(fine_title->GetFont().Bold());
    // The list is the fine nozzle's own catalogue, so name that nozzle, its side and its range
    // first.
    m_fine_nozzle_line = label(finish_page, finish_page->GetSizer(), wxEmptyString);
    m_fine_height_choice = choice(finish_page, finish_page->GetSizer(), {}, wxNOT_FOUND);
    m_fine_height = new wxTextCtrl(finish_page, wxID_ANY,
        wxString::Format("%.4g", m_input.draft.chosen_fine_height));
    finish_page->GetSizer()->Add(m_fine_height, 0, wxEXPAND | wxALL, FromDIP(8));
    // The status line takes space only while it has something to say, and the help sits right under
    // the dropdown.
    m_fine_height_status = label(finish_page, finish_page->GetSizer(), wxEmptyString);
    m_fine_height_status->Hide();
    // What a thinner fine layer costs, set for the mode on the way in.
    m_fine_help = label(finish_page, finish_page->GetSizer(), from_u8(wizard_fine_layer_help(m_input.draft.mode)));
    if (wxSizerItem *item = finish_page->GetSizer()->GetItem(m_fine_help))
        item->SetFlag(wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM);
    // Selecting a stock height re-resolves the cadences immediately. "Custom..." reveals the text
    // box and uses the same validation.
    m_fine_height_choice->Bind(wxEVT_CHOICE, [this, finish_page](wxCommandEvent &) {
        const int selected = m_fine_height_choice->GetSelection();
        const bool custom = selected >= 0 && std::size_t(selected) >= m_fine_height_values.size();
        m_fine_height->Show(custom);
        if (!custom && selected >= 0)
            m_fine_height->ChangeValue(wxString::Format("%.4g", m_fine_height_values[std::size_t(selected)]));
        finish_page->Layout();
        // A pick belongs to the finish it was made at.
        m_cadence_picked_id.reset();
        refresh_candidates();
    });
    m_fine_height->Bind(wxEVT_TEXT, [this](wxCommandEvent &) {
        // Drop the coarse height and ratio resolved for the old fine height, but remember the ratio
        // so refresh_candidates() can re-select the same cadence at the new one.
        auto &draft = m_input.draft;
        if (draft.chosen_cadence_ratio) m_previous_cadence_ratio = draft.chosen_cadence_ratio;
        draft.chosen_coarse_height.reset();
        draft.chosen_cadence_ratio.reset();
        m_candidates.clear();
        m_cadence_page = {};
        rebuild_speed_rows(std::nullopt);
        // The rows being estimated, and any pick, belong to the old height.
        m_cadence_picked_id.reset();
        if (m_ranking) m_ranking->cancel();
        // The coarse list is on this page, so it follows the typed height straight away.
        refresh_candidates();
    });

    auto *cadence_page = finish_page;
    auto *coarse_title = label(cadence_page, cadence_page->GetSizer(), _L("Coarse layer"));
    coarse_title->SetFont(coarse_title->GetFont().Bold());
    // What N is, set for the mode and the coarse nozzle on the way in.
    m_coarse_help = label(cadence_page, cadence_page->GetSizer(),
        from_u8(wizard_coarse_layer_help(m_input.draft.mode, m_input.effective_config)));
    // Every coarse layer as a radio button with its time and a Fastest tag, rebuilt as times
    // arrive. A row picked by hand stays picked.
    m_speed_rows_panel = new wxPanel(cadence_page, wxID_ANY);
    m_speed_rows_panel->SetSizer(new wxBoxSizer(wxVERTICAL));
    cadence_page->GetSizer()->Add(m_speed_rows_panel, 0, wxEXPAND | wxLEFT | wxRIGHT, FromDIP(16));
    // A resize that changes whether the tags fit beside the rows lays the rows out again.
    cadence_page->Bind(wxEVT_SIZE, [this](wxSizeEvent &event) {
        event.Skip();
        if (!m_speed_buttons.empty() && m_speed_tags_beside != (m_speed_rows_needed <= speed_rows_width()))
            CallAfter([this] {
                rebuild_speed_rows(m_speed_selection >= 0 ? std::optional<std::size_t>(std::size_t(m_speed_selection))
                                                          : std::nullopt);
            });
    });
    m_speed_summary = label(cadence_page, cadence_page->GetSizer(), wxEmptyString);
    m_one_nozzle_link = new wxButton(cadence_page, wxID_ANY, _L("Print with one nozzle instead"),
                                     wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    m_one_nozzle_link->Hide();
    cadence_page->GetSizer()->Add(m_one_nozzle_link, 0, wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(8));
    m_one_nozzle_link->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { choose_one_nozzle(); });
    // The exclusions sentence is the list's tooltip and a line on More options. This line tracks
    // the times: working out, done, or why there are none.
    m_cadence_time = new wxStaticText(cadence_page, wxID_ANY, wxEmptyString);
    cadence_page->GetSizer()->Add(m_cadence_time, 0, wxEXPAND | wxALL, FromDIP(8));
    // A bar under the progress line, filled by rows done.
    m_times_gauge = new wxGauge(cadence_page, wxID_ANY, 1, wxDefaultPosition, FromDIP(wxSize(-1, 8)));
    cadence_page->GetSizer()->Add(m_times_gauge, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(8));
    m_times_gauge->Hide();
    // The times are for the preset as it is now. Shown once a row has a time.
    m_times_note = label(cadence_page, cadence_page->GetSizer(), wxEmptyString);
    m_times_note->Hide();

    auto *review_page = page(_L("Check and apply"), 4);
    // What happens, in a few labelled rows, then whether it is ready. Every value Apply writes is
    // in the closed pane below.
    m_review_header = label(review_page, review_page->GetSizer(), wxEmptyString);
    m_review_header->SetFont(m_review_header->GetFont().Bold());
    m_review_rows = new wxPanel(review_page, wxID_ANY);
    m_review_rows->SetSizer(new wxFlexGridSizer(2, FromDIP(4), FromDIP(16)));
    review_page->GetSizer()->Add(m_review_rows, 0, wxEXPAND | wxLEFT | wxRIGHT, FromDIP(16));
    m_review_also = label(review_page, review_page->GetSizer(), wxEmptyString);
    // Consent for a shared process sits next to its consequence, and only when needed.
    m_shared_consent = new wxCheckBox(review_page, wxID_ANY, wxEmptyString);
    m_shared_consent->SetValue(m_input.draft.scope.allow_shared_process_changes);
    m_shared_consent->Hide();
    review_page->GetSizer()->Add(m_shared_consent, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(8));
    m_shared_consent->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent &) {
        m_input.draft.scope.allow_shared_process_changes = m_shared_consent->GetValue();
        refresh_review();
        refresh_navigation();
    });
    m_ready_line = label(review_page, review_page->GetSizer(), wxEmptyString);
    m_fix_link = new wxButton(review_page, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize,
                              wxBU_EXACTFIT);
    m_fix_link->Hide();
    review_page->GetSizer()->Add(m_fix_link, 0, wxLEFT | wxRIGHT | wxBOTTOM, FromDIP(8));
    m_fix_link->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        // Back to the step that fixes the blocker; the draft keeps every choice.
        if (m_fix_target == WizardFixTarget::Materials)
            m_pages->SetSelection(std::size_t(kMaterialsPage));
        else if (m_fix_target == WizardFixTarget::Speed)
            m_pages->SetSelection(std::size_t(kSpeedPage));
        else if (m_fix_target == WizardFixTarget::ProcessSetting && !m_fix_option.empty()) {
            // Close with nothing written; the Plater opens the setting in the Process tab.
            m_jump_option = m_fix_option;
            EndModal(wxID_CANCEL);
            return;
        }
        refresh_navigation();
    });
    m_details_pane = new wxCollapsiblePane(review_page, wxID_ANY, _L("Every setting this changes"),
                                           wxDefaultPosition, wxDefaultSize, wxCP_DEFAULT_STYLE | wxCP_NO_TLW_RESIZE);
    review_page->GetSizer()->Add(m_details_pane, 1, wxEXPAND | wxALL, FromDIP(8));
    {
        wxWindow *pane = m_details_pane->GetPane();
        auto *pane_sizer = new wxBoxSizer(wxVERTICAL);
        m_review_text = new wxTextCtrl(pane, wxID_ANY, wxEmptyString, wxDefaultPosition,
            FromDIP(wxSize(-1, 220)), wxTE_MULTILINE | wxTE_READONLY);
        pane_sizer->Add(m_review_text, 1, wxEXPAND);
        pane->SetSizer(pane_sizer);
    }
    m_details_pane->Bind(wxEVT_COLLAPSIBLEPANE_CHANGED, [review_page](wxCollapsiblePaneEvent &) {
        review_page->FitInside();
        review_page->Layout();
    });
    // Every advanced control, reached from any step.
    auto *tower_page = page(_L("More options"), 0);
    // Defaults match a setup that never opens this page (wizard_more_options).
    const WizardMoreOptions more_defaults = wizard_more_options(m_input.draft, bool(m_input.review_binding),
                                                                m_input.project_scope_locked);
    m_input.draft.scope.kind = more_defaults.scope;
    label(tower_page, tower_page->GetSizer(), _L("Where this setup applies"));
    m_scope = choice(tower_page, tower_page->GetSizer(),
        {_L("This plate only"), from_u8(wizard_whole_project_label(m_input.draft.scope.process_affected_plate_ids.size()))},
        more_defaults.scope == WizardScopeKind::CurrentPlate ? 0 : 1);
    m_scope->Enable(!more_defaults.scope_locked);
    m_scope->Bind(wxEVT_CHOICE, [this](wxCommandEvent &event) {
        read_more_options();
        event.Skip();
    });
    // The tower options in WizardTowerIntent order, plus explicit Enabled/Disabled overrides.
    label(tower_page, tower_page->GetSizer(),
          _L("Prime tower. Automatic sets every tower value two nozzles need."));
    // Same order as WizardTowerIntent: the selection index is the intent.
    m_tower = choice(tower_page, tower_page->GetSizer(),
        {_L("Keep this project's tower settings"), _L("Set up automatically (recommended)"),
         _L("Keep this project's settings, tower always on"),
         _L("Keep this project's settings, tower off (setup will not slice with two nozzles)")},
        int(m_input.draft.tower_intent));
    m_tower_explanation = label(tower_page, tower_page->GetSizer(), wxEmptyString);
    auto refresh_tower_explanation = [this] {
        const bool automatic = m_tower->GetSelection() == int(WizardTowerIntent::Automatic);
        set_wrapped(m_tower_explanation, from_u8(automatic ? m_input.tower.automatic_explanation
                                                           : m_input.tower.keep_explanation));
        m_tower_explanation->GetParent()->Layout();
    };
    m_tower->Bind(wxEVT_CHOICE, [refresh_tower_explanation](wxCommandEvent &event) {
        refresh_tower_explanation();
        event.Skip();
    });
    refresh_tower_explanation();
    // The one way to stay on a custom process. The rows re-rank on it.
    m_keep_process = new wxCheckBox(tower_page, wxID_ANY, _L("Keep my current process preset instead"));
    m_keep_process->SetValue(m_input.draft.keep_current_process);
    tower_page->GetSizer()->Add(m_keep_process, 0, wxALL, FromDIP(8));
    m_keep_process->Bind(wxEVT_CHECKBOX, [this](wxCommandEvent &) {
        m_input.draft.keep_current_process = m_keep_process->GetValue();
        m_cadence_picked_id.reset();
        // Re-rank only once the materials are captured; before that there are no rows.
        if (m_input.draft.resolved_physical_roles.size() == 2)
            refresh_candidates();
    });
    // The switch converts material values kept from before per-nozzle material settings, so it
    // shows only for a project that has such values. Moving a stock material to its profile for the
    // other nozzle is part of every setup and step 4 lists it.
    m_rebind = new wxCheckBox(tower_page, wxID_ANY,
        _L("Update each material's settings for its nozzle (recommended)"));
    m_rebind->SetValue(more_defaults.update_material_settings);
    tower_page->GetSizer()->Add(m_rebind, 0, wxALL, FromDIP(8));
    auto *rebind_note = label(tower_page, tower_page->GetSizer(), from_u8(wizard_material_settings_note(
        m_input.draft.scope.process_affected_plate_ids.size())));
    m_rebind->Show(bool(m_input.review_binding));
    rebind_note->Show(bool(m_input.review_binding));
    // Read-only: the flow type is the project's own and setup never changes it.
    {
        std::vector<std::string> flows;
        for (const int type : m_input.effective_config.nozzle_volume_type.values)
            flows.push_back(get_nozzle_volume_type_string(NozzleVolumeType(type)));
        label(tower_page, tower_page->GetSizer(),
              from_u8(wizard_nozzle_flow_line(m_input.effective_config.nozzle_diameter.values, flows)));
    }
    // The coarse layers the list leaves out, and why.
    m_cadence_exclusions = label(tower_page, tower_page->GetSizer(), wxEmptyString);
    // Any Body Split part can be pinned to an exact material slot here.
    m_body_slots_panel = new wxPanel(tower_page, wxID_ANY);
    m_body_slots_panel->SetSizer(new wxBoxSizer(wxVERTICAL));
    if (!m_input.body_rows.empty())
        label(m_body_slots_panel, m_body_slots_panel->GetSizer(), _L("Body Split parts: exact material slot"));
    const auto refresh_roles_on_choice = [this](wxCommandEvent &event) {
        refresh_body_roles_view();
        event.Skip();
    };
    // The same part names as step 1.
    for (std::size_t index = 0; index < m_input.body_rows.size(); ++index) {
        const auto &row = m_input.body_rows[index];
        std::vector<wxString> items{_L("Set by Fine or Coarse")};
        for (const std::string &material : m_input.filament_labels)
            items.push_back(from_u8(material));
        const std::size_t own_slot = row.logical_filament >= 1 ? std::size_t(row.logical_filament - 1) : std::size_t(-1);
        label(m_body_slots_panel, m_body_slots_panel->GetSizer(), part_row_text(index,
            own_slot < m_input.filament_labels.size() ? m_input.filament_labels[own_slot] : std::string()));
        // A part already on a slot other than the fine and coarse materials reopens on that slot.
        const int opened_on = wizard_body_slot_choice(row.logical_filament, m_input.draft.fine_logical_filament,
            m_input.draft.coarse_logical_filament, m_input.filament_labels.size());
        auto *slot = choice(m_body_slots_panel, m_body_slots_panel->GetSizer(), items, opened_on);
        slot->Bind(wxEVT_CHOICE, [this, index, refresh_roles_on_choice](wxCommandEvent &event) {
            m_body_slot_from_project[index] = false;
            refresh_roles_on_choice(event);
        });
        m_body_slot_overrides.push_back(slot);
        m_body_slot_from_project.push_back(opened_on != 0);
    }
    tower_page->GetSizer()->Add(m_body_slots_panel, 0, wxEXPAND);
    m_joining_panel = new wxPanel(tower_page, wxID_ANY);
    m_joining_panel->SetSizer(new wxBoxSizer(wxVERTICAL));
    label(m_joining_panel, m_joining_panel->GetSizer(), _L("Joining"));
    label(m_joining_panel, m_joining_panel->GetSizer(),
        _L("Parts that touch across the two nozzles start with interlocking beams on, so they print "
           "as one bonded part. Moving joints are not detected automatically: if a part is meant to "
           "move, choose Off for its assembly. Beams do not bridge gaps or guarantee bond strength. "
           "Width, depth and orientation are kept, and the setting is shared by every copy of the object."));
    std::map<std::size_t, std::size_t> joining_parts;
    for (const auto &row : m_input.body_rows)
        ++joining_parts[row.object_id.id];
    std::set<std::size_t> joining_objects;
    for (const auto &row : m_input.body_rows) {
        // A single-part object has nothing to join to.
        if (joining_parts[row.object_id.id] < 2 && row.painted_logical_filaments.empty()) continue;
        if (!joining_objects.insert(row.object_id.id).second) continue;
        const std::size_t id = row.object_id.id;
        const auto &draft = m_input.draft;
        const int selection = draft.enable_interlocking_objects.count(id) ? 1 :
                              draft.disable_interlocking_objects.count(id) ? 2 :
                              draft.keep_joining_objects.count(id) ? 0 :
                              m_input.touching_objects.count(id) ? 1 : 0;
        label(m_joining_panel, m_joining_panel->GetSizer(), from_u8(row.object_name));
        auto *joining = choice(m_joining_panel, m_joining_panel->GetSizer(),
            {_L("Keep current joining settings"), _L("Interlocking beams"),
             _L("Off, for a part meant to move")},
            selection);
        m_body_joining.emplace_back(id, joining);
    }
    tower_page->GetSizer()->Add(m_joining_panel, 0, wxEXPAND);
    for (const auto &[object_id, joining] : m_body_joining)
        joining->Bind(wxEVT_CHOICE, refresh_roles_on_choice);

    root->Add(m_pages, 1, wxEXPAND | wxALL, FromDIP(12));
    // A page coming into view wraps its labels to the width it has now.
    m_pages->Bind(wxEVT_BOOKCTRL_PAGE_CHANGED, [this](wxBookCtrlEvent &event) {
        relayout_page(m_pages->GetCurrentPage());
        event.Skip();
    });
    m_status = new wxStaticText(this, wxID_ANY, wxEmptyString);
    root->Add(m_status, 0, wxEXPAND | wxLEFT | wxRIGHT, FromDIP(16));
    reserve_wrap(m_status, FromDIP(8));
    auto *buttons = new wxBoxSizer(wxHORIZONTAL);
    auto button = [this, buttons](const wxString &text) {
        auto *control = new wxButton(this, wxID_ANY, text);
        buttons->Add(control, 0, wxALL, FromDIP(4));
        return control;
    };
    // An exit on every page that writes nothing, so there is always a way back to slicing.
    button(_L("Cancel"))->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { EndModal(wxID_CANCEL); });
    // Every advanced control is on one page, reached from any step.
    m_more = button(_L("More options"));
    m_more->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { open_more_options(); });
    buttons->AddStretchSpacer();
    m_back = button(_L("Back"));
    m_next = button(_L("Next"));
    m_done = button(_L("Done"));
    m_done->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { close_more_options(); });
    m_apply = button(_L("Apply"));
    m_apply_slice = button(_L("Apply and slice"));
    m_back->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { navigate(-1); });
    m_next->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { navigate(1); });
    auto finish = [this](bool slice) {
        if (!refresh_review() || !m_review.can_apply) return;
        m_slice_after_apply = slice;
        EndModal(wxID_OK);
    };
    m_apply->Bind(wxEVT_BUTTON, [finish](wxCommandEvent &) { finish(false); });
    m_apply_slice->Bind(wxEVT_BUTTON, [finish](wxCommandEvent &) { finish(true); });
    root->Add(buttons, 0, wxEXPAND | wxALL, FromDIP(8));
    Bind(wxEVT_CLOSE_WINDOW, [this](wxCloseEvent &) { EndModal(wxID_CANCEL); });
    // The status line under the pages wraps to the dialog's width.
    Bind(wxEVT_SIZE, [this](wxSizeEvent &event) {
        rewrap_labels(this);
        event.Skip();
    });
    SetMinSize(FromDIP(wxSize(660, 520)));
    SetSizerAndFit(root);
    SetSize(FromDIP(wxSize(720, 640)));
    // Each cadence row is estimated by a full slice of what Apply would publish for it, one row at
    // a time on a worker thread, reported back on the main thread.
    if (m_input.compose_ranking_slice) {
        WizardRankingHooks hooks;
        hooks.compose = [this](const std::string &row) { return compose_ranking_row(row); };
        hooks.run = [this](std::uint64_t job, const std::string &row, MixedNozzleRankingSlice slice,
                           std::shared_ptr<MixedNozzleRankingCancel> cancel) {
            // One slice at a time: the one before has reported or been cancelled already.
            if (m_ranking_thread.joinable())
                m_ranking_thread.join();
            std::weak_ptr<MixedNozzleRankingController> controller = m_ranking;
            m_ranking_thread = std::thread([controller, job, row, slice = std::move(slice), cancel]() {
                const WizardEstimate estimate = ranking_estimate(mixed_nozzle_slice_time(
                    *slice.model, slice.config, slice.is_bbl_printer, slice.plate_origin, {}, *cancel));
                wxGetApp().CallAfter([controller, job, row, estimate] {
                    if (const auto live = controller.lock())
                        live->deliver(job, row, estimate);
                });
            });
        };
        hooks.real_slice_running = m_input.real_slice_running;
        hooks.now = ranking_clock_seconds;
        hooks.changed = [this] { refresh_ranking(); };
        m_ranking = std::make_shared<MixedNozzleRankingController>(std::move(hooks), m_input.ranking_cache);
        m_ranking_timer = new wxTimer(this);
        Bind(wxEVT_TIMER, [this](wxTimerEvent &) { if (m_ranking) m_ranking->tick(); },
             m_ranking_timer->GetId());
        m_ranking_timer->Start(200);
    }
    refresh_derived_map();
    refresh_resolved_tools();
    refresh_fine_heights();
    apply_default_body_roles();
    // A Body Split setup starts step 2 on the materials the parts already print with.
    if (m_input.draft.mode == MixedNozzleSlicingMode::BodySplit)
        default_materials_from_parts();
    // Opening further in only changes where the reader starts: earlier steps are still reachable
    // with Back.
    int start = std::clamp(int(m_input.start_page), kModePage, kMorePage);
    if (start == kMorePage) {
        m_more_return_page = kModePage;
    } else if (start >= kSpeedPage && m_input.draft.mode != MixedNozzleSlicingMode::Off) {
        // A later step reads the materials the earlier ones would have captured.
        if (capture_assignments()) {
            refresh_fine_heights();
            refresh_candidates();
        } else {
            start = kMaterialsPage;
        }
        if (start == kReviewPage && !refresh_review())
            start = kSpeedPage;
    }
    m_pages->SetSelection(std::size_t(start));
    refresh_navigation();
    clamp_to_display();
    wxGetApp().UpdateDlgDarkUI(this);
}

MixedNozzleWizardDialog::~MixedNozzleWizardDialog()
{
    stop_ranking();
    delete m_ranking_timer;
}

void MixedNozzleWizardDialog::EndModal(int ret_code)
{
    // Whatever the caller does next, Apply and slice included, starts with no ranking slice left.
    stop_ranking();
    DPIDialog::EndModal(ret_code);
}

void MixedNozzleWizardDialog::stop_ranking()
{
    if (m_ranking_timer != nullptr)
        m_ranking_timer->Stop();
    if (m_ranking)
        m_ranking->shutdown();
    if (m_ranking_thread.joinable())
        m_ranking_thread.join();
}

void MixedNozzleWizardDialog::on_dpi_changed(const wxRect &) { clamp_to_display(); }

void MixedNozzleWizardDialog::clamp_to_display()
{
    const int index = wxDisplay::GetFromWindow(this);
    if (index == wxNOT_FOUND) return;
    const wxRect available = wxDisplay(unsigned(index)).GetClientArea();
    const wxSize maximum(std::max(1, available.width - FromDIP(24)),
                         std::max(1, available.height - FromDIP(24)));
    SetMinSize(wxSize(std::min(FromDIP(660), maximum.x), std::min(FromDIP(520), maximum.y)));
    SetSize(wxSize(std::min(GetSize().x, maximum.x), std::min(GetSize().y, maximum.y)));
    Layout();
}

bool MixedNozzleWizardDialog::capture_assignments()
{
    const int fine = m_fine_filament->GetSelection(), coarse = m_coarse_filament->GetSelection();
    if (fine < 0 || coarse < 0 || fine == coarse) {
        set_wrapped(m_status, _L("Pick a different material for the fine and coarse layers."));
        return false;
    }
    auto &draft = m_input.draft;
    draft.fine_logical_filament = std::size_t(fine);
    draft.coarse_logical_filament = std::size_t(coarse);
    draft.approved_key_deltas.erase(std::remove_if(draft.approved_key_deltas.begin(),
        draft.approved_key_deltas.end(), [](const WizardKeyDelta &delta) { return delta.source == "Native binding"; }),
        draft.approved_key_deltas.end());
    draft.native_binding.reset();
    draft.scope.allow_project_binding_changes = m_rebind->GetValue();
    m_input.effective_config = m_original_effective;
    m_binding_notes.clear();
    std::vector<std::size_t> selected{std::size_t(fine), std::size_t(coarse)};
    if (draft.mode == MixedNozzleSlicingMode::BodySplit) {
        // The parts' own slots and any exact slot from More options get their material settings
        // reviewed too.
        for (const auto &row : m_input.body_rows)
            if (row.logical_filament >= 1)
                selected.push_back(std::size_t(row.logical_filament - 1));
        for (const wxChoice *slot : m_body_slot_overrides)
            if (slot->GetSelection() >= 1)
                selected.push_back(std::size_t(slot->GetSelection() - 1));
    }
    std::sort(selected.begin(), selected.end());
    selected.erase(std::unique(selected.begin(), selected.end()), selected.end());
    if (m_rebind->GetValue() && m_input.review_binding) {
        auto binding = m_input.review_binding(selected);
        if (!binding.diagnostic.empty()) {
            set_wrapped(m_status, from_u8(binding.diagnostic));
            return false;
        }
        draft.native_binding = std::move(binding.approval);
        m_input.effective_config = std::move(binding.effective_config);
        draft.approved_key_deltas.insert(draft.approved_key_deltas.end(), binding.deltas.begin(), binding.deltas.end());
        m_binding_notes = std::move(binding.notes);
    }
    // Restore the proposed mapping before describing any material, or the previous nozzle and flow
    // column would be reported.
    refresh_derived_map();
    for (std::size_t logical : selected) {
        const auto resolved = resolve_mixed_nozzle_tool(m_input.effective_config, logical,
            MixedNozzleResolveScope::PhysicalToolOnly);
        const auto &caps = m_input.effective_config.filament_max_volumetric_speed.values;
        const std::string material = logical < m_input.filament_labels.size() ? m_input.filament_labels[logical] :
            "Material " + std::to_string(logical + 1);
        std::optional<std::size_t> cap_index;
        if (caps.size() == m_input.filament_labels.size())
            cap_index = logical;
        else if (resolved && resolved.tool->variant_column && *resolved.tool->variant_column >= 0)
            cap_index = std::size_t(*resolved.tool->variant_column);
        std::optional<double> cap;
        if (cap_index && *cap_index < caps.size() && std::isfinite(caps[*cap_index]))
            cap = caps[*cap_index];
        // One flow review line per material.
        m_binding_notes.push_back(wizard_material_flow_line(material,
            resolved ? std::optional<std::size_t>(resolved.tool->physical_extruder) : std::nullopt,
            resolved ? resolved.tool->nozzle_diameter : 0.,
            resolved && resolved.tool->nozzle_volume_type
                ? get_nozzle_volume_type_string(*resolved.tool->nozzle_volume_type) : std::string(),
            cap));
    }
    draft.resolved_physical_roles.clear();
    for (const auto &[logical, role] : std::vector<std::pair<int, std::string>>{{fine, "fine"}, {coarse, "coarse"}}) {
        const auto resolved = resolve_mixed_nozzle_tool(m_input.effective_config, logical,
            MixedNozzleResolveScope::PhysicalToolOnly);
        if (!resolved) {
            // Route the engine code through the wizard's wording so the static-map diagnostic never
            // shows as a prerequisite to satisfy elsewhere.
            const wxString reason = resolved.diagnostic
                ? wizard_review_message(resolved.diagnostic->stable_code())
                : _L("This material has no nozzle of its own yet. Pick another one.");
            set_wrapped(m_status, wxString::Format(_L("Material %d: %s"), logical + 1, reason));
            return false;
        }
        draft.resolved_physical_roles.push_back({std::size_t(logical),
            resolved.tool->physical_extruder, resolved.tool->nozzle_diameter, role});
    }
    return true;
}

MixedNozzleSlicingMode MixedNozzleWizardDialog::selected_mode() const
{
    if (m_mode_feature == nullptr || m_mode_body == nullptr)
        return MixedNozzleSlicingMode::FeatureSplit;
    // Off is the "Print with one nozzle instead" link, not a third card.
    return wizard_selected_mode({m_mode_feature->GetValue() && !m_off_chosen,
                                 m_mode_body->GetValue() && !m_off_chosen, m_off_chosen});
}

void MixedNozzleWizardDialog::refresh_derived_map()
{
    // The plate map is the wizard's own to set. Derive it from the two pickers and compose it into
    // the effective config, so every page resolves against the map Apply will write.
    const int fine = m_fine_filament == nullptr ? wxNOT_FOUND : m_fine_filament->GetSelection();
    const int coarse = m_coarse_filament == nullptr ? wxNOT_FOUND : m_coarse_filament->GetSelection();
    std::optional<std::size_t> fine_slot;
    std::optional<std::size_t> coarse_slot;
    if (fine >= 0) fine_slot = std::size_t(fine);
    if (coarse >= 0 && coarse != fine) coarse_slot = std::size_t(coarse);
    const std::size_t count = std::max<std::size_t>(m_input.filament_labels.size(),
        m_input.effective_config.filament_map.values.size());
    auto &draft = m_input.draft;
    // Always derive from the plate's map as setup found it; deriving from the previous pick's map
    // could leave a slot on the wrong nozzle.
    draft.derived_filament_map = wizard_derived_filament_map(
        m_input.effective_config.nozzle_diameter.values,
        m_original_effective.filament_map.values, count, fine_slot, coarse_slot);
    draft.derived_map_mode = fmmManual;
    m_input.effective_config.filament_map_mode.value = fmmManual;
    m_input.effective_config.filament_map.values = draft.derived_filament_map;
}

void MixedNozzleWizardDialog::refresh_resolved_tools()
{
    // Read-only projection of the pickers against the effective config with the derived map. What
    // the page says is what the transaction resolves.
    const auto describe = [this](::ComboBox *picker, bool coarse_role) {
        const int logical = picker->GetSelection();
        if (logical < 0) return _L("Select a material to see its nozzle.");
        const auto resolution = wizard_material_resolution(m_input.effective_config,
            std::size_t(logical), coarse_role);
        return from_u8(resolution.text);
    };
    set_wrapped(m_fine_resolved, describe(m_fine_filament, false));
    set_wrapped(m_coarse_resolved, describe(m_coarse_filament, true));
    // The slot setup puts on an empty coarse nozzle is the coarse material picked now.
    const int coarse = m_coarse_filament->GetSelection();
    if (m_materials_note != nullptr && !m_input.materials_empty_nozzle.empty() && coarse >= 0)
        set_wrapped(m_materials_note, from_u8(wizard_empty_nozzle_note(m_input.materials_empty_nozzle,
                                                                       std::size_t(coarse))));
    relayout_page(m_pages->GetPage(kMaterialsPage));
}

void MixedNozzleWizardDialog::refresh_fine_heights()
{
    // The fine tool's own limits capped by its diameter, as in build_wizard_candidates(). Rebuilt
    // when the material roles change, since they decide which nozzle is fine.
    WizardHeightEnvelope envelope;
    if (m_input.draft.fine_logical_filament) {
        const auto fine = resolve_mixed_nozzle_tool(m_input.effective_config,
            *m_input.draft.fine_logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
        if (fine) {
            envelope.min_height = resolved_min_layer_height(m_input.effective_config,
                                                            fine.tool->physical_extruder);
            envelope.max_height = std::min(
                resolved_max_layer_height(m_input.effective_config, fine.tool->physical_extruder),
                fine.tool->nozzle_diameter);
        }
    }
    m_fine_height_values = legal_fine_heights(m_input.effective_config, m_input.catalogue, envelope);

    const double current = m_input.draft.chosen_fine_height;
    // Each row says what its height is for and which cadences it opens; the row in effect is
    // marked.
    const WizardFinishPage finish = wizard_finish_page(m_input.effective_config,
        m_fine_height_values, m_input.draft.fine_logical_filament, current);
    set_wrapped(m_fine_nozzle_line, from_u8(finish.nozzle_line));
    m_fine_height_choice->Clear();
    int selected = wxNOT_FOUND;
    for (std::size_t i = 0; i < m_fine_height_values.size(); ++i) {
        // "0.08 mm (finest)", "0.1 mm (now)". The height in effect is selected below.
        const wxString row = i < finish.rows.size() ? from_u8(finish.rows[i].label)
                                                    : wxString::Format("%.4g mm", m_fine_height_values[i]);
        m_fine_height_choice->Append(row);
        if (std::abs(m_fine_height_values[i] - current) < 1e-9) selected = int(i);
    }
    m_fine_height_choice->Append(_L("Custom…"));
    const bool custom = selected == wxNOT_FOUND;
    m_fine_height_choice->SetSelection(custom ? int(m_fine_height_values.size()) : selected);
    m_fine_height->Show(custom);
    if (custom && current > 0.) m_fine_height->ChangeValue(wxString::Format("%.4g", current));
}

bool MixedNozzleWizardDialog::selected_fine_height(double &height)
{
    const int selected = m_fine_height_choice->GetSelection();
    if (selected >= 0 && std::size_t(selected) < m_fine_height_values.size()) {
        height = m_fine_height_values[std::size_t(selected)];
        return true;
    }
    // "Custom..." keeps the free-text path and its validation.
    return m_fine_height->GetValue().ToDouble(&height) && std::isfinite(height) && height > 0.;
}

bool MixedNozzleWizardDialog::refresh_candidates()
{
    double height = 0.;
    if (!selected_fine_height(height)) {
        set_wrapped(m_status, _L("Enter a positive fine layer height in millimeters."));
        return false;
    }
    // Custom is checked against the nozzle range named above the list, with one plain reason.
    const std::string height_rejection = wizard_custom_fine_height_rejection(
        m_input.effective_config, m_input.draft.fine_logical_filament, height);
    set_wrapped(m_fine_height_status, from_u8(height_rejection));
    m_fine_height_status->Show(!height_rejection.empty());
    if (!height_rejection.empty()) {
        set_wrapped(m_status, from_u8(height_rejection));
        m_cadence_page = {};
        rebuild_speed_rows(std::nullopt);
        if (m_ranking) m_ranking->cancel();
        return false;
    }
    m_input.draft.chosen_fine_height = height;
    set_wrapped(m_fine_help, from_u8(wizard_fine_layer_help(m_input.draft.mode)));
    set_wrapped(m_coarse_help, from_u8(wizard_coarse_layer_help(m_input.draft.mode, m_input.effective_config)));
    // The fine layer can change the process tier, so the line is read again.
    if (m_input.process_basis && m_input.draft.mode != MixedNozzleSlicingMode::Off) {
        set_wrapped(m_basis_line, from_u8(wizard_process_basis_line(m_input.process_basis(m_input.draft),
            m_input.current_process_name, m_input.effective_config.nozzle_diameter.values)));
    }
    m_candidates = build_wizard_candidates(m_input.draft, m_input.effective_config, m_input.catalogue);
    // Only the chosen finish's usable ratios. The single-nozzle baseline is its own line, and a
    // ratio the coarse nozzle cannot print is explained under the control.
    m_cadence_page = wizard_cadence_page(m_candidates, height, m_input.effective_config);
    set_wrapped(m_cadence_exclusions, from_u8(m_cadence_page.exclusions));
    // Prefer the ratio in effect before this fine height was chosen, never the previous coarse
    // height, which may no longer be legal.
    const std::optional<std::size_t> preferred = preferred_wizard_candidate(
        m_candidates, m_previous_cadence_ratio, height);
    std::optional<std::size_t> preferred_row;
    for (std::size_t i = 0; i < m_cadence_page.rows.size(); ++i)
        if (preferred && m_cadence_page.rows[i].candidate_index == *preferred)
            preferred_row = i;
    // On a project already set up, its own coarse layer is marked "(now)" and held as an explicit
    // pick, so arriving times do not move the selection; the note still says when another is
    // faster.
    wizard_mark_current(m_cadence_page, height, m_existing_layers);
    if (const std::optional<std::size_t> existing = wizard_existing_row(m_cadence_page, height, m_existing_layers);
        existing && m_cadence_page.rows[*existing].candidate_index < m_candidates.size()) {
        if (!m_cadence_picked_id)
            m_cadence_picked_id = m_candidates[m_cadence_page.rows[*existing].candidate_index].stable_id;
        if (*m_cadence_picked_id == m_candidates[m_cadence_page.rows[*existing].candidate_index].stable_id)
            preferred_row = existing;
    }
    rebuild_speed_rows(preferred_row);
    set_wrapped(m_status, m_cadence_page.rows.empty()
        ? from_u8(wizard_no_coarse_layer_line(height, m_input.effective_config))
        : wxString{});
    // Estimate these rows. The page stays usable while that runs.
    request_ranking();
    return !m_cadence_page.rows.empty();
}

void MixedNozzleWizardDialog::request_ranking()
{
    if (!m_ranking) {
        refresh_ranking();
        return;
    }
    // The project's own row first, then thickest to thinnest, with one nozzle only third, so the
    // likely answer and the saving come in first.
    std::vector<std::string> rows = wizard_ranking_order(m_cadence_page, m_candidates,
        wizard_existing_row(m_cadence_page, m_input.draft.chosen_fine_height, m_existing_layers));
    if (rows.empty()) {
        m_ranking->cancel();
        return;
    }
    m_ranking->request(std::move(rows));
}

void MixedNozzleWizardDialog::refresh_ranking()
{
    if (!m_ranking) {
        refresh_cadence_time_line();
        return;
    }
    if (m_candidates.empty() || m_cadence_page.rows.empty()) {
        set_wrapped(m_cadence_time, wxEmptyString);
        m_times_note->Hide();
        m_times_gauge->Hide();
        return;
    }
    // The row selected now, by candidate, so a re-sort keeps it.
    std::optional<std::size_t> kept;
    const int selected = m_speed_selection;
    if (selected >= 0 && std::size_t(selected) < m_cadence_page.rows.size())
        kept = m_cadence_page.rows[std::size_t(selected)].candidate_index;
    for (WizardCandidate &candidate : m_candidates)
        candidate.estimate = m_ranking->estimate(candidate.stable_id);
    const std::optional<WizardEstimate> baseline = m_ranking->estimate(WIZARD_RANKING_BASELINE_ROW);
    m_cadence_page = wizard_cadence_page(m_candidates, m_input.draft.chosen_fine_height,
                                         m_input.effective_config, baseline ? *baseline : WizardEstimate{});
    wizard_mark_current(m_cadence_page, m_input.draft.chosen_fine_height, m_existing_layers);
    // The row being sliced now says so.
    const std::optional<std::string> &slicing = m_ranking->running_row();
    wizard_mark_slicing(m_cadence_page, m_candidates, slicing,
                        slicing && *slicing == WIZARD_RANKING_BASELINE_ROW);
    // A hand pick stays while it is on the page. Otherwise a ranked page selects its fastest row
    // (the first of the tied ones), and an unranked page keeps its selection.
    bool explicit_pick = false;
    if (m_cadence_picked_id)
        for (const WizardCadenceRow &row : m_cadence_page.rows)
            if (row.candidate_index < m_candidates.size() &&
                m_candidates[row.candidate_index].stable_id == *m_cadence_picked_id) {
                kept = row.candidate_index;
                explicit_pick = true;
            }
    const std::optional<std::size_t> choice = wizard_ranked_selection(m_cadence_page, kept, explicit_pick);
    rebuild_speed_rows(choice);
    refresh_cadence_time_line();
}

void MixedNozzleWizardDialog::refresh_cadence_time_line()
{
    const int selected = m_speed_selection;
    const std::optional<std::size_t> selected_row = selected >= 0 &&
        std::size_t(selected) < m_cadence_page.rows.size() ? std::optional<std::size_t>(std::size_t(selected))
                                                           : std::nullopt;
    const bool explicit_pick = selected_row && m_cadence_picked_id &&
        m_cadence_page.rows[*selected_row].candidate_index < m_candidates.size() &&
        m_candidates[m_cadence_page.rows[*selected_row].candidate_index].stable_id == *m_cadence_picked_id;
    // One nozzle only and what the fastest choice saves, or that switching saves nothing, then the
    // progress line.
    set_wrapped(m_speed_summary, from_u8(wizard_speed_summary_line(m_cadence_page, m_input.draft.chosen_fine_height,
                                                                selected_row, explicit_pick)));
    m_one_nozzle_link->Show(wizard_switching_saves_no_time(m_cadence_page));
    set_wrapped(m_cadence_time, from_u8(wizard_speed_progress_line(m_cadence_page, bool(m_ranking))));
    const WizardSpeedProgress progress = wizard_speed_progress(m_cadence_page);
    m_times_gauge->SetRange(int(std::max<std::size_t>(1, progress.total)));
    m_times_gauge->SetValue(int(progress.done));
    m_times_gauge->Show(progress.working && bool(m_ranking));
    const std::string times_note = wizard_speed_times_note(m_cadence_page);
    set_wrapped(m_times_note, from_u8(times_note));
    m_times_note->Show(!times_note.empty());
    refresh_speed_next();
    relayout_page(m_pages->GetPage(kSpeedPage));
}

void MixedNozzleWizardDialog::rebuild_speed_rows(std::optional<std::size_t> select)
{
    if (m_speed_rows_panel == nullptr)
        return;
    m_speed_rows_panel->DestroyChildren();
    m_speed_rows_panel->GetSizer()->Clear();
    m_speed_buttons.clear();
    const MixedNozzleSlicingMode mode = m_input.draft.mode;
    std::vector<wxSizer *> tag_cells;
    int widest = 0;
    for (std::size_t index = 0; index < m_cadence_page.rows.size(); ++index) {
        auto *button = new wxRadioButton(m_speed_rows_panel, wxID_ANY,
            from_u8(wizard_speed_row_label(m_cadence_page.rows[index], mode)), wxDefaultPosition, wxDefaultSize,
            index == 0 ? wxRB_GROUP : 0);
        button->Bind(wxEVT_RADIOBUTTON, [this, index](wxCommandEvent &event) {
            m_speed_selection = int(index);
            // A row picked by hand stays picked when the estimates arrive.
            if (const WizardCandidate *picked = selected_candidate())
                m_cadence_picked_id = picked->stable_id;
            refresh_cadence_time_line();
            event.Skip();
        });
        widest = std::max(widest, button->GetBestSize().x);
        const std::string tag_text = wizard_speed_row_tag(m_cadence_page, index);
        auto *tag_cell = new wxBoxSizer(wxHORIZONTAL);
#if wxUSE_ACTIVITYINDICATOR
        // A small native spinner beside the row being sliced now.
        if (m_cadence_page.rows[index].slicing) {
            auto *spinner = new wxActivityIndicator(m_speed_rows_panel, wxID_ANY, wxDefaultPosition,
                                                    FromDIP(wxSize(16, 16)));
            spinner->Start();
            tag_cell->Add(spinner, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, FromDIP(4));
        }
#endif
        if (!tag_text.empty()) {
            auto *tag = new wxStaticText(m_speed_rows_panel, wxID_ANY, from_u8(tag_text));
            tag->SetFont(tag->GetFont().Bold());
            tag_cell->Add(tag, 0, wxALIGN_CENTER_VERTICAL);
        }
        tag_cells.push_back(tag_cell);
        m_speed_buttons.push_back(button);
    }
    // The tags line up in a column beside the rows, and go under their row when the page is too
    // narrow for both.
    int widest_tag = 0;
    for (wxSizer *cell : tag_cells)
        widest_tag = std::max(widest_tag, cell->GetMinSize().x);
    m_speed_rows_needed = widest + FromDIP(16) + widest_tag;
    m_speed_tags_beside = m_speed_rows_needed <= speed_rows_width();
    for (std::size_t index = 0; index < m_speed_buttons.size(); ++index) {
        auto *line = new wxBoxSizer(m_speed_tags_beside ? wxHORIZONTAL : wxVERTICAL);
        if (m_speed_tags_beside)
            m_speed_buttons[index]->SetMinSize(wxSize(widest, -1));
        line->Add(m_speed_buttons[index], 0, m_speed_tags_beside ? wxALIGN_CENTER_VERTICAL : 0);
        if (tag_cells[index]->IsEmpty())
            delete tag_cells[index];
        else
            line->Add(tag_cells[index], 0, m_speed_tags_beside ? wxALIGN_CENTER_VERTICAL | wxLEFT : wxLEFT,
                      FromDIP(m_speed_tags_beside ? 16 : 24));
        m_speed_rows_panel->GetSizer()->Add(line, 0, wxBOTTOM, FromDIP(4));
    }
    m_speed_selection = m_speed_buttons.empty() ? -1
        : select && *select < m_speed_buttons.size() ? int(*select) : 0;
    if (m_speed_selection >= 0)
        m_speed_buttons[std::size_t(m_speed_selection)]->SetValue(true);
    // The limit sentence is the list's tooltip, not a line on the page.
    m_speed_rows_panel->SetToolTip(from_u8(m_cadence_page.exclusions));
    m_speed_rows_panel->Layout();
    relayout_page(m_pages->GetPage(kSpeedPage));
    wxGetApp().UpdateDlgDarkUI(this);
}

int MixedNozzleWizardDialog::speed_rows_width()
{
    // The page less its scroll bar and the rows' own margins, as the labels wrap to it.
    wxWindow *page = m_pages->GetPage(kSpeedPage);
    const int width = page->GetSize().x - wxSystemSettings::GetMetric(wxSYS_VSCROLL_X, page) - FromDIP(32);
    // Before the first layout a page has no width of its own yet.
    return width < FromDIP(200) ? FromDIP(580) : width;
}

void MixedNozzleWizardDialog::choose_one_nozzle()
{
    // When switching saves nothing, offer the way to one nozzle here, through the Off path.
    const WizardModeSelection selection = wizard_select_mode(MixedNozzleSlicingMode::Off);
    m_mode_feature->SetValue(selection.feature);
    m_mode_body->SetValue(selection.body);
    m_off_chosen = selection.off;
    m_input.draft.mode = MixedNozzleSlicingMode::Off;
    if (!refresh_review())
        return;
    m_pages->SetSelection(std::size_t(kReviewPage));
    refresh_navigation();
}

MixedNozzleRankingSlice MixedNozzleWizardDialog::compose_ranking_row(const std::string &row_id)
{
    MixedNozzleRankingSlice none;
    const bool baseline = row_id == WIZARD_RANKING_BASELINE_ROW;
    // The baseline is made from the lowest-ratio row: the same process, all on the fine nozzle.
    const WizardCandidate *candidate = nullptr;
    for (const WizardCadenceRow &row : m_cadence_page.rows) {
        if (row.candidate_index >= m_candidates.size())
            continue;
        const WizardCandidate &on_page = m_candidates[row.candidate_index];
        if (baseline ? (candidate == nullptr || row.ratio < candidate->ratio.value_or(0))
                     : on_page.stable_id == row_id)
            candidate = &on_page;
    }
    if (candidate == nullptr || !m_input.compose_ranking_slice) {
        none.diagnostic = "this choice is no longer on the page";
        return none;
    }
    std::vector<WizardBodyAssignment> body;
    if (m_input.draft.mode == MixedNozzleSlicingMode::BodySplit && !body_assignments(body)) {
        none.diagnostic = "every part needs a material on one of the two nozzles";
        return none;
    }
    MixedNozzleRankingSlice slice = m_input.compose_ranking_slice(m_input.draft, *candidate, body);
    if (baseline && slice.model && slice.diagnostic.empty()) {
        if (!m_input.draft.fine_logical_filament) {
            none.diagnostic = "choose the fine material first";
            return none;
        }
        mixed_nozzle_single_nozzle_baseline(*slice.model, slice.config,
                                            *m_input.draft.fine_logical_filament, candidate->fine_height);
        slice.key = mixed_nozzle_ranking_key(slice.plate_id, *slice.model, slice.config);
    }
    return slice;
}

std::vector<WizardBodyRoleRow> MixedNozzleWizardDialog::body_role_rows() const
{
    std::vector<WizardBodyRoleRow> rows;
    for (std::size_t index = 0; index < m_input.body_rows.size(); ++index) {
        const auto &row = m_input.body_rows[index];
        WizardBodyRoleRow role_row;
        role_row.object_id = row.object_id.id;
        role_row.volume_id = row.volume_id.id;
        role_row.current_slot = row.logical_filament - 1;
        if (role_row.current_slot >= 0) {
            const auto resolved = resolve_mixed_nozzle_tool(m_input.effective_config,
                std::size_t(role_row.current_slot), MixedNozzleResolveScope::PhysicalToolOnly);
            if (resolved)
                role_row.current_physical = resolved.tool->physical_extruder;
        }
        role_row.volume = index < m_input.body_volumes.size() ? m_input.body_volumes[index] : 0.;
        role_row.painted = !row.painted_logical_filaments.empty();
        rows.push_back(role_row);
    }
    return rows;
}

std::vector<WizardBodyRole> MixedNozzleWizardDialog::body_roles() const
{
    std::vector<WizardBodyRole> roles;
    for (const auto &[fine, coarse] : m_body_roles)
        roles.push_back(coarse->GetValue() ? WizardBodyRole::Coarse : WizardBodyRole::Fine);
    return roles;
}

void MixedNozzleWizardDialog::apply_default_body_roles()
{
    // The fine nozzle is the one the fine material prints on, else the smaller one.
    std::size_t fine_physical = 0;
    const std::vector<double> &pair = m_input.effective_config.nozzle_diameter.values;
    if (pair.size() == 2 && pair[1] < pair[0])
        fine_physical = 1;
    if (m_input.draft.fine_logical_filament)
        if (const auto fine = resolve_mixed_nozzle_tool(m_input.effective_config,
                *m_input.draft.fine_logical_filament, MixedNozzleResolveScope::PhysicalToolOnly))
            fine_physical = fine.tool->physical_extruder;
    const std::vector<WizardBodyRole> roles = wizard_default_body_roles(body_role_rows(), fine_physical);
    for (std::size_t index = 0; index < roles.size() && index < m_body_roles.size(); ++index) {
        m_body_roles[index].first->SetValue(roles[index] == WizardBodyRole::Fine);
        m_body_roles[index].second->SetValue(roles[index] == WizardBodyRole::Coarse);
    }
    refresh_body_roles_view();
}

void MixedNozzleWizardDialog::refresh_body_roles_view()
{
    // A part that keeps its own third material says which, beside its Fine or Coarse choice.
    std::vector<WizardBodyAssignment> body;
    const bool resolved = body_assignments(body);
    for (std::size_t index = 0; index < m_body_role_notes.size(); ++index) {
        wxString note;
        if (resolved && index < body.size()) {
            const std::size_t slot = body[index].logical_filament;
            const bool role_slot = (m_input.draft.fine_logical_filament && slot == *m_input.draft.fine_logical_filament) ||
                                   (m_input.draft.coarse_logical_filament && slot == *m_input.draft.coarse_logical_filament);
            if (!role_slot && slot < m_input.filament_labels.size())
                note = "(" + from_u8(m_input.filament_labels[slot]) + ")";
        }
        m_body_role_notes[index]->SetLabel(note);
    }
    std::size_t joined = 0;
    for (const auto &[object_id, choice] : m_body_joining)
        joined += choice->GetSelection() == 1 ? 1 : 0;
    if (m_joining_summary != nullptr)
        set_wrapped(m_joining_summary, from_u8(wizard_joining_summary(joined)));
    if (m_body_assignments != nullptr)
        m_body_assignments->Layout();
}

bool MixedNozzleWizardDialog::body_assignments(std::vector<WizardBodyAssignment> &body) const
{
    body.clear();
    const auto &draft = m_input.draft;
    if (draft.resolved_physical_roles.size() != 2 || !draft.fine_logical_filament || !draft.coarse_logical_filament)
        return false;
    std::vector<std::optional<WizardBodySlotOverride>> overrides;
    for (const wxChoice *slot : m_body_slot_overrides) {
        const int selection = slot->GetSelection();
        if (selection < 1) {
            overrides.emplace_back();
            continue;
        }
        const auto resolved = resolve_mixed_nozzle_tool(m_input.effective_config, std::size_t(selection - 1),
                                                        MixedNozzleResolveScope::PhysicalToolOnly);
        if (!resolved)
            return false;
        overrides.push_back(WizardBodySlotOverride{std::size_t(selection - 1), resolved.tool->physical_extruder});
    }
    body = wizard_body_roles_to_slots(body_role_rows(), body_roles(), *draft.fine_logical_filament,
        *draft.coarse_logical_filament, draft.resolved_physical_roles[0].physical_extruder,
        draft.resolved_physical_roles[1].physical_extruder, overrides);
    return true;
}

const WizardCandidate *MixedNozzleWizardDialog::selected_candidate() const
{
    const int selected = m_speed_selection;
    if (selected < 0 || std::size_t(selected) >= m_cadence_page.rows.size())
        return nullptr;
    const std::size_t index = m_cadence_page.rows[std::size_t(selected)].candidate_index;
    return index < m_candidates.size() ? &m_candidates[index] : nullptr;
}

bool MixedNozzleWizardDialog::refresh_review()
{
    // Off is a mode change and nothing else: no candidate, cadence, staged edits or tower decision.
    if (m_input.draft.mode == MixedNozzleSlicingMode::Off) {
        auto &off_draft = m_input.draft;
        off_draft.selected_candidate_kind.reset();
        off_draft.selected_source_preset_id.reset();
        off_draft.chosen_coarse_height.reset();
        off_draft.chosen_cadence_ratio.reset();
        off_draft.staged_body_edits.clear();
        off_draft.enable_interlocking_objects.clear();
        off_draft.disable_interlocking_objects.clear();
        off_draft.keep_joining_objects.clear();
        off_draft.approved_key_deltas.clear();
        off_draft.derived_filament_map.clear();
        off_draft.derived_map_mode = fmmDefault;
        off_draft.tower_intent = WizardTowerIntent::Preserve;
        off_draft.take_preset_first_layer = false;
        m_review = WizardReview{};
        m_review.can_apply = true;
        m_review.affected_plate_ids.push_back(off_draft.scope.current_plate_id);
        WizardReviewSummaryInput off_input;
        off_input.mode = MixedNozzleSlicingMode::Off;
        if (m_input.review_native) {
            const auto native = m_input.review_native(off_draft, m_review);
            off_input.notes = native.notes;
            off_input.engine_refusal = native.engine_refusal;
            off_input.engine_refusal_key = native.engine_refusal_key;
            if (native.engine_refusal.empty())
                off_input.setup_diagnostic = native.diagnostic;
            if (!native.diagnostic.empty())
                m_review.can_apply = false;
        }
        m_shared_consent->Hide();
        show_review_summary(wizard_review_summary(off_input));
        set_wrapped(m_status, wxString{});
        return true;
    }
    const WizardCandidate *chosen = selected_candidate();
    if (chosen == nullptr) {
        set_wrapped(m_status, _L("Pick a coarse layer first."));
        return false;
    }
    auto &draft = m_input.draft;
    const WizardCandidate &candidate = *chosen;
    std::vector<WizardBodyAssignment> body;
    if (draft.mode == MixedNozzleSlicingMode::BodySplit && !body_assignments(body)) {
        set_wrapped(m_status, _L("Give every part a material that sits on one of the two nozzles."));
        return false;
    }
    // The row's kind, catalogue provenance, heights and body edits: the same draft the ranking
    // sliced for this row.
    draft = wizard_candidate_draft(draft, candidate, body);
    draft.enable_interlocking_objects.clear();
    draft.disable_interlocking_objects.clear();
    draft.keep_joining_objects.clear();
    if (draft.mode == MixedNozzleSlicingMode::BodySplit) {
        // Every assembly shown gets an explicit choice, so Apply does what this page shows.
        for (const auto &[object_id, choice] : m_body_joining) {
            const int selection = choice->GetSelection();
            (selection == 1 ? draft.enable_interlocking_objects
                            : selection == 2 ? draft.disable_interlocking_objects
                                             : draft.keep_joining_objects).insert(object_id);
        }
    }
    // What the slice will start with, and the preset's own answer next to it.
    const auto *first_layer_speeds = m_input.effective_config.option<ConfigOptionFloatsNullable>(
        "initial_layer_speed");
    const WizardFirstLayer first_layer = wizard_first_layer(
        m_input.effective_config.initial_layer_print_height.value,
        first_layer_speeds == nullptr || first_layer_speeds->values.empty()
            ? 0. : first_layer_speeds->values.front(),
        m_input.preset_first_layer_height, m_input.preset_first_layer_speed);
    // Apply always takes the preset's first layer and the review line says so.
    draft.scope.allow_shared_process_changes = m_shared_consent->GetValue();
    m_review = build_wizard_review(draft, candidate);
    // The consent box is shown only when the heights reach a process other plates share.
    m_shared_consent->SetLabel(from_u8(wizard_shared_consent_label(m_review.shared_plate_count)));
    m_shared_consent->Show(m_review.needs_shared_consent);
    m_pages->GetPage(kReviewPage)->Layout();
    m_fine_height->ChangeValue(wxString::Format("%.4g", candidate.fine_height));
    refresh_fine_heights();

    // The summary reads the review, the prepared Apply's notes and the tower ledger, once each.
    WizardReviewSummaryInput summary;
    summary.mode = draft.mode;
    summary.nozzle_diameters = m_input.effective_config.nozzle_diameter.values;
    for (const WizardPhysicalRole &role : draft.resolved_physical_roles)
        (role.role == "fine" ? summary.fine_physical : summary.coarse_physical) = role.physical_extruder;
    summary.fine_height = candidate.fine_height;
    summary.coarse_height = candidate.coarse_height;
    const auto material_label = [this](std::optional<std::size_t> slot) {
        return slot && *slot < m_input.filament_labels.size() ? m_input.filament_labels[*slot] : std::string();
    };
    summary.fine_material = material_label(draft.fine_logical_filament);
    summary.coarse_material = material_label(draft.coarse_logical_filament);
    if (draft.mode == MixedNozzleSlicingMode::BodySplit) {
        for (const WizardBodyAssignment &assignment : body)
            for (const auto &row : m_input.body_rows)
                if (row.object_id.id == assignment.object_id && row.volume_id.id == assignment.volume_id)
                    (assignment.coarse ? summary.coarse_parts : summary.fine_parts).push_back(row.volume_name);
        for (const auto &[object_id, choice] : m_body_joining)
            if (choice->GetSelection() == 1)
                for (const auto &row : m_input.body_rows)
                    if (row.object_id.id == object_id) {
                        summary.joined_assemblies.push_back(row.object_name);
                        break;
                    }
    }
    summary.estimate = candidate.estimate;
    if (m_ranking)
        summary.baseline = m_ranking->estimate(WIZARD_RANKING_BASELINE_ROW);
    // The supports as the project has them; setup keeps them. The prepared process below, when it
    // is read, has the final word.
    const std::vector<std::string> &filament_types = m_input.effective_config.filament_type.values;
    const auto filament_type = [&filament_types](std::optional<std::size_t> slot) {
        return slot && *slot < filament_types.size() ? filament_types[*slot] : std::string();
    };
    const auto support_side = [this, &material_label, &filament_type](int slot) {
        WizardSupportSide side;
        side.slot = slot;
        if (slot > 0) {
            const std::size_t logical = std::size_t(slot - 1);
            side.material = material_label(logical);
            side.type = filament_type(logical);
            if (const auto resolved = resolve_mixed_nozzle_tool(m_input.effective_config, logical,
                                                                MixedNozzleResolveScope::PhysicalToolOnly))
                side.physical = resolved.tool->physical_extruder;
        }
        return side;
    };
    const auto set_supports = [&summary, &support_side](bool on, int base, int interface_slot, int top_layers) {
        summary.supports = on;
        summary.support_base = support_side(base);
        summary.support_interface = support_side(interface_slot);
        summary.support_interface_top_layers = top_layers;
    };
    set_supports(m_input.effective_config.enable_support.value, m_input.effective_config.support_filament.value,
                 m_input.effective_config.support_interface_filament.value,
                 m_input.effective_config.support_interface_top_layers.value);
    summary.fine_material_type = filament_type(draft.fine_logical_filament);
    summary.tower_intent = draft.tower_intent;
    summary.changes = m_review.entries;
    summary.material_lines = m_binding_notes;
    summary.map_line = wizard_filament_map_review_line(draft.derived_filament_map,
        draft.fine_logical_filament, draft.coarse_logical_filament);
    summary.first_layer = first_layer;
    for (const std::string &error : m_review.errors)
        summary.blockers.push_back(error);
    for (const std::string &warning : m_review.warnings)
        summary.blockers.push_back(warning);
    if (m_review.can_apply && m_input.review_native) {
        const auto native = m_input.review_native(draft, m_review);
        summary.notes = native.notes;
        summary.tower_lines = native.tower_lines;
        summary.process_before = native.process_before;
        summary.process_after = native.process_after;
        summary.also_changed = native.also_changed;
        if (native.supports_read)
            set_supports(native.supports, native.support_filament, native.support_interface_filament,
                         native.support_interface_top_layers);
        // The first layer the prepared process prints, which is the preset's.
        if (native.first_layer_height > 0.)
            summary.first_layer = wizard_first_layer(first_layer.height, first_layer.speed,
                                                     native.first_layer_height, native.first_layer_speed);
        summary.engine_refusal = native.engine_refusal;
        summary.engine_refusal_key = native.engine_refusal_key;
        if (native.engine_refusal.empty())
            summary.setup_diagnostic = native.diagnostic;
        if (!native.diagnostic.empty())
            m_review.can_apply = false;
    }
    // A part whose material setup would change is said plainly, first.
    if (draft.mode == MixedNozzleSlicingMode::BodySplit) {
        std::vector<std::string> names;
        for (const auto &row : m_input.body_rows)
            names.push_back(row.volume_name);
        const std::vector<std::string> changes =
            wizard_body_material_changes(names, body_role_rows(), body, m_input.filament_labels);
        summary.also_changed.insert(summary.also_changed.begin(), changes.begin(), changes.end());
        // Why a part that shared its slot now prints with another one.
        if (!m_materials_picked_by_hand && !m_role_default_note.empty())
            summary.also_changed.insert(summary.also_changed.begin(), m_role_default_note);
    }
    show_review_summary(wizard_review_summary(summary));
    set_wrapped(m_status, wxString{});
    return true;
}

void MixedNozzleWizardDialog::default_materials_from_parts()
{
    const auto selection = [](const ::ComboBox *picker) {
        const int selected = picker->GetSelection();
        return selected >= 0 ? std::optional<std::size_t>(std::size_t(selected)) : std::nullopt;
    };
    // Both pickers come from the parts; one the parts leave without a slot is cleared. The plate's
    // own map says which nozzle a slot is on.
    const WizardPickerDefaults defaults = wizard_body_picker_defaults(body_role_rows(), body_roles(),
        m_original_effective.nozzle_diameter.values,
        m_original_effective.filament_map.values, m_original_effective.filament_type.values,
        m_input.filament_labels, selection(m_fine_filament), selection(m_coarse_filament));
    m_role_default_note = defaults.note;
    bool changed = false;
    const auto pick = [&changed](::ComboBox *picker, const std::optional<std::size_t> &slot) {
        const int target = slot && int(*slot) < int(picker->GetCount()) ? int(*slot) : wxNOT_FOUND;
        if (picker->GetSelection() == target)
            return;
        picker->SetSelection(target);
        changed = true;
    };
    pick(m_fine_filament, defaults.fine);
    pick(m_coarse_filament, defaults.coarse);
    if (!changed)
        return;
    refresh_derived_map();
    refresh_resolved_tools();
    refresh_body_roles_view();
}

void MixedNozzleWizardDialog::show_review_summary(const WizardReviewSummary &summary)
{
    wxWindow *page = m_pages->GetPage(kReviewPage);
    set_wrapped(m_review_header, from_u8(summary.header));
    m_review_rows->DestroyChildren();
    wxSizer *rows = m_review_rows->GetSizer();
    rows->Clear();
    for (const WizardSummaryRow &row : summary.rows) {
        rows->Add(new wxStaticText(m_review_rows, wxID_ANY, from_u8(row.label)), 0, wxALIGN_TOP);
        auto *value = new wxStaticText(m_review_rows, wxID_ANY, from_u8(row.value));
        rows->Add(value, 0, wxALIGN_TOP);
        // The rows sit 16 in from each side, 8 more than a plain label.
        reserve_wrap(value, FromDIP(8));
    }
    wxString also;
    for (const std::string &line : summary.also_changed)
        also += (also.empty() ? _L("Also changed:") + " " : wxString("; ")) + from_u8(line);
    set_wrapped(m_review_also, also);
    m_review_also->Show(!also.empty());
    set_wrapped(m_ready_line, from_u8(summary.ready_line));
    m_fix_target = summary.target;
    m_fix_option = summary.option_key;
    // A Process setting link needs a setting to open.
    const bool link = !summary.target_label.empty() &&
                      (summary.target != WizardFixTarget::ProcessSetting || !summary.option_key.empty());
    m_fix_link->SetLabel(from_u8(summary.target_label));
    m_fix_link->Show(link);
    m_details_pane->SetLabel(wxString::Format(_L("Every setting this changes (%zu)"), summary.details.size()));
    wxString details;
    for (const std::string &line : summary.details)
        details += (details.empty() ? wxString{} : wxString("\n")) + from_u8(line);
    if (m_input.draft.mode != MixedNozzleSlicingMode::Off && !summary.details.empty())
        details += "\n\n" + _L("Automatic sets every tower value listed above. A value you change yourself "
            "afterwards stays as you set it. The brim is set when slicing, because only then is the tower "
            "height known: 3 mm under 20 mm tall, 5 mm up to 60 mm, 8 mm above that.");
    m_review_text->ChangeValue(details);
    m_review.can_apply = m_review.can_apply && summary.ready;
    relayout_page(page);
}

void MixedNozzleWizardDialog::navigate(int direction)
{
    const int current = int(m_pages->GetSelection());
    // Navigation only moves values into the in-memory draft; Apply is the single publication
    // boundary.
    if (direction < 0 && current == kSpeedPage) {
        if (const WizardCandidate *candidate = selected_candidate()) {
            m_input.draft.chosen_coarse_height = candidate->coarse_height;
            m_input.draft.chosen_cadence_ratio = candidate->ratio;
        }
    }
    if (direction > 0) {
        if (current == kModePage) {
            m_input.draft.mode = selected_mode();
            // In Body Split, step 2 starts on the materials the parts already print with, so
            // accepting its defaults changes no part's material.
            if (m_input.draft.mode == MixedNozzleSlicingMode::BodySplit && !m_materials_picked_by_hand)
                default_materials_from_parts();
        }
        if (current == kMaterialsPage) {
            if (!capture_assignments()) return;
            refresh_body_roles_view();
            refresh_fine_heights();
            // The coarse list is on the next page; an empty list is explained there.
            refresh_candidates();
        }
        if (current == kSpeedPage) {
            const WizardCandidate *candidate = selected_candidate();
            if (candidate == nullptr) {
                set_wrapped(m_status, _L("Pick a coarse layer to continue."));
                return;
            }
            m_input.draft.chosen_coarse_height = candidate->coarse_height;
            m_input.draft.chosen_cadence_ratio = candidate->ratio;
            m_input.draft.tower_intent = WizardTowerIntent(m_tower->GetSelection());
            if (!refresh_review()) return;
        }
    }
    // Off has nothing to assign and no layers to choose, so it goes from the mode page straight to
    // Review and back.
    int target = current + direction;
    if (m_input.draft.mode == MixedNozzleSlicingMode::Off) {
        while (target > kModePage && target < kReviewPage && wizard_off_skips_page(target))
            target += direction > 0 ? 1 : -1;
        if (direction > 0 && target >= kReviewPage && !refresh_review())
            return;
    }
    m_pages->SetSelection(std::size_t(std::clamp(target, 0, kReviewPage)));
    refresh_navigation();
}

void MixedNozzleWizardDialog::open_more_options(bool at_joining)
{
    const int current = int(m_pages->GetSelection());
    if (current == kMorePage)
        return;
    m_more_return_page = current;
    m_pages->SetSelection(std::size_t(kMorePage));
    refresh_navigation();
    scroll_more_options(at_joining);
    // The page gets its final size once the book has laid it out; scroll again then.
    CallAfter([this, at_joining] {
        if (int(m_pages->GetSelection()) == kMorePage)
            scroll_more_options(at_joining);
    });
}

void MixedNozzleWizardDialog::scroll_more_options(bool at_joining)
{
    auto *page = dynamic_cast<wxScrolledWindow *>(m_pages->GetPage(std::size_t(kMorePage)));
    if (page == nullptr)
        return;
    if (!at_joining || !m_joining_panel->IsShown()) {
        page->Scroll(0, 0);
        return;
    }
    int step = 0;
    page->GetScrollPixelsPerUnit(nullptr, &step);
    const wxPoint at = page->CalcUnscrolledPosition(m_joining_panel->GetPosition());
    if (step > 0)
        page->Scroll(0, at.y / step);
    if (!m_body_joining.empty())
        m_body_joining.front().second->SetFocus();
}

void MixedNozzleWizardDialog::read_more_options()
{
    WizardMoreOptions options = wizard_more_options(m_input.draft, bool(m_input.review_binding),
                                                    m_input.project_scope_locked);
    options.scope = m_scope->GetSelection() == 1 ? WizardScopeKind::ProjectDefault : WizardScopeKind::CurrentPlate;
    options.tower = WizardTowerIntent(m_tower->GetSelection());
    options.update_material_settings = m_rebind->GetValue();
    options.keep_current_process = m_keep_process->GetValue();
    wizard_apply_more_options(m_input.draft, options);
}

void MixedNozzleWizardDialog::close_more_options()
{
    // "Done" returns to the step it came from with the choices made here in the draft.
    read_more_options();
    m_pages->SetSelection(std::size_t(std::clamp(m_more_return_page, kModePage, kReviewPage)));
    if (m_more_return_page == kReviewPage)
        refresh_review();
    refresh_navigation();
}

void MixedNozzleWizardDialog::refresh_navigation()
{
    const int page = int(m_pages->GetSelection());
    const bool more = page == kMorePage;
    const bool body_split = selected_mode() == MixedNozzleSlicingMode::BodySplit;
    m_body_assignments->Show(body_split);
    m_joining_panel->Show(body_split);
    m_body_slots_panel->Show(body_split);
    m_more->Show(!more);
    m_done->Show(more);
    m_back->Show(!more);
    m_back->Enable(page > 0);
    m_next->Show(!more && page < kReviewPage);
    // Next is held back only for a choice the wizard cannot resolve itself, and the sentence under
    // the buttons says what to change.
    if (page == kSpeedPage) {
        refresh_speed_next();
    } else if (wizard_page_can_block_next(page)) {
        std::string reason;
        if (page == kModePage) {
            // At least one part on each nozzle, which only the user can decide.
            if (body_split && !m_off_chosen)
                reason = wizard_body_roles_block_reason(body_role_rows(), body_roles());
        } else {
            const int fine = m_fine_filament->GetSelection();
            const int coarse = m_coarse_filament->GetSelection();
            // Body parts are chosen as Fine or Coarse on step 1, never left without a slot.
            reason = wizard_materials_block_reason(m_input.effective_config,
                fine >= 0 ? std::optional<std::size_t>(std::size_t(fine)) : std::nullopt,
                coarse >= 0 ? std::optional<std::size_t>(std::size_t(coarse)) : std::nullopt,
                false, {});
        }
        m_next->Enable(reason.empty());
        set_wrapped(m_status, reason.empty() ? wxString{} : from_u8(reason));
    } else {
        m_next->Enable(true);
    }
    m_apply->Show(page == kReviewPage);
    m_apply_slice->Show(page == kReviewPage);
    m_apply->Enable(page == kReviewPage && m_review.can_apply);
    m_apply_slice->Enable(page == kReviewPage && m_review.can_apply);
    m_pages->GetPage(kModePage)->Layout();
    m_pages->GetPage(kMaterialsPage)->Layout();
    // The page just shown, and the status line, wrap to their current width.
    relayout_page(m_pages->GetCurrentPage());
    rewrap_labels(this);
    Layout();
}

wxWindow *MixedNozzleWizardDialog::page_of(wxWindow *window)
{
    while (window != nullptr && window->GetParent() != m_pages)
        window = window->GetParent();
    return window != nullptr ? window : static_cast<wxWindow *>(this);
}

namespace {
// A label's unwrapped text and its last wrap, kept on the label as its client object.
struct WrapState : wxClientData
{
    wxString raw;
    wxString shown;
    int width {-1};
    int right_reserve {0};
};

WrapState &wrap_state(wxStaticText *text)
{
    auto *state = dynamic_cast<WrapState *>(text->GetClientObject());
    if (state == nullptr) {
        state = new WrapState;
        text->SetClientObject(state);
    }
    return *state;
}
}

void MixedNozzleWizardDialog::reserve_wrap(wxStaticText *text, int right_reserve)
{
    wrap_state(text).right_reserve = right_reserve;
}

void MixedNozzleWizardDialog::set_wrapped(wxStaticText *text, const wxString &label)
{
    WrapState &state = wrap_state(text);
    // Always from the new text: the old wrap must not be read back as the raw text.
    state.raw = label;
    state.shown.clear();
    state.width = -1;
    text->SetLabel(label);
    rewrap(text);
}

int MixedNozzleWizardDialog::wrap_width(wxStaticText *text, int right_reserve)
{
    wxWindow *container = page_of(text);
    // The label's left edge inside its container, through every panel it sits in.
    int left = 0;
    for (const wxWindow *window = text; window != nullptr && window != container; window = window->GetParent())
        left += window->GetPosition().x;
    if (left <= 0)
        left = FromDIP(8);
    // A page keeps room for its scroll bar whether the bar shows or not, so the width does not
    // flip back and forth as the bar comes and goes.
    const int available = container == this
        ? container->GetClientSize().x
        : container->GetSize().x - wxSystemSettings::GetMetric(wxSYS_VSCROLL_X, container);
    const int width = available - left - FromDIP(8) - right_reserve;
    // Before the first layout a page has no width of its own yet.
    return width < FromDIP(200) ? FromDIP(580) : width;
}

bool MixedNozzleWizardDialog::rewrap(wxStaticText *text)
{
    WrapState &state = wrap_state(text);
    // A plain SetLabel() since the last wrap: that is the text now.
    const wxString current = text->GetLabel();
    const bool changed = current != state.shown;
    if (changed)
        state.raw = current;
    const int width = wrap_width(text, state.right_reserve);
    // Also rewrap a label whose shown lines are wider than its width, whatever the last wrap said.
    int widest = 0;
    for (const wxString &line : wxSplit(current, '\n', '\0'))
        widest = std::max(widest, text->GetTextExtent(line).x);
    if (!changed && width == state.width && widest <= width)
        return false;
    text->SetLabel(state.raw);
    // wxStaticText::Wrap() does nothing for the width it last wrapped to, even after SetLabel()
    // put the unwrapped text back, so clear that width first.
    text->Wrap(-1);
    text->Wrap(width);
    state.shown = text->GetLabel();
    const bool rewrapped = changed || width != state.width || state.shown != current;
    state.width = width;
    // A word longer than the width stays as it is; reporting no change stops a relayout loop.
    return rewrapped;
}

void MixedNozzleWizardDialog::refresh_speed_next()
{
    if (int(m_pages->GetSelection()) != kSpeedPage)
        return;
    const WizardCandidate *selected = selected_candidate();
    const bool explicit_pick = selected != nullptr && m_cadence_picked_id && selected->stable_id == *m_cadence_picked_id;
    const std::optional<std::size_t> row = m_speed_selection >= 0
        ? std::optional<std::size_t>(std::size_t(m_speed_selection)) : std::nullopt;
    const wxString reason = from_u8(wizard_speed_block_reason(m_cadence_page, row, explicit_pick, bool(m_ranking)));
    m_next->Enable(reason.empty());
    if (!reason.empty())
        set_wrapped(m_status, reason);
    else if (!m_speed_block_text.empty() && wrap_state(m_status).raw == m_speed_block_text)
        set_wrapped(m_status, wxString{});
    m_speed_block_text = reason;
}

bool MixedNozzleWizardDialog::rewrap_labels(wxWindow *container)
{
    // Every label inside the container's panels, however it was made. The dialog's own labels are
    // the ones under the pages; the pages wrap on their own.
    bool any = false;
    const std::function<void(wxWindow *)> walk = [this, &any, &walk](wxWindow *parent) {
        for (wxWindow *child : parent->GetChildren()) {
            if (child == m_pages)
                continue;
            if (auto *text = dynamic_cast<wxStaticText *>(child))
                any = rewrap(text) || any;
            else if (child->IsKindOf(wxCLASSINFO(wxPanel)))
                walk(child);
        }
    };
    walk(container);
    return any;
}

void MixedNozzleWizardDialog::relayout_page(wxWindow *page)
{
    if (page == nullptr)
        return;
    page->Layout();
    if (rewrap_labels(page))
        page->Layout();
    if (auto *scrolled = dynamic_cast<wxScrolledWindow *>(page))
        scrolled->FitInside();
}

DissimilarNozzleDialog::DissimilarNozzleDialog(wxWindow *parent, const DissimilarNozzleSelection &selection,
                                               bool setup_supported, const std::vector<double> &previous_pair)
    : DPIDialog(parent, wxID_ANY, from_u8(dissimilar_nozzle_prompt_text(selection, setup_supported).title),
                wxDefaultPosition, wxDefaultSize, wxCAPTION | wxCLOSE_BOX)
{
    const DissimilarNozzlePromptText text = dissimilar_nozzle_prompt_text(selection, setup_supported, previous_pair);
    SetBackgroundColour(*wxWHITE);
    auto *root = new wxBoxSizer(wxVERTICAL);
    root->SetMinSize(FromDIP(wxSize(540, -1)));
    auto *message = new wxStaticText(this, wxID_ANY, from_u8(text.body));
    message->Wrap(FromDIP(500));
    root->Add(message, 0, wxALL | wxEXPAND, FromDIP(16));
    if (!text.unsupported.empty()) {
        auto *unsupported = new wxStaticText(this, wxID_ANY, from_u8(text.unsupported));
        unsupported->Wrap(FromDIP(500));
        root->Add(unsupported, 0, wxLEFT | wxRIGHT | wxBOTTOM | wxEXPAND, FromDIP(16));
    }
    // One Keep answer. The box decides whether setup opens on the kept pair afterwards.
    auto *set_up_now = new wxCheckBox(this, wxID_ANY, from_u8(text.set_up_now));
    set_up_now->SetValue(setup_supported);
    set_up_now->Enable(setup_supported);
    root->Add(set_up_now, 0, wxLEFT | wxRIGHT | wxBOTTOM | wxEXPAND, FromDIP(16));

    auto *buttons = new DialogButtons(this,
        {from_u8(text.use_left), from_u8(text.use_right), from_u8(text.cancel), from_u8(text.keep_both)},
        _L(from_u8(text.keep_both)), 2);
    buttons->GetButtonFromIndex(0)->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { m_decision = DissimilarNozzleDecision::LeftOnly; EndModal(wxID_OK); });
    buttons->GetButtonFromIndex(1)->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { m_decision = DissimilarNozzleDecision::RightOnly; EndModal(wxID_OK); });
    // Keeping the pair writes the project nozzles and nothing else; setup, when ticked, opens
    // afterwards on the kept pair.
    auto *keep = buttons->GetButtonFromIndex(3);
    keep->Enable(setup_supported);
    keep->Bind(wxEVT_BUTTON, [this, set_up_now](wxCommandEvent &) {
        m_decision = DissimilarNozzleDecision::KeepPair;
        m_set_up_now = set_up_now->GetValue();
        EndModal(wxID_OK);
    });
    buttons->GetCANCEL()->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { m_decision = DissimilarNozzleDecision::Cancel; EndModal(wxID_CANCEL); });
    // The left pair and the right pair keep at least the gap each pair has inside it; the long
    // labels leave no stretch between them otherwise.
    buttons->SetMinSize(wxSize(buttons->GetBestSize().x + FromDIP(ButtonProps::ChoiceButtonGap()), -1));
    root->Add(buttons, 0, wxEXPAND);
    Bind(wxEVT_CLOSE_WINDOW, [this](wxCloseEvent &) { m_decision = DissimilarNozzleDecision::Cancel; EndModal(wxID_CANCEL); });
    SetSizer(root);
    Layout();
    root->Fit(this);
    wxGetApp().UpdateDlgDarkUI(this);
}

} // namespace Slic3r::GUI
