#include "MixedNozzleSidebarPanel.hpp"
#include "TestMode.hpp"

#include "GUI.hpp"
#include "GUI_App.hpp"
#include "GUI_ObjectList.hpp"
#include "I18N.hpp"
#include "Plater.hpp"
#include "Tab.hpp"
#include "ValidationActionRouting.hpp"
#include "MixedNozzleAssignmentBindingTransaction.hpp"
#include "MixedNozzleDecisionDialog.hpp"
#include "MixedNozzleSetupDialog.hpp"
#include "MixedNozzleWizardModel.hpp"
#include "MixedNozzleWizardTransaction.hpp"
#include "Widgets/Label.hpp"
#include "libslic3r/PresetBundle.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <utility>
#include <wx/checklst.h>
#include <wx/menu.h>
#include <wx/dialog.h>
#include <wx/msgdlg.h>
#include "libslic3r/Print.hpp"

namespace Slic3r { namespace GUI {

namespace {

size_t configured_nozzle_count(const DynamicPrintConfig &full_config)
{
    const auto *nozzles = full_config.option<ConfigOptionFloats>("nozzle_diameter");
    return nozzles == nullptr ? 0 : nozzles->values.size();
}

std::string filament_color_label(const PresetBundle &bundle, size_t logical_index)
{
    const auto *colors = bundle.project_config.option<ConfigOptionStrings>("filament_colour");
    if (colors != nullptr && logical_index < colors->values.size() &&
        !colors->values[logical_index].empty())
        return colors->values[logical_index];
    const auto &names = bundle.filament_presets;
    if (logical_index >= names.size())
        return {};
    const Preset *preset = bundle.filaments.find_preset(names[logical_index]);
    return preset == nullptr || !preset->config.has("filament_colour")
        ? std::string() : preset->config.opt_string("filament_colour", 0);
}

// Messages name the per-object admission rule of the all-row validator, since the body count is
// independent of the two physical nozzle roles.
wxString body_diagnostic_text(BodySplitEditorDiagnostic diagnostic, size_t row_count)
{
    switch (diagnostic) {
    case BodySplitEditorDiagnostic::None:
        return _L("Change a part's material or layer, then Apply.");
    case BodySplitEditorDiagnostic::ExactlyTwoModelPartsRequired:
        return wxString::Format(_L("Body Split needs at least two parts, or one part with painted regions; found %zu. Paint regions for the fine nozzle with Color Painting, or use Split to parts."), row_count);
    case BodySplitEditorDiagnostic::LayerHeightInvalid:
        return _L("A body has an invalid base or effective layer height");
    case BodySplitEditorDiagnostic::LogicalFilamentInvalid:
        return _L("Pick a material for each part");
    case BodySplitEditorDiagnostic::DistinctLogicalFilamentsRequired:
        return _L("Put at least one part on each nozzle");
    case BodySplitEditorDiagnostic::PhysicalToolUnresolved:
        return _L("A body's material is not assigned to a nozzle. Set Filament grouping to Custom and put each material on a nozzle.");
    case BodySplitEditorDiagnostic::DistinctPhysicalToolsRequired:
        return _L("Put at least one part on each nozzle");
    case BodySplitEditorDiagnostic::PhysicalNozzleOrderingInvalid:
        return _L("The coarse parts must print on the larger nozzle");
    case BodySplitEditorDiagnostic::CadenceNotQualified:
        return _L("Each object needs one fine layer height and one coarse layer height that both nozzles can print");
    case BodySplitEditorDiagnostic::StaleTarget:
        return _L("The plate changed; select it again");
    case BodySplitEditorDiagnostic::DuplicateTarget:
        return _L("A part is listed twice; select the plate again");
    case BodySplitEditorDiagnostic::PaintedFilamentPhysicalToolMismatch:
        return _L("A painted material is not on a nozzle that can print its layer height");
    }
    return _L("The mixed-nozzle settings are not ready");
}

wxString assignment_binding_diagnostic_text(MixedNozzleAssignmentBindingDiagnostic diagnostic)
{
    switch (diagnostic) {
    case MixedNozzleAssignmentBindingDiagnostic::None:
        return _L("Change a material or layer, then Apply.");
    case MixedNozzleAssignmentBindingDiagnostic::Cancelled:
        return _L("Nothing was changed.");
    case MixedNozzleAssignmentBindingDiagnostic::MissingAssignment:
        return _L("There is nothing to apply. Click Reload and try again.");
    case MixedNozzleAssignmentBindingDiagnostic::InvalidBinding:
        return _L("These material settings can't be used. Click Reload and try again.");
    case MixedNozzleAssignmentBindingDiagnostic::StaleBinding:
        return _L("The material settings changed. Click Reload.");
    case MixedNozzleAssignmentBindingDiagnostic::StaleTarget:
        return _L("The model or plate changed. Click Reload.");
    case MixedNozzleAssignmentBindingDiagnostic::StaleConfiguration:
        return _L("The model or material settings changed. Click Reload.");
    case MixedNozzleAssignmentBindingDiagnostic::AssignmentRejected:
        return _L("This change can't be applied. Check each part's material and layer.");
    case MixedNozzleAssignmentBindingDiagnostic::Unsupported:
        return _L("This change can't be applied safely.");
    }
    return _L("The mixed-nozzle settings are not ready");
}

} // namespace

std::function<MixedNozzleAssignmentStageResult(MixedNozzleAssignmentBindingStageState &)>
make_body_split_stage_assignment(BodySplitApplyRequest request, BodySplitEditorDiagnostic &diagnostic)
{
    return [body_request = std::move(request), &diagnostic](MixedNozzleAssignmentBindingStageState &state) {
        const auto staged_maps = state.plate.get_real_filament_maps(state.bundle.project_config);
        const auto staged_volumes = state.plate.get_real_filament_volume_maps(state.bundle.project_config);
        PrintConfig staged_resolver = mixed_nozzle_resolver_config_from_full(
            state.bundle.full_config(true, staged_maps, staged_volumes));
        staged_resolver.filament_map_mode.value =
            state.plate.get_real_filament_map_mode(state.bundle.project_config);
        staged_resolver.filament_map.values = state.plate.get_real_filament_maps(state.bundle.project_config);
        const BodySplitApplyResult staged = apply_body_split_editor_request(
            state.model, *state.plate.config(), staged_resolver, body_request);
        diagnostic = staged.diagnostic;
        return MixedNozzleAssignmentStageResult{staged.applied, staged.changed};
    };
}

std::function<MixedNozzleAssignmentStageResult(MixedNozzleAssignmentBindingStageState &)>
make_feature_split_stage_assignment(FeatureSplitApplyRequest request, MixedNozzleSlicingMode effective_mode,
                                    size_t nozzle_count)
{
    return [feature_request = std::move(request), effective_mode, nozzle_count](
        MixedNozzleAssignmentBindingStageState &state) {
        const auto staged_maps = state.plate.get_real_filament_maps(state.bundle.project_config);
        const auto staged_volumes = state.plate.get_real_filament_volume_maps(state.bundle.project_config);
        const DynamicPrintConfig staged_full_config = state.bundle.full_config(
            true, staged_maps, staged_volumes);
        PrintConfig staged_resolver = feature_split_resolver_config_from_full(staged_full_config);
        staged_resolver.filament_map_mode.value =
            state.plate.get_real_filament_map_mode(state.bundle.project_config);
        staged_resolver.filament_map.values = state.plate.get_real_filament_maps(state.bundle.project_config);
        // The staging PartPlate owns only copied local config and has no live ObjectID, so the
        // captured stable plate identity in the request is what the validator compares; publication
        // preflight checks that identity against the live plate before the snapshot.
        const double staged_base_height = staged_full_config.has("layer_height")
            ? staged_full_config.opt_float("layer_height") : 0.;
        const FeatureSplitApplyResult staged = commit_feature_split_editor_request(
            state.bundle.project_config, state.bundle.prints.get_edited_preset().config, state.plate,
            effective_mode, state.plate.get_index(), feature_request.target.plate_id, nozzle_count,
            staged_resolver,
            feature_split_coarse_cadence_choices(staged_base_height, staged_resolver), feature_request);
        return MixedNozzleAssignmentStageResult{staged.applied, staged.changed};
    };
}

MixedNozzleRebindReviewResult review_mixed_nozzle_rebind(
    wxWindow *parent, PresetBundle &bundle, PartPlate &plate,
    const MixedNozzleRebindSignature *expected_signature)
{
    MixedNozzleRebindReviewResult result;
    const auto maps = plate.get_real_filament_maps(bundle.project_config);
    const auto volumes = plate.get_real_filament_volume_maps(bundle.project_config);
    const auto current_signature = bundle.mixed_nozzle_rebind_signature(maps, volumes);
    if (expected_signature != nullptr && current_signature != *expected_signature) {
        result.status = MixedNozzleRebindReviewStatus::Stale;
        return result;
    }

    auto rebind_plan = bundle.mixed_nozzle_rebind_plan(maps, volumes);
    if (!rebind_plan.offered)
        return result;

    wxDialog dialog(parent, wxID_ANY, _L("Update material settings"), wxDefaultPosition,
                    wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER);
    auto *sizer = new wxBoxSizer(wxVERTICAL);
    auto *explanation = new wxStaticText(&dialog, wxID_ANY,
        _L("Ticked settings change to the value shown for the nozzle each material is on now. Unticked settings keep this project's value, even when the two are equal. Tower prime for material changes uses the material's own default, not the nozzle-change prime. Each row lists the values for every nozzle; a change applies to the whole setting."));
    explanation->Wrap(parent->FromDIP(660));
    sizer->Add(explanation, 0, wxEXPAND | wxALL, parent->FromDIP(12));
    auto *choices = new wxCheckListBox(&dialog, wxID_ANY, wxDefaultPosition,
                                       parent->FromDIP(wxSize(700, 340)), 0, nullptr, wxLB_HSCROLL);
    const auto selected = rebind_plan.default_selection();
    const auto preset_labels = filament_preset_labels(bundle);
    std::vector<wxString> row_details;
    row_details.reserve(rebind_plan.entries.size());
    for (size_t i = 0; i < rebind_plan.entries.size(); ++i) {
        const auto &entry = rebind_plan.entries[i];
        const auto *definition = bundle.project_config.def()->get(entry.key);
        const bool is_prime = entry.key == "filament_prime_volume";
        const wxString label = is_prime ? _L("Tower prime for filament changes") :
            definition && !definition->label.empty() ? wxGetTranslation(from_u8(definition->label)) : from_u8(entry.key);
        const auto destination = mixed_nozzle_rebind_destination(entry.key);
        const wxString destination_text = destination == MixedNozzleRebindDestination::MaterialAncestor
            ? _L("material default") : _L("value for this nozzle");
        const wxString material = from_u8(filament_material_label(bundle, entry.logical_filament));
        const wxString color = from_u8(filament_color_label(bundle, entry.logical_filament));
        const wxString identity = entry.logical_filament < preset_labels.size()
            ? from_u8(preset_labels[entry.logical_filament]) : wxString();
        const wxString units = is_prime ? wxString::FromUTF8("mm³") :
            definition && !definition->sidetext.empty() ? from_u8(definition->sidetext) : wxString();
        const wxString text = wxString::Format(_L("Filament %d · %s: %s → %s %s · %s"),
            int(entry.logical_filament + 1), label, from_u8(entry.current_columns),
            from_u8(entry.sibling_columns), units, destination_text);
        row_details.push_back(identity + " · " + material + (color.empty() ? wxString() : " · " + color));
        choices->Append(text);
        choices->Check(unsigned(i), selected[i]);
    }
    sizer->Add(choices, 1, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, parent->FromDIP(12));
    auto *detail = new wxStaticText(&dialog, wxID_ANY,
                                    row_details.empty() ? wxString() : row_details.front());
    detail->Wrap(parent->FromDIP(660));
    sizer->Add(detail, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, parent->FromDIP(12));
    choices->Bind(wxEVT_LISTBOX, [detail, row_details](wxCommandEvent &event) {
        const int row = event.GetSelection();
        if (row >= 0 && size_t(row) < row_details.size()) {
            detail->SetLabel(row_details[size_t(row)]);
            detail->Wrap(detail->FromDIP(660));
            detail->GetParent()->Layout();
        }
    });
    sizer->Add(dialog.CreateSeparatedButtonSizer(wxOK | wxCANCEL), 0,
               wxEXPAND | wxALL, parent->FromDIP(12));
    if (auto *reset = dialog.FindWindow(wxID_OK))
        reset->SetLabel(_L("Update ticked settings"));
    dialog.SetSizerAndFit(sizer);
    wxGetApp().UpdateDlgDarkUI(&dialog);
    if (dialog.ShowModal() != wxID_OK) {
        result.status = MixedNozzleRebindReviewStatus::Cancelled;
        return result;
    }

    const auto current_maps = plate.get_real_filament_maps(bundle.project_config);
    const auto current_volumes = plate.get_real_filament_volume_maps(bundle.project_config);
    const auto after_signature = bundle.mixed_nozzle_rebind_signature(current_maps, current_volumes);
    if (after_signature != rebind_plan.signature ||
        (expected_signature != nullptr && after_signature != *expected_signature)) {
        result.status = MixedNozzleRebindReviewStatus::Stale;
        return result;
    }

    result.status = MixedNozzleRebindReviewStatus::Applied;
    result.plan = std::move(rebind_plan);
    result.accepted_rebind.resize(choices->GetCount());
    for (size_t i = 0; i < result.accepted_rebind.size(); ++i)
        result.accepted_rebind[i] = choices->IsChecked(unsigned(i));
    return result;
}

MixedNozzleSidebarPanel::MixedNozzleSidebarPanel(wxWindow *parent, Plater *plater)
    : wxScrolledWindow(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL | wxVSCROLL)
    , m_plater(plater)
{
    SetBackgroundColour(*wxWHITE);
    SetScrollRate(0, FromDIP(10));
    m_sizer = new wxBoxSizer(wxVERTICAL);

    // --- what was set up, in one line, then the lines that apply ----------------------------
    const auto add_line = [this](wxStaticText *&line) {
        line = new wxStaticText(this, wxID_ANY, wxEmptyString);
        line->SetFont(Label::Body_13);
        m_sizer->Add(line, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, FromDIP(8));
        add_wrapped_line(line);
    };
    add_line(m_mode_status);
    add_line(m_plate_line);
    add_line(m_guard_line);
    // The tower ownership state belongs in the visible status text, not in a tooltip.
    add_line(m_tower_status);
    add_line(m_sliced_line);
    add_line(m_coarse_line);
    // A readout of what the last slice produced; it never changes a setting.
    add_line(m_tower_readout);
    set_wrapped_label(m_tower_readout, from_u8(MIXED_NOZZLE_SIDEBAR_NOT_SLICED));
    // The sidebar width follows the window, so the lines wrap again when it changes.
    Bind(wxEVT_SIZE, [this](wxSizeEvent &event) {
        // The content follows the panel width; only its height scrolls.
        if (m_sizer != nullptr)
            SetVirtualSize(GetClientSize().x, m_sizer->GetMinSize().y);
        event.Skip();
        if (line_wrap_width() != m_wrap_width)
            CallAfter([this] { rewrap_lines(); });
    });

    // Shown only when a material still carries the other nozzle's settings.
    auto *rebind_row = new wxBoxSizer(wxHORIZONTAL);
    m_rebind_line = new wxStaticText(this, wxID_ANY, wxEmptyString);
    m_rebind_line->SetFont(Label::Body_13);
    m_rebind = new Button(this, _L("Update..."));
    add_wrapped_line(m_rebind_line, m_rebind);
    m_rebind->SetStyle(ButtonStyle::Regular, ButtonType::Compact);
    m_rebind->SetToolTip(_L("Review each material's settings for the nozzle it is on now. Only the "
                            "settings you tick change."));
    m_rebind->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { on_rebind(); });
    rebind_row->Add(m_rebind_line, 1, wxALIGN_CENTER_VERTICAL | wxRIGHT, FromDIP(8));
    rebind_row->Add(m_rebind, 0, wxALIGN_CENTER_VERTICAL);
    m_sizer->Add(rebind_row, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, FromDIP(8));

    // [Why fine or coarse?] [Change...] [Turn off]            [More]
    auto *actions_row = new wxBoxSizer(wxHORIZONTAL);
    m_time_decisions = new Button(this, _L("Why fine or coarse?"));
    m_time_decisions->SetStyle(ButtonStyle::Confirm, ButtonType::Compact);
    m_time_decisions->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { show_time_decisions(); });
    // "Change..." opens setup on its detail step with the current values.
    m_mode_setup = new Button(this, _L("Change..."));
    m_mode_setup->SetStyle(ButtonStyle::Confirm, ButtonType::Compact);
    m_mode_setup->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        if (m_plater == nullptr)
            return;
        // A plate that is off while another plate is on starts from step 1 instead.
        const bool change = m_signature.target_valid && m_signature.effective != MixedNozzleSlicingMode::Off;
        m_plater->open_mixed_nozzle_wizard(false, -1, nullptr,
            change ? MixedNozzleWizardPage::Speed : MixedNozzleWizardPage::ModeAndScope);
    });
    // The way back to one nozzle. It asks nothing and is one Undo step.
    m_mode_off = new Button(this, _L("Turn off"));
    m_mode_off->SetStyle(ButtonStyle::Regular, ButtonType::Compact);
    m_mode_off->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        if (m_plater != nullptr && m_plater->turn_mixed_nozzle_off(true))
            poll_refresh();
    });
    m_more = new Button(this, _L("More"));
    m_more->SetStyle(ButtonStyle::Regular, ButtonType::Compact);
    m_more->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { show_more_menu(); });
    actions_row->Add(m_time_decisions, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, FromDIP(6));
    actions_row->Add(m_mode_setup, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, FromDIP(6));
    actions_row->Add(m_mode_off, 0, wxALIGN_CENTER_VERTICAL);
    actions_row->AddStretchSpacer();
    actions_row->Add(m_more, 0, wxALIGN_CENTER_VERTICAL);
    m_sizer->Add(actions_row, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, FromDIP(8));

    // --- Body Split: one vertical card per body ----------------------------------------------
    m_body_panel = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    m_body_panel->SetBackgroundColour(*wxWHITE);
    m_body_sizer = new wxBoxSizer(wxVERTICAL);
    m_body_panel->SetSizer(m_body_sizer);
    m_sizer->Add(m_body_panel, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, FromDIP(8));

    // --- Feature Split: fine / coarse pair and process cadence -------------------------------
    m_feature_panel = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
    m_feature_panel->SetBackgroundColour(*wxWHITE);
    auto *feature_sizer = new wxBoxSizer(wxVERTICAL);
    m_feature_fine_label = new wxStaticText(m_feature_panel, wxID_ANY, _L("Fine material"));
    m_feature_fine_label->SetFont(Label::Body_13);
    m_feature_fine = new ComboBox(m_feature_panel, wxID_ANY, wxEmptyString, wxDefaultPosition,
                                  wxDefaultSize, 0, nullptr, wxCB_READONLY);
    m_feature_coarse_label = new wxStaticText(m_feature_panel, wxID_ANY, _L("Coarse material"));
    m_feature_coarse_label->SetFont(Label::Body_13);
    m_feature_coarse = new ComboBox(m_feature_panel, wxID_ANY, wxEmptyString, wxDefaultPosition,
                                    wxDefaultSize, 0, nullptr, wxCB_READONLY);
    m_feature_cadence = new ComboBox(m_feature_panel, wxID_ANY, wxEmptyString, wxDefaultPosition,
                                     wxDefaultSize, 0, nullptr, wxCB_READONLY);
    m_feature_cadence->Bind(wxEVT_COMBOBOX, [this](wxCommandEvent &) { mark_dirty(); });
    m_feature_fine->Bind(wxEVT_COMBOBOX, [this](wxCommandEvent &) {
        m_feature_selection.fine_selection = m_feature_fine->GetSelection();
        rebuild_feature_cadences(wxGetApp().preset_bundle->full_config(), true);
        mark_dirty();
    });
    m_feature_coarse->Bind(wxEVT_COMBOBOX, [this](wxCommandEvent &) {
        m_feature_selection.coarse_selection = m_feature_coarse->GetSelection();
        rebuild_feature_cadences(wxGetApp().preset_bundle->full_config(), true);
        mark_dirty();
    });
    feature_sizer->Add(m_feature_fine_label, 0, wxEXPAND);
    feature_sizer->Add(m_feature_fine, 0, wxEXPAND | wxBOTTOM, FromDIP(6));
    feature_sizer->Add(m_feature_coarse_label, 0, wxEXPAND);
    feature_sizer->Add(m_feature_coarse, 0, wxEXPAND | wxBOTTOM, FromDIP(6));
    feature_sizer->Add(m_feature_cadence, 0, wxEXPAND);
    m_feature_panel->SetSizer(feature_sizer);
    m_sizer->Add(m_feature_panel, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, FromDIP(8));

    // --- status + one-click Apply -------------------------------------------------------------
    m_status = new wxStaticText(this, wxID_ANY, wxEmptyString);
    m_status->SetFont(Label::Body_13);
    add_wrapped_line(m_status);
    m_sizer->Add(m_status, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, FromDIP(8));

    m_apply = new Button(this, _L("Apply"));
    m_apply->SetStyle(ButtonStyle::Confirm, ButtonType::Compact);
    m_apply->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { on_apply(); });
    m_reload = new Button(this, _L("Reload"));
    m_reload->SetStyle(ButtonStyle::Confirm, ButtonType::Compact);
    m_reload->SetToolTip(_L("Discard unapplied edits and reload current settings."));
    m_reload->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { reload(); });
    auto *actions = new wxBoxSizer(wxHORIZONTAL);
    actions->Add(m_reload, 0, wxRIGHT, FromDIP(8));
    actions->Add(m_apply);
    m_sizer->Add(actions, 0, wxALIGN_RIGHT | wxALL, FromDIP(8));

    SetSizer(m_sizer);
    // Start hidden and let the first poll_refresh() decide. The Sidebar is built inside the Plater
    // constructor, so reading the plate list or the preset bundle here would see half-built state.
    show_section(MixedNozzleSidebarSection::Hidden);
    Layout();
}

// Readout only, cached per slice so a periodic sidebar refresh never rescans the move stream.
void MixedNozzleSidebarPanel::refresh_tower_readout(PartPlate *plate)
{
    const auto pending = [this] {
        m_tower_readout_key = 0;
        m_slice_facts = {};
    };
    if (plate == nullptr || !plate->is_slice_result_valid())
        return pending();
    PrintBase *base = nullptr;
    GCodeResult *result = nullptr;
    plate->get_print(&base, &result, nullptr);
    const auto *print = dynamic_cast<const Print *>(base);
    const auto *slice = plate->get_slice_result();
    if (print == nullptr || slice == nullptr)
        return pending();
    const std::size_t key = mixed_nozzle_sidebar_slice_revision(true, m_target.plate_index,
        result == nullptr ? 0 : result->moves.size(), double(slice->print_statistics.modes[0].time));
    if (key == m_tower_readout_key)
        return;
    m_tower_readout_key = key;
    m_slice_facts = {};
    const auto &data = print->wipe_tower_data();
    const Vec2d footprint = data.bbx.size();
    const double width = footprint.x() > 0. ? footprint.x() : double(data.width);
    const double depth = footprint.y() > 0. ? footprint.y() : double(data.depth);
    double volume = 0.;
    for (const auto &entry : slice->print_statistics.wipe_tower_volumes_per_extruder)
        volume += entry.second;
    double seconds = 0.;
    if (result != nullptr)
        for (const auto &move : result->moves)
            if (move.extrusion_role == erWipeTower)
                seconds += double(move.time[0]);
    m_slice_facts.tower_readout = mixed_nozzle_sidebar_tower_readout_line(width, depth, volume, seconds);
    // The time, nozzle changes and coarse range of this slice, read once per slice.
    mixed_nozzle_sidebar_read_slice_statistics(slice->print_statistics, m_slice_facts);
    double height = 0.;
    for (const PrintObject *object : print->objects())
        height = std::max(height, unscale<double>(object->height()));
    if (height > 0.)
        m_slice_facts.model_height = height;
    const MixedNozzleDecisionData decisions = collect_mixed_nozzle_decision_data(m_plater, false);
    if (decisions.available) {
        m_slice_facts.coarse_known = true;
        if (decisions.summary.coarse_band_count > 0) {
            m_slice_facts.coarse_z_low = decisions.summary.coarse_z_low;
            m_slice_facts.coarse_z_high = decisions.summary.coarse_z_high;
        }
    }
}

// The mode line, the tower status line and the tower readout come from one model function, and
// are cheap enough to rebuild on a readout-only change that must leave an unapplied draft alone.
void MixedNozzleSidebarPanel::refresh_readouts()
{
    if (m_plater == nullptr)
        return;
    auto &plates = m_plater->get_partplate_list();
    PartPlate *plate = plates.get_plate(plates.get_curr_plate_index());
    auto &bundle = *wxGetApp().preset_bundle;
    // Composed for the plate's own map, as the slicer composes it.
    const DynamicPrintConfig full_config = plate != nullptr ? wizard_plate_effective_config(bundle, *plate)
                                                            : bundle.full_config();
    const auto *mask = full_config.option<ConfigOptionStrings>("different_settings_to_system");
    refresh_tower_readout(plate);

    MixedNozzleSidebarFacts facts = m_slice_facts;
    if (const auto *nozzles = full_config.option<ConfigOptionFloats>("nozzle_diameter"))
        facts.nozzle_diameters = nozzles->values;
    facts.fine_height = full_config.has("layer_height") ? full_config.opt_float("layer_height") : 0.;
    facts.coarse_height = full_config.has("mixed_nozzle_coarse_layer_height")
        ? full_config.opt_float("mixed_nozzle_coarse_layer_height") : 0.;
    if (m_section == MixedNozzleSidebarSection::Body) {
        std::vector<std::pair<double, double>> heights;
        for (const BodySplitEditorRow &row : m_body_model.rows)
            heights.emplace_back(row.base_layer_height, row.effective_layer_height);
        const auto counts = mixed_nozzle_sidebar_body_counts(heights);
        facts.fine_parts = counts.fine_parts;
        facts.coarse_parts = counts.coarse_parts;
        facts.coarse_height = counts.coarse_height;
    }
    const auto *tower = full_config.option<ConfigOptionBool>("enable_prime_tower");
    facts.tower_enabled = tower == nullptr || tower->value;
    facts.rebind_offered = m_rebind_offered;
    // A Body Split project on a Feature Split process preset is said here, with the Body Split
    // preset setup would switch it to.
    if (m_signature.target_valid && m_signature.effective == MixedNozzleSlicingMode::BodySplit)
        facts.guard_line = wizard_body_process_guard_line(bundle);

    const auto texts = mixed_nozzle_sidebar_lines(m_signature, bundle.project_config,
        mask == nullptr ? std::vector<std::string>{} : mask->values, facts);
    const bool any = m_section != MixedNozzleSidebarSection::Hidden;
    // A line's tooltip repeats it in full unless the line has its own explanation.
    const auto set_line = [this, any](wxStaticText *line, const std::string &text, bool show,
                                      const wxString &tooltip = wxString()) {
        if (line == nullptr)
            return;
        set_wrapped_label(line, from_u8(text));
        line->SetToolTip(tooltip.empty() ? from_u8(text) : tooltip);
        line->Show(any && show && !text.empty());
    };
    set_line(m_mode_status, texts.mode_status, true);
    set_line(m_plate_line, texts.plate_line, true);
    set_line(m_guard_line, texts.guard_line, true);
    set_line(m_tower_status, texts.tower_status, !texts.sliced);
    set_line(m_sliced_line, texts.sliced_line, texts.sliced);
    set_line(m_coarse_line, texts.coarse_line, texts.sliced);
    set_line(m_tower_readout, texts.tower_readout, true, texts.sliced
        ? _L("Prime tower size, material and time from the last slice of this plate.")
        : _L("The prime tower size, material and time show here after this plate is sliced."));
    set_line(m_rebind_line, texts.rebind_line, true);
    if (m_rebind != nullptr)
        m_rebind->Show(any && !texts.rebind_line.empty());
    if (m_mode_setup != nullptr) {
        const wxString label = m_signature.effective == MixedNozzleSlicingMode::Off ? _L("Set up...") : _L("Change...");
        if (m_mode_setup->GetLabel() != label)
            m_mode_setup->SetLabel(label);
    }
    if (m_mode_off != nullptr)
        m_mode_off->Show(any && m_signature.effective != MixedNozzleSlicingMode::Off);
    // "Why fine or coarse?" explains a Feature Split slice's layer bands, so it waits for one.
    // Body Split has no bands: each part prints on the nozzle its material is on.
    if (m_time_decisions != nullptr)
        m_time_decisions->Show(any && texts.sliced && m_section == MixedNozzleSidebarSection::Feature);
    relayout();
}

int MixedNozzleSidebarPanel::line_wrap_width() const
{
    // The lines sit inside an 8 DIP margin on each side.
    return GetClientSize().x - 2 * FromDIP(8);
}

void MixedNozzleSidebarPanel::add_wrapped_line(wxStaticText *line, wxWindow *beside)
{
    m_wrapped_lines.push_back({line, wxString(), beside});
}

void MixedNozzleSidebarPanel::set_wrapped_label(wxStaticText *line, const wxString &text)
{
    const auto entry = std::find_if(m_wrapped_lines.begin(), m_wrapped_lines.end(),
                                    [line](const WrappedLine &item) { return item.line == line; });
    if (entry == m_wrapped_lines.end()) {
        line->SetLabel(text);
        return;
    }
    entry->text = text;
    line->SetLabel(text);
    int width = line_wrap_width();
    if (entry->beside != nullptr)
        width -= entry->beside->GetBestSize().x + FromDIP(8);
    if (width > 0) {
        // Wrap() does nothing for the width it last wrapped to, even after SetLabel(), so clear
        // that width first.
        line->Wrap(-1);
        line->Wrap(width);
    }
}

void MixedNozzleSidebarPanel::rewrap_lines()
{
    const int width = line_wrap_width();
    if (width <= 0 || width == m_wrap_width)
        return;
    m_wrap_width = width;
    for (const WrappedLine &entry : m_wrapped_lines)
        set_wrapped_label(entry.line, entry.text);
    // Wrapping changes the lines' height only, so the parent keeps this width and this does
    // not come back here.
    relayout();
}

void MixedNozzleSidebarPanel::relayout()
{
    // Best sizes are cached, and showing or hiding a child does not clear them.
    for (const BodyRowControls &row : m_body_rows)
        if (row.card != nullptr)
            row.card->InvalidateBestSize();
    for (wxWindow *editor : {static_cast<wxWindow *>(m_body_panel), static_cast<wxWindow *>(m_feature_panel)})
        if (editor != nullptr) {
            editor->InvalidateBestSize();
            editor->Layout();
        }
    InvalidateBestSize();
    // The sidebar sizer sets this panel's height, so a line that wraps to more or fewer rows
    // needs it to run again, as the other sidebar sections do when their content changes.
    if (IsShown() && GetParent() != nullptr && GetBestSize().y != GetSize().y)
        GetParent()->Layout();
    if (m_sizer != nullptr)
        SetVirtualSize(GetClientSize().x, m_sizer->GetMinSize().y);
    Layout();
}

void MixedNozzleSidebarPanel::keep_actions_in_view()
{
    int unit = 0;
    GetScrollPixelsPerUnit(nullptr, &unit);
    if (unit <= 0 || m_apply == nullptr || !m_apply->IsShown())
        return;
    // In content coordinates: the top of the status line and the bottom of the buttons.
    const int top = CalcUnscrolledPosition(m_status->GetPosition()).y;
    const int bottom = CalcUnscrolledPosition(m_apply->GetPosition()).y + m_apply->GetSize().y + FromDIP(8);
    const int view_top = CalcUnscrolledPosition(wxPoint(0, 0)).y;
    const int view_height = GetClientSize().y;
    if (bottom <= view_top + view_height)
        return;
    const int y = std::min(top, bottom - view_height);
    Scroll(-1, (y + unit - 1) / unit);
}

int MixedNozzleSidebarPanel::height_limit() const
{
    wxWindow *parent = GetParent();
    if (parent == nullptr || parent->GetSizer() == nullptr)
        return INT_MAX;
    int others = 0;
    for (wxSizerItem *item : parent->GetSizer()->GetChildren()) {
        if (item->GetWindow() == this)
            others += item->GetMinSizeWithBorder().y - item->GetMinSize().y;
        else if (item->IsShown()) {
            item->CalcMin();
            int need = item->GetMinSizeWithBorder().y;
            // Some sections report less than their content needs; count what their sizer needs.
            if (item->GetWindow() != nullptr && item->GetWindow()->GetSizer() != nullptr)
                need = std::max(need, item->GetWindow()->GetSizer()->GetMinSize().y + need - item->GetMinSize().y);
            others += need;
        }
    }
    return std::max(0, parent->GetClientSize().y - others);
}

wxSize MixedNozzleSidebarPanel::DoGetBestSize() const
{
    if (m_sizer == nullptr)
        return wxScrolledWindow::DoGetBestSize();
    const wxSize content = m_sizer->GetMinSize();
    // Leave the Process settings some room. In a very short sidebar this section keeps a small
    // height and scrolls, and the sidebar scrolls rather than squeeze the other sections.
    const int limit = std::max(height_limit() - FromDIP(60), FromDIP(40));
    return {content.x, std::min(content.y, limit)};
}

void MixedNozzleSidebarPanel::show_time_decisions()
{
    // One shared dialog answers "why fine or coarse?" for the sidebar, Preview's Physical Tool
    // legend and the post-slice notification.
    show_mixed_nozzle_decision_dialog(this);
}

void MixedNozzleSidebarPanel::on_rebind()
{
    if (m_plater == nullptr || wxGetApp().preset_bundle == nullptr)
        return;
    auto &bundle = *wxGetApp().preset_bundle;
    auto &plates = m_plater->get_partplate_list();
    PartPlate *plate = plates.get_plate(plates.get_curr_plate_index());
    if (plate == nullptr)
        return;
    const auto target_signature = collect_signature();
    auto review = review_mixed_nozzle_rebind(this, bundle, *plate);
    if (review.status == MixedNozzleRebindReviewStatus::NotNeeded)
        return;
    if (review.status == MixedNozzleRebindReviewStatus::Cancelled)
        return;
    if (review.status == MixedNozzleRebindReviewStatus::Stale) {
        reject_changed_settings();
        return;
    }
    if (collect_signature() != target_signature) {
        reject_changed_settings();
        return;
    }
    plate = plates.get_plate(plates.get_curr_plate_index());
    if (plate == nullptr)
        return;
    MixedNozzleAssignmentBindingRequest request;
    request.rebind_plan = std::move(review.plan);
    request.accepted_rebind = std::move(review.accepted_rebind);
    request.rebind_action = MixedNozzleRebindAction::Apply;
    request.stage_assignment = [](MixedNozzleAssignmentBindingStageState &) {
        return MixedNozzleAssignmentStageResult{true, false};
    };
    auto transaction_plan = stage_mixed_nozzle_assignment_binding(bundle, m_plater->model(), *plate, request);
    MixedNozzleAssignmentBindingDiagnostic diagnostic = MixedNozzleAssignmentBindingDiagnostic::None;
    const bool applied = commit_mixed_nozzle_assignment_binding(
        bundle, m_plater->model(), std::vector<PartPlate *>{plate}, transaction_plan,
        {{},
         {},
         [this] {
             m_plater->set_plater_dirty(true);
             notify_mixed_nozzle_commit([this] {
                 m_plater->on_config_change(wxGetApp().preset_bundle->full_config());
             });
         },
         [this](const MixedNozzleSetupPlan &plan) {
             return m_plater->take_mixed_nozzle_setup_snapshot("Update material settings", plan);
         }}, &diagnostic);
    if (!applied) {
        reject_changed_settings();
        set_status(assignment_binding_diagnostic_text(diagnostic),
                   assignment_binding_diagnostic_text(diagnostic), false);
        return;
    }
    reload();
}

void MixedNozzleSidebarPanel::mark_dirty()
{
    m_editor_dirty = true;
    m_applied_signature.reset();
    update_status_and_enablement();
}

void MixedNozzleSidebarPanel::set_status(const wxString &text, const wxString &tooltip, bool enable_apply)
{
    const auto entry = std::find_if(m_wrapped_lines.begin(), m_wrapped_lines.end(),
                                    [this](const WrappedLine &item) { return item.line == m_status; });
    if (m_status != nullptr && entry != m_wrapped_lines.end() && entry->text != text) {
        set_wrapped_label(m_status, text);
        m_status->SetToolTip(tooltip);
        relayout();
        keep_actions_in_view();
    }
    m_apply->Enable(enable_apply);
}

void MixedNozzleSidebarPanel::set_ready_status(const wxString &ready, bool enable_apply)
{
    if (!m_editor_dirty && m_applied_signature && *m_applied_signature == m_signature)
        set_status(_L("Changes applied"), m_applied_tooltip, enable_apply);
    else
        set_status(ready, ready, enable_apply);
}

void MixedNozzleSidebarPanel::show_section(MixedNozzleSidebarSection section)
{
    m_section = section;
    const bool body = section == MixedNozzleSidebarSection::Body;
    const bool feature = section == MixedNozzleSidebarSection::Feature;
    const bool any = section != MixedNozzleSidebarSection::Hidden;
    // The inline editors sit behind "Edit here" in the More menu.
    const bool editor = (body || feature) && m_edit_here;
    m_body_panel->Show(body && m_edit_here);
    m_feature_panel->Show(feature && m_edit_here);
    m_status->Show(editor);
    m_apply->Show(editor);
    m_reload->Show(editor);
    // The lines decide their own visibility in refresh_readouts(); here they only follow the
    // section, so a hidden section never leaves one behind.
    for (wxWindow *line : std::initializer_list<wxWindow *>{m_mode_status, m_plate_line, m_guard_line,
             m_tower_status, m_sliced_line, m_coarse_line, m_tower_readout, m_rebind_line, m_rebind,
             m_time_decisions})
        if (line != nullptr && !any)
            line->Show(false);
    if (m_mode_status != nullptr && any)
        m_mode_status->Show(true);
    if (m_tower_readout != nullptr && any)
        m_tower_readout->Show(true);
    m_mode_setup->Show(any);
    m_more->Show(any);
    if (m_mode_off != nullptr)
        m_mode_off->Show(any);
    Show(any && !m_collapsed);
}

void MixedNozzleSidebarPanel::set_edit_here(bool edit_here)
{
    m_edit_here = edit_here;
    show_section(m_section);
    if (m_edit_here && m_section != MixedNozzleSidebarSection::ModeOnly &&
        m_section != MixedNozzleSidebarSection::Hidden)
        update_status_and_enablement();
    relayout();
    if (m_edit_here)
        keep_actions_in_view();
}

void MixedNozzleSidebarPanel::on_grouping()
{
    if (m_plater == nullptr)
        return;
    auto &plates = m_plater->get_partplate_list();
    const int current_index = plates.get_curr_plate_index();
    PartPlate *plate = plates.get_plate(current_index);
    if (!body_split_editor_target_valid(m_target, current_index, plate == nullptr ? 0 : plate->id().id)) {
        reject_changed_settings();
        return;
    }
    wxCommandEvent request;
    request.SetInt(0); // Prepare: the existing action invalidates the plate without slicing.
    m_plater->open_filament_map_setting_dialog(request);
    poll_refresh();
}

void MixedNozzleSidebarPanel::show_more_menu()
{
    const MixedNozzleSidebarMoreMenu labels;
    wxMenu menu;
    const int grouping = wxWindow::NewControlId();
    const int materials = wxWindow::NewControlId();
    const int edit_here = wxWindow::NewControlId();
    const int about = wxWindow::NewControlId();
    menu.Append(grouping, from_u8(labels.grouping));
    menu.Append(materials, from_u8(labels.update_materials));
    menu.Enable(materials, m_rebind_offered);
    menu.AppendCheckItem(edit_here, from_u8(labels.edit_here));
    menu.Check(edit_here, m_edit_here);
    menu.Enable(edit_here, m_section == MixedNozzleSidebarSection::Body ||
                               m_section == MixedNozzleSidebarSection::Feature);
    menu.AppendSeparator();
    menu.Append(about, from_u8(labels.about));
    const int chosen = TestMode::popup_menu(*this, menu, m_more->GetPosition() +
                                                     wxPoint(0, m_more->GetSize().GetHeight()));
    if (chosen == grouping)
        on_grouping();
    else if (chosen == materials)
        on_rebind();
    else if (chosen == edit_here)
        set_edit_here(!m_edit_here);
    else if (chosen == about && m_plater != nullptr)
        m_plater->show_mixed_nozzle_intro();
}

void MixedNozzleSidebarPanel::set_collapsed(bool collapsed)
{
    m_collapsed = collapsed;
    Show(m_section != MixedNozzleSidebarSection::Hidden && !m_collapsed);
}

MixedNozzleSidebarSignature MixedNozzleSidebarPanel::collect_signature() const
{
    if (m_plater == nullptr)
        return {};
    auto &plate_list = m_plater->get_partplate_list();
    const int current_index = plate_list.get_curr_plate_index();
    PartPlate *plate = plate_list.get_plate(current_index);

    MixedNozzleSidebarSignatureInput input;
    // The target is re-seeded from the live current plate, not frozen at construction: a plate
    // switch therefore shows up as a signature change and re-targets the editor for free.
    input.target = {current_index, plate == nullptr ? 0 : plate->id().id};
    input.current_plate_index = current_index;
    input.indexed_plate_id = plate == nullptr ? 0 : plate->id().id;
    if (plate != nullptr) {
        const auto &project = wxGetApp().preset_bundle->project_config;
        input.mode = effective_mixed_nozzle_mode(project, *plate);
        input.any_plate_on = mixed_nozzle_any_plate_on(project, plate_list.get_plate_list());
        const DynamicPrintConfig full_config = wxGetApp().preset_bundle->full_config();
        input.nozzle_count = configured_nozzle_count(full_config);
        input.filament_preset_count = wxGetApp().preset_bundle->filament_presets.size();
        input.base_layer_height = full_config.has("layer_height") ? full_config.opt_float("layer_height") : 0.;
        input.rows = mixed_nozzle_sidebar_row_signatures(plate->get_objects_on_this_plate());
        input.effective_inputs = mixed_nozzle_sidebar_config_signature(full_config);
        input.effective_inputs.push_back(std::to_string(plate->get_real_filament_map_mode(project)));
        for (int tool : plate->get_real_filament_maps(project))
            input.effective_inputs.push_back("tool:" + std::to_string(tool));
        const auto pair = plate->get_effective_feature_split_filaments(full_config, input.filament_preset_count);
        input.effective_inputs.push_back(pair ? std::to_string(pair->first) + "," + std::to_string(pair->second) : "<no pair>");
        // Preserve ownership too: an external local override must not be overwritten merely
        // because its current effective value happens to equal the inherited value.
        for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id", "internal_solid_filament_id",
                               "top_surface_filament_id", "bottom_surface_filament_id", "sparse_infill_filament_id",
                               "regional_grid_phase_rule", "mixed_nozzle_coarse_layer_height"}) {
            input.effective_inputs.push_back(project.has(key) ? project.opt_serialize(key) : "<inherit>");
            input.effective_inputs.push_back(plate->config()->has(key) ? plate->config()->opt_serialize(key) : "<inherit>");
        }
        auto &bundle = *wxGetApp().preset_bundle;
        const auto &process = bundle.prints.get_edited_preset().config;
        input.effective_inputs.push_back(process.has("mixed_nozzle_coarse_layer_height")
            ? process.opt_serialize("mixed_nozzle_coarse_layer_height") : "<unset>");
        input.effective_inputs.push_back(bundle.prints.get_edited_preset().name);
        input.effective_inputs.push_back(bundle.printers.get_edited_preset().name);
        const auto labels = filament_preset_labels(bundle);
        for (size_t i = 0; i < labels.size(); ++i) {
            input.effective_inputs.push_back(bundle.filament_presets[i]);
            input.effective_inputs.push_back(labels[i]);
            input.effective_inputs.push_back(filament_material_label(bundle, i));
        }
        // A wizard Apply writes the tower ledger and nothing else the loop above reads.
        const auto *tower_mask = full_config.option<ConfigOptionStrings>("different_settings_to_system");
        input.tower_inputs = mixed_nozzle_sidebar_tower_signature(bundle.project_config,
            tower_mask == nullptr ? std::vector<std::string>{} : tower_mask->values);
        // A finished slice changes no setting, so it needs its own input.
        if (plate->is_slice_result_valid()) {
            PrintBase *base = nullptr;
            GCodeResult *result = nullptr;
            plate->get_print(&base, &result, nullptr);
            const auto *slice = plate->get_slice_result();
            input.slice_revision = mixed_nozzle_sidebar_slice_revision(true, current_index,
                result == nullptr ? 0 : result->moves.size(),
                slice == nullptr ? 0. : double(slice->print_statistics.modes[0].time));
        }
    }
    return mixed_nozzle_sidebar_signature(input);
}

void MixedNozzleSidebarPanel::reload()
{
    m_editor_dirty = false;
    m_signature = collect_signature();
    rebuild();
}

void MixedNozzleSidebarPanel::reject_changed_settings()
{
    m_body_panel->Enable(false);
    m_feature_panel->Enable(false);
    wxString status = _L("Settings changed while the editor was open. Click Reload.");
    if (m_section == MixedNozzleSidebarSection::Feature && m_plater != nullptr) {
        auto &plates = m_plater->get_partplate_list();
        const int index = plates.get_curr_plate_index();
        PartPlate *plate = plates.get_plate(index);
        if (body_split_editor_target_valid(m_target, index, plate == nullptr ? 0 : plate->id().id)) {
            auto &bundle = *wxGetApp().preset_bundle;
            const auto applied = mixed_nozzle_sidebar_applied_feature_state(
                bundle.project_config, bundle.full_config(), *plate, bundle.filament_presets.size());
            if (applied.effective_pair && !applied.can_apply)
                status = from_u8(split_engine_message(applied.full_status).text) + " " + _L("Click Reload to update the editor.");
        }
    }
    set_status(status, status + " " +
               _L("No settings were changed. Reload discards unapplied edits and loads current settings."), false);
}

void MixedNozzleSidebarPanel::poll_refresh()
{
    const auto signature = collect_signature();
    if (signature == m_signature)
        return;
    // A finished slice or a tower profile write changes what the readouts say, not what the
    // editor is editing: refresh the lines and leave the draft alone.
    if (mixed_nozzle_sidebar_readout_only_change(m_signature, signature)) {
        m_signature = signature;
        refresh_readouts();
        return;
    }
    // Context changes discard the old editor immediately. Other competing changes preserve
    // the draft, but disable it until Reload; Apply independently checks the same baseline.
    if (mixed_nozzle_sidebar_preserve_draft(m_signature, signature, m_editor_dirty)) {
        reject_changed_settings();
        return;
    }
    m_editor_dirty = false;
    m_signature = signature;
    rebuild();
}

void MixedNozzleSidebarPanel::rebuild()
{
    if (m_plater == nullptr) {
        show_section(MixedNozzleSidebarSection::Hidden);
        return;
    }
    auto &plate_list = m_plater->get_partplate_list();
    const int current_index = plate_list.get_curr_plate_index();
    PartPlate *plate = plate_list.get_plate(current_index);
    m_target = {current_index, plate == nullptr ? 0 : plate->id().id};

    if (plate == nullptr) {
        show_section(MixedNozzleSidebarSection::Hidden);
        return;
    }
    auto &bundle = *wxGetApp().preset_bundle;
    const DynamicPrintConfig full_config = bundle.full_config();
    const bool topology_supported =
        mixed_nozzle_configured_topology_supported(configured_nozzle_count(full_config));
    const auto source = effective_mixed_nozzle_mode(bundle.project_config, *plate);
    // Hidden while every plate is off; the Printer row is the one place to start.
    const auto section = mixed_nozzle_sidebar_section(source.effective, topology_supported,
        mixed_nozzle_any_plate_on(bundle.project_config, plate_list.get_plate_list()));
    show_section(section);
    m_body_panel->Enable(true);
    m_feature_panel->Enable(true);
    if (section == MixedNozzleSidebarSection::Hidden) {
        GetParent()->Layout();
        return;
    }

    const auto ledger = mixed_nozzle_read_ledger(bundle.project_config, bundle.filament_presets.size());
    bool needs_rebind = false;
    for (size_t i = 0; i < bundle.filament_presets.size(); ++i)
        needs_rebind = needs_rebind || ledger.is_legacy(i);
    const auto *pair = bundle.project_config.option<ConfigOptionFloats>("nozzle_diameter");
    m_rebind_offered = needs_rebind && pair && pair->values.size() == 2 && pair->values[0] != pair->values[1];

    if (section == MixedNozzleSidebarSection::Body)
        rebuild_body_cards();
    else if (section == MixedNozzleSidebarSection::Feature)
        rebuild_feature_rows();
    // After the editors, so the Body Split summary can count fine and coarse parts.
    refresh_readouts();

    if (section == MixedNozzleSidebarSection::ModeOnly)
        set_status(wxEmptyString, wxEmptyString, false);
    else
        update_status_and_enablement();

    GetParent()->Layout();
    relayout();
}

void MixedNozzleSidebarPanel::rebuild_body_cards()
{
    auto &bundle = *wxGetApp().preset_bundle;
    auto &plate_list = m_plater->get_partplate_list();
    PartPlate *plate = plate_list.get_plate(m_target.plate_index);
    if (plate == nullptr)
        return;

    // Each slot's material settings for the nozzle this plate puts it on.
    const DynamicPrintConfig full_config = wizard_plate_effective_config(bundle, *plate);
    m_body_resolver = mixed_nozzle_resolver_config_from_full(full_config);
    m_body_resolver.filament_map_mode.value = plate->get_real_filament_map_mode(bundle.project_config);
    m_body_resolver.filament_map.values = plate->get_real_filament_maps(bundle.project_config);

    const auto preset_labels = filament_preset_labels(bundle);
    BodySplitPresentation presentation;
    for (size_t i = 0; i < preset_labels.size(); ++i)
        presentation.filaments.push_back({int(i + 1), preset_labels[i], filament_material_label(bundle, i)});
    const double inherited_height = full_config.has("layer_height") ? full_config.opt_float("layer_height") : 0.;
    m_body_model = build_body_split_editor_model(plate->get_objects_on_this_plate(), m_body_resolver,
                                                 presentation, inherited_height);

    m_body_sizer->Clear(true);
    m_body_rows.clear();
    // 34 chars is the Plate Settings bound; the sidebar card is narrower, so the identity is
    // bounded harder and the complete name stays in the tooltip and the accessible name.
    const auto identities = body_split_row_identities(m_body_model.rows, 28);
    for (size_t row_index = 0; row_index < m_body_model.rows.size(); ++row_index) {
        const BodySplitEditorRow &row = m_body_model.rows[row_index];
        BodyRowControls controls;

        controls.card = new wxPanel(m_body_panel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL);
        controls.card->SetBackgroundColour(*wxWHITE);
        auto *card_sizer = new wxBoxSizer(wxVERTICAL);

        const auto &identity = identities[row_index];
        controls.identity = new wxStaticText(controls.card, wxID_ANY, from_u8(identity.display),
                                             wxDefaultPosition, wxDefaultSize, wxST_ELLIPSIZE_END);
        controls.identity->SetFont(Label::Head_13);
        controls.identity->SetToolTip(from_u8(identity.tooltip));
        // The visible label is bounded, so the accessible name carries the complete identity.
        controls.identity->SetName(from_u8(identity.tooltip));
        card_sizer->Add(controls.identity, 0, wxEXPAND);

        controls.filament = new ComboBox(controls.card, wxID_ANY, wxEmptyString, wxDefaultPosition,
                                         wxDefaultSize, 0, nullptr, wxCB_READONLY);
        for (size_t i = 0; i < preset_labels.size(); ++i)
            controls.filament->Append(from_u8(body_split_filament_label(i + 1, preset_labels[i])));
        controls.filament->SetSelection(
            row.logical_filament > 0 && size_t(row.logical_filament) <= preset_labels.size()
                ? row.logical_filament - 1 : wxNOT_FOUND);
        card_sizer->Add(controls.filament, 0, wxEXPAND | wxTOP, FromDIP(2));

        // Material and physical tool are one wrapped caption line rather than two fixed columns:
        // at sidebar width there is no room for the Plate Settings 90 + 170 DIP pair.
        controls.caption = new wxStaticText(controls.card, wxID_ANY, wxEmptyString);
        controls.caption->SetFont(Label::Body_12);
        card_sizer->Add(controls.caption, 0, wxEXPAND | wxTOP, FromDIP(2));

        controls.cadence = new ComboBox(controls.card, wxID_ANY, wxEmptyString, wxDefaultPosition,
                                        wxDefaultSize, 0, nullptr, wxCB_READONLY);
        for (size_t choice = 0; choice < row.cadence_choices.size(); ++choice) {
            const double height = row.cadence_choices[choice];
            const int ratio = row.base_layer_height > 0. ? int(std::lround(height / row.base_layer_height)) : 0;
            controls.cadence->Append(from_u8(mixed_nozzle_sidebar_layer_choice_label(height, ratio == 1)));
            if ((!row.regional_height_explicit && ratio == 1) ||
                std::abs(row.effective_layer_height - height) <= 1e-8)
                controls.cadence->SetSelection(int(choice));
        }
        card_sizer->Add(controls.cadence, 0, wxEXPAND | wxTOP, FromDIP(2));

        auto *skin_row = new wxBoxSizer(wxHORIZONTAL);
        controls.fine_skins = new ::CheckBox(controls.card, wxID_ANY);
        controls.fine_skins->SetValue(row.fine_skins);
        controls.fine_skins->Enable(row.fine_skin_controls_applicable);
        controls.fine_skin_label = new wxStaticText(controls.card, wxID_ANY, _L("Fine skins"));
        controls.fine_skin_label->SetFont(Label::Body_12);
        controls.fine_skin_layers = new SpinInput(controls.card, wxEmptyString, wxEmptyString,
                                                  wxDefaultPosition, wxSize(FromDIP(70), -1),
                                                  wxSP_ARROW_KEYS, 1, 999, std::max(1, row.fine_skin_layers));
        controls.fine_skin_layers->Enable(row.fine_skin_controls_applicable);
        skin_row->Add(controls.fine_skins, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, FromDIP(4));
        skin_row->Add(controls.fine_skin_label, 1, wxALIGN_CENTER_VERTICAL | wxRIGHT, FromDIP(4));
        skin_row->Add(controls.fine_skin_layers, 0, wxALIGN_CENTER_VERTICAL);
        card_sizer->Add(skin_row, 0, wxEXPAND | wxTOP, FromDIP(2));

        controls.filament->Bind(wxEVT_COMBOBOX, [this](wxCommandEvent &) { mark_dirty(); });
        controls.cadence->Bind(wxEVT_COMBOBOX, [this](wxCommandEvent &) { mark_dirty(); });
        controls.fine_skins->Bind(wxEVT_TOGGLEBUTTON, [this](wxCommandEvent &event) {
            mark_dirty();
            // Let CheckBox run its own deferred bitmap update after the native toggle
            // completes; consuming this event leaves the old checkmark.
            event.Skip();
        });
        controls.fine_skin_layers->Bind(wxEVT_SPINCTRL, [this](wxCommandEvent &) { mark_dirty(); });
        controls.fine_skin_layers->Bind(wxEVT_TEXT_ENTER, [this](wxCommandEvent &) { mark_dirty(); });

        controls.card->SetSizer(card_sizer);
        m_body_sizer->Add(controls.card, 0, wxEXPAND | wxBOTTOM, FromDIP(8));
        m_body_rows.push_back(controls);
    }

    if (!m_body_model.excluded.empty()) {
        auto *excluded = new wxStaticText(m_body_panel, wxID_ANY,
            wxString::Format(_L("Modifiers, negative parts and supports not listed: %zu"),
                             m_body_model.excluded.size()),
            wxDefaultPosition, wxDefaultSize, wxST_ELLIPSIZE_END);
        excluded->SetFont(Label::Body_12);
        excluded->SetToolTip(_L("These follow the part they belong to and have no nozzle of their own here."));
        m_body_sizer->Add(excluded, 0, wxEXPAND);
    }
    // The main window's dark mode pass ran before these cards existed. Without it they keep
    // their white background under the dark theme's light text.
    for (wxWindow *child : m_body_panel->GetChildren())
        wxGetApp().UpdateDarkUIWin(child);
    m_body_sizer->Layout();
}

void MixedNozzleSidebarPanel::rebuild_feature_rows()
{
    auto &bundle = *wxGetApp().preset_bundle;
    auto &plate_list = m_plater->get_partplate_list();
    PartPlate *plate = plate_list.get_plate(m_target.plate_index);
    if (plate == nullptr)
        return;

    const auto preset_labels = filament_preset_labels(bundle);
    const DynamicPrintConfig full_config = bundle.full_config();
    const auto applied_state = mixed_nozzle_sidebar_applied_feature_state(
        bundle.project_config, full_config, *plate, preset_labels.size());
    const auto seed = build_feature_split_native_seed(preset_labels, applied_state.effective_pair);
    m_feature_fine->Clear();
    m_feature_coarse->Clear();
    for (const std::string &label : seed.labels) {
        m_feature_fine->Append(from_u8(label));
        m_feature_coarse->Append(from_u8(label));
    }
    // A signature-gated rebuild only happens when the live state actually moved, so re-seeding
    // from live state here is correct: there is no unapplied selection to preserve (poll_refresh
    // defers a rebuild while the user is mid-edit).
    m_feature_selection = seed.selection;
    m_feature_fine->SetSelection(m_feature_selection.fine_selection);
    m_feature_coarse->SetSelection(m_feature_selection.coarse_selection);

    rebuild_feature_cadences(full_config, false);
    m_feature_panel->GetSizer()->Layout();
}

void MixedNozzleSidebarPanel::rebuild_feature_cadences(const DynamicPrintConfig &full_config, bool preserve_draft)
{
    auto &bundle = *wxGetApp().preset_bundle;
    auto &plates = m_plater->get_partplate_list();
    PartPlate *plate = plates.get_plate(m_target.plate_index);
    if (plate == nullptr)
        return;
    double coarse_height = full_config.has("mixed_nozzle_coarse_layer_height")
        ? full_config.opt_float("mixed_nozzle_coarse_layer_height") : 0.;
    const int previous = m_feature_cadence->GetSelection();
    if (preserve_draft && previous >= 0 && size_t(previous) < m_feature_cadence_values.size())
        coarse_height = m_feature_cadence_values[size_t(previous)];
    m_feature_cadence_values.clear();
    m_feature_cadence->Clear();
    const auto pair = feature_split_pair_from_selections(m_feature_selection, bundle.filament_presets.size());
    if (!pair) {
        m_feature_cadence->Append(_L("Choose both materials first"), wxNullBitmap, DD_ITEM_STYLE_DISABLED);
        m_feature_cadence->SetSelection(0);
        m_feature_cadence->Enable(false);
        m_feature_cadence->SetToolTip(_L("Pick a fine material and a different coarse material to list the coarse layers."));
        return;
    }
    const double base_height = full_config.has("layer_height") ? full_config.opt_float("layer_height") : 0.;
    PrintConfig resolver = feature_split_resolver_config_from_full(full_config);
    resolver.filament_map_mode.value = plate->get_real_filament_map_mode(bundle.project_config);
    resolver.filament_map.values = plate->get_real_filament_maps(bundle.project_config);
    m_feature_cadence_values = feature_split_coarse_cadence_choices(base_height, resolver);
    int selected = wxNOT_FOUND;
    for (size_t i = 0; i < m_feature_cadence_values.size(); ++i) {
        const double height = m_feature_cadence_values[i];
        m_feature_cadence->Append(from_u8(mixed_nozzle_sidebar_layer_choice_label(height, false)));
        if (std::abs(height - coarse_height) <= 1e-6)
            selected = int(i);
    }
    if (m_feature_cadence_values.empty()) {
        m_feature_cadence->Append(_L("No coarse layer works here"), wxNullBitmap, DD_ITEM_STYLE_DISABLED);
        selected = 0;
    }
    m_feature_cadence->SetSelection(selected);
    if (selected == wxNOT_FOUND)
        m_feature_cadence->SetTextLabel(_L("Select a coarse layer"));
    m_feature_cadence->Enable(!m_feature_cadence_values.empty());
    m_feature_cadence->SetToolTip(_L("The coarse layer is shared by every plate. Apply saves it and both materials in one Undo step."));
}

void MixedNozzleSidebarPanel::update_status_and_enablement()
{
    if (collect_signature() != m_signature) {
        reject_changed_settings();
        return;
    }
    if (m_section == MixedNozzleSidebarSection::Body) {
        const auto &bundle = *wxGetApp().preset_bundle;
        std::vector<BodySplitNativeSelection> selections;
        for (size_t i = 0; i < m_body_model.rows.size() && i < m_body_rows.size(); ++i) {
            const int logical = m_body_rows[i].filament->GetSelection();
            selections.push_back({m_body_model.rows[i].object_id, m_body_model.rows[i].volume_id,
                                  logical, m_body_rows[i].cadence->GetSelection()});
            wxString tool_text = _L("Not on a nozzle yet");
            std::string material;
            if (logical >= 0 && size_t(logical) < wxGetApp().preset_bundle->filament_presets.size()) {
                material = filament_material_label(bundle, size_t(logical));
                const auto resolution = resolve_mixed_nozzle_tool(m_body_resolver, size_t(logical),
                                                                  MixedNozzleResolveScope::PhysicalToolOnly);
                // The nozzle by side and size, never a tool code.
                if (resolution.tool)
                    tool_text = from_u8(std::string(resolution.tool->physical_extruder == 0 ? "left " : "right ") +
                                        mixed_nozzle_diameter_text(resolution.tool->nozzle_diameter) + " mm nozzle");
                else if (resolution.diagnostic)
                    tool_text = from_u8(resolution.diagnostic->message());
            }
            const wxString caption = material.empty() ? tool_text
                                                      : from_u8(material) + " · " + tool_text;
            m_body_rows[i].caption->SetLabel(caption);
            m_body_rows[i].caption->SetToolTip(caption);
            m_body_rows[i].caption->Wrap(m_body_rows[i].card->GetSize().GetWidth());
        }
        // A caption can wrap to another row.
        relayout();
        const auto diagnostic = body_split_native_selection_diagnostic(m_body_model.rows, selections,
            wxGetApp().preset_bundle->filament_presets.size(), m_body_resolver);
        const wxString status = body_diagnostic_text(diagnostic, m_body_model.rows.size());
        if (diagnostic == BodySplitEditorDiagnostic::None)
            set_ready_status(status, true);
        else
            set_status(status, status, false);
        return;
    }
    if (m_section == MixedNozzleSidebarSection::Feature) {
        auto &bundle = *wxGetApp().preset_bundle;
        auto &plate_list = m_plater->get_partplate_list();
        PartPlate *plate = plate_list.get_plate(m_target.plate_index);
        if (plate == nullptr) {
            set_status(_L("The plate changed or is unavailable"),
                       _L("Select the plate again to edit its mixed-nozzle settings."), false);
            return;
        }
        const DynamicPrintConfig full_config = wizard_plate_effective_config(bundle, *plate);
        PrintConfig resolver = mixed_nozzle_resolver_config_from_full(full_config);
        resolver.filament_map_mode.value = plate->get_real_filament_map_mode(bundle.project_config);
        resolver.filament_map.values = plate->get_real_filament_maps(bundle.project_config);
        const auto pair = feature_split_pair_from_selections(m_feature_selection,
                                                             bundle.filament_presets.size());
        const auto mode = effective_mixed_nozzle_mode(bundle.project_config, *plate).effective;
        const auto state = build_feature_split_editor_state(mode, configured_nozzle_count(full_config),
            pair, bundle.filament_presets.size(), resolver);
        const int cadence_index = m_feature_cadence->GetSelection();
        const bool cadence_selected = cadence_index >= 0 && size_t(cadence_index) < m_feature_cadence_values.size();
        const wxString status = !pair
            ? _L("Pick a fine material and a different coarse material to list the coarse layers.")
            : state.can_apply && !cadence_selected ? _L("Select a coarse layer") : from_u8(split_engine_message(state.full_status).text);
        if (pair && state.can_apply && cadence_selected)
            set_ready_status(status, true);
        else
            set_status(status, status, false);
    }
}

void MixedNozzleSidebarPanel::on_apply()
{
    if (m_plater == nullptr)
        return;
    const auto route = mixed_nozzle_sidebar_commit_route(m_signature, collect_signature());
    bool applied = false;
    wxString tooltip;
    m_applied_signature.reset();
    switch (route) {
    case MixedNozzleSidebarCommitRoute::Body:
        applied = apply_body(tooltip);
        break;
    case MixedNozzleSidebarCommitRoute::Feature:
        applied = apply_feature(tooltip);
        break;
    case MixedNozzleSidebarCommitRoute::None:
        reject_changed_settings();
        return;
    }
    if (!applied)
        return;
    // reload() rebuilds the status from the new state, so it is told what was just applied.
    m_applied_signature = collect_signature();
    m_applied_tooltip = tooltip;
    reload();
}

bool MixedNozzleSidebarPanel::review_binding(PresetBundle &bundle, PartPlate &plate,
                                             MixedNozzleRebindReviewResult &review,
                                             std::optional<PresetBundle> &reviewed_bundle,
                                             const wxString &unchanged, const wxString &stale)
{
    review = review_mixed_nozzle_rebind(this, bundle, plate);
    if (review.status == MixedNozzleRebindReviewStatus::Cancelled) {
        set_status(_L("Nothing was changed."), unchanged, false);
        return false;
    }
    if (review.status == MixedNozzleRebindReviewStatus::Stale) {
        set_status(_L("Material settings changed while the review was open"), stale, false);
        return false;
    }
    // The live bundle stays untouched until the shared transaction commits; unchecked rows stay
    // preserved by the native rebind writer.
    if (review.plan) {
        reviewed_bundle.emplace(bundle);
        if (!mixed_nozzle_rebind_write(reviewed_bundle->project_config, *review.plan, review.accepted_rebind)) {
            set_status(_L("The material settings could not be applied. Click Reload."), unchanged, false);
            return false;
        }
    }
    return true;
}

bool MixedNozzleSidebarPanel::apply_body(wxString &tooltip)
{
    auto &bundle = *wxGetApp().preset_bundle;
    auto &plate_list = m_plater->get_partplate_list();
    PartPlate *plate = plate_list.get_plate(m_target.plate_index);
    if (plate == nullptr)
        return false;

    MixedNozzleRebindReviewResult binding_review;
    std::optional<PresetBundle> reviewed_bundle;
    if (!review_binding(bundle, *plate, binding_review, reviewed_bundle,
                        _L("Nothing was changed."),
                        _L("Nothing was changed. Click Reload to load the current materials.")))
        return false;
    PresetBundle &validation_bundle = reviewed_bundle ? *reviewed_bundle : bundle;
    const auto validation_maps = plate->get_real_filament_maps(validation_bundle.project_config);
    const auto validation_volumes = plate->get_real_filament_volume_maps(validation_bundle.project_config);
    const DynamicPrintConfig full_config = validation_bundle.full_config(true, validation_maps, validation_volumes);
    PrintConfig resolver = mixed_nozzle_resolver_config_from_full(full_config);
    resolver.filament_map_mode.value = plate->get_real_filament_map_mode(validation_bundle.project_config);
    resolver.filament_map.values = validation_maps;
    const double inherited_height = full_config.has("layer_height") ? full_config.opt_float("layer_height") : 0.;

    // on_apply checked the captured baseline before this route. Rebuild for the model's live
    // validation as well, so indices are never trusted across a configuration change.
    const BodySplitEditorModel live = build_body_split_editor_model(plate->get_objects_on_this_plate(),
                                                                    resolver, {}, inherited_height);
    if (live.rows.size() != m_body_model.rows.size()) {
        set_status(_L("The parts changed while you were editing. Click Reload."),
                   _L("Nothing was changed. Reload drops your edits and loads the current parts."), false);
        return false;
    }

    std::vector<BodySplitNativeSelection> selections;
    for (size_t i = 0; i < m_body_model.rows.size() && i < m_body_rows.size(); ++i)
        selections.push_back({m_body_model.rows[i].object_id, m_body_model.rows[i].volume_id,
                              m_body_rows[i].filament->GetSelection(),
                              m_body_rows[i].cadence->GetSelection()});
    // The sidebar has no phase control; the plate's stored rule is preserved rather than reset.
    const auto *phase_option = plate->config()->option<ConfigOptionEnum<RegionalGridPhaseRule>>("regional_grid_phase_rule");
    const RegionalGridPhaseRule phase = phase_option != nullptr ? phase_option->value
        : (full_config.has("regional_grid_phase_rule")
               ? full_config.option<ConfigOptionEnum<RegionalGridPhaseRule>>("regional_grid_phase_rule")->value
               : RegionalGridPhaseRule::KeepNominal);
    const auto live_request = build_body_split_live_request(live.rows, selections,
        bundle.filament_presets.size(), resolver, phase);
    if (!live_request.request) {
        set_status(_L("Body Split settings changed while you were editing. Click Reload."),
                   _L("No settings were changed. Reload discards unapplied edits and loads the current parts, layers and nozzles."), false);
        return false;
    }
    BodySplitApplyRequest request = *live_request.request;
    request.inherited_layer_height = inherited_height;

    merge_body_split_fine_skins(request, m_body_model.rows, m_body_rows.size(),
        [this](size_t i) { return m_body_rows[i].fine_skins->GetValue(); },
        [this](size_t i) { return m_body_rows[i].fine_skin_layers->GetValue(); });

    std::vector<size_t> affected_indices;
    for (ModelObject *object : plate->get_objects_on_this_plate())
        for (size_t index = 0; index < m_plater->model().objects.size(); ++index)
            if (m_plater->model().objects[index]->id() == object->id())
                affected_indices.push_back(index);

    // Stage the native validator and volume edits on copied owners. Publication then reacquires
    // the live volume IDs and applies the same native ModelConfig values inside one Undo step.
    BodySplitEditorDiagnostic staged_body_diagnostic = BodySplitEditorDiagnostic::None;
    MixedNozzleAssignmentBindingRequest transaction_request;
    transaction_request.rebind_plan = binding_review.plan;
    transaction_request.accepted_rebind = binding_review.accepted_rebind;
    if (transaction_request.rebind_plan)
        transaction_request.rebind_action = MixedNozzleRebindAction::Apply;
    transaction_request.stage_assignment = make_body_split_stage_assignment(request, staged_body_diagnostic);
    auto transaction_plan = stage_mixed_nozzle_assignment_binding(
        bundle, m_plater->model(), *plate, transaction_request);
    Plater *plater = m_plater;
    MixedNozzleAssignmentBindingDiagnostic diagnostic = MixedNozzleAssignmentBindingDiagnostic::None;
    const bool applied = commit_mixed_nozzle_assignment_binding(
        bundle, m_plater->model(), plate_list.get_plate_list(), transaction_plan,
        {{},
         {},
         [plater, plate, affected_indices] {
             plater->get_partplate_list().set_default_wipe_tower_pos_for_plate(plate->get_index(), false, true);
             plate->update_slice_result_valid_state(false);
             plater->changed_objects(affected_indices);
             plater->update_project_dirty_from_presets();
             plater->set_plater_dirty(true);
             // Each part's filament cell is read again from the parts, as setup's Apply does, so the
             // list shows the filament the G-code uses.
             if (wxGetApp().obj_list() != nullptr)
                 wxGetApp().obj_list()->update_objects_list_filament_column(
                     std::max<size_t>(1, wxGetApp().preset_bundle->filament_presets.size()));
         },
         [plater](const MixedNozzleSetupPlan &plan) {
             return plater->take_mixed_nozzle_setup_snapshot("Edit mixed-nozzle settings", plan);
         }}, &diagnostic);
    if (!applied) {
        const wxString status = diagnostic == MixedNozzleAssignmentBindingDiagnostic::AssignmentRejected
            ? body_diagnostic_text(staged_body_diagnostic, m_body_model.rows.size())
            : assignment_binding_diagnostic_text(diagnostic);
        set_status(status, status, false);
        return false;
    }
    tooltip = _L("One Undo step. Undo restores the previous settings.");
    return true;
}

bool MixedNozzleSidebarPanel::apply_feature(wxString &tooltip)
{
    auto &bundle = *wxGetApp().preset_bundle;
    auto &plate_list = m_plater->get_partplate_list();
    const int current_index = plate_list.get_curr_plate_index();
    PartPlate *plate = plate_list.get_plate(current_index);
    if (plate == nullptr)
        return false;

    MixedNozzleRebindReviewResult binding_review;
    std::optional<PresetBundle> reviewed_bundle;
    if (!review_binding(bundle, *plate, binding_review, reviewed_bundle,
                        _L("No Feature Split assignment settings were changed."),
                        _L("No Feature Split assignment settings were changed. Reload and review the current material values.")))
        return false;
    PresetBundle &validation_bundle = reviewed_bundle ? *reviewed_bundle : bundle;
    const auto validation_maps = plate->get_real_filament_maps(validation_bundle.project_config);
    const auto validation_volumes = plate->get_real_filament_volume_maps(validation_bundle.project_config);
    const DynamicPrintConfig full_config = validation_bundle.full_config(true, validation_maps, validation_volumes);
    PrintConfig resolver = feature_split_resolver_config_from_full(full_config);
    resolver.filament_map_mode.value = plate->get_real_filament_map_mode(validation_bundle.project_config);
    resolver.filament_map.values = validation_maps;
    const size_t nozzle_count = configured_nozzle_count(full_config);
    const auto effective_mode = effective_mixed_nozzle_mode(bundle.project_config, *plate).effective;

    auto live = build_feature_split_native_live_apply({m_target.plate_index, m_target.plate_id},
        m_feature_selection, current_index, plate->id().id, effective_mode, nozzle_count,
        bundle.filament_presets.size(), resolver);
    if (!live.request) {
        const wxString status = from_u8(split_engine_message(live.state.full_status).text);
        set_status(status, status, live.state.can_apply);
        return false;
    }

    const int cadence_selection = m_feature_cadence->GetSelection();
    if (cadence_selection < 0 || size_t(cadence_selection) >= m_feature_cadence_values.size()) {
        set_status(_L("Select a coarse layer"), wxEmptyString, false);
        return false;
    }
    live.request->coarse_layer_height = m_feature_cadence_values[size_t(cadence_selection)];

    FeatureSplitApplyRequest feature_request = *live.request;
    feature_request.coarse_layer_height = m_feature_cadence_values[size_t(cadence_selection)];
    MixedNozzleAssignmentBindingRequest transaction_request;
    transaction_request.rebind_plan = binding_review.plan;
    transaction_request.accepted_rebind = binding_review.accepted_rebind;
    if (transaction_request.rebind_plan)
        transaction_request.rebind_action = MixedNozzleRebindAction::Apply;
    transaction_request.stage_assignment =
        make_feature_split_stage_assignment(feature_request, effective_mode, nozzle_count);
    auto transaction_plan = stage_mixed_nozzle_assignment_binding(
        bundle, m_plater->model(), *plate, transaction_request);
    Plater *plater = m_plater;
    MixedNozzleAssignmentBindingDiagnostic diagnostic = MixedNozzleAssignmentBindingDiagnostic::None;
    const bool applied = commit_mixed_nozzle_assignment_binding(
        bundle, m_plater->model(), plate_list.get_plate_list(), transaction_plan,
        {{},
         [plater](std::optional<double> before, std::optional<double> after) {
             plater->record_feature_split_cadence_transition(before, after);
         },
         [plater, plate] {
             auto &plates = plater->get_partplate_list();
             plates.set_default_wipe_tower_pos_for_plate(plate->get_index(), false, true);
             if (Tab *print_tab = wxGetApp().get_tab(Preset::TYPE_PRINT); print_tab != nullptr) {
                 print_tab->update_dirty();
                 print_tab->reload_config();
             }
             plater->update_project_dirty_from_presets();
             plater->set_plater_dirty(true);
             notify_mixed_nozzle_commit([plater] {
                 plater->on_config_change(wxGetApp().preset_bundle->full_config());
             });
         },
         [plater](const MixedNozzleSetupPlan &plan) {
             return plater->take_mixed_nozzle_setup_snapshot("Edit mixed-nozzle settings", plan);
         }}, &diagnostic);
    if (!applied) {
        const wxString status = diagnostic == MixedNozzleAssignmentBindingDiagnostic::AssignmentRejected
            ? _L("This change can't be applied. Check the fine and coarse materials.")
            : assignment_binding_diagnostic_text(diagnostic);
        set_status(status, status, false);
        return false;
    }
    tooltip = _L("One Undo step. Undo restores both materials and the coarse layer.");
    return true;
}

void MixedNozzleSidebarPanel::msw_rescale()
{
    if (m_mode_setup != nullptr)
        m_mode_setup->Rescale();
    if (m_more != nullptr)
        m_more->Rescale();
    if (m_mode_off != nullptr)
        m_mode_off->Rescale();
    if (m_apply != nullptr)
        m_apply->Rescale();
    if (m_reload != nullptr)
        m_reload->Rescale();
    if (m_feature_cadence != nullptr)
        m_feature_cadence->Rescale();
    for (const BodyRowControls &controls : m_body_rows) {
        if (controls.filament != nullptr)
            controls.filament->Rescale();
        if (controls.cadence != nullptr)
            controls.cadence->Rescale();
        if (controls.fine_skin_layers != nullptr)
            controls.fine_skin_layers->Rescale();
    }
    if (m_feature_fine != nullptr)
        m_feature_fine->Rescale();
    if (m_feature_coarse != nullptr)
        m_feature_coarse->Rescale();
    relayout();
}

void MixedNozzleSidebarPanel::sys_color_changed()
{
    // A theme change is not a model change, so the signature would not move on its own. Force one
    // rebuild so every re-created child picks the new colours up.
    m_signature = mixed_nozzle_sidebar_stale_signature();
    Refresh();
}

}} // namespace Slic3r::GUI
