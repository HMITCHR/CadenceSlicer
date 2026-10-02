#include "PlateSettingsDialog.hpp"
#include "TestMode.hpp"
#include "MsgDialog.hpp"
#include "Widgets/DialogButtons.hpp"
#include "MixedNozzleNativeEntry.hpp"
#include "MixedNozzleSetupDialog.hpp"
#include "MixedNozzleSidebarController.hpp"
#include "ValidationActionRouting.hpp"

#include <wx/display.h>

namespace Slic3r { namespace GUI {
static constexpr int MIN_LAYER_VALUE = 2;
static constexpr int MAX_LAYER_VALUE = INT_MAX - 1;

wxDEFINE_EVENT(EVT_SET_BED_TYPE_CONFIRM, wxCommandEvent);
wxDEFINE_EVENT(EVT_NEED_RESORT_LAYERS, wxCommandEvent);

namespace {

// Physical tool count the Mixed-Nozzle surfaces check. mixed_nozzle_effective_nozzle_diameters()
// is the same overlay the sidebar and the printer tab read, so a project-owned dissimilar pair and
// a plain printer preset both report their real tool count here.
std::size_t plate_settings_configured_nozzle_count()
{
    PresetBundle* bundle = wxGetApp().preset_bundle;
    if (bundle == nullptr)
        return 0;
    const auto* nozzles = mixed_nozzle_effective_nozzle_diameters(
        bundle->project_config, bundle->printers.get_edited_preset().config);
    return nozzles == nullptr ? 0 : nozzles->values.size();
}

} // namespace

bool dispatch_plate_settings_acceptance(wxEvtHandler& layer_sequence_handler,
                                        wxEvtHandler& dialog_handler,
                                        int event_id)
{
    wxCommandEvent event(EVT_SET_BED_TYPE_CONFIRM, event_id);
    layer_sequence_handler.ProcessEvent(event);
    dialog_handler.ProcessEvent(event);
    return event.GetString() != "Invalid";
}

bool LayerSeqInfo::operator<(const LayerSeqInfo& another) const
{
    if (this->begin_layer_number < MIN_LAYER_VALUE)
        return false;
    if (another.begin_layer_number < MIN_LAYER_VALUE)
        return true;
    if (this->begin_layer_number == another.begin_layer_number) {
        if (this->end_layer_number < MIN_LAYER_VALUE)
            return false;
        if (another.end_layer_number < MIN_LAYER_VALUE)
            return true;
        return this->end_layer_number < another.end_layer_number;
    }
    return this->begin_layer_number < another.begin_layer_number;
}

LayerNumberTextInput::LayerNumberTextInput(wxWindow* parent, int layer_number, wxSize size, Type type, ValueType value_type)
    :ComboBox(parent, wxID_ANY, wxEmptyString, wxDefaultPosition, size, 0, NULL)
    , m_layer_number(layer_number)
    , m_type(type)
    , m_value_type(value_type)
{
    GetTextCtrl()->SetValidator(wxTextValidator(wxFILTER_DIGITS));
    GetTextCtrl()->SetFont(::Label::Body_14);
    Append(_L_CONTEXT("End", "Layer range"));
    Append(_L("Customize"));
    if (m_value_type == ValueType::End)
        SetSelection(0);
    if (m_value_type == ValueType::Custom) {
        SetSelection(1);
        update_label();
    }

    Bind(wxEVT_TEXT, [this](auto& evt) {
            if (m_value_type == ValueType::End) {
                // TextCtrl->SetValue() will generate a wxEVT_TEXT event
                GetTextCtrl()->ChangeValue(_L_CONTEXT("End", "Layer range"));
                return;
            }
            evt.Skip();
        });

    auto validate_input_value = [this](int gui_value) {
        // value should not be less than MIN_LAYER_VALUE, and should not be greater than MAX_LAYER_VALUE
        gui_value = std::clamp(gui_value, MIN_LAYER_VALUE, MAX_LAYER_VALUE);

        int begin_value = 0;
        int end_value = 0;
        LayerNumberTextInput* end_layer_input = nullptr;
        if (this->m_type == Type::Begin) {
            begin_value = gui_value;
            end_value = m_another_layer_input->get_layer_number();
            end_layer_input = m_another_layer_input;
        }
        if (this->m_type == Type::End) {
            begin_value = m_another_layer_input->get_layer_number();
            end_value = gui_value;
            end_layer_input = this;
        }

        // end value should not be less than begin value
        if (begin_value > end_value) {
            // set new value for end_layer_input
            if (this->m_type == Type::Begin) {
                if (end_layer_input->is_layer_number_valid()) {
                    end_layer_input->set_layer_number(begin_value);
                }
            }
            if (this->m_type == Type::End) {
                if (!this->is_layer_number_valid()) {
                    this->set_layer_number(begin_value);
                    wxCommandEvent evt(EVT_NEED_RESORT_LAYERS);
                    wxPostEvent(m_parent, evt);
                }
                else {
                    // do nothing
                    // reset to the last value for end_layer_input
                }
                return;
            }
        }
        m_layer_number = gui_value;
        wxCommandEvent evt(EVT_NEED_RESORT_LAYERS);
        wxPostEvent(m_parent, evt);
    };
    auto commit_layer_number_from_gui = [this, validate_input_value]() {
        if (m_value_type == ValueType::End)
            return;

        auto gui_str = GetTextCtrl()->GetValue().ToStdString();
        if (gui_str.empty()) {
            m_layer_number = -1;
            wxCommandEvent evt(EVT_NEED_RESORT_LAYERS);
            wxPostEvent(m_parent, evt);
        }
        if (!gui_str.empty()) {
            int gui_value = atoi(gui_str.c_str());
            validate_input_value(gui_value);
        }
        update_label();
    };
    Bind(wxEVT_TEXT_ENTER, [commit_layer_number_from_gui](wxEvent& evt) {
        commit_layer_number_from_gui();
        evt.Skip();
        });
    Bind(wxEVT_KILL_FOCUS, [commit_layer_number_from_gui](wxFocusEvent& evt) {
        commit_layer_number_from_gui();
        evt.Skip();
        });

    Bind(wxEVT_COMBOBOX, [this](auto& e) {
        if (e.GetSelection() == 0) {
            m_value_type = ValueType::End;
        }
        else if (e.GetSelection() == 1) {
            m_value_type = ValueType::Custom;
            m_layer_number = -1;
            update_label();
        }
        e.Skip();
        });
}

void LayerNumberTextInput::update_label()
{
    if (m_value_type == ValueType::End)
        return;

    if (!is_layer_number_valid()) {
        SetLabel("");
    }
    else
        SetLabel(std::to_string(m_layer_number));
}

void LayerNumberTextInput::set_layer_number(int layer_number)
{
    m_layer_number = layer_number;
    if (layer_number == MAX_LAYER_VALUE)
        m_value_type = ValueType::End;
    else
        m_value_type = ValueType::Custom;

    if (m_value_type == ValueType::End)
        SetSelection(0);
    if (m_value_type == ValueType::Custom) {
        SetSelection(1);
        update_label();
    }
}

int LayerNumberTextInput::get_layer_number()
{
    return m_value_type == ValueType::End ? MAX_LAYER_VALUE : m_layer_number;
}

bool LayerNumberTextInput::is_layer_number_valid()
{
    if (m_value_type == ValueType::End)
        return true;
    return m_layer_number >= MIN_LAYER_VALUE;
}

OtherLayersSeqPanel::OtherLayersSeqPanel(wxWindow* parent)
    :wxPanel(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize)
{
    m_bmp_delete = ScalableBitmap(this, "delete_filament");
    m_bmp_add = ScalableBitmap(this, "add_filament");

    SetBackgroundColour(*wxWHITE);

    wxBoxSizer* top_sizer = new wxBoxSizer(wxVERTICAL);

    wxBoxSizer* title_sizer = new wxBoxSizer(wxHORIZONTAL);
    m_other_layer_print_seq_choice = new ComboBox(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(FromDIP(240), -1), 0, NULL, wxCB_READONLY);
    m_other_layer_print_seq_choice->Append(_L("Auto"));
    m_other_layer_print_seq_choice->Append(_L("Customize"));
    m_other_layer_print_seq_choice->SetSelection(0);
    wxStaticText* other_layer_txt = new wxStaticText(this, wxID_ANY, _L("Other layer filament sequence"));
    other_layer_txt->SetFont(Label::Body_14);
    title_sizer->Add(other_layer_txt, 0, wxALIGN_CENTER | wxALIGN_LEFT, 0);
    title_sizer->AddStretchSpacer();
    title_sizer->Add(m_other_layer_print_seq_choice, 0, wxALIGN_CENTER | wxALIGN_RIGHT, 0);

    wxBoxSizer* buttons_sizer = new wxBoxSizer(wxHORIZONTAL);
    ScalableButton* add_layers_btn = new ScalableButton(this, wxID_ANY, m_bmp_add);
    add_layers_btn->SetBackgroundColour(GetBackgroundColour());
    ScalableButton* delete_layers_btn = new ScalableButton(this, wxID_ANY, m_bmp_delete);
    delete_layers_btn->SetBackgroundColour(GetBackgroundColour());
    buttons_sizer->Add(add_layers_btn, 0, wxLEFT | wxRIGHT | wxALIGN_CENTER, FromDIP(5));
    buttons_sizer->Add(delete_layers_btn, 0, wxLEFT | wxRIGHT | wxALIGN_CENTER, FromDIP(5));
    buttons_sizer->Show(false);

    m_layer_input_panel = new wxPanel(this);
    wxBoxSizer* layer_panel_sizer = new wxBoxSizer(wxVERTICAL);
    m_layer_input_panel->SetSizer(layer_panel_sizer);
    m_layer_input_panel->Hide();
    append_layer();

    top_sizer->Add(title_sizer, 0, wxEXPAND, 0);
    top_sizer->Add(buttons_sizer, 0, wxALIGN_CENTER, 0);
    top_sizer->Add(m_layer_input_panel, 0, wxEXPAND, 0);

    SetSizer(top_sizer);
    Layout();
    top_sizer->Fit(this);


    m_other_layer_print_seq_choice->Bind(wxEVT_COMBOBOX, [this, buttons_sizer](auto& e) {
        if (e.GetSelection() == 0) {
            m_layer_input_panel->Show(false);
            buttons_sizer->Show(false);
        }
        else if (e.GetSelection() == 1) {
            m_layer_input_panel->Show(true);
            buttons_sizer->Show(true);
        }
        m_parent->Layout();
        m_parent->Fit();
        });
    add_layers_btn->Bind(wxEVT_BUTTON, [this](wxEvent&) {
        Freeze();
        append_layer();
        m_parent->Layout();
        m_parent->Fit();
        Thaw();
        });
    delete_layers_btn->Bind(wxEVT_BUTTON, [this](wxEvent&) {
        popup_layer();
        m_parent->Layout();
        m_parent->Fit();
        });
    Bind(EVT_NEED_RESORT_LAYERS, [this](auto& evt) {
        std::vector<LayerSeqInfo> result;
        for (int i = 0; i < m_layer_input_sizer_list.size(); i++) {
            int begin_layer_number = m_begin_layer_input_list[i]->get_layer_number();
            int end_layer_number = m_end_layer_input_list[i]->get_layer_number();
            result.push_back({ begin_layer_number, end_layer_number, m_drag_canvas_list[i]->get_shape_list_order() });
        }
        if (!std::is_sorted(result.begin(), result.end())) {
            std::sort(result.begin(), result.end());
            sync_layers_print_seq(1, result);
        }
        result.swap(m_layer_seq_infos);
        });
    Bind(EVT_SET_BED_TYPE_CONFIRM, [this](auto& evt) {
        std::vector<LayerSeqInfo> result;
        for (int i = 0; i < m_layer_input_sizer_list.size(); i++) {
            int begin_layer_number = m_begin_layer_input_list[i]->get_layer_number();
            int end_layer_number = m_end_layer_input_list[i]->get_layer_number();

            if (!m_begin_layer_input_list[i]->is_layer_number_valid() || !m_end_layer_input_list[i]->is_layer_number_valid()) {
                MessageDialog msg_dlg(nullptr, _L("Please input layer value (>= 2)."), wxEmptyString, wxICON_WARNING | wxOK);
                msg_dlg.ShowModal();
                evt.SetString("Invalid");
                return;
            }

            result.push_back({ begin_layer_number, end_layer_number, m_drag_canvas_list[i]->get_shape_list_order() });
        }
        result.swap(m_layer_seq_infos);
        });
}

void OtherLayersSeqPanel::append_layer(const LayerSeqInfo* layer_info)
{
    wxBoxSizer* layer_panel_sizer = static_cast<wxBoxSizer*>(m_layer_input_panel->GetSizer());

    wxStaticText* choose_layer_head_txt = new wxStaticText(m_layer_input_panel, wxID_ANY, _L("Layer"));
    choose_layer_head_txt->SetFont(Label::Body_14);

    LayerNumberTextInput* begin_layer_input = new LayerNumberTextInput(m_layer_input_panel, -1, wxSize(FromDIP(100), -1), LayerNumberTextInput::Type::Begin, LayerNumberTextInput::ValueType::Custom);

    wxStaticText* choose_layer_to_txt = new wxStaticText(m_layer_input_panel, wxID_ANY, _L("to"));
    choose_layer_to_txt->SetFont(Label::Body_14);

    LayerNumberTextInput* end_layer_input = new LayerNumberTextInput(m_layer_input_panel, -1, wxSize(FromDIP(100), -1), LayerNumberTextInput::Type::End, LayerNumberTextInput::ValueType::End);

    begin_layer_input->link(end_layer_input);
    if (m_begin_layer_input_list.size() == 0) {
        begin_layer_input->set_layer_number(MIN_LAYER_VALUE);
        end_layer_input->set_layer_number(MAX_LAYER_VALUE);
    }

    const std::vector<std::string> extruder_colours = wxGetApp().plater()->get_extruder_colors_from_plater_config();
    std::vector<int> order(extruder_colours.size());
    for (int i = 0; i < order.size(); i++) {
        order[i] = i + 1;
    }
    auto drag_canvas = new DragCanvas(m_layer_input_panel, extruder_colours, order);

    if (layer_info) {
        begin_layer_input->set_layer_number(std::max(MIN_LAYER_VALUE, layer_info->begin_layer_number));
        end_layer_input->set_layer_number(std::min(MAX_LAYER_VALUE, layer_info->end_layer_number));
        drag_canvas->set_shape_list(extruder_colours, layer_info->print_sequence);
    }

    wxBoxSizer* single_layer_input_sizer = new wxBoxSizer(wxHORIZONTAL);
    single_layer_input_sizer->Add(choose_layer_head_txt, 0, wxRIGHT | wxALIGN_CENTER, FromDIP(5));
    single_layer_input_sizer->Add(begin_layer_input, 0, wxLEFT | wxRIGHT | wxALIGN_CENTER, FromDIP(5));
    single_layer_input_sizer->Add(choose_layer_to_txt, 0, wxLEFT | wxRIGHT | wxALIGN_CENTER, 0);
    single_layer_input_sizer->Add(end_layer_input, 0, wxLEFT | wxRIGHT | wxALIGN_CENTER, FromDIP(5));
    single_layer_input_sizer->AddStretchSpacer();
    single_layer_input_sizer->Add(drag_canvas, 0, wxLEFT | wxALIGN_CENTER, FromDIP(5));
    layer_panel_sizer->Add(single_layer_input_sizer, 0, wxEXPAND | wxBOTTOM, FromDIP(10));
    m_layer_input_sizer_list.push_back(single_layer_input_sizer);
    m_begin_layer_input_list.push_back(begin_layer_input);
    m_end_layer_input_list.push_back(end_layer_input);
    m_drag_canvas_list.push_back(drag_canvas);
}

void OtherLayersSeqPanel::popup_layer()
{
    if (m_layer_input_sizer_list.size() > 1) {
        m_layer_input_sizer_list.back()->Clear(true);
        m_layer_input_sizer_list.pop_back();
        m_begin_layer_input_list.pop_back();
        m_end_layer_input_list.pop_back();
        m_drag_canvas_list.pop_back();
    }
}

void OtherLayersSeqPanel::clear_all_layers()
{
    for (auto sizer : m_layer_input_sizer_list) {
        sizer->Clear(true);
    }
    m_layer_input_sizer_list.clear();
    m_begin_layer_input_list.clear();
    m_end_layer_input_list.clear();
    m_drag_canvas_list.clear();
}

void OtherLayersSeqPanel::sync_layers_print_seq(int selection, const std::vector<LayerSeqInfo>& seq)
{
    if (m_other_layer_print_seq_choice != nullptr) {
        if (selection == 1) {
            clear_all_layers();
            Freeze();
            for (int i = 0; i < seq.size(); i++) {
                append_layer(&seq[i]);
            }
            Thaw();
        }
        m_other_layer_print_seq_choice->SetSelection(selection);

        wxCommandEvent event(wxEVT_COMBOBOX);
        event.SetInt(selection);
        event.SetEventObject(m_other_layer_print_seq_choice);
        wxPostEvent(m_other_layer_print_seq_choice, event);
    }
}


PlateSettingsDialog::PlateSettingsDialog(wxWindow* parent, const wxString& title, bool only_layer_seq, int target_plate_index,
                                         size_t target_plate_id, const wxPoint& pos, const wxSize& size, long style)
:DPIDialog(parent, wxID_ANY, title, pos, size, style)
{
    SetBackgroundColour(*wxWHITE);
    wxBoxSizer* m_sizer_main = new wxBoxSizer(wxVERTICAL);
    auto m_line_top = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxSize(FromDIP(650), -1));
    m_line_top->SetBackgroundColour(wxColour(166, 169, 170));
    m_sizer_main->Add(m_line_top, 0, wxEXPAND, 0);

    wxFlexGridSizer* top_sizer = new wxFlexGridSizer(0, 2, FromDIP(5), 0);
    top_sizer->AddGrowableCol(0,1);
    top_sizer->SetFlexibleDirection(wxBOTH);
    top_sizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    auto plate_name_txt = new wxStaticText(this, wxID_ANY, _L("Plate name"));
    plate_name_txt->SetFont(Label::Body_14);
    m_ti_plate_name = new TextInput(this, wxString::FromDouble(0.0), "", "", wxDefaultPosition, wxSize(FromDIP(240),-1), wxTE_PROCESS_ENTER);
    top_sizer->Add(plate_name_txt, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxTOP | wxBOTTOM, FromDIP(5));
    top_sizer->Add(m_ti_plate_name, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxTOP | wxBOTTOM, FromDIP(5));

    m_bed_type_choice = new ComboBox( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(FromDIP(240),-1), 0, NULL, wxCB_READONLY );
    auto pm           = wxGetApp().plater()->get_curr_printer_model();
    if (pm) {
        m_cur_combox_bed_types.clear();
        m_bed_type_choice->AppendString(_L("Same as Global Plate Type"));
        const ConfigOptionDef *bed_type_def = print_config_def.get("curr_bed_type");
        int                    index        = 0;
        for (auto item : bed_type_def->enum_labels) {
            index++;
            bool find = std::find(pm->not_support_bed_types.begin(), pm->not_support_bed_types.end(), item) != pm->not_support_bed_types.end();
            if (!find) {
                m_bed_type_choice->AppendString(_L(item));
                m_cur_combox_bed_types.emplace_back(BedType(index));
            }
        }

    } else {
        for (BedType i = btDefault; i < btCount; i = BedType(int(i) + 1)) {
            m_bed_type_choice->Append(to_bed_type_name(i));
        }
    }

    if (!wxGetApp().preset_bundle->is_bbl_vendor())
      m_bed_type_choice->Disable();

    wxStaticText* m_bed_type_txt = new wxStaticText(this, wxID_ANY, _L("Bed type"));
    m_bed_type_txt->SetFont(Label::Body_14);
    top_sizer->Add(m_bed_type_txt, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxTOP | wxBOTTOM, FromDIP(5));
    top_sizer->Add(m_bed_type_choice, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxTOP | wxBOTTOM, FromDIP(5));

    // Print Sequence
    m_print_seq_choice = new ComboBox( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(FromDIP(240),-1), 0, NULL, wxCB_READONLY );
    m_print_seq_choice->Append(_L("Same as Global Print Sequence"));
    for (auto i = PrintSequence::ByLayer; i < PrintSequence::ByDefault; i = PrintSequence(int(i) + 1)) {
        m_print_seq_choice->Append(to_print_sequence_name(i));
    }
    wxStaticText* m_print_seq_txt = new wxStaticText(this, wxID_ANY, _L("Print sequence"));
    m_print_seq_txt->SetFont(Label::Body_14);
    top_sizer->Add(m_print_seq_txt, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxTOP | wxBOTTOM, FromDIP(5));
    top_sizer->Add(m_print_seq_choice, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxTOP | wxBOTTOM, FromDIP(5));

    // Spiral mode
    m_spiral_mode_choice = new ComboBox(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(FromDIP(240), -1), 0, NULL, wxCB_READONLY);
    m_spiral_mode_choice->Append(_L("Same as Global"));
    m_spiral_mode_choice->Append(_L("Enable"));
    m_spiral_mode_choice->Append(_L("Disable"));
    m_spiral_mode_choice->SetSelection(0);
    wxStaticText* spiral_mode_txt = new wxStaticText(this, wxID_ANY, _L("Spiral vase"));
    spiral_mode_txt->SetFont(Label::Body_14);
    top_sizer->Add(spiral_mode_txt, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxTOP | wxBOTTOM, FromDIP(5));
    top_sizer->Add(m_spiral_mode_choice, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxTOP | wxBOTTOM, FromDIP(5));

    // Mixed-Nozzle mode is project/plate config, never a Process-preset field. Changes go through
    // the setup wizard.
    auto *mixed_label = new wxStaticText(this, wxID_ANY, _L("Mixed-nozzle slicing"));
    mixed_label->SetFont(Label::Body_14);
    auto *mixed_panel = new wxPanel(this);
    m_mixed_label = mixed_label;
    m_mixed_panel = mixed_panel;
    auto *mixed_sizer = new wxBoxSizer(wxHORIZONTAL);
    auto *mixed_status = new wxStaticText(mixed_panel, wxID_ANY, wxEmptyString);
    auto *mixed_setup = new Button(mixed_panel, _L("Set up..."));
    mixed_setup->SetStyle(::ButtonStyle::Confirm, ::ButtonType::Compact);
    auto &plate_list = wxGetApp().plater()->get_partplate_list();
    const int mixed_target_index = plate_settings_target_index(target_plate_index, plate_list.get_curr_plate_index(),
                                                               plate_list.get_plate_list().size());
    PartPlate *const mixed_target = plate_list.get_plate(mixed_target_index);
    const size_t indexed_target_id = mixed_target == nullptr ? 0 : mixed_target->id().id;
    m_feature_target = {mixed_target_index, target_plate_id == 0 ? indexed_target_id : target_plate_id};
    m_body_target = {m_feature_target.plate_index, m_feature_target.plate_id};
    auto refresh_mixed = [this, mixed_status, mixed_setup, mixed_target_index] {
        auto &current_list = wxGetApp().plater()->get_partplate_list();
        PartPlate *const indexed_plate = current_list.get_plate(mixed_target_index);
        const std::size_t nozzle_count = plate_settings_configured_nozzle_count();
        auto show_mixed_block = [this](bool show) {
            if (m_mixed_label != nullptr)
                m_mixed_label->Show(show);
            if (m_mixed_panel != nullptr)
                m_mixed_panel->Show(show);
        };
        if (!body_split_editor_target_valid(m_body_target, current_list.get_curr_plate_index(),
                                            indexed_plate == nullptr ? 0 : indexed_plate->id().id)) {
            // Fail closed on a stale plate, but the topology check does not depend on the plate, so
            // a single-nozzle printer still hides the whole block rather than showing a dead row.
            show_mixed_block(feature_split_plate_settings_visibility(
                MixedNozzleSlicingMode::Off, nozzle_count).mixed_block);
            mixed_status->SetLabel(_L("Plate unavailable"));
            mixed_setup->Disable();
            relayout_after_visibility_change();
            return;
        }
        auto &project = wxGetApp().preset_bundle->project_config;
        const auto source = effective_mixed_nozzle_mode(project, *indexed_plate);
        const auto visibility = feature_split_plate_settings_visibility(source.effective, nozzle_count);
        show_mixed_block(visibility.mixed_block);
        // Worded like the Printer row, for this plate. "Change..." once it is on.
        const auto row = mixed_nozzle_plate_row_texts(source, mixed_nozzle_project_default(project));
        mixed_status->SetLabel(from_u8(row.status));
        mixed_setup->SetLabel(from_u8(row.button));
        mixed_setup->Enable(visibility.mixed_block);
        refresh_feature_split_editor(false);
        refresh_body_split_editor();
        relayout_after_visibility_change();
    };
    refresh_mixed();
    mixed_setup->Bind(wxEVT_BUTTON, [this, refresh_mixed, mixed_target_index](wxCommandEvent &) {
        PartPlate *const expected = wxGetApp().plater()->get_partplate_list().get_plate(mixed_target_index);
        if (expected != nullptr && expected->id().id == m_body_target.plate_id) {
            // One wizard for every entry point, with this plate's current values: a plate that is
            // on opens at the detail step, one that is off at step 1.
            const bool change = effective_mixed_nozzle_mode(wxGetApp().preset_bundle->project_config,
                                                            *expected).effective != MixedNozzleSlicingMode::Off;
            wxGetApp().plater()->open_mixed_nozzle_wizard(true, mixed_target_index, expected,
                change ? MixedNozzleWizardPage::Speed : MixedNozzleWizardPage::ModeAndScope);
        }
        refresh_mixed();
    });
    // More holds Edit here (the inline Feature editor) and About, worded as in the sidebar.
    auto *mixed_more = new Button(mixed_panel, _L("More"));
    mixed_more->SetStyle(::ButtonStyle::Regular, ::ButtonType::Compact);
    mixed_more->Bind(wxEVT_BUTTON, [this, mixed_more](wxCommandEvent &) { show_mixed_more_menu(mixed_more); });
    mixed_sizer->Add(mixed_status, 1, wxALIGN_CENTER_VERTICAL);
    mixed_sizer->Add(mixed_setup, 0, wxALIGN_CENTER_VERTICAL | wxLEFT, FromDIP(8));
    mixed_sizer->Add(mixed_more, 0, wxALIGN_CENTER_VERTICAL | wxLEFT, FromDIP(6));
    mixed_panel->SetSizer(mixed_sizer);
    top_sizer->Add(mixed_label, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxTOP | wxBOTTOM, FromDIP(5));
    top_sizer->Add(mixed_panel, 1, wxEXPAND | wxALIGN_CENTER_VERTICAL | wxTOP | wxBOTTOM, FromDIP(5));

    m_feature_fine_label = new wxStaticText(this, wxID_ANY, _L("Fine Shell logical filament"));
    m_feature_coarse_label = new wxStaticText(this, wxID_ANY, _L("Coarse Core logical filament"));
    // The coarse cadence combo; see refresh_feature_split_editor() for the choices and pre-selection.
    m_feature_cadence_label = new wxStaticText(this, wxID_ANY, _L("Coarse cadence (all plates)"));
    m_feature_status_label = new wxStaticText(this, wxID_ANY, _L("Mixed-Nozzle status"));
    for (wxStaticText* label : {m_feature_fine_label, m_feature_coarse_label, m_feature_cadence_label,
                                m_feature_status_label})
        label->SetFont(Label::Body_14);
    auto* feature_fine = new ComboBox(this, wxID_ANY, wxEmptyString, wxDefaultPosition,
                                      wxSize(FromDIP(240), -1), 0, nullptr, wxCB_READONLY);
    auto* feature_coarse = new ComboBox(this, wxID_ANY, wxEmptyString, wxDefaultPosition,
                                        wxSize(FromDIP(240), -1), 0, nullptr, wxCB_READONLY);
    // Matches the Body Split cadence combo's 170 DIP bound rather than the 240 DIP fine/coarse
    // combos, so the OK/Cancel row stays visible in a narrow app window.
    auto* feature_cadence = new ComboBox(this, wxID_ANY, wxEmptyString, wxDefaultPosition,
                                         wxSize(FromDIP(170), -1), 0, nullptr, wxCB_READONLY);
    const wxString feature_cadence_scope = _L("This is a Process setting shared by all plates.");
    m_feature_cadence_label->SetToolTip(feature_cadence_scope);
    feature_cadence->SetToolTip(feature_cadence_scope);
    auto* feature_status = new wxStaticText(this, wxID_ANY, wxEmptyString, wxDefaultPosition,
        wxSize(FromDIP(feature_split_status_width_dip()), -1), feature_split_status_window_style());
    feature_status->SetMinSize(wxSize(FromDIP(feature_split_status_width_dip()), -1));
    feature_status->SetMaxSize(wxSize(FromDIP(feature_split_status_width_dip()), -1));
    m_feature_fine = feature_fine;
    m_feature_coarse = feature_coarse;
    m_feature_cadence = feature_cadence;
    m_feature_status = feature_status;
    top_sizer->Add(m_feature_fine_label, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxTOP | wxBOTTOM, FromDIP(5));
    top_sizer->Add(feature_fine, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxTOP | wxBOTTOM, FromDIP(5));
    top_sizer->Add(m_feature_coarse_label, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxTOP | wxBOTTOM, FromDIP(5));
    top_sizer->Add(feature_coarse, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxTOP | wxBOTTOM, FromDIP(5));
    top_sizer->Add(m_feature_cadence_label, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxTOP | wxBOTTOM, FromDIP(5));
    top_sizer->Add(feature_cadence, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxTOP | wxBOTTOM, FromDIP(5));
    top_sizer->Add(m_feature_status_label, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxTOP | wxBOTTOM, FromDIP(5));
    top_sizer->Add(feature_status, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxTOP | wxBOTTOM, FromDIP(5));
    feature_fine->Bind(wxEVT_COMBOBOX, [this](wxCommandEvent&) {
        if (!m_feature_lifecycle.may_access_state(bool(m_feature_fine) && bool(m_feature_coarse)))
            return;
        m_feature_selection.fine_selection = m_feature_fine->GetSelection();
        update_feature_split_status();
    });
    feature_coarse->Bind(wxEVT_COMBOBOX, [this](wxCommandEvent&) {
        if (!m_feature_lifecycle.may_access_state(bool(m_feature_fine) && bool(m_feature_coarse)))
            return;
        m_feature_selection.coarse_selection = m_feature_coarse->GetSelection();
        update_feature_split_status();
    });
    feature_cadence->Bind(wxEVT_COMBOBOX, [this](wxCommandEvent&) {
        if (!m_feature_lifecycle.may_access_state(bool(m_feature_cadence)))
            return;
        update_feature_split_status();
    });
    refresh_feature_split_editor(true);

    m_body_panel = new wxPanel(this);
    auto *body_sizer = new wxBoxSizer(wxVERTICAL);
    auto *body_title = new wxStaticText(m_body_panel, wxID_ANY, _L("Body Split details"));
    body_title->SetFont(Label::Head_14);
    body_sizer->Add(body_title, 0, wxBOTTOM, FromDIP(6));
    m_body_table = new wxFlexGridSizer(0, 7, FromDIP(4), FromDIP(8));
    m_body_table->AddGrowableCol(0, 1);
    for (const wxString &heading : {_L("Body"), _L("Logical filament"), _L("Material"), _L("Physical tool"), _L("Cadence"),
                                    _L("Fine skins"), _L("Skin layers")}) {
        auto *text = new wxStaticText(m_body_panel, wxID_ANY, heading);
        text->SetFont(Label::Body_13);
        m_body_table->Add(text, 0, wxALIGN_CENTER_VERTICAL);
    }
    body_sizer->Add(m_body_table, 0, wxEXPAND);

    auto *phase_sizer = new wxBoxSizer(wxHORIZONTAL);
    phase_sizer->Add(new wxStaticText(m_body_panel, wxID_ANY, _L("After rendezvous")), 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, FromDIP(8));
    m_body_phase = new ComboBox(m_body_panel, wxID_ANY, wxEmptyString, wxDefaultPosition,
                                wxSize(FromDIP(240), -1), 0, nullptr, wxCB_READONLY);
    m_body_phase->Append(_L("Keep current shared grid"));
    m_body_phase->Append(_L("Rephase from plate start"));
    m_body_phase->Bind(wxEVT_COMBOBOX, [this](wxCommandEvent &) {
        m_body_editor_dirty = true;
        m_body_transaction.cancel();
        update_body_split_apply_enablement();
    });
    phase_sizer->Add(m_body_phase, 0, wxALIGN_CENTER_VERTICAL);
    body_sizer->Add(phase_sizer, 0, wxTOP, FromDIP(8));

    auto *status_sizer = new wxBoxSizer(wxHORIZONTAL);
    // bounded_body_split_text() bounds characters, not pixels; ellipsizing keeps the label
    // inside FromDIP(420).
    m_body_status = new wxStaticText(m_body_panel, wxID_ANY, wxEmptyString, wxDefaultPosition,
        wxSize(FromDIP(420), -1), wxST_ELLIPSIZE_END);
    m_body_cancel = new Button(m_body_panel, _L("Cancel"));
    m_body_cancel->SetStyle(::ButtonStyle::Regular, ::ButtonType::Compact);
    status_sizer->Add(m_body_status, 1, wxALIGN_CENTER_VERTICAL | wxRIGHT, FromDIP(8));
    status_sizer->Add(m_body_cancel, 0, wxALIGN_CENTER_VERTICAL);
    body_sizer->Add(status_sizer, 0, wxEXPAND | wxTOP, FromDIP(8));
    m_body_panel->SetSizer(body_sizer);
    m_sizer_main->Add(m_body_panel, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, FromDIP(30));
    m_body_cancel->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
        m_body_transaction.cancel();
        refresh_body_split_editor();
    });
    Bind(wxEVT_ACTIVATE, [this](wxActivateEvent &event) {
        if (event.GetActive()) {
            refresh_feature_split_editor(false);
            // m_body_editor_dirty defers the rebuild after any widget edit; otherwise an
            // activation would discard it.
            if (!m_body_transaction.has_pending() && !m_body_editor_dirty)
                refresh_body_split_editor();
        }
        event.Skip();
    });
    refresh_body_split_editor();

    // First layer filament sequence
    m_first_layer_print_seq_choice = new ComboBox(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize(FromDIP(240), -1), 0, NULL, wxCB_READONLY);
    m_first_layer_print_seq_choice->Append(_L("Auto"));
    m_first_layer_print_seq_choice->Append(_L("Customize"));
    m_first_layer_print_seq_choice->SetSelection(0);
    m_first_layer_print_seq_choice->Bind(wxEVT_COMBOBOX, [this](auto& e) {
        if (e.GetSelection() == 0) {
            m_drag_canvas->Hide();
        }
        else if (e.GetSelection() == 1) {
            m_drag_canvas->Show();
        }
        Layout();
        Fit();
        });
    wxStaticText* first_layer_txt = new wxStaticText(this, wxID_ANY, _L("First layer filament sequence"));
    first_layer_txt->SetFont(Label::Body_14);
    top_sizer->Add(first_layer_txt, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxTOP | wxBOTTOM, FromDIP(5));
    top_sizer->Add(m_first_layer_print_seq_choice, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxTOP | wxBOTTOM, FromDIP(5));

    const std::vector<std::string> extruder_colours = wxGetApp().plater()->get_extruder_colors_from_plater_config();
    std::vector<int> order(extruder_colours.size());
    for (int i = 0; i < order.size(); i++) {
        order[i] = i + 1;
    }
    m_drag_canvas = new DragCanvas(this, extruder_colours, order);
    m_drag_canvas->Hide();
    top_sizer->Add(0, 0, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT, 0);
    top_sizer->Add(m_drag_canvas, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxBOTTOM, FromDIP(10));

    m_sizer_main->Add(top_sizer, 0, wxEXPAND | wxTOP | wxLEFT | wxRIGHT, FromDIP(30));

    // Other layer filament sequence
    m_other_layers_seq_panel = new OtherLayersSeqPanel(this);
    m_sizer_main->AddSpacer(FromDIP(5));
    m_sizer_main->Add(m_other_layers_seq_panel, 0, wxEXPAND | wxLEFT | wxRIGHT, FromDIP(30));

    auto dlg_btns = new DialogButtons(this, {"OK", "Cancel"});
    m_ok_button = dlg_btns->GetOK();
    update_ok_button();

    m_ok_button->Bind(wxEVT_BUTTON, [this](auto& e) {
        if (!validate_feature_split_acceptance())
            return;
        if (m_body_panel != nullptr && m_body_panel->IsShown() && !apply_body_split_editor())
            return;
        if (!dispatch_plate_settings_acceptance(*static_cast<wxEvtHandler*>(m_other_layers_seq_panel),
                                                *GetEventHandler(), GetId()))
            return;
        if (this->IsModal())
            EndModal(wxID_YES);
        else
            this->Close();
        });

    dlg_btns->GetCANCEL()->Bind(wxEVT_BUTTON, [this](auto& e) {
        if (this->IsModal())
            EndModal(wxID_NO);
        else
            this->Close();
        });

    m_sizer_main->AddSpacer(FromDIP(20));
    m_sizer_main->Add(dlg_btns, 0, wxEXPAND);

    SetSizer(m_sizer_main);
    Layout();
    m_sizer_main->Fit(this);
    clamp_dialog_width_to_display();

    CenterOnParent();

    wxGetApp().UpdateDlgDarkUI(this);

    if (only_layer_seq) {
        for (auto item : top_sizer->GetChildren()) {
            if (item->GetWindow())
                item->GetWindow()->Show(false);
        }
        first_layer_txt->Show();
        m_first_layer_print_seq_choice->Show();
        m_drag_canvas->Show();
        m_other_layers_seq_panel->Show();
        Layout();
        Fit();
        clamp_dialog_width_to_display();
        CenterOnParent();
    }
}

void PlateSettingsDialog::relayout_after_visibility_change()
{
    // refresh_mixed() runs once from the constructor before SetSizer(), where Fit() would size the
    // dialog from an incomplete layout and clamp_dialog_width_to_display() would SetSize() on it.
    if (GetSizer() == nullptr)
        return;
    Layout();
    Fit();
    clamp_dialog_width_to_display();
}

void PlateSettingsDialog::clamp_dialog_width_to_display()
{
    // A Fit() must never grow the dialog past the display it opened on.
    wxDisplay display(this);
    const wxRect client_area = display.GetClientArea();
    if (client_area.GetWidth() <= 0)
        return;
    const int max_width = std::max(FromDIP(400), client_area.GetWidth() - FromDIP(40));
    const wxSize current = GetSize();
    if (current.GetWidth() > max_width)
        SetSize(wxSize(max_width, current.GetHeight()));

    // Keep a clamped or naturally-sized dialog from opening partly off-screen.
    const wxSize final_size = GetSize();
    wxPoint pos = GetPosition();
    if (pos.x + final_size.GetWidth() > client_area.GetRight())
        pos.x = std::max(client_area.GetLeft(), client_area.GetRight() - final_size.GetWidth());
    if (pos.y + final_size.GetHeight() > client_area.GetBottom())
        pos.y = std::max(client_area.GetTop(), client_area.GetBottom() - final_size.GetHeight());
    if (pos != GetPosition())
        SetPosition(pos);
}

void PlateSettingsDialog::set_body_split_status(const wxString& text, const wxString& tooltip, bool body_ok)
{
    if (m_body_status == nullptr)
        return;
    const BodySplitBoundedText bounded = bounded_body_split_text(text.ToStdString(), 90);
    m_body_status->SetLabel(from_u8(bounded.display));
    m_body_status->SetToolTip(tooltip.empty() ? from_u8(bounded.tooltip) : tooltip);
    m_body_ok = body_ok;
    update_ok_button();
}

// OK saves the Body table, so it is off while a shown table has an edit OK cannot save.
void PlateSettingsDialog::update_ok_button()
{
    if (m_ok_button != nullptr)
        m_ok_button->Enable(m_body_ok || m_body_panel == nullptr || !m_body_panel->IsShown());
}

void PlateSettingsDialog::set_feature_split_status(const std::string& full_status)
{
    if (!m_feature_lifecycle.may_access_state(bool(m_feature_status)))
        return;
    // The label reads the sentence only; an engine code stays on the tooltip's Details line.
    const auto bounded = bounded_feature_split_text(split_engine_message(full_status).text, 1000);
    m_feature_status->SetLabel(from_u8(bounded.display));
    m_feature_status->SetToolTip(from_u8(engine_message_for_display(full_status)));
}

void PlateSettingsDialog::refresh_feature_split_editor(bool restore_live_selection)
{
    if (!m_feature_lifecycle.may_access_state(bool(m_feature_fine) && bool(m_feature_coarse) &&
                                              bool(m_feature_cadence) && bool(m_feature_status)))
        return;
    auto* plater = wxGetApp().plater();
    auto show_rows = [this](bool show) {
        m_feature_fine_label->Show(show);
        m_feature_coarse_label->Show(show);
        m_feature_cadence_label->Show(show);
        m_feature_status_label->Show(show);
        m_feature_fine->Show(show);
        m_feature_coarse->Show(show);
        m_feature_cadence->Show(show);
        m_feature_status->Show(show);
    };
    if (plater == nullptr) {
        show_rows(false);
        return;
    }
    auto& plate_list = plater->get_partplate_list();
    PartPlate* plate = plate_list.get_plate(m_feature_target.plate_index);
    if (!feature_split_target_valid(m_feature_target, plate_list.get_curr_plate_index(),
                                    plate == nullptr ? 0 : plate->id().id)) {
        show_rows(true);
        m_feature_fine->Disable();
        m_feature_coarse->Disable();
        m_feature_cadence->Disable();
        set_feature_split_status("Target plate changed or is unavailable");
        Layout();
        return;
    }

    auto& project = wxGetApp().preset_bundle->project_config;
    const auto mode = effective_mixed_nozzle_mode(project, *plate).effective;
    const auto visibility = feature_split_plate_settings_visibility(
        mode, plate_settings_configured_nozzle_count());
    m_feature_edit_here_offered = feature_split_plate_settings_edit_here_offered(visibility);
    // Filled below whenever the mode has Feature rows; shown only after Edit here.
    show_rows(feature_split_plate_settings_rows_shown(visibility, m_feature_edit_here));
    if (!visibility.feature_rows) {
        relayout_after_visibility_change();
        return;
    }

    const std::vector<std::string> preset_labels = filament_preset_labels(*wxGetApp().preset_bundle);
    const auto effective_pair = plate->get_effective_feature_split_filaments(project, preset_labels.size());
    const auto seed = build_feature_split_native_seed(preset_labels, effective_pair);
    const FeatureSplitSelection retained = m_feature_selection;
    m_feature_fine->Clear();
    m_feature_coarse->Clear();
    for (const std::string& label : seed.labels) {
        m_feature_fine->Append(from_u8(label));
        m_feature_coarse->Append(from_u8(label));
    }
    m_feature_selection = restore_live_selection ? seed.selection : retained;
    if (m_feature_selection.fine_selection < 0 || size_t(m_feature_selection.fine_selection) >= seed.labels.size())
        m_feature_selection.fine_selection = wxNOT_FOUND;
    if (m_feature_selection.coarse_selection < 0 || size_t(m_feature_selection.coarse_selection) >= seed.labels.size())
        m_feature_selection.coarse_selection = wxNOT_FOUND;
    m_feature_fine->SetSelection(m_feature_selection.fine_selection);
    m_feature_coarse->SetSelection(m_feature_selection.coarse_selection);
    m_feature_fine->Enable();
    m_feature_coarse->Enable();

    // base_layer_height, the resolver and the coarse height all come from full_config(), so this
    // pre-selects the effective value, which is the Process preset's.
    const DynamicPrintConfig feature_full_config = wxGetApp().preset_bundle->full_config();
    const PrintConfig feature_resolver = mixed_nozzle_resolver_config_from_full(feature_full_config);
    const double base_layer_height = feature_full_config.has("layer_height")
        ? feature_full_config.opt_float("layer_height") : 0.;
    const double current_height = feature_full_config.has("mixed_nozzle_coarse_layer_height")
        ? feature_full_config.opt_float("mixed_nozzle_coarse_layer_height") : 0.;
    m_feature_cadence_values = feature_split_coarse_cadence_choices(base_layer_height, feature_resolver);
    m_feature_cadence->Clear();
    int cadence_selection = wxNOT_FOUND;
    std::vector<wxString> cadence_labels;
    cadence_labels.reserve(m_feature_cadence_values.size());
    for (size_t i = 0; i < m_feature_cadence_values.size(); ++i) {
        const int ratio = base_layer_height > 0. ? int(std::lround(m_feature_cadence_values[i] / base_layer_height)) : 0;
        // Same wording the Body table uses, so the two editors read identically and the control
        // needs less width for the same information. The "(all plates)" scope stays on the row
        // label and its tooltip.
        const wxString label = wxString::Format(_L("Coarse cadence, N=%d, %.3f mm"), ratio,
                                                m_feature_cadence_values[i]);
        m_feature_cadence->Append(label);
        cadence_labels.push_back(label);
        if (std::abs(m_feature_cadence_values[i] - current_height) <= 1e-6)
            cadence_selection = int(i);
    }
    m_feature_cadence->SetSelection(cadence_selection);
    m_feature_cadence->Enable(!m_feature_cadence_values.empty());
    // Measured like the Body table so neither the closed label nor the popup rows clip;
    // clamp_dialog_width_to_display() still bounds the dialog to its display.
    ComboBox* const cadence_combo = m_feature_cadence;
    cadence_combo->SetMinSize(wxSize(mixed_nozzle_cadence_combo_width(cadence_labels, FromDIP(170), FromDIP(40),
        [cadence_combo](const wxString& text) { return cadence_combo->GetTextExtent(text).x; }), -1));

    update_feature_split_status();
    relayout_after_visibility_change();
}

std::optional<double> PlateSettingsDialog::feature_split_coarse_layer_height_choice() const
{
    if (!m_feature_cadence)
        return std::nullopt;
    const int selection = m_feature_cadence->GetSelection();
    if (selection < 0 || size_t(selection) >= m_feature_cadence_values.size())
        return std::nullopt;
    return m_feature_cadence_values[size_t(selection)];
}

void PlateSettingsDialog::update_feature_split_status()
{
    if (!m_feature_lifecycle.may_access_state(bool(m_feature_fine) && bool(m_feature_coarse) && bool(m_feature_status)))
        return;
    auto* plater = wxGetApp().plater();
    if (plater == nullptr)
        return;
    auto& plate_list = plater->get_partplate_list();
    PartPlate* plate = plate_list.get_plate(m_feature_target.plate_index);
    if (!feature_split_target_valid(m_feature_target, plate_list.get_curr_plate_index(),
                                    plate == nullptr ? 0 : plate->id().id)) {
        set_feature_split_status("Target plate changed or is unavailable");
        return;
    }
    auto& bundle = *wxGetApp().preset_bundle;
    const DynamicPrintConfig full_config = bundle.full_config();
    PrintConfig resolver = mixed_nozzle_resolver_config_from_full(full_config);
    resolver.filament_map_mode.value = plate->get_real_filament_map_mode(bundle.project_config);
    resolver.filament_map.values = plate->get_real_filament_maps(bundle.project_config);
    const auto pair = feature_split_pair_from_selections(m_feature_selection, bundle.filament_presets.size());
    const size_t nozzle_count = full_config.has("nozzle_diameter")
        ? full_config.option<ConfigOptionFloats>("nozzle_diameter")->values.size() : 0;
    const auto mode = effective_mixed_nozzle_mode(bundle.project_config, *plate).effective;
    const auto state = build_feature_split_editor_state(mode, nozzle_count, pair,
                                                        bundle.filament_presets.size(), resolver);
    set_feature_split_status(state.full_status);
}

bool PlateSettingsDialog::validate_feature_split_acceptance()
{
    auto* plater = wxGetApp().plater();
    if (plater == nullptr)
        return false;
    auto& plate_list = plater->get_partplate_list();
    PartPlate* plate = plate_list.get_plate(m_feature_target.plate_index);
    if (!feature_split_target_valid(m_feature_target, plate_list.get_curr_plate_index(),
                                    plate == nullptr ? 0 : plate->id().id)) {
        set_feature_split_status("Target plate changed or is unavailable");
        return false;
    }
    const auto mode = effective_mixed_nozzle_mode(wxGetApp().preset_bundle->project_config, *plate).effective;
    if (!feature_split_editor_visible(mode))
        return true;
    update_feature_split_status();
    const auto pair = feature_split_pair_from_selections(m_feature_selection,
        wxGetApp().preset_bundle->filament_presets.size());
    if (!pair) {
        // The answer is the same with the rows closed; open them so the user sees why.
        if (!m_feature_edit_here)
            set_feature_edit_here(true);
        if (m_feature_fine && m_feature_selection.fine_selection == wxNOT_FOUND)
            m_feature_fine->SetFocus();
        else if (m_feature_coarse)
            m_feature_coarse->SetFocus();
        return false;
    }
    const DynamicPrintConfig full_config = wxGetApp().preset_bundle->full_config();
    PrintConfig resolver = mixed_nozzle_resolver_config_from_full(full_config);
    resolver.filament_map_mode.value = plate->get_real_filament_map_mode(wxGetApp().preset_bundle->project_config);
    resolver.filament_map.values = plate->get_real_filament_maps(wxGetApp().preset_bundle->project_config);
    const size_t nozzle_count = full_config.has("nozzle_diameter")
        ? full_config.option<ConfigOptionFloats>("nozzle_diameter")->values.size() : 0;
    const auto state = build_feature_split_editor_state(mode, nozzle_count, pair,
        wxGetApp().preset_bundle->filament_presets.size(), resolver);
    if (!state.can_apply && !m_feature_edit_here)
        set_feature_edit_here(true);
    if (!state.can_apply && m_feature_status)
        m_feature_status->SetFocus();
    return state.can_apply;
}

void PlateSettingsDialog::set_feature_edit_here(bool edit_here)
{
    m_feature_edit_here = edit_here;
    // Only visibility changes: the rows keep what they hold, so OK accepts the same selection
    // and cadence whether they are open, closed or were never opened.
    const bool show = m_feature_edit_here_offered && m_feature_edit_here;
    for (wxWindow *row : std::initializer_list<wxWindow *>{m_feature_fine_label, m_feature_coarse_label,
             m_feature_cadence_label, m_feature_status_label, m_feature_fine.get(), m_feature_coarse.get(),
             m_feature_cadence.get(), m_feature_status.get()})
        if (row != nullptr)
            row->Show(show);
    relayout_after_visibility_change();
}

void PlateSettingsDialog::show_mixed_more_menu(wxWindow* anchor)
{
    const MixedNozzleSidebarMoreMenu labels;
    wxMenu menu;
    const int edit_here = wxWindow::NewControlId();
    const int about = wxWindow::NewControlId();
    menu.AppendCheckItem(edit_here, from_u8(labels.edit_here));
    menu.Check(edit_here, m_feature_edit_here);
    menu.Enable(edit_here, m_feature_edit_here_offered);
    menu.AppendSeparator();
    menu.Append(about, from_u8(labels.about));
    const wxPoint position = anchor == nullptr ? wxDefaultPosition :
        ScreenToClient(anchor->GetScreenPosition()) + wxPoint(0, anchor->GetSize().GetHeight());
    const int chosen = TestMode::popup_menu(*this, menu, position);
    if (chosen == edit_here)
        set_feature_edit_here(!m_feature_edit_here);
    else if (chosen == about && wxGetApp().plater() != nullptr)
        wxGetApp().plater()->show_mixed_nozzle_intro();
}

void PlateSettingsDialog::focus_feature_split_editor(const wxString& target)
{
    // A validation action that points at a Feature row opens the rows first.
    if (!plate_settings_event_opens_feature_rows(target.ToStdString()))
        return;
    m_feature_edit_here = true;
    refresh_feature_split_editor(false);
    if (target == "feature_fine" && m_feature_fine)
        m_feature_fine->SetFocus();
    else if (target == "feature_coarse" && m_feature_coarse)
        m_feature_coarse->SetFocus();
    if (m_feature_status)
        m_feature_status->Show();
}

void PlateSettingsDialog::refresh_body_split_editor()
{
    if (m_body_panel == nullptr)
        return;
    auto *plater = wxGetApp().plater();
    if (plater == nullptr) {
        m_body_panel->Hide();
        update_ok_button();
        return;
    }
    auto &plate_list = plater->get_partplate_list();
    PartPlate *plate = plate_list.get_plate(m_body_target.plate_index);
    if (!body_split_editor_target_valid(m_body_target, plate_list.get_curr_plate_index(),
                                        plate == nullptr ? 0 : plate->id().id)) {
        m_body_panel->Show();
        set_body_split_status(_L("The plate changed. Close and reopen Plate Settings."),
                              _L("Close and reopen Plate Settings for the current plate."), false);
        Layout();
        return;
    }

    auto &project = wxGetApp().preset_bundle->project_config;
    const auto source = effective_mixed_nozzle_mode(project, *plate);
    const auto visibility = feature_split_plate_settings_visibility(
        source.effective, plate_settings_configured_nozzle_count());
    if (!visibility.body_editor) {
        m_body_panel->Hide();
        update_ok_button();
        relayout_after_visibility_change();
        return;
    }
    m_body_panel->Show();

    const DynamicPrintConfig full_config = wxGetApp().preset_bundle->full_config();
    m_body_resolver = mixed_nozzle_resolver_config_from_full(full_config);
    m_body_resolver.filament_map_mode.value = plate->get_real_filament_map_mode(project);
    m_body_resolver.filament_map.values = plate->get_real_filament_maps(project);

    BodySplitPresentation presentation;
    const auto &preset_names = wxGetApp().preset_bundle->filament_presets;
    const std::vector<std::string> preset_labels = filament_preset_labels(*wxGetApp().preset_bundle);
    for (size_t i = 0; i < preset_names.size(); ++i)
        presentation.filaments.push_back(
            {int(i + 1), preset_labels[i], filament_material_label(*wxGetApp().preset_bundle, i)});
    const double inherited_height = full_config.has("layer_height") ? full_config.opt_float("layer_height") : 0.;
    m_body_model = build_body_split_editor_model(plate->get_objects_on_this_plate(), m_body_resolver,
                                                 presentation, inherited_height);

    m_body_table->Clear(true);
    for (const wxString &heading : {_L("Body"), _L("Logical filament"), _L("Material"), _L("Physical tool"), _L("Cadence"),
                                    _L("Fine skins"), _L("Skin layers")}) {
        auto *text = new wxStaticText(m_body_panel, wxID_ANY, heading);
        text->SetFont(Label::Body_13);
        m_body_table->Add(text, 0, wxALIGN_CENTER_VERTICAL);
    }
    m_body_filament_choices.clear();
    m_body_cadence_choices.clear();
    m_body_material_labels.clear();
    m_body_tool_labels.clear();
    m_body_fine_skin_checks.clear();
    m_body_fine_skin_layer_spins.clear();
    const auto body_identities = body_split_row_identities(m_body_model.rows, 34);
    for (size_t row_index = 0; row_index < m_body_model.rows.size(); ++row_index) {
        const BodySplitEditorRow &row = m_body_model.rows[row_index];
        const auto &body_text = body_identities[row_index];
        auto *body = new wxStaticText(m_body_panel, wxID_ANY, from_u8(body_text.display));
        body->SetToolTip(from_u8(body_text.tooltip));
        // The visible label is bounded, so the accessible name carries the complete identity.
        body->SetName(from_u8(body_text.tooltip));
        m_body_table->Add(body, 0, wxALIGN_CENTER_VERTICAL);

        // Narrow enough that the OK/Cancel row stays visible in a narrow app window.
        auto *filament = new ComboBox(m_body_panel, wxID_ANY, wxEmptyString, wxDefaultPosition,
                                      wxSize(FromDIP(150), -1), 0, nullptr, wxCB_READONLY);
        for (size_t i = 0; i < preset_labels.size(); ++i)
            filament->Append(from_u8(body_split_filament_label(i + 1, preset_labels[i])));
        filament->SetSelection(row.logical_filament > 0 && size_t(row.logical_filament) <= preset_names.size()
            ? row.logical_filament - 1 : wxNOT_FOUND);
        m_body_filament_choices.push_back(filament);
        m_body_table->Add(filament, 0, wxALIGN_CENTER_VERTICAL);

        // The widest columns: "tool" can hold a full diagnostic sentence. Fixed width, ellipsis
        // and tooltip, as for the Body column.
        auto *material = new wxStaticText(m_body_panel, wxID_ANY, from_u8(row.material_label), wxDefaultPosition,
            wxSize(FromDIP(90), -1), wxST_ELLIPSIZE_END);
        material->SetMinSize(wxSize(FromDIP(90), -1));
        material->SetMaxSize(wxSize(FromDIP(90), -1));
        material->SetToolTip(from_u8(row.material_label));
        auto *tool = new wxStaticText(m_body_panel, wxID_ANY, wxEmptyString, wxDefaultPosition,
            wxSize(FromDIP(170), -1), wxST_ELLIPSIZE_END);
        tool->SetMinSize(wxSize(FromDIP(170), -1));
        tool->SetMaxSize(wxSize(FromDIP(170), -1));
        m_body_material_labels.push_back(material);
        m_body_tool_labels.push_back(tool);
        m_body_table->Add(material, 0, wxALIGN_CENTER_VERTICAL);
        m_body_table->Add(tool, 0, wxALIGN_CENTER_VERTICAL);

        auto *cadence = new ComboBox(m_body_panel, wxID_ANY, wxEmptyString, wxDefaultPosition,
                                     wxSize(FromDIP(170), -1), 0, nullptr, wxCB_READONLY);
        const auto& choices = row.cadence_choices;
        std::vector<wxString> cadence_labels;
        cadence_labels.reserve(choices.size());
        for (size_t choice = 0; choice < choices.size(); ++choice) {
            const int ratio = int(std::lround(choices[choice] / row.base_layer_height));
            const wxString label = ratio == 1
                ? wxString::Format(_L("Fine cadence, %.3f mm"), choices[choice])
                : wxString::Format(_L("Coarse cadence, N=%d, %.3f mm"), ratio, choices[choice]);
            cadence->Append(label);
            cadence_labels.push_back(label);
            if ((!row.regional_height_explicit && ratio == 1) ||
                std::abs(row.effective_layer_height - choices[choice]) <= 1e-8)
                cadence->SetSelection(int(choice));
        }
        cadence->SetMinSize(wxSize(mixed_nozzle_cadence_combo_width(cadence_labels, FromDIP(170), FromDIP(40),
            [cadence](const wxString& text) { return cadence->GetTextExtent(text).x; }), -1));
        m_body_cadence_choices.push_back(cadence);
        m_body_table->Add(cadence, 0, wxALIGN_CENTER_VERTICAL);

        // Fine-skin checkbox and skin-layer spinner, enabled only on a coarse-cadence row: a
        // fine-cadence body already prints its shell with the fine nozzle at fine rows.
        auto *fine_skins = new CheckBox(m_body_panel, wxID_ANY);
        fine_skins->SetValue(row.fine_skins);
        fine_skins->Enable(row.fine_skin_controls_applicable);
        m_body_fine_skin_checks.push_back(fine_skins);
        m_body_table->Add(fine_skins, 0, wxALIGN_CENTER_VERTICAL);

        auto *fine_skin_layers = new SpinInput(m_body_panel, wxEmptyString, wxEmptyString, wxDefaultPosition,
                                               wxSize(FromDIP(80), -1), wxSP_ARROW_KEYS, 1, 999,
                                               std::max(1, row.fine_skin_layers));
        fine_skin_layers->Enable(row.fine_skin_controls_applicable);
        m_body_fine_skin_layer_spins.push_back(fine_skin_layers);
        m_body_table->Add(fine_skin_layers, 0, wxALIGN_CENTER_VERTICAL);

        filament->Bind(wxEVT_COMBOBOX, [this](wxCommandEvent &) {
            m_body_editor_dirty = true;
            m_body_transaction.cancel();
            update_body_split_apply_enablement();
        });
        cadence->Bind(wxEVT_COMBOBOX, [this](wxCommandEvent &) {
            m_body_editor_dirty = true;
            m_body_transaction.cancel();
            update_body_split_apply_enablement();
        });
        fine_skins->Bind(wxEVT_TOGGLEBUTTON, [this](wxCommandEvent &event) {
            m_body_editor_dirty = true;
            m_body_transaction.cancel();
            update_body_split_apply_enablement();
            // Let CheckBox run its own deferred bitmap update after the native
            // toggle completes; consuming this event leaves the old checkmark.
            event.Skip();
        });
        fine_skin_layers->Bind(wxEVT_SPINCTRL, [this](wxCommandEvent &) {
            m_body_editor_dirty = true;
            m_body_transaction.cancel();
            update_body_split_apply_enablement();
        });
        fine_skin_layers->Bind(wxEVT_TEXT_ENTER, [this](wxCommandEvent &) {
            m_body_editor_dirty = true;
            m_body_transaction.cancel();
            update_body_split_apply_enablement();
        });
    }

    if (!m_body_model.excluded.empty()) {
        auto *excluded = new wxStaticText(m_body_panel, wxID_ANY,
            wxString::Format(_L("%zu excluded modifier/negative/support volume(s)"), m_body_model.excluded.size()));
        excluded->SetToolTip(_L("Excluded volumes are read-only and are not Body Split ownership rows."));
        m_body_table->Add(excluded, 0, wxTOP, FromDIP(4));
        for (int i = 1; i < 7; ++i)
            m_body_table->AddSpacer(0);
    }

    const auto *phase = plate->config()->option<ConfigOptionEnum<RegionalGridPhaseRule>>("regional_grid_phase_rule");
    const RegionalGridPhaseRule phase_value = phase != nullptr ? phase->value :
        (full_config.has("regional_grid_phase_rule")
            ? full_config.option<ConfigOptionEnum<RegionalGridPhaseRule>>("regional_grid_phase_rule")->value
            : RegionalGridPhaseRule::KeepNominal);
    m_body_phase->SetSelection(phase_value == RegionalGridPhaseRule::Rephase ? 1 : 0);
    // The rebuild is in sync with the model, so clear the guard and let the next edit defer an
    // activation rebuild again. Cleared first so the status below reads as unedited.
    m_body_editor_dirty = false;
    update_body_split_apply_enablement();
    m_body_panel->GetSizer()->Layout();
    Layout();
    Fit();
    clamp_dialog_width_to_display();
}

void PlateSettingsDialog::update_body_split_apply_enablement()
{
    if (m_body_panel == nullptr || !m_body_panel->IsShown())
        return;
    std::vector<BodySplitNativeSelection> selections;
    for (size_t i = 0; i < m_body_model.rows.size() && i < m_body_filament_choices.size() && i < m_body_cadence_choices.size(); ++i) {
        selections.push_back({m_body_model.rows[i].object_id, m_body_model.rows[i].volume_id,
                              m_body_filament_choices[i]->GetSelection(), m_body_cadence_choices[i]->GetSelection()});
        const int logical = m_body_filament_choices[i]->GetSelection();
        std::string material;
        wxString tool_text = _L("Not on a nozzle yet");
        if (logical >= 0 && size_t(logical) < wxGetApp().preset_bundle->filament_presets.size()) {
            material = filament_material_label(*wxGetApp().preset_bundle, size_t(logical));
            const auto resolution = resolve_mixed_nozzle_tool(m_body_resolver, size_t(logical), MixedNozzleResolveScope::PhysicalToolOnly);
            if (resolution.tool)
                tool_text = from_u8(std::string(resolution.tool->physical_extruder == 0 ? "left " : "right ") +
                                    mixed_nozzle_diameter_text(resolution.tool->nozzle_diameter) + " mm nozzle");
            else if (resolution.diagnostic)
                tool_text = from_u8(resolution.diagnostic->message());
        }
        m_body_material_labels[i]->SetLabel(from_u8(material));
        m_body_material_labels[i]->SetToolTip(from_u8(material));
        m_body_tool_labels[i]->SetLabel(tool_text);
        m_body_tool_labels[i]->SetToolTip(tool_text);
    }
    const auto diagnostic = body_split_native_selection_diagnostic(m_body_model.rows, selections,
        wxGetApp().preset_bundle->filament_presets.size(), m_body_resolver);
    wxString status = _L("Change a part's material or layer, then click OK.");
    switch (diagnostic) {
    case BodySplitEditorDiagnostic::None: break;
    case BodySplitEditorDiagnostic::ExactlyTwoModelPartsRequired: status = wxString::Format(_L("Body Split needs at least two parts, or one part with painted regions; found %zu. Paint regions for the fine nozzle with Color Painting, or use Split to parts."), m_body_model.rows.size()); break;
    case BodySplitEditorDiagnostic::LayerHeightInvalid: status = _L("A body has an invalid base or effective layer height"); break;
    case BodySplitEditorDiagnostic::LogicalFilamentInvalid: status = _L("Select a filament for each body"); break;
    case BodySplitEditorDiagnostic::DistinctLogicalFilamentsRequired: status = _L("Assign bodies to both nozzles"); break;
    case BodySplitEditorDiagnostic::PhysicalToolUnresolved: status = _L("A body's material is not assigned to a nozzle. Set Filament grouping to Custom and put each material on a nozzle."); break;
    case BodySplitEditorDiagnostic::DistinctPhysicalToolsRequired: status = _L("Bodies must print on both nozzles"); break;
    case BodySplitEditorDiagnostic::PhysicalNozzleOrderingInvalid: status = _L("The coarse body must print on the larger nozzle"); break;
    case BodySplitEditorDiagnostic::CadenceNotQualified: status = _L("Each layer height must be a whole multiple of the base layer height that its nozzle can print, and the base layer height must meet both nozzles' minimum"); break;
    case BodySplitEditorDiagnostic::StaleTarget: status = _L("The parts changed. Close and reopen Plate Settings."); break;
    case BodySplitEditorDiagnostic::DuplicateTarget: status = _L("A part is listed twice. Close and reopen Plate Settings."); break;
    case BodySplitEditorDiagnostic::PaintedFilamentPhysicalToolMismatch: status = _L("A painted material is not on a nozzle that can print its layer height"); break;
    }
    // OK applies a valid edit, so say so as soon as the edit is made.
    if (diagnostic == BodySplitEditorDiagnostic::None && m_body_editor_dirty) {
        set_body_split_status(_L("Body Split changes ready for OK"),
                              _L("OK saves the changes as one Undo step. Cancel drops them."), true);
        return;
    }
    set_body_split_status(status, status, diagnostic == BodySplitEditorDiagnostic::None);
}

bool PlateSettingsDialog::apply_body_split_editor()
{
    auto *plater = wxGetApp().plater();
    if (plater == nullptr)
        return false;
    auto &plate_list = plater->get_partplate_list();
    PartPlate *plate = plate_list.get_plate(m_body_target.plate_index);
    if (!body_split_editor_target_valid(m_body_target, plate_list.get_curr_plate_index(),
                                        plate == nullptr ? 0 : plate->id().id)) {
        set_body_split_status(_L("The plate changed. Close and reopen Plate Settings."),
                              _L("No settings were changed. Reopen Plate Settings for the current plate."), false);
        return false;
    }

    const DynamicPrintConfig full_config = wxGetApp().preset_bundle->full_config();
    PrintConfig resolver = mixed_nozzle_resolver_config_from_full(full_config);
    resolver.filament_map_mode.value = plate->get_real_filament_map_mode(wxGetApp().preset_bundle->project_config);
    resolver.filament_map.values = plate->get_real_filament_maps(wxGetApp().preset_bundle->project_config);
    const double inherited_height = full_config.has("layer_height") ? full_config.opt_float("layer_height") : 0.;
    const BodySplitEditorModel live = build_body_split_editor_model(plate->get_objects_on_this_plate(), resolver, {}, inherited_height);
    if (live.rows.size() != m_body_model.rows.size()) {
        set_body_split_status(_L("The parts changed while you were editing."),
                              _L("Nothing was changed. Cancel and reopen Plate Settings to load the current parts."), false);
        return false;
    }

    std::vector<BodySplitNativeSelection> selections;
    for (size_t i = 0; i < m_body_model.rows.size(); ++i)
        selections.push_back({m_body_model.rows[i].object_id, m_body_model.rows[i].volume_id,
                              m_body_filament_choices[i]->GetSelection(), m_body_cadence_choices[i]->GetSelection()});
    const auto live_request = build_body_split_live_request(live.rows, selections,
        wxGetApp().preset_bundle->filament_presets.size(), resolver,
        m_body_phase->GetSelection() == 1 ? RegionalGridPhaseRule::Rephase : RegionalGridPhaseRule::KeepNominal);
    if (!live_request.request) {
        set_body_split_status(_L("Body Split settings changed while you were editing."),
                              _L("Nothing was changed. Cancel and reopen Plate Settings to load the current parts, layers and nozzles."), false);
        return false;
    }
    BodySplitApplyRequest request = *live_request.request;
    request.inherited_layer_height = inherited_height;

    merge_body_split_fine_skins(request, m_body_model.rows,
        std::min(m_body_fine_skin_checks.size(), m_body_fine_skin_layer_spins.size()),
        [this](size_t i) { return m_body_fine_skin_checks[i]->GetValue(); },
        [this](size_t i) { return m_body_fine_skin_layer_spins[i]->GetValue(); });

    if (m_body_transaction.stage(request)) {
        set_body_split_status(_L("Body Split changes ready for OK"),
                              _L("OK saves the changes as one Undo step. Cancel drops them."), true);
        return true;
    }
    return false;
}

PlateSettingsDialog::~PlateSettingsDialog()
{
    m_feature_lifecycle.begin_shutdown();
    m_feature_fine = nullptr;
    m_feature_coarse = nullptr;
    m_feature_cadence = nullptr;
    m_feature_status = nullptr;
}

void PlateSettingsDialog::sync_bed_type(BedType type)
{
    if (m_bed_type_choice != nullptr) {
        for (int i = 0; i < m_cur_combox_bed_types.size(); i++) {
            if (m_cur_combox_bed_types[i] == type) {
                m_bed_type_choice->SetSelection(i + 1);//+1 because same as global
                return;
            }
        }
        m_bed_type_choice->SetSelection(0);
    }
}

void PlateSettingsDialog::sync_print_seq(int print_seq)
{
    if (m_print_seq_choice != nullptr) {
        m_print_seq_choice->SetSelection(print_seq);
    }
}

void PlateSettingsDialog::sync_first_layer_print_seq(int selection, const std::vector<int>& seq)
{
    if (m_first_layer_print_seq_choice != nullptr) {
        if (selection == 1) {
            const std::vector<std::string> extruder_colours = wxGetApp().plater()->get_extruder_colors_from_plater_config();
            m_drag_canvas->set_shape_list(extruder_colours, seq);
        }
        m_first_layer_print_seq_choice->SetSelection(selection);

        wxCommandEvent event(wxEVT_COMBOBOX);
        event.SetInt(selection);
        event.SetEventObject(m_first_layer_print_seq_choice);
        wxPostEvent(m_first_layer_print_seq_choice, event);
    }
}

void PlateSettingsDialog::sync_other_layers_print_seq(int selection, const std::vector<LayerPrintSequence>& seq) {
    if (selection == 1) {
        std::vector<LayerSeqInfo> sequences;
        sequences.reserve(seq.size());
        for (int i = 0; i < seq.size(); i++) {
            LayerSeqInfo info{ seq[i].first.first, seq[i].first.second, seq[i].second };
            sequences.push_back(info);
        }
        m_other_layers_seq_panel->sync_layers_print_seq(selection, sequences);
    }
    else {
        m_other_layers_seq_panel->sync_layers_print_seq(selection, {});
    }
}

void PlateSettingsDialog::sync_spiral_mode(bool spiral_mode, bool as_global)
{
    if (m_spiral_mode_choice) {
        if (as_global) {
            m_spiral_mode_choice->SetSelection(0);
        }
        else {
            if (spiral_mode)
                m_spiral_mode_choice->SetSelection(1);
            else
                m_spiral_mode_choice->SetSelection(2);
        }
    }
}

wxString PlateSettingsDialog::to_bed_type_name(BedType bed_type) {
    switch (bed_type) {
    case btDefault:
        return _L("Same as Global Plate Type");
    default: {
        const ConfigOptionDef *bed_type_def = print_config_def.get("curr_bed_type");
        return _(bed_type_def->enum_labels[size_t(bed_type) - 1]);
        }
    }
    return _L("Same as Global Plate Type");
}

wxString PlateSettingsDialog::to_print_sequence_name(PrintSequence print_seq) {
    switch (print_seq) {
    case PrintSequence::ByLayer:
        return _L("By Layer");
    case PrintSequence::ByObject:
        return _L("By Object");
    default:
        return _L("By Layer");
    }
    return _L("By Layer");
}

void PlateSettingsDialog::on_dpi_changed(const wxRect& suggested_rect)
{
}

wxString PlateSettingsDialog::get_plate_name() const {
    return m_ti_plate_name->GetTextCtrl()->GetValue(); 
}

void PlateSettingsDialog::set_plate_name(const wxString &name) { m_ti_plate_name->GetTextCtrl()->SetValue(name); }

BedType PlateSettingsDialog::get_bed_type_choice()
{
    if (m_bed_type_choice != nullptr) {
        int choice = m_bed_type_choice->GetSelection();
        if (choice > 0) {
            return m_cur_combox_bed_types[choice - 1];//-1 because same as globlal
        }
    }
    return BedType::btDefault;
};

std::vector<int> PlateSettingsDialog::get_first_layer_print_seq()
{
    return m_drag_canvas->get_shape_list_order();
}


//PlateNameEditDialog
PlateNameEditDialog::PlateNameEditDialog(wxWindow *parent, wxWindowID id, const wxString &title, const wxPoint &pos, const wxSize &size, long style)
    : DPIDialog(parent, id, title, pos, size, style)
{
    SetBackgroundColour(*wxWHITE);
    wxBoxSizer *m_sizer_main = new wxBoxSizer(wxVERTICAL);
    auto        m_line_top   = new wxPanel(this, wxID_ANY, wxDefaultPosition, wxSize(FromDIP(400), -1));
    m_line_top->SetBackgroundColour(wxColour(166, 169, 170));
    m_sizer_main->Add(m_line_top, 0, wxEXPAND, 0);
    m_sizer_main->Add(0, 0, 0, wxTOP, FromDIP(5));

    wxFlexGridSizer *top_sizer = new wxFlexGridSizer(0, 2, FromDIP(5), 0);
    top_sizer->AddGrowableCol(0, 1);
    top_sizer->SetFlexibleDirection(wxBOTH);
    top_sizer->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

    auto plate_name_txt = new wxStaticText(this, wxID_ANY, _L("Plate name"));
    plate_name_txt->SetFont(Label::Body_14);
    m_ti_plate_name = new TextInput(this, wxString::FromDouble(0.0), "", "", wxDefaultPosition, wxSize(FromDIP(240), -1), wxTE_PROCESS_ENTER);
    m_ti_plate_name->Bind(wxEVT_TEXT_ENTER, [this](wxCommandEvent &e) {
        if (this->IsModal())
            EndModal(wxID_YES);
        else
            this->Close();
    });
    top_sizer->Add(plate_name_txt, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_LEFT | wxALL, FromDIP(5));
    top_sizer->Add(m_ti_plate_name, 0, wxALIGN_CENTER_VERTICAL | wxALIGN_RIGHT | wxALL, FromDIP(5));
    m_ti_plate_name->GetTextCtrl()->SetMaxLength(250);

    m_sizer_main->Add(top_sizer, 0, wxEXPAND | wxALL, FromDIP(30));

    auto dlg_btns = new DialogButtons(this, {"OK", "Cancel"});

    dlg_btns->GetOK()->Bind(wxEVT_BUTTON, [this](wxCommandEvent &e) {
        if (this->IsModal())
            EndModal(wxID_YES);
        else
            this->Close();
    });

    dlg_btns->GetCANCEL()->Bind(wxEVT_BUTTON, [this](wxCommandEvent &e) {
        if (this->IsModal())
            EndModal(wxID_NO);
        else
            this->Close();
    });

    m_sizer_main->Add(dlg_btns, 0, wxEXPAND, FromDIP(20));

    SetSizer(m_sizer_main);
    Layout();
    m_sizer_main->Fit(this);

    CenterOnParent();

    wxGetApp().UpdateDlgDarkUI(this);
}

PlateNameEditDialog::~PlateNameEditDialog() {}

void PlateNameEditDialog::on_dpi_changed(const wxRect &suggested_rect)
{
}


wxString PlateNameEditDialog::get_plate_name() const { return m_ti_plate_name->GetTextCtrl()->GetValue(); }

void PlateNameEditDialog::set_plate_name(const wxString &name) {
    m_ti_plate_name->GetTextCtrl()->SetValue(name);
    m_ti_plate_name->GetTextCtrl()->SetFocus();
    m_ti_plate_name->GetTextCtrl()->SetInsertionPointEnd();
}


}
} // namespace Slic3r::GUI
