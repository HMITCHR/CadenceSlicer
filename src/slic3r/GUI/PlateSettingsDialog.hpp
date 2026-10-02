#ifndef slic3r_GUI_PlateSettingsDialog_hpp_
#define slic3r_GUI_PlateSettingsDialog_hpp_

#include "Plater.hpp"
#include "PartPlate.hpp"
#include "Widgets/Button.hpp"
#include "Widgets/RadioBox.hpp"
#include "Widgets/ComboBox.hpp"
#include "Widgets/CheckBox.hpp"
#include "Widgets/SpinInput.hpp"
#include "DragCanvas.hpp"
#include "BodySplitEditorModel.hpp"
#include "FeatureSplitEditorNative.hpp"
#include "libslic3r/ParameterUtils.hpp"

#include <wx/weakref.h>

namespace Slic3r { namespace GUI {

wxDECLARE_EVENT(EVT_SET_BED_TYPE_CONFIRM, wxCommandEvent);
wxDECLARE_EVENT(EVT_NEED_RESORT_LAYERS, wxCommandEvent);

bool dispatch_plate_settings_acceptance(wxEvtHandler& layer_sequence_handler,
                                        wxEvtHandler& dialog_handler,
                                        int event_id);

struct LayerSeqInfo {
    int begin_layer_number;
    int end_layer_number;
    std::vector<int> print_sequence;

    bool operator<(const LayerSeqInfo& another) const;
};

class LayerNumberTextInput : public ComboBox {
public:
    enum class Type {
        Begin,
        End
    };
    enum class ValueType {
        Custom,
        End
    };
    LayerNumberTextInput(wxWindow* parent, int layer_number, wxSize size, Type type, ValueType value_type = ValueType::Custom);
    void link(LayerNumberTextInput* layer_input) { 
        if (m_another_layer_input) return; 
        m_another_layer_input = layer_input; 
        layer_input->link(this); }
    void set_layer_number(int layer_number);
    int get_layer_number();
    Type get_input_type() { return m_type; }
    ValueType get_value_type() { return m_value_type; }
    bool is_layer_number_valid();

protected:
    void update_label();

private:
    LayerNumberTextInput* m_another_layer_input{ nullptr };
    int m_layer_number;
    Type m_type;
    ValueType m_value_type;
};

class OtherLayersSeqPanel : public wxPanel {
public:
    OtherLayersSeqPanel(wxWindow* parent);

    void sync_layers_print_seq(int selection, const std::vector<LayerSeqInfo>& seq);

    int get_layers_print_seq_choice() { return m_other_layer_print_seq_choice->GetSelection(); };

    std::vector<LayerSeqInfo> get_layers_print_seq_infos() { return m_layer_seq_infos; }

protected:
    void append_layer(const LayerSeqInfo* layer_info = nullptr);
    void popup_layer();
    void clear_all_layers();

private:
    ScalableBitmap  m_bmp_delete;
    ScalableBitmap  m_bmp_add;
    ComboBox* m_other_layer_print_seq_choice{ nullptr };
    wxPanel* m_layer_input_panel{ nullptr };
    std::vector<wxBoxSizer*> m_layer_input_sizer_list;
    std::vector<LayerNumberTextInput*> m_begin_layer_input_list;
    std::vector<LayerNumberTextInput*> m_end_layer_input_list;
    std::vector<DragCanvas*> m_drag_canvas_list;
    std::vector<LayerSeqInfo> m_layer_seq_infos;
};

class PlateSettingsDialog : public DPIDialog
{
public:
    enum ButtonStyle {
        ONLY_CONFIRM = 0,
        CONFIRM_AND_CANCEL = 1,
        MAX_STYLE_NUM = 2
    };
    PlateSettingsDialog(
        wxWindow* parent,
        const wxString& title = wxEmptyString,
        bool only_layer_seq = false,
        int target_plate_index = -1,
        size_t target_plate_id = 0,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long            style = wxCLOSE_BOX | wxCAPTION
    );

    ~PlateSettingsDialog();
    void sync_bed_type(BedType type);
    void sync_print_seq(int print_seq = 0);
    void sync_first_layer_print_seq(int selection, const std::vector<int>& seq = std::vector<int>());
    void sync_other_layers_print_seq(int selection, const std::vector<LayerPrintSequence>& seq);
    void sync_spiral_mode(bool spiral_mode, bool as_global);
    wxString to_bed_type_name(BedType bed_type);
    wxString to_print_sequence_name(PrintSequence print_seq);
    void on_dpi_changed(const wxRect& suggested_rect) override;

    int get_print_seq_choice() {
        int choice = 0;
        if (m_print_seq_choice != nullptr)
            choice =  m_print_seq_choice->GetSelection();
        return choice;
    }

    BedType get_bed_type_choice();

    wxString get_plate_name() const;
    void set_plate_name(const wxString& name);
    FeatureSplitStableTarget feature_split_target() const { return m_feature_target; }
    FeatureSplitSelection feature_split_selection() const { return m_feature_selection; }
    // The coarse-cadence selection as the mm height it represents, nullopt when nothing is
    // selected (e.g. no qualified cadence for the current topology).
    std::optional<double> feature_split_coarse_layer_height_choice() const;
    bool validate_feature_split_acceptance();
    void focus_feature_split_editor(const wxString& target);
    BodySplitPlateSettingsAcceptanceResult accept_body_split(const BodySplitStableTarget& target,
                                                              int current_plate_index,
                                                              size_t indexed_plate_id,
                                                              MixedNozzleSlicingMode effective_mode,
                                                              Model& model,
                                                              DynamicPrintConfig& plate_config,
                                                              const PrintConfig& resolver_config,
                                                              const BodySplitMutationHooks& hooks = {})
    { return accept_body_split_plate_settings(target, current_plate_index, indexed_plate_id,
                                               effective_mode, m_body_transaction, model, plate_config,
                                               resolver_config, hooks); }

    int get_first_layer_print_seq_choice() {
        int choice = 0;
        if (m_first_layer_print_seq_choice != nullptr)
            choice = m_first_layer_print_seq_choice->GetSelection();
        return choice;
    };

    int get_other_layers_print_seq_choice() {
        if (m_other_layers_seq_panel)
            return m_other_layers_seq_panel->get_layers_print_seq_choice();
        return 0;
    };

    std::vector<int> get_first_layer_print_seq();

    std::vector<LayerPrintSequence> get_other_layers_print_seq_infos() {
        const std::vector<LayerSeqInfo>& layer_seq_infos = m_other_layers_seq_panel->get_layers_print_seq_infos();
        std::vector<LayerPrintSequence> result;
        result.reserve(layer_seq_infos.size());
        for (int i = 0; i < layer_seq_infos.size(); i++) {
            LayerPrintSequence item = std::make_pair(std::make_pair(layer_seq_infos[i].begin_layer_number, layer_seq_infos[i].end_layer_number), layer_seq_infos[i].print_sequence);
            result.push_back(item);
        }
        return result;
    }

    int get_spiral_mode_choice() {
        int choice = 0;
        if (m_spiral_mode_choice != nullptr)
            choice = m_spiral_mode_choice->GetSelection();
        return choice;
    };

    bool get_spiral_mode(){
        return false;
    }

protected:
    void add_layers();
    void delete_layers();
    void refresh_body_split_editor();
    bool apply_body_split_editor();
    void update_body_split_apply_enablement();
    void set_body_split_status(const wxString& text, const wxString& tooltip, bool body_ok);
    void update_ok_button();
    void refresh_feature_split_editor(bool restore_live_selection = false);
    void update_feature_split_status();
    void set_feature_split_status(const std::string& full_status);
    // After a Fit() that can widen the dialog from its table or status content, clamp it back so
    // it never exceeds the display it opened on.
    void clamp_dialog_width_to_display();
    // Re-fit after a Show()/Hide() changed which rows the dialog carries, so hiding the
    // Mixed-Nozzle block shrinks the dialog instead of leaving a gap. No-op until the
    // constructor has called SetSizer().
    void relayout_after_visibility_change();

protected:
    ComboBox* m_bed_type_choice { nullptr };
    std::vector<BedType> m_cur_combox_bed_types;
    ComboBox* m_print_seq_choice { nullptr };
    ComboBox* m_first_layer_print_seq_choice { nullptr };
    ComboBox* m_spiral_mode_choice { nullptr };
    DragCanvas* m_drag_canvas;
    OtherLayersSeqPanel* m_other_layers_seq_panel;
    TextInput *m_ti_plate_name;

    // The whole Mixed-Nozzle block: hidden together with its editors on a printer that does not
    // present a supported two-physical-tool topology (feature_split_plate_settings_visibility()).
    wxStaticText* m_mixed_label {nullptr};
    wxPanel* m_mixed_panel {nullptr};
    // The Feature rows show only after "Edit here" in the More menu, as in the sidebar.
    bool m_feature_edit_here {false};
    bool m_feature_edit_here_offered {false};
    void set_feature_edit_here(bool edit_here);
    void show_mixed_more_menu(wxWindow* anchor);

    FeatureSplitStableTarget m_feature_target;
    FeatureSplitSelection m_feature_selection;
    FeatureSplitNativeLifecycle m_feature_lifecycle;
    wxStaticText* m_feature_fine_label {nullptr};
    wxStaticText* m_feature_coarse_label {nullptr};
    wxStaticText* m_feature_status_label {nullptr};
    wxWeakRef<ComboBox> m_feature_fine;
    wxWeakRef<ComboBox> m_feature_coarse;
    wxWeakRef<wxStaticText> m_feature_status;
    // "Coarse cadence" combo, from feature_split_coarse_cadence_choices() filtered to ratio >= 2.
    // m_feature_cadence_values maps combo index to mm height.
    wxStaticText* m_feature_cadence_label {nullptr};
    wxWeakRef<ComboBox> m_feature_cadence;
    std::vector<double> m_feature_cadence_values;

    BodySplitStableTarget m_body_target;
    wxPanel* m_body_panel {nullptr};
    wxFlexGridSizer* m_body_table {nullptr};
    ComboBox* m_body_phase {nullptr};
    wxStaticText* m_body_status {nullptr};
    Button* m_ok_button {nullptr};
    // False while the shown Body table holds an edit OK cannot save.
    bool m_body_ok {true};
    Button* m_body_cancel {nullptr};
    BodySplitEditorModel m_body_model;
    PrintConfig m_body_resolver;
    std::vector<ComboBox*> m_body_filament_choices;
    std::vector<ComboBox*> m_body_cadence_choices;
    std::vector<wxStaticText*> m_body_material_labels;
    std::vector<wxStaticText*> m_body_tool_labels;
    // Fine-skin checkbox and skin-layer spinner, one pair per row, enabled only for a
    // coarse-cadence row (BodySplitEditorRow::fine_skin_controls_applicable).
    std::vector<CheckBox*> m_body_fine_skin_checks;
    std::vector<SpinInput*> m_body_fine_skin_layer_spins;
    BodySplitEditorTransaction m_body_transaction;
    // Set by every Body Split editor widget's change handler and cleared when the rows are rebuilt.
    // has_pending() stays false until OK stages, so this is what stops a window activation from
    // discarding an unsaved edit.
    bool m_body_editor_dirty {false};
};

class PlateNameEditDialog : public DPIDialog
{
public:
    enum ButtonStyle { ONLY_CONFIRM = 0, CONFIRM_AND_CANCEL = 1, MAX_STYLE_NUM = 2 };
    PlateNameEditDialog(wxWindow *      parent,
                        wxWindowID      id    = wxID_ANY,
                        const wxString &title = wxEmptyString,
                        const wxPoint & pos   = wxDefaultPosition,
                        const wxSize &  size  = wxDefaultSize,
                        long            style = wxCLOSE_BOX | wxCAPTION);

    ~PlateNameEditDialog();
    void     on_dpi_changed(const wxRect &suggested_rect) override;

    wxString get_plate_name() const;
    void     set_plate_name(const wxString &name);

protected:
    TextInput *m_ti_plate_name;
};
}} // namespace Slic3r::GUI

#endif
