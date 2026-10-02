#ifndef slic3r_GUI_MixedNozzleSidebarPanel_hpp_
#define slic3r_GUI_MixedNozzleSidebarPanel_hpp_

// The "Mixed Nozzle" sidebar section: mode, per-body assignments and cadence in one place.
// Each body is a vertical card because the Plate Settings body grid is wider than the sidebar.
// Apply stages the request on native copies and publishes it through the shared assignment
// transaction, so one click is one Undo step. This panel and PlateSettingsDialog both
// re-validate against live state before writing, so a lost race is rejected, not written.

#include "FeatureSplitEditorNative.hpp"
#include "MixedNozzleSidebarController.hpp"
#include "Widgets/Button.hpp"
#include "Widgets/CheckBox.hpp"
#include "Widgets/ComboBox.hpp"
#include "Widgets/SpinInput.hpp"
#include "libslic3r/MixedNozzleBinding.hpp"

#include <wx/panel.h>
#include <wx/scrolwin.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r {

class PresetBundle;

namespace GUI {

class Plater;
class PartPlate;
struct MixedNozzleAssignmentStageResult;
struct MixedNozzleAssignmentBindingStageState;

enum class MixedNozzleRebindReviewStatus {
    NotNeeded,
    Applied,
    Cancelled,
    Stale,
};

struct MixedNozzleRebindReviewResult {
    MixedNozzleRebindReviewStatus status {MixedNozzleRebindReviewStatus::NotNeeded};
    std::optional<MixedNozzleRebindPlan> plan;
    std::vector<bool> accepted_rebind;
};

// Show the native per-entry Rebind review and return the checked rows. Callers stage the plan
// together with their Body/Feature request, so Cancel leaves assignment and material state
// untouched. An optional expected signature rejects a target captured before the review.
MixedNozzleRebindReviewResult review_mixed_nozzle_rebind(
    wxWindow *parent, PresetBundle &bundle, PartPlate &plate,
    const MixedNozzleRebindSignature *expected_signature = nullptr);

// Assignment-binding stages for the inline editors: resolve on the staged copies and validate the
// request there. The Body stage reports its validator result through `diagnostic`.
std::function<MixedNozzleAssignmentStageResult(MixedNozzleAssignmentBindingStageState &)>
make_body_split_stage_assignment(BodySplitApplyRequest request, BodySplitEditorDiagnostic &diagnostic);
std::function<MixedNozzleAssignmentStageResult(MixedNozzleAssignmentBindingStageState &)>
make_feature_split_stage_assignment(FeatureSplitApplyRequest request, MixedNozzleSlicingMode effective_mode,
                                    size_t nozzle_count);

// Scrolls only when the sidebar cannot give it its full height, so a short window never
// squashes the editor or pushes the Process section out of the window.
class MixedNozzleSidebarPanel : public wxScrolledWindow
{
public:
    MixedNozzleSidebarPanel(wxWindow *parent, Plater *plater);

    // Called by the sidebar's bounded timer; unrelated child UPDATE_UI events must not
    // repeatedly compose the full preset bundle. Native Undo/Redo explicitly reloads.
    void poll_refresh();
    void reload();

    // What the section currently shows. Hidden means the caller must hide the title and the
    // separator too, or a single-nozzle printer grows an empty "Mixed Nozzle" header.
    MixedNozzleSidebarSection current_section() const { return m_section; }

    // Click-to-collapse state, owned here so a signature-driven rebuild cannot silently
    // re-expand a section the user folded away.
    void set_collapsed(bool collapsed);
    bool is_collapsed() const { return m_collapsed; }

    void msw_rescale();
    void sys_color_changed();

private:
    struct BodyRowControls {
        wxPanel *card {nullptr};
        wxStaticText *identity {nullptr};
        wxStaticText *caption {nullptr};
        ComboBox *filament {nullptr};
        ComboBox *cadence {nullptr};
        // ::CheckBox is the Widgets/ toggle button, not the Field subclass Slic3r::GUI::CheckBox.
        ::CheckBox *fine_skins {nullptr};
        SpinInput *fine_skin_layers {nullptr};
        wxStaticText *fine_skin_label {nullptr};
    };

    MixedNozzleSidebarSignature collect_signature() const;
    void reject_changed_settings();
    void rebuild();
    void rebuild_body_cards();
    void rebuild_feature_rows();
    void rebuild_feature_cadences(const DynamicPrintConfig &full_config, bool preserve_draft);
    void update_status_and_enablement();
    void on_apply();
    void show_time_decisions();
    void on_rebind();
    // The "More" menu (grouping, material settings, Edit here, About) and its actions.
    void show_more_menu();
    void on_grouping();
    void set_edit_here(bool edit_here);
    // True when applied; `tooltip` then says what Undo restores.
    bool apply_body(wxString &tooltip);
    bool apply_feature(wxString &tooltip);
    // "Changes applied" in place of a ready status, while the applied state is still on screen.
    void set_ready_status(const wxString &ready, bool enable_apply);
    // Rebind review shared by both applies. An approved plan is composed on `reviewed_bundle`, a
    // copy, so validation sees the staged material limits. False after setting the status.
    bool review_binding(PresetBundle &bundle, PartPlate &plate, MixedNozzleRebindReviewResult &review,
                        std::optional<PresetBundle> &reviewed_bundle, const wxString &unchanged,
                        const wxString &stale);
    void mark_dirty();
    void set_status(const wxString &text, const wxString &tooltip, bool enable_apply);
    void show_section(MixedNozzleSidebarSection section);

    Plater *m_plater {nullptr};

    // Re-seeded from the current plate on every rebuild, so a plate switch re-targets the editor.
    BodySplitStableTarget m_target;
    MixedNozzleSidebarSignature m_signature;
    MixedNozzleSidebarSection m_section {MixedNozzleSidebarSection::Hidden};

    // The user is mid-edit: an idle rebuild must never eat an unapplied selection. A
    // plate/mode/topology switch overrides it.
    bool m_editor_dirty {false};
    // The state an inline Apply produced. The status says "Changes applied" while the panel still
    // shows it and nothing was edited since.
    std::optional<MixedNozzleSidebarSignature> m_applied_signature;
    wxString m_applied_tooltip;
    bool m_collapsed {false};

    wxBoxSizer *m_sizer {nullptr};

    // The summary line, then lines shown only when they apply.
    wxStaticText *m_mode_status {nullptr};
    wxStaticText *m_plate_line {nullptr};
    wxStaticText *m_guard_line {nullptr};
    // The prime tower before slicing, and what the last slice produced.
    wxStaticText *m_tower_status {nullptr};
    wxStaticText *m_sliced_line {nullptr};
    wxStaticText *m_coarse_line {nullptr};
    wxStaticText *m_tower_readout {nullptr};
    wxStaticText *m_rebind_line {nullptr};
    // The summary, readout and status lines wrap to the panel width instead of being cut off. Each
    // keeps its unwrapped text, so a width change can wrap it again. A line that shares its row
    // with a button leaves room for it.
    struct WrappedLine {
        wxStaticText *line {nullptr};
        wxString text;
        wxWindow *beside {nullptr};
    };
    std::vector<WrappedLine> m_wrapped_lines;
    void add_wrapped_line(wxStaticText *line, wxWindow *beside = nullptr);
    int m_wrap_width {0};
    int line_wrap_width() const;
    void set_wrapped_label(wxStaticText *line, const wxString &text);
    void rewrap_lines();
    // Lays the panel out again, and the sidebar too when the panel's height changed. Every
    // change to a line or an editor row ends here.
    void relayout();
    // After a status change in a scrolled section, scrolls down just enough to show the status
    // line and Apply.
    void keep_actions_in_view();
    // The height the other sidebar sections leave free.
    int height_limit() const;
    wxSize DoGetBestSize() const override;
    Button *m_mode_off {nullptr};
    std::size_t m_tower_readout_key {0};
    // What the last slice produced, empty while there is none. Cached so a refresh that only
    // re-lays out cannot lose it.
    MixedNozzleSidebarFacts m_slice_facts;
    void refresh_tower_readout(PartPlate *plate);
    // Builds the lines above the editor, after a finished slice or a wizard Apply alike.
    void refresh_readouts();
    Button *m_mode_setup {nullptr};
    Button *m_more {nullptr};
    Button *m_rebind {nullptr};
    // The inline editor sits behind "Edit here" in the More menu. Kept across rebuilds.
    bool m_edit_here {false};
    bool m_rebind_offered {false};

    wxPanel *m_body_panel {nullptr};
    wxBoxSizer *m_body_sizer {nullptr};
    std::vector<BodyRowControls> m_body_rows;
    BodySplitEditorModel m_body_model;
    PrintConfig m_body_resolver;

    wxPanel *m_feature_panel {nullptr};
    wxStaticText *m_feature_fine_label {nullptr};
    wxStaticText *m_feature_coarse_label {nullptr};
    ComboBox *m_feature_fine {nullptr};
    ComboBox *m_feature_coarse {nullptr};
    ComboBox *m_feature_cadence {nullptr};
    std::vector<double> m_feature_cadence_values;
    FeatureSplitSelection m_feature_selection;

    wxStaticText *m_status {nullptr};
    Button *m_apply {nullptr};
    Button *m_reload {nullptr};
    Button *m_time_decisions {nullptr};
};

} // namespace GUI
} // namespace Slic3r

#endif
