#ifndef slic3r_GUI_MixedNozzleSetupDialog_hpp_
#define slic3r_GUI_MixedNozzleSetupDialog_hpp_

#include "GUI_Utils.hpp"
#include "MixedNozzleNativeEntry.hpp"
#include "MixedNozzleRankingController.hpp"
#include "MixedNozzleWizardModel.hpp"
#include "BodySplitEditorModel.hpp"

#include <array>
#include <functional>
#include <memory>
#include <set>
#include <thread>
#include <utility>
#include <vector>
#include <wx/string.h>

class wxStaticText;
class wxGauge;
class wxTimer;
class wxRadioButton;
class ComboBox;
class Button;
class wxSimplebook;
class wxChoice;
class wxCheckBox;
class wxTextCtrl;
class wxButton;
class wxCollapsiblePane;
class wxPanel;

namespace Slic3r::GUI {

// The wizard works on copied values; the Plater validates and publishes after it returns, so
// closing or cancelling changes nothing.
struct WizardBindingReview {
    std::optional<WizardNativeBindingApproval> approval;
    FullPrintConfig effective_config;
    std::vector<WizardKeyDelta> deltas;
    std::vector<std::string> notes;
    std::string diagnostic;
};

struct WizardNativeReview {
    std::vector<std::string> notes;
    // Every value the automatic tower policy owns, with its provenance.
    std::vector<std::string> tower_lines;
    std::string diagnostic;
    // What the Review summary needs from the prepared Apply: the process preset before and after,
    // the first layer the process will print, and the values Apply corrects on its own.
    std::string process_before;
    std::string process_after;
    double first_layer_height {0.};
    double first_layer_speed {0.};
    std::vector<std::string> also_changed;
    // The engine's refusal, apart from a setup diagnostic. `diagnostic` carries it too.
    std::string engine_refusal;
    // The Process setting that refusal names.
    std::string engine_refusal_key;
};

// The step the wizard opens on, the one the entry point's question belongs to.
enum class MixedNozzleWizardPage : int {
    ModeAndScope = 0,
    Materials = 1,
    // "Detail and speed": the fine layer and the coarse layer on one page.
    Speed = 2,
    Review = 3,
    // Reached from any step with "More options"; "Done" returns to that step.
    MoreOptions = 4,
};

// What each tower option applies or preserves, rendered under the control.
struct MixedNozzleTowerChoicePresentation {
    bool automatic_default {false};
    std::string automatic_explanation;
    std::string keep_explanation;
};

struct MixedNozzleWizardDialogInput {
    WizardDraft draft;
    MixedNozzleWizardPage start_page {MixedNozzleWizardPage::ModeAndScope};
    // An entry carrying a new project nozzle pair is a project edit: the controller refuses a pair
    // outside project scope, so the scope control is fixed.
    bool project_scope_locked {false};
    FullPrintConfig effective_config;
    std::vector<WizardCatalogueRow> catalogue;
    std::vector<BodySplitEditorRow> body_rows;
    // Each body row's volume in mm3, parallel to body_rows, for the default Fine part.
    std::vector<double> body_volumes;
    // Each part's size in mm (x, y, z), parallel to body_rows.
    std::vector<std::array<double, 3>> body_sizes;
    // Select one part in the 3D view behind the dialog. The Plater restores the user's selection
    // when the dialog closes.
    std::function<void(std::size_t object_id, std::size_t volume_id)> highlight_part;
    // The mode the project slices with now; "Print with one nozzle instead" shows when set up.
    MixedNozzleSlicingMode configured_mode {MixedNozzleSlicingMode::Off};
    // Assemblies whose bodies touch across the two nozzles start on beam interlocking; others keep
    // their current settings.
    std::set<std::size_t> touching_objects;
    std::vector<std::string> filament_labels;
    // Shown under the material pickers when setup had to put a material on a nozzle itself.
    std::string materials_note;
    // True when this opening counts as the first-run introduction, so the Plater records it as
    // seen.
    bool show_intro_line {false};
    // filament_colour per slot for the material picker squares. A short or empty list falls back to
    // grey.
    std::vector<std::string> filament_colours;
    // The first layer the selected process preset itself states.
    double preset_first_layer_height {0.};
    double preset_first_layer_speed {0.};
    MixedNozzleTowerChoicePresentation tower;
    std::function<WizardBindingReview(const std::vector<std::size_t> &)> review_binding;
    std::function<WizardNativeReview(const WizardDraft &, const WizardReview &)> review_native;
    // The slice one cadence row would run with if Apply published it. Empty turns ranking off.
    std::function<MixedNozzleRankingSlice(const WizardDraft &, const WizardCandidate &,
                                          const std::vector<WizardBodyAssignment> &)> compose_ranking_slice;
    // True while the plater's own slice runs; ranking waits for it.
    std::function<bool()> real_slice_running;
    // Estimates kept across wizard openings, by plate, model and config.
    std::shared_ptr<WizardEstimateCache> ranking_cache;
    // The process preset every coarse layer row uses, read the way Apply reads it. Empty hides the
    // line.
    std::function<WizardProcessBasis(const WizardDraft &)> process_basis;
    std::string current_process_name;
};

// The one page "?" opens, from the Printer row or step 1. It reads and writes nothing.
class MixedNozzleIntroDialog final : public DPIDialog
{
public:
    explicit MixedNozzleIntroDialog(wxWindow *parent);
    void on_dpi_changed(const wxRect &) override {}
};

class MixedNozzleWizardDialog final : public DPIDialog
{
public:
    MixedNozzleWizardDialog(wxWindow *parent, MixedNozzleWizardDialogInput input);
    ~MixedNozzleWizardDialog() override;
    // Leaving by any button stops the ranking slice before the caller acts.
    void EndModal(int ret_code) override;
    const WizardDraft &draft() const { return m_input.draft; }
    const WizardReview &review() const { return m_review; }
    bool slice_after_apply() const { return m_slice_after_apply; }
    // The Process setting the user asked to open from Review. Nothing was written; the caller opens
    // it.
    const std::string &jump_option() const { return m_jump_option; }
    void on_dpi_changed(const wxRect &) override;

private:
    friend struct TestModeWizardAccess;
    MixedNozzleSlicingMode selected_mode() const;
    // Derive the plate filament map from the pickers, put it on the draft and compose it into the
    // effective config.
    void refresh_derived_map();
    void refresh_resolved_tools();
    bool capture_assignments();
    void refresh_fine_heights();
    bool selected_fine_height(double &height);
    bool refresh_candidates();
    // Maps the selected page row back to its candidate. Null when nothing usable is selected.
    const WizardCandidate *selected_candidate() const;
    bool refresh_review();
    // Lay the summary out on the Review page.
    void show_review_summary(const WizardReviewSummary &summary);
    // Which nozzle each Body Split body prints on, from the Materials page. False when a body has
    // no material with a resolved nozzle.
    bool body_assignments(std::vector<WizardBodyAssignment> &body) const;
    // Ask for estimates of the rows, rebuild the page as they arrive, compose one row's slice, and
    // stop everything when the dialog goes.
    void request_ranking();
    void refresh_ranking();
    MixedNozzleRankingSlice compose_ranking_row(const std::string &row_id);
    void stop_ranking();
    void navigate(int direction);
    // More options is its own page, left with "Done" for the step it was opened from. It opens at
    // the top, or at Joining for step 1's "Change joining...".
    void open_more_options(bool at_joining = false);
    void scroll_more_options(bool at_joining);
    void close_more_options();
    // The More options controls into the draft.
    void read_more_options();
    void refresh_navigation();
    void clamp_to_display();

    // wxStaticText::Wrap() is one-shot. Every wxStaticText on a page, and the status line under the
    // pages, is rewrapped from its unwrapped text to the width it has on its page whenever that
    // page is laid out or resized and whenever its text is set. The unwrapped text is kept on the
    // label itself, so a label made or destroyed later needs no registering. A plain SetLabel() is
    // noticed at the next rewrap.
    wxWindow *page_of(wxWindow *window);
    // Room kept on the right beyond the label's own border: a button beside it, or the border of a
    // panel it sits in.
    void reserve_wrap(wxStaticText *text, int right_reserve);
    void set_wrapped(wxStaticText *text, const wxString &label);
    int wrap_width(wxStaticText *text, int right_reserve);
    bool rewrap(wxStaticText *text);
    // Every label on a page, or the status line when the container is the dialog.
    bool rewrap_labels(wxWindow *container);
    // Lay a page out, wrap its labels to the positions that gives, and lay it out again when a
    // wrap changed.
    void relayout_page(wxWindow *page);

    MixedNozzleWizardDialogInput m_input;
    FullPrintConfig m_original_effective;
    std::vector<std::string> m_binding_notes;
    WizardReview m_review;
    std::vector<WizardCandidate> m_candidates;
    WizardCadencePage m_cadence_page;
    wxSimplebook *m_pages {nullptr};
    // The modes are cards, so the mode is a radio pair rather than a combo.
    wxRadioButton *m_mode_feature {nullptr};
    wxRadioButton *m_mode_body {nullptr};
    wxChoice *m_scope {nullptr};
    wxChoice *m_tower {nullptr};
    wxStaticText *m_tower_explanation {nullptr};
    wxCheckBox *m_rebind {nullptr};
    // Bitmap combos rather than wxChoice, so each row can carry its slot colour.
    ::ComboBox *m_fine_filament {nullptr};
    ::ComboBox *m_coarse_filament {nullptr};
    wxStaticText *m_fine_resolved {nullptr};
    wxStaticText *m_coarse_resolved {nullptr};
    wxWindow *m_body_assignments {nullptr};
    // Per Body Split part: Fine and Coarse on step 1, the kept-slot note, and the exact slot on
    // More options (0 = set by Fine or Coarse).
    std::vector<std::pair<wxRadioButton *, wxRadioButton *>> m_body_roles;
    std::vector<wxStaticText *> m_body_role_notes;
    std::vector<wxChoice *> m_body_slot_overrides;
    wxPanel *m_body_slots_panel {nullptr};
    wxStaticText *m_joining_summary {nullptr};
    wxButton *m_off_link {nullptr};
    bool m_off_chosen {false};
    std::vector<WizardBodyRoleRow> body_role_rows() const;
    std::vector<WizardBodyRole> body_roles() const;
    void apply_default_body_roles();
    void refresh_body_roles_view();
    std::vector<std::pair<std::size_t, wxChoice *>> m_body_joining;
    // The per-assembly joining list on More options, shown for Body Split.
    wxPanel *m_joining_panel {nullptr};
    // The stock heights offered by the dropdown, parallel to m_fine_height_choice's items. The
    // trailing "Custom..." item has no entry here and reveals m_fine_height instead.
    std::vector<double> m_fine_height_values;
    // Which nozzle the list belongs to, and what a typed height is measured against.
    wxStaticText *m_fine_nozzle_line {nullptr};
    // The process basis line, and the More options switch to keep the current preset.
    wxStaticText *m_basis_line {nullptr};
    wxCheckBox *m_keep_process {nullptr};
    wxChoice *m_fine_height_choice {nullptr};
    wxTextCtrl *m_fine_height {nullptr};
    wxStaticText *m_fine_height_status {nullptr};
    // The help under the fine layer dropdown. Its last clause names the coarse role of the mode.
    wxStaticText *m_fine_help {nullptr};
    // What N is, under the "Coarse layer" heading.
    wxStaticText *m_coarse_help {nullptr};
    // The coarse layer rows as radio buttons with their tags, and which is selected.
    wxPanel *m_speed_rows_panel {nullptr};
    std::vector<wxRadioButton *> m_speed_buttons;
    int m_speed_selection {-1};
    wxStaticText *m_speed_summary {nullptr};
    wxButton *m_one_nozzle_link {nullptr};
    void rebuild_speed_rows(std::optional<std::size_t> select);
    // The width the rows have, what the rows and their tags need side by side, and whether the tags
    // sit beside the rows or under them.
    int speed_rows_width();
    int m_speed_rows_needed {0};
    bool m_speed_tags_beside {true};
    void choose_one_nozzle();
    // The coarse layers the list leaves out and why, on More options and as the list's tooltip.
    wxStaticText *m_cadence_exclusions {nullptr};
    // The ratio in effect before the fine height was last edited, so refresh_candidates() can
    // re-select it at the new height after the draft's coarse height and ratio are cleared.
    std::optional<int> m_previous_cadence_ratio;
    // The fine and coarse layer of a project already set up. Empty on a fresh setup.
    std::optional<std::pair<double, double>> m_existing_layers;
    // Step 2 follows the parts' own materials until a material is picked by hand.
    bool m_materials_picked_by_hand {false};
    // What step 2's defaults did when the fine and coarse parts share a slot. Dropped once a
    // material is picked by hand.
    std::string m_role_default_note;
    void default_materials_from_parts();
    // The baseline line and the live ranking status under the cadence list.
    wxStaticText *m_cadence_time {nullptr};
    // The times are for the preset's current settings, and how to see them again.
    wxStaticText *m_times_note {nullptr};
    // Rows done, as a bar under the progress line.
    wxGauge *m_times_gauge {nullptr};
    std::shared_ptr<MixedNozzleRankingController> m_ranking;
    wxTimer *m_ranking_timer {nullptr};
    // The one slice running for the ranking, joined before the next starts and when the dialog goes.
    std::thread m_ranking_thread;
    // The row picked by hand in this session, by stable id. Arriving estimates never move the
    // selection off it; a static default is not a pick.
    std::optional<std::string> m_cadence_picked_id;
    // The baseline line, the ranking status and the note on the selected row.
    void refresh_cadence_time_line();
    // Next on Detail and speed waits while the selected row is only the default and has no time.
    void refresh_speed_next();
    // The reason refresh_speed_next() put on the status line, so it clears only its own.
    wxString m_speed_block_text;
    // The Review page: a header, the labelled rows, "Also changed", the ready line with a link to
    // the fixing step, and every setting in a closed pane.
    wxStaticText *m_review_header {nullptr};
    wxPanel *m_review_rows {nullptr};
    wxStaticText *m_review_also {nullptr};
    wxStaticText *m_ready_line {nullptr};
    wxButton *m_fix_link {nullptr};
    WizardFixTarget m_fix_target {WizardFixTarget::None};
    wxCollapsiblePane *m_details_pane {nullptr};
    wxTextCtrl *m_review_text {nullptr};
    // Shown only when the heights reach a process other plates share.
    wxCheckBox *m_shared_consent {nullptr};
    wxStaticText *m_status {nullptr};
    wxButton *m_more {nullptr};
    wxButton *m_done {nullptr};
    int m_more_return_page {0};
    wxButton *m_back {nullptr};
    wxButton *m_next {nullptr};
    wxButton *m_apply {nullptr};
    wxButton *m_apply_slice {nullptr};
    bool m_slice_after_apply {false};
    std::string m_jump_option;
    std::string m_fix_option;
};

class DissimilarNozzleDialog final : public DPIDialog
{
public:
    DissimilarNozzleDialog(wxWindow *parent, const DissimilarNozzleSelection &selection, bool setup_supported,
                           const std::vector<double> &previous_pair = {});
    DissimilarNozzleDecision decision() const { return m_decision; }
    // "Set up mixed-nozzle slicing now", read when the answer is Keep both nozzles.
    bool set_up_now() const { return m_set_up_now; }
    void on_dpi_changed(const wxRect &) override {}

private:
    DissimilarNozzleDecision m_decision {DissimilarNozzleDecision::Cancel};
    bool m_set_up_now {true};
};

} // namespace Slic3r::GUI
#endif
