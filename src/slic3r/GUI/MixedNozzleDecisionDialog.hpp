#ifndef slic3r_GUI_MixedNozzleDecisionDialog_hpp_
#define slic3r_GUI_MixedNozzleDecisionDialog_hpp_

// One dialog, opened from the Prepare sidebar button, Preview's Physical Tool legend and the
// post-slice notification. Every other surface reads the same MixedNozzleDecisionReport aggregates,
// so none can answer "why fine or coarse here?" differently.

#include "MixedNozzleDecisionReport.hpp"

#include <vector>

class wxWindow;

namespace Slic3r { namespace GUI {

class Plater;

// Everything the report surfaces need from one plate, gathered once.
struct MixedNozzleDecisionData {
    bool available {false};
    // A mixed-nozzle Feature Split plate, whether or not it produced usable evidence.
    bool feature_split {false};
    MixedNozzleDecisionSummary               summary;
    std::vector<MixedNozzleDecisionBandRow>  bands;
    std::vector<MixedNozzleDecisionRangeRow> ranges;
};

// require_valid_slice_result guards against reporting an older slice: a snapshot can outlive its
// G-code. The one caller passing false is the end-of-slice push, which runs before the plate is
// marked valid; the dialog it opens re-checks with the guard on.
MixedNozzleDecisionData collect_mixed_nozzle_decision_data(Plater *plater, bool require_valid_slice_result = true);

// Opens the shared report dialog, or an explanatory message when there is nothing to show.
void show_mixed_nozzle_decision_dialog(wxWindow *parent);

// The layer-slider notes for the current plate; empty when there is nothing to annotate.
std::vector<MixedNozzleBandNote> collect_mixed_nozzle_band_notes(Plater *plater);

}} // namespace Slic3r::GUI

#endif // slic3r_GUI_MixedNozzleDecisionDialog_hpp_
