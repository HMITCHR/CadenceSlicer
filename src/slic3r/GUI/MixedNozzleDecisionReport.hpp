#ifndef slic3r_GUI_MixedNozzleDecisionReport_hpp_
#define slic3r_GUI_MixedNozzleDecisionReport_hpp_

// Why each layer band used the fine or coarse nozzle.
//
// Everything the report says is a pure function of the published AdvisorFeatureBand evidence plus
// three plate-level numbers, free of wxWidgets, Print and PartPlate, so every surface says the same
// thing and the wording can be tested headlessly.
//
// The stored AdvisorFeatureBand::reason strings are matched verbatim elsewhere; they are mapped to
// plain words here and never rewritten at the source.

#include "libslic3r/MixedNozzleAdvisorObservations.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r { namespace GUI {

// The one-word answer to "why". Mapped once, in one place, for every surface.
enum class MixedNozzleDecisionReason {
    // The comparison ran and coarse won: this band's infill paid for the hand-off.
    SavesTime,
    // The comparison ran and fine won: switching here would have cost more than it saved.
    CostsMore,
    // The band never reached the comparison: native geometry or cadence required fine.
    NotEligible,
    // The comparison could not be priced (no tower, no native cost) and the band fell back.
    Fallback,
    // No decision evidence at all.
    Unavailable,
};

// One published band. Seconds are absent exactly where the engine's evidence is absent; never
// substitute zero.
struct MixedNozzleDecisionBandRow {
    std::size_t band_index {0};   // 1-based, as published
    std::size_t region_id {0};
    double      z_low {0.};
    double      z_high {0.};
    bool        coarse {false};
    std::string decision;         // "Coarse" or "Fine"
    std::optional<double> fine_seconds;
    std::optional<double> coarse_seconds;
    std::optional<double> switch_seconds;
    std::optional<double> tower_seconds;
    std::optional<double> handoff_seconds;
    std::optional<double> tower_volume_mm3;
    // fine - coarse - hand-off. Positive means using the coarse nozzle here saved time.
    std::optional<double> net_seconds;
    MixedNozzleDecisionReason reason {MixedNozzleDecisionReason::Unavailable};
    std::string reason_word;
    std::string raw_reason;       // the engine's diagnostic string, unaltered
};

// Consecutive bands that got the same answer for the same reason, collapsed into one row.
struct MixedNozzleDecisionRangeRow {
    double      z_low {0.};
    double      z_high {0.};
    std::string decision;
    MixedNozzleDecisionReason reason {MixedNozzleDecisionReason::Unavailable};
    std::string reason_word;
    std::size_t band_count {0};
    std::size_t first_band_index {0};
    std::size_t last_band_index {0};
    std::optional<double> min_net_seconds;
    // Arithmetic mean of the priced bands in this range.
    std::optional<double> typical_net_seconds;
    std::optional<double> max_net_seconds;
};

// Plate-level numbers the report cannot derive from bands. Tower material is the final tower's real
// material (PrintStatistics::wipe_tower_volumes_per_extruder), never a sum of per-band trial
// deltas.
struct MixedNozzleDecisionInput {
    const AdvisorSliceObservations *observations {nullptr};
    std::optional<std::size_t>      physical_tool_changes;
    std::optional<double>           tower_volume_mm3;
    std::optional<double>           tower_mass_g;
    // The coarse nozzle's diameter, for the after-slice headline. Absent reads "the coarse nozzle".
    std::optional<double>           coarse_nozzle_diameter;
};

struct MixedNozzleDecisionSummary {
    bool        available {false};
    std::size_t band_count {0};
    std::size_t coarse_band_count {0};
    std::optional<double> coarse_z_low;
    // The last Z at which the coarse nozzle was used; the "above this, ..." line explains it.
    std::optional<double> coarse_z_high;
    std::optional<double> estimated_saving_seconds;
    std::optional<std::size_t> physical_tool_changes;
    std::optional<double> tower_volume_mm3;
    std::optional<double> tower_mass_g;
    std::optional<double> coarse_nozzle_diameter;
    std::optional<double> typical_cost_seconds;
    std::optional<double> typical_switch_seconds;
    std::optional<double> typical_tower_seconds;
    // Why the first band above coarse_z_high did not use the coarse nozzle.
    MixedNozzleDecisionReason above_reason {MixedNozzleDecisionReason::Unavailable};
};

std::vector<MixedNozzleDecisionBandRow>  build_decision_band_rows(const AdvisorSliceObservations &observations);
std::vector<MixedNozzleDecisionRangeRow> build_decision_range_rows(const std::vector<MixedNozzleDecisionBandRow> &rows);
MixedNozzleDecisionSummary               build_decision_summary(const MixedNozzleDecisionInput &input);

// The summary block, in plain words, one string per line.
std::vector<std::string> format_decision_summary(const MixedNozzleDecisionSummary &summary);
// The same thing squeezed into one line, for the post-slice notification.
std::string              format_decision_headline(const MixedNozzleDecisionSummary &summary);
// The legend.
std::string              mixed_nozzle_decision_legend();

// What the layer slider shows for the band under the cursor: decision, one-word reason and the
// net seconds. Precomputed per slice so a mouse-move costs a lookup, not a snapshot read.
struct MixedNozzleBandNote {
    double      z_low {0.};
    double      z_high {0.};
    std::string text;
};
std::vector<MixedNozzleBandNote> build_decision_band_notes(const std::vector<MixedNozzleDecisionBandRow> &rows);
// nullptr when no band covers this Z.
const MixedNozzleBandNote *find_decision_band_note(const std::vector<MixedNozzleBandNote> &notes, double print_z);

}} // namespace Slic3r::GUI

#endif // slic3r_GUI_MixedNozzleDecisionReport_hpp_
