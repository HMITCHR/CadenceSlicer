#pragma once

#include "libslic3r.h"
#include "RegionalGrids.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r {

// Whether a sparse-infill configuration may own a committed coarse band: only a pattern whose
// geometry is the same on every layer survives dropping the intermediate one. Shared by
// validation and the pre-mutation check so they cannot disagree.
enum InfillPattern : int;
[[nodiscard]] bool regional_band_admits_sparse_infill_pattern(InfillPattern pattern);
[[nodiscard]] bool regional_band_admits_sparse_infill_density(double density_percent);

// Whether Feature Split's native-combine sparse cadence may use this pattern. Native combine
// draws the pattern once on the band's top layer, so no per-layer equivalence is needed; the
// exclusions concern patterns that rely on the individual layers a band removes.
[[nodiscard]] bool feature_combine_admits_sparse_infill_pattern(InfillPattern pattern);

// Why a resolved plan could not be rendered. Every integrity failure is named and the text
// stays empty, so a partial block is never emitted.
enum class RegionalPlanFormatError : unsigned char {
    None = 0,
    NoBands,               // no committed band, so the block would declare no plan at all
    NoRefusalGrid,         // the refusal side table is empty and so describes no fine grid
    BandIndexNotAscending, // a band's top index does not rise above its first
    BandIndexOutOfRange,   // a band names a fine index the refusal grid does not have
    BandOverlap,           // two bands claim the same fine index
    CommittedIndexRefused, // a banded index also carries a refusal; the side tables disagree
    UncoveredIndex,        // an unbanded index carries no refusal and so is in neither part
    MixedBinding,          // the bands disagree on which region and tool own the plan
    GridNotContiguous,
    CellHeightIllegal,
    RendezvousNotShared,
    EventPlaneWithoutCell,
    FeatureBandInvalid,    // a Feature record failed the formatter's integrity checks
};

// Which producer wrote a RegionalGridPlanRecord (Body by default). Feature records carry one
// committed band per RegionCell: cell_index is the band index, first_lattice_row and
// last_lattice_row are its first/last fine PrintObject layer indices (Feature Split has no
// lattice), and the Z fields are as for a Body cell.
enum class RegionalPlanRecordMode : unsigned char { Body = 0, Feature = 1 };

struct RegionalGridPlanRecord
{
    size_t                region_id;
    unsigned int          physical_extruder_id;
    coordf_t              nozzle_diameter;
    coordf_t              cadence;
    RegionalGridPhaseRule phase_rule;
    std::vector<RegionCell> cells;
    RegionalPlanRecordMode  mode { RegionalPlanRecordMode::Body };
};

// One object-scoped line with the physical tools and heights the support base and interface
// resolved to, so the plan block can be checked against support too.
struct RegionalSupportPlanRecord {
    unsigned int base_tool;
    coordf_t     base_height;
    unsigned int interface_tool;
    coordf_t     interface_height;
};

struct RegionalLayerPlanText
{
    RegionalPlanFormatError error; // None exactly when text holds one complete block
    std::string             text;  // empty on any error, so a refused plan is never half-written
};

// Native-grid plan grammar. Returns empty text on any integrity failure.
[[nodiscard]] RegionalLayerPlanText format_regional_layer_plan(
    const std::vector<RegionalGridPlanRecord> &grids,
    const std::vector<RendezvousPlane>        &rendezvous,
    const std::vector<coordf_t>               &event_planes,
    coordf_t                                   tolerance,
    coordf_t                                   base_cadence,
    int                                         toolchange_total = 0,
    int                                         toolchange_per_band = 0,
    const std::optional<RegionalSupportPlanRecord> &support = std::nullopt);

// Feature Split band grammar. bands may be empty (no sparse infill scheduled): the block still
// opens with a "; SRL_FEATURE bands=<n>" header. Every record present must be
// RegionalPlanRecordMode::Feature with at least one cell. Value-only.
[[nodiscard]] RegionalLayerPlanText format_regional_layer_plan_feature(
    const std::vector<RegionalGridPlanRecord> &bands);

} // namespace Slic3r
