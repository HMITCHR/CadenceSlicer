#pragma once

#include "libslic3r.h"
#include "ExPolygon.hpp"
#include "PrintConfig.hpp"

#include <cstddef>
#include <functional>
#include <optional>
#include <vector>

namespace Slic3r {

enum class RegionalRendezvousReason : unsigned char {
    StackedContact = 0,
    FirstLayer,
    UserExact,
    InterlockingDropped,
    FineSkin,
};

struct RegionalForcedPlane
{
    coordf_t                         z;
    RegionalRendezvousReason         reason;
    std::vector<size_t>              regions;
    coordf_t                         max_width { 0. };
};

struct RendezvousPlane
{
    coordf_t                         z;
    RegionalRendezvousReason         reason;
    std::vector<size_t>              regions;
    coordf_t                         max_width { 0. };
};

struct RegionCell
{
    size_t   region_id;
    size_t   cell_index;
    size_t   first_lattice_row;
    size_t   last_lattice_row;
    coordf_t bottom_z;
    coordf_t top_z;
    coordf_t slice_z;
    coordf_t height;
    // Physical extruder that prints this cell: the region's own binding, overridden by
    // Print::regional_layer_plan_export() only for fine_skin cells.
    unsigned int physical_extruder_id { 0 };
    // One fine row of a coarse cell converted by the skin window. Set by the planner, never
    // inferred from height (the first layer and a short last cell are not fine-skin cells).
    bool fine_skin { false };
};

struct RegionalGridPlan
{
    size_t                       region_id;
    RegionalGridPhaseRule        phase_rule;
    std::vector<RegionCell>      cells;
    std::vector<RendezvousPlane> rendezvous;
};

// Builds one region's contiguous cell grid from its nominal tops and the ownership lattice.
// Preconditions (enforced by validation): ascending lattice planes, nominal_planes.front() is the
// region's first plane, fixed cadence after it. Cell 0 spans [0, first plane] at the vendor
// first-layer height, which need not match the cadence.
// minimum_cell_rows is the fewest lattice rows a cell may span (from the nozzle minimum); above 1,
// a contact cut that would leave a shorter cell is moved, merged or dropped for this region, and
// maximum_first_cell_height caps how far cell 0 may grow. Fine-skin rows are exempt.
[[nodiscard]] RegionalGridPlan plan_regional_grid(
    size_t                                  region_id,
    const std::vector<coordf_t>            &lattice_planes,
    const std::vector<coordf_t>            &nominal_planes,
    const std::vector<RegionalForcedPlane> &forced_planes,
    RegionalGridPhaseRule                   phase_rule,
    size_t                                  minimum_cell_rows = 1,
    coordf_t                                maximum_first_cell_height = 0.);

struct RegionalOwnershipResolution
{
    std::vector<ExPolygons> resolved;
    bool                    b01_overlap { false };
    double                  b01_overlap_mm2 { 0. };
};

// Resolves one sampling row in ascending declared-volume order; each later region clips all
// earlier ones, as slices_to_regions() does. The hairline check runs on the raw samples before
// precedence, after opening by the supplied threshold.
[[nodiscard]] RegionalOwnershipResolution resolve_regional_ownership(
    const std::vector<ExPolygons> &raw_samples,
    const std::vector<size_t>     &precedence,
    coordf_t                       b01_hairline_mm = 0.01);

struct RegionalInterlockingConfig
{
    bool   enabled { false };
    size_t beam_layer_count { 0 };
};

struct RegionalInterlockingDecision
{
    coordf_t bottom_z { 0. };
    coordf_t top_z { 0. };
    bool     apply { false };
};

struct NativeRegionCellState
{
    RegionCell cell;
    ExPolygons own_sample;
    ExPolygons footprint;
    double     clipping_residue_mm2 { 0. };
};

struct NativeRegionalPlanningInput
{
    std::vector<coordf_t>                 lattice_planes;
    std::vector<std::vector<coordf_t>>    nominal_planes;
    std::vector<std::vector<ExPolygons>>  lattice_ownership;
    std::vector<size_t>                   precedence;
    coordf_t                              interface_tolerance { 0. };
    RegionalGridPhaseRule                 phase_rule { RegionalGridPhaseRule::KeepNominal };
    RegionalInterlockingConfig            interlocking;
    std::function<ExPolygons(size_t, coordf_t)> sample_native_ownership;
    // Per-region opt-in for fine-skin cells; empty, out-of-range or false means off.
    std::vector<bool> fine_skin_enabled;
    // Fine-skin window in fine rows at each exposed face (mixed_nozzle_body_fine_skin_layers).
    // Out-of-range or non-positive means off.
    std::vector<int>  fine_skin_layers;
    // Coarse line width (mm) per region. A footprint change seeds a window only if it survives an
    // inward offset of half this width, filtering chamfer and fillet slivers. Empty or
    // non-positive disables the filter.
    std::vector<double> fine_skin_coarse_line_width;
    // Per region, the fewest lattice rows a cell may span (nozzle minimum over base cadence,
    // rounded up) and the tallest cell 0 the nozzle can lay. Empty, 1 and 0 give the ordinary grid.
    std::vector<size_t>   minimum_cell_rows;
    std::vector<coordf_t> maximum_first_cell_height;
    // Z-contouring: when set, every cell but the shared first layer is sliced at its own bottom
    // plus this offset (the smallest zaa_min_z of the regions that ask for it) instead of at its
    // midpoint, as new_layers() slices an ordinary layer. Unset keeps every cell on its midpoint.
    std::optional<coordf_t> contour_slice_offset;
};

// Complete planning state: sampled ownership and cell footprints only.
struct NativeRegionalGridState
{
    std::vector<coordf_t>                       lattice_planes;
    // Raw model-part samples in declared-volume order, kept for the hairline check before
    // slices_to_regions() resolves ownership.
    std::vector<std::vector<ExPolygons>>        raw_lattice_samples;
    std::vector<std::vector<ExPolygons>>        lattice_ownership;
    std::vector<RegionalForcedPlane>            forced_planes;
    std::vector<RegionalGridPlan>               region_plans;
    std::vector<std::vector<NativeRegionCellState>> cells;
    std::vector<coordf_t>                       event_planes;
    std::vector<RendezvousPlane>                rendezvous;
    std::vector<RegionalInterlockingDecision>   interlocking_decisions;
    // The plane the beam voxels are phased from: the first coarse plane at or above the lowest
    // lattice row on which two bodies touch. The slicing step starts the beam lattice on the row
    // above it, so the planned voxels and the deposited beams share one phase. Zero when
    // interlocking is off.
    coordf_t                                    interlocking_origin_z { 0. };
};

[[nodiscard]] NativeRegionalGridState plan_native_regional_grids(
    const NativeRegionalPlanningInput &input);

[[nodiscard]] std::vector<coordf_t> native_regional_cell_midplanes(
    const NativeRegionalPlanningInput &input,
    std::vector<RegionalGridPlan> *complete_plans = nullptr);

} // namespace Slic3r
