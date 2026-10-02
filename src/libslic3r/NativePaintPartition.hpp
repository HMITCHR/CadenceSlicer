#pragma once

#include "ExPolygon.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace Slic3r {

// Rows are indexed as [layer][region]. The input contains only the staged parent regions;
// the result is indexed by the explicit target PrintRegion IDs and therefore retains empty
// painted-region ghosts.
using NativePaintSliceRows = std::vector<std::vector<ExPolygons>>;

struct NativePaintRegionMapping
{
    // Both region IDs are zero-based PrintObject region IDs.
    size_t parent_region_id { 0 };
    // This is the zero-based slot in the returned native segmentation result. Slot zero is the
    // painted Extruder1 state; it is never the unpainted/default state.
    size_t logical_filament { 0 };
    size_t target_region_id { 0 };
    // Explicit ownership for geometry omitted from native segmentation. This is independent of
    // logical_filament so a real slot-zero paint cannot be confused with a parent's residual.
    bool parent_default { false };
};

// Mapping rows are indexed by layer and may therefore change at a layer-range boundary.
using NativePaintRegionMappings = std::vector<std::vector<NativePaintRegionMapping>>;

// Partition staged parent slices using an already-computed native painting segmentation.
//
// `staged_parent_slices` is [layer][parent region], `segmentation` is
// [layer][segmentation slot], `region_mappings` is [layer][parent/slot -> target], and the
// returned rows are [layer][target region]. `parent_default=true` is the explicit residual owner
// for a parent; it is separate from every segmentation slot, including slot zero. The helper is
// value-only: it does not inspect or mutate Model, Layer, PrintRegion, or grid state.
// `target_region_count` includes empty painted-region ghosts.
//
// A nullopt result means the staged inputs or mapping are malformed or ambiguous. Validation is
// completed before the returned partition is constructed so callers can fail closed before any
// LayerRegion mutation.
std::optional<NativePaintSliceRows> partition_native_painted_slices(
    const NativePaintSliceRows &staged_parent_slices,
    const NativePaintSliceRows &segmentation,
    size_t target_region_count,
    const NativePaintRegionMappings &region_mappings);

} // namespace Slic3r
