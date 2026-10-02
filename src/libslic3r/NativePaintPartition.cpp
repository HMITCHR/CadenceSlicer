#include "NativePaintPartition.hpp"

#include "ClipperUtils.hpp"

#include <algorithm>
#include <utility>

namespace Slic3r {

namespace {

using MappingTable = std::vector<std::vector<std::optional<size_t>>>;

void append_to_target(NativePaintSliceRows &owned,
                      std::vector<std::vector<bool>> &needs_merge,
                      size_t layer_id,
                      size_t target_region_id,
                      ExPolygons polygons)
{
    if (polygons.empty())
        return;

    ExPolygons &destination = owned[layer_id][target_region_id];
    if (destination.empty())
        destination = std::move(polygons);
    else {
        append(destination, std::move(polygons));
        needs_merge[layer_id][target_region_id] = true;
    }
}

} // namespace

std::optional<NativePaintSliceRows> partition_native_painted_slices(
    const NativePaintSliceRows &staged_parent_slices,
    const NativePaintSliceRows &segmentation,
    size_t target_region_count,
    const NativePaintRegionMappings &region_mappings)
{
    if (staged_parent_slices.size() != segmentation.size() ||
        staged_parent_slices.size() != region_mappings.size())
        return std::nullopt;

    const size_t parent_region_count = staged_parent_slices.empty() ? 0 : staged_parent_slices.front().size();
    const size_t segmentation_slot_count = segmentation.empty() ? 0 : segmentation.front().size();

    if (target_region_count < parent_region_count)
        return std::nullopt;

    for (const auto &row : staged_parent_slices)
        if (row.size() != parent_region_count)
            return std::nullopt;
    for (const auto &row : segmentation)
        if (row.size() != segmentation_slot_count)
            return std::nullopt;

    // The native segmentation API omits facet state zero (the unpainted/default state), then
    // stores Extruder1 painting in returned slot zero. Keep the omitted residual owner in a
    // separate table so a real slot-zero paint always requires its own explicit target mapping.
    std::vector<MappingTable> painted_targets(
        staged_parent_slices.size(),
        MappingTable(parent_region_count, std::vector<std::optional<size_t>>(segmentation_slot_count)));
    std::vector<std::vector<std::optional<size_t>>> parent_defaults(
        staged_parent_slices.size(), std::vector<std::optional<size_t>>(parent_region_count));

    for (size_t layer_id = 0; layer_id < region_mappings.size(); ++layer_id) {
        for (const NativePaintRegionMapping &mapping : region_mappings[layer_id]) {
            if (mapping.parent_region_id >= parent_region_count ||
                mapping.target_region_id >= target_region_count)
                return std::nullopt;

            if (mapping.parent_default) {
                // Parent-default is an explicit residual owner, not a segmentation slot. Its
                // canonical target is the same parent region and its slot field stays zero only
                // for stable aggregate initialization and diagnostics.
                if (mapping.logical_filament != 0 || mapping.target_region_id != mapping.parent_region_id ||
                    parent_defaults[layer_id][mapping.parent_region_id])
                    return std::nullopt;
                parent_defaults[layer_id][mapping.parent_region_id] = mapping.target_region_id;
                continue;
            }

            if (mapping.logical_filament >= segmentation_slot_count)
                return std::nullopt;
            std::optional<size_t> &target = painted_targets[layer_id][mapping.parent_region_id][mapping.logical_filament];
            if (target)
                return std::nullopt;
            target = mapping.target_region_id;
        }

        // Every staged parent needs an explicit owner for its omitted residual geometry. This is
        // deliberately independent of whether segmentation has any painted polygons on a layer.
        for (size_t parent_region_id = 0; parent_region_id < parent_region_count; ++parent_region_id)
            if (!parent_defaults[layer_id][parent_region_id])
                return std::nullopt;

        // apply_mm_segmentation resolves every active painted extruder against every parent
        // region's PaintedRegion entry. Preserve that explicit empty-ghost mapping contract for
        // every nonempty native segmentation slot, including returned slot zero.
        for (size_t segmentation_slot = 0; segmentation_slot < segmentation_slot_count; ++segmentation_slot)
            if (!segmentation[layer_id][segmentation_slot].empty())
                for (size_t parent_region_id = 0; parent_region_id < parent_region_count; ++parent_region_id)
                    if (!painted_targets[layer_id][parent_region_id][segmentation_slot])
                        return std::nullopt;
    }

    NativePaintSliceRows owned(staged_parent_slices.size(),
                               std::vector<ExPolygons>(target_region_count));
    std::vector<std::vector<bool>> needs_merge(staged_parent_slices.size(),
                                               std::vector<bool>(target_region_count, false));

    for (size_t layer_id = 0; layer_id < staged_parent_slices.size(); ++layer_id) {
        const MappingTable &mappings = painted_targets[layer_id];
        const auto &segmented_row = segmentation[layer_id];
        const bool layer_split = std::any_of(segmented_row.begin(), segmented_row.end(),
                                             [](const ExPolygons &polygons) { return !polygons.empty(); });
        if (!layer_split) {
            // No native painted geometry means every staged parent remains wholly owned by its
            // explicit default target and all painted-region ghosts stay empty.
            for (size_t parent_region_id = 0; parent_region_id < parent_region_count; ++parent_region_id)
                owned[layer_id][*parent_defaults[layer_id][parent_region_id]] =
                    staged_parent_slices[layer_id][parent_region_id];
            continue;
        }

        for (size_t parent_region_id = 0; parent_region_id < parent_region_count; ++parent_region_id) {
            const ExPolygons &parent = staged_parent_slices[layer_id][parent_region_id];
            if (parent.empty())
                continue;

            const bool has_transfer = [&]() {
                for (size_t slot = 0; slot < segmented_row.size(); ++slot)
                    if (!segmented_row[slot].empty() &&
                        *mappings[parent_region_id][slot] != parent_region_id &&
                        !intersection_ex(parent, segmented_row[slot]).empty())
                        return true;
                return false;
            }();
            if (!has_transfer) {
                // An empty ghost or painted self-target must not numerically erode an
                // untouched body through the residual sliver filter.
                append_to_target(owned, needs_merge, layer_id,
                                 *parent_defaults[layer_id][parent_region_id], parent);
                continue;
            }

            // Match apply_mm_segmentation(): only painted pieces whose explicit target differs
            // from the parent are moved. A painted self-target remains in the residual path,
            // avoiding duplicate polygons on the parent region.
            for (size_t segmentation_slot = 0; segmentation_slot < segmented_row.size(); ++segmentation_slot) {
                const ExPolygons &segmented = segmented_row[segmentation_slot];
                if (segmented.empty())
                    continue;

                const size_t target_region_id = *mappings[parent_region_id][segmentation_slot];
                if (target_region_id != parent_region_id)
                    append_to_target(owned, needs_merge, layer_id, target_region_id,
                                     intersection_ex(parent, segmented));
            }

            Polygons remaining = to_polygons(parent);
            for (size_t segmentation_slot = 0; segmentation_slot < segmented_row.size(); ++segmentation_slot) {
                const ExPolygons &segmented = segmented_row[segmentation_slot];
                if (segmented.empty())
                    continue;

                const size_t target_region_id = *mappings[parent_region_id][segmentation_slot];
                if (target_region_id != parent_region_id) {
                    remaining = diff(remaining, segmented);
                    if (remaining.empty())
                        break;
                }
            }

            if (!remaining.empty()) {
                // Preserve the native unprintable-sliver filter from apply_mm_segmentation().
                Polygons residual = opening(union_ex(remaining), scaled<float>(5. * EPSILON),
                                               scaled<float>(5. * EPSILON));
                append_to_target(owned, needs_merge, layer_id,
                                 *parent_defaults[layer_id][parent_region_id], union_ex(residual));
            }
        }

        for (size_t target_region_id = 0; target_region_id < target_region_count; ++target_region_id)
            if (needs_merge[layer_id][target_region_id])
                owned[layer_id][target_region_id] = closing_ex(
                    owned[layer_id][target_region_id], scaled<float>(10. * EPSILON));
    }

    return owned;
}

} // namespace Slic3r
