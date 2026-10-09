#include "RegionalGrids.hpp"
#include "ClipperUtils.hpp"
#include "BoundingBox.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iterator>
#include <optional>
#include <utility>

namespace Slic3r {
namespace {

constexpr coordf_t grid_epsilon = 1e-8;

bool same_plane(coordf_t lhs, coordf_t rhs)
{
    return std::abs(lhs - rhs) <= grid_epsilon;
}

void append_distinct(std::vector<coordf_t> &planes, coordf_t z)
{
    if (planes.empty() || !same_plane(planes.back(), z))
        planes.push_back(z);
}

size_t lattice_index(const std::vector<coordf_t> &lattice, coordf_t z)
{
    const auto it = std::lower_bound(lattice.begin(), lattice.end(), z - grid_epsilon);
    assert(it != lattice.end() && same_plane(*it, z));
    return size_t(it - lattice.begin());
}

std::vector<RegionalForcedPlane> relevant_forced_planes(
    const std::vector<RegionalForcedPlane> &forced_planes,
    coordf_t first_plane,
    coordf_t maximum_plane)
{
    std::vector<RegionalForcedPlane> relevant;
    for (const RegionalForcedPlane &forced : forced_planes)
        if (forced.z > first_plane + grid_epsilon && forced.z <= maximum_plane + grid_epsilon)
            relevant.push_back(forced);
    std::sort(relevant.begin(), relevant.end(),
              [](const RegionalForcedPlane &lhs, const RegionalForcedPlane &rhs) { return lhs.z < rhs.z; });
    relevant.erase(std::unique(relevant.begin(), relevant.end(),
                               [](const RegionalForcedPlane &lhs, const RegionalForcedPlane &rhs) {
                                   return same_plane(lhs.z, rhs.z);
                               }),
                   relevant.end());
    return relevant;
}

std::vector<coordf_t> keep_nominal_boundaries(
    const std::vector<coordf_t> &nominal,
    const std::vector<RegionalForcedPlane> &forced)
{
    std::vector<coordf_t> boundaries = nominal;
    for (const RegionalForcedPlane &plane : forced)
        if (plane.z < nominal.back() - grid_epsilon)
            boundaries.push_back(plane.z);
    std::sort(boundaries.begin(), boundaries.end());
    boundaries.erase(std::unique(boundaries.begin(), boundaries.end(), same_plane), boundaries.end());
    return boundaries;
}

std::vector<coordf_t> rephased_boundaries(
    const std::vector<coordf_t> &lattice,
    const std::vector<coordf_t> &nominal,
    const std::vector<RegionalForcedPlane> &forced)
{
    std::vector<RegionalForcedPlane> cuts;
    cuts.reserve(forced.size());
    std::copy_if(forced.begin(), forced.end(), std::back_inserter(cuts), [&nominal](const RegionalForcedPlane &plane) {
        return std::none_of(nominal.begin(), nominal.end(), [&plane](coordf_t z) { return same_plane(z, plane.z); });
    });
    if (cuts.empty() || nominal.size() < 2)
        return nominal;

    const coordf_t cadence = nominal[1] - nominal[0];
    assert(cadence > grid_epsilon);

    std::vector<coordf_t> boundaries;
    boundaries.reserve(nominal.size() + forced.size());
    append_distinct(boundaries, nominal.front());

    coordf_t target = nominal[1];
    for (const RegionalForcedPlane &plane : cuts) {
        while (target < plane.z - grid_epsilon) {
            append_distinct(boundaries, target);
            target += cadence;
        }
        if (same_plane(target, plane.z)) {
            append_distinct(boundaries, target);
            target += cadence;
        } else {
            assert(plane.z < target);
            append_distinct(boundaries, plane.z);
            target = plane.z + cadence;
        }
    }

    const coordf_t maximum_plane = lattice.back();
    while (target <= maximum_plane + grid_epsilon) {
        append_distinct(boundaries, std::min(target, maximum_plane));
        target += cadence;
    }
    if (boundaries.back() < maximum_plane - grid_epsilon &&
        maximum_plane - boundaries.back() <= cadence + grid_epsilon)
        append_distinct(boundaries, maximum_plane);

    return boundaries;
}

// Holds one region's cell boundaries to its nozzle's minimum, counted in lattice rows.
// boundaries are the region's cell tops, boundaries[0] being the top of cell 0. A boundary that is
// a contact plane or the region's own top is not moved; a nominal boundary may be. A short cell is
// repaired in this order, each step only if it leaves every touched cell legal:
//   1. move the nominal boundary below it down, so the cell below gives it rows;
//   2. move the nominal boundary above it up, so the cell above gives it rows;
//   3. when it is cell 1, fold it into cell 0, which may grow up to the nozzle maximum;
//   4. merge it with the cell below, or else the one above, when the merged cell is no taller
//      than the cadence (a contact cut is then not applied to this region);
//   5. move every boundary below it down by the rows it lacks, from the region's first plane up, with
//      cell 0 taking the rows (up to the nozzle maximum), when no contact plane lies below it: the grid
//      then lands on its top, or on a contact plane above it, instead of losing it;
//   6. when it is the region's top cell, drop it.
// Anything still short is left for the caller's envelope check to refuse.
void hold_cells_to_minimum_rows(
    std::vector<coordf_t>                  &boundaries,
    const std::vector<coordf_t>            &lattice,
    const std::vector<RegionalForcedPlane> &contact,
    size_t                                  minimum_rows,
    size_t                                  cadence_rows,
    coordf_t                                maximum_first_cell_height)
{
    if (minimum_rows <= 1 || boundaries.size() < 2)
        return;
    const auto is_contact = [&contact](coordf_t z) {
        return std::any_of(contact.begin(), contact.end(),
                           [z](const RegionalForcedPlane &plane) { return same_plane(plane.z, z); });
    };
    const auto rows_between = [&lattice](coordf_t lower, coordf_t upper) {
        return long(lattice_index(lattice, upper)) - long(lattice_index(lattice, lower));
    };
    const long k = long(minimum_rows);
    const long n_rows = long(std::max<size_t>(cadence_rows, minimum_rows));
    // Step 5: the boundaries under cell i, each moved down by deficit rows. Cell 1 then loses those rows
    // and is folded into cell 0, which may grow up to the nozzle maximum. Every cell keeps its rows.
    const auto shift_cells_below_down = [&](const std::vector<coordf_t> &tops, size_t i, long deficit)
        -> std::optional<std::vector<coordf_t>> {
        if (deficit <= 0 || i < 2 || maximum_first_cell_height <= 0.)
            return std::nullopt;
        std::vector<coordf_t> out = tops;
        for (size_t j = 1; j < i; ++j) {
            const size_t row = lattice_index(lattice, out[j]);
            if (is_contact(out[j]) || row < size_t(deficit))
                return std::nullopt;
            out[j] = lattice[row - size_t(deficit)];
        }
        // Cell 1 now spans fewer rows than before; fold it into cell 0 when it is short.
        if (rows_between(out[0], out[1]) < k) {
            if (out[1] > maximum_first_cell_height + grid_epsilon)
                return std::nullopt;
            out.erase(out.begin());
            --i;
        }
        for (size_t j = 1; j <= i; ++j)
            if (rows_between(out[j - 1], out[j]) < k || rows_between(out[j - 1], out[j]) > n_rows)
                return std::nullopt;
        return out;
    };
    // Bound only guards against malformed input; every repair makes progress.
    for (size_t pass = 0; pass < 4 * boundaries.size() + 8; ++pass) {
        bool changed = false;
        for (size_t i = 1; i < boundaries.size() && !changed; ++i) {
            const long rows = rows_between(boundaries[i - 1], boundaries[i]);
            if (rows >= k)
                continue;
            const size_t last = boundaries.size() - 1;
            const auto movable = [&](size_t index) {
                return index != 0 && index != last && !is_contact(boundaries[index]);
            };
            if (i >= 2 && movable(i - 1) && rows_between(boundaries[i - 2], boundaries[i]) >= 2 * k) {
                boundaries[i - 1] = lattice[lattice_index(lattice, boundaries[i]) - size_t(k)];
                changed = true;
            } else if (i + 1 <= last && movable(i) && rows_between(boundaries[i - 1], boundaries[i + 1]) >= 2 * k) {
                boundaries[i] = lattice[lattice_index(lattice, boundaries[i - 1]) + size_t(k)];
                changed = true;
            } else if (i == 1 && maximum_first_cell_height > 0. &&
                       boundaries[1] <= maximum_first_cell_height + grid_epsilon) {
                boundaries.erase(boundaries.begin());
                changed = true;
            } else if (i >= 2 && rows_between(boundaries[i - 2], boundaries[i]) <= n_rows) {
                boundaries.erase(boundaries.begin() + long(i - 1));
                changed = true;
            } else if (i + 1 <= last && rows_between(boundaries[i - 1], boundaries[i + 1]) <= n_rows) {
                boundaries.erase(boundaries.begin() + long(i));
                changed = true;
            } else if (std::optional<std::vector<coordf_t>> shifted = shift_cells_below_down(boundaries, i, k - rows); shifted) {
                boundaries = std::move(*shifted);
                changed = true;
            } else if (i == last) {
                boundaries.pop_back();
                changed = true;
            }
        }
        if (!changed)
            break;
    }
}

RegionalGridPlan plan_regional_grid_held_to_minimum(
    size_t                                  region_id,
    const std::vector<coordf_t>            &lattice_planes,
    const std::vector<coordf_t>            &nominal_planes,
    const std::vector<RegionalForcedPlane> &forced_planes,
    RegionalGridPhaseRule                   phase_rule,
    size_t                                  minimum_cell_rows,
    coordf_t                                maximum_first_cell_height)
{
    // Contact cuts are decided first and held to the minimum; fine-skin rows are added after, on
    // top of that tiling. plan_fine_skin_conversions() derives its windows from the same tiling
    // with the skin rows left out, so the cells it converts are exactly the cells found here.
    std::vector<RegionalForcedPlane> contact_planes;
    std::vector<RegionalForcedPlane> skin_planes;
    // A fine-skin plane cuts only the region that opted in, as on the default path.
    for (const RegionalForcedPlane &plane : forced_planes)
        if (plane.reason != RegionalRendezvousReason::FineSkin)
            contact_planes.push_back(plane);
        else if (std::find(plane.regions.begin(), plane.regions.end(), region_id) != plane.regions.end())
            skin_planes.push_back(plane);

    const coordf_t maximum_forced_plane = phase_rule == RegionalGridPhaseRule::KeepNominal ?
        nominal_planes.back() : lattice_planes.back();
    const std::vector<RegionalForcedPlane> contact = relevant_forced_planes(
        contact_planes, nominal_planes.front(), maximum_forced_plane);
    std::vector<coordf_t> boundaries = phase_rule == RegionalGridPhaseRule::KeepNominal ?
        keep_nominal_boundaries(nominal_planes, contact) :
        rephased_boundaries(lattice_planes, nominal_planes, contact);
    const coordf_t maximum_plane = phase_rule == RegionalGridPhaseRule::KeepNominal ?
        std::min(nominal_planes.back(), lattice_planes.back()) : lattice_planes.back();
    boundaries.erase(std::remove_if(boundaries.begin(), boundaries.end(), [maximum_plane](coordf_t z) {
        return z > maximum_plane + grid_epsilon;
    }), boundaries.end());
    if (boundaries.empty() || boundaries.back() < maximum_plane - grid_epsilon)
        append_distinct(boundaries, maximum_plane);

    const coordf_t base = lattice_planes.size() > 1 ? lattice_planes[1] - lattice_planes[0] : 0.;
    const size_t cadence_rows = nominal_planes.size() > 1 && base > grid_epsilon ?
        size_t(std::lround((nominal_planes[1] - nominal_planes[0]) / base)) : minimum_cell_rows;
    hold_cells_to_minimum_rows(boundaries, lattice_planes, contact, minimum_cell_rows, cadence_rows,
                               maximum_first_cell_height);

    const coordf_t own_first_plane = boundaries.front();
    // Skin rows are one lattice row each, on the fine tool, and may sit inside cell 0 too.
    const std::vector<RegionalForcedPlane> skins = relevant_forced_planes(skin_planes, 0., boundaries.back());
    for (const RegionalForcedPlane &plane : skins)
        boundaries.push_back(plane.z);
    std::sort(boundaries.begin(), boundaries.end());
    boundaries.erase(std::unique(boundaries.begin(), boundaries.end(), same_plane), boundaries.end());

    RegionalGridPlan plan{region_id, phase_rule, {}, {}};
    plan.cells.reserve(boundaries.size());
    coordf_t bottom_z = 0.;
    size_t first_row = 0;
    for (size_t cell_index = 0; cell_index < boundaries.size(); ++cell_index) {
        const coordf_t top_z = boundaries[cell_index];
        const size_t last_row = lattice_index(lattice_planes, top_z);
        assert(last_row >= first_row);
        plan.cells.push_back({region_id, cell_index, first_row, last_row, bottom_z, top_z,
                              0.5 * (bottom_z + top_z), top_z - bottom_z});
        bottom_z = top_z;
        first_row = last_row + 1;
    }

    const auto is_boundary = [&boundaries](coordf_t z) {
        return std::any_of(boundaries.begin(), boundaries.end(), [z](coordf_t top) { return same_plane(top, z); });
    };
    plan.rendezvous.push_back({own_first_plane, RegionalRendezvousReason::FirstLayer, {region_id}, 0.});
    // A contact cut this region could not take at its nozzle's minimum is not a plane it meets.
    for (const RegionalForcedPlane &plane : contact)
        if (is_boundary(plane.z))
            plan.rendezvous.push_back({plane.z, plane.reason, plane.regions, plane.max_width});
    for (const RegionalForcedPlane &plane : skins)
        plan.rendezvous.push_back({plane.z, plane.reason, plane.regions, plane.max_width});
    return plan;
}

} // namespace

RegionalGridPlan plan_regional_grid(
    size_t                                  region_id,
    const std::vector<coordf_t>            &lattice_planes,
    const std::vector<coordf_t>            &nominal_planes,
    const std::vector<RegionalForcedPlane> &forced_planes,
    RegionalGridPhaseRule                   phase_rule,
    size_t                                  minimum_cell_rows,
    coordf_t                                maximum_first_cell_height)
{
    assert(!lattice_planes.empty());
    assert(!nominal_planes.empty());
    // A region whose nozzle cannot lay the shared first layer starts on its own first plane, a whole
    // number of base rows above it.
    assert(nominal_planes.front() >= lattice_planes.front() - grid_epsilon);
    assert(same_plane(lattice_planes[lattice_index(lattice_planes, nominal_planes.front())], nominal_planes.front()));
    assert(std::is_sorted(lattice_planes.begin(), lattice_planes.end()));
    assert(std::is_sorted(nominal_planes.begin(), nominal_planes.end()));

    if (minimum_cell_rows > 1)
        return plan_regional_grid_held_to_minimum(region_id, lattice_planes, nominal_planes, forced_planes,
                                                  phase_rule, minimum_cell_rows, maximum_first_cell_height);

    const coordf_t maximum_forced_plane = phase_rule == RegionalGridPhaseRule::KeepNominal ?
        nominal_planes.back() : lattice_planes.back();
    // A fine-skin plane cuts only the opting-in region's cell; other forced planes apply to every
    // region. Filter before de-duplication so a contact plane is never dropped for a skin plane.
    std::vector<RegionalForcedPlane> own_planes;
    own_planes.reserve(forced_planes.size());
    for (const RegionalForcedPlane &plane : forced_planes)
        if (plane.reason != RegionalRendezvousReason::FineSkin ||
            std::find(plane.regions.begin(), plane.regions.end(), region_id) != plane.regions.end())
            own_planes.push_back(plane);
    const std::vector<RegionalForcedPlane> forced = relevant_forced_planes(
        own_planes, nominal_planes.front(), maximum_forced_plane);

    std::vector<coordf_t> boundaries = phase_rule == RegionalGridPhaseRule::KeepNominal ?
        keep_nominal_boundaries(nominal_planes, forced) :
        rephased_boundaries(lattice_planes, nominal_planes, forced);
    // REPHASE runs to the top of the ownership lattice. KEEP preserves the nominal planes, so its
    // last cell ends at the region's nominal top; extending it would invent a boundary and could
    // emit a cell taller than the cadence.
    const coordf_t maximum_plane = phase_rule == RegionalGridPhaseRule::KeepNominal ?
        std::min(nominal_planes.back(), lattice_planes.back()) : lattice_planes.back();
    boundaries.erase(std::remove_if(boundaries.begin(), boundaries.end(), [maximum_plane](coordf_t z) {
        return z > maximum_plane + grid_epsilon;
    }), boundaries.end());
    if (boundaries.empty() || boundaries.back() < maximum_plane - grid_epsilon)
        append_distinct(boundaries, maximum_plane);

    RegionalGridPlan plan{region_id, phase_rule, {}, {}};
    plan.cells.reserve(boundaries.size());
    coordf_t bottom_z = 0.;
    size_t first_row = 0;
    for (size_t cell_index = 0; cell_index < boundaries.size(); ++cell_index) {
        const coordf_t top_z = boundaries[cell_index];
        const size_t last_row = lattice_index(lattice_planes, top_z);
        assert(last_row >= first_row);
        plan.cells.push_back({region_id, cell_index, first_row, last_row, bottom_z, top_z,
                              0.5 * (bottom_z + top_z), top_z - bottom_z});
        bottom_z = top_z;
        first_row = last_row + 1;
    }

    plan.rendezvous.push_back({nominal_planes.front(), RegionalRendezvousReason::FirstLayer,
                               {region_id}, 0.});
    for (const RegionalForcedPlane &plane : forced)
        plan.rendezvous.push_back({plane.z, plane.reason, plane.regions, plane.max_width});

    return plan;
}

RegionalOwnershipResolution resolve_regional_ownership(
    const std::vector<ExPolygons>                &raw_samples,
    const std::vector<size_t>                    &precedence,
    coordf_t                                      b01_hairline_mm,
    double                                        b01_sliver_mm2,
    const std::vector<std::pair<size_t, size_t>> &kept_overlaps)
{
    // Empty painted-region ghosts retain indexed metadata without participating in
    // precedence. Every nonempty owner must still occur exactly once.
    assert(precedence.size() <= raw_samples.size());
#ifndef NDEBUG
    std::vector<bool> included(raw_samples.size(), false);
    for (size_t region : precedence) {
        assert(region < raw_samples.size());
        assert(!included[region]);
        included[region] = true;
    }
    for (size_t region = 0; region < raw_samples.size(); ++region)
        assert(included[region] || raw_samples[region].empty());
#endif
    RegionalOwnershipResolution out{raw_samples, false, 0.};

    for (size_t i = 0; i < raw_samples.size(); ++i)
        for (size_t j = i + 1; j < raw_samples.size(); ++j) {
            if (std::any_of(kept_overlaps.begin(), kept_overlaps.end(), [i, j](const std::pair<size_t, size_t> &pair) {
                    return (pair.first == i && pair.second == j) || (pair.first == j && pair.second == i);
                }))
                continue;
            ExPolygons shared = intersection_ex(raw_samples[i], raw_samples[j], ApplySafetyOffset::Yes);
            if (b01_hairline_mm > 0.)
                shared = opening_ex(shared, scaled<float>(b01_hairline_mm));
            const double shared_mm2 = unscale<double>(unscale<double>(std::abs(area(shared))));
            if (!shared.empty() && shared_mm2 >= b01_sliver_mm2) {
                out.b01_overlap = true;
                out.b01_overlap_mm2 = std::max(out.b01_overlap_mm2, shared_mm2);
            }
        }

    for (size_t high_pos = precedence.size(); high_pos-- > 0;) {
        const size_t high = precedence[high_pos];
        assert(high < out.resolved.size());
        for (size_t low_pos = 0; low_pos < high_pos; ++low_pos) {
            const size_t low = precedence[low_pos];
            assert(low < out.resolved.size());
            out.resolved[low] = diff_ex(out.resolved[low], out.resolved[high], ApplySafetyOffset::Yes);
        }
    }
    return out;
}

bool body_split_smaller_part_keeps_overlap(double smaller_mm3, double bigger_mm3, double shared_mm3)
{
    return bigger_mm3 > 0. && smaller_mm3 <= body_split_kept_part_max_share * bigger_mm3 &&
           shared_mm3 <= body_split_kept_overlap_max_share * bigger_mm3;
}

std::optional<std::vector<size_t>> precedence_with_kept_overlaps(
    const std::vector<size_t>                    &precedence,
    const std::vector<std::pair<size_t, size_t>> &loser_winner)
{
    // Repeatedly take the first remaining region that no remaining region has to precede.
    std::vector<size_t> remaining = precedence;
    std::vector<size_t> out;
    out.reserve(precedence.size());
    while (!remaining.empty()) {
        const auto next = std::find_if(remaining.begin(), remaining.end(), [&](size_t region) {
            return std::none_of(loser_winner.begin(), loser_winner.end(), [&](const std::pair<size_t, size_t> &pair) {
                return pair.second == region &&
                       std::find(remaining.begin(), remaining.end(), pair.first) != remaining.end();
            });
        });
        if (next == remaining.end())
            return std::nullopt;
        out.push_back(*next);
        remaining.erase(next);
    }
    return out;
}

double body_split_overlap_sliver_mm2(double finest_nozzle_diameter)
{
    return std::isfinite(finest_nozzle_diameter) && finest_nozzle_diameter > 0. ?
        0.25 * M_PI * finest_nozzle_diameter * finest_nozzle_diameter : 0.;
}

namespace {

// Per-region envelope inputs; defaults give the ordinary grid.
size_t region_minimum_cell_rows(const NativeRegionalPlanningInput &input, size_t region)
{
    return region < input.minimum_cell_rows.size() ? std::max<size_t>(1, input.minimum_cell_rows[region]) : 1;
}

coordf_t region_maximum_first_cell_height(const NativeRegionalPlanningInput &input, size_t region)
{
    return region < input.maximum_first_cell_height.size() ? input.maximum_first_cell_height[region] : 0.;
}

RegionalGridPlan plan_input_region_grid(const NativeRegionalPlanningInput &input, size_t region,
                                        const std::vector<RegionalForcedPlane> &forced)
{
    return plan_regional_grid(region, input.lattice_planes, input.nominal_planes[region], forced, input.phase_rule,
                              region_minimum_cell_rows(input, region), region_maximum_first_cell_height(input, region));
}

// True when some region's grid is not the shared-first-layer, one-row-minimum grid. Only then
// do the published rendezvous planes need re-deriving from the cells.
bool envelope_grids_active(const NativeRegionalPlanningInput &input)
{
    for (size_t region = 0; region < input.nominal_planes.size(); ++region)
        if (region_minimum_cell_rows(input, region) > 1 ||
            (!input.nominal_planes[region].empty() && !input.lattice_planes.empty() &&
             !same_plane(input.nominal_planes[region].front(), input.lattice_planes.front())))
            return true;
    return false;
}

double area_mm2(const ExPolygons &polygons)
{
    return unscale<double>(unscale<double>(std::abs(area(polygons))));
}

coordf_t contact_width_mm(const ExPolygons &contact)
{
    if (contact.empty())
        return 0.;
    const BoundingBox box = get_extents(contact);
    return unscale<double>(std::max(box.size().x(), box.size().y()));
}

// A contact thinner than this is slicing noise, not a contact: the same part sliced on two rows
// can differ by a hairline along its sloped or curved sides (text sunk into a base showed 0.004 mm2
// on rows inside the letters). A forced plane there split the base's cells for nothing, and the
// cell repair then dropped the real plane under the letters. The same hairline as the overlap check.
constexpr coordf_t contact_hairline_mm = 0.01;

bool survives_contact_tolerance(ExPolygons contact, coordf_t tolerance)
{
    if (contact.empty())
        return false;
    contact = opening_ex(contact, tolerance > 0. ? scaled<float>(std::max(0.5 * tolerance, contact_hairline_mm)) :
                                                   scaled<float>(contact_hairline_mm));
    return !contact.empty();
}

// Two regions on one and the same cell grid: every plane either of them ends a cell on, the other
// does too, so nothing between them needs a meeting plane.
bool regions_share_grid(const NativeRegionalPlanningInput &input, size_t lhs, size_t rhs)
{
    if (lhs >= input.nominal_planes.size() || rhs >= input.nominal_planes.size())
        return false;
    const std::vector<coordf_t> &a = input.nominal_planes[lhs];
    const std::vector<coordf_t> &b = input.nominal_planes[rhs];
    return !a.empty() && a.size() == b.size() &&
           std::equal(a.begin(), a.end(), b.begin(), [](coordf_t x, coordf_t y) { return same_plane(x, y); });
}

std::vector<RegionalForcedPlane> detect_forced_planes(const NativeRegionalPlanningInput &input)
{
    std::vector<RegionalForcedPlane> forced;
    const size_t regions = input.lattice_ownership.size();
    const size_t rows = input.lattice_planes.size();
    for (size_t plane = 0; plane + 1 < rows; ++plane) {
        bool is_forced = false;
        coordf_t max_width = 0.;
        for (size_t source = 0; source < regions; ++source)
            for (size_t other = 0; other < regions; ++other) {
                if (source == other)
                    continue;
                // Two regions on the same grid meet on every plane anyway. Forcing one would only split
                // the cells of regions that take no part in the contact, such as a coarse body beside
                // a fine body whose painted colour steps against it on its own shell rows.
                if (regions_share_grid(input, source, other))
                    continue;
                const ExPolygons changed = xor_ex(input.lattice_ownership[source][plane],
                                                  input.lattice_ownership[source][plane + 1],
                                                  ApplySafetyOffset::Yes);
                ExPolygons other_union = input.lattice_ownership[other][plane];
                other_union.insert(other_union.end(), input.lattice_ownership[other][plane + 1].begin(),
                                   input.lattice_ownership[other][plane + 1].end());
                other_union = union_ex(other_union);
                ExPolygons contact = intersection_ex(changed, other_union, ApplySafetyOffset::Yes);
                if (survives_contact_tolerance(contact, input.interface_tolerance)) {
                    is_forced = true;
                    max_width = std::max(max_width, contact_width_mm(contact));
                }
            }
        // A kept overlap removed the bigger part under the smaller one, so its owned rows no longer
        // show where its own top (or bottom) meets the smaller part. Its raw rows still do.
        for (const auto &[bigger, smaller] : input.kept_overlaps) {
            if (bigger >= input.kept_overlap_raw_rows.size() || smaller >= regions ||
                input.kept_overlap_raw_rows[bigger].size() != rows || regions_share_grid(input, bigger, smaller))
                continue;
            const ExPolygons changed = xor_ex(input.kept_overlap_raw_rows[bigger][plane],
                                              input.kept_overlap_raw_rows[bigger][plane + 1], ApplySafetyOffset::Yes);
            ExPolygons other_union = input.lattice_ownership[smaller][plane];
            other_union.insert(other_union.end(), input.lattice_ownership[smaller][plane + 1].begin(),
                               input.lattice_ownership[smaller][plane + 1].end());
            other_union = union_ex(other_union);
            const ExPolygons contact = intersection_ex(changed, other_union, ApplySafetyOffset::Yes);
            if (survives_contact_tolerance(contact, input.interface_tolerance)) {
                is_forced = true;
                max_width = std::max(max_width, contact_width_mm(contact));
            }
        }
        if (is_forced)
            forced.push_back({input.lattice_planes[plane], RegionalRendezvousReason::StackedContact,
                              input.precedence, max_width});
    }
    return forced;
}

// A fine-skin seed window (N rows) marks the coarse cells it touches, and each such cell is
// converted whole into single-row fine cells, so a region's cells are only fine- or
// coarse-height. The seed is one region's own footprint changing between adjacent lattice rows,
// computed before surfaces have types (cells are planned at posSlice).
struct FineSkinConversion
{
    // Extra forced planes, all regions merged, in the shape plan_regional_grid consumes.
    std::vector<RegionalForcedPlane>                        planes;
    // Per region, the [bottom_z, top_z] span of each coarse cell that was converted.
    std::vector<std::vector<std::pair<coordf_t, coordf_t>>> spans;
};

// A seed is a body-interface handover, not a real face, when its eroded delta is at least 90%
// covered by other regions' ownership on the adjacent lattice row the change lands on. Outer
// top/bottom seeds have no adjacent row and are never handovers.
bool seed_is_body_handover(
    const NativeRegionalPlanningInput &input, size_t region, size_t adjacent_row,
    const ExPolygons &eroded_delta)
{
    if (eroded_delta.empty())
        return false;
    ExPolygons other_union;
    for (size_t other = 0; other < input.lattice_ownership.size(); ++other) {
        if (other == region || adjacent_row >= input.lattice_ownership[other].size())
            continue;
        const ExPolygons &owned = input.lattice_ownership[other][adjacent_row];
        other_union.insert(other_union.end(), owned.begin(), owned.end());
    }
    if (other_union.empty())
        return false;
    other_union = union_ex(other_union);
    const double delta_area = area_mm2(eroded_delta);
    if (delta_area <= 0.)
        return false;
    const double covered_area = area_mm2(intersection_ex(eroded_delta, other_union, ApplySafetyOffset::Yes));
    return covered_area >= 0.90 * delta_area;
}

FineSkinConversion plan_fine_skin_conversions(
    const NativeRegionalPlanningInput      &input,
    const std::vector<RegionalForcedPlane> &contact_planes)
{
    FineSkinConversion out;
    out.spans.resize(input.nominal_planes.size());
    const size_t rows = input.lattice_planes.size();
    if (rows == 0)
        return out;

    for (size_t region = 0; region < input.nominal_planes.size(); ++region) {
        if (region >= input.fine_skin_enabled.size() || !input.fine_skin_enabled[region] ||
            region >= input.lattice_ownership.size())
            continue;
        // The window is the same fine_skin_layers rows on every face, not the shell-layer counts.
        const int skin_layers = region < input.fine_skin_layers.size() ?
            input.fine_skin_layers[region] : 0;
        if (skin_layers <= 0)
            continue;
        const auto &owned = input.lattice_ownership[region];
        const double coarse_line_width = region < input.fine_skin_coarse_line_width.size() ?
            input.fine_skin_coarse_line_width[region] : 0.;

        // Only a face wide enough for the coarse tool to print as solid is a skin: erode the delta by
        // half the coarse line width, which removes chamfer, fillet and draft slivers (area alone does
        // not). Without a width the delta is returned unchanged. Returns the eroded polygon, which the
        // handover test also uses.
        const auto eroded_or_original = [coarse_line_width](const ExPolygons &delta) -> ExPolygons {
            if (delta.empty())
                return {};
            if (coarse_line_width <= 0.)
                return delta;
            return offset_ex(delta, float(-scale_(0.5 * coarse_line_width)));
        };

        // (a) Seed: the lattice rows the top/bottom/bridge shell windows reach.
        std::vector<char> in_window(rows, 0);
        const auto mark = [&](size_t first_row, size_t last_row) {
            for (size_t row = first_row; row <= last_row && row < rows; ++row)
                in_window[row] = 1;
        };
        for (size_t plane = 0; plane + 1 < rows; ++plane) {
            // Footprint appears going up (a floor or ceiling underside): the window sits above the
            // boundary; the handover test uses the row below.
            const ExPolygons bottom_eroded = eroded_or_original(
                diff_ex(owned[plane + 1], owned[plane], ApplySafetyOffset::Yes));
            if (!bottom_eroded.empty() && !seed_is_body_handover(input, region, plane, bottom_eroded))
                mark(plane + 1, plane + size_t(skin_layers));
            // Footprint vanishes going up (a roof or pocket floor): the window sits below the boundary;
            // the handover test uses the row above.
            const ExPolygons top_eroded = eroded_or_original(
                diff_ex(owned[plane], owned[plane + 1], ApplySafetyOffset::Yes));
            if (!top_eroded.empty() && !seed_is_body_handover(input, region, plane + 1, top_eroded))
                mark(plane + 1 >= size_t(skin_layers) ? plane + 1 - size_t(skin_layers) : 0, plane);
        }
        // A region's own outer top/bottom face has no row pair to diff, so seed it unconditionally.
        if (!owned[rows - 1].empty())
            mark(rows >= size_t(skin_layers) ? rows - size_t(skin_layers) : 0, rows - 1);
        if (!owned[0].empty())
            mark(0, std::min(rows - 1, size_t(skin_layers) - 1));

        // (b) Snap the seed to the region's no-skin coarse tiling. The shared first layer is left
        // alone. A cell already one row tall is still flagged when reached, so it gets the fine tool.
        const RegionalGridPlan baseline = plan_input_region_grid(input, region, contact_planes);
        for (const RegionCell &cell : baseline.cells) {
            if (cell.last_lattice_row <= cell.first_lattice_row && cell.first_lattice_row == 0)
                continue;
            bool touched = false;
            for (size_t row = cell.first_lattice_row; row <= cell.last_lattice_row && row < rows; ++row)
                touched = touched || in_window[row] != 0;
            if (!touched)
                continue;
            for (size_t row = cell.first_lattice_row; row <= cell.last_lattice_row && row < rows; ++row)
                out.planes.push_back({input.lattice_planes[row], RegionalRendezvousReason::FineSkin,
                                      {region}, 0.});
            out.spans[region].emplace_back(cell.bottom_z, cell.top_z);
        }
    }
    return out;
}

// Z-contouring slices a cell at its bottom plus the minimum Z, as new_layers() slices an ordinary
// layer. The shared first layer keeps its midpoint there too. Nothing changes without an offset.
void place_contour_slice_planes(RegionalGridPlan &plan, const std::optional<coordf_t> &offset)
{
    if (!offset)
        return;
    for (RegionCell &cell : plan.cells)
        if (cell.bottom_z > grid_epsilon) {
            assert(*offset <= cell.height + grid_epsilon);
            cell.slice_z = cell.bottom_z + *offset;
        }
}

bool z_ranges_overlap(const RegionCell &lhs, const RegionCell &rhs)
{
    return lhs.bottom_z < rhs.top_z - grid_epsilon && rhs.bottom_z < lhs.top_z - grid_epsilon;
}

bool regions_touch_on_lattice_row(const NativeRegionalPlanningInput &input, size_t row)
{
    for (size_t lhs = 0; lhs < input.lattice_ownership.size(); ++lhs) {
        if (input.lattice_ownership[lhs][row].empty())
            continue;
        for (size_t rhs = lhs + 1; rhs < input.lattice_ownership.size(); ++rhs) {
            if (input.lattice_ownership[rhs][row].empty())
                continue;
            const ExPolygons lhs_near = offset_ex(input.lattice_ownership[lhs][row], ClipperSafetyOffset);
            const ExPolygons rhs_near = offset_ex(input.lattice_ownership[rhs][row], ClipperSafetyOffset);
            if (!intersection_ex(lhs_near, rhs_near, ApplySafetyOffset::No).empty())
                return true;
        }
    }
    return false;
}

std::pair<bool, bool> voxel_interface_coverage(
    const NativeRegionalPlanningInput &input, coordf_t bottom_z, coordf_t top_z)
{
    bool any_contact = false;
    bool all_contact = true;
    bool has_rows = false;
    coordf_t row_bottom = 0.;
    for (size_t row = 0; row < input.lattice_planes.size(); ++row) {
        const coordf_t row_top = input.lattice_planes[row];
        if (row_top > bottom_z + grid_epsilon && row_bottom < top_z - grid_epsilon) {
            has_rows = true;
            const bool touches = regions_touch_on_lattice_row(input, row);
            any_contact = any_contact || touches;
            all_contact = all_contact && touches;
        }
        row_bottom = row_top;
        if (row_bottom >= top_z - grid_epsilon)
            break;
    }
    return {any_contact, has_rows && all_contact};
}

} // namespace

NativeRegionalGridState plan_native_regional_grids(const NativeRegionalPlanningInput &input)
{
    assert(!input.lattice_planes.empty());
    assert(input.nominal_planes.size() == input.lattice_ownership.size());
    assert(input.precedence.size() == input.nominal_planes.size());
    assert(bool(input.sample_native_ownership));
    for (const auto &rows : input.lattice_ownership)
        assert(rows.size() == input.lattice_planes.size());

    NativeRegionalGridState state;
    state.lattice_planes = input.lattice_planes;
    state.lattice_ownership = input.lattice_ownership;
    state.forced_planes = detect_forced_planes(input);
    // Conversions are derived from the no-skin tiling, so compute them from the contact planes
    // before any skin plane is appended.
    const FineSkinConversion fine_skins = plan_fine_skin_conversions(input, state.forced_planes);
    state.forced_planes.insert(state.forced_planes.end(),
                               fine_skins.planes.begin(), fine_skins.planes.end());
    state.region_plans.reserve(input.nominal_planes.size());
    state.cells.resize(input.nominal_planes.size());

    for (size_t region = 0; region < input.nominal_planes.size(); ++region) {
        state.region_plans.push_back(plan_input_region_grid(input, region, state.forced_planes));
        place_contour_slice_planes(state.region_plans.back(), input.contour_slice_offset);
    }

    // Flag cells from converted coarse cells; downstream tool and flow selection read this flag.
    for (size_t region = 0; region < state.region_plans.size() && region < fine_skins.spans.size(); ++region)
        for (RegionCell &cell : state.region_plans[region].cells)
            for (const std::pair<coordf_t, coordf_t> &span : fine_skins.spans[region])
                if (cell.bottom_z >= span.first - grid_epsilon && cell.top_z <= span.second + grid_epsilon) {
                    cell.fine_skin = true;
                    break;
                }

    // Build cell ownership highest-precedence first. This is cell-level clipping,
    // never a merge of products generated on another cadence.
    for (size_t precedence_pos = input.precedence.size(); precedence_pos-- > 0;) {
        const size_t region = input.precedence[precedence_pos];
        const RegionalGridPlan &grid = state.region_plans[region];
        auto &region_cells = state.cells[region];
        region_cells.reserve(grid.cells.size());
        for (const RegionCell &cell : grid.cells) {
            ExPolygons own_sample = input.sample_native_ownership(region, cell.slice_z);
            ExPolygons higher_footprints;
            for (size_t high_pos = precedence_pos + 1; high_pos < input.precedence.size(); ++high_pos) {
                const size_t high_region = input.precedence[high_pos];
                for (const NativeRegionCellState &higher : state.cells[high_region])
                    if (z_ranges_overlap(cell, higher.cell))
                        higher_footprints.insert(higher_footprints.end(), higher.footprint.begin(), higher.footprint.end());
            }
            if (!higher_footprints.empty())
                higher_footprints = union_ex(higher_footprints);
            const ExPolygons clipped = higher_footprints.empty() ? ExPolygons{} :
                intersection_ex(own_sample, higher_footprints, ApplySafetyOffset::Yes);
            ExPolygons footprint = higher_footprints.empty() ? own_sample :
                diff_ex(own_sample, higher_footprints, ApplySafetyOffset::Yes);
            region_cells.push_back({cell, std::move(own_sample), std::move(footprint), area_mm2(clipped)});
        }
    }

    // Native regional tops are independent. Remove only trailing empty cells;
    // an empty first cell remains legal when another region has material there.
    for (size_t region = 0; region < state.cells.size(); ++region) {
        auto &cells = state.cells[region];
        while (!cells.empty() && cells.back().footprint.empty())
            cells.pop_back();
        state.region_plans[region].cells.resize(cells.size());
        state.region_plans[region].rendezvous.erase(
            std::remove_if(state.region_plans[region].rendezvous.begin(), state.region_plans[region].rendezvous.end(),
                [&cells](const RendezvousPlane &plane) {
                    return cells.empty() || plane.z > cells.back().cell.top_z + grid_epsilon;
                }),
            state.region_plans[region].rendezvous.end());
    }

    // The retained native cells are authoritative. A planned cell whose trailing
    // footprint was discarded is not an event, while every retained cell top is.
    for (const auto &regional_cells : state.cells)
        for (const NativeRegionCellState &cell : regional_cells)
            state.event_planes.push_back(cell.cell.top_z);
    std::sort(state.event_planes.begin(), state.event_planes.end());
    state.event_planes.erase(std::unique(state.event_planes.begin(), state.event_planes.end(), same_plane),
                             state.event_planes.end());

    state.rendezvous.push_back({input.lattice_planes.front(), RegionalRendezvousReason::FirstLayer,
                                input.precedence, 0.});
    for (const RegionalForcedPlane &forced : state.forced_planes)
        // A fine-skin plane subdivides one region's cell; it is not a rendezvous (the other region
        // has no cell top there).
        if (forced.reason != RegionalRendezvousReason::FineSkin)
            state.rendezvous.push_back({forced.z, forced.reason, forced.regions, forced.max_width});

    // A region with no cell spanning a plane (for example a fully painted-over body) cannot meet
    // another region there, so drop it from that plane. A region present but without a cell top
    // there is kept, so a real handover inside a cell is still refused. A plane left with fewer
    // than two regions is not a rendezvous.
    {
        const auto present_at = [&state](size_t region, coordf_t z) {
            return region < state.cells.size() &&
                   std::any_of(state.cells[region].begin(), state.cells[region].end(),
                               [z](const NativeRegionCellState &cell) {
                                   return cell.cell.bottom_z < z + grid_epsilon && cell.cell.top_z > z - grid_epsilon;
                               });
        };
        std::vector<char> thinned(state.rendezvous.size(), 0);
        for (size_t index = 0; index < state.rendezvous.size(); ++index) {
            RendezvousPlane &plane = state.rendezvous[index];
            const size_t before = plane.regions.size();
            plane.regions.erase(std::remove_if(plane.regions.begin(), plane.regions.end(),
                                               [&](size_t region) { return !present_at(region, plane.z); }),
                                plane.regions.end());
            thinned[index] = plane.regions.size() != before ? 1 : 0;
        }
        size_t kept = 0;
        for (size_t index = 0; index < state.rendezvous.size(); ++index)
            if (!thinned[index] || state.rendezvous[index].regions.size() >= 2) {
                if (kept != index)
                    state.rendezvous[kept] = std::move(state.rendezvous[index]);
                ++kept;
            }
        state.rendezvous.resize(kept);
    }

    // When a region starts on its own first plane or holds cells to its nozzle's minimum, a
    // published plane may not end a cell in every listed region: move the first-layer plane up to
    // the first plane all regions meet at, drop regions that could not take a contact cut, and
    // drop planes left with fewer than two regions.
    if (envelope_grids_active(input)) {
        const auto has_top = [&state](size_t region, coordf_t z) {
            return region < state.cells.size() &&
                   std::any_of(state.cells[region].begin(), state.cells[region].end(),
                               [z](const NativeRegionCellState &cell) { return same_plane(cell.cell.top_z, z); });
        };
        coordf_t first_met = input.lattice_planes.front();
        for (size_t region : input.precedence)
            if (region < state.cells.size() && !state.cells[region].empty())
                first_met = std::max(first_met, state.cells[region].front().cell.top_z);
        for (RendezvousPlane &plane : state.rendezvous) {
            if (plane.reason == RegionalRendezvousReason::FirstLayer)
                plane.z = first_met;
            plane.regions.erase(std::remove_if(plane.regions.begin(), plane.regions.end(),
                                               [&](size_t region) { return !has_top(region, plane.z); }),
                                plane.regions.end());
        }
        state.rendezvous.erase(std::remove_if(state.rendezvous.begin(), state.rendezvous.end(),
                                              [](const RendezvousPlane &plane) { return plane.regions.size() < 2; }),
                               state.rendezvous.end());
    }

    if (input.interlocking.enabled) {
        assert(input.interlocking.beam_layer_count > 0);
        const size_t coarse_region = size_t(std::distance(input.nominal_planes.begin(),
            std::max_element(input.nominal_planes.begin(), input.nominal_planes.end(),
                [](const auto &lhs, const auto &rhs) {
                    if (lhs.size() < 2 || rhs.size() < 2)
                        return lhs.size() > rhs.size();
                    return lhs[1] - lhs[0] < rhs[1] - rhs[0];
                })));
        const auto &nominal = input.nominal_planes[coarse_region];
        const coordf_t fine_cadence = input.lattice_planes.size() > 1 ?
            input.lattice_planes[1] - input.lattice_planes[0] : nominal.front();
        const coordf_t voxel_height = 2. * coordf_t(input.interlocking.beam_layer_count) * fine_cadence;
        // The shared first layer, and a coarse first cell standing on the bed, are never transferred.
        coordf_t bottom = input.lattice_planes.front();
        if (coarse_region < state.region_plans.size() && !state.region_plans[coarse_region].cells.empty())
            bottom = std::max(bottom, state.region_plans[coarse_region].cells.front().top_z);
        // Phase the voxels from where the bodies start to touch, not from the bed, so a body standing
        // on another one gets a first voxel covering the bottom of the contact band.
        for (size_t row = 0; row < input.lattice_planes.size(); ++row)
            if (regions_touch_on_lattice_row(input, row)) {
                const coordf_t contact_bottom = row == 0 ? 0. : input.lattice_planes[row - 1];
                const auto plane = std::find_if(nominal.begin(), nominal.end(),
                    [contact_bottom](coordf_t z) { return z >= contact_bottom - grid_epsilon; });
                if (plane != nominal.end()) {
                    // If the band starts at a stacked contact, start one coarse layer lower so the first voxel
                    // straddles that plane and interlocks both ways.
                    const bool stacked_start = std::any_of(state.forced_planes.begin(), state.forced_planes.end(),
                        [contact_bottom, &plane](const RegionalForcedPlane &forced) {
                            return forced.reason == RegionalRendezvousReason::StackedContact &&
                                   forced.z >= contact_bottom - grid_epsilon && forced.z <= *plane + grid_epsilon;
                        });
                    bottom = std::max(bottom, stacked_start && plane != nominal.begin() ? *std::prev(plane) : *plane);
                }
                break;
            }
        state.interlocking_origin_z = bottom;
        const coordf_t valid_top = nominal.empty() ? bottom : nominal.back();
        while (bottom + voxel_height <= valid_top + grid_epsilon) {
            const coordf_t top = bottom + voxel_height;
            const auto [any_contact, full_contact] = voxel_interface_coverage(input, bottom, top);
            const auto crossing = std::find_if(state.forced_planes.begin(), state.forced_planes.end(),
                [bottom, top](const RegionalForcedPlane &forced) {
                    // A fine-skin plane does not interrupt the two-region interface.
                    return forced.reason != RegionalRendezvousReason::FineSkin &&
                           forced.z > bottom + grid_epsilon && forced.z < top - grid_epsilon;
                });
            // Skip voxels outside the interface. A partially covered voxel is kept only when a forced
            // plane explains the interruption, and is then recorded as a dropped beam.
            if (!any_contact || (!full_contact && crossing == state.forced_planes.end())) {
                bottom = top;
                continue;
            }
            // A forced plane inside the voxel is a cell boundary in both regions, so by itself it does not
            // interrupt the interface. Row-uniformity is checked on the real geometry by the rollback
            // pass in PrintObjectSlice.cpp. A voxel straddling a stacked contact, with contact on every
            // row above it, is also a full interface.
            bool stacked_full = false;
            if (!full_contact && crossing != state.forced_planes.end() &&
                crossing->reason == RegionalRendezvousReason::StackedContact) {
                const auto [above_any, above_full] = voxel_interface_coverage(input, crossing->z, top);
                stacked_full = above_any && above_full;
            }
            const bool apply = full_contact || stacked_full;
            state.interlocking_decisions.push_back({bottom, top, apply});
            if (!apply && crossing != state.forced_planes.end())
                state.rendezvous.push_back({crossing->z, RegionalRendezvousReason::InterlockingDropped,
                                            crossing->regions, crossing->max_width});
            bottom = top;
        }
    }

    return state;
}

std::vector<coordf_t> native_regional_cell_midplanes(const NativeRegionalPlanningInput &input,
                                                   std::vector<RegionalGridPlan> *complete_plans)
{
    assert(!input.lattice_planes.empty());
    assert(input.nominal_planes.size() == input.lattice_ownership.size());
    // The sampled midplanes must include the fine-skin cells.
    std::vector<RegionalForcedPlane> forced = detect_forced_planes(input);
    const FineSkinConversion fine_skins = plan_fine_skin_conversions(input, forced);
    forced.insert(forced.end(), fine_skins.planes.begin(), fine_skins.planes.end());
    std::vector<coordf_t> midplanes;
    if (complete_plans)
        complete_plans->clear();
    for (size_t region = 0; region < input.nominal_planes.size(); ++region) {
        RegionalGridPlan grid = plan_input_region_grid(input, region, forced);
        place_contour_slice_planes(grid, input.contour_slice_offset);
        for (const RegionCell &cell : grid.cells)
            midplanes.push_back(cell.slice_z);
        if (complete_plans)
            complete_plans->push_back(grid);
    }
    std::sort(midplanes.begin(), midplanes.end());
    midplanes.erase(std::unique(midplanes.begin(), midplanes.end(), same_plane), midplanes.end());
    return midplanes;
}

} // namespace Slic3r
