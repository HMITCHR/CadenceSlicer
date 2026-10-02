#include "RegionalLayerBands.hpp"

#include "LocalesUtils.hpp"
#include "PrintConfig.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>

namespace Slic3r {

bool regional_band_admits_sparse_infill_pattern(InfillPattern pattern)
{
    // A band prints two layers' sparse fill as one 2h extrusion, so the pattern must draw the same
    // geometry on both layers: none of these consults Fill::z or rotates/shifts by layer_id (ipGrid
    // only reverses direction). Everything else is refused, including ipRectilinear (alternates by
    // layer parity), ipLateralLattice (shift scales with z) and every z-sampled pattern.
    switch (pattern) {
    case ipGrid:
    case ipTriangles:
    case ipStars:
    case ipAlignedRectilinear:
        return true;
    default:
        return false;
    }
}

bool regional_band_admits_sparse_infill_density(double density_percent)
{
    // 0% has nothing to combine and 100% is solid; anything between is admitted, since the band
    // is a surface retype independent of line spacing.
    return density_percent > EPSILON && density_percent < 100. - EPSILON;
}

bool feature_combine_admits_sparse_infill_pattern(InfillPattern pattern)
{
    // ipLockedZag interlocks with the shell over a Z-depth window that assumes every layer in it is
    // drawn separately, which a merged band removes. ipLightning is generated for the whole object
    // from the layer stack. Every other pattern is admitted: native combine draws it once, on the
    // band's top layer.
    switch (pattern) {
    case ipLockedZag:
    case ipLightning:
        return false;
    default:
        return true;
    }
}

namespace {

// Fixed decimal places for every emitted Z and height; the consumer rounds band tops to this
// precision before comparing.
constexpr int regional_plan_decimals = 8;

std::string regional_plan_number(coordf_t value)
{
    return float_to_string_decimal_point(value, regional_plan_decimals);
}

} // namespace

RegionalLayerPlanText format_regional_layer_plan(
    const std::vector<RegionalGridPlanRecord> &grids,
    const std::vector<RendezvousPlane>        &rendezvous,
    const std::vector<coordf_t>               &event_planes,
    coordf_t                                   tolerance,
    coordf_t                                   base_cadence,
    int                                         toolchange_total,
    int                                         toolchange_per_band,
    const std::optional<RegionalSupportPlanRecord> &support)
{
    constexpr coordf_t epsilon = 1e-8;
    const auto same = [](coordf_t lhs, coordf_t rhs) { return std::abs(lhs - rhs) <= 1e-8; };
    const auto has_top = [&same](const RegionalGridPlanRecord &grid, coordf_t z) {
        return std::any_of(grid.cells.begin(), grid.cells.end(),
                           [z, &same](const RegionCell &cell) { return same(cell.top_z, z); });
    };

    if (grids.empty() || base_cadence <= epsilon)
        return {RegionalPlanFormatError::GridNotContiguous, {}};

    for (const RegionalGridPlanRecord &grid : grids) {
        if (grid.cells.empty() || grid.cadence <= epsilon)
            return {RegionalPlanFormatError::GridNotContiguous, {}};
        coordf_t expected_bottom = 0.;
        for (size_t index = 0; index < grid.cells.size(); ++index) {
            const RegionCell &cell = grid.cells[index];
            if (cell.region_id != grid.region_id || cell.cell_index != index ||
                !same(cell.bottom_z, expected_bottom) || cell.top_z <= cell.bottom_z + epsilon)
                return {RegionalPlanFormatError::GridNotContiguous, {}};
            const coordf_t derived_height = cell.top_z - cell.bottom_z;
            const coordf_t base_multiple = derived_height / base_cadence;
            // Cell 0 is the shared first layer at the vendor first-layer height, which is neither capped by
            // the cadence nor a multiple of the base; validation keeps it inside both tools' envelopes.
            // Cell 0 still has to agree with its own Z span.
            const bool shared_first_layer = index == 0 && cell.bottom_z <= epsilon;
            if (!same(cell.height, derived_height) ||
                (!shared_first_layer &&
                 (derived_height > grid.cadence + epsilon ||
                  std::abs(base_multiple - std::round(base_multiple)) > epsilon)))
                return {RegionalPlanFormatError::CellHeightIllegal, {}};
            expected_bottom = cell.top_z;
        }
    }

    for (const RendezvousPlane &plane : rendezvous) {
        if (plane.regions.size() < 2)
            return {RegionalPlanFormatError::RendezvousNotShared, {}};
        for (size_t region : plane.regions) {
            const auto grid = std::find_if(grids.begin(), grids.end(), [region](const RegionalGridPlanRecord &candidate) {
                return candidate.region_id == region;
            });
            if (grid == grids.end() || !has_top(*grid, plane.z))
                return {RegionalPlanFormatError::RendezvousNotShared, {}};
        }
    }

    for (coordf_t event : event_planes)
        if (std::none_of(grids.begin(), grids.end(), [event, &has_top](const RegionalGridPlanRecord &grid) {
                return has_top(grid, event);
            }))
            return {RegionalPlanFormatError::EventPlaneWithoutCell, {}};
    for (const RegionalGridPlanRecord &grid : grids)
        for (const RegionCell &cell : grid.cells)
            if (std::none_of(event_planes.begin(), event_planes.end(), [&cell, &same](coordf_t event) {
                    return same(event, cell.top_z);
                }))
                return {RegionalPlanFormatError::EventPlaneWithoutCell, {}};

    std::vector<const RegionalGridPlanRecord *> ordered;
    ordered.reserve(grids.size());
    for (const RegionalGridPlanRecord &grid : grids)
        ordered.push_back(&grid);
    std::sort(ordered.begin(), ordered.end(), [](const RegionalGridPlanRecord *lhs, const RegionalGridPlanRecord *rhs) {
        return lhs->region_id < rhs->region_id;
    });

    const auto phase_name = [](RegionalGridPhaseRule phase) {
        return phase == RegionalGridPhaseRule::KeepNominal ? "KEEP" : "REPHASE";
    };
    const auto reason_name = [](RegionalRendezvousReason reason) {
        switch (reason) {
        case RegionalRendezvousReason::StackedContact:      return "SRL-R01";
        case RegionalRendezvousReason::FirstLayer:          return "SRL-R02";
        case RegionalRendezvousReason::UserExact:           return "SRL-R03";
        case RegionalRendezvousReason::InterlockingDropped: return "SRL-R04";
        case RegionalRendezvousReason::FineSkin:            return "SRL-R05";
        }
        return "SRL-R00";
    };

    std::string text = "; SRL_PLAN_START\n";
    for (const RegionalGridPlanRecord *grid : ordered) {
        text += "; SRL_GRID region=" + std::to_string(grid->region_id) +
                " tool=" + std::to_string(grid->physical_extruder_id) +
                " nozzle=" + float_to_string_decimal_point(grid->nozzle_diameter, 2) +
                " cadence=" + regional_plan_number(grid->cadence) +
                " cells=" + std::to_string(grid->cells.size()) +
                " top_z=" + regional_plan_number(grid->cells.back().top_z);
        if (!same(grid->cadence, base_cadence))
            text += " phase=" + std::string(phase_name(grid->phase_rule));
        text += '\n';
    }
    for (const RegionalGridPlanRecord *grid : ordered)
        if (!same(grid->cadence, base_cadence))
            for (const RegionCell &cell : grid->cells)
                text += "; SRL_CELL region=" + std::to_string(grid->region_id) +
                        " tool=" + std::to_string(cell.physical_extruder_id) +
                        " z_lo=" + regional_plan_number(cell.bottom_z) +
                        " z_hi=" + regional_plan_number(cell.top_z) +
                        " height=" + regional_plan_number(cell.height) + "\n";
    for (const RendezvousPlane &plane : rendezvous) {
        text += "; SRL_RENDEZVOUS z=" + regional_plan_number(plane.z) +
                " reason=" + reason_name(plane.reason) + " regions=";
        for (size_t index = 0; index < plane.regions.size(); ++index) {
            if (index != 0)
                text += ',';
            text += std::to_string(plane.regions[index]);
        }
        if (plane.max_width > epsilon)
            text += " contact_w=" + regional_plan_number(plane.max_width);
        text += '\n';
    }
    if (support)
        text += "; SRL_SUPPORT base_tool=" + std::to_string(support->base_tool) +
                " base_height=" + regional_plan_number(support->base_height) +
                " interface_tool=" + std::to_string(support->interface_tool) +
                " interface_height=" + regional_plan_number(support->interface_height) + "\n";
    text += "; SRL_TOLERANCE value=" + regional_plan_number(tolerance) + "\n";
    text += "; SRL_TOOLCHANGES total=" + std::to_string(toolchange_total) +
            " per_band=" + std::to_string(toolchange_per_band) + "\n";
    text += "; SRL_PLAN_END\n";
    return {RegionalPlanFormatError::None, std::move(text)};
}

RegionalLayerPlanText format_regional_layer_plan_feature(const std::vector<RegionalGridPlanRecord> &bands)
{
    // Zero bands is a valid Feature Split plan (no sparse infill scheduled): emit the header only.
    for (const RegionalGridPlanRecord &group : bands) {
        if (group.mode != RegionalPlanRecordMode::Feature || group.cells.empty() ||
            group.cadence <= EPSILON || group.nozzle_diameter <= EPSILON)
            return {RegionalPlanFormatError::FeatureBandInvalid, {}};
        for (const RegionCell &cell : group.cells)
            if (cell.top_z <= cell.bottom_z + EPSILON || ! is_approx(cell.height, cell.top_z - cell.bottom_z))
                return {RegionalPlanFormatError::FeatureBandInvalid, {}};
    }

    std::vector<const RegionalGridPlanRecord *> ordered;
    ordered.reserve(bands.size());
    for (const RegionalGridPlanRecord &group : bands)
        ordered.push_back(&group);
    std::sort(ordered.begin(), ordered.end(), [](const RegionalGridPlanRecord *lhs, const RegionalGridPlanRecord *rhs) {
        return lhs->region_id < rhs->region_id;
    });

    size_t total_bands = 0;
    for (const RegionalGridPlanRecord *group : ordered)
        total_bands += group->cells.size();

    // The header line always comes first, so a consumer can tell an empty plan from a parse failure.
    std::string text = "; SRL_PLAN_START\n";
    text += "; SRL_FEATURE bands=" + std::to_string(total_bands) + "\n";
    for (const RegionalGridPlanRecord *group : ordered)
        for (const RegionCell &cell : group->cells)
            text += "; SRL_BAND mode=feature region=" + std::to_string(group->region_id) +
                    " tool=" + std::to_string(group->physical_extruder_id) +
                    " nozzle=" + float_to_string_decimal_point(group->nozzle_diameter, 2) +
                    " cadence=" + regional_plan_number(group->cadence) +
                    " z_lo=" + regional_plan_number(cell.bottom_z) +
                    " z_hi=" + regional_plan_number(cell.top_z) +
                    " height=" + regional_plan_number(cell.height) + "\n";
    text += "; SRL_PLAN_END\n";
    return {RegionalPlanFormatError::None, std::move(text)};
}

} // namespace Slic3r
