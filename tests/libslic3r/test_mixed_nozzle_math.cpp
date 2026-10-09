#include <catch2/catch_all.hpp>

#include "libslic3r/RegionalGrids.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Slicing.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using Catch::Matchers::WithinAbs;
using namespace Slic3r;

namespace {

std::vector<coordf_t> planes(coordf_t first, coordf_t last, coordf_t step)
{
    std::vector<coordf_t> out;
    for (coordf_t z = first; z <= last + 1e-9; z += step)
        out.push_back(z);
    return out;
}

void require_contiguous(const RegionalGridPlan &plan)
{
    REQUIRE_FALSE(plan.cells.empty());
    CHECK_THAT(plan.cells.front().bottom_z, WithinAbs(0., 1e-9));
    for (size_t i = 0; i < plan.cells.size(); ++i) {
        const RegionCell &cell = plan.cells[i];
        CHECK(cell.cell_index == i);
        CHECK_THAT(cell.height, WithinAbs(cell.top_z - cell.bottom_z, 1e-9));
        CHECK_THAT(cell.slice_z, WithinAbs(0.5 * (cell.bottom_z + cell.top_z), 1e-9));
        if (i > 0)
            CHECK_THAT(cell.bottom_z, WithinAbs(plan.cells[i - 1].top_z, 1e-9));
    }
}

void require_cell_heights_within(const RegionalGridPlan &plan, coordf_t h, coordf_t h_r)
{
    REQUIRE_FALSE(plan.cells.empty());
    for (const RegionCell &cell : plan.cells) {
        INFO("cell " << cell.cell_index << " [" << cell.bottom_z << ", " << cell.top_z << "]");
        CHECK(cell.height >= h - 1e-9);
        CHECK(cell.height <= h_r + 1e-9);
        const coordf_t multiple = cell.height / h;
        CHECK_THAT(multiple, WithinAbs(std::round(multiple), 1e-9));
    }
}

void require_keep_nominal_contract(const RegionalGridPlan &plan,
                                   const std::vector<coordf_t> &nominal,
                                   const std::vector<RegionalForcedPlane> &forced)
{
    REQUIRE_FALSE(plan.cells.empty());
    for (const RegionCell &cell : plan.cells) {
        const bool is_nominal = std::any_of(nominal.begin(), nominal.end(),
            [&cell](coordf_t z) { return std::abs(z - cell.top_z) < 1e-9; });
        const bool is_forced = std::any_of(forced.begin(), forced.end(),
            [&cell](const RegionalForcedPlane &plane) { return std::abs(plane.z - cell.top_z) < 1e-9; });
        INFO("cell " << cell.cell_index << " top_z " << cell.top_z
             << " is neither a nominal plane nor a forced plane");
        if (!is_nominal)
            CHECK(is_forced);
    }
    CHECK_THAT(plan.cells.back().top_z, WithinAbs(nominal.back(), 1e-9));
}

ExPolygons rectangle(double x0, double y0, double x1, double y1)
{
    ExPolygon box;
    box.contour.points = {{scale_(x0), scale_(y0)}, {scale_(x1), scale_(y0)},
                          {scale_(x1), scale_(y1)}, {scale_(x0), scale_(y1)}};
    return {std::move(box)};
}

double area_mm2(const ExPolygons &polygons)
{
    return unscale<double>(unscale<double>(area(polygons)));
}

} // namespace

TEST_CASE("Regional grid solver preserves nominal cells without forced planes", "[TestRebuild][RegionalMath]")
{
    const std::vector<coordf_t> lattice = planes(0.1, 0.5, 0.1);
    const std::vector<coordf_t> nominal{0.1, 0.3, 0.5};

    for (RegionalGridPhaseRule rule : {RegionalGridPhaseRule::Rephase, RegionalGridPhaseRule::KeepNominal}) {
        const RegionalGridPlan plan = plan_regional_grid(7, lattice, nominal, {}, rule);
        require_contiguous(plan);
        require_cell_heights_within(plan, 0.1, 0.2);
        if (rule == RegionalGridPhaseRule::KeepNominal)
            require_keep_nominal_contract(plan, nominal, {});
        REQUIRE(plan.cells.size() == nominal.size());
        REQUIRE(plan.rendezvous.size() == 1);
        CHECK(plan.rendezvous.front().reason == RegionalRendezvousReason::FirstLayer);
        for (size_t i = 0; i < nominal.size(); ++i)
            CHECK_THAT(plan.cells[i].top_z, WithinAbs(nominal[i], 1e-9));
    }
}

TEST_CASE("Regional grid solver applies the selected phase rule around one forced plane", "[TestRebuild][RegionalMath]")
{
    const std::vector<coordf_t> lattice = planes(0.1, 0.6, 0.1);
    const std::vector<coordf_t> nominal{0.1, 0.3, 0.5};
    const std::vector<RegionalForcedPlane> forced{{0.2, RegionalRendezvousReason::StackedContact, {0, 1}, 1.5}};

    const RegionalGridPlan rephase = plan_regional_grid(1, lattice, nominal, forced, RegionalGridPhaseRule::Rephase);
    const RegionalGridPlan keep = plan_regional_grid(1, lattice, nominal, forced, RegionalGridPhaseRule::KeepNominal);
    require_contiguous(rephase);
    require_contiguous(keep);
    require_cell_heights_within(rephase, 0.1, 0.2);
    require_cell_heights_within(keep, 0.1, 0.2);
    require_keep_nominal_contract(keep, nominal, forced);

    REQUIRE(rephase.cells.size() == 4);
    CHECK_THAT(rephase.cells[0].top_z, WithinAbs(0.1, 1e-9));
    CHECK_THAT(rephase.cells[1].top_z, WithinAbs(0.2, 1e-9));
    CHECK_THAT(rephase.cells[2].top_z, WithinAbs(0.4, 1e-9));
    CHECK_THAT(rephase.cells[3].top_z, WithinAbs(0.6, 1e-9));

    REQUIRE(keep.cells.size() == 4);
    CHECK_THAT(keep.cells[0].top_z, WithinAbs(0.1, 1e-9));
    CHECK_THAT(keep.cells[1].top_z, WithinAbs(0.2, 1e-9));
    CHECK_THAT(keep.cells[2].top_z, WithinAbs(0.3, 1e-9));
    CHECK_THAT(keep.cells[3].top_z, WithinAbs(0.5, 1e-9));

    for (const RegionalGridPlan *plan : {&rephase, &keep}) {
        REQUIRE(plan->rendezvous.size() == 2);
        const RendezvousPlane &contact = plan->rendezvous.back();
        CHECK_THAT(contact.z, WithinAbs(0.2, 1e-9));
        CHECK(contact.reason == RegionalRendezvousReason::StackedContact);
        CHECK(contact.regions == std::vector<size_t>{0, 1});
        CHECK_THAT(contact.max_width, WithinAbs(1.5, 1e-9));
        for (const RegionCell &cell : plan->cells)
            if (std::abs(cell.height - 0.1) >= 1e-9)
                CHECK_THAT(cell.height, WithinAbs(0.2, 1e-9));
    }

    // A fine-skin plane cuts only the region that opted in. As region 0's skin,
    // the plane that cut region 1 above as a contact leaves region 1 uncut.
    const std::vector<RegionalForcedPlane> skin{{0.2, RegionalRendezvousReason::FineSkin, {0}, 1.5}};
    for (RegionalGridPhaseRule rule : {RegionalGridPhaseRule::Rephase, RegionalGridPhaseRule::KeepNominal}) {
        const RegionalGridPlan plain = plan_regional_grid(1, lattice, nominal, {}, rule);
        const RegionalGridPlan other = plan_regional_grid(1, lattice, nominal, skin, rule);
        REQUIRE(other.cells.size() == plain.cells.size());
        for (size_t i = 0; i < plain.cells.size(); ++i)
            CHECK_THAT(other.cells[i].top_z, WithinAbs(plain.cells[i].top_z, 1e-9));
        const RegionalGridPlan own = plan_regional_grid(0, lattice, nominal, skin, rule);
        CHECK(std::any_of(own.cells.begin(), own.cells.end(),
            [](const RegionCell &cell) { return std::abs(cell.top_z - 0.2) < 1e-9; }));
    }
}

TEST_CASE("Regional grid solver keeps fine-skin planes in their own region when cells are held to a minimum", "[TestRebuild][RegionalMath]")
{
    // Two lattice rows per cell sends the solver down its held-to-minimum path.
    const std::vector<coordf_t> lattice = planes(0.1, 1.1, 0.1);
    const std::vector<coordf_t> nominal = planes(0.1, 1.1, 0.2);
    const std::vector<RegionalForcedPlane> skin{{0.6, RegionalRendezvousReason::FineSkin, {0}, 0.}};
    for (RegionalGridPhaseRule rule : {RegionalGridPhaseRule::Rephase, RegionalGridPhaseRule::KeepNominal}) {
        const RegionalGridPlan plain = plan_regional_grid(1, lattice, nominal, {}, rule, 2);
        const RegionalGridPlan other = plan_regional_grid(1, lattice, nominal, skin, rule, 2);
        require_contiguous(other);
        REQUIRE(other.cells.size() == plain.cells.size());
        for (size_t i = 0; i < plain.cells.size(); ++i)
            CHECK_THAT(other.cells[i].top_z, WithinAbs(plain.cells[i].top_z, 1e-9));
        CHECK(std::none_of(other.rendezvous.begin(), other.rendezvous.end(),
            [](const RendezvousPlane &plane) { return plane.reason == RegionalRendezvousReason::FineSkin; }));
        const RegionalGridPlan own = plan_regional_grid(0, lattice, nominal, skin, rule, 2);
        CHECK(std::any_of(own.cells.begin(), own.cells.end(),
            [](const RegionCell &cell) { return std::abs(cell.top_z - 0.6) < 1e-9; }));
    }
}

TEST_CASE("Regional grid solver keeps both phase-rule tails bounded", "[TestRebuild][RegionalMath]")
{
    const std::vector<coordf_t> lattice = planes(0.1, 1.0, 0.1);
    std::vector<coordf_t> nominal{0.1};
    const std::vector<coordf_t> rest = planes(0.3, 0.9, 0.2);
    nominal.insert(nominal.end(), rest.begin(), rest.end());
    const std::vector<RegionalForcedPlane> forced{{0.6, RegionalRendezvousReason::StackedContact, {0, 1}, 1.5}};

    const RegionalGridPlan rephase = plan_regional_grid(1, lattice, nominal, forced, RegionalGridPhaseRule::Rephase);
    const RegionalGridPlan keep = plan_regional_grid(1, lattice, nominal, forced, RegionalGridPhaseRule::KeepNominal);
    require_contiguous(rephase);
    require_contiguous(keep);
    require_cell_heights_within(rephase, 0.1, 0.2);
    require_cell_heights_within(keep, 0.1, 0.2);
    require_keep_nominal_contract(keep, nominal, forced);

    CHECK_THAT(rephase.cells.back().top_z, WithinAbs(1.0, 1e-9));
    CHECK_THAT(keep.cells.back().top_z, WithinAbs(0.9, 1e-9));

    const auto rephase_contact = std::find_if(rephase.cells.begin(), rephase.cells.end(),
        [](const RegionCell &cell) { return std::abs(cell.top_z - 0.6) < 1e-9; });
    const auto keep_contact = std::find_if(keep.cells.begin(), keep.cells.end(),
        [](const RegionCell &cell) { return std::abs(cell.top_z - 0.6) < 1e-9; });
    REQUIRE(rephase_contact != rephase.cells.end());
    REQUIRE(keep_contact != keep.cells.end());
    CHECK_THAT(rephase_contact->height, WithinAbs(0.1, 1e-9));
    CHECK_THAT(keep_contact->height, WithinAbs(0.1, 1e-9));
    CHECK_THAT((keep_contact + 1)->height, WithinAbs(0.1, 1e-9));
    CHECK_THAT((rephase_contact + 1)->height, WithinAbs(0.2, 1e-9));
}

TEST_CASE("Regional profile uses only its declared physical nozzle and fixed cadence", "[TestRebuild][RegionalMath]")
{
    PrintConfig print_config;
    print_config.nozzle_diameter.values = {0.2, 0.4};
    print_config.min_layer_height.values = {0.04, 0.08};
    print_config.max_layer_height.values = {0.14, 0.28};
    print_config.filament_map.values = {1, 2};
    print_config.filament_map_mode.value = fmmManual;
    print_config.initial_layer_print_height.value = 0.1;

    PrintObjectConfig object_config;
    object_config.layer_height.value = 0.1;
    object_config.enable_support.value = true;
    object_config.raft_layers.value = 2;

    const RegionalSlicingPlan plan = make_regional_slicing_plan(
        print_config, object_config, 1.2, 1, Vec3d::Ones(), 0.2);

    CHECK_THAT(plan.parameters.layer_height, WithinAbs(0.2, 1e-12));
    CHECK_THAT(plan.parameters.min_layer_height, WithinAbs(0.08, 1e-12));
    CHECK_THAT(plan.parameters.max_layer_height, WithinAbs(0.28, 1e-12));
    CHECK(plan.parameters.raft_layers() == 0);
    CHECK(plan.profile == std::vector<coordf_t>{0., 0.1, 0.1, 0.1, 0.1, 0.2, 1.2, 0.2});
    REQUIRE(plan.nominal_planes.size() > 2);
    CHECK_THAT(plan.nominal_planes[0], WithinAbs(0.1, 1e-12));
    CHECK_THAT(plan.nominal_planes[1], WithinAbs(0.3, 1e-12));
}

TEST_CASE("Native regional planner forces a stacked handoff after an open gap", "[TestRebuild][RegionalMath]")
{
    const std::vector<coordf_t> lattice = planes(.1, 1.0, .1);
    std::vector<coordf_t> coarse_nominal{0.1};
    const std::vector<coordf_t> coarse_rest = planes(.3, .9, .2);
    coarse_nominal.insert(coarse_nominal.end(), coarse_rest.begin(), coarse_rest.end());

    NativeRegionalPlanningInput input;
    input.lattice_planes = lattice;
    input.nominal_planes = {planes(.1, .8, .1), coarse_nominal};
    input.precedence = {0, 1};
    input.interface_tolerance = 0.;
    input.lattice_ownership.resize(2, std::vector<ExPolygons>(lattice.size()));

    for (size_t row = 0; row < lattice.size(); ++row) {
        const double z = .05 + .1 * double(row);
        if (z < .5) {
            input.lattice_ownership[0][row] = rectangle(0., 0., 2., 3.);
            input.lattice_ownership[1][row] = rectangle(4., 0., 8., 3.);
        } else if (z < .8) {
            input.lattice_ownership[0][row] = rectangle(0., 0., 4., 3.);
            input.lattice_ownership[1][row] = rectangle(4., 0., 8., 3.);
        } else {
            input.lattice_ownership[1][row] = rectangle(0., 0., 8., 3.);
        }
    }
    input.sample_native_ownership = [&input](size_t region, coordf_t z) {
        const size_t row = std::min(size_t(std::max(0., std::floor(z / 0.1))), input.lattice_planes.size() - 1);
        return input.lattice_ownership[region][row];
    };

    for (RegionalGridPhaseRule rule : {RegionalGridPhaseRule::Rephase, RegionalGridPhaseRule::KeepNominal}) {
        input.phase_rule = rule;
        const NativeRegionalGridState state = plan_native_regional_grids(input);

        REQUIRE(state.forced_planes.size() == 1);
        CHECK_THAT(state.forced_planes.front().z, WithinAbs(.8, 1e-9));
        CHECK(state.forced_planes.front().reason == RegionalRendezvousReason::StackedContact);
        REQUIRE(state.region_plans.size() == 2);
        require_contiguous(state.region_plans[0]);
        require_contiguous(state.region_plans[1]);
        require_cell_heights_within(state.region_plans[1], .1, .2);
        CHECK_THAT(state.region_plans[1].cells.back().top_z,
                   WithinAbs(rule == RegionalGridPhaseRule::Rephase ? 1.0 : .9, 1e-9));
    }
}

TEST_CASE("Declared later-volume precedence resolves a narrow overlap", "[TestRebuild][RegionalMath]")
{
    const std::vector<ExPolygons> raw{
        rectangle(0., 0., 8., 6.),
        rectangle(7.996, 0., 16., 6.)
    };
    const RegionalOwnershipResolution ownership = resolve_regional_ownership(raw, {0, 1}, 0.01);

    CHECK_FALSE(ownership.b01_overlap);
    CHECK_THAT(ownership.b01_overlap_mm2, WithinAbs(0., 1e-9));
    CHECK_THAT(area_mm2(intersection_ex(ownership.resolved[0], ownership.resolved[1])), WithinAbs(0., 1e-9));
    CHECK_THAT(area_mm2(ownership.resolved[1]), WithinAbs(48.024, 1e-5));
    CHECK_THAT(area_mm2(ownership.resolved[0]), WithinAbs(47.976, 2e-4));
}

// A flush inlay part in its pocket shares a rounding sliver with the pocket's walls: 0.017 mm2 on one
// layer in a measured text inlay, wider than the 0.01 mm opening removes. Under one dot of the finest
// nozzle it is not an overlap; a part sunk 0.3 mm into another still is.
TEST_CASE("A sliver smaller than one dot of the finest nozzle is not a Body Split overlap", "[TestRebuild][RegionalMath]")
{
    const double sliver = body_split_overlap_sliver_mm2(0.2);
    CHECK_THAT(sliver, WithinAbs(0.25 * M_PI * 0.04, 1e-9));

    // 0.025 mm by 0.67 mm shared: about 0.017 mm2.
    const std::vector<ExPolygons> inlay{
        rectangle(0., 0., 8., 6.),
        rectangle(7.975, 2., 12., 2.67)
    };
    const RegionalOwnershipResolution flush = resolve_regional_ownership(inlay, {0, 1}, 0.01, sliver);
    CHECK_FALSE(flush.b01_overlap);
    // The sliver still goes to the later part, so the two never share a road.
    CHECK_THAT(area_mm2(intersection_ex(flush.resolved[0], flush.resolved[1])), WithinAbs(0., 1e-9));
    // Without the threshold it is reported, as before.
    CHECK(resolve_regional_ownership(inlay, {0, 1}, 0.01).b01_overlap);

    const std::vector<ExPolygons> sunk{
        rectangle(0., 0., 8., 6.),
        rectangle(7.7, 0., 16., 6.)
    };
    const RegionalOwnershipResolution overlap = resolve_regional_ownership(sunk, {0, 1}, 0.01, sliver);
    CHECK(overlap.b01_overlap);
    CHECK_THAT(overlap.b01_overlap_mm2, WithinAbs(1.8, 1e-3));
}

// A smaller part sunk into a bigger one keeps the shared space only when the overlap is a minor part
// of the bigger one and the smaller part is clearly the smaller. Text sunk 0.2 mm into a 60 x 40 x 10
// base (237 mm2 of letters) shares about 47 mm3 of 24000.
TEST_CASE("Body Split keeps a minor overlap for the smaller part and refuses the rest", "[TestRebuild][RegionalMath]")
{
    CHECK(body_split_smaller_part_keeps_overlap(450., 24000., 47.));
    // A part wholly inside the bigger one, if small enough, is fine too.
    CHECK(body_split_smaller_part_keeps_overlap(300., 24000., 300.));
    // Over 10% of the bigger part shared.
    CHECK_FALSE(body_split_smaller_part_keeps_overlap(300., 1200., 300.));
    CHECK(body_split_smaller_part_keeps_overlap(300., 1200., 120.));
    // Not clearly the smaller part.
    CHECK_FALSE(body_split_smaller_part_keeps_overlap(300., 300., 15.));
    CHECK(body_split_smaller_part_keeps_overlap(150., 300., 15.));
    CHECK_FALSE(body_split_smaller_part_keeps_overlap(0., 0., 0.));

    // The winner goes after its loser; other regions keep their order.
    CHECK(*precedence_with_kept_overlaps({0, 1, 2}, {{0, 1}}) == std::vector<size_t>{0, 1, 2});
    CHECK(*precedence_with_kept_overlaps({1, 0, 2}, {{0, 1}}) == std::vector<size_t>{0, 1, 2});
    CHECK(*precedence_with_kept_overlaps({2, 1, 0}, {{0, 2}, {1, 2}}) == std::vector<size_t>{1, 0, 2});
    CHECK_FALSE(precedence_with_kept_overlaps({0, 1}, {{0, 1}, {1, 0}}).has_value());

    // A kept pair is not reported, and its shared area goes to the later region.
    const std::vector<ExPolygons> sunk{
        rectangle(0., 0., 20., 20.),
        rectangle(4., 4., 12., 12.)
    };
    CHECK(resolve_regional_ownership(sunk, {0, 1}, 0.01).b01_overlap);
    const RegionalOwnershipResolution kept = resolve_regional_ownership(sunk, {0, 1}, 0.01, 0., {{0, 1}});
    CHECK_FALSE(kept.b01_overlap);
    CHECK_THAT(area_mm2(kept.resolved[1]), WithinAbs(64., 1e-3));
    CHECK_THAT(area_mm2(kept.resolved[0]), WithinAbs(336., 1e-3));
}

TEST_CASE("Native regional planner assigns cells to two disjoint coarse regions",
          "[TestRebuild][RegionalMath]")
{
    NativeRegionalPlanningInput input;
    input.lattice_planes = planes(0.1, 1.0, 0.1);
    const std::vector<coordf_t> fine_nominal = input.lattice_planes;
    const std::vector<coordf_t> coarse_nominal = planes(0.1, 0.9, 0.2);
    input.nominal_planes = {fine_nominal, coarse_nominal, coarse_nominal};
    input.precedence     = {0, 1, 2};
    input.phase_rule     = RegionalGridPhaseRule::KeepNominal;
    input.lattice_ownership.assign(3, std::vector<ExPolygons>(input.lattice_planes.size()));
    for (size_t row = 0; row < input.lattice_planes.size(); ++row) {
        input.lattice_ownership[0][row] = rectangle(0., 0., 2., 2.);
        input.lattice_ownership[1][row] = rectangle(3., 0., 5., 2.);
        input.lattice_ownership[2][row] = rectangle(6., 0., 8., 2.);
    }
    input.sample_native_ownership = [](size_t region, coordf_t) {
        static const ExPolygons fine = rectangle(0., 0., 2., 2.);
        static const ExPolygons coarse_a = rectangle(3., 0., 5., 2.);
        static const ExPolygons coarse_b = rectangle(6., 0., 8., 2.);
        return region == 0 ? fine : region == 1 ? coarse_a : coarse_b;
    };

    const NativeRegionalGridState state = plan_native_regional_grids(input);

    REQUIRE(state.cells.size() == 3);
    CHECK(state.cells[0].size() == 10);
    CHECK(state.cells[1].size() == 5);
    CHECK(state.cells[2].size() == 5);
    for (const NativeRegionCellState &cell : state.cells[1])
        CHECK_FALSE(cell.footprint.empty());
    for (const NativeRegionCellState &cell : state.cells[2])
        CHECK_FALSE(cell.footprint.empty());
}

namespace {

bool fine_skin_at(const RegionalGridPlan &plan, coordf_t z)
{
    for (const RegionCell &cell : plan.cells)
        if (z > cell.bottom_z + 1e-9 && z < cell.top_z - 1e-9)
            return cell.fine_skin;
    return false;
}

NativeRegionalPlanningInput single_region_steps()
{
    NativeRegionalPlanningInput input;
    input.lattice_planes = planes(.1, 1.3, .1);
    input.nominal_planes = {{.1, .3, .5, .7, .9, 1.1, 1.3}};
    input.precedence = {0};
    input.phase_rule = RegionalGridPhaseRule::KeepNominal;
    input.lattice_ownership.resize(1, std::vector<ExPolygons>(13));
    for (size_t row = 0; row < 13; ++row) {
        const double edge = row < 5 ? 10. : row < 9 ? 9.95 : 7.;
        input.lattice_ownership[0][row] = rectangle(0., 0., edge, 10.);
    }
    const auto ownership = input.lattice_ownership;
    input.sample_native_ownership = [ownership](size_t region, coordf_t z) {
        const size_t row = std::min(size_t(std::max(0., std::floor(z / .1))), size_t(12));
        return ownership[region][row];
    };
    input.fine_skin_enabled = {true};
    input.fine_skin_layers = {1};
    return input;
}

} // namespace

TEST_CASE("A thin contour step is filtered while a broad exposed step keeps its fine skin",
          "[TestRebuild][RegionalMath]")
{
    NativeRegionalPlanningInput unfiltered = single_region_steps();
    const NativeRegionalGridState without_filter = plan_native_regional_grids(unfiltered);
    REQUIRE(without_filter.region_plans.size() == 1);
    CHECK(fine_skin_at(without_filter.region_plans[0], .45));

    NativeRegionalPlanningInput filtered = single_region_steps();
    filtered.fine_skin_coarse_line_width = {.4};
    const NativeRegionalGridState with_filter = plan_native_regional_grids(filtered);
    REQUIRE(with_filter.region_plans.size() == 1);
    CHECK_FALSE(fine_skin_at(with_filter.region_plans[0], .45));
    CHECK(fine_skin_at(with_filter.region_plans[0], .85));
    const auto midplanes = native_regional_cell_midplanes(filtered);
    for (const RegionCell &cell : with_filter.region_plans[0].cells)
        CHECK(std::any_of(midplanes.begin(), midplanes.end(), [&cell](coordf_t z) {
            return std::abs(z - cell.slice_z) < 1e-9;
        }));
}

TEST_CASE("A full-width handoff to a neighboring region does not create an exposed fine skin",
          "[TestRebuild][RegionalMath]")
{
    NativeRegionalPlanningInput input;
    input.lattice_planes = planes(.1, 1.1, .1);
    input.nominal_planes = {{.1, .3, .5, .7, .9, 1.1}, {.1, .3, .5, .7, .9, 1.1}};
    input.precedence = {0, 1};
    input.phase_rule = RegionalGridPhaseRule::KeepNominal;
    input.lattice_ownership.resize(2, std::vector<ExPolygons>(11));
    const ExPolygons whole = rectangle(0., 0., 12., 8.);
    const ExPolygons transferred = rectangle(9., 0., 12., 8.);
    const ExPolygons remainder = diff_ex(whole, transferred);
    for (size_t row = 0; row < 11; ++row) {
        input.lattice_ownership[0][row] = row < 5 ? whole : remainder;
        input.lattice_ownership[1][row] = row < 5 ? ExPolygons{} : transferred;
    }
    const auto ownership = input.lattice_ownership;
    input.sample_native_ownership = [ownership](size_t region, coordf_t z) {
        const size_t row = std::min(size_t(std::max(0., std::floor(z / .1))), size_t(10));
        return ownership[region][row];
    };
    input.fine_skin_enabled = {true, false};
    input.fine_skin_layers = {2, 0};

    const NativeRegionalGridState state = plan_native_regional_grids(input);
    REQUIRE(state.region_plans.size() == 2);
    CHECK_FALSE(fine_skin_at(state.region_plans[0], .35));
    CHECK_FALSE(fine_skin_at(state.region_plans[0], .6));
    CHECK(fine_skin_at(state.region_plans[0], .95));
}
