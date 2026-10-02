#include <catch2/catch_all.hpp>

#include "slic3r/GUI/MixedNozzleWizardModel.hpp"

using namespace Slic3r;
using namespace Slic3r::GUI;

namespace {

FullPrintConfig two_tool_config()
{
    FullPrintConfig config;
    config.filament_map_mode.value = fmmManual;
    config.filament_map.values = {1, 2};
    config.nozzle_diameter.values = {.2, .6};
    config.min_layer_height.values = {.04, .08};
    config.max_layer_height.values = {.15, .45};
    config.layer_height.value = .1;
    config.mixed_nozzle_coarse_layer_height.value = .3;
    config.mixed_nozzle_allowed_cadence_ratios.values = {2, 3, 4};
    return config;
}

WizardCandidate timed_row(int ratio, double seconds, unsigned switches)
{
    WizardCandidate row;
    row.stable_id = "ratio-" + std::to_string(ratio);
    row.kind = WizardCandidateKind::MixedCadence;
    row.mode = MixedNozzleSlicingMode::FeatureSplit;
    row.fine_height = .1;
    row.coarse_height = .1 * ratio;
    row.ratio = ratio;
    row.eligibility = WizardCandidateEligibility::Eligible;
    row.estimate = wizard_estimate(seconds, switches, 10.);
    return row;
}

} // namespace

TEST_CASE("Equal catalogue seeds keep the same identity regardless of input order",
          "[TestRebuild][WizardMath]")
{
    const WizardCatalogueRow later{"z-row", std::nullopt, .1, .4, 4, "Custom", false, {}};
    const WizardCatalogueRow earlier{"a-row", std::nullopt, .1, .4, 4, "Custom", false, {}};
    const auto chosen = [](const std::vector<WizardCatalogueRow> &rows) {
        WizardDraft draft;
        draft.mode = MixedNozzleSlicingMode::FeatureSplit;
        draft.fine_logical_filament = 0;
        draft.coarse_logical_filament = 1;
        const auto candidates = build_wizard_candidates(draft, two_tool_config(), rows);
        for (const WizardCandidate &candidate : candidates)
            if (candidate.fine_height == .1 && candidate.coarse_height == std::optional<double>{.4})
                return candidate.stable_id;
        return std::string{};
    };
    CHECK(chosen({later, earlier}) == "a-row");
    CHECK(chosen({earlier, later}) == "a-row");
}

TEST_CASE("A near-time tie selects fewer nozzle changes unless the user chose a row",
          "[TestRebuild][WizardMath]")
{
    const FullPrintConfig config = two_tool_config();
    std::vector<WizardCandidate> rows{timed_row(2, 1000., 8), timed_row(3, 1015., 3)};
    const WizardCadencePage tied = wizard_cadence_page(rows, .1, config);
    REQUIRE(tied.rows.size() == 2);
    REQUIRE(tied.ranked);
    REQUIRE(tied.fastest_group == 2);
    CHECK(wizard_ranked_selection(tied, std::nullopt) == std::optional<size_t>{1});
    CHECK(wizard_ranked_selection(tied, std::size_t(0), true) == std::optional<size_t>{0});

    rows[1].estimate = wizard_estimate(1040., 3, 10.);
    const WizardCadencePage clear = wizard_cadence_page(rows, .1, config);
    REQUIRE(clear.fastest_group == 1);
    CHECK(wizard_ranked_selection(clear, std::nullopt) == std::optional<size_t>{0});
}

TEST_CASE("Detail and speed holds Next while the default row has no time, not a row picked by hand",
          "[TestRebuild][WizardMath]")
{
    const FullPrintConfig config = two_tool_config();
    // Two rows timed, the other still waiting its turn.
    std::vector<WizardCandidate> rows{timed_row(2, 1000., 0), timed_row(3, 900., 20), timed_row(4, 800., 15)};
    rows[0].estimate = WizardEstimate{};
    const WizardCadencePage page = wizard_cadence_page(rows, .1, config);
    REQUIRE(page.rows.size() == 3);
    std::optional<std::size_t> waiting;
    std::optional<std::size_t> timed;
    for (std::size_t i = 0; i < page.rows.size(); ++i)
        (page.rows[i].ratio == 2 ? waiting : timed) = i;
    REQUIRE(waiting);
    // Detail and speed is page 2.
    REQUIRE(wizard_page_can_block_next(2));

    CHECK_FALSE(wizard_speed_block_reason(page, waiting, false, true).empty());
    CHECK(wizard_speed_block_reason(page, waiting, true, true).empty());
    CHECK(wizard_speed_block_reason(page, timed, false, true).empty());
    // Without ranking the rows never get a time, so there is nothing to wait for.
    CHECK(wizard_speed_block_reason(page, waiting, false, false).empty());
    // Before the first estimate arrives no row has one.
    WizardCadencePage unknown = page;
    unknown.rows[*waiting].estimate.reset();
    CHECK_FALSE(wizard_speed_block_reason(unknown, waiting, false, true).empty());
    // A row whose slice failed has its answer, and is not waited on.
    rows[0].estimate = WizardEstimate{};
    rows[0].estimate->status = WizardEstimateStatus::Failed;
    const WizardCadencePage failed = wizard_cadence_page(rows, .1, config);
    for (std::size_t i = 0; i < failed.rows.size(); ++i)
        if (failed.rows[i].ratio == 2)
            CHECK(wizard_speed_block_reason(failed, i, false, true).empty());
}
