#include <catch2/catch_all.hpp>

#include "fff_print/rebuild_harness.hpp"
#include "wizard_apply_support.hpp"
#include "libslic3r/MixedNozzleBinding.hpp"
#include "libslic3r/MixedNozzleRanking.hpp"
#include "slic3r/GUI/MixedNozzleRankingController.hpp"

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::GUI;
using namespace Slic3r::GUI::WizardTest;

namespace {

void add_rank_object(WizardProject &project, MixedNozzleSlicingMode mode)
{
    if (mode == MixedNozzleSlicingMode::BodySplit)
        project.add_object("two bodies", {{"fine", Vec3d(45., 45., 12.), Vec3d::Zero(), 1},
                                          {"coarse", Vec3d(45., 45., 12.), Vec3d(50., 0., 0.), 2}},
                           Vec2d(95., 95.));
    else
        project.add_object("large part", {{"part", Vec3d(85., 85., 16.), Vec3d::Zero(), 1}},
                           Vec2d(55., 55.));
}

struct RankedPrint {
    std::shared_ptr<Model> model;
    DynamicPrintConfig config;
    MixedNozzleSliceTime time;
};

RankedPrint ranked_print(MixedNozzleSlicingMode mode, int ratio, double coarse_cap = 0.)
{
    WizardProject project(h2d_02_06_spec());
    add_rank_object(project, mode);
    SetupChoice choice;
    choice.mode = mode;
    choice.fine_height = 0.10;
    choice.ratio = ratio;
    if (mode == MixedNozzleSlicingMode::BodySplit)
        choice.joining = JoiningChoice::Off;
    const WizardApplyResult applied = run_wizard_setup(project, choice);
    REQUIRE(applied.applied);
    RankedPrint result;
    result.model = std::make_shared<Model>(project.model);
    result.config = wizard_plate_effective_config(project.bundle, project.plate);
    result.config.apply(*project.plate.config());
    if (coarse_cap > 0.) {
        auto *caps = result.config.option<ConfigOptionFloats>("filament_max_volumetric_speed", true);
        REQUIRE(caps != nullptr);
        PrintConfig resolver;
        resolver.apply(result.config, true);
        const MixedNozzleToolResolution coarse = resolve_mixed_nozzle_tool(
            resolver, 1, MixedNozzleResolveScope::PhysicalToolOnly);
        REQUIRE(coarse);
        REQUIRE(coarse.tool->variant_column.has_value());
        REQUIRE(*coarse.tool->variant_column >= 0);
        REQUIRE(size_t(*coarse.tool->variant_column) < caps->values.size());
        caps->values[*coarse.tool->variant_column] = coarse_cap;
        mixed_nozzle_ledger_add_key(result.config, 2, {1}, "filament_max_volumetric_speed");
    }
    MixedNozzleRankingCancel cancel;
    result.time = mixed_nozzle_slice_time(*result.model, result.config, true, Vec3d::Zero(), {}, cancel);
    INFO(result.time.diagnostic);
    REQUIRE(result.time.status == MixedNozzleSliceTimeStatus::Estimated);
    REQUIRE(result.time.seconds > 0.);
    return result;
}

WizardCandidate row(int ratio, MixedNozzleSlicingMode mode, const MixedNozzleSliceTime &time)
{
    WizardCandidate candidate;
    candidate.stable_id = "cadence-" + std::to_string(ratio);
    candidate.kind = WizardCandidateKind::MixedCadence;
    candidate.mode = mode;
    candidate.fine_height = 0.10;
    candidate.coarse_height = 0.10 * ratio;
    candidate.ratio = ratio;
    candidate.eligibility = WizardCandidateEligibility::Eligible;
    candidate.estimate = wizard_estimate(time.seconds, time.switches, time.tower_mm3);
    return candidate;
}

} // namespace

TEST_CASE("Real Feature and Body ranking slices order eligible wizard choices by printed time",
          "[TestRebuild][Ranking]")
{
    const MixedNozzleSlicingMode mode = GENERATE(MixedNozzleSlicingMode::FeatureSplit,
                                                 MixedNozzleSlicingMode::BodySplit);
    const RankedPrint short_cadence = ranked_print(mode, 2);
    const RankedPrint long_cadence = ranked_print(mode, 3);
    if (mode == MixedNozzleSlicingMode::BodySplit) {
        CHECK(short_cadence.time.switches > 0);
        CHECK(long_cadence.time.switches > 0);
        CHECK(short_cadence.time.tower_mm3 > 0.);
        CHECK(long_cadence.time.tower_mm3 > 0.);

        // The ranking worker applies this same model/config twice before exporting. Compare its
        // estimate with the independent exported print, including the priced tower switches.
        Print direct;
        direct.set_status_silent();
        direct.is_BBL_printer() = true;
        direct.set_plate_origin(Vec3d::Zero());
        direct.apply(*long_cadence.model, long_cadence.config);
        direct.apply(*long_cadence.model, long_cadence.config);
        const CadenceTest::Facts exported = CadenceTest::slice(direct);
        INFO(exported.refusal.string);
        REQUIRE(exported.refusal.string.empty());
        REQUIRE_FALSE(exported.gcode.empty());
        CAPTURE(long_cadence.time.seconds, exported.seconds);
        CHECK(long_cadence.time.seconds == Catch::Approx(exported.seconds).epsilon(0.001));
    }

    FullPrintConfig effective;
    effective.apply(short_cadence.config, true);
    const std::vector<WizardCandidate> candidates = {
        row(2, mode, short_cadence.time), row(3, mode, long_cadence.time)};
    const WizardCadencePage page = wizard_cadence_page(candidates, 0.10, effective);
    REQUIRE(page.rows.size() == 2);
    REQUIRE(page.ranked);
    CHECK(page.rows.front().ratio == (short_cadence.time.seconds <= long_cadence.time.seconds ? 2 : 3));
    CHECK(page.rows.front().estimate->seconds <= page.rows.back().estimate->seconds);
    const auto selection = wizard_ranked_selection(page, std::nullopt);
    REQUIRE(selection);
    if (!page.no_clear_fastest)
        CHECK(*selection == 0); // the public selection is a ranked-page row index
    else {
        CHECK(page.fastest_group == 2);
        CHECK(wizard_seconds_tie(page.rows.front().estimate->seconds,
                                 page.rows.back().estimate->seconds));
    }
}

TEST_CASE("A binding coarse material speed cap changes the full-slice ranking estimate",
          "[TestRebuild][Ranking]")
{
    const RankedPrint normal = ranked_print(MixedNozzleSlicingMode::BodySplit, 3, 30.);
    const RankedPrint capped = ranked_print(MixedNozzleSlicingMode::BodySplit, 3, 0.8);
    REQUIRE(normal.time.switches > 0);
    REQUIRE(normal.time.tower_mm3 > 0.);
    CHECK(capped.time.seconds > normal.time.seconds);
    CHECK(capped.time.switches == normal.time.switches);
    CHECK(capped.time.tower_mm3 > 0.);
    CHECK(mixed_nozzle_ranking_key(0, *capped.model, capped.config) !=
          mixed_nozzle_ranking_key(0, *normal.model, normal.config));
}

TEST_CASE("A changed configuration and Turn off clear an old visible ranking decision",
          "[TestRebuild][Ranking]")
{
    const RankedPrint first = ranked_print(MixedNozzleSlicingMode::BodySplit, 2);
    DynamicPrintConfig changed_config = first.config;
    changed_config.set_key_value("sparse_infill_density", new ConfigOptionPercent(30));
    auto *off_mode = changed_config.option<ConfigOptionEnum<MixedNozzleSlicingMode>>(
        "mixed_nozzle_slicing_mode", true);
    REQUIRE(off_mode != nullptr);

    double clock = 0.;
    DynamicPrintConfig live_config = first.config;
    struct Job { std::uint64_t id; std::string row; MixedNozzleRankingSlice slice; };
    std::vector<Job> launched;
    WizardRankingHooks hooks;
    hooks.compose = [&](const std::string &) {
        MixedNozzleRankingSlice slice;
        slice.model = first.model;
        slice.config = live_config;
        slice.key = mixed_nozzle_ranking_key(0, *slice.model, slice.config);
        return slice;
    };
    hooks.run = [&](std::uint64_t id, const std::string &row, MixedNozzleRankingSlice slice,
                    std::shared_ptr<MixedNozzleRankingCancel>) {
        launched.push_back({id, row, std::move(slice)});
    };
    hooks.now = [&] { return clock; };
    hooks.real_slice_running = [] { return false; };
    MixedNozzleRankingController controller(std::move(hooks), std::make_shared<WizardEstimateCache>());
    controller.request({"current cadence"});
    clock += kWizardRankingDebounceSeconds + 0.01;
    controller.tick();
    REQUIRE(launched.size() == 1);
    controller.deliver(launched.back().id, launched.back().row,
                       wizard_estimate(first.time.seconds, first.time.switches, first.time.tower_mm3));
    REQUIRE(controller.estimate("current cadence"));
    CHECK(controller.estimate("current cadence")->status == WizardEstimateStatus::Estimated);

    live_config = changed_config;
    controller.request({"current cadence"});
    REQUIRE(controller.estimate("current cadence"));
    CHECK(controller.estimate("current cadence")->status == WizardEstimateStatus::Pending);
    clock += kWizardRankingDebounceSeconds + 0.01;
    controller.tick();
    REQUIRE(launched.size() == 2); // changed configuration cannot reuse the first decision
    CHECK(launched.back().slice.key != launched.front().slice.key);

    // Turn off removes the pending cadence; a late result from its old job cannot restore it.
    controller.cancel();
    controller.deliver(launched.back().id, launched.back().row,
                       wizard_estimate(first.time.seconds, first.time.switches, first.time.tower_mm3));
    REQUIRE(controller.estimate("current cadence"));
    CHECK(controller.estimate("current cadence")->status == WizardEstimateStatus::Unavailable);
    off_mode->value = MixedNozzleSlicingMode::Off;
    live_config = changed_config;
    controller.request({});
    CHECK_FALSE(controller.estimate("current cadence"));
    CHECK(mixed_nozzle_ranking_key(0, *first.model, live_config) != launched.front().slice.key);
}
