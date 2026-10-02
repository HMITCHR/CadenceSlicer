// Guards for reported bugs that the build 29 test rebuild deleted, restored from the build 28 suite as it was
// adapted to build 31. Each case names
// the reported bug it guards (OB-nn in the bug coverage audit) or says it was rated KEEP. Source
// greps, exact copy pins and cases the test_rebuild_* files already cover were left out. Helpers
// that became file-local in f58636ca59 are reached through the public function that calls them.

#include <catch2/catch_all.hpp>

#include "libslic3r/MixedNozzleConfig.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "slic3r/GUI/MixedNozzleNativeEntry.hpp"
#include "slic3r/GUI/MixedNozzleWizardModel.hpp"
#include "slic3r/GUI/ProjectNozzleFlowState.hpp"
#include "slic3r/GUI/SidebarNozzleCards.hpp"
#include "slic3r/GUI/Widgets/MultiNozzleSync.hpp"

#include "wizard_apply_support.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::GUI;
using namespace Slic3r::GUI::WizardTest;

namespace {

bool has_text(const std::string &text, const std::string &part) { return text.find(part) != std::string::npos; }

std::string diameter_text(double diameter)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(1) << diameter;
    return out.str();
}

double machine_limit(const Preset &preset, const char *key)
{
    const auto *option = preset.config.option<ConfigOptionFloats>(key);
    REQUIRE(option != nullptr);
    REQUIRE(option->values.size() >= 2);
    return option->values.front();
}

// The wizard's input for an H2D fine/coarse pair, each physical side from its own machine preset.
FullPrintConfig resolver_for_pair(PresetBundle &bundle, double fine_diameter, double coarse_diameter)
{
    const Preset *fine_machine = bundle.printers.find_preset("Bambu Lab H2D " + diameter_text(fine_diameter) + " nozzle");
    const Preset *coarse_machine = bundle.printers.find_preset("Bambu Lab H2D " + diameter_text(coarse_diameter) + " nozzle");
    REQUIRE(fine_machine != nullptr);
    REQUIRE(coarse_machine != nullptr);

    FullPrintConfig config;
    config.filament_map_mode.value = fmmManual;
    config.filament_map.values = {1, 2};
    config.nozzle_diameter.values = {fine_diameter, coarse_diameter};
    config.min_layer_height.values = {machine_limit(*fine_machine, "min_layer_height"),
                                      machine_limit(*coarse_machine, "min_layer_height")};
    config.max_layer_height.values = {machine_limit(*fine_machine, "max_layer_height"),
                                      machine_limit(*coarse_machine, "max_layer_height")};
    config.layer_height.value = 0.12;
    config.mixed_nozzle_coarse_layer_height.value = 0.36;
    config.extruder_type.values.clear();
    config.nozzle_volume_type.values.clear();
    config.printer_extruder_id.values.clear();
    config.printer_extruder_variant.values.clear();
    const std::array<const Preset *, 2> machines{fine_machine, coarse_machine};
    for (size_t physical = 0; physical < machines.size(); ++physical) {
        const auto *types = machines[physical]->config.option<ConfigOptionEnumsGeneric>("extruder_type");
        const auto *volumes = machines[physical]->config.option<ConfigOptionEnumsGeneric>("default_nozzle_volume_type");
        const auto *variants = machines[physical]->config.option<ConfigOptionStrings>("extruder_variant_list");
        REQUIRE(types != nullptr);
        REQUIRE(volumes != nullptr);
        REQUIRE(variants != nullptr);
        config.extruder_type.values.push_back(types->values.front());
        config.nozzle_volume_type.values.push_back(volumes->values.front());
        std::stringstream list(variants->values.front());
        for (std::string variant; std::getline(list, variant, ',');)
            if (!variant.empty()) {
                config.printer_extruder_id.values.push_back(int(physical + 1));
                config.printer_extruder_variant.values.push_back(variant);
            }
    }
    config.mixed_nozzle_filament_provenance.values = {"bound", "bound"};
    config.mixed_nozzle_filament_explicit_keys.values = {"", ""};
    REQUIRE_FALSE(config.filament_max_volumetric_speed.values.empty());
    config.filament_max_volumetric_speed.values.resize(2, config.filament_max_volumetric_speed.values.front());
    config.mixed_nozzle_allowed_cadence_ratios.values = {2};
    return config;
}

WizardDraft draft_for(MixedNozzleSlicingMode mode, double fine_height, double coarse_height)
{
    WizardDraft draft;
    draft.scope.kind = WizardScopeKind::CurrentPlate;
    draft.scope.current_plate_id = 42;
    draft.scope.process_affected_plate_ids = {42, 43};
    draft.mode = mode;
    draft.fine_logical_filament = 0;
    draft.coarse_logical_filament = 1;
    draft.chosen_fine_height = fine_height;
    draft.chosen_coarse_height = coarse_height;
    draft.chosen_cadence_ratio = int(std::lround(coarse_height / fine_height));
    return draft;
}

// 0.2 / 0.6 pair at 0.12 mm: the N2 (0.24) and N3 (0.36) rows.
struct RankedFinish {
    FullPrintConfig config;
    std::vector<WizardCandidate> candidates;
    std::size_t n2 {0};
    std::size_t n3 {0};
};

RankedFinish ranked_finish()
{
    PresetBundle bundle = installed_bbl_profiles();
    RankedFinish finish;
    finish.config = resolver_for_pair(bundle, 0.2, 0.6);
    finish.candidates = build_wizard_candidates(draft_for(MixedNozzleSlicingMode::FeatureSplit, 0.12, 0.36),
                                                finish.config, {});
    const WizardCadencePage page = wizard_cadence_page(finish.candidates, 0.12, finish.config);
    REQUIRE(page.rows.size() == 2);
    REQUIRE(page.rows[0].ratio == 2);
    REQUIRE(page.rows[1].ratio == 3);
    finish.n2 = page.rows[0].candidate_index;
    finish.n3 = page.rows[1].candidate_index;
    return finish;
}

// The build 16 cube: 0.2/0.8 pair, 0.08 mm fine, rows N2 to N7.
struct RankCube {
    FullPrintConfig config;
    std::vector<WizardCandidate> candidates;
    std::map<int, std::size_t> by_ratio;
};

RankCube rank_cube(const std::map<int, double> &seconds_by_ratio)
{
    PresetBundle bundle = installed_bbl_profiles();
    RankCube cube;
    cube.config = resolver_for_pair(bundle, 0.2, 0.8);
    cube.candidates = build_wizard_candidates(draft_for(MixedNozzleSlicingMode::FeatureSplit, 0.08, 0.16),
                                              cube.config, {});
    for (const WizardCadenceRow &row : wizard_cadence_page(cube.candidates, 0.08, cube.config).rows)
        cube.by_ratio[row.ratio] = row.candidate_index;
    for (const auto &[ratio, seconds] : seconds_by_ratio) {
        REQUIRE(cube.by_ratio.count(ratio) == 1);
        cube.candidates[cube.by_ratio.at(ratio)].estimate = wizard_estimate(seconds, 10, 0.);
    }
    return cube;
}

// 0.56 and 0.48 tie (1.7%); 0.16 is 22 min slower than the 2 h 30 min all-fine baseline and 54 min
// slower than 0.56; 0.24 is within 2% of the baseline.
const std::map<int, double> kRankCubeSeconds{
    {2, 10320.}, {3, 9100.}, {4, 8200.}, {5, 8000.}, {6, 7200.}, {7, 7080.}};

std::optional<std::size_t> row_of(const WizardCadencePage &page, std::size_t candidate)
{
    for (std::size_t index = 0; index < page.rows.size(); ++index)
        if (page.rows[index].candidate_index == candidate)
            return index;
    return std::nullopt;
}

std::string label_of_ratio(const WizardCadencePage &page, int ratio)
{
    for (const WizardCadenceRow &row : page.rows)
        if (row.ratio == ratio)
            return row.label;
    FAIL("no row for ratio " << ratio);
    return {};
}

bool ends_with(const std::string &text, const std::string &tail)
{
    return text.size() >= tail.size() && text.compare(text.size() - tail.size(), tail.size(), tail) == 0;
}

// Two logical materials on a 0.2 Standard / 0.6 High Flow H2D, each with both variant columns.
FullPrintConfig expanded_variant_material_config()
{
    FullPrintConfig config;
    config.filament_map_mode.value = fmmManual;
    config.filament_map.values = {1, 2};
    config.nozzle_diameter.values = {0.2, 0.6};
    config.min_layer_height.values = {0.04, 0.08};
    config.max_layer_height.values = {0.20, 0.60};
    config.mixed_nozzle_allowed_cadence_ratios.values = {2};
    config.extruder_type.values = {int(etDirectDrive), int(etDirectDrive)};
    config.nozzle_volume_type.values = {int(nvtStandard), int(nvtHighFlow)};
    config.printer_extruder_id.values = {1, 2};
    config.printer_extruder_variant.values = {"Direct Drive Standard", "Direct Drive High Flow"};
    config.filament_type.values = {"PLA", "PLA"};
    config.filament_vendor.values = {"Wizard vendor", "Wizard vendor"};
    config.filament_colour.values = {"#F2754E", "#F2754E"};
    config.filament_ids.values = {"shared-material", "shared-material"};
    config.mixed_nozzle_filament_explicit_keys.values = {"", ""};
    config.mixed_nozzle_filament_provenance.values = {"bound", "bound"};
    config.filament_self_index.values = {1, 1, 2, 2};
    config.filament_extruder_variant.values = {"Direct Drive Standard", "Direct Drive High Flow",
                                               "Direct Drive Standard", "Direct Drive High Flow"};
    config.layer_height.value = 0.10;
    config.mixed_nozzle_coarse_layer_height.value = 0.30;
    return config;
}

} // namespace

// ------------------------------------------------------------------------------------------------
// Step 3: ranking, selection and the speed line (RANK-2, notes 6-8)
// ------------------------------------------------------------------------------------------------

TEST_CASE("OB-10 OB-13: a static default gives way to the tied fastest rows; a slower pick is kept and its cost is said",
          "[TestRebuild][OwnerGuard][WizardSpeed]")
{
    const RankCube cube = rank_cube(kRankCubeSeconds);
    const WizardCadencePage page = wizard_cadence_page(cube.candidates, 0.08, cube.config,
                                                       wizard_estimate(9000., 0, 0.));
    REQUIRE(page.rows.size() == 6);
    REQUIRE(page.ranked);
    CHECK(page.no_clear_fastest);
    CHECK(page.fastest_group == 2);
    CHECK(page.rows[0].ratio == 7);
    CHECK(page.rows[1].ratio == 6);
    const std::size_t n2 = cube.by_ratio.at(2);
    const auto summary = [&page](std::optional<std::size_t> selected, bool explicit_pick) {
        return wizard_speed_summary_line(page, 0.08, selected, explicit_pick);
    };

    SECTION("OB-10: the 0.16 mm row was only the static default") {
        const std::optional<std::size_t> selected = wizard_ranked_selection(page, n2, false);
        CHECK(selected == std::optional<std::size_t>{0});
        CHECK_FALSE(has_text(summary(selected, false), "This choice"));
    }
    SECTION("OB-13: the user picked the 0.16 mm row, and Back then Next keeps it") {
        const std::optional<std::size_t> selected = wizard_ranked_selection(page, n2, true);
        REQUIRE(selected);
        CHECK(page.rows[*selected].ratio == 2);
        CHECK(has_text(summary(selected, true),
                       "This choice is about 54 min slower than the fastest row and about 22 min slower than "
                       "printing everything on the fine nozzle."));
    }
    SECTION("A pick inside the tied fastest group has nothing to say") {
        const std::optional<std::size_t> selected = wizard_ranked_selection(page, cube.by_ratio.at(6), true);
        CHECK(selected == std::optional<std::size_t>{1});
        CHECK_FALSE(has_text(summary(selected, true), "This choice"));
    }
    SECTION("OB-13: a row slower than printing everything fine says so even when not picked by hand") {
        const std::optional<std::size_t> n2_row = row_of(page, n2);
        REQUIRE(n2_row);
        const std::string line = summary(n2_row, false);
        CHECK(has_text(line, "This choice is about 22 min slower than printing everything on the fine nozzle."));
        CHECK_FALSE(has_text(line, "fastest row"));
        // Within 2% of one nozzle only is not called slower: 0.24 is 1 min slower.
        CHECK_FALSE(has_text(summary(row_of(page, cube.by_ratio.at(3)), false), "This choice"));
    }
}

TEST_CASE("OB-11 OB-13 OB-23: tied rows each show their time; a hand pick outside a 2 percent tie says what it costs",
          "[TestRebuild][OwnerGuard][WizardSpeed]")
{
    RankedFinish finish = ranked_finish();
    // 14 h against 14 h 23 min (2.7% apart): a clear fastest.
    finish.candidates[finish.n3].estimate = wizard_estimate(50400., 300, 0.);
    finish.candidates[finish.n2].estimate = wizard_estimate(51780., 100, 0.);
    const WizardCadencePage apart = wizard_cadence_page(finish.candidates, 0.12, finish.config,
                                                        wizard_estimate(60000., 0, 0.));
    REQUIRE(apart.ranked);
    CHECK(apart.fastest_group == 1);
    CHECK(wizard_speed_row_tag(apart, 0) == "Fastest");
    CHECK(wizard_speed_row_tag(apart, 1).empty());
    CHECK(has_text(wizard_speed_summary_line(apart, 0.12, std::size_t(1), true),
                   "This choice is about 23 min slower than the fastest row."));
    CHECK_FALSE(has_text(wizard_speed_summary_line(apart, 0.12, std::size_t(1), false), "This choice"));

    // OB-11: 1.9% apart is a tie. Each row reads "About as fast" and carries its own time.
    finish.candidates[finish.n2].estimate = wizard_estimate(51350., 100, 0.);
    const WizardCadencePage close = wizard_cadence_page(finish.candidates, 0.12, finish.config,
                                                        wizard_estimate(60000., 0, 0.));
    CHECK(close.fastest_group == 2);
    CHECK(close.no_clear_fastest);
    for (std::size_t index = 0; index < 2; ++index) {
        CAPTURE(index);
        CHECK(wizard_speed_row_tag(close, index) == "About as fast");
        CHECK(has_text(wizard_speed_row_label(close.rows[index], MixedNozzleSlicingMode::FeatureSplit), "about "));
    }
    CHECK(wizard_speed_row_label(close.rows[0], MixedNozzleSlicingMode::FeatureSplit) !=
          wizard_speed_row_label(close.rows[1], MixedNozzleSlicingMode::FeatureSplit));

    // Against one nozzle only: within 2% is not slower, more than 2% is.
    finish.candidates[finish.n3].estimate = wizard_estimate(50800., 300, 0.);
    finish.candidates[finish.n2].estimate = wizard_estimate(52000., 100, 0.);
    const WizardCadencePage near_page = wizard_cadence_page(finish.candidates, 0.12, finish.config,
                                                            wizard_estimate(50000., 0, 0.));
    CHECK_FALSE(wizard_switching_saves_no_time(near_page));
    CHECK_FALSE(has_text(wizard_speed_summary_line(near_page, 0.12, std::size_t(0), false), "This choice"));
    CHECK(has_text(wizard_speed_summary_line(near_page, 0.12, std::size_t(1), true),
                   "slower than printing everything on the fine nozzle"));

    // Every row more than 2% slower than one nozzle only: the line says so, with how much faster.
    finish.candidates[finish.n2].estimate = wizard_estimate(27000., 402, 0.);
    finish.candidates[finish.n3].estimate = wizard_estimate(26000., 208, 0.);
    const WizardCadencePage slower = wizard_cadence_page(finish.candidates, 0.12, finish.config,
                                                         wizard_estimate(20000., 0, 0.));
    CHECK(wizard_switching_saves_no_time(slower));
    const std::string line = wizard_speed_summary_line(slower, 0.12, std::size_t(0), false);
    CHECK(has_text(line, "switching nozzles does not save time"));
    CHECK(has_text(line, "One nozzle only is about 1 h 40 min faster."));
}

TEST_CASE("OB-20: timed rows go on top fastest first while the rest wait in layer order",
          "[TestRebuild][OwnerGuard][WizardSpeed]")
{
    const RankCube cube = rank_cube({{5, 8000.}, {7, 7080.}});
    const WizardCadencePage page = wizard_cadence_page(cube.candidates, 0.08, cube.config, WizardEstimate{});
    REQUIRE(page.rows.size() == 6);
    CHECK_FALSE(page.ranked);
    std::vector<int> order;
    for (const WizardCadenceRow &row : page.rows)
        order.push_back(row.ratio);
    CHECK(order == std::vector<int>{7, 5, 2, 3, 4, 6});
    // No "Fastest" tag until every row has a time, and the progress line counts the timed rows.
    CHECK(wizard_speed_row_tag(page, 0).empty());
    CHECK(has_text(wizard_speed_progress_line(page, true), "2 of 6 done"));

    // Nothing timed: layer order.
    RankCube failed = rank_cube({});
    for (WizardCandidate &candidate : failed.candidates) {
        candidate.estimate = WizardEstimate{};
        candidate.estimate->status = WizardEstimateStatus::Failed;
    }
    const WizardCadencePage none = wizard_cadence_page(failed.candidates, 0.08, failed.config,
                                                       wizard_estimate(9000., 0, 0.));
    CHECK(none.rows.front().ratio == 2);
}

TEST_CASE("OB-22: the project's own coarse layer reads now and stays selected when times arrive",
          "[TestRebuild][OwnerGuard][WizardSpeed]")
{
    const RankCube cube = rank_cube(kRankCubeSeconds);
    WizardCadencePage page = wizard_cadence_page(cube.candidates, 0.08, cube.config, wizard_estimate(9000., 0, 0.));
    // The project was set up at N5, 0.40 mm.
    const std::optional<std::pair<double, double>> existing = std::make_pair(0.08, 0.40);
    wizard_mark_current(page, 0.08, existing);
    const std::optional<std::size_t> current = wizard_existing_row(page, 0.08, existing);
    REQUIRE(current);
    CHECK(page.rows[*current].current);
    CHECK(page.rows[*current].ratio == 5);
    CHECK(has_text(wizard_speed_row_label(page.rows[*current], MixedNozzleSlicingMode::FeatureSplit), "(now)"));
    // Held as an explicit pick: the ranked page keeps it, and says what it costs.
    const std::size_t n5 = cube.by_ratio.at(5);
    CHECK(wizard_ranked_selection(page, n5, true) == current);
    CHECK(has_text(wizard_speed_summary_line(page, 0.08, current, true),
                   "This choice is about 15 min slower than the fastest row."));
    // A fresh setup has no own row: the fastest is selected, and nothing reads "(now)".
    WizardCadencePage fresh = wizard_cadence_page(cube.candidates, 0.08, cube.config, wizard_estimate(9000., 0, 0.));
    wizard_mark_current(fresh, 0.08, std::nullopt);
    for (const WizardCadenceRow &row : fresh.rows) {
        CHECK_FALSE(row.current);
        CHECK_FALSE(has_text(wizard_speed_row_label(row, MixedNozzleSlicingMode::FeatureSplit), "(now)"));
    }
    CHECK(wizard_ranked_selection(fresh, n5, false) == std::optional<std::size_t>{0});
}

TEST_CASE("OB-24: the one nozzle only line is always there while ranking runs and never ends in four dots",
          "[TestRebuild][OwnerGuard][WizardSpeed]")
{
    RankedFinish finish = ranked_finish();
    finish.candidates[finish.n2].estimate = wizard_estimate(27000., 402, 0.);
    finish.candidates[finish.n3].estimate = wizard_estimate(23160., 208, 0.);
    const std::string lead = "One nozzle only, 0.12 mm everywhere: ";

    // Every row timed, one nozzle only still waiting.
    const WizardCadencePage waiting = wizard_cadence_page(finish.candidates, 0.12, finish.config, WizardEstimate{});
    const std::string waiting_line = wizard_speed_summary_line(waiting, 0.12, std::size_t(0), false);
    CHECK(waiting_line.rfind(lead, 0) == 0);
    CHECK(has_text(waiting_line, "waiting"));
    // One nozzle only could not be timed: the reason.
    WizardEstimate refused;
    refused.status = WizardEstimateStatus::Unavailable;
    refused.note = "choose the fine material first";
    const WizardCadencePage none = wizard_cadence_page(finish.candidates, 0.12, finish.config, refused);
    const std::string refused_line = wizard_speed_summary_line(none, 0.12, std::size_t(0), false);
    CHECK(refused_line.rfind(lead, 0) == 0);
    CHECK(has_text(refused_line, "choose the fine material first"));
    // Timed: its time.
    const WizardCadencePage timed = wizard_cadence_page(finish.candidates, 0.12, finish.config,
                                                        wizard_estimate(30660., 0, 0.));
    CHECK(wizard_speed_summary_line(timed, 0.12, std::size_t(0), false).rfind(lead + "about 8 h 31 min.", 0) == 0);

    // Slicing now: one sentence end, not four dots.
    RankCube cube = rank_cube({{7, 7080.}});
    WizardCadencePage slicing = wizard_cadence_page(cube.candidates, 0.08, cube.config, WizardEstimate{});
    wizard_mark_slicing(slicing, cube.candidates, std::string("single-nozzle-baseline"), true);
    const std::string slicing_line = wizard_speed_summary_line(slicing, 0.08, std::nullopt, false);
    CHECK(ends_with(slicing_line, "slicing now..."));
    CHECK_FALSE(has_text(slicing_line, "...."));

    // No ranking at all (no estimates, no baseline): nothing under the list.
    for (WizardCandidate &candidate : finish.candidates)
        candidate.estimate.reset();
    CHECK(wizard_speed_summary_line(wizard_cadence_page(finish.candidates, 0.12, finish.config), 0.12,
                                    std::nullopt, false).empty());
}

TEST_CASE("KEEP: each cadence row states its estimate, switch count or why it has none",
          "[TestRebuild][OwnerGuard][WizardSpeed]")
{
    // wizard_estimate_text is file-local; every cadence row label and the baseline carry it.
    RankedFinish finish = ranked_finish();
    finish.candidates[finish.n3].estimate = wizard_estimate(23160., 208, 0.);
    finish.candidates[finish.n2].estimate = wizard_estimate(3600., 1, 0.);
    WizardCadencePage page = wizard_cadence_page(finish.candidates, 0.12, finish.config, wizard_estimate(2520., 0, 0.));
    CHECK(ends_with(label_of_ratio(page, 3), ": about 6 h 26 min, 208 switches"));
    CHECK(ends_with(label_of_ratio(page, 2), ": about 1 h, 1 switch"));
    finish.candidates[finish.n2].estimate = wizard_estimate(2520., 0, 0.);
    page = wizard_cadence_page(finish.candidates, 0.12, finish.config, wizard_estimate(2520., 0, 0.));
    CHECK(ends_with(label_of_ratio(page, 2), ": about 42 min, no switches"));

    // Note 8: no row is skipped as too slow; a row without a time is still estimating or failed.
    finish.candidates[finish.n2].estimate = WizardEstimate{};
    page = wizard_cadence_page(finish.candidates, 0.12, finish.config, WizardEstimate{});
    CHECK(ends_with(label_of_ratio(page, 2), ": estimating"));
    CHECK(ends_with(page.baseline, ": estimating"));
    finish.candidates[finish.n2].estimate->status = WizardEstimateStatus::Failed;
    page = wizard_cadence_page(finish.candidates, 0.12, finish.config, wizard_estimate(30660., 0, 0.));
    CHECK(ends_with(label_of_ratio(page, 2), ": could not be sliced"));
    CHECK(ends_with(wizard_speed_row_label(page.rows[1], MixedNozzleSlicingMode::FeatureSplit), "could not be sliced"));
    finish.candidates[finish.n2].estimate->status = WizardEstimateStatus::Unavailable;
    finish.candidates[finish.n2].estimate->note = "the slice failed";
    page = wizard_cadence_page(finish.candidates, 0.12, finish.config, wizard_estimate(30660., 0, 0.));
    CHECK(ends_with(label_of_ratio(page, 2), ": no estimate: the slice failed"));
}

TEST_CASE("OB-14: the excluded coarse layer line names the layer and the nozzle's limit",
          "[TestRebuild][OwnerGuard][WizardSpeed]")
{
    const RankCube cube = rank_cube({});
    const WizardCadencePage page = wizard_cadence_page(cube.candidates, 0.08, cube.config);
    for (const WizardCadenceRow &row : page.rows)
        CHECK(row.ratio <= 7);
    CAPTURE(page.exclusions);
    CHECK(has_text(page.exclusions, "0.64 mm"));
    CHECK(has_text(page.exclusions, "0.8 mm nozzle"));
    CHECK(has_text(page.exclusions, "0.56 mm"));
}

// ------------------------------------------------------------------------------------------------
// Step 1 and 2: mode, parts and materials (notes 9-13)
// ------------------------------------------------------------------------------------------------

TEST_CASE("OB-25: a multi-part object on two filaments opens step 1 on Body Split even in a Feature Split project",
          "[TestRebuild][OwnerGuard][WizardParts]")
{
    CHECK(wizard_default_mode(MixedNozzleSlicingMode::FeatureSplit, true) == MixedNozzleSlicingMode::BodySplit);
    CHECK(wizard_default_mode(MixedNozzleSlicingMode::BodySplit, true) == MixedNozzleSlicingMode::BodySplit);
    CHECK(wizard_default_mode(MixedNozzleSlicingMode::Off, false) == MixedNozzleSlicingMode::FeatureSplit);
}

TEST_CASE("OB-27: Body Split step 2 starts on the parts' own materials and changes none of them",
          "[TestRebuild][OwnerGuard][WizardParts]")
{
    const std::vector<std::string> labels{"1: eSUN PLA+", "2: PLA Basic", "3: eSUN PLA+", "4: PETG Basic",
                                          "5: ASA", "6: PETG HF", "7: PLA Basic", "8: PETG Translucent"};
    // A two-part cover: Body11 (slot 6, coarse) listed first, then Body12 (slot 4, fine).
    WizardBodyRoleRow body11;
    body11.object_id = 3;
    body11.volume_id = 11;
    body11.current_slot = 5;
    body11.current_physical = 1;
    body11.volume = 14000.;
    WizardBodyRoleRow body12 = body11;
    body12.volume_id = 12;
    body12.current_slot = 3;
    body12.volume = 900.;

    for (const std::size_t slot4_nozzle : {std::size_t(0), std::size_t(1)}) {
        CAPTURE(slot4_nozzle);
        body12.current_physical = slot4_nozzle;
        const std::vector<WizardBodyRoleRow> rows{body11, body12};
        const std::vector<WizardBodyRole> roles = wizard_default_body_roles(rows, 0);
        REQUIRE(roles == std::vector<WizardBodyRole>{WizardBodyRole::Coarse, WizardBodyRole::Fine});
        const WizardPickerDefaults picker =
            wizard_body_picker_defaults(rows, roles, {}, {}, {}, {}, std::nullopt, std::nullopt);
        CHECK(picker.fine == std::optional<std::size_t>{3});
        CHECK(picker.coarse == std::optional<std::size_t>{5});

        // The map setup writes puts slot 4 left and slot 6 right; every other slot keeps the plate's.
        const std::vector<int> plate_map{1, 1, 1, 1, 1, 2, 1, 1};
        std::vector<int> saved = plate_map;
        if (slot4_nozzle == 1)
            saved[3] = 2;
        CHECK(wizard_derived_filament_map({0.2, 0.4}, saved, 8, picker.fine, picker.coarse) == plate_map);

        std::vector<WizardBodyRoleRow> mapped = rows;
        mapped[1].current_physical = 0;
        const std::vector<WizardBodyAssignment> body = wizard_body_roles_to_slots(mapped, roles, 3, 5, 0, 1, {});
        REQUIRE(body.size() == 2);
        CHECK(body[0].logical_filament == 5);
        CHECK(body[0].coarse);
        CHECK(body[1].logical_filament == 3);
        CHECK_FALSE(body[1].coarse);
        CHECK(wizard_body_material_changes({"Body11", "Body12"}, rows, body, labels).empty());
    }

    // The old default (slot 1 for the fine layers) is listed on step 4.
    const std::vector<WizardBodyRoleRow> rows{body11, body12};
    const std::vector<WizardBodyAssignment> changed{{3, 11, 5, true}, {3, 12, 0, false}};
    const std::vector<std::string> lines = wizard_body_material_changes({"Body11", "Body12"}, rows, changed, labels);
    REQUIRE(lines.size() == 1);
    CHECK(has_text(lines.front(), "Body12"));
    CHECK(has_text(lines.front(), "4: PETG Basic"));
    CHECK(has_text(lines.front(), "1: eSUN PLA+"));

    // Note 12b: the slots the parts use come before the object's own slot.
    CHECK(wizard_object_material_slots({{1, {6, 4}}}) == std::vector<std::size_t>{5, 3, 0});
    CHECK(wizard_object_material_slots({{2, {0, 0, 5}}}) == std::vector<std::size_t>{1, 4});
    CHECK(wizard_object_material_slots({{3, {}}}) == std::vector<std::size_t>{2});
}

TEST_CASE("OB-27: Body Split parts sharing one slot never take a stale coarse material",
          "[TestRebuild][OwnerGuard][WizardParts]")
{
    const std::vector<std::string> labels{"1: eSUN PLA+", "2: PLA Basic", "3: eSUN PLA+", "4: PETG Basic",
                                          "5: ASA", "6: PETG HF", "7: PLA Basic", "8: PETG Translucent"};
    const std::vector<std::string> types{"PLA", "PLA", "PLA", "PETG", "ASA", "PETG", "PLA", "PETG"};
    const std::vector<double> nozzles{0.2, 0.4};
    WizardBodyRoleRow body11;
    body11.object_id = 3;
    body11.volume_id = 11;
    body11.current_slot = 3;
    body11.current_physical = 0;
    body11.volume = 14000.;
    WizardBodyRoleRow body12 = body11;
    body12.volume_id = 12;
    body12.volume = 900.;
    const std::vector<WizardBodyRoleRow> rows{body11, body12};
    const std::vector<WizardBodyRole> roles{WizardBodyRole::Coarse, WizardBodyRole::Fine};
    // The pickers as an earlier opening left them: fine on slot 4, coarse on PLA slot 1.
    const std::optional<std::size_t> stale_fine{3};
    const std::optional<std::size_t> stale_coarse{0};

    SECTION("slot 4 on the fine nozzle: the coarse parts take the closest PETG slot on the coarse nozzle") {
        const std::vector<int> map{1, 1, 1, 1, 1, 2, 1, 1};
        const WizardPickerDefaults defaults =
            wizard_body_picker_defaults(rows, roles, nozzles, map, types, labels, stale_fine, stale_coarse);
        CHECK(defaults.fine == std::optional<std::size_t>{3});
        CHECK(defaults.coarse == std::optional<std::size_t>{5});
        CHECK(has_text(defaults.note, "6: PETG HF"));
        REQUIRE(defaults.fine);
        REQUIRE(defaults.coarse);
        const std::vector<WizardBodyAssignment> body =
            wizard_body_roles_to_slots(rows, roles, *defaults.fine, *defaults.coarse, 0, 1, {});
        const std::vector<std::string> lines = wizard_body_material_changes({"Body11", "Body12"}, rows, body, labels);
        REQUIRE(lines.size() == 1);
        CHECK(has_text(lines.front(), "Body11"));
        CHECK(has_text(lines.front(), "6: PETG HF"));
        CHECK_FALSE(has_text(lines.front(), "PLA"));
    }
    SECTION("slot 4 on the coarse nozzle: it stays coarse, and the fine parts take a PETG slot on the fine nozzle") {
        const std::vector<int> map{1, 1, 1, 2, 1, 2, 1, 1};
        std::vector<WizardBodyRoleRow> coarse_rows = rows;
        for (WizardBodyRoleRow &row : coarse_rows)
            row.current_physical = 1;
        const WizardPickerDefaults defaults =
            wizard_body_picker_defaults(coarse_rows, roles, nozzles, map, types, labels, stale_coarse, stale_fine);
        CHECK(defaults.coarse == std::optional<std::size_t>{3});
        CHECK(defaults.fine == std::optional<std::size_t>{7});
        CHECK(has_text(defaults.note, "8: PETG Translucent"));
    }
    SECTION("no other slot of the same material: the other picker is left empty, not on PLA") {
        const std::vector<std::string> one_petg{"PLA", "PLA", "PLA", "PETG", "ASA", "PLA", "PLA", "PLA"};
        const std::vector<int> map{1, 1, 1, 1, 1, 2, 1, 1};
        const WizardPickerDefaults defaults =
            wizard_body_picker_defaults(rows, roles, nozzles, map, one_petg, labels, stale_fine, stale_coarse);
        CHECK(defaults.fine == std::optional<std::size_t>{3});
        CHECK_FALSE(defaults.coarse.has_value());
        CHECK_FALSE(defaults.note.empty());
    }
    SECTION("parts on their own slots keep them, with nothing to say") {
        std::vector<WizardBodyRoleRow> own = rows;
        own[0].current_slot = 5;
        own[0].current_physical = 1;
        const std::vector<int> map{1, 1, 1, 1, 1, 2, 1, 1};
        const WizardPickerDefaults defaults =
            wizard_body_picker_defaults(own, roles, nozzles, map, types, labels, stale_fine, stale_coarse);
        CHECK(defaults.fine == std::optional<std::size_t>{3});
        CHECK(defaults.coarse == std::optional<std::size_t>{5});
        CHECK(defaults.note.empty());
    }
}

TEST_CASE("OB-27: the materials are matched from what each nozzle holds",
          "[TestRebuild][OwnerGuard][WizardParts]")
{
    const std::vector<double> pair{0.2, 0.6};
    // A kitten model: eight PLA slots, slot 7 on the right nozzle, the model on slot 1.
    const WizardMaterialDefaults kitten =
        wizard_default_materials(pair, {1, 1, 1, 1, 1, 1, 2, 1}, std::vector<std::string>(8, "PLA"), {0});
    CHECK(kitten.fine == std::optional<std::size_t>{0});
    CHECK(kitten.coarse == std::optional<std::size_t>{6});
    CHECK(kitten.note.empty());
    // Same type first on the coarse nozzle.
    const WizardMaterialDefaults typed = wizard_default_materials(pair, {1, 2, 2}, {"PETG", "PLA", "PETG"}, {0});
    CHECK(typed.coarse == std::optional<std::size_t>{2});
    // The objects' slot is the fine material only when it sits on the fine nozzle.
    const WizardMaterialDefaults moved = wizard_default_materials(pair, {2, 1}, {"PLA", "PLA"}, {0});
    CHECK(moved.fine == std::optional<std::size_t>{1});
    CHECK(moved.coarse == std::optional<std::size_t>{0});
    // The smaller nozzle is the fine one whichever side it is on.
    const WizardMaterialDefaults reversed = wizard_default_materials({0.6, 0.2}, {1, 2}, {"PLA", "PLA"}, {});
    CHECK(reversed.fine == std::optional<std::size_t>{1});
    CHECK(reversed.coarse == std::optional<std::size_t>{0});
    // Nothing on the coarse nozzle: another slot goes there, and the page says so.
    const WizardMaterialDefaults fallback = wizard_default_materials(pair, {1, 1}, {"PLA", "PLA"}, {1});
    CHECK(fallback.fine == std::optional<std::size_t>{1});
    CHECK(fallback.coarse == std::optional<std::size_t>{0});
    CHECK_FALSE(fallback.note.empty());
    // One material only: the coarse choice is left to the user.
    const WizardMaterialDefaults single = wizard_default_materials(pair, {1}, {"PLA"}, {0});
    CHECK(single.fine == std::optional<std::size_t>{0});
    CHECK_FALSE(single.coarse.has_value());
}

TEST_CASE("OB-26: Body Split part rows give each part's size",
          "[TestRebuild][OwnerGuard][WizardParts]")
{
    // wizard_part_size_text is file-local; wizard_part_row_text puts it on the step 1 row.
    CHECK(ends_with(wizard_part_row_text("4: PETG Basic", "Case", "Body12", false, 87.2, 54.4, 3.2),
                    "   87 x 54 x 3.2 mm"));
    CHECK(ends_with(wizard_part_row_text("", "Case", "Body12", false, 10., 9.96, 3.), "   10 x 10 x 3 mm"));
    CHECK(ends_with(wizard_part_row_text("", "Case", "Body12", false, 120.5, 0.4, 2.26), "   121 x 0.4 x 2.3 mm"));
    // Several objects name the object too; no size known leaves it out rather than guessing.
    CHECK(has_text(wizard_part_row_text("6: PETG HF", "Case", "Body11", true, 90., 56., 4.), "Case / Body11"));
    CHECK(wizard_part_row_text("", "Case", "Body11", false, 0., 0., 0.) == "Body11");
}

TEST_CASE("OB-29: the one-nozzle Body Split refusal names the parts, slots and nozzle",
          "[TestRebuild][OwnerGuard][WizardParts]")
{
    const std::vector<double> pair{0.2, 0.4};
    const WizardPartOnNozzle body11{"Body11", 5, 1, 0.16};
    const WizardPartOnNozzle body12{"Body12", 3, 1, 0.08};
    const std::string line = wizard_one_nozzle_parts_line({body11, body12}, pair);
    CAPTURE(line);
    for (const char *part : {"Body11 (slot 6)", "Body12 (slot 4)", "right 0.4 mm nozzle", "Put slot 4 on the left nozzle"})
        CHECK(has_text(line, part));
    // No layer height of their own: the last part is still the one to move.
    WizardPartOnNozzle plain11 = body11;
    plain11.layer_height = 0.;
    WizardPartOnNozzle plain12 = body12;
    plain12.layer_height = 0.;
    CHECK(has_text(wizard_one_nozzle_parts_line({plain11, plain12}, pair), "Put slot 4 on the left nozzle"));
    // All on the left on one slot: the last part needs a slot on the right.
    const std::string shared = wizard_one_nozzle_parts_line(
        {{"Lid", 0, 0, 0.}, {"Arm", 0, 0, 0.}, {"Pin", 0, 0, 0.}}, pair);
    CHECK(has_text(shared, "left 0.2 mm nozzle"));
    CHECK(has_text(shared, "Give Pin a slot on the right nozzle"));
    // Nothing to say when the parts are already on both nozzles, or there is one part.
    WizardPartOnNozzle left12 = body12;
    left12.physical = 0;
    CHECK(wizard_one_nozzle_parts_line({body11, left12}, pair).empty());
    CHECK(wizard_one_nozzle_parts_line({body11}, pair).empty());
}

TEST_CASE("OB-30: a coarse material with no profile for its nozzle or flow type blocks step 2",
          "[TestRebuild][OwnerGuard][WizardParts]")
{
    // wizard_material_has_variant is file-local; wizard_material_resolution is what step 2 calls.
    FullPrintConfig config = expanded_variant_material_config();
    CHECK_FALSE(wizard_material_resolution(config, 1, true).needs_input);

    SECTION("Logical filament 2 keeps only its fine-nozzle column") {
        config.filament_self_index.values = {1, 1, 2};
        config.filament_extruder_variant.values = {"Direct Drive Standard", "Direct Drive High Flow",
                                                   "Direct Drive Standard"};
        CHECK_FALSE(wizard_material_resolution(config, 0, true).needs_input);
        const WizardMaterialResolution coarse = wizard_material_resolution(config, 1, true);
        CHECK(coarse.needs_input);
        CHECK(has_text(coarse.text, "0.6 mm"));
        CHECK(wizard_materials_block_reason(config, std::size_t(0), std::size_t(1), false, {}) == coarse.text);
    }
    SECTION("H2C Hybrid rack: the material's own flow column decides") {
        config.nozzle_volume_type.values = {int(nvtStandard), int(nvtHybrid)};
        config.filament_volume_map.values = {int(nvtStandard), int(nvtHighFlow)};
        CHECK_FALSE(wizard_material_resolution(config, 1, true).needs_input);
        // The coarse material has a Standard column only, and the rack runs it on High Flow.
        config.filament_self_index.values = {1, 1, 2};
        config.filament_extruder_variant.values = {"Direct Drive Standard", "Direct Drive High Flow",
                                                   "Direct Drive Standard"};
        CHECK(wizard_material_resolution(config, 1, true).needs_input);
        // On Standard it prints.
        config.filament_volume_map.values = {int(nvtStandard), int(nvtStandard)};
        CHECK_FALSE(wizard_material_resolution(config, 1, true).needs_input);
    }
}

TEST_CASE("KEEP: the Body preset choice reads the nozzle pair from the name or its parent, either order",
          "[TestRebuild][OwnerGuard][BodyPreset]")
{
    // wizard_process_pair is file-local; wizard_body_process_choice is what Apply calls.
    const std::vector<WizardProcessPresetRow> rows{
        {"MN Body 0.4-0.2 Standard 0.10-0.20 @BBL", "", 0.10, 0.20, true},
        {"MN Body 0.2-0.6 Standard 0.12-0.36 @BBL", "", 0.12, 0.36, true},
    };
    const std::string feature = "MN Feature 0.2-0.4 0.08-0.24 Candidate @BBL";
    // A name stating 0.4-0.2 is the 0.2/0.4 pair.
    const auto exact = wizard_body_process_choice(rows, feature, "", {0.2, 0.4}, 0.10, 0.20);
    CHECK(exact.current_is_feature);
    CHECK(exact.preset == "MN Body 0.4-0.2 Standard 0.10-0.20 @BBL");
    CHECK(exact.exact_heights);
    // A user preset takes the pair its parent names, and is kept.
    const auto kept = wizard_body_process_choice(rows, "My body preset", "MN Body 0.4-0.2 Detail 0.08-0.16 @BBL",
                                                 {0.4, 0.2}, 0.10, 0.20);
    CHECK(kept.current_is_body);
    CHECK(kept.preset.empty());
    // A parent for another pair is not kept, and a pair with no Body preset says so.
    CHECK_FALSE(wizard_body_process_choice(rows, "My body preset", "MN Body 0.2-0.8 Detail 0.08-0.16 @BBL",
                                           {0.2, 0.4}, 0.10, 0.20).current_is_body);
    CHECK(wizard_body_process_choice(rows, feature, "", {0.2, 0.8}, 0.10, 0.40).no_body_preset_for_pair);
}

TEST_CASE("OB-39: engine refusals are rewritten in plain words with no codes",
          "[TestRebuild][OwnerGuard][WizardCopy]")
{
    const std::string line = wizard_admission_refusal_line(
        "[SRL-A02] [MNS-MAP-A01] Every synchronized body's filament assignments must resolve to one of the "
        "two installed physical tools, and both tools must be used by synchronized multi-nozzle layering",
        "filament_map", "Case");
    CAPTURE(line);
    CHECK(line.rfind("Case: ", 0) == 0);
    for (const char *word : {"synchronized", "physical tool", "SRL-", "MNS-", "["})
        CHECK_FALSE(has_text(line, word));
    const std::string ironing = wizard_admission_refusal_line(
        "[SRL-A20] Fuzzy, scarf, ironing, and Z-contouring modes are not supported by synchronized "
        "multi-nozzle layering.", "ironing_type", "");
    CHECK(has_text(ironing, "Setting to change: Ironing type."));
    CHECK_FALSE(has_text(ironing, "ironing_type"));
    CHECK_FALSE(has_text(wizard_admission_refusal_line("[SRL-A05] One instance.", "mixed_nozzle_slicing_mode", ""),
                         "Setting to change"));
}

// ------------------------------------------------------------------------------------------------
// Apply on a two-part cover (notes 12 and 14)
// ------------------------------------------------------------------------------------------------

namespace {

constexpr const char *kCoverPrinter = "Bambu Lab H2D 0.2 nozzle";
constexpr const char *kCoverProcess = "MN Body 0.2-0.4 Detail 0.08-0.16 @BBL";

const std::vector<std::string> &cover_filaments()
{
    static const std::vector<std::string> names{
        "eSUN PLA+ @BBL H2D 0.2 nozzle", "Bambu PLA Basic @BBL H2D 0.2 nozzle",
        "eSUN PLA+ @BBL H2D 0.2 nozzle", "Bambu PETG Basic @BBL H2D 0.2 nozzle",
        "Generic ASA @BBL H2D 0.2 nozzle", "Bambu PETG HF @BBL H2D 0.4 nozzle",
        "Bambu PLA Basic @BBL H2D 0.2 nozzle", "Bambu PETG Translucent @BBL H2D 0.2 nozzle"};
    return names;
}

// A two-part cover project on the installed BBL profiles: project map all on tool 1, the
// plate's manual map with slot 6 on tool 2, Body11 on slot 6 at 0.16 and Body12 on slot 4 at 0.08.
struct CoverFixture {
    PresetBundle bundle;
    Model model;
    PartPlate current_plate;
    WizardNativeOwners owners;

    CoverFixture()
        : bundle(installed_bbl_profiles()),
          current_plate(nullptr, Vec3d::Zero(), 350, 320, 325, nullptr, &model),
          owners{&bundle, &model, {&current_plate}, &current_plate}
    {
        current_plate.set_index(0);
        Preset *printer = bundle.printers.find_preset(kCoverPrinter, false, true);
        REQUIRE(printer != nullptr);
        printer->is_visible = true;
        Preset *process = bundle.prints.find_preset(kCoverProcess, false, true);
        REQUIRE(process != nullptr);
        process->is_visible = true;
        for (const std::string &name : cover_filaments()) {
            Preset *preset = bundle.filaments.find_preset(name, false, true);
            INFO(name);
            REQUIRE(preset != nullptr);
            preset->is_visible = true;
        }
        REQUIRE(bundle.printers.select_preset_by_name(kCoverPrinter, true));
        REQUIRE(bundle.prints.select_preset_by_name(kCoverProcess, true));
        REQUIRE(bundle.filaments.select_preset_by_name(cover_filaments().front(), true));
        bundle.filament_presets = cover_filaments();

        auto &project = bundle.project_config;
        project.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.4});
        project.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.08});
        project.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.28});
        project.set_key_value("filament_map", new ConfigOptionInts(std::vector<int>(8, 1)));
        project.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmAutoForFlush));
        project.set_key_value("filament_volume_map", new ConfigOptionInts(std::vector<int>(8, 0)));
        project.set_key_value("mixed_nozzle_slicing_mode",
            new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::BodySplit));
        project.set_key_value("mixed_nozzle_filament_provenance",
            new ConfigOptionStrings(std::vector<std::string>(8, "bound")));
        project.set_key_value("mixed_nozzle_filament_explicit_keys",
            new ConfigOptionStrings(std::vector<std::string>(8, "")));
        bundle.update_compatible(PresetSelectCompatibleType::Never);

        DynamicPrintConfig &plate = *current_plate.config();
        plate.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
        plate.set_key_value("filament_map", new ConfigOptionInts{1, 1, 1, 1, 1, 2, 1, 1});

        ModelObject *object = model.add_object();
        object->name = "Two-part cover";
        object->config.set_key_value("extruder", new ConfigOptionInt(1));
        ModelVolume *body11 = object->add_volume(make_cube(40., 40., 3.), ModelVolumeType::MODEL_PART, false);
        body11->name = "Body11";
        body11->config.set_key_value("extruder", new ConfigOptionInt(6));
        body11->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.16));
        ModelVolume *body12 = object->add_volume(make_cube(20., 10., 3.), ModelVolumeType::MODEL_PART, false);
        body12->name = "Body12";
        body12->set_offset(Vec3d(45., 0., 0.));
        body12->config.set_key_value("extruder", new ConfigOptionInt(4));
        body12->config.set_key_value("regional_layer_height", new ConfigOptionFloat(0.08));
        object->add_instance();
        current_plate.add_instance(0, 0, false);
    }
};

} // namespace

TEST_CASE("OB-27: Body Split setup on a two-part cover keeps every part's material",
          "[TestRebuild][OwnerGuard][WizardApply]")
{
    CoverFixture fixture;
    const std::vector<std::string> presets_before = fixture.bundle.filament_presets;
    const ModelObject *object = fixture.model.objects.front();
    const ModelVolume *body11 = object->volumes[0];
    const ModelVolume *body12 = object->volumes[1];

    WizardDraft draft;
    draft.signature.selected_printer_id = fixture.bundle.printers.get_edited_preset().name;
    draft.signature.selected_process_id = fixture.bundle.prints.get_edited_preset().name;
    draft.signature.plate_ids.push_back(fixture.current_plate.id().id);
    for (const ModelVolume *volume : object->volumes) {
        WizardVolumeSignature signature;
        signature.object_id = object->id().id;
        signature.volume_id = volume->id().id;
        signature.config_revision = static_cast<const ModelConfig &>(volume->config).timestamp();
        draft.signature.volumes.push_back(signature);
    }

    // What the Plater hands the wizard: the plate's composed config.
    FullPrintConfig effective;
    effective.apply(wizard_plate_effective_config(fixture.bundle, fixture.current_plate), true);
    REQUIRE(effective.nozzle_diameter.values == std::vector<double>{0.2, 0.4});
    const std::vector<int> plate_map = effective.filament_map.values;
    REQUIRE(plate_map == std::vector<int>{1, 1, 1, 1, 1, 2, 1, 1});

    // Step 2's first defaults, as the Plater makes them (note 12b: the parts' slots first).
    const WizardMaterialDefaults initial = wizard_default_materials(
        effective.nozzle_diameter.values, plate_map, effective.filament_type.values,
        wizard_object_material_slots({{1, {6, 4}}}));
    REQUIRE(initial.fine.has_value());
    const auto rows_for = [&](const std::vector<int> &map) {
        FullPrintConfig mapped = effective;
        mapped.filament_map_mode.value = fmmManual;
        mapped.filament_map.values = map;
        std::vector<WizardBodyRoleRow> rows;
        for (const ModelVolume *volume : {body11, body12}) {
            WizardBodyRoleRow row;
            row.object_id = object->id().id;
            row.volume_id = volume->id().id;
            row.current_slot = volume->config.get().opt_int("extruder") - 1;
            const MixedNozzleToolResolution resolved = resolve_mixed_nozzle_tool(
                mapped, std::size_t(row.current_slot), MixedNozzleResolveScope::PhysicalToolOnly);
            if (resolved)
                row.current_physical = resolved.tool->physical_extruder;
            row.volume = volume->mesh().stats().volume;
            rows.push_back(row);
        }
        return rows;
    };
    std::optional<std::size_t> initial_coarse = initial.coarse;
    if (initial_coarse && initial_coarse == initial.fine)
        initial_coarse.reset();
    const std::vector<int> initial_map = wizard_derived_filament_map(
        effective.nozzle_diameter.values, plate_map, 8, initial.fine, initial_coarse);
    const std::vector<WizardBodyRole> roles = wizard_default_body_roles(rows_for(initial_map), 0);
    REQUIRE(roles == std::vector<WizardBodyRole>{WizardBodyRole::Coarse, WizardBodyRole::Fine});
    const WizardPickerDefaults picker =
        wizard_body_picker_defaults(rows_for(initial_map), roles, {}, {}, {}, {}, std::nullopt, std::nullopt);
    CHECK(picker.fine == std::optional<std::size_t>{3});
    CHECK(picker.coarse == std::optional<std::size_t>{5});
    REQUIRE(picker.fine);
    REQUIRE(picker.coarse);

    const std::vector<int> derived = wizard_derived_filament_map(
        effective.nozzle_diameter.values, plate_map, 8, picker.fine, picker.coarse);
    CHECK(derived == plate_map);
    const std::vector<WizardBodyRoleRow> rows = rows_for(derived);
    const std::vector<WizardBodyAssignment> body =
        wizard_body_roles_to_slots(rows, roles, *picker.fine, *picker.coarse, 0, 1, {});
    std::vector<std::string> labels;
    for (std::size_t slot = 0; slot < cover_filaments().size(); ++slot)
        labels.push_back(std::to_string(slot + 1) + ": " + cover_filaments()[slot]);
    CHECK(wizard_body_material_changes({"Body11", "Body12"}, rows, body, labels).empty());

    // Accept every default: step 3's 0.16 mm row, then Apply.
    draft.scope.kind = WizardScopeKind::CurrentPlate;
    draft.scope.current_plate_id = fixture.current_plate.id().id;
    draft.scope.process_affected_plate_ids = {fixture.current_plate.id().id};
    draft.scope.allow_shared_process_changes = true;
    draft.mode = MixedNozzleSlicingMode::BodySplit;
    draft.fine_logical_filament = *picker.fine;
    draft.coarse_logical_filament = *picker.coarse;
    draft.resolved_physical_roles = {{*picker.fine, 0, 0.2, "fine"}, {*picker.coarse, 1, 0.4, "coarse"}};
    draft.chosen_fine_height = 0.08;
    draft.chosen_coarse_height = 0.16;
    draft.chosen_cadence_ratio = 2;
    draft.derived_filament_map = derived;
    draft.derived_map_mode = fmmManual;
    FullPrintConfig row_config = effective;
    row_config.filament_map_mode.value = fmmManual;
    row_config.filament_map.values = derived;
    const std::vector<WizardCandidate> candidates = build_wizard_candidates(draft, row_config, {});
    const WizardCandidate *candidate = nullptr;
    for (const WizardCadenceRow &row : wizard_cadence_page(candidates, 0.08, row_config).rows)
        if (std::abs(row.coarse_height - 0.16) < 1e-9)
            candidate = &candidates[row.candidate_index];
    REQUIRE(candidate != nullptr);
    draft = wizard_candidate_draft(draft, *candidate, body);
    const WizardReview review = build_wizard_review(draft, *candidate);
    for (const std::string &error : review.errors)
        UNSCOPED_INFO(error);
    REQUIRE(review.can_apply);
    PreparedWizardApply prepared = prepare_wizard_apply(draft, review, fixture.owners);
    INFO(prepared.diagnostic);
    REQUIRE(prepared.diagnostic.empty());
    // Step 4 says nothing about a part's material, and no slot's profile changes.
    for (const std::string &line : prepared.also_changed) {
        CAPTURE(line);
        CHECK_FALSE(has_text(line, "Body1"));
        CHECK_FALSE(has_text(line, " profile: "));
    }
    CHECK_FALSE(prepared.filament_presets_changed);

    HookLog log;
    const WizardApplyResult result = commit_wizard_apply(std::move(prepared), fixture.owners, hooks_for(log));
    INFO(result.diagnostic);
    REQUIRE(result.applied);
    CHECK(body11->config.get().opt_int("extruder") == 6);
    CHECK(body12->config.get().opt_int("extruder") == 4);
    CHECK(fixture.bundle.filament_presets == presets_before);
    const std::vector<int> after = fixture.current_plate.get_real_filament_maps(fixture.bundle.project_config);
    REQUIRE(after.size() >= 6);
    CHECK(after[3] == 1);
    CHECK(after[5] == 2);
}

TEST_CASE("OB-30: a slot's material settings are composed for the nozzle the plate puts it on",
          "[TestRebuild][OwnerGuard][WizardApply]")
{
    CoverFixture fixture;
    const auto binding = [](const DynamicPrintConfig &config, std::size_t slot) {
        const auto *values = config.option<ConfigOptionStrings>("mixed_nozzle_filament_binding");
        REQUIRE(values != nullptr);
        REQUIRE(slot < values->values.size());
        return values->values[slot];
    };
    const std::vector<int> plate_map = fixture.current_plate.get_real_filament_maps(fixture.bundle.project_config);
    REQUIRE(plate_map.size() == 8);
    REQUIRE(plate_map[5] == 2);
    // The project-level map alone puts slot 6 on the 0.2, as a build 18 file saved it.
    CHECK(binding(fixture.bundle.full_config(), 5) == "sibling:Bambu PETG HF @BBL H2D 0.2 nozzle");
    // Every composition the slice, the saved project and the setup read follows the plate's map.
    const std::string own = "base:Bambu PETG HF @BBL H2D 0.4 nozzle";
    CHECK(binding(fixture.bundle.full_config(true, plate_map), 5) == own);
    CHECK(binding(wizard_plate_effective_config(fixture.bundle, fixture.current_plate), 5) == own);
    CHECK(binding(fixture.bundle.full_config_secure(plate_map), 5) == own);
    CHECK(binding(wizard_plate_effective_config(fixture.bundle, fixture.current_plate), 3) ==
          "base:Bambu PETG Basic @BBL H2D 0.2 nozzle");
    CHECK_FALSE(fixture.bundle.mixed_nozzle_rebind_plan(plate_map,
        fixture.current_plate.get_real_filament_volume_maps(fixture.bundle.project_config)).offered);
}

// ------------------------------------------------------------------------------------------------
// High Flow, nozzle pair, sidebar cards and File > New
// ------------------------------------------------------------------------------------------------

namespace {

DynamicPrintConfig project_with(std::vector<double> pair, std::vector<int> flows)
{
    DynamicPrintConfig project;
    if (!pair.empty())
        project.set_key_value("nozzle_diameter", new ConfigOptionFloats(pair));
    project.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true)->values = std::move(flows);
    return project;
}

} // namespace

TEST_CASE("OB-05 OB-31: a project that owns a dissimilar pair keeps its flow types through a printer refresh",
          "[TestRebuild][OwnerGuard][HighFlow]")
{
    const int standard = int(nvtStandard);
    const int high_flow = int(nvtHighFlow);
    CHECK(project_owns_nozzle_flows(project_with({0.2, 0.8}, {standard, high_flow}), 2));
    CHECK_FALSE(project_owns_nozzle_flows(project_with({}, {standard, high_flow}), 2));
    CHECK_FALSE(project_owns_nozzle_flows(project_with({0.4, 0.4}, {standard, high_flow}), 2));
    CHECK_FALSE(project_owns_nozzle_flows(project_with({0.2, 0.}, {standard, high_flow}), 2));
    CHECK_FALSE(project_owns_nozzle_flows(project_with({0.2, 0.8}, {standard, high_flow}), 1));
    CHECK_FALSE(project_owns_nozzle_flows(project_with({0.2, 0.8}, {high_flow}), 2));

    // The real bundle refresh paths on a 0.2 Standard / 0.6 High Flow project whose printer
    // defaults are Standard on both sides.
    PresetBundle bundle;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("filament_diameter", new ConfigOptionFloats{1.75, 1.75});
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    config.set_key_value("print_extruder_id", new ConfigOptionInts{1, 2});
    config.set_key_value("print_extruder_variant", new ConfigOptionStrings{"Direct Drive Standard", "Direct Drive High Flow"});
    config.set_key_value("extruder_type",
        new ConfigOptionEnumsGeneric{int(ExtruderType::etDirectDrive), int(ExtruderType::etDirectDrive)});
    config.set_key_value("nozzle_volume_type", new ConfigOptionEnumsGeneric{standard, high_flow});
    config.set_key_value("extruder_variant_list", new ConfigOptionStrings{"Direct Drive Standard", "Direct Drive High Flow"});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.08});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.20, 0.60});
    DynamicPrintConfig process = config;
    process.erase("filament_map");
    process.erase("filament_map_mode");
    process.erase("filament_volume_map");
    bundle.prints.load_preset("", "process-A", process, true);
    bundle.printers.load_preset("", "printer-A", config, true);
    bundle.filaments.load_preset("", "material-0", config, true);
    bundle.filaments.load_preset("", "material-1", config, false);
    bundle.filament_presets = {"material-0", "material-1"};
    bundle.project_config = config;
    auto &project = bundle.project_config;
    auto *flows = new ConfigOptionEnumsGeneric(project.def()->get("nozzle_volume_type")->enum_keys_map);
    flows->values = {standard, high_flow};
    project.set_key_value("nozzle_volume_type", flows);
    bundle.printers.get_edited_preset().config.set_key_value(
        "default_nozzle_volume_type", new ConfigOptionEnumsGeneric{standard, standard});
    const bool preserve = GENERATE(true, false);
    CAPTURE(preserve);
    SECTION("extruder count refresh") {
        with_project_nozzle_flows_preserved(project, preserve, [&] { bundle.on_extruders_count_changed(2); });
        CHECK(bundle.extruder_ams_counts.size() == 2);
    }
    SECTION("base profile refresh restoring a saved Standard preference") {
        with_project_nozzle_flows_preserved(project, preserve, [&] {
            REQUIRE(project.option<ConfigOptionEnumsGeneric>("nozzle_volume_type")->deserialize("Standard,Standard"));
            project.set_key_value("has_filament_switcher", new ConfigOptionBool(false));
        });
        CHECK_FALSE(project.opt_bool("has_filament_switcher"));
    }
    CHECK(project.option<ConfigOptionEnumsGeneric>("nozzle_volume_type")->values ==
          (preserve ? std::vector<int>{standard, high_flow} : std::vector<int>{standard, standard}));
}

TEST_CASE("OB-05 OB-31: the Flow combo offers High Flow per nozzle and keeps the project's type through a rebuild",
          "[TestRebuild][OwnerGuard][HighFlow]")
{
    const int standard = int(nvtStandard);
    const int high_flow = int(nvtHighFlow);
    // High Flow follows each nozzle's own diameter, not the printer variant's.
    const ConfigOptionFloats fine_coarse{0.2, 0.6};
    CHECK_FALSE(mixed_nozzle_high_flow_offered(&fine_coarse, 0, "0.2", "Bambu Lab H2D"));
    CHECK(mixed_nozzle_high_flow_offered(&fine_coarse, 1, "0.2", "Bambu Lab H2D"));
    const ConfigOptionFloats fine_pair{0.2, 0.2};
    CHECK_FALSE(mixed_nozzle_high_flow_offered(&fine_pair, 1, "0.2", "Bambu Lab H2D"));
    const ConfigOptionFloats coarse_pair{0.4, 0.6};
    CHECK(mixed_nozzle_high_flow_offered(&coarse_pair, 0, "0.2", "Bambu Lab H2D"));
    CHECK_FALSE(mixed_nozzle_high_flow_offered(&fine_coarse, 1, "0.2", "Bambu Lab X1E"));
    CHECK_FALSE(mixed_nozzle_high_flow_offered(nullptr, 0, "0.2", "Bambu Lab X1"));
    CHECK(mixed_nozzle_high_flow_offered(nullptr, 0, "0.6", "Bambu Lab X1"));

    // A High Flow the gate briefly does not offer stays listed and selected, never Standard.
    const auto gated = sidebar_flow_choices({standard}, high_flow);
    CHECK(gated.types == std::vector<int>{standard, high_flow});
    CHECK(gated.selection == 1);
    const auto both = sidebar_flow_choices({standard, high_flow}, high_flow);
    CHECK(both.selection == 1);
    CHECK(sidebar_flow_choices({standard}, standard).selection == 0);
    CHECK(sidebar_flow_choices({}, -1).selection == -1);
}

TEST_CASE("OB-09: a printer refresh keeps the project nozzle pair and never reads a card past its data",
          "[TestRebuild][OwnerGuard][NozzleCards]")
{
    // Whatever variant is selected, the project's pair decides the refresh route.
    DynamicPrintConfig project;
    project.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.6});
    DynamicPrintConfig variant_06;
    variant_06.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.6, 0.6});
    DynamicPrintConfig variant_one_entry;
    variant_one_entry.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.6});
    CHECK(mixed_nozzle_sync_preserves_project(project, variant_06));
    CHECK(mixed_nozzle_sync_preserves_project(project, variant_one_entry));
    CHECK(project.option<ConfigOptionFloats>("nozzle_diameter")->values == std::vector<double>{0.2, 0.6});
    // Reseeding the boxes with the pair they hold is a no-op, never the preset switch that clears it.
    CHECK(resolve_sidebar_diameter_route({0.2, 0.6}, mixed_nozzle_diameter_text(0.2), mixed_nozzle_diameter_text(0.6),
                                         true, true) == SidebarDiameterRoute::NoOp);
    DynamicPrintConfig no_pair;
    CHECK_FALSE(mixed_nozzle_sync_preserves_project(no_pair, variant_06));
    DynamicPrintConfig homogeneous;
    homogeneous.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.6, 0.6});
    CHECK_FALSE(mixed_nozzle_sync_preserves_project(homogeneous, variant_06));

    // A one-entry per-extruder vector: the right card reads the first entry; none skips the row.
    CHECK(sidebar_nozzle_card_entry(2, 1) == std::optional<std::size_t>{1});
    CHECK(sidebar_nozzle_card_entry(1, 1) == std::optional<std::size_t>{0});
    CHECK_FALSE(sidebar_nozzle_card_entry(0, 1).has_value());
    CHECK(sidebar_enum_label_valid(1, 2));
    CHECK_FALSE(sidebar_enum_label_valid(2, 2));
    CHECK_FALSE(sidebar_enum_label_valid(-1, 2));
}

TEST_CASE("OB-15: changing a nozzle of a project-owned pair asks first and never opens setup unasked",
          "[TestRebuild][OwnerGuard][NozzlePrompt]")
{
    const std::vector<double> owned{0.2, 0.8};
    CHECK(resolve_sidebar_diameter_route(owned, "0.2", "0.6", true, true) == SidebarDiameterRoute::DissimilarPrompt);
    CHECK(resolve_sidebar_diameter_route(owned, "0.4", "0.8", true, true) == SidebarDiameterRoute::DissimilarPrompt);
    CHECK(resolve_sidebar_diameter_route(owned, "0.2", "0.8", true, true) == SidebarDiameterRoute::NoOp);
    CHECK(resolve_sidebar_diameter_route(owned, "0.8", "0.8", true, true) == SidebarDiameterRoute::PresetSwitch);
    // Finding B: either side changing on a .2/.4 pair is never inert.
    const std::vector<double> small{0.2, 0.4};
    CHECK(resolve_sidebar_diameter_route(small, "0.4", "0.4", true, true) == SidebarDiameterRoute::PresetSwitch);
    CHECK(resolve_sidebar_diameter_route(small, "0.2", "0.2", true, true) == SidebarDiameterRoute::PresetSwitch);
    CHECK(resolve_sidebar_diameter_route(small, "0.6", "0.4", true, true) == SidebarDiameterRoute::DissimilarPrompt);
    // A printer-owned pair keeps the legacy routes.
    CHECK(resolve_sidebar_diameter_route({0.4, 0.4}, "0.4", "0.2", false, true) == SidebarDiameterRoute::DissimilarPrompt);
    CHECK(resolve_sidebar_diameter_route({0.4, 0.4}, "0.6", "0.6", false, true) == SidebarDiameterRoute::PresetSwitch);

    // The prompt's answers: only a ticked Keep opens setup; a single side switches; Cancel returns.
    const DissimilarNozzleSelection captured{"0.2", "0.6", "Bambu Lab H2D 0.2 nozzle"};
    const auto keep_only = resolve_dissimilar_nozzle_decision(captured, DissimilarNozzleDecision::KeepPair, false);
    CHECK(keep_only.keep_pair);
    CHECK_FALSE(keep_only.open_setup);
    CHECK(keep_only.return_before_preset_lookup);
    const auto keep_and_set_up = resolve_dissimilar_nozzle_decision(captured, DissimilarNozzleDecision::KeepPair, true);
    CHECK(keep_and_set_up.keep_pair);
    CHECK(keep_and_set_up.open_setup);
    const auto left_only = resolve_dissimilar_nozzle_decision(captured, DissimilarNozzleDecision::LeftOnly, true);
    CHECK_FALSE(left_only.open_setup);
    CHECK(left_only.single_diameter == std::optional<std::string>{"0.2"});
    const auto right_only = resolve_dissimilar_nozzle_decision(captured, DissimilarNozzleDecision::RightOnly, true);
    CHECK(right_only.single_diameter == std::optional<std::string>{"0.6"});
    const auto cancel = resolve_dissimilar_nozzle_decision(captured, DissimilarNozzleDecision::Cancel, true);
    CHECK_FALSE(cancel.open_setup);
    CHECK_FALSE(cancel.keep_pair);
    CHECK(cancel.return_before_preset_lookup);
}

TEST_CASE("OB-08: a slot's profile for its own nozzle is not flagged in a mixed-nozzle project",
          "[TestRebuild][OwnerGuard][NozzleCards]")
{
    CHECK(mixed_nozzle_slot_preset_fits_nozzle(true, "base:Bambu PLA Basic @BBL H2D 0.8 nozzle"));
    CHECK_FALSE(mixed_nozzle_slot_preset_fits_nozzle(true, "sibling:Bambu PLA Basic @BBL H2D 0.8 nozzle"));
    CHECK_FALSE(mixed_nozzle_slot_preset_fits_nozzle(true, "unresolved:missing"));
    CHECK_FALSE(mixed_nozzle_slot_preset_fits_nozzle(false, "base:Bambu PLA Basic @BBL H2D 0.8 nozzle"));
}

TEST_CASE("OB-32: File > New starts mixed-nozzle slicing Off and keeps the installed pair",
          "[TestRebuild][OwnerGuard][NewProject]")
{
    // Every mixed-nozzle key a project carries is setup state that a new project resets.
    const auto &reset = mixed_nozzle_new_project_reset_keys();
    const auto listed = [&reset](const std::string &key) { return std::find(reset.begin(), reset.end(), key) != reset.end(); };
    {
        PresetBundle fresh;
        for (const std::string &key : fresh.project_config.keys())
            if (key.rfind("mixed_nozzle_", 0) == 0) {
                INFO(key);
                CHECK(listed(key));
            }
    }
    for (const char *key : {"nozzle_diameter", "min_layer_height", "max_layer_height", "nozzle_volume_type",
                            "filament_colour", "filament_map", "filament_volume_map", "filament_nozzle_map"})
        CHECK_FALSE(listed(key));

    // The previous project: Feature Split on a 0.2 Standard / 0.8 High Flow pair with setup state.
    PresetBundle bundle;
    DynamicPrintConfig &previous_project = bundle.project_config;
    previous_project.set_key_value("mixed_nozzle_slicing_mode",
        new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::FeatureSplit));
    previous_project.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.8});
    previous_project.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.16});
    previous_project.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.56});
    previous_project.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true)->values = {int(nvtStandard), int(nvtHighFlow)};
    previous_project.option<ConfigOptionStrings>("filament_colour", true)->values = {"#FFD700", "#000000"};
    previous_project.set_key_value("mixed_nozzle_tower_policy_state", new ConfigOptionString("v1:owned"));
    previous_project.set_key_value("mixed_nozzle_tower_policy_fresh", new ConfigOptionBool(false));
    previous_project.option<ConfigOptionStrings>("mixed_nozzle_filament_explicit_keys", true)->values = {"filament_max_volumetric_speed", ""};
    previous_project.option<ConfigOptionStrings>("mixed_nozzle_filament_provenance", true)->values = {"bound", "bound"};
    previous_project.option<ConfigOptionStrings>("mixed_nozzle_filament_binding", true)->values = {"base:", "sibling:"};
    const DynamicPrintConfig previous = bundle.project_config;

    // What Plater::new_project() does: the ordinary reset, then the mixed-nozzle new project step.
    bundle.reset_project_embedded_presets();
    CHECK(apply_mixed_nozzle_new_project(bundle.project_config, previous));
    const DynamicPrintConfig &project = bundle.project_config;
    CHECK(project.opt_enum<MixedNozzleSlicingMode>("mixed_nozzle_slicing_mode") == MixedNozzleSlicingMode::Off);
    CHECK_FALSE(project.has("mixed_nozzle_tower_policy_state"));
    CHECK(project.opt_bool("mixed_nozzle_tower_policy_fresh"));
    for (const char *key : {"mixed_nozzle_filament_explicit_keys", "mixed_nozzle_filament_provenance",
                            "mixed_nozzle_filament_binding"})
        CHECK(project.option<ConfigOptionStrings>(key)->values.empty());
    for (const char *key : {"nozzle_diameter", "min_layer_height", "max_layer_height", "nozzle_volume_type", "filament_colour"}) {
        INFO(key);
        REQUIRE(project.option(key) != nullptr);
        CHECK(project.opt_serialize(key) == previous.opt_serialize(key));
    }
    CHECK_FALSE(apply_mixed_nozzle_new_project(bundle.project_config, previous));

    // No pair before: none is invented, and a stray legacy coarse height is erased.
    DynamicPrintConfig body_previous;
    body_previous.set_key_value("mixed_nozzle_slicing_mode",
        new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::BodySplit));
    DynamicPrintConfig stray = body_previous;
    stray.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(0.24));
    CHECK(apply_mixed_nozzle_new_project(stray, body_previous));
    CHECK_FALSE(stray.has("nozzle_diameter"));
    CHECK_FALSE(stray.has("mixed_nozzle_coarse_layer_height"));
    CHECK(stray.opt_enum<MixedNozzleSlicingMode>("mixed_nozzle_slicing_mode") == MixedNozzleSlicingMode::Off);
}
