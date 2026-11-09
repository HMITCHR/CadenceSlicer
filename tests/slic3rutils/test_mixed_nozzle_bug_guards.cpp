// Guards for reported bugs in the setup wizard and project handling. Helpers that are file-local in
// the sources are reached through the public function that calls them.

#include <catch2/catch_all.hpp>

#include "libslic3r/MixedNozzleConfig.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "slic3r/GUI/MixedNozzleWizardModel.hpp"

#include "wizard_apply_support.hpp"

#include <array>
#include <clocale>
#include <cmath>
#include <cstdio>
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

// A cube: 0.2/0.8 pair, 0.08 mm fine, rows N2 to N7.
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
// slower than 0.56.
const std::map<int, double> kRankCubeSeconds{
    {2, 10320.}, {3, 9100.}, {4, 8200.}, {5, 8000.}, {6, 7200.}, {7, 7080.}};

std::optional<std::size_t> row_of(const WizardCadencePage &page, std::size_t candidate)
{
    for (std::size_t index = 0; index < page.rows.size(); ++index)
        if (page.rows[index].candidate_index == candidate)
            return index;
    return std::nullopt;
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

// A 0.2 / 0.6 pair at 0.1 mm with no machine presets behind it.
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

} // namespace

// ------------------------------------------------------------------------------------------------
// Step 3: ranking and selection
// ------------------------------------------------------------------------------------------------

TEST_CASE("Step 3 orders, ties and selects the coarse layer rows by their times",
          "[TestRebuild][BugGuard][WizardSpeed]")
{
    const RankCube cube = rank_cube(kRankCubeSeconds);
    const WizardEstimate baseline = wizard_estimate(9000., 0, 0.);
    const std::size_t n2 = cube.by_ratio.at(2), n5 = cube.by_ratio.at(5), n6 = cube.by_ratio.at(6),
                      n7 = cube.by_ratio.at(7);
    const auto page_of = [&cube](const std::vector<WizardCandidate> &candidates, const WizardEstimate &one) {
        return wizard_cadence_page(candidates, 0.08, cube.config, one);
    };

    SECTION("the tied fastest rows lead; a static default gives way, a pick by hand is kept") {
        const WizardCadencePage page = page_of(cube.candidates, baseline);
        REQUIRE(page.rows.size() == 6);
        REQUIRE(page.ranked);
        CHECK(page.no_clear_fastest);
        CHECK(page.fastest_group == 2);
        CHECK(page.rows[0].ratio == 7);
        CHECK(page.rows[1].ratio == 6);
        CHECK(wizard_ranked_selection(page, n2, false) == std::optional<std::size_t>{0});
        CHECK(wizard_ranked_selection(page, n2, true) == row_of(page, n2));
        CHECK(wizard_ranked_selection(page, n6, true) == std::optional<std::size_t>{1});
        CHECK_FALSE(wizard_offer_one_nozzle(page, std::size_t(0)));
    }
    SECTION("more than 2 percent apart is a clear fastest; a tie goes to the row with fewer nozzle changes") {
        std::vector<WizardCandidate> candidates = cube.candidates;
        candidates[n6].estimate = wizard_estimate(7280., 10, 0.);
        CHECK(page_of(candidates, baseline).fastest_group == 1);
        candidates[n6].estimate = wizard_estimate(7200., 3, 0.);
        const WizardCadencePage tied = page_of(candidates, baseline);
        REQUIRE(tied.fastest_group == 2);
        CHECK(wizard_ranked_selection(tied, std::nullopt) == row_of(tied, n6));
        CHECK(wizard_ranked_selection(tied, n7, true) == row_of(tied, n7));
    }
    SECTION("one nozzle only is offered when every row is slower than it, or the row changes no nozzle") {
        // Within 2 percent of one nozzle only is not slower.
        CHECK_FALSE(wizard_switching_saves_no_time(page_of(cube.candidates, wizard_estimate(7000., 0, 0.))));
        const WizardCadencePage slower = page_of(cube.candidates, wizard_estimate(6000., 0, 0.));
        CHECK(wizard_switching_saves_no_time(slower));
        CHECK(wizard_offer_one_nozzle(slower, std::size_t(0)));
        std::vector<WizardCandidate> candidates = cube.candidates;
        candidates[n7].estimate = wizard_estimate(7080., 0, 0.);
        const WizardCadencePage idle = page_of(candidates, baseline);
        CHECK(wizard_offer_one_nozzle(idle, row_of(idle, n7)));
        CHECK_FALSE(wizard_offer_one_nozzle(idle, row_of(idle, n6)));
    }
    SECTION("timed rows go on top fastest first while the rest wait in layer order") {
        const RankCube partial = rank_cube({{5, 8000.}, {7, 7080.}});
        const WizardCadencePage page = page_of(partial.candidates, WizardEstimate{});
        REQUIRE(page.rows.size() == 6);
        CHECK_FALSE(page.ranked);
        std::vector<int> order;
        for (const WizardCadenceRow &row : page.rows)
            order.push_back(row.ratio);
        CHECK(order == std::vector<int>{7, 5, 2, 3, 4, 6});
        RankCube failed = rank_cube({});
        for (WizardCandidate &candidate : failed.candidates) {
            candidate.estimate = WizardEstimate{};
            candidate.estimate->status = WizardEstimateStatus::Failed;
        }
        CHECK(page_of(failed.candidates, baseline).rows.front().ratio == 2);
    }
    SECTION("the project's own coarse layer stays selected when times arrive") {
        WizardCadencePage page = page_of(cube.candidates, baseline);
        const std::optional<std::pair<double, double>> existing = std::make_pair(0.08, 0.40);
        wizard_mark_current(page, 0.08, existing);
        const std::optional<std::size_t> current = wizard_existing_row(page, 0.08, existing);
        REQUIRE(current);
        CHECK(page.rows[*current].current);
        CHECK(page.rows[*current].ratio == 5);
        CHECK(wizard_ranked_selection(page, n5, true) == current);
        // A fresh setup has no own row, and the fastest is selected.
        WizardCadencePage fresh = page_of(cube.candidates, baseline);
        wizard_mark_current(fresh, 0.08, std::nullopt);
        for (const WizardCadenceRow &row : fresh.rows)
            CHECK_FALSE(row.current);
        CHECK(wizard_ranked_selection(fresh, n5, false) == std::optional<std::size_t>{0});
    }
    SECTION("Detail and speed holds Next while the default row has no time, not a row picked by hand") {
        REQUIRE(wizard_page_can_block_next(2));
        std::vector<WizardCandidate> candidates = cube.candidates;
        const auto holds = [&](bool explicit_pick, bool ranking) {
            const WizardCadencePage page = page_of(candidates, baseline);
            return !wizard_speed_block_reason(page, row_of(page, n2), explicit_pick, ranking).empty();
        };
        candidates[n2].estimate = WizardEstimate{};
        CHECK(holds(false, true));
        CHECK_FALSE(holds(true, true));
        // Without ranking the rows never get a time, so there is nothing to wait for.
        CHECK_FALSE(holds(false, false));
        const WizardCadencePage page = page_of(candidates, baseline);
        CHECK(wizard_speed_block_reason(page, row_of(page, n7), false, true).empty());
        // Before the first estimate arrives the row has none, and a row whose slice failed has its answer.
        candidates[n2].estimate.reset();
        CHECK(holds(false, true));
        candidates[n2].estimate = WizardEstimate{};
        candidates[n2].estimate->status = WizardEstimateStatus::Failed;
        CHECK_FALSE(holds(false, true));
    }
    SECTION("a row that could not be sliced is not kept selected, and Review cannot apply it") {
        std::vector<WizardCandidate> candidates = cube.candidates;
        WizardEstimate failed;
        failed.status = WizardEstimateStatus::Failed;
        WizardEstimate refused;
        refused.status = WizardEstimateStatus::Unavailable;
        refused.refused = true;
        refused.refusal = "refused";
        for (const WizardEstimate &bad : {failed, refused}) {
            candidates[n5].estimate = bad;
            const WizardCadencePage page = page_of(candidates, baseline);
            REQUIRE_FALSE(page.ranked);
            for (const bool explicit_pick : {true, false}) {
                CAPTURE(explicit_pick);
                const std::optional<std::size_t> selected = wizard_ranked_selection(page, n5, explicit_pick);
                REQUIRE(selected);
                CHECK(page.rows[*selected].ratio == 7);
            }
        }
        // A row only waiting for its time stays selected.
        candidates[n5].estimate = WizardEstimate{};
        const WizardCadencePage waiting = page_of(candidates, baseline);
        CHECK(wizard_ranked_selection(waiting, n5, true) == row_of(waiting, n5));

        WizardReviewSummaryInput input;
        input.mode = MixedNozzleSlicingMode::FeatureSplit;
        input.nozzle_diameters = {0.2, 0.8};
        input.fine_height = 0.08;
        input.coarse_height = 0.40;
        input.estimate = failed;
        const WizardReviewSummary summary = wizard_review_summary(input);
        CHECK_FALSE(summary.ready);
        CHECK(summary.target == WizardFixTarget::Speed);
        input.estimate = refused;
        CHECK_FALSE(wizard_review_summary(input).ready);
        input.estimate = wizard_estimate(8000., 10, 0.);
        CHECK(wizard_review_summary(input).ready);
    }
}

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

// ------------------------------------------------------------------------------------------------
// Step 1 and 2: mode, parts and materials
// ------------------------------------------------------------------------------------------------

TEST_CASE("Step 1 opens on the project's own mode; a new setup picks Body Split for a multi-part object",
          "[TestRebuild][BugGuard][WizardParts][MixedNozzleSetup]")
{
    // A project that has a mode keeps it, painted or multi-part.
    CHECK(wizard_default_mode(MixedNozzleSlicingMode::FeatureSplit, true) == MixedNozzleSlicingMode::FeatureSplit);
    CHECK(wizard_default_mode(MixedNozzleSlicingMode::BodySplit, false) == MixedNozzleSlicingMode::BodySplit);
    CHECK(wizard_default_mode(MixedNozzleSlicingMode::Off, true) == MixedNozzleSlicingMode::BodySplit);
    // Painted colors on more than two tool changer tools: Body Split cannot pair them, so a new setup opens on
    // Feature Split, and a project's own Body Split is kept.
    CHECK(wizard_default_mode(MixedNozzleSlicingMode::Off, true, true) == MixedNozzleSlicingMode::FeatureSplit);
    CHECK(wizard_default_mode(MixedNozzleSlicingMode::BodySplit, true, true) == MixedNozzleSlicingMode::BodySplit);
}

TEST_CASE("Step 2 pickers start on the material each nozzle holds and never on a stale or unmapped slot",
          "[TestRebuild][BugGuard][WizardParts]")
{
    SECTION("Body Split parts sharing one slot") {
        const std::vector<std::string> labels{"1: eSUN PLA+", "2: PLA Basic", "3: eSUN PLA+", "4: PETG Basic",
                                              "5: ASA", "6: PETG HF", "7: PLA Basic", "8: PETG Translucent"};
        const std::vector<std::string> types{"PLA", "PLA", "PLA", "PETG", "ASA", "PETG", "PLA", "PETG"};
        const std::vector<std::string> one_petg{"PLA", "PLA", "PLA", "PETG", "ASA", "PLA", "PLA", "PLA"};
        const std::vector<WizardBodyRole> roles{WizardBodyRole::Coarse, WizardBodyRole::Fine};
        // The pickers as an earlier opening left them: fine on slot 4, coarse on PLA slot 1.
        const std::optional<std::size_t> stale_fine{3}, stale_coarse{0};
        struct Row {
            const char *what;
            std::vector<int> map;
            bool unique_petg;
            int body11_slot;
            std::size_t body11_physical, body12_physical;
            bool swap_picks;
            std::optional<std::size_t> fine, coarse;
            bool note;
        };
        const std::vector<Row> table{
            {"slot 4 fine: coarse takes the closest PETG on the coarse nozzle", {1, 1, 1, 1, 1, 2, 1, 1}, false, 3, 0, 0,
             false, 3, 5, true},
            {"slot 4 coarse: fine takes a PETG on the fine nozzle", {1, 1, 1, 2, 1, 2, 1, 1}, false, 3, 1, 1, true, 7, 3,
             true},
            {"no other slot of that material: coarse left empty", {1, 1, 1, 1, 1, 2, 1, 1}, true, 3, 0, 0, false, 3,
             std::nullopt, true},
            {"no PETG mapped to the coarse nozzle: coarse left empty", {1, 1, 1, 1, 1, 1, 1, 1}, false, 3, 0, 0, false,
             3, std::nullopt, true},
            {"parts on their own slots keep them", {1, 1, 1, 1, 1, 2, 1, 1}, false, 5, 1, 0, false, 3, 5, false},
        };
        for (const Row &row : table) {
            INFO(row.what);
            WizardBodyRoleRow body11;
            body11.object_id = 3;
            body11.volume_id = 11;
            body11.current_slot = row.body11_slot;
            body11.current_physical = row.body11_physical;
            body11.volume = 14000.;
            WizardBodyRoleRow body12 = body11;
            body12.volume_id = 12;
            body12.current_slot = 3;
            body12.current_physical = row.body12_physical;
            body12.volume = 900.;
            const WizardPickerDefaults defaults = wizard_body_picker_defaults(
                {body11, body12}, roles, {0.2, 0.4}, row.map, row.unique_petg ? one_petg : types, labels,
                row.swap_picks ? stale_coarse : stale_fine, row.swap_picks ? stale_fine : stale_coarse);
            CHECK(defaults.fine == row.fine);
            CHECK(defaults.coarse == row.coarse);
            CHECK(defaults.note.empty() == !row.note);
        }
    }
    SECTION("Feature and Body Split defaults from the plate's filament map") {
        struct Row {
            const char *what;
            std::vector<double> nozzles;
            std::vector<int> map;
            std::vector<std::string> types;
            std::vector<std::size_t> object_slots;
            std::optional<std::size_t> fine, coarse;
            std::optional<bool> note;
        };
        const std::vector<double> pair{0.2, 0.6};
        std::vector<Row> table{
            {"eight PLA slots, slot 7 on the right", pair, {1, 1, 1, 1, 1, 1, 2, 1}, std::vector<std::string>(8, "PLA"),
             {0}, 0, 6, false},
            {"same type first on the coarse nozzle", pair, {1, 2, 2}, {"PETG", "PLA", "PETG"}, {0}, 0, 2, false},
            {"the objects' slot is fine only on the fine nozzle", pair, {2, 1}, {"PLA", "PLA"}, {0}, 1, 0, false},
            {"the smaller nozzle is fine on either side", {0.6, 0.2}, {1, 2}, {"PLA", "PLA"}, {}, 1, 0, false},
            {"one material only", pair, {1}, {"PLA"}, {0}, 0, std::nullopt, std::nullopt},
        };
        for (const std::vector<double> &sizes : {std::vector<double>{.2, .4}, std::vector<double>{.2, .6},
                                                 std::vector<double>{.4, .8}}) {
            const std::vector<double> tools{sizes[0], sizes[0], sizes[0], sizes[1]};
            table.push_back({"nothing mapped to the coarse nozzle", sizes, {1, 1, 1}, {"PLA", "PETG", "PLA"}, {0}, 0,
                             std::nullopt, true});
            table.push_back({"a tool changer: the coarse tool's own slot", tools, {1, 2, 3, 4},
                             {"PLA", "PLA", "PLA", "PETG"}, {0}, 0, 3, false});
            table.push_back({"a tool changer with fewer materials than tools", tools, {1, 2, 3}, {"PLA", "PLA", "PLA"},
                             {0}, 0, std::nullopt, true});
        }
        for (const Row &row : table) {
            INFO(row.what);
            CAPTURE(row.nozzles);
            const WizardMaterialDefaults defaults =
                wizard_default_materials(row.nozzles, row.map, row.types, row.object_slots);
            CHECK(defaults.fine == row.fine);
            CHECK(defaults.coarse == row.coarse);
            if (row.note)
                CHECK(defaults.note.empty() == !*row.note);
        }
    }
    SECTION("a coarse material with no profile for its nozzle or flow type blocks step 2") {
        FullPrintConfig config = expanded_variant_material_config();
        CHECK_FALSE(wizard_material_resolution(config, 1, true).needs_input);
        const auto drop_coarse_column = [](FullPrintConfig &edited) {
            edited.filament_self_index.values = {1, 1, 2};
            edited.filament_extruder_variant.values = {"Direct Drive Standard", "Direct Drive High Flow",
                                                       "Direct Drive Standard"};
        };
        SECTION("logical filament 2 keeps only its fine-nozzle column") {
            drop_coarse_column(config);
            CHECK_FALSE(wizard_material_resolution(config, 0, true).needs_input);
            CHECK(wizard_material_resolution(config, 1, true).needs_input);
            CHECK_FALSE(wizard_materials_block_reason(config, std::size_t(0), std::size_t(1), false, {}).empty());
        }
        SECTION("H2C Hybrid rack: the material's own flow column decides") {
            config.nozzle_volume_type.values = {int(nvtStandard), int(nvtHybrid)};
            config.filament_volume_map.values = {int(nvtStandard), int(nvtHighFlow)};
            CHECK_FALSE(wizard_material_resolution(config, 1, true).needs_input);
            drop_coarse_column(config);
            CHECK(wizard_material_resolution(config, 1, true).needs_input);
            config.filament_volume_map.values = {int(nvtStandard), int(nvtStandard)};
            CHECK_FALSE(wizard_material_resolution(config, 1, true).needs_input);
        }
    }
}

TEST_CASE("The Body preset choice reads the nozzle pair from the name or its parent, either order",
          "[TestRebuild][BugGuard][BodyPreset]")
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

// ------------------------------------------------------------------------------------------------
// Apply on a two-part object
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

// A two-part project on the installed BBL profiles: project map all on tool 1, the
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
        object->name = "Two-part object";
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

TEST_CASE("Body Split setup on a two-part object keeps every part's material",
          "[TestRebuild][BugGuard][WizardApply]")
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

    // Step 2's first defaults, as the Plater makes them: the parts' slots come before the object's own.
    const std::vector<std::size_t> object_slots = wizard_object_material_slots({{1, {6, 4}}});
    CHECK(object_slots == std::vector<std::size_t>{5, 3, 0});
    const WizardMaterialDefaults initial = wizard_default_materials(
        effective.nozzle_diameter.values, plate_map, effective.filament_type.values, object_slots);
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
    // Step 2 starts on the parts' own materials.
    const WizardPickerDefaults picker =
        wizard_body_picker_defaults(rows_for(initial_map), roles, {}, {}, {}, {}, std::nullopt, std::nullopt);
    CHECK(picker.fine == std::optional<std::size_t>{3});
    CHECK(picker.coarse == std::optional<std::size_t>{5});
    REQUIRE(picker.fine);
    REQUIRE(picker.coarse);

    // The map setup writes keeps the plate's, also when an older save had slot 4 on the coarse nozzle.
    const std::vector<int> derived = wizard_derived_filament_map(
        effective.nozzle_diameter.values, plate_map, 8, picker.fine, picker.coarse);
    CHECK(derived == plate_map);
    std::vector<int> saved = plate_map;
    saved[3] = 2;
    CHECK(wizard_derived_filament_map(effective.nozzle_diameter.values, saved, 8, picker.fine, picker.coarse) ==
          plate_map);
    const std::vector<WizardBodyRoleRow> rows = rows_for(derived);
    const std::vector<WizardBodyAssignment> body =
        wizard_body_roles_to_slots(rows, roles, *picker.fine, *picker.coarse, 0, 1, {});
    std::vector<std::string> labels;
    for (std::size_t slot = 0; slot < cover_filaments().size(); ++slot)
        labels.push_back(std::to_string(slot + 1) + ": " + cover_filaments()[slot]);
    CHECK(wizard_body_material_changes({"Body11", "Body12"}, rows, body, labels).empty());
    // The old default (slot 1 for the fine part) would change one part's material.
    std::vector<WizardBodyAssignment> old_default = body;
    REQUIRE(old_default.size() == 2);
    old_default[1].logical_filament = 0;
    CHECK(wizard_body_material_changes({"Body11", "Body12"}, rows, old_default, labels).size() == 1);

    // An exact slot from More options wins over the part's role, and reads as set when setup opens
    // again; a part on the fine or coarse material, or on a slot the project lacks, reads as set by its role.
    const std::vector<std::optional<WizardBodySlotOverride>> pinned_slot{std::nullopt, WizardBodySlotOverride{6, 0}};
    const std::vector<WizardBodyAssignment> pinned =
        wizard_body_roles_to_slots(rows, roles, *picker.fine, *picker.coarse, 0, 1, pinned_slot);
    REQUIRE(pinned.size() == 2);
    CHECK(pinned[1].logical_filament == 6);
    CHECK(wizard_body_slot_choice(7, picker.fine, picker.coarse, 8) == 7);
    for (const int slot : {4, 6, 0, 9})
        CHECK(wizard_body_slot_choice(slot, picker.fine, picker.coarse, 8) == 0);

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

TEST_CASE("A material composes the other nozzle's values when the two profiles lay out their variants differently",
          "[TestRebuild][BugGuard]")
{
    // The H2D 0.4 and 0.6 material profiles carry a third variant column (E3D High Flow); the 0.2 and
    // 0.8 ones have two. Slot 1 is the 0.4 eSUN PLA+ profile on the 0.2 left nozzle, slot 2 the 0.2 PLA
    // Basic profile on the 0.6 right nozzle (the spec's map), both on Standard flow.
    ProfileSpec spec = h2d_02_06_spec();
    spec.materials = {"eSUN PLA+ @BBL H2D", "Bambu PLA Basic @BBL H2D 0.2 nozzle"};
    spec.also_visible_materials = {"eSUN PLA+ @BBL H2D 0.2 nozzle", "Bambu PLA Basic @BBL H2D 0.6 nozzle"};
    WizardProject project(spec);
    const auto variants = [&](const std::string &name) {
        const Preset *found = project.bundle.filaments.find_preset(name, false, true);
        INFO(name);
        REQUIRE(found != nullptr);
        return found->config.option<ConfigOptionStrings>("filament_extruder_variant")->values;
    };
    REQUIRE(variants("eSUN PLA+ @BBL H2D").size() == 3);
    REQUIRE(variants("eSUN PLA+ @BBL H2D 0.2 nozzle").size() == 2);
    REQUIRE(variants("Bambu PLA Basic @BBL H2D 0.2 nozzle").size() == 2);
    REQUIRE(variants("Bambu PLA Basic @BBL H2D 0.6 nozzle").size() == 3);

    const auto speeds = [](const DynamicPrintConfig &config) {
        const auto *speed = config.option<ConfigOptionFloats>("filament_max_volumetric_speed");
        REQUIRE(speed != nullptr);
        return speed->values;
    };
    // eSUN PLA+ on a 0.2 nozzle: 1.8 (the 0.4 profile says 16). PLA Basic on a 0.6 Standard: 30 (0.2: 2).
    const std::vector<double> speed = speeds(project.bundle.full_config());
    REQUIRE(speed.size() == 2);
    CHECK_THAT(speed[0], Catch::Matchers::WithinAbs(1.8, 1e-9));
    CHECK_THAT(speed[1], Catch::Matchers::WithinAbs(30., 1e-9));

    // A plate map that differs from the project map (slot 6 on the 0.4 nozzle): every composition the
    // slice, the saved project and setup read gives the same values, and no rebind is offered.
    CoverFixture cover;
    const std::vector<int> plate_map = cover.current_plate.get_real_filament_maps(cover.bundle.project_config);
    REQUIRE(plate_map.size() == 8);
    REQUIRE(plate_map[5] == 2);
    const std::vector<double> composed = speeds(cover.bundle.full_config(true, plate_map));
    CHECK(speeds(wizard_plate_effective_config(cover.bundle, cover.current_plate)) == composed);
    CHECK_FALSE(cover.bundle.mixed_nozzle_rebind_plan(plate_map,
        cover.current_plate.get_real_filament_volume_maps(cover.bundle.project_config)).offered);
}

namespace {
// The app's C locale follows its language; Windows picks that from the regional format. Switches to
// the first comma-decimal locale this machine has for the life of the object.
struct CommaDecimalLocale
{
    std::string previous = std::setlocale(LC_ALL, nullptr);
    const char *name = nullptr;
    CommaDecimalLocale()
    {
        for (const char *candidate : {"de_DE.UTF-8", "de_DE", "German_Germany.1252", "de-DE", "fr_FR.UTF-8", "fr_FR"})
            if (std::setlocale(LC_ALL, candidate) != nullptr) {
                char text[8] = "";
                std::snprintf(text, sizeof(text), "%.1f", 0.5);
                if (text[1] == ',') {
                    name = candidate;
                    return;
                }
            }
        std::setlocale(LC_ALL, previous.c_str());
    }
    ~CommaDecimalLocale() { std::setlocale(LC_ALL, previous.c_str()); }
};
} // namespace

TEST_CASE("Setup finds each nozzle's material profile and the MN presets in a comma-decimal language",
          "[TestRebuild][BugGuard][Locale]")
{
    CoverFixture fixture;
    const std::vector<int> plate_map = fixture.current_plate.get_real_filament_maps(fixture.bundle.project_config);
    REQUIRE(plate_map.size() == 8);
    const std::vector<WizardProcessPresetRow> rows{
        {"MN Body 0.4-0.2 Standard 0.10-0.20 @BBL", "", 0.10, 0.20, true},
        {"MN Body 0.2-0.6 Standard 0.12-0.36 @BBL", "", 0.12, 0.36, true},
    };
    const CommaDecimalLocale comma;
    if (comma.name == nullptr)
        SKIP("no comma-decimal locale installed");
    CAPTURE(comma.name);
    const DynamicPrintConfig composed = fixture.bundle.full_config(true, plate_map);
    const auto *values = composed.option<ConfigOptionStrings>("mixed_nozzle_filament_binding");
    REQUIRE(values != nullptr);
    REQUIRE(values->values.size() > 5);
    // Each slot is bound to its own nozzle's profile, as in English.
    CHECK(values->values[5] == "base:Bambu PETG HF @BBL H2D 0.4 nozzle");
    CHECK(values->values[3] == "base:Bambu PETG Basic @BBL H2D 0.2 nozzle");
    // The MN Body preset for the pair is found from its name.
    const auto choice = wizard_body_process_choice(rows, "MN Feature 0.2-0.4 0.08-0.24 Candidate @BBL", "",
                                                   {0.2, 0.4}, 0.10, 0.20);
    CHECK(choice.preset == "MN Body 0.4-0.2 Standard 0.10-0.20 @BBL");
    CHECK_FALSE(choice.no_body_preset_for_pair);
}

// ------------------------------------------------------------------------------------------------
// File > New
// ------------------------------------------------------------------------------------------------

TEST_CASE("File > New starts mixed-nozzle slicing Off and keeps the installed pair",
          "[TestRebuild][BugGuard][NewProject]")
{
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
