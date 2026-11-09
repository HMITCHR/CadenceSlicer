#include <catch2/catch_all.hpp>

#include "fff_print/mixed_nozzle_harness.hpp"
#include "wizard_apply_support.hpp"
#include "libslic3r/MixedNozzleBinding.hpp"
#include "libslic3r/MixedNozzleRanking.hpp"
#include "libslic3r/TriangleSelector.hpp"
#include "slic3r/GUI/MixedNozzleRankingController.hpp"

#include <clocale>
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

SetupChoice rank_choice(MixedNozzleSlicingMode mode, int ratio)
{
    SetupChoice choice;
    choice.mode = mode;
    choice.fine_height = 0.10;
    choice.ratio = ratio;
    if (mode == MixedNozzleSlicingMode::BodySplit)
        choice.joining = JoiningChoice::Off;
    return choice;
}

MixedNozzleSliceTime time_of(const MixedNozzleRankingSlice &slice)
{
    MixedNozzleRankingCancel cancel;
    return mixed_nozzle_slice_time(*slice.model, slice.config, slice.is_bbl_printer, slice.plate_origin, {}, cancel,
                                   slice.plate_index);
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
    const WizardApplyResult applied = run_wizard_setup(project, rank_choice(mode, ratio));
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

        // A binding coarse material speed cap slows the estimate and gives the slice another ranking key.
        const RankedPrint capped = ranked_print(mode, 3, 0.8);
        CHECK(capped.time.seconds > long_cadence.time.seconds);
        CHECK(capped.time.switches == long_cadence.time.switches);
        CHECK(capped.time.tower_mm3 > 0.);
        CHECK(mixed_nozzle_ranking_key(0, *capped.model, capped.config) !=
              mixed_nozzle_ranking_key(0, *long_cadence.model, long_cadence.config));
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

    // An empty plate is refused before the export reads its first object, for the row and the one-nozzle baseline.
    for (const bool baseline : {false, true}) {
        CAPTURE(baseline);
        Model empty(*short_cadence.model);
        empty.clear_objects();
        DynamicPrintConfig config = short_cadence.config;
        if (baseline)
            mixed_nozzle_single_nozzle_baseline(empty, config, 1, 0.10);
        MixedNozzleRankingCancel cancel;
        CHECK(mixed_nozzle_slice_time(empty, config, true, Vec3d::Zero(), {}, cancel).status ==
              MixedNozzleSliceTimeStatus::Refused);
    }

    // The same object at 100% density: Feature Split leaves the coarse nozzle nothing (no change, all solid), Body
    // Split refuses the row, and one nozzle only still gets a time.
    WizardProject solid(h2d_02_06_spec());
    add_rank_object(solid, mode);
    solid.model.objects.back()->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(100));
    const WizardSetupStage stage = build_wizard_setup_stage(solid, rank_choice(mode, 2));
    const MixedNozzleRankingSlice solid_row = wizard_candidate_staged_slice(stage.draft, stage.candidate, stage.body,
                                                                            solid.owners);
    if (mode == MixedNozzleSlicingMode::FeatureSplit) {
        INFO(solid_row.diagnostic);
        REQUIRE(solid_row.diagnostic.empty());
        REQUIRE(solid_row.model);
        const MixedNozzleSliceTime solid_time = time_of(solid_row);
        INFO(solid_time.diagnostic);
        REQUIRE(solid_time.status == MixedNozzleSliceTimeStatus::Estimated);
        CHECK(solid_time.switches == 0);
        CHECK(solid_time.all_solid);
    } else
        CHECK_FALSE(solid_row.diagnostic.empty());
    const MixedNozzleRankingSlice one =
        wizard_baseline_ranking_slice(solid_row, stage.draft.fine_logical_filament, 0.10);
    INFO(one.diagnostic);
    REQUIRE(one.diagnostic.empty());
    REQUIRE(one.model);
    const MixedNozzleSliceTime one_time = time_of(one);
    CHECK(one_time.status == MixedNozzleSliceTimeStatus::Estimated);
    CHECK(one_time.switches == 0);
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

namespace {

// The C locale a Turkish regional format gives the app, for the life of the object. Stays "C" when
// this machine has no Turkish locale.
struct TurkishLocale
{
    std::string previous = std::setlocale(LC_ALL, nullptr);
    TurkishLocale()
    {
        for (const char *name : {"tr_TR.UTF-8", "tr_TR", "Turkish_Turkey.1254", "tr-TR"})
            if (std::setlocale(LC_ALL, name) != nullptr)
                return;
    }
    ~TurkishLocale() { std::setlocale(LC_ALL, previous.c_str()); }
};

} // namespace

TEST_CASE("Feature Split setup on 0.4 / 0.8 applies when the project's layer limits do not fit an MN preset",
          "[TestRebuild][BugGuard][ProcessBasis]")
{
    // A 0.4 / 0.8 project whose 0.4 nozzle carries the 0.8 profile's 0.16 minimum (stale) does not fit any
    // MN preset, so setup keeps the current process. The plate is locked, which only keeps it out of
    // Arrange, and the process has 5 wall loops.
    struct Variant {
        const char *start;
        bool stale;
        bool turkish;
        bool keep_process;
    };
    const Variant variant = GENERATE(Variant{"0.4", false, false, false}, Variant{"0.8", false, true, false},
                                     Variant{"0.4", true, false, false}, Variant{"0.8", true, true, false},
                                     Variant{"0.4", false, false, true});
    const std::string start = variant.start;
    CAPTURE(start, variant.stale, variant.turkish, variant.keep_process);
    std::optional<TurkishLocale> locale;
    if (variant.turkish)
        locale.emplace();
    ProfileSpec spec;
    spec.vendor = "BBL";
    spec.printer = start == "0.4" ? "Bambu Lab H2D 0.4 nozzle" : "Bambu Lab H2D 0.8 nozzle";
    spec.process = start == "0.4" ? "0.20mm Standard @BBL H2D" : "0.40mm Standard @BBL H2D 0.8 nozzle";
    const std::string m4 = "Bambu PLA Basic @BBL H2D", m8 = "Bambu PLA Basic @BBL H2D 0.8 nozzle";
    spec.materials = start == "0.4" ? std::vector<std::string>{m4, m4} : std::vector<std::string>{m8, m8};
    spec.also_visible_materials = {start == "0.4" ? m8 : m4};
    spec.nozzles = {0.4, 0.8};
    spec.min_heights = variant.stale ? std::vector<double>{0.16, 0.16} : std::vector<double>{0.08, 0.16};
    spec.max_heights = {0.28, 0.56};
    spec.tower = Vec2d(260., 120.);
    WizardProject project(spec);
    add_rank_object(project, MixedNozzleSlicingMode::FeatureSplit);
    project.plate.lock(true);
    project.bundle.prints.get_edited_preset().config.set_key_value("wall_loops", new ConfigOptionInt(5));
    SetupChoice choice;
    choice.mode = MixedNozzleSlicingMode::FeatureSplit;
    choice.fine_slot = 1;
    choice.coarse_slot = 0;
    choice.fine_height = 0.20;
    choice.ratio = 2;
    choice.scope = WizardScopeKind::ProjectDefault;
    choice.keep_current_process = variant.keep_process;

    // Step 3's row slices, and Apply goes through.
    const WizardSetupStage stage = build_wizard_setup_stage(project, choice);
    const MixedNozzleRankingSlice row = wizard_candidate_staged_slice(stage.draft, stage.candidate, stage.body,
                                                                      project.owners);
    CHECK(row.diagnostic.empty());
    CHECK(row.model != nullptr);
    REQUIRE(run_wizard_setup(project, choice).applied);
    CHECK(project.plate.is_locked());
    const auto pair = project.plate.get_effective_feature_split_filaments(project.bundle.project_config, 2);
    REQUIRE(pair);
    CHECK(pair->first == 2);
    CHECK(pair->second == 1);
    const Preset &process = project.bundle.prints.get_edited_preset();
    CAPTURE(process.name);
    if (variant.keep_process)
        CHECK(process.config.opt_int("wall_loops") == 5);
    else if (variant.stale)
        CHECK(process.name == spec.process);
    else {
        // Setup's MN preset replaces the project's wall loops with its own.
        CHECK(process.name.rfind("MN Feature 0.4-0.8 ", 0) == 0);
        CHECK(process.config.opt_int("wall_loops") < 5);
    }
}

namespace {

// A two-color plate on a printer with two fine tools and one coarse tool (filament i on tool i), Feature Split
// with the sparse infill on the coarse tool's filament, as a ranking row hands it to the baseline.
MixedNozzleRankingSlice two_color_tool_changer_row(const std::vector<double> &pair)
{
    MixedNozzleRankingSlice row;
    row.model = std::make_shared<Model>();
    ModelObject *object = row.model->add_object();
    for (int filament : {1, 2}) {
        ModelVolume *volume = object->add_volume(make_cube(10., 10., 5.), ModelVolumeType::MODEL_PART, false);
        volume->set_offset(Vec3d(12. * filament, 0., 0.));
        volume->config.set_key_value("extruder", new ConfigOptionInt(filament));
    }
    object->add_instance();
    row.config = DynamicPrintConfig::full_print_config();
    row.config.set_key_value("nozzle_diameter", new ConfigOptionFloats{pair[0], pair[0], pair[1]});
    row.config.set_key_value("filament_map", new ConfigOptionInts{1, 2, 3});
    row.config.set_key_value("filament_colour", new ConfigOptionStrings{"#C0C0C0", "#20A040", "#C0C0C0"});
    row.config.set_key_value("sparse_infill_filament_id", new ConfigOptionInt(3));
    row.config.set_key_value("mixed_nozzle_slicing_mode",
                             new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::FeatureSplit));
    return row;
}

} // namespace

// Colors are the parts' slots and the painted colors; the coarse tool's infill slot is not a model color. More
// colors than tools on a printer that prints each material on its own tool asks first, and the comparison
// keeps each color on its own tool unless that setting is off.
TEST_CASE("Setup counts the plate's colors, asks when there are more than tools, and compares with them kept",
          "[TestRebuild][Ranking][ColorsTools]")
{
    const auto pair = GENERATE(std::vector<double>{.2, .4}, std::vector<double>{.2, .6}, std::vector<double>{.4, .8});
    CAPTURE(pair);
    MixedNozzleRankingSlice row = two_color_tool_changer_row(pair);
    std::vector<ModelObject *> objects(row.model->objects.begin(), row.model->objects.end());
    const std::size_t tools = row.config.option<ConfigOptionFloats>("nozzle_diameter")->values.size();
    CHECK(mixed_nozzle_model_color_slots(objects) == std::vector<std::size_t>{0, 1});
    CHECK(wizard_colors_tools_notice(2, tools, true).empty());

    // The one-nozzle comparison: each color on its own tool by default, everything on the fine one when off.
    const double fine = pair[0] < .3 ? .08 : .12;
    const MixedNozzleRankingSlice kept =
        wizard_baseline_ranking_slice(two_color_tool_changer_row(pair), std::size_t(0), fine);
    REQUIRE(kept.model);
    CHECK(kept.config.option<ConfigOptionInts>("filament_map")->values == std::vector<int>{1, 2, 3});
    CHECK(kept.model->objects.front()->volumes[1]->config.opt_int("extruder") == 2);
    const MixedNozzleRankingSlice plain =
        wizard_baseline_ranking_slice(two_color_tool_changer_row(pair), std::size_t(0), fine, false);
    REQUIRE(plain.model);
    CHECK(plain.config.option<ConfigOptionInts>("filament_map")->values == std::vector<int>{1, 1, 1});
    CHECK(plain.model->objects.front()->volumes[1]->config.opt_int("extruder") == 1);

    // Paint colors 4 and 5 on the first part: four colors on three tools.
    ModelVolume *part = objects.front()->volumes.front();
    TriangleSelector selector(part->mesh());
    selector.set_facet(0, EnforcerBlockerType(4));
    selector.set_facet(1, EnforcerBlockerType(5));
    REQUIRE(part->mmu_segmentation_facets.set(selector));
    const std::vector<std::size_t> colors = mixed_nozzle_model_color_slots(objects);
    CHECK(colors == std::vector<std::size_t>{0, 1, 3, 4});
    CHECK_FALSE(wizard_colors_tools_notice(colors.size(), tools, true).empty());
    // A printer whose nozzles each take several materials (two-nozzle Bambu printers) has nothing to ask.
    CHECK(wizard_colors_tools_notice(colors.size(), 2, false).empty());
}
