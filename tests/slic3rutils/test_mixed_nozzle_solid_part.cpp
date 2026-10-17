#include <catch2/catch_all.hpp>

#include "wizard_apply_support.hpp"
#include "slic3r/GUI/MixedNozzleRankingController.hpp"

#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::GUI;
using namespace Slic3r::GUI::WizardTest;

namespace {

// The Windows tester's 2026-10-12 case: a fresh H2D project, 0.4 left and 0.8 right, two Generic ABS
// slots, material 2 fine and material 1 coarse, fine 0.20 with the 0.40 coarse row, and the object
// set to 100% sparse infill. Two touching parts so Body Split has a part for each nozzle.
ProfileSpec solid_0408_spec()
{
    ProfileSpec spec;
    spec.vendor = "BBL";
    spec.printer = "Bambu Lab H2D 0.4 nozzle";
    spec.process = "0.20mm Balanced Strength @BBL H2D";
    spec.materials = {"Generic ABS @BBL H2D", "Generic ABS @BBL H2D"};
    spec.nozzles = {0.4, 0.8};
    spec.min_heights = {0.08, 0.16};
    spec.max_heights = {0.28, 0.56};
    spec.tower = Vec2d(260., 120.);
    return spec;
}

void add_solid_block(WizardProject &project)
{
    // ABS does not print on the Cool Plate the bare bundle starts with.
    project.bundle.project_config.set_deserialize_strict("curr_bed_type", "Textured PEI Plate");
    project.plate.config()->set_deserialize_strict("curr_bed_type", "Textured PEI Plate");
    ModelObject *object = project.add_object("solid block",
        {{"base", Vec3d(40., 40., 8.), Vec3d::Zero(), 1}, {"lid", Vec3d(40., 40., 6.), Vec3d(0., 0., 7.), 1}},
        Vec2d(120., 120.));
    // Object level, as the object list's Strength setting writes it; setup's process preset does not
    // reset it.
    object->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(100));
}

SetupChoice solid_choice(MixedNozzleSlicingMode mode)
{
    SetupChoice choice;
    choice.mode = mode;
    choice.fine_slot = 1;
    choice.coarse_slot = 0;
    choice.fine_height = 0.20;
    choice.ratio = 2;
    choice.scope = WizardScopeKind::ProjectDefault;
    return choice;
}

MixedNozzleSliceTime time_of(const MixedNozzleRankingSlice &slice)
{
    MixedNozzleRankingCancel cancel;
    return mixed_nozzle_slice_time(*slice.model, slice.config, slice.is_bbl_printer, slice.plate_origin, {}, cancel,
                                   slice.plate_index);
}

// Step 3's page for the stage's row, with the row's and the one-nozzle estimates as the dialog has them.
WizardCadencePage page_for(WizardProject &project, const WizardSetupStage &stage, const WizardEstimate &row,
                           const WizardEstimate &baseline, std::optional<std::size_t> &selected)
{
    FullPrintConfig effective;
    effective.apply(wizard_plate_effective_config(project.bundle, project.plate), true);
    effective.filament_map_mode.value = fmmManual;
    effective.filament_map.values = stage.draft.derived_filament_map;
    std::vector<WizardCandidate> candidates = build_wizard_candidates(stage.draft, effective, {});
    for (WizardCandidate &candidate : candidates)
        if (candidate.ratio == stage.candidate.ratio && candidate.coarse_height)
            candidate.estimate = row;
    WizardCadencePage page = wizard_cadence_page(candidates, 0.20, effective, baseline);
    selected = wizard_ranked_selection(page, std::nullopt, false);
    return page;
}

bool has_text(const std::string &text, const std::string &part) { return text.find(part) != std::string::npos; }

} // namespace

TEST_CASE("Feature Split on a part at 100% density says the coarse nozzle has no sparse infill",
          "[TestRebuild][BugGuard][SolidPart]")
{
    // With no sparse infill the 0.8 prints nothing. Step 3 said "To use both nozzles, raise the sparse
    // infill density.", which cannot be done from 100%.
    WizardProject project(solid_0408_spec());
    add_solid_block(project);
    const WizardSetupStage stage = build_wizard_setup_stage(project, solid_choice(MixedNozzleSlicingMode::FeatureSplit));
    const MixedNozzleRankingSlice row = wizard_candidate_staged_slice(stage.draft, stage.candidate, stage.body,
                                                                      project.owners);
    REQUIRE(row.diagnostic.empty());
    REQUIRE(row.model);
    const MixedNozzleSliceTime row_time = time_of(row);
    INFO(row_time.diagnostic);
    REQUIRE(row_time.status == MixedNozzleSliceTimeStatus::Estimated);
    CHECK(row_time.switches == 0);
    CHECK_FALSE(row_time.sparse_infill);
    CHECK(row_time.all_solid);
    const MixedNozzleRankingSlice one = wizard_baseline_ranking_slice(row, stage.draft.fine_logical_filament, 0.20);
    REQUIRE(one.diagnostic.empty());
    const WizardEstimate baseline = wizard_ranking_estimate(time_of(one));
    REQUIRE(baseline.status == WizardEstimateStatus::Estimated);

    std::optional<std::size_t> selected;
    const WizardCadencePage page = page_for(project, stage, wizard_ranking_estimate(row_time), baseline, selected);
    REQUIRE(selected);
    const std::string line = wizard_speed_summary_line(page, 0.20, selected, false);
    CHECK(has_text(line, "The 0.8 mm nozzle has nothing to print on this plate. Feature Split gives it the sparse "
                         "infill, and at 100% density there is none (the inside is all solid infill, which the "
                         "0.4 mm nozzle prints). To use both nozzles, lower the sparse infill density below 100%. "
                         "Or choose Print with one nozzle instead."));
    CHECK_FALSE(has_text(line, "This row prints on"));
    CHECK_FALSE(has_text(line, "raise the sparse infill density"));
    CHECK(wizard_offer_one_nozzle(page, selected));
    CHECK(line.find("\xE2\x80\x94") == std::string::npos);
}

TEST_CASE("A Body Split row on a part at 100% density says why Slice refuses it, and one nozzle still gets a time",
          "[TestRebuild][BugGuard][SolidPart]")
{
    // Body Split cannot print 100% density (SRL-A14). Step 3 said only "no time: Slice would refuse
    // this row." on the row and on the one-nozzle line, though one nozzle prints the part fine.
    WizardProject project(solid_0408_spec());
    add_solid_block(project);
    const WizardSetupStage stage = build_wizard_setup_stage(project, solid_choice(MixedNozzleSlicingMode::BodySplit));
    const MixedNozzleRankingSlice row = wizard_candidate_staged_slice(stage.draft, stage.candidate, stage.body,
                                                                      project.owners);
    CHECK(row.diagnostic.rfind("Slice would refuse this row. ", 0) == 0);
    CHECK(has_text(row.diagnostic, "Body Split needs a sparse infill density above 0% and below 100%."));

    const MixedNozzleRankingSlice one = wizard_baseline_ranking_slice(row, stage.draft.fine_logical_filament, 0.20);
    CHECK(one.diagnostic.empty());
    REQUIRE(one.model);
    const MixedNozzleSliceTime one_time = time_of(one);
    INFO(one_time.diagnostic);
    const WizardEstimate baseline = wizard_ranking_estimate(one_time);
    REQUIRE(baseline.status == WizardEstimateStatus::Estimated);
    CHECK(baseline.switches == 0);

    // The page: the row has no time and says why; the one-nozzle line has its time.
    WizardEstimate refused;
    refused.status = WizardEstimateStatus::Unavailable;
    refused.note = row.diagnostic;
    std::optional<std::size_t> selected;
    const WizardCadencePage page = page_for(project, stage, refused, baseline, selected);
    REQUIRE(page.rows.size() == 1);
    CHECK(has_text(wizard_speed_row_label(page.rows[0], MixedNozzleSlicingMode::BodySplit),
                   "no time: Slice would refuse this row. solid block: Body Split cannot slice this as set up. "
                   "Body Split needs a sparse infill density above 0% and below 100%."));
    const std::string line = wizard_speed_summary_line(page, 0.20, selected, false);
    CHECK(line.rfind("One nozzle only, 0.2 mm everywhere: about ", 0) == 0);
    CHECK_FALSE(has_text(line, "refuse"));

    // A row setup itself refuses has nothing to print on one nozzle either, and keeps its reason.
    MixedNozzleRankingSlice setup_refused;
    setup_refused.diagnostic = "Setup would refuse this row. Pick a fine and a coarse material in step 2.";
    CHECK(wizard_baseline_ranking_slice(setup_refused, std::size_t(1), 0.20).diagnostic == setup_refused.diagnostic);
}

TEST_CASE("Step 4 says plainly when setup's process preset lowers the process density and wall loops",
          "[TestRebuild][E2E][SolidPart][StrengthLine]")
{
    // The tester typed 100% into the sidebar: a process value. Setup switches to the MN Feature
    // preset and its 15% replaces it (5 walls become 2). Step 4 said so only inside the collapsed
    // "Every setting this changes". Object and part values are kept, so they are not named.
    WizardProject project(solid_0408_spec());
    project.bundle.project_config.set_deserialize_strict("curr_bed_type", "Textured PEI Plate");
    project.plate.config()->set_deserialize_strict("curr_bed_type", "Textured PEI Plate");
    project.add_object("solid block",
        {{"base", Vec3d(40., 40., 8.), Vec3d::Zero(), 1}, {"lid", Vec3d(40., 40., 6.), Vec3d(0., 0., 7.), 1}},
        Vec2d(120., 120.));
    DynamicPrintConfig &process = project.bundle.prints.get_edited_preset().config;
    process.set_key_value("sparse_infill_density", new ConfigOptionPercent(100));
    process.set_key_value("wall_loops", new ConfigOptionInt(5));

    SECTION("setup's own preset")
    {
        const WizardSetupStage stage = build_wizard_setup_stage(project, solid_choice(MixedNozzleSlicingMode::FeatureSplit));
        const PreparedWizardApply prepared = prepare_wizard_apply(stage.draft, stage.review, project.owners);
        REQUIRE(prepared.diagnostic.empty());
        INFO(prepared.setup_plan.after.process_name << ": " << prepared.strength_line);
        REQUIRE(prepared.setup_plan.after.process_name.rfind("MN Feature 0.4-0.8", 0) == 0);
        // The MN preset's thinner layers come with more shell layers (6 to 9 top, 4 to 7 bottom), which
        // is not less strength, so they are not named.
        CHECK(prepared.strength_line ==
              "This changes Sparse infill density from 100% to 15% and Wall loops from 5 to 2. To keep yours, "
              "choose \"Keep my current process preset instead\" in More options.");
        CHECK(prepared.strength_line.find("\xE2\x80\x94") == std::string::npos);

        // Shown on its own, not only in the collapsed list.
        WizardReviewSummaryInput input;
        input.mode = MixedNozzleSlicingMode::FeatureSplit;
        input.nozzle_diameters = {0.4, 0.8};
        input.fine_height = 0.20;
        input.strength_line = prepared.strength_line;
        CHECK(wizard_review_summary(input).strength_line == prepared.strength_line);
    }
    SECTION("Keep my current process preset instead")
    {
        SetupChoice choice = solid_choice(MixedNozzleSlicingMode::FeatureSplit);
        choice.keep_current_process = true;
        const WizardSetupStage stage = build_wizard_setup_stage(project, choice);
        const PreparedWizardApply prepared = prepare_wizard_apply(stage.draft, stage.review, project.owners);
        REQUIRE(prepared.diagnostic.empty());
        CHECK(prepared.strength_line.empty());
        CHECK(prepared.setup_plan.after.process_config.opt_int("wall_loops") == 5);
    }
}

TEST_CASE("The strength line names each changed value once, top and bottom shells together",
          "[TestRebuild][StrengthLine]")
{
    const std::vector<WizardKeyDelta> deltas{
        {"sparse_infill_density", "100%", "15%", {}, {}, {}},
        {"wall_loops", "2", "2", {}, {}, {}},
        {"top_shell_layers", "6", "4", {}, {}, {}},
        {"bottom_shell_layers", "6", "4", {}, {}, {}},
        {"layer_height", "0.2", "0.1", {}, {}, {}}};
    CHECK(wizard_strength_change_line(deltas, false) ==
          "This changes Sparse infill density from 100% to 15% and Top and bottom shell layers from 6 to 4.");
    CHECK(wizard_strength_change_line({{"layer_height", "0.2", "0.1", {}, {}, {}}}, true).empty());
    // More shell layers than before is not named.
    CHECK(wizard_strength_change_line({{"top_shell_layers", "6", "9", {}, {}, {}},
                                       {"bottom_shell_layers", "4", "3", {}, {}, {}}}, false) ==
          "This changes Bottom shell layers from 4 to 3.");
}
