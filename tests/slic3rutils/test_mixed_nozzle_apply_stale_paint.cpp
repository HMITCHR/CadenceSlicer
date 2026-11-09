// Setup never applies over a project that changed after it opened, nor a malformed change.

#include <catch2/catch_all.hpp>

#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleSelector.hpp"

#include "wizard_apply_support.hpp"

#include <string>

using namespace Slic3r;
using namespace Slic3r::GUI;
using namespace Slic3r::GUI::WizardTest;

TEST_CASE("Setup Apply after a paint or settings edit, or with a malformed change, writes nothing",
          "[TestRebuild][B35]")
{
    WizardProject project(h2d_02_06_spec());
    project.add_object("two bodies", {{"left body", Vec3d(10., 20., 10.), Vec3d::Zero(), 1},
                                      {"right body", Vec3d(10., 20., 10.), Vec3d(15., 0., 0.), 2}},
                       Vec2d(100., 100.));
    SetupChoice choice;
    choice.mode = MixedNozzleSlicingMode::BodySplit;
    choice.fine_height = 0.10;
    choice.ratio = 3;
    // Setup open on Review.
    WizardSetupStage stage = build_wizard_setup_stage(project, choice);
    const DynamicPrintConfig process_before = project.bundle.prints.get_edited_preset().config;

    ModelVolume *part = project.model.objects.front()->volumes[1];
    bool refused = true;
    SECTION("no edit") { refused = false; }
    SECTION("settings edit") { part->config.set_key_value("extruder", new ConfigOptionInt(1)); }
    SECTION("paint edit")
    {
        TriangleSelector selector(part->mesh());
        selector.set_facet(0, EnforcerBlockerType::Extruder1);
        REQUIRE(part->mmu_segmentation_facets.set(selector));
    }
    SECTION("a process change that is not a layer height")
    {
        const std::string current = process_before.opt_serialize("layer_height");
        stage.review.entries.push_back({"layer_height", current, "not-a-layer-height", "Resolver", "Shared process", "mm"});
    }

    PreparedWizardApply prepared = prepare_wizard_apply(stage.draft, stage.review, project.owners);
    CHECK(prepared.diagnostic.empty() != refused);
    HookLog log;
    const WizardApplyResult result = commit_wizard_apply(std::move(prepared), project.owners, hooks_for(log));
    CHECK(result.applied != refused);
    if (refused) {
        CHECK(log.snapshots == 0);
        CHECK(log.notifications == 0);
        CHECK(project.bundle.prints.get_edited_preset().config == process_before);
        CHECK(project.plate.get_real_mixed_nozzle_slicing_mode(project.bundle.project_config) ==
              MixedNozzleSlicingMode::Off);
    } else {
        CHECK(log.notifications == 1);
        CHECK(project.plate.get_real_mixed_nozzle_slicing_mode(project.bundle.project_config) ==
              MixedNozzleSlicingMode::BodySplit);
    }
}
