// Setup never applies over paint that changed after it opened.

#include <catch2/catch_all.hpp>

#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleSelector.hpp"

#include "wizard_apply_support.hpp"

#include <string>

using namespace Slic3r;
using namespace Slic3r::GUI;
using namespace Slic3r::GUI::WizardTest;

TEST_CASE("A paint edit after setup opened makes its Apply stale, as a settings edit does",
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
    run_wizard_setup(project, choice);

    // Setup opened again: its draft records each part as it is now.
    const WizardDraft draft = wizard_entry_draft(project);
    const std::string stale = "Mixed-Nozzle wizard model target is stale.";
    REQUIRE(prepare_wizard_apply(draft, project.owners).diagnostic != stale);

    ModelVolume *part = project.model.objects.front()->volumes[1];
    SECTION("settings edit")
    {
        part->config.set_key_value("extruder", new ConfigOptionInt(1));
    }
    SECTION("paint edit")
    {
        TriangleSelector selector(part->mesh());
        selector.set_facet(0, EnforcerBlockerType::Extruder1);
        REQUIRE(part->mmu_segmentation_facets.set(selector));
    }
    CHECK(prepare_wizard_apply(draft, project.owners).diagnostic == stale);
}
