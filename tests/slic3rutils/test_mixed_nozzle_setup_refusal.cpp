#include <catch2/catch_all.hpp>

#include "wizard_apply_support.hpp"

#include <algorithm>
#include <clocale>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::GUI;
using namespace Slic3r::GUI::WizardTest;

namespace {

// The Windows tester's v1.1 setup: H2D, 0.4 left and 0.8 right, two Generic ABS slots, material 2
// fine and material 1 coarse, fine 0.20 with the 0.40 coarse row.
ProfileSpec abs_0408_spec(const std::string &start)
{
    ProfileSpec spec;
    spec.vendor = "BBL";
    spec.printer = start == "0.4" ? "Bambu Lab H2D 0.4 nozzle" : "Bambu Lab H2D 0.8 nozzle";
    spec.process = start == "0.4" ? "0.20mm Balanced Strength @BBL H2D" : "0.40mm Standard @BBL H2D 0.8 nozzle";
    spec.materials = {"Generic ABS @BBL H2D", "Generic ABS @BBL H2D"};
    spec.nozzles = {0.4, 0.8};
    spec.min_heights = {0.08, 0.16};
    spec.max_heights = {0.28, 0.56};
    spec.tower = Vec2d(260., 120.);
    return spec;
}

SetupChoice abs_0408_choice()
{
    SetupChoice choice;
    choice.mode = MixedNozzleSlicingMode::FeatureSplit;
    choice.fine_slot = 1;
    choice.coarse_slot = 0;
    choice.fine_height = 0.20;
    choice.ratio = 2;
    choice.scope = WizardScopeKind::ProjectDefault;
    return choice;
}

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

TEST_CASE("Feature Split setup on 0.4 / 0.8 applies to a project with a locked plate",
          "[TestRebuild][BugGuard][LockedPlate]")
{
    // A locked plate only stays out of Arrange. Setup refused it ("affected plate is locked"), so
    // every step 3 row said "Setup would refuse this row." and step 4 could not apply.
    const std::string start = GENERATE(std::string("0.4"), std::string("0.8"));
    const bool turkish = GENERATE(false, true);
    CAPTURE(start, turkish);
    std::optional<TurkishLocale> locale;
    if (turkish)
        locale.emplace();
    WizardProject project(abs_0408_spec(start));
    project.add_object("large part", {{"part", Vec3d(85., 85., 16.), Vec3d::Zero(), 1}}, Vec2d(55., 55.));
    project.plate.lock(true);

    const WizardSetupStage stage = build_wizard_setup_stage(project, abs_0408_choice());
    const MixedNozzleRankingSlice row = wizard_candidate_staged_slice(stage.draft, stage.candidate, stage.body,
                                                                      project.owners);
    CHECK(row.diagnostic.empty());
    CHECK(row.model != nullptr);

    CHECK(run_wizard_setup(project, abs_0408_choice()).applied);
    CHECK(project.plate.is_locked());
    CHECK(project.bundle.prints.get_edited_preset().name.rfind("MN Feature 0.4-0.8 ", 0) == 0);
    const auto pair = project.plate.get_effective_feature_split_filaments(project.bundle.project_config, 2);
    REQUIRE(pair);
    CHECK(pair->first == 2);
    CHECK(pair->second == 1);
}

TEST_CASE("Setup refusals in step 3 and step 4 say why in plain words",
          "[TestRebuild][BugGuard][SetupRefusal]")
{
    // A check a project can trip has its own words, in the row and in step 4.
    const std::string binding = "Mixed-Nozzle wizard native binding requires an explicit project-wide review.";
    CHECK(wizard_setup_row_refusal(binding) ==
          "Setup would refuse this row. Updating each material's settings for its nozzle has to cover every "
          "plate in the project. Open setup again.");
    CHECK(wizard_setup_diagnostic_message(binding) ==
          "Updating each material's settings for its nozzle has to cover every plate in the project. Open "
          "setup again.");
    const std::string process = "Mixed-Nozzle setup process selection is unavailable.";
    CHECK(wizard_setup_row_refusal(process).rfind("Setup would refuse this row. The process preset setup "
                                                  "picked does not fit", 0) == 0);
    CHECK(wizard_setup_diagnostic_message(process).find("Keep my current process preset instead") != std::string::npos);

    // Any other internal check goes to the log; the user sees a plain sentence without the check name.
    const std::string other = "Mixed-Nozzle wizard process delta is invalid.";
    CHECK(wizard_setup_row_refusal(other) == "Setup would refuse this row.");
    CHECK(wizard_setup_diagnostic_message(other) ==
          "Setup could not finish with these choices. Go back over the materials and layer heights, or open "
          "setup again.");

    // A project that moved under the open wizard still says so, and a plain sentence passes through.
    CHECK(wizard_setup_diagnostic_message("Mixed-Nozzle wizard process delta is stale.") ==
          "Setup could not finish because the project changed while it was open. Open setup again.");
    const std::string plain = "This coarse layer cannot be used. The 0.8 mm nozzle prints at most 0.56 mm.";
    CHECK(wizard_setup_row_refusal(plain) == "Setup would refuse this row. " + plain);
    CHECK(wizard_setup_diagnostic_message(plain) == plain);

    for (const std::string &diagnostic : {binding, process, other, plain})
        for (const std::string &text : {wizard_setup_row_refusal(diagnostic), wizard_setup_diagnostic_message(diagnostic)}) {
            CAPTURE(text);
            CHECK(text.find("Mixed-Nozzle") == std::string::npos);
            CHECK(text.find("process delta") == std::string::npos);
            CHECK(text.find("\xE2\x80\x94") == std::string::npos); // no em dash
        }
}

TEST_CASE("Every setting this changes names each setting by its label, not its config key",
          "[TestRebuild][BugGuard][SetupRefusal]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("flush_into_infill", new ConfigOptionBool(true));
    config.set_key_value("flush_into_objects", new ConfigOptionBool(true));
    config.set_key_value("flush_into_support", new ConfigOptionBool(true));
    config.set_key_value("infill_combination", new ConfigOptionBool(true));
    config.set_key_value("minimum_sparse_infill_area", new ConfigOptionFloat(15.));
    config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipGyroid));

    std::vector<std::string> lines;
    for (const WizardProcessFix &fix : wizard_process_fixes(config, MixedNozzleSlicingMode::BodySplit))
        lines.push_back(fix.line);
    const auto has = [&lines](const std::string &line) {
        return std::find(lines.begin(), lines.end(), line) != lines.end();
    };
    CHECK(has("Flush into objects' infill: on to off"));
    CHECK(has("Flush into this object: on to off"));
    CHECK(has("Flush into objects' support: on to off"));
    CHECK(has("Infill combination: on to off"));
    CHECK(has("Minimum sparse infill threshold: to 0 mm2"));
    CHECK(has("Sparse infill pattern: to Grid (Body Split needs a pattern that is the same on every layer)"));
    for (const std::string &line : lines) {
        CAPTURE(line);
        CHECK(line.find("flush_into") == std::string::npos);
        CHECK(line.find('_') == std::string::npos); // no config key
        CHECK(line.find("\xE2\x80\x94") == std::string::npos); // no em dash
    }
}
