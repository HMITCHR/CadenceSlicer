// The Feature Split time check prices the tower, the nozzle changes and the band infill before the
// print is made, and keeps the coarse bands only when they save time. Its predicted saving has to
// agree with what the exported G-code takes, or it keeps plans that come out slower than printing
// everything on the fine nozzle.

#include <catch2/catch_all.hpp>

#include "wizard_apply_support.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::GUI;
using namespace Slic3r::GUI::WizardTest;

namespace {

// The H2D with a 0.2 mm left and a 0.8 mm right nozzle, the pair the small-print report measured.
ProfileSpec h2d_02_08_spec()
{
    ProfileSpec spec = h2d_02_06_spec();
    spec.nozzles = {0.2, 0.8};
    spec.min_heights = {0.04, 0.16};
    spec.max_heights = {0.14, 0.56};
    spec.also_visible_materials = {"Bambu PLA Basic @BBL H2D 0.6 nozzle", "Bambu PLA Basic @BBL H2D 0.8 nozzle"};
    return spec;
}

struct Sliced {
    double seconds { 0. };
    // The whole-print digest's figures as key=value pairs; empty when no band reached the check.
    std::map<std::string, std::string> digest;
};

Sliced slice_with(WizardProject &project, const DynamicPrintConfig &config)
{
    Sliced out;
    Print print;
    print.set_status_silent();
    print.is_BBL_printer() = project.bundle.is_bbl_vendor();
    print.set_plate_origin(project.plate.get_origin());
    print.apply(project.model, config);
    print.apply(project.model, config);
    const std::string refusal = print.validate().string;
    INFO(refusal);
    REQUIRE(refusal.empty());
    print.process();
    // The first line is the decision; a "whole_print rebuilt" line, when present, prices the chosen
    // plan again on the tower built for it, and its figures replace the first line's.
    std::istringstream lines(print.feature_economics_whole_print_digest());
    bool first = true;
    for (std::string line; std::getline(lines, line); first = false) {
        if (!first && line.rfind("whole_print rebuilt ", 0) != 0)
            continue;
        std::istringstream items(line);
        for (std::string item; items >> item; ) {
            const size_t equals = item.find('=');
            if (equals != std::string::npos)
                out.digest[item.substr(0, equals)] = item.substr(equals + 1);
        }
    }
    ScopedTemporaryFile file(".gcode");
    GCodeProcessorResult result;
    print.export_gcode(file.string(), &result, nullptr);
    out.seconds = result.print_statistics.modes[size_t(PrintEstimatedStatistics::ETimeMode::Normal)].time;
    return out;
}

double digest_value(const Sliced &sliced, const char *key)
{
    const auto found = sliced.digest.find(key);
    return found == sliced.digest.end() ? 0. : std::stod(found->second);
}

// The same plate with every feature on the fine nozzle: Off, with no role sent anywhere else.
DynamicPrintConfig all_fine(DynamicPrintConfig config)
{
    config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::Off));
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id", "internal_solid_filament_id",
                            "top_surface_filament_id", "bottom_surface_filament_id", "sparse_infill_filament_id"})
        config.set_key_value(key, new ConfigOptionInt(0));
    return config;
}

} // namespace

TEST_CASE("The time check's predicted saving agrees with the exported print times",
          "[TestRebuild][Economics][TimeCheck]")
{
    struct Scene {
        const char *name;
        double      side;
        double      height;
        double      density;
        int         ratio;
        double      column; // height of a thin column standing on the block, 0 for none
    };
    // A cube too small to win, one near break-even on the tallest coarse layer, a dense block whose
    // coarse bands clearly pay for the tower, and the same block under a thin column whose bands go
    // fine, so the tower that is built ends far lower and narrower than the one first priced.
    const Scene scene = GENERATE(Scene{"small cube", 20., 10., 15., 3, 0.}, Scene{"break-even cube", 30., 15., 15., 5, 0.},
                                 Scene{"dense block", 50., 8., 40., 3, 0.}, Scene{"block under a column", 50., 8., 40., 3, 62.});
    CAPTURE(scene.name);
    WizardProject project(h2d_02_08_spec());
    std::vector<WizardProject::Part> parts{{"block", Vec3d(scene.side, scene.side, scene.height), Vec3d::Zero(), 1}};
    if (scene.column > 0.)
        parts.push_back({"column", Vec3d(8., 8., scene.column), Vec3d(0.5 * scene.side - 4., 0.5 * scene.side - 4., scene.height), 1});
    ModelObject *object = project.add_object(scene.name, parts, Vec2d(100., 100.));
    object->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(scene.density));
    SetupChoice choice;
    choice.fine_height = 0.10;
    choice.ratio = scene.ratio;
    REQUIRE(run_wizard_setup(project, choice).applied);
    const DynamicPrintConfig config = plate_slice_config(project);

    const Sliced mixed = slice_with(project, config);
    const Sliced fine = slice_with(project, all_fine(config));
    // With no whole-print digest every band went fine on its own numbers, and the check expects
    // the fine print.
    const double predicted = mixed.digest.empty() ? 0. : digest_value(mixed, "all_fine") - digest_value(mixed, "chosen");
    const double real = fine.seconds - mixed.seconds;
    const double tower = digest_value(mixed, "chosen_tower") + digest_value(mixed, "chosen_switches");
    CAPTURE(mixed.seconds, fine.seconds, predicted, real, tower);
    CHECK(std::abs(predicted - real) <= std::max(60., 0.1 * tower));
    // Whatever it chose, the plate does not come out more than 2 % slower than the fine print.
    CHECK(mixed.seconds <= 1.02 * fine.seconds);
}
