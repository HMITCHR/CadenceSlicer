#include <catch2/catch_all.hpp>

#include "wizard_apply_support.hpp"

#include "libslic3r/PrintConfig.hpp"

#include <algorithm>
#include <cstring>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::GUI;
using namespace Slic3r::GUI::WizardTest;

namespace {

// An H2D holding `left` and `right`, on the larger nozzle's stock printer preset, one PLA slot per
// nozzle in left, right order. Either nozzle can go on either side of an H2D.
ProfileSpec h2d_pair_spec(double left, double right)
{
    const auto suffix = [](double nozzle) {
        return nozzle == 0.4 ? std::string() : " " + std::string(nozzle == 0.2 ? "0.2" : nozzle == 0.6 ? "0.6" : "0.8") + " nozzle";
    };
    const auto min_height = [](double nozzle) { return nozzle == 0.2 ? 0.04 : nozzle == 0.4 ? 0.08 : nozzle == 0.6 ? 0.12 : 0.16; };
    const auto max_height = [](double nozzle) { return nozzle == 0.2 ? 0.14 : nozzle == 0.4 ? 0.28 : nozzle == 0.6 ? 0.42 : 0.56; };
    ProfileSpec spec = h2d_02_06_spec();
    const double coarse = std::max(left, right);
    spec.printer = "Bambu Lab H2D" + (coarse == 0.4 ? std::string(" 0.4 nozzle") : suffix(coarse));
    spec.process = coarse == 0.8 ? "0.40mm Standard @BBL H2D 0.8 nozzle" : coarse == 0.6 ? "0.30mm Standard @BBL H2D 0.6 nozzle"
                                                                         : "0.20mm Standard @BBL H2D";
    spec.materials = {"Bambu PLA Basic @BBL H2D" + suffix(left), "Bambu PLA Basic @BBL H2D" + suffix(right)};
    spec.also_visible_materials.clear();
    for (const char *nozzle : {" 0.2 nozzle", "", " 0.6 nozzle", " 0.8 nozzle"})
        spec.also_visible_materials.push_back(std::string("Bambu PLA Basic @BBL H2D") + nozzle);
    spec.nozzles = {left, right};
    spec.min_heights = {min_height(left), min_height(right)};
    spec.max_heights = {max_height(left), max_height(right)};
    return spec;
}

// The G-code header's "; key = value" lines.
std::map<std::string, std::string> header_values(const std::string &gcode)
{
    std::map<std::string, std::string> values;
    std::istringstream lines(gcode);
    std::string line;
    while (std::getline(lines, line)) {
        if (line.rfind("; ", 0) != 0)
            continue;
        const std::size_t equals = line.find(" = ");
        if (equals == std::string::npos)
            continue;
        values[line.substr(2, equals - 2)] = line.substr(equals + 3);
    }
    return values;
}

std::vector<std::string> split_values(const std::string &value)
{
    std::vector<std::string> out;
    std::stringstream stream(value);
    std::string item;
    while (std::getline(stream, item, ','))
        out.push_back(item);
    return out;
}

// What a nozzle printed with: its own column of every per-nozzle process value in the header, and
// the fastest gap fill it laid.
struct NozzleValues {
    std::map<std::string, std::string> process;
    float max_gap_fill_speed {0.f};
};

struct PairSlice {
    std::string process_name;
    NozzleValues fine;
    NozzleValues coarse;
};

PairSlice set_up_and_slice(double left, double right, MixedNozzleSlicingMode mode, double fine_height, int ratio)
{
    WizardProject project(h2d_pair_spec(left, right));
    const bool fine_left = left < right;
    if (mode == MixedNozzleSlicingMode::BodySplit)
        project.add_object("two bodies", {{"left body", Vec3d(10., 20., 10.), Vec3d::Zero(), 1},
                                          {"right body", Vec3d(10., 20., 10.), Vec3d(15., 0., 0.), 2}},
                           Vec2d(100., 100.));
    else
        project.add_object("box", {{"box", Vec3d(40., 40., 8.), Vec3d::Zero(), 1}}, Vec2d(100., 100.));
    SetupChoice choice;
    choice.mode = mode;
    choice.fine_slot = fine_left ? 0 : 1;
    choice.coarse_slot = fine_left ? 1 : 0;
    choice.fine_height = fine_height;
    choice.ratio = ratio;
    run_wizard_setup(project, choice);
    const PlateSlice slice = admitted_slice(project);

    PairSlice out;
    out.process_name = project.bundle.prints.get_edited_preset().name;
    const std::size_t fine_tool = fine_left ? 0 : 1;
    const auto header = header_values(slice.gcode);
    for (const std::string &key : print_options_with_variant) {
        const auto found = header.find(key);
        if (found == header.end())
            continue;
        const std::vector<std::string> columns = split_values(found->second);
        if (columns.size() != 2)
            continue;
        out.fine.process[key] = columns[fine_tool];
        out.coarse.process[key] = columns[1 - fine_tool];
    }
    for (const auto &move : slice.moves)
        if (move.type == EMoveType::Extrude && move.extrusion_role == erGapFill) {
            NozzleValues &nozzle = move.physical_tool_id == fine_tool ? out.fine : out.coarse;
            nozzle.max_gap_fill_speed = std::max(nozzle.max_gap_fill_speed, move.feedrate);
        }
    return out;
}

void check_same_values(const std::map<std::string, std::string> &plain, const std::map<std::string, std::string> &mirrored)
{
    REQUIRE_FALSE(plain.empty());
    for (const auto &[key, value] : plain) {
        INFO(key);
        REQUIRE(mirrored.count(key) == 1);
        CHECK(mirrored.at(key) == value);
    }
}

} // namespace

// The MN process presets hold one column of speeds and accelerations for the fine nozzle and one
// for the coarse nozzle. With the fine nozzle on the right, each nozzle must still print with the
// column for its own size, exactly as when the fine nozzle is on the left.
TEST_CASE("Setup on an H2D with the fine nozzle on the right gives each nozzle its own size's speeds",
          "[TestRebuild][E2E][Mirrored]")
{
    struct Pair { double fine, coarse; MixedNozzleSlicingMode mode; double fine_height; int ratio; };
    const Pair pair = GENERATE(Pair{0.2, 0.6, MixedNozzleSlicingMode::FeatureSplit, 0.10, 3},
                               Pair{0.4, 0.8, MixedNozzleSlicingMode::BodySplit, 0.20, 2});
    CAPTURE(pair.fine, pair.coarse, int(pair.mode));
    const PairSlice plain = set_up_and_slice(pair.fine, pair.coarse, pair.mode, pair.fine_height, pair.ratio);
    const PairSlice mirrored = set_up_and_slice(pair.coarse, pair.fine, pair.mode, pair.fine_height, pair.ratio);
    INFO("plain process " << plain.process_name << ", mirrored process " << mirrored.process_name);
    REQUIRE(mirrored.process_name == plain.process_name);
    REQUIRE(plain.process_name.rfind("MN ", 0) == 0);
    {
        INFO("fine nozzle");
        check_same_values(plain.fine.process, mirrored.fine.process);
    }
    {
        INFO("coarse nozzle");
        check_same_values(plain.coarse.process, mirrored.coarse.process);
    }
    // The fine nozzle's gap fill never runs faster than its own column allows.
    const float fine_gap_limit = std::stof(plain.fine.process.at("gap_infill_speed"));
    CHECK(mirrored.fine.max_gap_fill_speed <= fine_gap_limit + 0.5f);
}
