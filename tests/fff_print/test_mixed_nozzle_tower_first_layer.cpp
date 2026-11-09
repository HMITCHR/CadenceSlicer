// Prime tower: feedrate after an in-tower travel, the tower's first layer on every bed-level variant, the tower
// width of a plate planned as Off, and the Fine skins tower refusal.

#include <catch2/catch_all.hpp>

#include "mixed_nozzle_harness.hpp"
#include "mixed_nozzle_fixtures.hpp"
#include "mixed_nozzle_printers.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using namespace mixed_nozzle_fixtures;

namespace {
using Move = GCodeProcessorResult::MoveVertex;

std::vector<size_t> ramming_lines(const std::string &gcode)
{
    return bracketed_lines(gcode, {"; NOZZLE_CHANGE_START"}, {"; NOZZLE_CHANGE_END"});
}

bool on_line(const std::vector<size_t> &lines, const Move &move)
{
    return move.gcode_id < lines.size() && lines[move.gcode_id] != 0;
}

// Filaments 1 and 3 on the left 0.2 nozzle, 2 and 4 on the right coarse nozzle, Body Split, no support.
DynamicPrintConfig four_filament_body_config(double coarse_d, double coarse_min, double coarse_max, double fine, double coarse)
{
    DynamicPrintConfig config = coarse_base_support_config(MixedNozzleSlicingMode::BodySplit);
    config.set_key_value("printable_area", new ConfigOptionPoints{Vec2d(0., 0.), Vec2d(320., 0.), Vec2d(320., 320.), Vec2d(0., 320.)});
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, coarse_d});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, coarse_min});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, coarse_max});
    config.set_key_value("layer_height", new ConfigOptionFloat(fine));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(fine));
    config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(coarse));
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{int(std::lround(coarse / fine))});
    config.set_key_value("mixed_nozzle_process_nozzle_diameters", new ConfigOptionFloats{0.2, coarse_d});
    config.set_key_value("filament_diameter", new ConfigOptionFloats(4, 1.75));
    config.set_key_value("filament_map", new ConfigOptionInts{1, 2, 1, 2});
    config.set_key_value("flush_volumes_matrix", new ConfigOptionFloats(32, 0.));
    config.set_key_value("filament_colour", new ConfigOptionStrings{"#FF0000", "#0000FF", "#00FF00", "#FFFF00"});
    config.set_key_value("filament_self_index", new ConfigOptionInts{1, 2, 3, 4});
    config.set_key_value("filament_volume_map", new ConfigOptionInts(4, 0));
    config.set_key_value("filament_start_gcode", new ConfigOptionStrings(4, ""));
    config.set_key_value("filament_end_gcode", new ConfigOptionStrings(4, ""));
    config.set_key_value("filament_type", new ConfigOptionStrings(4, "PLA"));
    config.set_key_value("filament_adhesiveness_category", new ConfigOptionInts(4, 100));
    config.set_key_value("filament_soluble", new ConfigOptionBools(4, false));
    config.set_key_value("filament_is_support", new ConfigOptionBools(4, false));
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id", "internal_solid_filament_id",
                            "top_surface_filament_id", "bottom_surface_filament_id", "sparse_infill_filament_id"})
        config.set_key_value(key, new ConfigOptionInt(0));
    config.set_key_value("enable_support", new ConfigOptionBool(false));
    config.set_key_value("wipe_tower_wall_type", new ConfigOptionEnum<WipeTowerWallType>(wtwRib));
    config.set_key_value("prime_tower_brim_width", new ConfigOptionFloat(-1.));
    fill_per_filament_values(config);
    return config;
}

// four_filament_body_config with the coarse nozzle on the left: filaments 1 and 3 on the right 0.2.
void put_coarse_left(DynamicPrintConfig &config, double coarse_d, double coarse_min, double coarse_max)
{
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{coarse_d, 0.2});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{coarse_min, 0.04});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{coarse_max, 0.14});
    config.set_key_value("mixed_nozzle_process_nozzle_diameters", new ConfigOptionFloats{coarse_d, 0.2});
    config.set_key_value("filament_map", new ConfigOptionInts{2, 1, 2, 1});
}

ModelVolume *add_block(ModelObject *object, const Vec3d &size, const Vec3d &offset, int filament, double height)
{
    ModelVolume *volume = object->add_volume(make_cube(size.x(), size.y(), size.z()), ModelVolumeType::MODEL_PART, false);
    volume->set_offset(offset);
    volume->config.set_key_value("extruder", new ConfigOptionInt(filament));
    volume->config.set_key_value("regional_layer_height", new ConfigOptionFloat(height));
    return volume;
}

void place_and_apply(ModelObject *object, Model &model, Print &print, const DynamicPrintConfig &config)
{
    object->add_instance();
    object->instances.front()->set_offset(Vec3d(60., 60., 0.));
    object->ensure_on_bed();
    print.apply(model, config);
    print.set_status_silent();
}

// Fine skins on a coarse body on the bed, a fine block on top and a fine strip on the front face from `strip_z` up
// (raised text on the top and the side): with switches on every coarse layer the fine filament is the tower's usual
// wall filament.
void add_fine_skins_plate(Model &model, Print &print, const DynamicPrintConfig &config, double fine, double coarse, double strip_z)
{
    ModelObject *object = model.add_object();
    object->name = "fine-skins";
    ModelVolume *body = add_block(object, Vec3d(20., 12., 3.), Vec3d::Zero(), 2, coarse);
    body->config.set_key_value("mixed_nozzle_body_fine_skins", new ConfigOptionBool(true));
    for (const char *key : {"internal_solid_filament_id", "top_surface_filament_id", "bottom_surface_filament_id"})
        body->config.set_key_value(key, new ConfigOptionInt(1));
    add_block(object, Vec3d(8., 6., 1.), Vec3d(6., 3., 3.), 1, fine);
    add_block(object, Vec3d(10., 1., 2.), Vec3d(5., -1., strip_z), 1, fine);
    place_and_apply(object, model, print, config);
}

// What the tower lays on and near the bed, read from the exported G-code in print order.
struct TowerBedAudit {
    double first_z            = 0.;  // lowest tower road
    double near_bed_length    = 0.;  // tower roads up to 0.6 mm
    double over_bare_bed      = 0.;  // of those, above the first tower layer with nothing under them
    double from_bed_late      = 0.;  // roads above the first tower layer whose own height reaches down to the bed
    double bed_contact_fast   = 0.;  // bed-contact roads (not ramming) faster than the tower's first-layer speed
    double bed_ramming_fast   = 0.;  // ramming on the bed faster than that speed
    double first_extent       = 0.;  // smaller side of the first tower layer's bounds
    double top_extent         = 0.;  // smaller side of the tower's bounds above 2 mm (past the base chamfer)
    double wall_widest[2]     = {0., 0.}; // per nozzle: widest first-layer road that is not purge, ramming or block fill
    std::set<int> bed_tools;
    std::string  levels;             // one line per tower Z, for the failure message
};

TowerBedAudit audit_tower_bed(const CadenceTest::Facts &facts, double first_layer_height)
{
    TowerBedAudit audit;
    const std::vector<size_t> ramming = ramming_lines(facts.gcode);
    // Lines inside a tool change or a block fill: what is left of the tower is its wall and brim.
    const std::vector<size_t> not_wall = bracketed_lines(facts.gcode, {"; CP TOOLCHANGE START", "; CP EMPTY GRID START"},
                                                         {"; CP TOOLCHANGE END", "; CP EMPTY GRID END"});
    const double cell = 0.5;
    std::map<std::pair<int, int>, float> top;
    auto key = [cell](double x, double y) { return std::make_pair(int(std::floor(x / cell)), int(std::floor(y / cell))); };
    auto top_near = [&](double x, double y) {
        float best = 0.f;
        const auto at = key(x, y);
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy) {
                const auto found = top.find({at.first + dx, at.second + dy});
                if (found != top.end())
                    best = std::max(best, found->second);
            }
        return best;
    };
    struct Level { double length = 0., bare = 0., fastest = 0.; std::set<int> tools; double height = 0.; };
    std::map<int, Level> levels; // by Z in microns
    BoundingBoxf bounds[2];      // the first tower layer, and the tower above 2 mm
    audit.first_z = 1e9;
    for (const Move &move : facts.moves)
        if (move.type == EMoveType::Extrude && move.extrusion_role == erWipeTower)
            audit.first_z = std::min(audit.first_z, double(move.position.z()));
    Vec3f previous = Vec3f::Zero();
    for (const Move &move : facts.moves) {
        const Vec3f from = previous;
        previous = move.position;
        if (move.type != EMoveType::Extrude || move.extrusion_role != erWipeTower)
            continue;
        const double z = move.position.z();
        const double length = (move.position.head<2>() - from.head<2>()).norm();
        if (length <= 0.)
            continue;
        if (z > 0.6) {
            if (z > 2.)
                bounds[1].merge(Vec2d(move.position.x(), move.position.y()));
            continue;
        }
        const bool on_first = z <= first_layer_height + 0.02;
        const bool rams     = on_line(ramming, move);
        Level &level = levels[int(std::lround(z * 1000.))];
        level.length += length;
        level.tools.insert(int(move.physical_tool_id));
        level.height  = move.height;
        level.fastest = std::max(level.fastest, double(move.feedrate));
        audit.near_bed_length += length;
        const int    steps = std::max(1, int(std::ceil(length / 0.25)));
        const double step  = length / steps;
        std::vector<std::pair<double, double>> points;
        double bare = 0.;
        for (int i = 0; i <= steps; ++i) {
            const double t = double(i) / steps;
            const double x = from.x() + t * (move.position.x() - from.x());
            const double y = from.y() + t * (move.position.y() - from.y());
            points.emplace_back(x, y);
            if (top_near(x, y) <= 1e-6f)
                bare += step;
        }
        bare = std::min(bare, length);
        if (on_first) {
            audit.bed_tools.insert(int(move.physical_tool_id));
            if (move.physical_tool_id < 2 && ! rams && move.gcode_id < not_wall.size() && not_wall[move.gcode_id] == 0)
                audit.wall_widest[move.physical_tool_id] = std::max(audit.wall_widest[move.physical_tool_id], double(move.width));
            bounds[0].merge(Vec2d(move.position.x(), move.position.y()));
        }
        if (! on_first && z - double(move.height) < 0.02)
            audit.from_bed_late += length;
        if (bare > 0. && ! on_first) {
            audit.over_bare_bed += bare;
            level.bare += bare;
        }
        // Bed contact: on the first tower layer, or anywhere with nothing under it.
        if ((on_first || bare > 0.5 * length) && move.feedrate > 15.01f)
            (rams ? audit.bed_ramming_fast : audit.bed_contact_fast) += length;
        for (const auto &point : points) {
            float &cell_top = top[key(point.first, point.second)];
            cell_top = std::max(cell_top, float(z));
        }
    }
    for (int i = 0; i < 2; ++i)
        (i == 0 ? audit.first_extent : audit.top_extent) =
            bounds[i].defined ? std::min(bounds[i].size().x(), bounds[i].size().y()) : 0.;
    std::ostringstream out;
    for (const auto &[z, level] : levels) {
        out << "Z " << z / 1000. << " h " << level.height << " tools";
        for (int tool : level.tools)
            out << " " << tool;
        out << " length " << level.length << " over bare bed " << level.bare << " fastest " << level.fastest << " mm/s; ";
    }
    audit.levels = out.str();
    return audit;
}

// The coarse tool's tower roads on the tower's bed level, with widths measured from E the way the
// G-code checker does it. Needs a rectangle wall: the brim is what lies well outside the bed level's
// solid fill.
struct CoarseBedLevel {
    double first_z        = 0.;
    double narrowest      = 1e9; // narrowest coarse road on the bed level
    double brim_width     = 0.;  // mean width of the bed-level brim loops
    double chamfer_width  = 0.;  // mean width of the chamfer loops on the next coarse level
    double fullness       = 0.;  // deposited volume / (area x height) in the middle of the bed level
    double fine_widest    = 0.;  // widest fine-tool tower road anywhere, not counting ramming
};

CoarseBedLevel audit_coarse_bed_level(const CadenceTest::Facts &facts, int coarse_tool)
{
    CoarseBedLevel audit;
    const std::vector<size_t> ramming = ramming_lines(facts.gcode);
    const std::vector<size_t> block_fill = bracketed_lines(facts.gcode, {"; CP EMPTY GRID START"}, {"; CP EMPTY GRID END"});
    const double filament_area = 0.25 * M_PI * 1.75 * 1.75;
    struct Road { Vec2d a, b; double z, height, width, area; int tool; bool rams, fill; };
    std::vector<Road> roads;
    Vec3f previous = Vec3f::Zero();
    for (const Move &move : facts.moves) {
        const Vec3f from = previous;
        previous = move.position;
        if (move.type != EMoveType::Extrude || move.extrusion_role != erWipeTower || move.delta_extruder <= 0.f || move.height <= 0.f)
            continue;
        const Vec2d a = from.head<2>().cast<double>(), b = move.position.head<2>().cast<double>();
        const double length = (b - a).norm();
        // Short roads carry too few digits of E to measure.
        if (length < 1.)
            continue;
        const double area = double(move.delta_extruder) * filament_area / length;
        roads.push_back({a, b, double(move.position.z()), double(move.height), area / double(move.height) + double(move.height) * (1. - M_PI / 4.),
                         area, int(move.physical_tool_id), on_line(ramming, move), on_line(block_fill, move)});
    }
    if (roads.empty())
        return audit;
    audit.first_z = 1e9;
    for (const Road &road : roads)
        audit.first_z = std::min(audit.first_z, road.z);
    double second_z = 1e9;
    for (const Road &road : roads)
        if (road.tool == coarse_tool && road.z > audit.first_z + 1e-3)
            second_z = std::min(second_z, road.z);
    // The bed level's solid fill; the wall runs at most 1 mm outside it and the brim beyond.
    BoundingBoxf fill;
    for (const Road &road : roads)
        if (road.fill && road.tool == coarse_tool && std::abs(road.z - audit.first_z) < 1e-3) {
            fill.merge(road.a);
            fill.merge(road.b);
        }
    if (! fill.defined)
        return audit;
    auto outside_body = [&fill](const Vec2d &p) {
        return p.x() < fill.min.x() - 1.1 || p.x() > fill.max.x() + 1.1 || p.y() < fill.min.y() - 1.1 || p.y() > fill.max.y() + 1.1;
    };
    // The middle of the solid fill.
    const Vec2d lo = fill.min + Vec2d(1., 1.), hi = fill.max - Vec2d(1., 1.);
    // Length of the segment a-b inside [lo, hi] (Liang-Barsky).
    auto clipped_length = [&lo, &hi](const Vec2d &a, const Vec2d &b) {
        double t0 = 0., t1 = 1.;
        const Vec2d d = b - a;
        for (int axis = 0; axis < 2; ++axis) {
            if (std::abs(d[axis]) < 1e-12) {
                if (a[axis] < lo[axis] || a[axis] > hi[axis])
                    return 0.;
                continue;
            }
            double ta = (lo[axis] - a[axis]) / d[axis], tb = (hi[axis] - a[axis]) / d[axis];
            if (ta > tb)
                std::swap(ta, tb);
            t0 = std::max(t0, ta);
            t1 = std::min(t1, tb);
        }
        return t1 > t0 ? (t1 - t0) * d.norm() : 0.;
    };
    double brim_length = 0., brim_sum = 0., chamfer_length = 0., chamfer_sum = 0., volume = 0., height = 0.;
    for (const Road &road : roads) {
        const double length = (road.b - road.a).norm();
        if (road.tool != coarse_tool) {
            if (! road.rams)
                audit.fine_widest = std::max(audit.fine_widest, road.width);
            continue;
        }
        const bool brim_loop = outside_body(road.a) && outside_body(road.b);
        if (std::abs(road.z - audit.first_z) < 1e-3) {
            if (! road.rams)
                audit.narrowest = std::min(audit.narrowest, road.width);
            if (brim_loop) {
                brim_length += length;
                brim_sum += road.width * length;
            }
            volume += road.area * clipped_length(road.a, road.b);
            height = road.height;
        } else if (std::abs(road.z - second_z) < 1e-3 && brim_loop) {
            chamfer_length += length;
            chamfer_sum += road.width * length;
        }
    }
    audit.brim_width    = brim_length > 0. ? brim_sum / brim_length : 0.;
    audit.chamfer_width = chamfer_length > 0. ? chamfer_sum / chamfer_length : 0.;
    const double box_area = std::max(0., hi.x() - lo.x()) * std::max(0., hi.y() - lo.y());
    audit.fullness = box_area > 0. && height > 0. ? volume / (box_area * height) : 0.;
    return audit;
}

// The first layer of the exported G-code: its height tag and whether anything is extruded on it, and
// whether the tower's first road is on it.
struct FirstGcodeLayer {
    double z            = 0.;
    bool   extrudes     = false;
    bool   tower        = false;
    bool   tower_before = false; // a tower road before the first layer change
};

FirstGcodeLayer first_gcode_layer(const std::string &gcode)
{
    FirstGcodeLayer out;
    std::istringstream lines(gcode);
    std::string line;
    int layer = 0;
    while (std::getline(lines, line)) {
        if (line.rfind("; CHANGE_LAYER", 0) == 0) {
            if (++layer > 1)
                break;
            continue;
        }
        if (line.rfind("; WIPE_TOWER_START", 0) == 0 || line.rfind("; FEATURE: Prime tower", 0) == 0)
            (layer == 0 ? out.tower_before : out.tower) = true;
        if (layer != 1)
            continue;
        if (line.rfind("; Z_HEIGHT: ", 0) == 0)
            out.z = std::atof(line.c_str() + 12);
        else if (line.rfind(";Z:", 0) == 0)
            out.z = std::atof(line.c_str() + 3);
        else if (line.rfind("G1 ", 0) == 0 || line.rfind("G2 ", 0) == 0 || line.rfind("G3 ", 0) == 0) {
            // A road: a move in X or Y that extrudes (an unretract alone is not one).
            const size_t e = line.find(" E");
            const bool moves = line.find(" X") < e || line.find(" Y") < e;
            if (e != std::string::npos && moves && line.find(';') > e && std::atof(line.c_str() + e + 2) > 0.)
                out.extrudes = true;
        }
    }
    return out;
}

// Nozzle changes in the move stream, and how many of them start on the tower (the new nozzle's first road there).
std::pair<size_t, size_t> changes_primed(const CadenceTest::Facts &facts)
{
    size_t changes = 0, primed = 0;
    int active = -1;
    for (const Move &move : facts.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        const int tool = int(move.physical_tool_id);
        if (active != -1 && tool != active) {
            ++changes;
            primed += move.extrusion_role == erWipeTower ? 1 : 0;
        }
        active = tool;
    }
    return {changes, primed};
}

// The tower's roads as the G-code tags them (under "; FEATURE: Prime tower", widths from "; LINE_WIDTH:", which every
// printer writes): the widest road outside a nozzle change, the widest inside one (the nozzle-change width), and the
// physical tools that lay them.
struct TowerWidths { double roads = 0., nozzle_change = 0.; std::set<int> tools; };

TowerWidths tower_widths(const CadenceTest::Facts &facts)
{
    const std::vector<size_t> nozzle_change = ramming_lines(facts.gcode);
    std::vector<double> width(1, 0.); // per one-based line: the width of a tower road, 0 elsewhere
    std::istringstream lines(facts.gcode);
    std::string line, feature;
    double tagged = 0.;
    while (std::getline(lines, line)) {
        if (line.rfind("; FEATURE: ", 0) == 0)
            feature = line.substr(11);
        else if (line.rfind("; LINE_WIDTH: ", 0) == 0)
            tagged = std::atof(line.c_str() + 14);
        const std::string code = line.substr(0, line.find(';'));
        const size_t e = code.find(" E");
        const bool road = feature == "Prime tower" && code.rfind("G1 ", 0) == 0 && e != std::string::npos &&
                          std::atof(code.c_str() + e + 2) > 0. &&
                          (code.find(" X") != std::string::npos || code.find(" Y") != std::string::npos);
        width.push_back(road ? tagged : 0.);
    }
    TowerWidths out;
    for (const Move &move : facts.moves) {
        if (move.type != EMoveType::Extrude || move.gcode_id >= width.size() || width[move.gcode_id] <= 0.)
            continue;
        double &widest = on_line(nozzle_change, move) ? out.nozzle_change : out.roads;
        widest = std::max(widest, width[move.gcode_id]);
        out.tools.insert(int(move.physical_tool_id));
    }
    return out;
}
} // namespace

// In a mixed mode the tower's travels name the tower travel speed. A road after one that
// names no feedrate must go back to the print feedrate, not run at the travel speed (which the
// volumetric ceiling then cuts down to whatever the filament allows, instead of the intended speed).
TEST_CASE("Feature Split: a tower road that names no feedrate does not run at the tower's travel speed",
          "[TestRebuild][TowerSpeed]")
{
    CadenceTest::Scene scene;
    scene.config = feature_split_tower_config(.20, .60, false);
    scene.config.set_key_value("wipe_tower_wall_type", new ConfigOptionEnum<WipeTowerWallType>(wtwRib));
    scene.config.set_key_value("mixed_nozzle_auto_pad", new ConfigOptionBool(true));
    scene.config.set_key_value("prime_tower_width", new ConfigOptionFloat(60.));
    scene.config.set_key_value("prime_tower_brim_width", new ConfigOptionFloat(-1.));
    scene.config.set_key_value("first_layer_flow_ratio", new ConfigOptionFloat(1.));
    apply_tower_exit_retraction(scene.config);
    // A limit no tower road reaches, so the volumetric ceiling cannot hide an inherited travel speed.
    scene.config.set_key_value("filament_max_volumetric_speed", new ConfigOptionFloatsNullable{200., 200.});
    scene.populate = [](Model &model, Print &print, const DynamicPrintConfig &config) {
        init_feature_flow_fixture(model, print, config, 4., 60.);
    };
    const double travel = scene.config.option<ConfigOptionFloatsNullable>("travel_speed")->get_at(0);
    const CadenceTest::Facts facts = CadenceTest::slice(scene);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    const std::vector<size_t> ramming = ramming_lines(facts.gcode);
    size_t roads = 0, at_travel_speed = 0;
    double fastest = 0.;
    for (const Move &move : facts.moves) {
        if (move.type != EMoveType::Extrude || move.extrusion_role != erWipeTower || on_line(ramming, move))
            continue;
        ++ roads;
        fastest = std::max(fastest, double(move.feedrate));
        at_travel_speed += std::abs(move.feedrate - travel) < 0.01 ? 1 : 0;
    }
    CAPTURE(roads, at_travel_speed, fastest, travel);
    CHECK(roads > 0);
    CHECK(at_travel_speed == 0);
    // The tower's own cap for a road is 90 mm/s (F5400).
    CHECK(fastest <= 90.01);
}

namespace {
// One bed-level variant: Body Split blocks side by side on the bed (10 x 12 x 3 mm, even filaments on the coarse
// nozzle), or the fine-skins plate.
struct BedLevelRow
{
    const char *name;
    double coarse_d, coarse_min, coarse_max, fine, coarse;
    bool swapped;                // the coarse nozzle on the left
    std::vector<int> filaments;  // the blocks left to right
    double fine_lift;            // the fine blocks start this far up
    double first_layer;          // 0: the fine height
    WipeTowerWallType wall;
    bool fine_skins;             // the fine-skins plate in place of the blocks
    int bed_tools;               // who lays the tower's bed level: 0 not checked, 1 the coarse tool alone, 2 both
};

CadenceTest::Facts slice_bed_level(const BedLevelRow &row)
{
    CadenceTest::Scene scene;
    scene.config = four_filament_body_config(row.coarse_d, row.coarse_min, row.coarse_max, row.fine, row.coarse);
    scene.config.set_key_value("wipe_tower_wall_type", new ConfigOptionEnum<WipeTowerWallType>(row.wall));
    if (row.first_layer > 0.)
        scene.config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(row.first_layer));
    if (row.swapped)
        put_coarse_left(scene.config, row.coarse_d, row.coarse_min, row.coarse_max);
    scene.populate = [row](Model &model, Print &print, const DynamicPrintConfig &config) {
        if (row.fine_skins)
            return add_fine_skins_plate(model, print, config, row.fine, row.coarse, 0.9);
        ModelObject *object = model.add_object();
        object->name = "bed-level";
        for (size_t i = 0; i < row.filaments.size(); ++i) {
            const bool coarse = row.filaments[i] % 2 == 0;
            add_block(object, Vec3d(10., 12., 3.), Vec3d(10. * double(i), 0., coarse ? 0. : row.fine_lift), row.filaments[i],
                      coarse ? row.coarse : row.fine);
        }
        place_and_apply(object, model, print, config);
    };
    return CadenceTest::slice(scene);
}
} // namespace

// The tower's first layer on every bed-level variant: a fine switch on layer 1 under a coarse arrival on layer 2 (it
// laid only its prime road on the bed, and the coarse arrival a step from the bed over bare bed); a nozzle change on
// layer 1; a print that starts on the coarse nozzle; a coarse body whose nozzle cannot lay the first layer, so it
// starts with a thicker first cell (the tower was left out, and the G-code's first layer was empty); fine skins under
// a coarse body (the fine nozzle laid a level over its maximum). The print starts with the layer that lays the part's
// first roads, the tower and its brim start there, nothing later goes down over bare bed, bed contact and bed ramming
// run at the first-layer speed, a coarse wall is at its own width, and every nozzle change starts on the tower.
TEST_CASE("The tower's first layer: laid with the part, at each tool's width and the first-layer speed",
          "[TestRebuild][TowerFirstLayer][FirstGcodeLayer]")
{
    const BedLevelRow row = GENERATE(
        BedLevelRow{"fine switch on layer 1, 0.2/0.6", 0.6, 0.12, 0.42, 0.10, 0.20, false, {1, 3, 2}, 0., 0., wtwRib, false, 0},
        BedLevelRow{"fine switch on layer 1, 0.2/0.6", 0.6, 0.12, 0.42, 0.10, 0.20, true, {1, 3, 2}, 0., 0., wtwRib, false, 0},
        BedLevelRow{"fine switch on layer 1, 0.2/0.8", 0.8, 0.16, 0.56, 0.08, 0.16, false, {1, 3, 2}, 0., 0., wtwRib, false, 0},
        // 0.12 fine layers: the 0.6 nozzle's first legal layer is the print's first layer.
        BedLevelRow{"nozzle change on layer 1", 0.6, 0.12, 0.42, 0.12, 0.24, false, {1, 2}, 0., 0., wtwRib, false, 2},
        BedLevelRow{"nozzle change on layer 1", 0.6, 0.12, 0.42, 0.12, 0.24, true, {1, 2}, 0., 0., wtwRib, false, 2},
        BedLevelRow{"coarse bed level", 0.6, 0.12, 0.42, 0.12, 0.24, false, {2, 1}, 1.2, 0., wtwRib, false, 1},
        BedLevelRow{"coarse bed level", 0.6, 0.12, 0.42, 0.12, 0.24, true, {2, 1}, 1.2, 0., wtwRib, false, 1},
        // 0.2/0.6 Fine 0.10-0.30: the 0.6 cannot lay 0.10, so its first cell is 0.20 over an empty first layer. A
        // rectangle wall, for the coarse bed level audit.
        BedLevelRow{"coarse first cell, 0.2/0.6 Fine", 0.6, 0.12, 0.42, 0.10, 0.30, false, {2, 1}, 1.2, 0., wtwRectangle, false, 1},
        BedLevelRow{"coarse first cell, 0.2/0.6 Fine", 0.6, 0.12, 0.42, 0.10, 0.30, true, {2, 1}, 1.2, 0., wtwRectangle, false, 1},
        // 0.2/0.8 Standard 0.08-0.32 on a 0.10 first layer: first cell 0.18.
        BedLevelRow{"coarse first cell, 0.2/0.8 Standard", 0.8, 0.16, 0.56, 0.08, 0.32, false, {2, 1}, 1.2, 0.10, wtwRib, false, 1},
        BedLevelRow{"coarse first cell, 0.2/0.8 Standard", 0.8, 0.16, 0.56, 0.08, 0.32, true, {2, 1}, 1.2, 0.10, wtwRib, false, 1},
        // The coarse nozzle lays the first layer itself.
        BedLevelRow{"coarse first layer, 0.2/0.6 Standard", 0.6, 0.12, 0.42, 0.12, 0.36, false, {2, 1}, 1.2, 0.12, wtwRib, false, 1},
        BedLevelRow{"coarse first layer, 0.2/0.4 Standard", 0.4, 0.08, 0.28, 0.10, 0.20, false, {2, 1}, 1.2, 0.10, wtwRib, false, 1},
        BedLevelRow{"fine skins under a coarse body", 0.6, 0.12, 0.42, 0.10, 0.30, false, {}, 0., 0., wtwRib, true, 0});
    CAPTURE(row.name, row.swapped);
    const int coarse_tool = row.swapped ? 0 : 1;
    const CadenceTest::Facts facts = slice_bed_level(row);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    double first_model_z = 1e9;
    size_t fine_on_bed = 0;
    for (const Move &move : facts.moves)
        if (CadenceTest::model_road(move)) {
            first_model_z = std::min(first_model_z, double(move.position.z()));
            fine_on_bed += int(move.physical_tool_id) != coarse_tool && move.position.z() < 0.15f ? 1 : 0;
        }
    const TowerBedAudit audit = audit_tower_bed(facts, first_model_z);
    const FirstGcodeLayer first = first_gcode_layer(facts.gcode);
    const std::pair<size_t, size_t> changes = changes_primed(facts);
    INFO(audit.levels);
    CAPTURE(first_model_z, first.z, first.extrudes, first.tower, first.tower_before, audit.first_z, audit.near_bed_length,
            audit.over_bare_bed, audit.from_bed_late, audit.bed_contact_fast, audit.bed_ramming_fast, audit.first_extent,
            audit.top_extent, audit.wall_widest[0], audit.wall_widest[1], audit.bed_tools.size(), changes.first, changes.second);
    // The scene.
    if (row.bed_tools == 1)
        REQUIRE(audit.bed_tools == std::set<int>{coarse_tool});
    if (row.bed_tools == 2)
        REQUIRE(audit.bed_tools.size() == 2);
    if (row.fine_skins)
        REQUIRE(fine_on_bed > 0);
    // The print starts with the layer that lays the part's first roads; the tower starts there too, with its brim.
    CHECK(first.extrudes);
    CHECK(std::abs(first.z - first_model_z) < 1e-3);
    CHECK_FALSE(first.tower_before);
    CHECK(first.tower);
    CHECK(std::abs(audit.first_z - first_model_z) < 1e-3);
    CHECK(audit.first_extent > audit.top_extent + 1.);
    // No later level goes down over bare bed or as a step from the bed, into or beside the bed level.
    CHECK(audit.over_bare_bed <= 0.02 * audit.near_bed_length);
    CHECK(audit.from_bed_late == 0.);
    // Bed contact and ramming laid on the bed go no faster than the rest of the first layer.
    CHECK(audit.bed_contact_fast == 0.);
    CHECK(audit.bed_ramming_fast == 0.);
    // A wall drawn by the coarse tool is at its own width, not the fine nozzle's first-layer width.
    if (audit.wall_widest[coarse_tool] > 0.)
        CHECK(audit.wall_widest[coarse_tool] >= 0.75 * row.coarse_d);
    // Every nozzle change starts on the tower.
    CHECK(changes.first > 0);
    CHECK(changes.second == changes.first);
    if (row.bed_tools == 1) {
        CHECK(audit.wall_widest[coarse_tool] > 0.);
        // The coarse nozzle lays the first layer at a height within its own limits, no thinner than the first layer.
        CHECK(first.z >= std::max(row.first_layer, row.coarse_min) - 1e-3);
        CHECK(first.z <= row.coarse_max + 1e-3);
    }
    if (row.wall == wtwRectangle) {
        // The coarse tool's bed level (brim loops, solid fill, base) at its own width and full, and the brim's
        // first-layer flow boost capped so the road is no wider than 1.25 times its nozzle.
        const CoarseBedLevel level = audit_coarse_bed_level(facts, coarse_tool);
        CAPTURE(level.first_z, level.narrowest, level.brim_width, level.chamfer_width, level.fullness, level.fine_widest);
        REQUIRE(level.brim_width > 0.);
        REQUIRE(level.chamfer_width > 0.);
        CHECK(level.narrowest >= 0.75 * row.coarse_d);
        // The brim road is already 1.25 x the nozzle, so the 1.15 first-layer boost is capped away: the brim is as wide
        // as the chamfer loops above it, and no wider than the cap.
        CHECK_THAT(level.brim_width / level.chamfer_width, Catch::Matchers::WithinAbs(1.0, 0.035));
        CHECK(level.brim_width <= 1.25 * row.coarse_d + 0.03);
        // About as full as a single nozzle's solid fill (94 percent at 0.75 mm wide and 0.2 mm high).
        CHECK(level.fullness >= 0.88);
        CHECK(level.fullness <= 1.02);
        // The fine nozzle's tower roads stay at its own width.
        CHECK(level.fine_widest <= 0.2 * 1.25 + 0.01);
    }
}

namespace {
// A printer for the tower width scene: the H2D-like 0.2/0.6 pair (filaments 1 and 3 on the 0.2) with the coarse
// nozzle on either side, or a U1 fitted 0.2, 0.2, 0.2, 0.8 (filaments 1 and 2 on their own 0.2 tools, the coarse tool
// last). off_nozzle: the nozzle whose width Off draws the tower at.
struct WidthPrinter { const char *name; bool u1; bool swapped; int coarse_tool; double off_nozzle; };

// Two fine colors side by side and nothing for the coarse nozzle (no sparse infill, so Feature Split has no coarse
// band to lay; small parts and raised text print this way).
CadenceTest::Facts slice_two_fine_colors(const WidthPrinter &printer, MixedNozzleSlicingMode mode)
{
    CadenceTest::Scene scene;
    if (printer.u1) {
        scene.bambu = false;
        scene.config = CadenceTest::u1_config({.2, .2, .2, .8});
        // Every role on the volume's own filament.
        CadenceTest::feature_split_heights(scene.config, .08, .32, 4, 0, 0, {.2, .8});
        scene.config.set_key_value("wipe_tower_x", new ConfigOptionFloats{120.});
        scene.config.set_key_value("wipe_tower_y", new ConfigOptionFloats{140.});
    } else {
        scene.config = four_filament_body_config(0.6, 0.12, 0.42, 0.10, 0.20);
        if (printer.swapped)
            put_coarse_left(scene.config, 0.6, 0.12, 0.42);
    }
    scene.config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(mode));
    scene.config.set_key_value("sparse_infill_density", new ConfigOptionPercent(0.));
    if (mode == MixedNozzleSlicingMode::Off)
        scene.config.set_key_value("wipe_tower_no_sparse_layers", new ConfigOptionBool(false));
    scene.populate = [second = printer.u1 ? 2 : 3](Model &model, Print &print, const DynamicPrintConfig &config) {
        ModelObject *object = model.add_object();
        object->name = "two-fine-colors";
        for (int filament : {1, second}) {
            ModelVolume *volume = object->add_volume(make_cube(10., 12., 3.), ModelVolumeType::MODEL_PART, false);
            volume->set_offset(Vec3d(filament == 1 ? 0. : 10., 0., 0.));
            volume->config.set_key_value("extruder", new ConfigOptionInt(filament));
        }
        place_and_apply(object, model, print, config);
    };
    return CadenceTest::slice(scene);
}
} // namespace

// A Feature Split plate that never uses the coarse nozzle gets Off's tower. Off draws the tower at one shared width,
// the last filament's nozzle (an index past the nozzles takes the first), so the fine nozzle laid roads as wide as the
// coarse one's (0.75 mm on the H2D with the coarse nozzle first, 1.0 mm on the U1). The plate's tower now follows the
// smallest nozzle, as in the mixed-nozzle modes; Off keeps the stock width, where every tool may thicken the tower.
TEST_CASE("A Feature Split plate planned as Off draws its tower at the fine nozzle's width", "[TestRebuild][TowerWidth][ToolChanger]")
{
    const WidthPrinter printer = GENERATE(WidthPrinter{"H2D", false, false, 1, .2}, WidthPrinter{"H2D, coarse on the left", false, true, 0, .6},
                                          WidthPrinter{"U1 0.2, 0.2, 0.2, 0.8", true, false, 3, .8});
    CAPTURE(printer.name);
    const CadenceTest::Facts split = slice_two_fine_colors(printer, MixedNozzleSlicingMode::FeatureSplit);
    INFO(split.refusal.string);
    REQUIRE(split.refusal.string.empty());
    REQUIRE_FALSE(split.gcode.empty());
    const TowerWidths fine = tower_widths(split);
    CAPTURE(fine.roads, fine.nozzle_change, fine.tools.size());
    // The scene: a tower, laid only by the fine tools (the plate is planned as Off).
    REQUIRE_FALSE(fine.tools.empty());
    REQUIRE(fine.tools.count(printer.coarse_tool) == 0);
    CHECK(fine.roads <= 0.2 * 1.25 + 0.01);
    // The 0.2 nozzle's nozzle-change width (a 0.8's is 1.4 mm).
    CHECK(fine.nozzle_change <= 0.5 + 0.01);

    const CadenceTest::Facts off = slice_two_fine_colors(printer, MixedNozzleSlicingMode::Off);
    INFO(off.refusal.string);
    REQUIRE(off.refusal.string.empty());
    REQUIRE_FALSE(off.gcode.empty());
    const TowerWidths stock = tower_widths(off);
    CAPTURE(stock.roads);
    REQUIRE_FALSE(stock.tools.empty());
    CHECK_THAT(stock.roads, Catch::Matchers::WithinAbs(printer.off_nozzle * 1.25, 0.01));
}

// Fine skins put the fine filament on the tower only where the skins and the fine bodies are. With a fine body starting
// a gap above the bottom skins, the fine filament can arrive a coarse step above the last tower level it laid, taller
// than the fine nozzle can lay, and the slice stopped with an internal tower error (SRL-TOWER-STRUCTURE). It is now
// refused as a Fine skins setting (SRL-A60). Whether it happens depends on where the fine bodies start against the
// coarse cells, so it is found when the tower is planned. Sweep: every pair, every coarse height the pair qualifies
// for (whole multiples of half the fine nozzle, up to 0.7 times the coarse nozzle), the fine strip starting at 0.5,
// 0.9 or 1.3 mm. Only the cells that stopped with the internal error are refused, and every other one slices.
TEST_CASE("Fine skins that would leave the fine nozzle a tower level over its maximum are refused, on every pair and height",
          "[TestRebuild][TowerFirstLayer][Pairs]")
{
    struct SweepPair { const char *name; double fine_d, coarse_d; bool swapped; };
    const std::vector<SweepPair> pairs{{"0.2/0.4", 0.2, 0.4, false}, {"0.2/0.6", 0.2, 0.6, false},
                                       {"0.2/0.6 fine right", 0.2, 0.6, true}, {"0.2/0.8", 0.2, 0.8, false},
                                       {"0.4/0.6", 0.4, 0.6, false}, {"0.4/0.8", 0.4, 0.8, false}};
    // The cells that stopped with the internal tower error.
    const std::set<std::string> refused_cells{"0.2/0.6 0.20 0.9", "0.2/0.6 fine right 0.20 0.9", "0.2/0.8 0.20 0.9",
                                              "0.2/0.8 0.50 0.9"};
    std::vector<std::string> failures;
    size_t refused = 0, sliced_cells = 0;
    for (const SweepPair &pair : pairs) {
        const double fine_h = 0.5 * pair.fine_d;
        const int    ratio  = int(std::floor(0.7 * pair.coarse_d / fine_h + 1e-9));
        for (int r = 2; r <= ratio; ++r)
            for (double strip_z : {0.5, 0.9, 1.3}) {
                const double coarse_h = fine_h * r;
                std::ostringstream cell;
                cell << pair.name << " " << std::fixed << std::setprecision(2) << coarse_h << " " << std::setprecision(1) << strip_z;
                CadenceTest::Scene scene;
                scene.config = four_filament_body_config(pair.coarse_d, 0.2 * pair.coarse_d, 0.7 * pair.coarse_d, fine_h, coarse_h);
                const std::vector<double> nozzles = pair.swapped ? std::vector<double>{pair.coarse_d, pair.fine_d} :
                                                                   std::vector<double>{pair.fine_d, pair.coarse_d};
                scene.config.set_key_value("nozzle_diameter", new ConfigOptionFloats(nozzles));
                scene.config.set_key_value("min_layer_height", new ConfigOptionFloats{0.2 * nozzles[0], 0.2 * nozzles[1]});
                scene.config.set_key_value("max_layer_height", new ConfigOptionFloats{0.7 * nozzles[0], 0.7 * nozzles[1]});
                scene.config.set_key_value("mixed_nozzle_process_nozzle_diameters", new ConfigOptionFloats(nozzles));
                if (pair.swapped)
                    scene.config.set_key_value("filament_map", new ConfigOptionInts{2, 1, 2, 1});
                scene.populate = [coarse_h, fine_h, strip_z](Model &model, Print &print, const DynamicPrintConfig &config) {
                    add_fine_skins_plate(model, print, config, fine_h, coarse_h, strip_z);
                };
                std::string message;
                bool sliced = false;
                try {
                    const CadenceTest::Facts facts = CadenceTest::slice(scene);
                    message = facts.refusal.string;
                    sliced  = message.empty() && !facts.gcode.empty();
                } catch (const std::exception &error) {
                    message = error.what();
                }
                const bool expect_refusal = refused_cells.count(cell.str()) != 0;
                const bool is_a60 = message.find("SRL-A60") != std::string::npos;
                if (expect_refusal ? !is_a60 : !sliced)
                    failures.push_back(cell.str() + ": " + (message.empty() ? std::string("sliced") : message));
                refused += is_a60 ? 1 : 0;
                sliced_cells += sliced ? 1 : 0;
            }
    }
    std::ostringstream report;
    for (const std::string &failure : failures)
        report << failure << '\n';
    INFO(report.str());
    CAPTURE(sliced_cells);
    CHECK(failures.empty());
    CHECK(refused == refused_cells.size());
}
