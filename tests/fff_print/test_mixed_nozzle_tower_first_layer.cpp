// Prime tower: feedrate after an in-tower travel, the bed level under a fine switch on
// layer 1, and the coarse nozzle's first-layer wall width and ramming speed.

#include <catch2/catch_all.hpp>

#include "mixed_nozzle_harness.hpp"
#include "mixed_nozzle_fixtures.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace mixed_nozzle_fixtures;

namespace {
using Move = GCodeProcessorResult::MoveVertex;

// One-based G-code line numbers (gcode_id) that sit inside a nozzle-change (ramming) bracket.
std::vector<bool> ramming_lines(const std::string &gcode)
{
    std::vector<bool> out(1, false);
    std::istringstream lines(gcode);
    std::string line;
    bool ramming = false;
    while (std::getline(lines, line)) {
        if (line.rfind("; NOZZLE_CHANGE_START", 0) == 0)
            ramming = true;
        out.push_back(ramming);
        if (line.rfind("; NOZZLE_CHANGE_END", 0) == 0)
            ramming = false;
    }
    return out;
}

// With CADENCE_TEST_DUMP_DIR set, keeps a case's G-code there for inspection.
void dump_gcode(const std::string &name, const std::string &gcode)
{
    if (const char *dir = std::getenv("CADENCE_TEST_DUMP_DIR")) {
        std::ofstream out(std::string(dir) + "/" + name + ".gcode");
        out << gcode;
    }
}

CadenceTest::Scene feature_tower_scene(double fine_d, double coarse_d, double height)
{
    CadenceTest::Scene scene;
    scene.config = feature_split_tower_config(fine_d, coarse_d, false);
    scene.config.set_key_value("wipe_tower_wall_type", new ConfigOptionEnum<WipeTowerWallType>(wtwRib));
    scene.config.set_key_value("mixed_nozzle_auto_pad", new ConfigOptionBool(true));
    scene.config.set_key_value("prime_tower_width", new ConfigOptionFloat(60.));
    scene.config.set_key_value("prime_tower_brim_width", new ConfigOptionFloat(-1.));
    scene.config.set_key_value("first_layer_flow_ratio", new ConfigOptionFloat(1.));
    apply_tower_exit_retraction(scene.config);
    fill_per_filament_values(scene.config);
    scene.populate = [height](Model &model, Print &print, const DynamicPrintConfig &config) {
        init_feature_flow_fixture(model, print, config, height, 60.);
    };
    return scene;
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
    const std::vector<bool> ramming = ramming_lines(facts.gcode);
    // Lines inside a tool change or a block fill: what is left of the tower is its wall and brim.
    std::vector<bool> not_wall(1, false);
    {
        std::istringstream lines(facts.gcode);
        std::string line;
        bool inside = false;
        while (std::getline(lines, line)) {
            if (line.rfind("; CP TOOLCHANGE START", 0) == 0 || line.rfind("; CP EMPTY GRID START", 0) == 0)
                inside = true;
            not_wall.push_back(inside);
            if (line.rfind("; CP TOOLCHANGE END", 0) == 0 || line.rfind("; CP EMPTY GRID END", 0) == 0)
                inside = false;
        }
    }
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
    double min_x[2] = {1e9, 1e9}, max_x[2] = {-1e9, -1e9}, min_y[2] = {1e9, 1e9}, max_y[2] = {-1e9, -1e9};
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
            if (z > 2.) {
                min_x[1] = std::min<double>(min_x[1], move.position.x());
                max_x[1] = std::max<double>(max_x[1], move.position.x());
                min_y[1] = std::min<double>(min_y[1], move.position.y());
                max_y[1] = std::max<double>(max_y[1], move.position.y());
            }
            continue;
        }
        const bool on_first = z <= first_layer_height + 0.02;
        const bool rams     = move.gcode_id < ramming.size() && ramming[move.gcode_id];
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
            if (move.physical_tool_id < 2 && ! rams && move.gcode_id < not_wall.size() && ! not_wall[move.gcode_id])
                audit.wall_widest[move.physical_tool_id] = std::max(audit.wall_widest[move.physical_tool_id], double(move.width));
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
        if (on_first) {
            min_x[0] = std::min<double>(min_x[0], move.position.x());
            max_x[0] = std::max<double>(max_x[0], move.position.x());
            min_y[0] = std::min<double>(min_y[0], move.position.y());
            max_y[0] = std::max<double>(max_y[0], move.position.y());
        }
        for (const auto &point : points) {
            float &cell_top = top[key(point.first, point.second)];
            cell_top = std::max(cell_top, float(z));
        }
    }
    audit.first_extent = std::min(max_x[0] - min_x[0], max_y[0] - min_y[0]);
    audit.top_extent   = std::min(max_x[1] - min_x[1], max_y[1] - min_y[1]);
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

// Three blocks side by side on the bed: filament 1 and filament 3 on the fine nozzle (a fine switch
// on layer 1) and filament 2 on the coarse nozzle, whose first layer ends at the second fine layer.
CadenceTest::Facts slice_fine_switch_on_layer_one(double coarse_d, double coarse_min, double coarse_max, double fine, double coarse,
                                                  bool swapped = false, std::vector<int> filaments = {1, 3, 2},
                                                  double fine_lift = 0., WipeTowerWallType wall_type = wtwRib)
{
    CadenceTest::Scene scene;
    scene.config = four_filament_body_config(coarse_d, coarse_min, coarse_max, fine, coarse);
    scene.config.set_key_value("wipe_tower_wall_type", new ConfigOptionEnum<WipeTowerWallType>(wall_type));
    if (swapped) {
        scene.config.set_key_value("nozzle_diameter", new ConfigOptionFloats{coarse_d, 0.2});
        scene.config.set_key_value("min_layer_height", new ConfigOptionFloats{coarse_min, 0.04});
        scene.config.set_key_value("max_layer_height", new ConfigOptionFloats{coarse_max, 0.14});
        scene.config.set_key_value("mixed_nozzle_process_nozzle_diameters", new ConfigOptionFloats{coarse_d, 0.2});
        scene.config.set_key_value("filament_map", new ConfigOptionInts{2, 1, 2, 1});
    }
    scene.populate = [fine, coarse, filaments, fine_lift](Model &model, Print &print, const DynamicPrintConfig &config) {
        ModelObject *object = model.add_object();
        object->name = "fine-switch-on-layer-one";
        for (size_t i = 0; i < filaments.size(); ++i) {
            ModelVolume *volume = object->add_volume(make_cube(10., 12., 3.), ModelVolumeType::MODEL_PART, false);
            // Fine bodies can start above the bed (on the coarse one's neighbour column they are simply later).
            volume->set_offset(Vec3d(10. * double(i), 0., filaments[i] % 2 == 0 ? 0. : fine_lift));
            volume->config.set_key_value("extruder", new ConfigOptionInt(filaments[i]));
            // Even filaments are on the coarse nozzle.
            volume->config.set_key_value("regional_layer_height", new ConfigOptionFloat(filaments[i] % 2 == 0 ? coarse : fine));
        }
        object->add_instance();
        object->instances.front()->set_offset(Vec3d(60., 60., 0.));
        object->ensure_on_bed();
        print.apply(model, config);
        print.set_status_silent();
    };
    return CadenceTest::slice(scene);
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
    const std::vector<bool> ramming = ramming_lines(facts.gcode);
    // Lines inside a block fill.
    std::vector<bool> block_fill(1, false);
    {
        std::istringstream lines(facts.gcode);
        std::string line;
        bool inside = false;
        while (std::getline(lines, line)) {
            if (line.rfind("; CP EMPTY GRID START", 0) == 0)
                inside = true;
            block_fill.push_back(inside);
            if (line.rfind("; CP EMPTY GRID END", 0) == 0)
                inside = false;
        }
    }
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
                         area, int(move.physical_tool_id), move.gcode_id < ramming.size() && ramming[move.gcode_id],
                         move.gcode_id < block_fill.size() && block_fill[move.gcode_id]});
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
} // namespace

// In a mixed mode the tower's travels name the tower travel speed. A road after one that
// names no feedrate must go back to the print feedrate, not run at the travel speed (which the
// volumetric ceiling then cuts down to whatever the filament allows, instead of the intended speed).
TEST_CASE("Feature Split: a tower road that names no feedrate does not run at the tower's travel speed",
          "[TestRebuild][TowerSpeed]")
{
    CadenceTest::Scene scene = feature_tower_scene(.20, .60, 4.);
    // A limit no tower road reaches, so the volumetric ceiling cannot hide an inherited travel speed.
    scene.config.set_key_value("filament_max_volumetric_speed", new ConfigOptionFloatsNullable{200., 200.});
    const double travel = scene.config.option<ConfigOptionFloatsNullable>("travel_speed")->get_at(0);
    const CadenceTest::Facts facts = CadenceTest::slice(scene);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    const std::vector<bool> ramming = ramming_lines(facts.gcode);
    size_t roads = 0, at_travel_speed = 0;
    double fastest = 0.;
    for (const Move &move : facts.moves) {
        if (move.type != EMoveType::Extrude || move.extrusion_role != erWipeTower)
            continue;
        if (move.gcode_id < ramming.size() && ramming[move.gcode_id])
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

// A fine switch on layer 1 under a coarse arrival on layer 2 used to lay only its prime
// road on the bed. The coarse arrival then laid a step from the bed over mostly bare bed, at full
// speed, and the next fine level printed at the bed height again.
TEST_CASE("A fine switch on layer 1 lays the tower's whole bed level; nothing higher goes down over bare bed",
          "[TestRebuild][TowerFirstLayer]")
{
    struct Pair { const char *name; double coarse_d, coarse_min, coarse_max, fine, coarse; bool swapped; };
    const Pair pair = GENERATE(Pair{"0.2/0.6", 0.6, 0.12, 0.42, 0.10, 0.20, false},
                               Pair{"0.2/0.8", 0.8, 0.16, 0.56, 0.08, 0.16, false},
                               Pair{"0.2/0.6, fine on the right", 0.6, 0.12, 0.42, 0.10, 0.20, true});
    CAPTURE(pair.name);
    const CadenceTest::Facts facts = slice_fine_switch_on_layer_one(pair.coarse_d, pair.coarse_min, pair.coarse_max,
                                                                    pair.fine, pair.coarse, pair.swapped);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    dump_gcode(std::string("bed-level-") + (pair.swapped ? "swapped-" : "") + std::to_string(int(pair.coarse_d * 10)), facts.gcode);
    const TowerBedAudit audit = audit_tower_bed(facts, pair.fine);
    INFO(audit.levels);
    CAPTURE(audit.first_z, audit.near_bed_length, audit.over_bare_bed, audit.from_bed_late, audit.bed_contact_fast,
            audit.first_extent, audit.top_extent);
    CHECK(std::abs(audit.first_z - pair.fine) < 1e-3);
    CHECK(audit.over_bare_bed <= 0.02 * audit.near_bed_length);
    // No later level is laid as a step from the bed, into or beside the bed level.
    CHECK(audit.from_bed_late == 0.);
    CHECK(audit.bed_contact_fast == 0.);
    // The bed level carries the brim (3 mm a side on a short tower), so it is wider than the tower.
    CHECK(audit.first_extent > audit.top_extent + 4.);
}

// When both nozzles lay the tower's bed level (a nozzle change on layer 1), the coarse
// tool's wall there is at its own width, not the fine nozzle's, and ramming laid on the bed goes
// no faster than the rest of the first layer.
TEST_CASE("A nozzle change on layer 1: the coarse tool's bed wall is at its own width and bed ramming is at first-layer speed",
          "[TestRebuild][TowerFirstLayer]")
{
    // 0.12 fine layers: the 0.6 nozzle's first legal layer is the print's first layer.
    const bool swapped = GENERATE(false, true);
    CAPTURE(swapped);
    const CadenceTest::Facts facts = slice_fine_switch_on_layer_one(0.6, 0.12, 0.42, 0.12, 0.24, swapped, {1, 2});
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    dump_gcode(std::string("nozzle-change-l1-") + (swapped ? "swapped" : "plain"), facts.gcode);
    const TowerBedAudit audit = audit_tower_bed(facts, 0.12);
    const int coarse_tool = swapped ? 0 : 1;
    INFO(audit.levels);
    CAPTURE(audit.first_z, audit.bed_ramming_fast, audit.bed_contact_fast, audit.wall_widest[0], audit.wall_widest[1],
            audit.bed_tools.size(), coarse_tool);
    // The scene: both nozzles lay tower roads on the first layer.
    REQUIRE(audit.bed_tools.size() == 2);
    CHECK(audit.bed_ramming_fast == 0.);
    CHECK(audit.bed_contact_fast == 0.);
    // A wall drawn by the coarse tool is at least about its nozzle wide; drawn by the fine tool there is none to check.
    if (audit.wall_widest[coarse_tool] > 0.)
        CHECK(audit.wall_widest[coarse_tool] >= 0.9 * 0.6);
}

// A print that starts on the coarse nozzle has the coarse tool lay the tower's bed level.
// Its wall there was drawn at the fine nozzle's first-layer width (0.25 mm from a 0.6 mm nozzle).
TEST_CASE("A tower bed level laid by the coarse tool has its wall at the coarse tool's width", "[TestRebuild][TowerFirstLayer]")
{
    const bool swapped = GENERATE(false, true);
    CAPTURE(swapped);
    // The coarse body stands on the bed; the fine body starts 1.2 mm up, so layer 1 is coarse only.
    const CadenceTest::Facts facts = slice_fine_switch_on_layer_one(0.6, 0.12, 0.42, 0.12, 0.24, swapped, {2, 1}, 1.2);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    dump_gcode(std::string("coarse-bed-level-") + (swapped ? "swapped" : "plain"), facts.gcode);
    const TowerBedAudit audit = audit_tower_bed(facts, 0.12);
    const int coarse_tool = swapped ? 0 : 1;
    INFO(audit.levels);
    CAPTURE(audit.first_z, audit.bed_contact_fast, audit.wall_widest[0], audit.wall_widest[1], audit.bed_tools.size(), coarse_tool);
    // The scene: only the coarse tool lays the first tower layer.
    REQUIRE(audit.bed_tools.size() == 1);
    REQUIRE(audit.bed_tools.count(coarse_tool) == 1);
    CHECK(audit.wall_widest[coarse_tool] >= 0.9 * 0.6);
    CHECK(audit.bed_contact_fast == 0.);
}

// A coarse body on the bed whose nozzle cannot lay the 0.10 mm first layer starts with a 0.20 mm
// first cell, so nothing prints on the first layer. The tower was then left out altogether, and
// every nozzle change went without a prime.
TEST_CASE("A coarse body that starts above an empty first layer still gets the prime tower, primed at every nozzle change",
          "[TestRebuild][TowerFirstLayer]")
{
    const bool swapped = GENERATE(false, true);
    CAPTURE(swapped);
    // 0.10 fine, 0.30 coarse on a 0.6 nozzle with a 0.12 minimum; the fine body starts 1.2 mm up.
    const CadenceTest::Facts facts = slice_fine_switch_on_layer_one(0.6, 0.12, 0.42, 0.10, 0.30, swapped, {2, 1}, 1.2);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    dump_gcode(std::string("coarse-first-cell-") + (swapped ? "swapped" : "plain"), facts.gcode);

    double first_model_z = 1e9;
    for (const Move &move : facts.moves)
        if (CadenceTest::model_road(move))
            first_model_z = std::min(first_model_z, double(move.position.z()));
    const TowerBedAudit audit = audit_tower_bed(facts, first_model_z);
    INFO(audit.levels);
    CAPTURE(first_model_z, audit.first_z, audit.first_extent, audit.top_extent);
    // The scene: the first printing layer is the coarse first cell, above the 0.10 mm first layer.
    REQUIRE(first_model_z > 0.15);
    REQUIRE(audit.first_z < 1e8);
    // The tower starts with the part, and its bed level is wider than the tower above (its brim).
    CHECK(std::abs(audit.first_z - first_model_z) < 1e-3);
    CHECK(audit.first_extent > audit.top_extent + 1.);

    // Every nozzle change starts on the tower: the new nozzle's first road there is a tower road.
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
    CAPTURE(changes, primed);
    CHECK(changes > 0);
    CHECK(primed == changes);
}

// A coarse tool laying the tower's bed level draws its wall at its own width. Resetting the flow
// after the wall dropped the first-layer boost from the brim loops laid next.
TEST_CASE("A tower bed level laid by the coarse tool keeps the brim's first-layer flow boost",
          "[TestRebuild][TowerFirstLayer]")
{
    const bool swapped = GENERATE(false, true);
    CAPTURE(swapped);
    // The coarse body stands on the bed with a 0.20 mm first cell; the fine body starts 1.2 mm up.
    const CadenceTest::Facts facts = slice_fine_switch_on_layer_one(0.6, 0.12, 0.42, 0.10, 0.30, swapped, {2, 1}, 1.2, wtwRectangle);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    dump_gcode(std::string("coarse-bed-width-") + (swapped ? "swapped" : "plain"), facts.gcode);
    const int coarse_tool = swapped ? 0 : 1;
    const CoarseBedLevel audit = audit_coarse_bed_level(facts, coarse_tool);
    CAPTURE(audit.first_z, audit.narrowest, audit.brim_width, audit.chamfer_width, audit.fullness, audit.fine_widest);
    // The scene: the coarse tool lays the bed level, and a brim and a chamfer were found.
    REQUIRE(audit.first_z > 0.15);
    REQUIRE(audit.brim_width > 0.);
    REQUIRE(audit.chamfer_width > 0.);
    // The brim is laid with the 1.15 first-layer boost over the chamfer loops above it.
    CHECK_THAT(audit.brim_width / audit.chamfer_width, Catch::Matchers::WithinAbs(1.15, 0.035));
}
