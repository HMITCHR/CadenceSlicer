#include <catch2/catch_all.hpp>

#include "mixed_nozzle_harness.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Geometry/ConvexHull.hpp"
#include "libslic3r/Layer.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;

// The combined test project, as the app saved it: an H2D 0.2/0.6, a Body Split L (a 20 mm
// column, 50 mm tall, with a 40 mm arm whose underside is at 40 mm) on the 0.6 in PLA (filament 7) at
// 0.4 mm, and "CADENCE" text sunk 0.8 mm into the top of the arm, flush, on the 0.2 in PLA (filament
// 1) at 0.1 mm. Organic tree support under the arm with its base in filament 7 and a PETG interface
// (filament 4) on the 0.2. Fine layers 0.1, first layer 0.1, which the 0.6 (minimum 0.12) cannot lay,
// so the L starts on the bed with a 0.2 mm first cell.

namespace {
using Move = GCodeProcessorResult::MoveVertex;

// What each variant is checked for.
enum Check : unsigned {
    NoEmptyLayers  = 1 << 0, // no empty-layer warning
    BandsOnCells   = 1 << 1, // coarse support bands end on the L's own layers
    ChangesInTower = 1 << 2, // a tower on layer 1 and every tool change inside a tower block
    TextOnBase     = 1 << 3, // the L under the text reaches up to the text's bottom
    NoSupportAbove = 1 << 4, // no support roads above `support_above`
    TopSurface     = 1 << 5, // the coarse top around the text is mostly top surface, with no inner walls
    TopGaps        = 1 << 6, // the coarse top leaves no open gaps near the letters
    PocketFill     = 1 << 7, // the pockets the letters wall in get no more plastic than they hold
    GapFillSpeed   = 1 << 8, // the top's gap fill and pockets are laid at gap fill speed
};

struct Variant {
    std::string name;
    double      first_layer = 0.1;
    double      coarse      = 0.4;
    bool        organic     = true;
    bool        support     = true;
    // The fine layer height, for the text and the plate.
    double      fine        = 0.1;
    // The project in tests/data.
    std::string file        = "body-split-sunk-text-L.3mf";
    unsigned    checks      = 0;
    // Moves the text up (mm) and stretches it along X about its own centre, to vary how it meets the base.
    double      text_dz      = 0.;
    double      text_scale_x = 1.;
    double      support_above = 41.;
};

struct SlicedProject {
    Model               model;
    Print               print;
    CadenceTest::Facts  facts;
    std::string         failure;
};

std::unique_ptr<SlicedProject> slice_l_project(const Variant &variant)
{
    auto out = std::make_unique<SlicedProject>();
    DynamicPrintConfig config;
    out->model = CadenceTest::load_project(variant.file, config);
    REQUIRE(out->model.objects.size() == 1);
    REQUIRE(out->model.objects.front()->volumes.size() == 2);
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(variant.first_layer));
    config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(variant.coarse));
    config.set_key_value("enable_support", new ConfigOptionBool(variant.support));
    config.set_key_value("support_type", new ConfigOptionEnum<SupportType>(variant.organic ? stTreeAuto : stNormalAuto));
    config.set_key_value("support_style", new ConfigOptionEnum<SupportMaterialStyle>(variant.organic ? smsTreeOrganic : smsDefault));
    config.set_key_value("layer_height", new ConfigOptionFloat(variant.fine));
    for (ModelVolume *volume : out->model.objects.front()->volumes) {
        const bool base = volume->name == "L" || volume->name == "block";
        volume->config.set_key_value("regional_layer_height", new ConfigOptionFloat(base ? variant.coarse : variant.fine));
        if (!base && variant.text_dz != 0.)
            volume->set_offset(volume->get_offset() + Vec3d(0., 0., variant.text_dz));
        if (!base && variant.text_scale_x != 1.)
            volume->set_scaling_factor(X, volume->get_scaling_factor(X) * variant.text_scale_x);
    }
    out->print.is_BBL_printer() = true;
    out->print.apply(out->model, config);
    out->print.set_status_silent();
    try {
        out->facts = CadenceTest::slice(out->print);
    } catch (const std::exception &error) {
        out->failure = "threw " + std::string(error.what());
        return out;
    }
    if (!out->facts.refusal.string.empty())
        out->failure = "refused: " + out->facts.refusal.string;
    else if (out->facts.gcode.empty())
        out->failure = "no G-code";
    return out;
}

const PrintObject &the_object(const SlicedProject &project) { return *project.print.objects().front(); }

// The region of the coarse L (filament 7), by its cells' height.
size_t coarse_region(const PrintObject &object)
{
    const auto &cells = object.native_regional_grid_state().cells;
    size_t best = 0;
    double tallest = 0.;
    for (size_t region = 0; region < cells.size(); ++region)
        for (const NativeRegionCellState &cell : cells[region])
            if (cell.cell.height > tallest + EPSILON) {
                tallest = cell.cell.height;
                best = region;
            }
    return best;
}

bool is_support(const Move &move)
{
    return move.type == EMoveType::Extrude &&
        (move.extrusion_role == erSupportMaterial || move.extrusion_role == erSupportMaterialInterface ||
         move.extrusion_role == erSupportTransition);
}

std::string empty_layers(const SlicedProject &project)
{
    const auto warnings = project.print.step_state_with_warnings(psGCodeExport).warnings;
    return std::any_of(warnings.begin(), warnings.end(), [](const PrintStateBase::Warning &warning) {
               return warning.message_id == PrintStateBase::SlicingEmptyGcodeLayers;
           }) ? "warns of empty layers" : "";
}

// The support's bed layer is 0.1 on the fine nozzle and the L's first cell is 0.2, so bands counted from the
// support's own first layer ended 0.1 off every L cell: each coarse step printed twice. Below the arm only the
// L prints, so its roads give its layers.
std::string bands_on_cells(const SlicedProject &project, const Variant &variant)
{
    std::set<long> l_layers;
    for (const Move &move : project.facts.moves)
        if (CadenceTest::model_road(move) && move.position.z() < 39.5f)
            l_layers.insert(std::lround(double(move.position.z()) * 1000.));
    size_t bands = 0, off = 0;
    double first_off = 0.;
    for (const Move &move : project.facts.moves)
        if (is_support(move) && move.position.z() < 39.f && double(move.height) > variant.fine + 1e-3) {
            ++bands;
            if (l_layers.count(std::lround(double(move.position.z()) * 1000.)) == 0 && off++ == 0)
                first_off = double(move.position.z());
        }
    std::ostringstream what;
    if (bands == 0 || off != 0)
        what << off << " of " << bands << " coarse support roads off the L's layers, the first at Z " << first_off;
    return what.str();
}

// Layer 1 holds only the support's bed layer; the L starts at 0.2. The tower still starts on layer 1, so every
// nozzle and filament change is primed.
std::string changes_in_tower(const SlicedProject &project, const Variant &variant)
{
    std::ostringstream what;
    if (std::none_of(project.facts.moves.begin(), project.facts.moves.end(), [&variant](const Move &move) {
            return move.type == EMoveType::Extrude && move.extrusion_role == erWipeTower &&
                double(move.position.z()) < variant.first_layer + 1e-3;
        }))
        what << "no tower on layer 1; ";
    std::istringstream gcode(project.facts.gcode);
    std::string line, z = "?", first_off;
    bool printing = false, in_tower = false;
    size_t changes = 0, off_tower = 0;
    while (std::getline(gcode, line)) {
        if (line.rfind("; CHANGE_LAYER", 0) == 0)
            printing = true;
        else if (line.rfind("; MACHINE_END_GCODE_START", 0) == 0)
            break;
        else if (line.rfind("; Z_HEIGHT:", 0) == 0)
            z = line.substr(11);
        else if (line.rfind("; WIPE_TOWER_START", 0) == 0)
            in_tower = true;
        else if (line.rfind("; WIPE_TOWER_END", 0) == 0)
            in_tower = false;
        else if (printing && line.size() > 1 && line[0] == 'T' && std::isdigit((unsigned char) line[1]) &&
                 std::stoi(line.substr(1)) < 255) {
            ++changes;
            if (!in_tower && off_tower++ == 0)
                first_off = z;
        }
    }
    if (changes == 0 || off_tower != 0)
        what << off_tower << " of " << changes << " tool changes off the tower, the first at Z " << first_off;
    return what.str();
}

// Rows of the same letters differ by slicing noise along their sides. Those slivers are not contacts: the L's
// cells inside the letters stay whole, the L reaches the letters' bottom, and no support fills a hollow there.
std::string text_on_base(const SlicedProject &project)
{
    const PrintObject &object = the_object(project);
    const size_t l_region = coarse_region(object);
    // The text's bottom: the lowest layer where a region other than the L prints, minus its cell height.
    double text_bottom = std::numeric_limits<double>::max();
    ExPolygons text_footprint;
    for (const Layer *layer : object.layers())
        for (const LayerRegion *region : layer->regions())
            if (size_t(region->region().print_object_region_id()) != l_region && !region->slices.empty() &&
                layer->print_z - region->height() < text_bottom - EPSILON) {
                text_bottom    = layer->print_z - region->height();
                text_footprint = to_expolygons(region->slices.surfaces);
            }
    if (text_footprint.empty())
        return "no text";
    double l_top_under_text = 0.;
    const ExPolygons inner_text = offset_ex(text_footprint, -float(scale_(0.05)));
    for (const Layer *layer : object.layers())
        for (const LayerRegion *region : layer->regions())
            if (size_t(region->region().print_object_region_id()) == l_region && layer->print_z < text_bottom + 0.5 &&
                !intersection_ex(to_expolygons(region->slices.surfaces), inner_text).empty())
                l_top_under_text = std::max(l_top_under_text, layer->print_z);
    std::ostringstream what;
    if (std::abs(l_top_under_text - text_bottom) > EPSILON)
        what << "the L under the text ends at " << l_top_under_text << ", the text starts at " << text_bottom;
    return what.str();
}

std::string support_above(const SlicedProject &project, double z)
{
    size_t above = 0;
    double lowest = std::numeric_limits<double>::max();
    for (const Move &move : project.facts.moves)
        if (move.type == EMoveType::Extrude &&
            (move.extrusion_role == erSupportMaterial || move.extrusion_role == erSupportMaterialInterface) &&
            double(move.position.z()) > z) {
            ++above;
            lowest = std::min(lowest, double(move.position.z()));
        }
    std::ostringstream what;
    if (above != 0)
        what << above << " support roads above Z " << z << ", the lowest at Z " << lowest;
    return what.str();
}

// A coarse Body Split body lays its top surfaces around the letters with one wall and leaves the rest to the top
// surface pattern, not rings of walls around every letter.
std::string top_surface(const SlicedProject &project)
{
    // The L's top: the highest extrusion off the tower.
    double top_z = 0.;
    for (const Move &move : project.facts.moves)
        if (move.type == EMoveType::Extrude && move.extrusion_role != erWipeTower)
            top_z = std::max(top_z, double(move.position.z()));
    // Area laid by the 0.6 (roads wider than the 0.2 can lay) on the top layer, by role.
    std::map<ExtrusionRole, double> area;
    const Move *previous = nullptr;
    for (const Move &move : project.facts.moves) {
        if (previous != nullptr && move.type == EMoveType::Extrude && move.extrusion_role != erWipeTower &&
            std::abs(double(move.position.z()) - top_z) < 1e-3 && move.width > 0.45f)
            area[move.extrusion_role] += double((move.position - previous->position).head<2>().norm()) * double(move.width);
        previous = &move;
    }
    double total = 0.;
    for (const auto &[role, value] : area)
        total += value;
    const double top_share = total > 0. ? area[erTopSolidInfill] / total : 0.;
    std::ostringstream what;
    if (total < 500. || top_share < 0.6 || area[erPerimeter] > 0.)
        what << "top Z " << top_z << ", coarse area " << total << " mm2, top surface share " << top_share
             << ", inner walls " << area[erPerimeter] << " mm2";
    return what.str();
}

// What the coarse body leaves open on its top layer within 4 mm of the text: the base's top outline less the letters (grown 0.05 mm) and its edge (0.11 mm), minus the area its roads
// cover at their width.
struct CoarseTopGaps {
    double top_z   = 0.;
    double near    = 0.; // mm2 of base top within 4 mm of the letters
    double open    = 0.; // mm2 of it no road covers
    double largest = 0.;
    size_t spots   = 0;  // open spots of 0.1 mm2 or more
};

// The area each road covers, by role: its centreline grown by half its width with round ends and joins, as
// the plastic lies (ExtrusionPath::polygons_covered_by_width cuts each piece square, which leaves notches
// where a wall is split into pieces).
void road_cover(const ExtrusionEntity &entity, std::map<ExtrusionRole, Polygons> &out)
{
    auto path_cover = [&out](const ExtrusionPath &path) {
        const Polyline line = path.polyline.to_polyline();
        if (line.size() >= 2)
            polygons_append(out[path.role()], offset(line, float(scale_(path.width / 2.)), jtRound,
                                                     double(scale_(0.005)), etOpenRound));
    };
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
        for (const ExtrusionEntity *child : collection->entities)
            road_cover(*child, out);
    } else if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity))
        path_cover(*path);
    else if (const auto *multipath = dynamic_cast<const ExtrusionMultiPath *>(&entity)) {
        for (const ExtrusionPath &piece : multipath->paths)
            path_cover(piece);
    } else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity)) {
        for (const ExtrusionPath &piece : loop->paths)
            path_cover(piece);
    }
}

CoarseTopGaps coarse_top_gaps(const PrintObject &object, size_t l_region)
{
    CoarseTopGaps gaps;
    const Layer       *top   = nullptr;
    const LayerRegion *l_top = nullptr;
    for (const Layer *layer : object.layers())
        for (const LayerRegion *region : layer->regions())
            if (size_t(region->region().print_object_region_id()) == l_region && !region->perimeters.empty() &&
                (top == nullptr || layer->print_z > top->print_z)) {
                top   = layer;
                l_top = region;
            }
    if (top == nullptr)
        return gaps;
    gaps.top_z = top->print_z;
    // The letters: the text in the base's top cell (sunk) or in the cell above it (raised).
    const double cell = l_top->height();
    ExPolygons letters;
    for (const Layer *layer : object.layers())
        if (layer->print_z > top->print_z - cell + EPSILON && layer->print_z < top->print_z + cell + EPSILON)
            for (const LayerRegion *region : layer->regions())
                if (size_t(region->region().print_object_region_id()) != l_region)
                    append(letters, to_expolygons(region->slices.surfaces));
    letters = union_ex(letters);
    std::map<ExtrusionRole, Polygons> roles;
    road_cover(l_top->perimeters, roles);
    road_cover(l_top->fills, roles);
    road_cover(l_top->thin_fills, roles);
    Polygons covered;
    for (const auto &[role, polygons] : roles)
        append(covered, polygons);
    ExPolygons outline = to_expolygons(l_top->slices.surfaces);
    append(outline, letters);
    const ExPolygons target = offset_ex(union_ex(outline), -float(scale_(0.11)));
    const ExPolygons near   = intersection_ex(diff_ex(target, offset_ex(letters, float(scale_(0.05)))),
                                              offset_ex(letters, float(scale_(4.))));
    const double mm2 = SCALING_FACTOR * SCALING_FACTOR;
    for (const ExPolygon &piece : near)
        gaps.near += piece.area() * mm2;
    const ExPolygons open = diff_ex(near, union_(covered));
    for (const ExPolygon &hole : open) {
        const double a = hole.area() * mm2;
        gaps.open += a;
        gaps.largest = std::max(gaps.largest, a);
        if (a >= 0.1)
            ++gaps.spots;
    }
    return gaps;
}

// No open crescents or wedges where the last top line meets a curved or angled wall, and no lines between walls
// just under the smallest gap the walls' gap fill takes. Letter notches narrower than a 0.6 road may stay.
std::string top_gaps(const SlicedProject &project)
{
    const CoarseTopGaps gaps = coarse_top_gaps(the_object(project), coarse_region(the_object(project)));
    std::ostringstream what;
    if (gaps.near < 50. || gaps.open >= 1.)
        what << "top Z " << gaps.top_z << ", base top near the letters " << gaps.near << " mm2, open " << gaps.open
             << " mm2 in " << gaps.spots << " spots, largest " << gaps.largest;
    return what.str();
}

// How full the coarse body's top layer lays each pocket the text walls in: for each letter, its convex hull
// less the letters (the counter, open or closed), and for each pair of neighbouring letters their joint hull
// less the letters and both counters (the space between them), each cut to the part the letters wall in (the
// letters' outline closed by 2.5 mm; a hull also takes in open ground past the letters' tips, where extra
// plastic can spread). Fill is the coarse roads' volume there (the centreline inside, times the road's mm3
// per mm, from the G-code) over the pocket's area times the layer height.
struct PocketVolume {
    std::string name;
    double      area = 0.;
    double      fill = 0.;
};

// The plastic the coarse nozzle lays inside region on the layer at top_z, from the G-code moves (each move's
// length inside times its mm3 per mm, flow ratios included).
double volume_inside(const std::vector<Move> &moves, double top_z, const Point &shift, const ExPolygons &region)
{
    // The coarse nozzle: the one laying top surface with roads wider than the 0.2 can lay.
    int extruder = -1;
    for (const Move &move : moves)
        if (move.type == EMoveType::Extrude && move.extrusion_role == erTopSolidInfill && move.width > 0.45f &&
            std::abs(double(move.position.z()) - top_z) < 1e-3)
            extruder = move.extruder_id;
    double      volume   = 0.;
    const Move *previous = nullptr;
    for (const Move &move : moves) {
        if (previous != nullptr && move.type == EMoveType::Extrude && int(move.extruder_id) == extruder &&
            move.extrusion_role != erWipeTower && std::abs(double(move.position.z()) - top_z) < 1e-3) {
            // The moves are on the bed, the region in the object's own coordinates.
            const Polyline line(Point(Point::new_scale(previous->position.x(), previous->position.y()) - shift),
                                Point(Point::new_scale(move.position.x(), move.position.y()) - shift));
            for (const Polyline &piece : intersection_pl(Polylines{line}, region))
                volume += unscale<double>(piece.length()) * double(move.mm3_per_mm);
        }
        previous = &move;
    }
    return volume;
}

std::vector<PocketVolume> coarse_top_pocket_fill(const PrintObject &object, size_t l_region, const std::vector<Move> &moves)
{
    const Layer       *top   = nullptr;
    const LayerRegion *l_top = nullptr;
    for (const Layer *layer : object.layers())
        for (const LayerRegion *region : layer->regions())
            if (size_t(region->region().print_object_region_id()) == l_region && !region->perimeters.empty() &&
                (top == nullptr || layer->print_z > top->print_z)) {
                top   = layer;
                l_top = region;
            }
    std::vector<PocketVolume> out;
    if (top == nullptr)
        return out;
    ExPolygons letters;
    for (const LayerRegion *region : top->regions())
        if (size_t(region->region().print_object_region_id()) != l_region)
            append(letters, to_expolygons(region->slices.surfaces));
    letters = union_ex(letters);
    std::sort(letters.begin(), letters.end(), [](const ExPolygon &a, const ExPolygon &b) {
        return get_extents(a).min.x() < get_extents(b).min.x();
    });
    const ExPolygons base = intersection_ex(to_expolygons(l_top->slices.surfaces), closing_ex(letters, float(scale_(2.5))));
    const double     mm2  = SCALING_FACTOR * SCALING_FACTOR;
    auto hull_less = [&](const Polygons &hull_of, const ExPolygons &also_less) {
        ExPolygons less = letters;
        append(less, also_less);
        return intersection_ex(diff_ex(ExPolygons{ExPolygon(Geometry::convex_hull(hull_of))}, less), base);
    };
    auto measure = [&](const std::string &name, const ExPolygons &pocket) {
        double area = 0.;
        for (const ExPolygon &piece : pocket)
            area += piece.area() * mm2;
        if (area < 5.)
            return;
        const double volume = volume_inside(moves, top->print_z, object.instances().front().shift, pocket);
        out.push_back({name, area, volume / (area * l_top->height())});
    };
    std::vector<ExPolygons> counters;
    for (size_t i = 0; i < letters.size(); ++i) {
        counters.push_back(hull_less(to_polygons(letters[i]), {}));
        measure("counter " + std::to_string(i + 1), counters.back());
    }
    for (size_t i = 0; i + 1 < letters.size(); ++i) {
        Polygons pair = to_polygons(letters[i]);
        append(pair, to_polygons(letters[i + 1]));
        ExPolygons both = counters[i];
        append(both, counters[i + 1]);
        measure("between " + std::to_string(i + 1) + "-" + std::to_string(i + 2), hull_less(pair, both));
    }
    return out;
}

// A pocket the text walls in at full height has nowhere to put extra plastic, so it gets at most what it holds.
std::string pocket_fill(const SlicedProject &project)
{
    const std::vector<PocketVolume> pockets = coarse_top_pocket_fill(the_object(project), coarse_region(the_object(project)),
                                                                   project.facts.moves);
    double fullest = 0., emptiest = 10.;
    std::ostringstream what;
    for (const PocketVolume &pocket : pockets) {
        what << " " << pocket.name << " " << pocket.fill << ";";
        fullest  = std::max(fullest, pocket.fill);
        emptiest = std::min(emptiest, pocket.fill);
    }
    // Seven letters: seven counters (the E's arms count) and six spaces between them.
    if (pockets.size() < 12 || fullest > 1.0 || emptiest < 0.85)
        return std::to_string(pockets.size()) + " pockets:" + what.str();
    return "";
}

// The top's gap fill and its top lines inside the pockets run at no more than 50 mm/s; the open top keeps the
// preset's speed.
std::string gap_fill_speed(const SlicedProject &project)
{
    // The coarse top: the highest top surface laid with roads wider than the 0.2 can lay.
    double top_z    = 0.;
    int    extruder = -1;
    for (const Move &move : project.facts.moves)
        if (move.type == EMoveType::Extrude && move.extrusion_role == erTopSolidInfill && move.width > 0.45f &&
            double(move.position.z()) > top_z) {
            top_z    = double(move.position.z());
            extruder = move.extruder_id;
        }
    size_t gap_fill = 0;
    double fastest_gap_fill = 0., slow_top = 0., fast_top = 0.;
    const Move *previous = nullptr;
    for (const Move &move : project.facts.moves) {
        if (previous != nullptr && move.type == EMoveType::Extrude && int(move.extruder_id) == extruder &&
            std::abs(double(move.position.z()) - top_z) < 1e-3) {
            const double length = double((move.position - previous->position).head<2>().norm());
            if (move.extrusion_role == erGapFill) {
                ++gap_fill;
                fastest_gap_fill = std::max(fastest_gap_fill, double(move.feedrate));
            } else if (move.extrusion_role == erTopSolidInfill)
                (move.feedrate <= 50.01f ? slow_top : fast_top) += length;
        }
        previous = &move;
    }
    std::ostringstream what;
    if (gap_fill == 0 || fastest_gap_fill > 50.01 || slow_top < 10. || fast_top < slow_top)
        what << "top Z " << top_z << ", " << gap_fill << " gap fill moves, fastest " << fastest_gap_fill
             << " mm/s; top surface at 50 mm/s or less " << slow_top << " mm, faster " << fast_top << " mm";
    return what.str();
}
} // namespace

// Each saved project variant is sliced once and checked for what it covers. Failures are collected and
// reported together.
TEST_CASE("Text sunk into a Body Split base sits on the base, and no support grows under it", "[TestRebuild][Support][BodySplit]")
{
    const std::string organic_012 = "body-split-L-organic-fine012.3mf";
    const std::string raised_0208 = "body-split-L-text-raised-0208.3mf";
    const std::string sunk_0208   = "body-split-L-text-sunk-0208.3mf";
    const std::string thin_text   = "body-split-block-thin-text.3mf";
    const auto variant = [](std::string name, unsigned checks) {
        Variant out;
        out.name   = std::move(name);
        out.checks = checks;
        return out;
    };
    Variant normal_040 = variant("normal 0.10/0.40", NoEmptyLayers | BandsOnCells | ChangesInTower);
    normal_040.organic = false;
    Variant organic_030 = variant("organic 0.10/0.30", NoEmptyLayers | BandsOnCells | ChangesInTower);
    organic_030.coarse = 0.3;
    Variant normal_030 = variant("normal 0.10/0.30", NoEmptyLayers | BandsOnCells | TextOnBase | NoSupportAbove);
    normal_030.coarse  = 0.3;
    normal_030.organic = false;
    Variant first_012 = variant("organic 0.12/0.40", ChangesInTower | TextOnBase | NoSupportAbove | TopSurface);
    first_012.first_layer = 0.12;
    Variant bare_040 = variant("no support 0.10/0.40", TextOnBase | NoSupportAbove);
    bare_040.support = false;
    Variant bare_030 = variant("no support 0.10/0.30", TopSurface);
    bare_030.support = false;
    bare_030.coarse  = 0.3;
    // Fine layers 0.12 from a 0.10 first layer, coarse 0.36: a branch that widens inside a band is not left as a
    // fine piece with nothing under it.
    std::vector<Variant> variants{
        variant("organic 0.10/0.40", NoEmptyLayers | BandsOnCells | ChangesInTower | TextOnBase | NoSupportAbove |
                                     TopSurface | TopGaps | PocketFill | GapFillSpeed),
        normal_040, organic_030, normal_030, first_012, bare_040, bare_030,
        Variant{"organic 0.10/0.36 fine 0.12", 0.1, 0.36, true, true, 0.12, organic_012, NoEmptyLayers},
        Variant{"organic 0.12/0.36 fine 0.12", 0.12, 0.36, true, true, 0.12, organic_012, NoEmptyLayers},
        // 0.2/0.8 at fine 0.08: the 0.8 needs two fine layers per cell, and the text's bottom sits one fine layer
        // off those cells' tops.
        Variant{"raised 0.10/0.16 fine 0.08", 0.1, 0.16, true, true, 0.08, raised_0208, NoEmptyLayers | TextOnBase | TopGaps},
        Variant{"sunk 0.10/0.16 fine 0.08", 0.1, 0.16, true, true, 0.08, sunk_0208, NoEmptyLayers | TextOnBase},
        // Thin strokes on a block 10 mm tall: no support between the letters on top of the block.
        Variant{"thin text, normal 0.12/0.36", 0.12, 0.36, false, true, 0.12, thin_text, NoSupportAbove, 0., 1., 10.05},
        Variant{"thin text, normal 0.10/0.36", 0.1, 0.36, false, true, 0.12, thin_text, NoSupportAbove, 0., 1., 10.05},
    };
    // The saved plate with its text moved or stretched.
    struct Moved { const char *name; double dz; double scale_x; unsigned checks; };
    for (const Moved &text : {Moved{"letters 0.95 wide", 0., 0.95, TopGaps}, Moved{"letters 1.05 wide", 0., 1.05, TopGaps},
                              Moved{"half sunk (0.4 mm proud)", 0.4, 1., TopGaps | PocketFill | GapFillSpeed},
                              Moved{"raised on the top", 0.8, 1., TopGaps}}) {
        Variant moved = variant(text.name, text.checks);
        moved.text_dz      = text.dz;
        moved.text_scale_x = text.scale_x;
        variants.push_back(moved);
    }

    std::vector<std::string> failures;
    for (const Variant &v : variants) {
        const std::unique_ptr<SlicedProject> project = slice_l_project(v);
        if (!project->failure.empty()) {
            failures.push_back(v.name + ": " + project->failure);
            continue;
        }
        const std::pair<Check, std::function<std::string()>> checks[] = {
            {NoEmptyLayers, [&] { return empty_layers(*project); }},
            {BandsOnCells, [&] { return bands_on_cells(*project, v); }},
            {ChangesInTower, [&] { return changes_in_tower(*project, v); }},
            {TextOnBase, [&] { return text_on_base(*project); }},
            {NoSupportAbove, [&] { return support_above(*project, v.support_above); }},
            {TopSurface, [&] { return top_surface(*project); }},
            {TopGaps, [&] { return top_gaps(*project); }},
            {PocketFill, [&] { return pocket_fill(*project); }},
            {GapFillSpeed, [&] { return gap_fill_speed(*project); }},
        };
        for (const auto &[check, run] : checks)
            if ((v.checks & check) != 0)
                if (const std::string failure = run(); !failure.empty())
                    failures.push_back(v.name + ": " + failure);
    }
    std::ostringstream report;
    for (const std::string &failure : failures)
        report << failure << '\n';
    INFO(report.str());
    CHECK(failures.empty());
}
