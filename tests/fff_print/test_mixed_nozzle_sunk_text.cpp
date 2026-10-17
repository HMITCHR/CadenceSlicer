#include <catch2/catch_all.hpp>

#include "mixed_nozzle_harness.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Geometry/ConvexHull.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Utils.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;

// The owner's combined test project, as the app saved it: an H2D 0.2/0.6, a Body Split L (a 20 mm
// column, 50 mm tall, with a 40 mm arm whose underside is at 40 mm) on the 0.6 in PLA (filament 7) at
// 0.4 mm, and "CADENCE" text sunk 0.8 mm into the top of the arm, flush, on the 0.2 in PLA (filament
// 1) at 0.1 mm. Organic tree support under the arm with its base in filament 7 and a PETG interface
// (filament 4) on the 0.2. Fine layers 0.1, first layer 0.1, which the 0.6 (minimum 0.12) cannot lay,
// so the L starts on the bed with a 0.2 mm first cell.

namespace {
using Move = GCodeProcessorResult::MoveVertex;

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
    // Moves the text up (mm) and stretches it along X about its own centre, to vary how it meets the base.
    double      text_dz      = 0.;
    double      text_scale_x = 1.;
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
    {
        // The 3MF reader unpacks into the temporary directory.
        ScopedTemporaryDir unpack("cadence-3mf");
        const std::string previous = temporary_dir();
        set_temporary_dir(unpack.string());
        ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Enable);
        out->model = Model::read_from_file(std::string(TEST_DATA_DIR) + "/" + variant.file, &config, &substitutions,
                                           LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
        set_temporary_dir(previous);
    }
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
        out->failure = variant.name + ": threw " + error.what();
        return out;
    }
    if (!out->facts.refusal.string.empty())
        out->failure = variant.name + ": refused: " + out->facts.refusal.string;
    else if (out->facts.gcode.empty())
        out->failure = variant.name + ": no G-code";
    return out;
}

bool has_warning(const PrintStateBase::StateWithWarnings &state, PrintStateBase::SlicingNotificationType code)
{
    return std::any_of(state.warnings.begin(), state.warnings.end(), [code](const PrintStateBase::Warning &warning) {
        return warning.message_id == code;
    });
}

std::string report(const std::vector<std::string> &failures)
{
    std::ostringstream out;
    for (const std::string &failure : failures)
        out << failure << '\n';
    return out.str();
}

const PrintObject &the_object(const SlicedProject &project) { return *project.print.objects().front(); }

// The tallest road in a support layer's coarse body (support_fills).
double tallest_road(const ExtrusionEntity &entity)
{
    double tallest = 0.;
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
        for (const ExtrusionEntity *child : collection->entities)
            tallest = std::max(tallest, tallest_road(*child));
    } else if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity))
        tallest = double(path->height);
    else if (const auto *multipath = dynamic_cast<const ExtrusionMultiPath *>(&entity)) {
        for (const ExtrusionPath &path : multipath->paths)
            tallest = std::max(tallest, double(path.height));
    } else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity)) {
        for (const ExtrusionPath &path : loop->paths)
            tallest = std::max(tallest, double(path.height));
    }
    return tallest;
}

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
} // namespace

TEST_CASE("A Body Split body on the bed with coarse support under it slices with no empty-layer warning", "[TestRebuild][Support][BodySplit]")
{
    // The bands of the support under the arm are 0.4 (or 0.3) tall but sit on rows where the L prints
    // nothing; the check took those rows as one 0.1 event layer tall.
    std::vector<std::string> failures;
    for (const Variant &variant : {Variant{"organic 0.10/0.40"}, Variant{"normal 0.10/0.40", 0.1, 0.4, false},
                                   Variant{"organic 0.10/0.30", 0.1, 0.3}, Variant{"normal 0.10/0.30", 0.1, 0.3, false}}) {
        const std::unique_ptr<SlicedProject> project = slice_l_project(variant);
        if (!project->failure.empty()) {
            failures.push_back(project->failure);
            continue;
        }
        if (has_warning(project->print.step_state_with_warnings(psGCodeExport), PrintStateBase::SlicingEmptyGcodeLayers))
            failures.push_back(variant.name + ": warns of empty layers");
    }
    INFO(report(failures));
    CHECK(failures.empty());
}

TEST_CASE("Coarse support bands end where the Body Split body they stand beside ends its cells", "[TestRebuild][Support][BodySplit]")
{
    // The support's bed layer is 0.1 on the fine nozzle and the L's first cell is 0.2, so bands counted
    // from the support's own first layer ended 0.1 off every L cell: each coarse step printed twice.
    std::vector<std::string> failures;
    for (const Variant &variant : {Variant{"organic 0.10/0.40"}, Variant{"normal 0.10/0.40", 0.1, 0.4, false},
                                   Variant{"organic 0.10/0.30", 0.1, 0.3}, Variant{"normal 0.10/0.30", 0.1, 0.3, false}}) {
        const std::unique_ptr<SlicedProject> project = slice_l_project(variant);
        if (!project->failure.empty()) {
            failures.push_back(project->failure);
            continue;
        }
        const PrintObject &object = the_object(*project);
        const size_t l_region = coarse_region(object);
        std::vector<double> l_tops;
        for (const NativeRegionCellState &cell : object.native_regional_grid_state().cells[l_region])
            l_tops.push_back(cell.cell.top_z);
        const auto on_l_top = [&l_tops](double z) {
            return std::any_of(l_tops.begin(), l_tops.end(), [z](double top) { return std::abs(top - z) < EPSILON; });
        };
        // Coarse bands (roads taller than a fine layer) beside the column, under the arm. Thin pieces the
        // fine nozzle lays at its own layers may sit anywhere.
        size_t off = 0, bands = 0;
        std::ostringstream what;
        for (const SupportLayer *layer : object.support_layers()) {
            if (layer->print_z > 39. || tallest_road(layer->support_fills) < 0.1 + EPSILON)
                continue;
            ++bands;
            if (!on_l_top(layer->print_z) && off++ < 3)
                what << " " << layer->print_z;
        }
        if (bands == 0 || off != 0)
            failures.push_back(variant.name + ": " + std::to_string(off) + " of " + std::to_string(bands) +
                               " support layers off the L's cells, at Z" + what.str());
    }
    INFO(report(failures));
    CHECK(failures.empty());
}

TEST_CASE("The prime tower is built from layer 1, with its brim, when layer 1 prints only support", "[TestRebuild][Support][BodySplit][TowerFirstLayer]")
{
    // Layer 1 holds only the support's bed layer; the L starts at 0.2. The tower was switched off by
    // reading that layer, so every nozzle and filament change went unprimed.
    std::vector<std::string> failures;
    for (const Variant &variant : {Variant{"organic 0.10/0.40"}, Variant{"normal 0.10/0.40", 0.1, 0.4, false},
                                   Variant{"organic 0.10/0.30", 0.1, 0.3}, Variant{"organic 0.12/0.40", 0.12, 0.4}}) {
        const std::unique_ptr<SlicedProject> project = slice_l_project(variant);
        if (!project->failure.empty()) {
            failures.push_back(project->failure);
            continue;
        }
        const CadenceTest::Facts &facts = project->facts;
        std::ostringstream what;
        what << variant.name << ":";
        bool ok = true;
        // Tower roads on layer 1, and the brim: layer 1 reaches past the fullest of the next tower levels.
        BoundingBoxf first;
        std::map<double, std::pair<size_t, BoundingBoxf>> above;
        for (const Move &move : facts.moves) {
            if (move.type != EMoveType::Extrude || move.extrusion_role != erWipeTower)
                continue;
            const double z = double(move.position.z());
            const Vec2d xy(move.position.x(), move.position.y());
            if (z < variant.first_layer + 1e-3)
                first.merge(xy);
            else if (z < variant.first_layer + 2.) {
                auto &level = above[std::round(z * 1000.) / 1000.];
                ++level.first;
                level.second.merge(xy);
            }
        }
        const std::pair<size_t, BoundingBoxf> *fullest = nullptr;
        for (const auto &level : above)
            if (fullest == nullptr || level.second.first > fullest->first)
                fullest = &level.second;
        const double brim = first.defined && fullest != nullptr ?
            std::min({fullest->second.min.x() - first.min.x(), fullest->second.min.y() - first.min.y(),
                      first.max.x() - fullest->second.max.x(), first.max.y() - fullest->second.max.y()}) : 0.;
        if (!first.defined) {
            what << " no tower on layer 1;";
            ok = false;
        } else if (brim < 1.) {
            what << " tower brim " << brim << " mm;";
            ok = false;
        }
        // Every tool change after the start G-code is inside a tower block.
        std::istringstream gcode(facts.gcode);
        std::string line, z = "?";
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
                    what << " first change off the tower at Z" << z << ";";
            }
        }
        if (changes == 0 || off_tower != 0) {
            what << " " << off_tower << " of " << changes << " tool changes off the tower;";
            ok = false;
        }
        if (!ok)
            failures.push_back(what.str());
    }
    INFO(report(failures));
    CHECK(failures.empty());
}

TEST_CASE("Text sunk into a Body Split base sits on the base, and no support grows under it", "[TestRebuild][Support][BodySplit]")
{
    // Rows of the same letters differ by slicing noise along their sides (thousandths of a mm2). Those
    // slivers counted as contacts, split the L's cells inside the letters, and the cell repair then
    // dropped the plane at the letters' bottom: the L stopped 0.2 to 0.3 mm under the text and the
    // support filled that hollow with the PETG interface.
    std::vector<std::string> failures;
    for (const Variant &variant : {Variant{"organic 0.12/0.40", 0.12, 0.4}, Variant{"organic 0.10/0.40"},
                                   Variant{"normal 0.10/0.30", 0.1, 0.3, false}, Variant{"no support 0.10/0.40", 0.1, 0.4, true, false}}) {
        const std::unique_ptr<SlicedProject> project = slice_l_project(variant);
        if (!project->failure.empty()) {
            failures.push_back(project->failure);
            continue;
        }
        const PrintObject &object = the_object(*project);
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
        if (text_footprint.empty()) {
            failures.push_back(variant.name + ": no text");
            continue;
        }
        // The L under the letters reaches up to their bottom.
        double l_top_under_text = 0.;
        const ExPolygons inner_text = offset_ex(text_footprint, -float(scale_(0.05)));
        for (const Layer *layer : object.layers())
            for (const LayerRegion *region : layer->regions())
                if (size_t(region->region().print_object_region_id()) == l_region && layer->print_z < text_bottom + 0.5 &&
                    !intersection_ex(to_expolygons(region->slices.surfaces), inner_text).empty())
                    l_top_under_text = std::max(l_top_under_text, layer->print_z);
        std::ostringstream what;
        bool ok = true;
        if (std::abs(l_top_under_text - text_bottom) > EPSILON) {
            what << " the L under the text ends at " << l_top_under_text << ", the text starts at " << text_bottom << ";";
            ok = false;
        }
        // No support inside the L: well above the arm's underside (40 mm, whose first cell may end up to a
        // coarse layer higher on the L's grid).
        size_t inside = 0;
        double lowest = std::numeric_limits<double>::max();
        for (const Move &move : project->facts.moves)
            if (move.type == EMoveType::Extrude &&
                (move.extrusion_role == erSupportMaterial || move.extrusion_role == erSupportMaterialInterface) &&
                double(move.position.z()) > 41.) {
                ++inside;
                lowest = std::min(lowest, double(move.position.z()));
            }
        if (inside != 0) {
            what << " " << inside << " support roads inside the L, the lowest at Z " << lowest << ";";
            ok = false;
        }
        if (!ok)
            failures.push_back(variant.name + ":" + what.str());
    }
    INFO(report(failures));
    CHECK(failures.empty());
}

TEST_CASE("Organic branches that widen inside a coarse band are not laid in the air", "[TestRebuild][Support][BodySplit]")
{
    // Fine layers 0.12 from a 0.10 first layer, coarse 0.36: the bands under the arm end on the L's cells
    // (..., 11.02, 11.38, ...), so the fine rows inside a band (11.14, 11.26) print nothing until the band
    // does, at its top. A branch that widened at 11.26 kept its fringe there as a fine piece, with nothing
    // under it at 11.14, 0.12 mm in the air, and the plate was refused for empty layers.
    std::vector<std::string> failures;
    // The same L and text from the Body Split 0.2-0.6 Standard preset (organic base on the 0.6, PETG interface).
    const std::string file = "body-split-L-organic-fine012.3mf";
    for (const Variant &variant : {Variant{"organic 0.10/0.36 fine 0.12", 0.1, 0.36, true, true, 0.12, file},
                                   Variant{"organic 0.12/0.36 fine 0.12", 0.12, 0.36, true, true, 0.12, file},
                                   Variant{"organic 0.10/0.40", 0.1, 0.4}}) {
        const std::unique_ptr<SlicedProject> project = slice_l_project(variant);
        if (!project->failure.empty()) {
            failures.push_back(project->failure);
            continue;
        }
        if (has_warning(project->print.step_state_with_warnings(psGCodeExport), PrintStateBase::SlicingEmptyGcodeLayers))
            failures.push_back(variant.name + ": warns of empty layers");
    }
    INFO(report(failures));
    CHECK(failures.empty());
}

TEST_CASE("A Body Split base whose nozzle needs two fine layers per cell still reaches the text on it", "[TestRebuild][BodySplit]")
{
    // 0.2/0.8 at fine 0.08 and first layer 0.10: the 0.8 needs two fine layers (0.16) per cell, and its
    // cells counted up from its 0.18 first cell end at 9.94, 10.10, ..., 49.94, 50.10. The text's bottom
    // (raised on the L's top at 50.0, or sunk to 49.2) sits one fine layer off those tops, so the L's last
    // cell there was one layer short and was dropped: the L stopped 0.08 under the text, and with raised
    // text the plate was refused for empty layers.
    std::vector<std::string> failures;
    for (const Variant &variant : {Variant{"raised 0.10/0.16 fine 0.08", 0.1, 0.16, true, true, 0.08, "body-split-L-text-raised-0208.3mf"},
                                   Variant{"sunk 0.10/0.16 fine 0.08", 0.1, 0.16, true, true, 0.08, "body-split-L-text-sunk-0208.3mf"}}) {
        const std::unique_ptr<SlicedProject> project = slice_l_project(variant);
        if (!project->failure.empty()) {
            failures.push_back(project->failure);
            continue;
        }
        if (has_warning(project->print.step_state_with_warnings(psGCodeExport), PrintStateBase::SlicingEmptyGcodeLayers))
            failures.push_back(variant.name + ": warns of empty layers");
        const PrintObject &object = the_object(*project);
        const size_t l_region = coarse_region(object);
        double text_bottom = std::numeric_limits<double>::max();
        ExPolygons text_footprint;
        for (const Layer *layer : object.layers())
            for (const LayerRegion *region : layer->regions())
                if (size_t(region->region().print_object_region_id()) != l_region && !region->slices.empty() &&
                    layer->print_z - region->height() < text_bottom - EPSILON) {
                    text_bottom    = layer->print_z - region->height();
                    text_footprint = to_expolygons(region->slices.surfaces);
                }
        if (text_footprint.empty()) {
            failures.push_back(variant.name + ": no text");
            continue;
        }
        double l_top_under_text = 0.;
        const ExPolygons inner_text = offset_ex(text_footprint, -float(scale_(0.05)));
        for (const Layer *layer : object.layers())
            for (const LayerRegion *region : layer->regions())
                if (size_t(region->region().print_object_region_id()) == l_region && layer->print_z < text_bottom + 0.5 &&
                    !intersection_ex(to_expolygons(region->slices.surfaces), inner_text).empty())
                    l_top_under_text = std::max(l_top_under_text, layer->print_z);
        if (std::abs(l_top_under_text - text_bottom) > EPSILON) {
            std::ostringstream what;
            what << variant.name << ": the L under the text ends at " << l_top_under_text << ", the text starts at " << text_bottom;
            failures.push_back(what.str());
        }
    }
    INFO(report(failures));
    CHECK(failures.empty());
}

TEST_CASE("Thin text on a Body Split base gets no support between its letters", "[TestRebuild][Support][BodySplit]")
{
    // "Cadence 0.2" (5 mm Arial, strokes about 0.5 mm) raised 0.8 mm on a 60 x 40 x 10 block, 0.2/0.6 at
    // 0.12/0.36, normal support from the 0.2 with a PETG interface. The sharp-tail check filtered the layer
    // below by the first region's road, the 0.6 base's, which dropped the thin strokes there; each stroke
    // above then looked like a tail with nothing under it, and support and PETG grew between the letters
    // on top of the block. Nothing on this part needs support above the block's top.
    std::vector<std::string> failures;
    const std::string file = "body-split-block-thin-text.3mf";
    for (const Variant &variant : {Variant{"normal 0.12/0.36", 0.12, 0.36, false, true, 0.12, file},
                                   Variant{"normal 0.10/0.36", 0.1, 0.36, false, true, 0.12, file}}) {
        const std::unique_ptr<SlicedProject> project = slice_l_project(variant);
        if (!project->failure.empty()) {
            failures.push_back(project->failure);
            continue;
        }
        size_t above = 0;
        double lowest = std::numeric_limits<double>::max();
        for (const Move &move : project->facts.moves)
            if (move.type == EMoveType::Extrude &&
                (move.extrusion_role == erSupportMaterial || move.extrusion_role == erSupportMaterialInterface) &&
                double(move.position.z()) > 10.05) {
                ++above;
                lowest = std::min(lowest, double(move.position.z()));
            }
        if (above != 0) {
            std::ostringstream what;
            what << variant.name << ": " << above << " support roads above the block's top, the lowest at Z " << lowest;
            failures.push_back(what.str());
        }
    }
    INFO(report(failures));
    CHECK(failures.empty());
}

TEST_CASE("A plain object its nozzle cannot lay is refused in words about the object", "[TestRebuild][BodySplit]")
{
    // A Body Split plate, 0.2/0.8 at 0.08 on a 0.10 first layer: a block with painted text, and the L on
    // its own, wholly on the 0.8. The L is a plain object and prints at the plate's 0.08 layers, which the
    // 0.8 (0.16 to 0.56) cannot lay. The refusal named the role key the region resolved
    // ("outer_wall_filament_id names filament 7 ..."); it now names the object and its filament.
    Model model;
    DynamicPrintConfig config;
    {
        ScopedTemporaryDir unpack("cadence-3mf");
        const std::string previous = temporary_dir();
        set_temporary_dir(unpack.string());
        ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Enable);
        model = Model::read_from_file(std::string(TEST_DATA_DIR) + "/body-split-plate-plain-coarse-object.3mf", &config, &substitutions,
                                      LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
        set_temporary_dir(previous);
    }
    REQUIRE(model.objects.size() == 2);
    Print print;
    print.is_BBL_printer() = true;
    print.apply(model, config);
    print.set_status_silent();
    const StringObjectException refusal = print.validate();
    INFO(refusal.string);
    REQUIRE(refusal.string.find("[SRL-A38]") != std::string::npos);
    CHECK(refusal.string.find("\"L\"") != std::string::npos);
    CHECK(refusal.string.find("filament 7") != std::string::npos);
    CHECK(refusal.string.find("0.8 mm nozzle") != std::string::npos);
    CHECK(refusal.string.find("_filament_id") == std::string::npos);
    CHECK(refusal.opt_key == "outer_wall_filament_id");
}

TEST_CASE("The coarse base's top around sunk text is laid as top surface and not as rings of walls", "[TestRebuild][BodySplit]")
{
    // Owner print of the combined L (build 5): the 0.6 laid the top of the L around the letters as four
    // 0.66 mm walls around every letter and the edge, so the top was nearly all walls meeting at odd
    // angles, with about 250 gap fill roads per layer and open slivers between the letters. Only about
    // 10 percent of it was top surface. A coarse Body Split body now lays its top surfaces with one wall,
    // the stock Bambu default, and leaves the rest to the top surface pattern.
    std::vector<std::string> failures;
    for (const Variant &variant : {Variant{"organic 0.10/0.40"}, Variant{"organic 0.12/0.40", 0.12, 0.4},
                                   Variant{"no support 0.10/0.30", 0.1, 0.3, true, false}}) {
        const std::unique_ptr<SlicedProject> project = slice_l_project(variant);
        if (!project->failure.empty()) {
            failures.push_back(project->failure);
            continue;
        }
        // The L's top: the highest extrusion off the tower.
        double top_z = 0.;
        for (const Move &move : project->facts.moves)
            if (move.type == EMoveType::Extrude && move.extrusion_role != erWipeTower)
                top_z = std::max(top_z, double(move.position.z()));
        // Area laid by the 0.6 (roads wider than the 0.2 can lay) on the top layer, by role.
        std::map<ExtrusionRole, double> area;
        const Move *previous = nullptr;
        for (const Move &move : project->facts.moves) {
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
        what << variant.name << ": top Z " << top_z << ", coarse area " << total << " mm2, top surface "
             << area[erTopSolidInfill] << ", inner walls " << area[erPerimeter] << ", outer walls "
             << area[erExternalPerimeter] << ", gap fill " << area[erGapFill] << " (top share " << top_share << ")";
        INFO(what.str());
        if (total < 500. || top_share < 0.6 || area[erPerimeter] > 0.)
            failures.push_back(what.str());
    }
    INFO(report(failures));
    CHECK(failures.empty());
}

namespace {
// What the coarse body leaves open on its top layer within 4 mm of the text, like topcov.py on the G-code:
// the base's top outline less the letters (grown 0.05 mm) and its edge (0.11 mm), minus the area its roads
// cover at their width.
struct CoarseTopGaps {
    double top_z   = 0.;
    double near    = 0.; // mm2 of base top within 4 mm of the letters
    double open    = 0.; // mm2 of it no road covers
    double largest = 0.;
    size_t spots   = 0;  // open spots of 0.1 mm2 or more
    size_t gap_fill_roads = 0;
};

size_t count_gap_fill(const ExtrusionEntity &entity)
{
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
        size_t n = 0;
        for (const ExtrusionEntity *child : collection->entities)
            n += count_gap_fill(*child);
        return n;
    }
    return entity.role() == erGapFill ? 1 : 0;
}

// The area each road covers, by role: its centreline grown by half its width with round ends and joins, as
// the plastic lies (ExtrusionPath::polygons_covered_by_width cuts each piece square, which leaves notches
// where a wall is split into pieces).
void road_cover(const ExtrusionEntity &entity, std::map<ExtrusionRole, Polygons> &out)
{
    auto path_cover = [&out](const ExtrusionPath &path) {
        const Polyline line = path.polyline.to_polyline();
        if (line.size() >= 2)
            polygons_append(out[path.role()], offset(line, float(scale_(path.width / 2.)), ClipperLib::jtRound,
                                                     double(scale_(0.005)), ClipperLib::etOpenRound));
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
    gaps.gap_fill_roads = count_gap_fill(l_top->perimeters) + count_gap_fill(l_top->fills) + count_gap_fill(l_top->thin_fills);
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
} // namespace

TEST_CASE("The coarse base's top leaves no open gaps around text on it", "[TestRebuild][BodySplit]")
{
    // Owner print of the combined L (build 9): the 0.6 left 3.3 mm2 open in the blue top around the sunk
    // letters, in 29 spots. Most were crescents and wedges where the last 0.66 mm top line stops short of a
    // curved or angled wall (the plate has gap_fill_target "nowhere"), and one was a clear line between the
    // walls around E and N, 0.08 mm wide, just under the smallest gap the walls' gap fill takes. A coarse
    // Body Split top now gap-fills both and pulls its top lines further into the wall. Text standing proud
    // (half sunk) used to leave the base's top laid with four walls, and raised text took about 2 mm around
    // its letters out of the top (min_width_top_surface at 300 % of a 0.66 mm line) and ringed it with walls.
    // Measured before the change (this test on build 10): sunk 3.8 mm2, letters 0.95 wide 2.1, 1.05 wide 3.6,
    // half sunk 6.8, raised 2.1. Letter notches narrower than a 0.6 road stay.
    std::vector<std::string> failures;
    Variant narrow{"sunk, letters 0.95 wide"};
    narrow.text_scale_x = 0.95;
    Variant wide{"sunk, letters 1.05 wide"};
    wide.text_scale_x = 1.05;
    Variant half{"half sunk (0.4 mm proud)"};
    half.text_dz = 0.4;
    Variant raised{"raised on the top"};
    raised.text_dz = 0.8;
    for (const Variant &variant : {Variant{"sunk (owner plate)"}, narrow, wide, half, raised,
                                   Variant{"raised 0.10/0.16 fine 0.08", 0.1, 0.16, true, true, 0.08, "body-split-L-text-raised-0208.3mf"}}) {
        const std::unique_ptr<SlicedProject> project = slice_l_project(variant);
        if (!project->failure.empty()) {
            failures.push_back(project->failure);
            continue;
        }
        const CoarseTopGaps gaps = coarse_top_gaps(the_object(*project), coarse_region(the_object(*project)));
        std::ostringstream what;
        what << variant.name << ": top Z " << gaps.top_z << ", base top near the letters " << gaps.near << " mm2, open "
             << gaps.open << " mm2, " << gaps.spots << " spots of 0.1 mm2 or more, largest " << gaps.largest << ", "
             << gaps.gap_fill_roads << " gap fill roads";
        INFO(what.str());
        CHECK(gaps.near > 50.);
        if (gaps.near < 50. || gaps.open >= 1.)
            failures.push_back(what.str());
    }
    INFO(report(failures));
    CHECK(failures.empty());
}

namespace {
// How full the coarse body's top layer lays each pocket the text walls in: for each letter, its convex hull
// less the letters (the counter, open or closed), and for each pair of neighbouring letters their joint hull
// less the letters and both counters (the space between them), each cut to the part the letters wall in (the
// letters' outline closed by 2.5 mm; a hull also takes in open ground past the letters' tips, where extra
// plastic can spread). Fill is the coarse roads' volume there (the centreline inside, times the road's mm3
// per mm, from the G-code) over the pocket's area times the layer height.
struct PocketFill {
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

std::vector<PocketFill> coarse_top_pocket_fill(const PrintObject &object, size_t l_region, const std::vector<Move> &moves)
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
    std::vector<PocketFill> out;
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
} // namespace

TEST_CASE("The coarse base's top fills the pockets the text walls in with no more than they hold", "[TestRebuild][BodySplit]")
{
    // Owner prints of the combined L with the gap fix (MC-1): lumps in the letter counters and between the
    // letters on the blue top. The 0.6 laid 2 to 9 % more plastic there than the room between the black
    // walls (build 9 laid 0.95 to 1.03 and had no lumps). A pocket the text walls in at full height has
    // nowhere to put the extra. Its top lines and gap fill now run at a lower flow there, so each pocket gets
    // at most what it holds, while the gaps around the letters stay closed (the test above).
    std::vector<std::string> failures;
    Variant half{"half sunk (0.4 mm proud)"};
    half.text_dz = 0.4;
    for (const Variant &variant : {Variant{"sunk (owner plate)"}, half}) {
        const std::unique_ptr<SlicedProject> project = slice_l_project(variant);
        if (!project->failure.empty()) {
            failures.push_back(project->failure);
            continue;
        }
        const std::vector<PocketFill> pockets = coarse_top_pocket_fill(the_object(*project), coarse_region(the_object(*project)), project->facts.moves);
        std::ostringstream what;
        what << variant.name << ":";
        double fullest = 0., emptiest = 10.;
        for (const PocketFill &pocket : pockets) {
            what << " " << pocket.name << " " << pocket.fill << " (" << pocket.area << " mm2);";
            fullest  = std::max(fullest, pocket.fill);
            emptiest = std::min(emptiest, pocket.fill);
        }
        INFO(what.str());
        // Seven letters: seven counters (the E's arms count) and six spaces between them.
        CHECK(pockets.size() >= 12);
        if (pockets.size() < 12 || fullest > 1.0 || emptiest < 0.85)
            failures.push_back(what.str());
    }
    INFO(report(failures));
    CHECK(failures.empty());
}

TEST_CASE("The coarse base's top lays its gap fill and the pockets the text walls in at gap fill speed", "[TestRebuild][BodySplit]")
{
    // Owner prints of the combined L with the gap fix (MC-1), on an MN Feature preset: lumps in the letter
    // counters and between the letters on the blue top. The gap fill MC-1 forces there ran at about
    // 209 mm/s and up to 31 mm3/s (the preset's coarse gap infill speed is 250, stock 0.6 values). A coarse
    // Body Split top now lays its gap fill, and its top lines inside the pockets the text walls in, at no more
    // than 50 mm/s (the MN Body presets' value). The open top keeps the preset's speed.
    std::vector<std::string> failures;
    Variant half{"half sunk (0.4 mm proud)"};
    half.text_dz = 0.4;
    for (const Variant &variant : {Variant{"sunk (owner plate)"}, half}) {
        const std::unique_ptr<SlicedProject> project = slice_l_project(variant);
        if (!project->failure.empty()) {
            failures.push_back(project->failure);
            continue;
        }
        // The coarse top: the highest top surface laid with roads wider than the 0.2 can lay.
        double top_z    = 0.;
        int    extruder = -1;
        for (const Move &move : project->facts.moves)
            if (move.type == EMoveType::Extrude && move.extrusion_role == erTopSolidInfill && move.width > 0.45f &&
                double(move.position.z()) > top_z) {
                top_z    = double(move.position.z());
                extruder = move.extruder_id;
            }
        size_t gap_fill = 0;
        double fastest_gap_fill = 0., slow_top = 0., fast_top = 0.;
        const Move *previous = nullptr;
        for (const Move &move : project->facts.moves) {
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
        what << variant.name << ": top Z " << top_z << ", " << gap_fill << " gap fill moves, fastest " << fastest_gap_fill
             << " mm/s; top surface at 50 mm/s or less " << slow_top << " mm, faster " << fast_top << " mm";
        INFO(what.str());
        if (gap_fill == 0 || fastest_gap_fill > 50.01 || slow_top < 10. || fast_top < slow_top)
            failures.push_back(what.str());
    }
    INFO(report(failures));
    CHECK(failures.empty());
}
