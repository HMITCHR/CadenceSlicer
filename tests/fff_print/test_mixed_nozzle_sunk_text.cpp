#include <catch2/catch_all.hpp>

#include "mixed_nozzle_harness.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
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
    for (ModelVolume *volume : out->model.objects.front()->volumes)
        volume->config.set_key_value("regional_layer_height",
                                     new ConfigOptionFloat(volume->name == "L" || volume->name == "block" ? variant.coarse : variant.fine));
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
