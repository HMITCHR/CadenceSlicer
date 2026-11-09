#include "libslic3r/Exception.hpp"
#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/Config.hpp"
#include "ExtrusionEntity.hpp"
#include "Exception.hpp"
#include "libslic3r/ObjectID.hpp"
#include "libslic3r/FilamentGroup.hpp"
#include "Print.hpp"
#include "Slicing.hpp"
#include "ToolOrdering.hpp"
#include "Layer.hpp"
#include "ClipperUtils.hpp"
#include "ParameterUtils.hpp"
#include "GCode/ToolOrderUtils.hpp"
#include "FilamentGroupUtils.hpp"
#include "MultiNozzleUtils.hpp"
#include "MixedNozzleConfig.hpp"
#include "FilamentMixer.hpp"
#include "LocalesUtils.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "Utils.hpp"
#include "format.hpp"
#include "I18N.hpp"
#include "../BeltBrim.hpp"

#include <boost/log/trivial.hpp>
#include <vector>
#include <string>
#include <utility>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <functional>
#include <exception>
#include <memory>
#include <tuple>
#include <iostream>

// #define SLIC3R_DEBUG

// Make assert active if SLIC3R_DEBUG
#ifdef SLIC3R_DEBUG
    #define DEBUG
    #define _DEBUG
    #undef NDEBUG
#endif

#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <algorithm>
#include <string>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <unordered_map>

#include <libslic3r.h>
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/SurfaceCollection.hpp"

namespace Slic3r {

    //! macro used to mark string used at localization,
    //! return same string

#ifndef _L
#define _L(s) Slic3r::I18N::translate(s)
#endif

const static bool g_wipe_into_objects = false;
constexpr double similar_color_threshold_de2000 = 20.0;

FeatureObjectRoleOwner classify_feature_object_role(ExtrusionRole role) noexcept
{
    switch (role) {
    case erExternalPerimeter:
        return FeatureObjectRoleOwner::OuterWall;
    case erPerimeter:
        return FeatureObjectRoleOwner::InnerWall;
    case erInternalInfill:
        return FeatureObjectRoleOwner::SparseInfill;
    case erSolidInfill:
    case erInternalBridgeInfill:
        return FeatureObjectRoleOwner::InternalSolidInfill;
    case erTopSolidInfill:
        return FeatureObjectRoleOwner::TopSurface;
    case erBottomSurface:
        return FeatureObjectRoleOwner::BottomSurface;
    case erGapFill:
        return FeatureObjectRoleOwner::OuterWall;
    case erBridgeInfill:
        // External bridge infill is a surface-quality role: the bottom-surface (fine) owner.
        return FeatureObjectRoleOwner::BottomSurface;
    case erSupportMaterial:
    case erSupportTransition:
        // Tree-support transition goes with the support base, as GCode.cpp groups it.
        return FeatureObjectRoleOwner::SupportBase;
    case erSupportMaterialInterface:
        return FeatureObjectRoleOwner::SupportInterface;
    default:
        return FeatureObjectRoleOwner::Unsupported;
    }
}

static std::string feature_object_role_text(ExtrusionRole role)
{
    switch (role) {
    case erNone:                     return "Undefined";
    case erPerimeter:                return "Inner wall";
    case erExternalPerimeter:        return "Outer wall";
    case erOverhangPerimeter:        return "Overhang wall";
    case erInternalInfill:           return "Sparse infill";
    case erSolidInfill:              return "Internal solid infill";
    case erTopSolidInfill:           return "Top surface";
    case erBottomSurface:            return "Bottom surface";
    case erIroning:                  return "Ironing";
    case erBridgeInfill:             return "Bridge";
    case erInternalBridgeInfill:     return "Internal Bridge";
    case erGapFill:                  return "Gap infill";
    case erSkirt:                    return "Skirt";
    case erBrim:                     return "Brim";
    case erSupportMaterial:          return "Support";
    case erSupportMaterialInterface: return "Support interface";
    case erSupportTransition:        return "Support transition";
    case erWipeTower:                return "Prime tower";
    case erCustom:                   return "Custom";
    case erMixed:                    return "Multiple";
    case erCount:                    return "Role count sentinel";
    }
    return "Unknown role (" + std::to_string(static_cast<unsigned int>(role)) + ")";
}

unsigned feature_object_role_filament(ExtrusionRole role, const PrintRegion &region)
{
    const int *filament_id = nullptr;
    const char *config_key = nullptr;
    switch (classify_feature_object_role(role)) {
    case FeatureObjectRoleOwner::OuterWall:
        filament_id = &region.config().outer_wall_filament_id.value;
        config_key = "outer_wall_filament_id";
        break;
    case FeatureObjectRoleOwner::InnerWall:
        filament_id = &region.config().inner_wall_filament_id.value;
        config_key = "inner_wall_filament_id";
        break;
    case FeatureObjectRoleOwner::SparseInfill:
        filament_id = &region.config().sparse_infill_filament_id.value;
        config_key = "sparse_infill_filament_id";
        break;
    case FeatureObjectRoleOwner::InternalSolidInfill:
        filament_id = &region.config().internal_solid_filament_id.value;
        config_key = "internal_solid_filament_id";
        break;
    case FeatureObjectRoleOwner::TopSurface:
        filament_id = &region.config().top_surface_filament_id.value;
        config_key = "top_surface_filament_id";
        break;
    case FeatureObjectRoleOwner::BottomSurface:
        filament_id = &region.config().bottom_surface_filament_id.value;
        config_key = "bottom_surface_filament_id";
        break;
    case FeatureObjectRoleOwner::Unsupported:
        throw SlicingError(Slic3r::format(
            "[SRL-F06] Feature Split does not support object extrusion role '%1%'.",
            feature_object_role_text(role)));
    }

    if (filament_id == nullptr)
        throw SlicingError(Slic3r::format(
            "[SRL-F06] Feature Split does not support object extrusion role '%1%'.",
            feature_object_role_text(role)));
    if (*filament_id <= 0)
        throw SlicingError(Slic3r::format(
            "[SRL-F06] Feature Split cannot place the '%1%' feature because '%2%' is not a valid filament number.",
            feature_object_role_text(role), config_key));
    return unsigned(*filament_id - 1);
}

unsigned require_feature_object_filament_offered(unsigned zero_based_logical_filament, const LayerTools &layer_tools)
{
    if (! layer_tools.has_extruder(zero_based_logical_filament))
        throw SlicingError(Slic3r::format(
            "[SRL-F07] Feature Split filament '%1%' is not available on this layer.",
            zero_based_logical_filament + 1));
    return zero_based_logical_filament;
}

// Height of the first path in an extrusion entity, or zero when it has none.
static double first_path_height(const ExtrusionEntity &entity)
{
    if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity))
        return path->height;
    if (const auto *multi_path = dynamic_cast<const ExtrusionMultiPath *>(&entity))
        return multi_path->paths.empty() ? 0. : multi_path->paths.front().height;
    if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity))
        return loop->paths.empty() ? 0. : loop->paths.front().height;
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity))
        for (const ExtrusionEntity *child : collection->entities)
            if (child != nullptr)
                return first_path_height(*child);
    return 0.;
}

// Sparse infill that combine_infill() left fine-owned (see LayerRegion::feature_split_sparse_fine_owned()),
// or that is thinner than the coarse nozzle can lay, prints on the region's outer wall tool; every other
// role uses feature_object_role_filament().
static unsigned feature_object_leaf_owner(ExtrusionRole role, const PrintRegion &region, unsigned extruder_override,
                                          const LayerRegion *layerm, double height)
{
    const FeatureObjectRoleOwner owner = classify_feature_object_role(role);
    if (owner == FeatureObjectRoleOwner::Unsupported)
        return feature_object_role_filament(role, region) + 1;
    if (extruder_override != 0)
        return extruder_override;
    if (owner == FeatureObjectRoleOwner::SparseInfill && layerm != nullptr &&
        layerm->feature_split_sparse_fine_owned_at(height))
        return unsigned(region.config().outer_wall_filament_id.value);
    return feature_object_role_filament(role, region) + 1;
}

static void add_feature_object_leaf_filament(ExtrusionRole role, const PrintRegion &region, LayerTools &layer_tools,
                                              unsigned extruder_override, std::vector<int> *first_layer_extruders,
                                              const LayerRegion *layerm, double height)
{
    const unsigned owner = feature_object_leaf_owner(role, region, extruder_override, layerm, height);
    layer_tools.extruders.emplace_back(owner);
    if (first_layer_extruders != nullptr && role == erExternalPerimeter)
        first_layer_extruders->emplace_back(owner);
}

static bool collect_feature_object_leaf_filaments(const ExtrusionEntity &entity, const PrintRegion &region, LayerTools &layer_tools,
                                                   unsigned extruder_override, std::vector<int> *first_layer_extruders,
                                                   const LayerRegion *layerm)
{
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
        bool has_extrusion = false;
        for (const ExtrusionEntity *child : collection->entities)
            if (child != nullptr)
                has_extrusion |= collect_feature_object_leaf_filaments(*child, region, layer_tools, extruder_override, first_layer_extruders, layerm);
        return has_extrusion;
    }

    if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&entity)) {
        // As GCode.cpp's split_mixed_perimeters: the loop's own role decides which wall owns the
        // whole loop, overhang segments included; an all-overhang loop counts as outer wall.
        const ExtrusionRole loop_role = loop->role();
        const ExtrusionRole owner_role = (loop_role == erOverhangPerimeter) ? erExternalPerimeter : loop_role;
        for (const ExtrusionPath &path : loop->paths) {
            const ExtrusionRole effective_role = (path.role() == erOverhangPerimeter) ? owner_role : path.role();
            add_feature_object_leaf_filament(effective_role, region, layer_tools, extruder_override, first_layer_extruders, layerm, path.height);
        }
        return !loop->paths.empty();
    }

    if (const auto *multi_path = dynamic_cast<const ExtrusionMultiPath *>(&entity)) {
        for (const ExtrusionPath &path : multi_path->paths)
            add_feature_object_leaf_filament(path.role(), region, layer_tools, extruder_override, first_layer_extruders, layerm, path.height);
        return !multi_path->paths.empty();
    }

    if (const auto *path = dynamic_cast<const ExtrusionPath *>(&entity)) {
        add_feature_object_leaf_filament(path->role(), region, layer_tools, extruder_override, first_layer_extruders, layerm, path->height);
        return true;
    }

    add_feature_object_leaf_filament(entity.role(), region, layer_tools, extruder_override, first_layer_extruders, layerm, 0.);
    return true;
}

static std::set<int>get_filament_by_type(const std::vector<unsigned int>& used_filaments, const PrintConfig* print_config, const std::string& type)
{
    std::set<int> target_filaments;
    for (unsigned int filament_id : used_filaments) {
        std::string filament_type = print_config->filament_type.get_at(filament_id);
        if (filament_type == type)
            target_filaments.insert(filament_id);
    }
    return target_filaments;
}


// Returns true in case that extruder a comes before b (b does not have to be present). False otherwise.
bool LayerTools::is_extruder_order(unsigned int a, unsigned int b) const
{
    if (a == b)
        return false;

    for (auto extruder : extruders) {
        if (extruder == a)
            return true;
        if (extruder == b)
            return false;
    }

    return false;
}

bool check_filament_printable_after_group(const std::vector<unsigned int> &used_filaments, const std::vector<int> &filament_maps, const PrintConfig *print_config)
{
    for (unsigned int filament_id : used_filaments) {
        std::string filament_type = print_config->filament_type.get_at(filament_id);
        int printable_status = print_config->filament_printable.get_at(filament_id);
        int extruder_idx = filament_maps[filament_id];
        if (!(printable_status >> extruder_idx & 1)) {
            std::string error_msg = extruder_idx == 0 ?
                                        Slic3r::format(_L("Grouping error: %1% cannot be placed in the left nozzle"), filament_type) :
                                        Slic3r::format(_L("Grouping error: %1% cannot be placed in the right nozzle"), filament_type);
            throw Slic3r::RuntimeError(error_msg);
        }
    }
    return true;
}

// Return a zero based extruder from the region, or extruder_override if overriden.
// The region accessors below resolve mixed-color slots to the physical filament chosen for this
// layer by resolve_mixed_filaments(), because a virtual slot id is never a real tool. resolve_mixed()
// returns its argument unchanged for every filament that is not a mixed slot.
unsigned int LayerTools::wall_extruder_id(const PrintRegion &region) const
{
	assert(region.config().outer_wall_filament_id.value > 0);
	unsigned int result = ((this->extruder_override == 0) ? region.config().outer_wall_filament_id.value : this->extruder_override) - 1;
	return resolve_mixed(result);
}

unsigned int LayerTools::sparse_infill_filament_id(const PrintRegion &region) const
{
	assert(region.config().sparse_infill_filament_id.value > 0);
	unsigned int result = ((this->extruder_override == 0) ? region.config().sparse_infill_filament_id.value : this->extruder_override) - 1;
	return resolve_mixed(result);
}

unsigned int LayerTools::internal_solid_filament_id(const PrintRegion &region) const
{
	assert(region.config().internal_solid_filament_id.value > 0);
	unsigned int result = ((this->extruder_override == 0) ? region.config().internal_solid_filament_id.value : this->extruder_override) - 1;
	return resolve_mixed(result);
}

// 1 based filament of an island's extrusions; the tool ordering and the G-code export must agree on it.
static unsigned int region_filament(const ExtrusionEntityCollection &extrusions, const PrintRegionConfig &config)
{
    if (extrusions.has_infill()) {
        if (!extrusions.has_solid_infill())
            return config.sparse_infill_filament_id.value;
        // Orca: Gap fill takes the filament of the surface it fills.
        ExtrusionRole role = erNone;
        for (const ExtrusionEntity *ee : extrusions.entities)
            if (const ExtrusionRole er = ee->role(); er != erGapFill)
                role = (role == erNone || role == er) ? er : erMixed;
        if (role == erTopSolidInfill || role == erIroning)
            return config.top_surface_filament_id.value;
        if (role == erBottomSurface)
            return config.bottom_surface_filament_id.value;
        return config.internal_solid_filament_id.value;
    }
    // Walls, and the gap fill between them.
    const ExtrusionEntitiesPtr &walls = extrusions.entities;
    return !walls.empty() && std::all_of(walls.begin(), walls.end(), [](const ExtrusionEntity *ee) { return is_inner_wall(*ee); }) ?
        config.inner_wall_filament_id.value : config.outer_wall_filament_id.value;
}

// Returns a zero based extruder this eec should be printed with, according to PrintRegion config or extruder_override if overriden.
unsigned int LayerTools::extruder(const ExtrusionEntityCollection &extrusions, const PrintRegion &region,
                                  const LayerRegion *layerm) const
{
	assert(region.config().outer_wall_filament_id.value > 0);
	assert(region.config().sparse_infill_filament_id.value > 0);
	assert(region.config().internal_solid_filament_id.value > 0);
	assert(region.config().top_surface_filament_id.value > 0);
	assert(region.config().bottom_surface_filament_id.value > 0);
	// 1 based extruder ID.
    unsigned int extruder = 1;
    if (this->extruder_override == 0) {
        // A converted fine-skin cell prints entirely on its own tool, overriding per-role ids.
        if (layerm != nullptr && layerm->cell_filament_id() > 0)
            return unsigned(layerm->cell_filament_id()) - 1;
        if (extrusions.has_infill() && ! extrusions.has_solid_infill() && layerm != nullptr &&
            layerm->feature_split_sparse_fine_owned_at(first_path_height(extrusions))) {
            // Feature Split sparse infill left fine-owned by combine_infill(), or thinner than the
            // coarse nozzle can lay, prints on the outer wall tool.
            extruder = region.config().outer_wall_filament_id.value;
        } else {
            extruder = region_filament(extrusions, region.config());
        }
    } else
        extruder = this->extruder_override;

    unsigned int result = (extruder == 0) ? 0 : extruder - 1;
    return resolve_mixed(result);
}

static double calc_max_layer_height(const PrintConfig &config, double max_object_layer_height)
{
    double max_layer_height = std::numeric_limits<double>::max();
    for (size_t i = 0; i < config.nozzle_diameter.values.size(); ++ i) {
        // max_layer_height may be shorter than the extruder count; get_at() clamps.
        double mlh = config.max_layer_height.get_at(i);
        if (mlh == 0.)
            mlh = 0.75 * config.nozzle_diameter.values[i];
        max_layer_height = std::min(max_layer_height, mlh);
    }
    // The Prusa3D Fast (0.35mm layer height) print profile sets a higher layer height than what is normally allowed
    // by the nozzle. This is a hack and it works by increasing extrusion width. See GH #3919.
    return std::max(max_layer_height, max_object_layer_height);
}

// The largest cadence this object deposits, as input to calc_max_layer_height(). Outside Body
// Split it is the nominal layer height; otherwise the tallest published cell height, or the
// configured regional heights when no grid is published.
// Unclamped: validation already limits every regional height to max(min_n max_layer_height(n),
// layer_height), so the resulting bound is unchanged. If that check is relaxed, clamp here.
static double native_object_cadence_for_wipe_tower(const PrintObject &object)
{
    double height = object.config().layer_height.value;
    if (! is_mixed_nozzle_body_split(object.print()->config()) || ! is_body_split_object(object.print()->config(), *object.model_object()))
        return height;

    bool found_cell = false;
    for (const std::vector<LayerRegion *> &grid : object.region_grids())
        for (const LayerRegion *cell : grid) {
            height = std::max(height, double(cell->height()));
            found_cell = true;
        }

    if (! found_cell)
        for (size_t region_id = 0; region_id < object.num_printing_regions(); ++region_id) {
            const double configured = object.printing_region(region_id).config().regional_layer_height.value;
            if (configured > 0.)
                height = std::max(height, configured);
        }
    return height;
}

// Base and coarse cadence of a Body Split object, for the lagging-tower decision; zeros otherwise.
static std::pair<coordf_t, coordf_t> body_split_tower_heights(const PrintObject &object)
{
    if (! is_mixed_nozzle_body_split(object.print()->config()))
        return {0., 0.};
    const coordf_t base = object.config().layer_height.value;
    coordf_t coarse = 0.;
    for (size_t region_id = 0; region_id < object.num_printing_regions(); ++region_id) {
        const double configured = object.printing_region(region_id).config().regional_layer_height.value;
        if (configured > base + EPSILON)
            coarse = std::max(coarse, coordf_t(configured));
    }
    return {base, coarse};
}

// The raw cadence above, through the same nozzle-limit reduction as the nominal height.
coordf_t ToolOrdering::max_layer_height_for_wipe_tower(const PrintObject &object)
{
    return calc_max_layer_height(object.print()->config(), native_object_cadence_for_wipe_tower(object));
}

//calculate the flush weight (first value) and filament change count(second value)
// Nozzle-aware flush-stat calculator. Resolves each
// filament in the print sequence to its physical nozzle via the grouping result and tracks a
// per-nozzle NozzleStatusRecorder, so flush weight and flush_filament_change_count are charged
// per physical nozzle. For single-nozzle-per-extruder printers (H2D/X1/...) nozzle_id == extruder_id,
// so every returned value is identical to the extruder-level calculation. Out-of-range
// filament ids resolve to no nozzle and are skipped.
static FilamentChangeStats calc_filament_change_info_by_toolorder(const PrintConfig* config, const MultiNozzleUtils::LayeredNozzleGroupResult& group_result, const std::vector<FlushMatrix>& flush_matrix, const std::vector<std::vector<unsigned int>>& layer_sequences)
{
    FilamentChangeStats ret;
    std::unordered_map<int, int> flush_volume_per_filament;

    MultiNozzleUtils::NozzleStatusRecorder recorder;
    int total_filament_change_count = 0;
    int total_flush_filament_change_count = 0;
    float total_filament_flush_weight = 0;

    int old_filament_id = -1;
    const auto in_range = [](int index, size_t size) { return index >= 0 && size_t(index) < size; };
    for (size_t layer_idx = 0; layer_idx < layer_sequences.size(); ++layer_idx) {
        const auto& ls = layer_sequences[layer_idx];
        for (const auto& filament : ls) {
            auto nozzle = group_result.get_nozzle_for_filament(filament, layer_idx);
            if (!nozzle)
                continue;

            int new_extruder_id           = nozzle->extruder_id;
            int new_nozzle_id_in_extruder = nozzle->group_id;
            int new_filament_id_in_nozzle = filament;
            int old_filament_id_in_nozzle = recorder.get_filament_in_nozzle(new_nozzle_id_in_extruder);

            bool filament_in_nozzle_change = old_filament_id_in_nozzle != -1 && new_filament_id_in_nozzle != old_filament_id_in_nozzle;
            bool filament_change           = old_filament_id != -1 && old_filament_id != new_filament_id_in_nozzle;

            if (filament_in_nozzle_change) {
                total_flush_filament_change_count++;
                // Orca: bounds-checked; an out-of-range entry still counts the change, only its
                // volume is unknown (these stats never reach the G-code).
                if (in_range(new_extruder_id, flush_matrix.size())) {
                    const FlushMatrix &nozzle_matrix = flush_matrix[new_extruder_id];
                    if (in_range(old_filament_id_in_nozzle, nozzle_matrix.size()) &&
                        in_range(new_filament_id_in_nozzle, nozzle_matrix[old_filament_id_in_nozzle].size()))
                        flush_volume_per_filament[filament] += int(nozzle_matrix[old_filament_id_in_nozzle][new_filament_id_in_nozzle]);
                }
            }
            if (filament_change)
                total_filament_change_count++;
            old_filament_id = new_filament_id_in_nozzle;
            recorder.set_nozzle_status(new_nozzle_id_in_extruder, new_filament_id_in_nozzle, new_extruder_id);
        }
    }

    for (auto& fv : flush_volume_per_filament) {
        float weight = config->filament_density.get_at(fv.first) * 0.001 * fv.second;
        total_filament_flush_weight += weight;
    }

    ret.filament_change_count       = total_filament_change_count;
    ret.flush_filament_change_count = total_flush_filament_change_count;
    ret.filament_flush_weight       = (int)total_filament_flush_weight;

    return ret;
}

static void apply_first_layer_order(const DynamicPrintConfig* config, std::vector<unsigned int>& tool_order);

void ToolOrdering::handle_dontcare_extruder(const std::vector<unsigned int>& tool_order_layer0)
{
    const PrintConfig* print_config = m_print_config_ptr;
    if (!print_config && m_print_object_ptr)
        print_config = &m_print_object_ptr->print()->config();

    if(m_layer_tools.empty() || tool_order_layer0.empty())
        return;

    // Reorder the extruders of first layer
    {
        LayerTools& lt = m_layer_tools[0];
        std::vector<unsigned int> layer0_extruders = lt.extruders;
        lt.extruders.clear();
        for (unsigned int extruder_id : tool_order_layer0) {
            auto iter = std::find(layer0_extruders.begin(), layer0_extruders.end(), extruder_id);
            if (iter != layer0_extruders.end()) {
                lt.extruders.push_back(extruder_id);
                *iter = (unsigned int)-1;
            }
        }

        for (unsigned int extruder_id : layer0_extruders) {
            if (extruder_id == 0)
                continue;

            if (extruder_id != (unsigned int)-1)
                lt.extruders.push_back(extruder_id);
        }

        // all extruders are zero
        if (lt.extruders.empty()) {
            lt.extruders.push_back(tool_order_layer0[0]);
        }
    }

    int last_extruder_id = m_layer_tools[0].extruders.back();
    for (int i = 1; i < m_layer_tools.size(); i++) {
        LayerTools& lt = m_layer_tools[i];

        // Extruders in lt.extruders are already sorted.

        if (lt.extruders.empty())
            continue;
        if (lt.extruders.size() == 1 && lt.extruders.front() == 0)
            lt.extruders.front() = last_extruder_id;
        else {
            if (lt.extruders.front() == 0)
                // Pop the "don't care" extruder, the "don't care" region will be merged with the next one.
                lt.extruders.erase(lt.extruders.begin());

            if (print_config == nullptr
                || print_config->toolchange_ordering == ToolChangeOrderingType::Default)
            {
                // Reorder the extruders to start with the last one.
                for (size_t i = 1; i < lt.extruders.size(); ++i) {
                    if (lt.extruders[i] == last_extruder_id) {
                        // Move the last extruder to the front.
                        std::rotate(
                            lt.extruders.begin(),
                            lt.extruders.begin() + i,
                            lt.extruders.begin() + i + 1
                        );
                        break;
                    }
                }
            }
        }
        last_extruder_id = lt.extruders.back();
    }

    // Reindex the extruders, so they are zero based, not 1 based.
    for (LayerTools& lt : m_layer_tools){
        for (unsigned int& extruder_id : lt.extruders) {
            assert(extruder_id > 0);
            --extruder_id;
        }
    }
}

void ToolOrdering::handle_dontcare_extruder(unsigned int last_extruder_id)
{
    const PrintConfig* print_config = m_print_config_ptr;
    if (!print_config && m_print_object_ptr)
        print_config = &m_print_object_ptr->print()->config();

    if(m_layer_tools.empty())
        return;
    if(last_extruder_id == (unsigned int)-1){
        // The initial print extruder has not been decided yet.
        // Initialize the last_extruder_id with the first non-zero extruder id used for the print.
        last_extruder_id = 0;
        for (size_t i = 0; i < m_layer_tools.size() && last_extruder_id == 0; ++ i) {
            const LayerTools &lt = m_layer_tools[i];
            for (unsigned int extruder_id : lt.extruders)
                if (extruder_id > 0) {
                    last_extruder_id = extruder_id;
                    break;
                }
        }
        if (last_extruder_id == 0)
            // Nothing to extrude.
            return;
    }else{
        // 1 based idx
        ++ last_extruder_id;
    }

    for (LayerTools &lt : m_layer_tools) {
        // Extruders in lt.extruders are already sorted.

        if (lt.extruders.empty())
            continue;
        if (lt.extruders.size() == 1 && lt.extruders.front() == 0)
            lt.extruders.front() = last_extruder_id;
        else {
            if (lt.extruders.front() == 0)
                // Pop the "don't care" extruder, the "don't care" region will be merged with the next one.
                lt.extruders.erase(lt.extruders.begin());

            if (print_config == nullptr
                || print_config->toolchange_ordering == ToolChangeOrderingType::Default)
            {
                // Reorder the extruders to start with the last one.
                for (size_t i = 1; i < lt.extruders.size(); ++i) {
                    if (lt.extruders[i] == last_extruder_id) {
                        // Move the last extruder to the front.
                        std::rotate(
                            lt.extruders.begin(),
                            lt.extruders.begin() + i,
                            lt.extruders.begin() + i + 1
                        );
                        break;
                    }
                }
            }

            if (lt == m_layer_tools[0]) {
                // On first layer with wipe tower, prefer a soluble extruder
                // at the beginning, so it is not wiped on the first layer.
                if (print_config && print_config->enable_prime_tower) {
                    for (size_t i = 0; i<lt.extruders.size(); ++i)
                        if (print_config->filament_soluble.get_at(lt.extruders[i]-1)) { // 1-based...
                            std::swap(lt.extruders[i], lt.extruders.front());
                            break;
                        }
                }

                // Then, if we specified the tool order, apply it now
                apply_first_layer_order(m_print_full_config, lt.extruders);
            }

        }
        last_extruder_id = lt.extruders.back();
    }

    // Reindex the extruders, so they are zero based, not 1 based.
    for (LayerTools &lt : m_layer_tools){
        for (unsigned int &extruder_id : lt.extruders) {
            assert(extruder_id > 0);
            -- extruder_id;
        }
    }
}

// Whether any layer uses a filament on the coarse nozzle of a mixed-nozzle plate.
bool ToolOrdering::uses_coarse_nozzle(const PrintConfig &config) const
{
    const int coarse_extruder = mixed_nozzle_tower_coarse_extruder(config);
    if (coarse_extruder < 0)
        return true;
    for (const LayerTools &lt : m_layer_tools)
        for (unsigned int tool : lt.extruders) {
            const std::optional<size_t> extruder = physical_extruder_for_filament(config, tool);
            if (! extruder || int(*extruder) == coarse_extruder)
                return true;
        }
    return false;
}

bool ToolOrdering::insert_wipe_tower_extruder()
{
    if (!m_print_config_ptr || !m_print_config_ptr->enable_prime_tower)
        return false;
    // Belt mode has no classic wipe tower; the dedicated wipe tower filament
    // must not inject extra toolchanges into the purge prism planning.
    if (m_print_config_ptr->belt_printer)
        return false;
    if (m_print_config_ptr->wipe_tower_filament == 0)
        return false;

    bool changed = false;
    const unsigned int wipe_extruder = (unsigned int)(m_print_config_ptr->wipe_tower_filament - 1);
    for (LayerTools &lt : m_layer_tools) {
        if (lt.wipe_tower_partitions > 0) {
            if (std::find(lt.extruders.begin(), lt.extruders.end(), wipe_extruder) == lt.extruders.end()) {
                lt.extruders.emplace_back(wipe_extruder);
                changed = true;
            }
        }
    }
    return changed;
}

void ToolOrdering::sort_and_build_data(const Print& print, unsigned int first_extruder, bool prime_multi_material)
{
    // if first extruder is -1, we can decide the first layer tool order before doing reorder function
    // so we shouldn't reorder first layer in reorder function
    bool reorder_first_layer = (first_extruder != (unsigned int)(-1));
    this->resolve_mixed_filaments(print.config());
    reorder_extruders_for_minimum_flush_volume(reorder_first_layer);
    this->enforce_mixed_component_order();
    m_sorted = true;

    double max_layer_height = 0.;
    double object_bottom_z = 0.;
    // A Body Split body whose nozzle cannot lay the first layer starts on the bed with a thicker
    // first cell, so its first printing layer is not the bottom of the part. Without a raft the
    // part stands on the bed.
    const bool mixed_nozzle_on_bed = is_mixed_nozzle_slicing_enabled(print.config());
    for (const auto& object : print.objects()) {
        if (mixed_nozzle_on_bed && ! object->slicing_parameters().has_raft()) {
            max_layer_height = std::max(max_layer_height, native_object_cadence_for_wipe_tower(*object));
            continue;
        }
        for (const Layer* layer : object->layers()) {
            if (layer->has_extrusions()) {
                object_bottom_z = layer->print_z - layer->height;
                break;
            }
        }
        max_layer_height = std::max(max_layer_height, native_object_cadence_for_wipe_tower(*object));
    }

    max_layer_height = calc_max_layer_height(print.config(), max_layer_height);

    // All Body Split objects on a plate share one coarse ratio, so the first one decides.
    for (const auto &object : print.objects()) {
        const std::pair<coordf_t, coordf_t> heights = body_split_tower_heights(*object);
        if (heights.second > 0.) {
            this->set_mixed_nozzle_body_heights(heights.first, heights.second);
            break;
        }
    }

    // Feature Split can end with every band on the fine nozzle (the time check sends them back), or
    // never reach the coarse nozzle at all. The switches left are then ordinary filament changes,
    // and the tower is planned with Off's tower config: no solid base, no levels between switches.
    // Its rows stay Cadence's tower rows (see cadence_tower_rows()).
    const auto plan_tower = [&]() {
        m_tower_planned_as_off = is_mixed_nozzle_feature_split(print.config()) && ! this->uses_coarse_nozzle(print.config());
        if (! m_tower_planned_as_off) {
            this->fill_wipe_tower_partitions(print.config(), object_bottom_z, max_layer_height);
            return;
        }
        this->fill_wipe_tower_partitions(mixed_nozzle_tower_config_as_off(print.config()), object_bottom_z, max_layer_height);
    };
    plan_tower();
    if (this->insert_wipe_tower_extruder()) {
        reorder_extruders_for_minimum_flush_volume(reorder_first_layer);
        // Orca reorders a second time here (BBS has no such path); re-enforce so the
        // mixed sub-layer component order survives the extra pass.
        this->enforce_mixed_component_order();
        plan_tower();
    }

    this->collect_extruder_statistics(prime_multi_material);
}

void ToolOrdering::sort_and_build_data(const PrintObject& object , unsigned int first_extruder, bool prime_multi_material)
{
    // if first extruder is -1, we can decide the first layer tool order before doing reorder function
    // so we shouldn't reorder first layer in reorder function
    bool reorder_first_layer = (first_extruder != (unsigned int)(-1));
    this->resolve_mixed_filaments(object.print()->config());
    reorder_extruders_for_minimum_flush_volume(reorder_first_layer);
    this->enforce_mixed_component_order();
    m_sorted = true;

    double max_layer_height = ToolOrdering::max_layer_height_for_wipe_tower(object);

    {
        const std::pair<coordf_t, coordf_t> heights = body_split_tower_heights(object);
        this->set_mixed_nozzle_body_heights(heights.first, heights.second);
    }

    this->fill_wipe_tower_partitions(object.print()->config(), object.layers().front()->print_z - object.layers().front()->height, max_layer_height);
    if (this->insert_wipe_tower_extruder()) {
        reorder_extruders_for_minimum_flush_volume(reorder_first_layer);
        // Orca reorders a second time here (BBS has no such path); re-enforce so the
        // mixed sub-layer component order survives the extra pass.
        this->enforce_mixed_component_order();
        this->fill_wipe_tower_partitions(object.print()->config(), object.layers().front()->print_z - object.layers().front()->height, max_layer_height);
    }

    this->collect_extruder_statistics(prime_multi_material);
}


// For the use case when each object is printed separately
// (print->config().print_sequence == PrintSequence::ByObject is true).
ToolOrdering::ToolOrdering(const PrintObject &object, unsigned int first_extruder, bool prime_multi_material)
{
    m_print_full_config = &object.print()->full_print_config();
    m_print_config_ptr = &object.print()->config();
    m_print_object_ptr = &object;
    m_print = const_cast<Print*>(object.print());
    if (object.layers().empty())
        return;

    // Initialize the print layers for just a single object.
    {
        // construct layer tools by z height
        std::vector<coordf_t> zs;
        zs.reserve(zs.size() + object.layers().size() + object.support_layers().size());
        for (auto layer : object.layers())
            zs.emplace_back(layer->print_z);
        for (auto layer : object.support_layers())
            zs.emplace_back(layer->print_z);
        // Belt brim apron bands sit below the object's first layer and have no
        // layer of their own, but tools_for_layer() asserts an exact Z match, so
        // their print_z must be part of the ordering.
        for (const BeltBrimBand &band : object.belt_brim_prologue())
            zs.emplace_back(band.print_z);
        this->initialize_layers(zs);
    }

    // Collect extruders reuqired to print the layers. Add dontcare extruders
    this->collect_extruders(object, std::vector<std::pair<double, unsigned int>>());

    // BBS
    // Reorder the extruders to minimize tool switches.
    std::vector<unsigned int> first_layer_tool_order;
    if (first_extruder == (unsigned int) -1) {
        first_layer_tool_order = generate_first_layer_tool_order(object);
    }

    if (!first_layer_tool_order.empty()) {
        this->handle_dontcare_extruder(first_layer_tool_order);
    } else {
        this->handle_dontcare_extruder(first_extruder);
    }

    this->collect_extruder_statistics(prime_multi_material);

    double max_layer_height = ToolOrdering::max_layer_height_for_wipe_tower(object);

    m_first_layer_band = object.layers().size() > 1 && object.layers()[1]->in_first_layer_band();
    this->mark_skirt_layers(object.print()->config(), max_layer_height);
}

// For the use case when all objects are printed at once.
// (print->config().print_sequence == PrintSequence::ByObject is false).
ToolOrdering::ToolOrdering(const Print &print, unsigned int first_extruder, bool prime_multi_material)
{
    m_print_full_config = &print.full_print_config();
    m_print = const_cast<Print *>(&print);  // for update the context of print
    m_print_config_ptr = &print.config();

    // Initialize the print layers for all objects and all layers.
    coordf_t max_layer_height = 0.;
    {
        std::vector<coordf_t> zs;
        for (auto object : print.objects()) {
            zs.reserve(zs.size() + object->layers().size() + object->support_layers().size());
            for (auto layer : object->layers())
                zs.emplace_back(layer->print_z);
            for (auto layer : object->support_layers())
                zs.emplace_back(layer->print_z);
            // See the single-object ctor: belt brim apron bands need their own
            // ordering entries or tools_for_layer() will assert.
            for (const BeltBrimBand &band : object->belt_brim_prologue())
                zs.emplace_back(band.print_z);

            max_layer_height = std::max(max_layer_height, native_object_cadence_for_wipe_tower(*object));
        }
        this->initialize_layers(zs);
    }
    max_layer_height = calc_max_layer_height(print.config(), max_layer_height);

	// Use the extruder switches from Model::custom_gcode_per_print_z to override the extruder to print the object.
	// Do it only if all the objects were configured to be printed with a single extruder.
	std::vector<std::pair<double, unsigned int>> per_layer_extruder_switches;

    // BBS
	if (auto num_filaments = unsigned(print.config().filament_diameter.size());
		num_filaments > 1 && print.object_extruders().size() == 1 && // the current Print's configuration is CustomGCode::MultiAsSingle
        //BBS: replace model custom gcode with current plate custom gcode
        print.model().get_curr_plate_custom_gcodes().mode == CustomGCode::MultiAsSingle) {
		// Printing a single extruder platter on a printer with more than 1 extruder (or single-extruder multi-material).
		// There may be custom per-layer tool changes available at the model.
        per_layer_extruder_switches = custom_tool_changes(print.model().get_curr_plate_custom_gcodes(), num_filaments);
	}

    // Collect extruders reuqired to print the layers.
    for (auto object : print.objects())
        this->collect_extruders(*object, per_layer_extruder_switches);

    // Reorder the extruders to minimize tool switches.
    std::vector<unsigned int> first_layer_tool_order;
    if (first_extruder == (unsigned int)-1) {
        first_layer_tool_order = generate_first_layer_tool_order(print);
    }

    if(!first_layer_tool_order.empty())
        this->handle_dontcare_extruder(first_layer_tool_order);
    else
        this->handle_dontcare_extruder(first_extruder);

    this->collect_extruder_statistics(prime_multi_material);

    m_first_layer_band = std::any_of(print.objects().begin(), print.objects().end(), [](const PrintObject *object) {
        return object->layers().size() > 1 && object->layers()[1]->in_first_layer_band();
    });
    this->mark_skirt_layers(print.config(), max_layer_height);
}

static void apply_first_layer_order(const DynamicPrintConfig* config, std::vector<unsigned int>& tool_order) {
    const ConfigOptionInts* first_layer_print_sequence_op = config->option<ConfigOptionInts>("first_layer_print_sequence");
    if (first_layer_print_sequence_op) {
        const std::vector<int>& print_sequence_1st = first_layer_print_sequence_op->values;
        if (print_sequence_1st.size() >= tool_order.size()) {
            std::sort(tool_order.begin(), tool_order.end(), [&print_sequence_1st](int lh, int rh) {
                auto lh_it = std::find(print_sequence_1st.begin(), print_sequence_1st.end(), lh);
                auto rh_it = std::find(print_sequence_1st.begin(), print_sequence_1st.end(), rh);

                if (lh_it == print_sequence_1st.end() || rh_it == print_sequence_1st.end())
                    return false;

                return lh_it < rh_it;
            });
        }
    }
}

// BBS
std::vector<unsigned int> ToolOrdering::generate_first_layer_tool_order(const Print& print)
{
    std::vector<unsigned int> tool_order;
    int initial_extruder_id = -1;
    std::map<int, double> min_areas_per_extruder;

    for (auto object : print.objects()) {
        const Layer* target_layer = nullptr;
        for(auto layer : object->layers()){
            for(auto layerm : layer->regions()){
                for(auto& expoly : layerm->raw_slices){
                    if (!offset_ex(expoly, -0.2 * scale_(print.config().initial_layer_line_width)).empty()) {
                        target_layer = layer;
                        break;
                    }
                }
                if(target_layer)
                    break;
            }
            if(target_layer)
                break;
        }

        if(!target_layer)
            return tool_order;

        for (auto layerm : target_layer->regions()) {
            int extruder_id = layerm->region().config().option("outer_wall_filament_id")->getInt();

            for (auto expoly : layerm->raw_slices) {
                const double nozzle_diameter = print.config().nozzle_diameter.get_at(0);
                const coordf_t initial_layer_line_width = print.config().get_abs_value("initial_layer_line_width", nozzle_diameter);

                if (offset_ex(expoly, -0.2 * scale_(initial_layer_line_width)).empty())
                    continue;

                double contour_area = expoly.contour.area();
                auto iter = min_areas_per_extruder.find(extruder_id);
                if (iter == min_areas_per_extruder.end()) {
                    min_areas_per_extruder.insert({ extruder_id, contour_area });
                }
                else {
                    if (contour_area < min_areas_per_extruder.at(extruder_id)) {
                        min_areas_per_extruder[extruder_id] = contour_area;
                    }
                }
            }
        }
    }

    double max_minimal_area = 0.;
    for (auto ape : min_areas_per_extruder) {
        auto iter = tool_order.begin();
        for (; iter != tool_order.end(); iter++) {
            if (min_areas_per_extruder.at(*iter) < min_areas_per_extruder.at(ape.first))
                break;
        }

        tool_order.insert(iter, ape.first);
    }

    apply_first_layer_order(m_print_full_config, tool_order);

    return tool_order;
}

std::vector<unsigned int> ToolOrdering::generate_first_layer_tool_order(const PrintObject& object)
{
    std::vector<unsigned int> tool_order;
    int initial_extruder_id = -1;
    std::map<int, double> min_areas_per_extruder;
    const Layer* target_layer = nullptr;
    for(auto layer : object.layers()){
        for(auto layerm : layer->regions()){
            for(auto& expoly : layerm->raw_slices){
                if (!offset_ex(expoly, -0.2 * scale_(object.config().line_width)).empty()) {
                    target_layer = layer;
                    break;
                }
            }
            if(target_layer)
                break;
        }
        if(target_layer)
            break;
    }

    if(!target_layer)
        return tool_order;

    for (auto layerm : target_layer->regions()) {
        int extruder_id = layerm->region().config().option("outer_wall_filament_id")->getInt();
        for (auto expoly : layerm->raw_slices) {
            const double nozzle_diameter = object.print()->config().nozzle_diameter.get_at(0);
            const coordf_t line_width = object.config().get_abs_value("line_width", nozzle_diameter);

            if (offset_ex(expoly, -0.2 * scale_(line_width)).empty())
                continue;

            double contour_area = expoly.contour.area();
            auto iter = min_areas_per_extruder.find(extruder_id);
            if (iter == min_areas_per_extruder.end()) {
                min_areas_per_extruder.insert({ extruder_id, contour_area });
            }
            else {
                if (contour_area < min_areas_per_extruder.at(extruder_id)) {
                    min_areas_per_extruder[extruder_id] = contour_area;
                }
            }
        }
    }

    double max_minimal_area = 0.;
    for (auto ape : min_areas_per_extruder) {
        auto iter = tool_order.begin();
        for (; iter != tool_order.end(); iter++) {
            if (min_areas_per_extruder.at(*iter) < min_areas_per_extruder.at(ape.first))
                break;
        }

        tool_order.insert(iter, ape.first);
    }

    apply_first_layer_order(m_print_full_config, tool_order);

    return tool_order;
}

void ToolOrdering::initialize_layers(std::vector<coordf_t> &zs)
{
    sort_remove_duplicates(zs);
    // Merge numerically very close Z values.
    for (size_t i = 0; i < zs.size();) {
        // Find the last layer with roughly the same print_z.
        size_t j = i + 1;
        coordf_t zmax = zs[i] + EPSILON;
        for (; j < zs.size() && zs[j] <= zmax; ++ j) ;
        // Assign an average print_z to the set of layers with nearly equal print_z.
        m_layer_tools.emplace_back(LayerTools(0.5 * (zs[i] + zs[j-1])));
        i = j;
    }
}

// Collect extruders reuqired to print layers.
void ToolOrdering::collect_extruders(const PrintObject &object, const std::vector<std::pair<double, unsigned int>> &per_layer_extruder_switches)
{
    // Extruder overrides are ordered by print_z.
    std::vector<std::pair<double, unsigned int>>::const_iterator it_per_layer_extruder_override;
	it_per_layer_extruder_override = per_layer_extruder_switches.begin();
    unsigned int extruder_override = 0;

    // Pre-compute 1-based IDs of mixed filament slots for per-object tracking.
    // mixed_slots_1based covers ALL mixed slots (needed by calc_slot_lh for
    // accurate layer height when a slot skips layers). gradient_slots_1based
    // and per_part_slots_1based are subsets for gradient-specific logic.
    std::set<unsigned int> mixed_slots_1based;
    std::set<unsigned int> gradient_slots_1based;
    std::set<unsigned int> per_part_slots_1based;
    {
        const PrintConfig &cfg       = object.print()->config();
        const auto        &is_mixed  = cfg.filament_is_mixed.values;
        const auto        &grad_flags = cfg.filament_mixed_gradient.values;
        const auto        &per_part_flags = cfg.filament_mixed_gradient_per_part.values;
        const auto        &comp_strs = cfg.filament_mixed_components.values;
        for (size_t i = 0; i < is_mixed.size(); ++i) {
            if (!is_mixed[i])
                continue;
            auto comps = parse_mixed_components(i < comp_strs.size() ? comp_strs[i] : "");
            if (comps.size() < 2)
                continue;
            mixed_slots_1based.insert(static_cast<unsigned int>(i + 1));
            // Gradient/per-part are only defined for 2-component slots; keep their
            // tracking limited to them (mirrors the is_gradient guard at resolve time).
            if (comps.size() != 2)
                continue;
            if (i >= grad_flags.size() || !grad_flags[i])
                continue;
            gradient_slots_1based.insert(static_cast<unsigned int>(i + 1));
            if (i < per_part_flags.size() && per_part_flags[i])
                per_part_slots_1based.insert(static_cast<unsigned int>(i + 1));
        }
    }

    // BBS: collect first layer extruders of an object's wall, which will be used by brim generator
    int layerCount = 0;
    std::vector<int> firstLayerExtruders;
    firstLayerExtruders.clear();

    // Collect the object extruders.
    for (auto layer : object.layers()) {
        LayerTools &layer_tools = this->tools_for_layer(layer->print_z);

        m_object_all_layer_indices[&object].push_back(
            static_cast<size_t>(&layer_tools - m_layer_tools.data()));

        // Override extruder with the next
    	for (; it_per_layer_extruder_override != per_layer_extruder_switches.end() && it_per_layer_extruder_override->first < layer->print_z + EPSILON; ++ it_per_layer_extruder_override)
    		extruder_override = (int)it_per_layer_extruder_override->second;

        // Store the current extruder override (set to zero if no overriden), so that layer_tools.wiping_extrusions().is_overridable_and_mark() will use it.
        layer_tools.extruder_override = extruder_override;

        // Snapshot extruders before this object's regions to track new additions.
        const size_t ext_snapshot = layer_tools.extruders.size();

        // What extruders are required to print this object layer?
        for (const LayerRegion *layerm : layer->regions()) {
            const PrintRegion &region = layerm->region();

            if (is_mixed_nozzle_feature_split(object.print()->config())) {
                bool has_feature_object_extrusions = false;
                std::vector<int> *first_layer_extruders = layerCount == 0 ? &firstLayerExtruders : nullptr;
                for (const ExtrusionEntity *entity : layerm->perimeters.entities)
                    if (entity != nullptr)
                        has_feature_object_extrusions |= collect_feature_object_leaf_filaments(*entity, region, layer_tools, extruder_override, first_layer_extruders, layerm);
                for (const ExtrusionEntity *entity : layerm->fills.entities)
                    if (entity != nullptr)
                        has_feature_object_extrusions |= collect_feature_object_leaf_filaments(*entity, region, layer_tools, extruder_override, first_layer_extruders, layerm);
                layer_tools.has_object |= has_feature_object_extrusions;
                continue;
            }

            // A Body Split cell with its own tool deposits every role with it; zero keeps the
            // region's per-role ids.
            const int  cell_filament = layerm->cell_filament_id();
            const auto role_filament = [cell_filament](int region_filament) {
                return cell_filament > 0 ? cell_filament : region_filament;
            };

            if (! layerm->perimeters.entities.empty()) {
                bool something_nonoverriddable = true;

                if (m_print_config_ptr) { // in this case print->config().print_sequence != PrintSequence::ByObject (see ToolOrdering constructors)
                    something_nonoverriddable = false;
                    for (const auto& eec : layerm->perimeters.entities) // let's check if there are nonoverriddable entities
                        if (!layer_tools.wiping_extrusions().is_overriddable_and_mark(dynamic_cast<const ExtrusionEntityCollection&>(*eec), *m_print_config_ptr, object, region))
                            something_nonoverriddable = true;
                }

                if (something_nonoverriddable){
               		layer_tools.extruders.emplace_back((extruder_override == 0) ? role_filament(region.config().outer_wall_filament_id.value) : extruder_override);
                    const ExtrusionEntitiesPtr &islands = layerm->perimeters.entities;
                    const bool has_inner_wall = std::any_of(islands.begin(), islands.end(), [](const ExtrusionEntity *island) {
                        const ExtrusionEntitiesPtr &walls = static_cast<const ExtrusionEntityCollection*>(island)->entities;
                        return std::any_of(walls.begin(), walls.end(), [](const ExtrusionEntity *ee) { return is_inner_wall(*ee); });
                    });
                    if (extruder_override == 0 && region.config().wall_loops.value > 1 && has_inner_wall)
                        layer_tools.extruders.emplace_back(role_filament(region.config().inner_wall_filament_id.value));
                    if (layerCount == 0) {
                        firstLayerExtruders.emplace_back((extruder_override == 0) ? role_filament(region.config().outer_wall_filament_id.value) : extruder_override);
                    }
                }

                layer_tools.has_object = true;
            }

            std::vector<unsigned int> fill_filaments;
            bool something_nonoverriddable = false;
            for (const ExtrusionEntity *ee : layerm->fills.entities) {
                // fill represents infill extrusions of a single island.
                const auto *fill = dynamic_cast<const ExtrusionEntityCollection*>(ee);
                if (! fill->entities.empty())
                    fill_filaments.emplace_back(region_filament(*fill, region.config()));

                if (m_print_config_ptr) {
                    if (! layer_tools.wiping_extrusions().is_overriddable_and_mark(*fill, *m_print_config_ptr, object, region))
                        something_nonoverriddable = true;
                }
            }

            if (something_nonoverriddable || !m_print_config_ptr) {
            	if (extruder_override == 0) {
                    for (unsigned int fill_filament : fill_filaments)
                        layer_tools.extruders.emplace_back(role_filament(int(fill_filament)));
                } else if (! fill_filaments.empty())
            		layer_tools.extruders.emplace_back(extruder_override);
            }
            if (! fill_filaments.empty())
                layer_tools.has_object = true;
        }

        // Record mixed slot usage for this object at this layer.
        // All mixed slots are tracked (not just gradient) so that calc_slot_lh
        // can compute accurate layer heights even when a slot skips layers.
        if (!mixed_slots_1based.empty()) {
            size_t layer_idx = static_cast<size_t>(&layer_tools - m_layer_tools.data());
            std::set<unsigned int> seen;
            for (size_t ei = ext_snapshot; ei < layer_tools.extruders.size(); ++ei) {
                unsigned int ext_1based = layer_tools.extruders[ei];
                if (mixed_slots_1based.count(ext_1based) && seen.insert(ext_1based).second)
                    m_mixed_object_layers[ext_1based - 1][&object].push_back(layer_idx);
            }
        }

        // Per-part gradient: walk LayerRegions and record which (slot, ModelVolume) pairs
        // contributed to this layer. Only regions tagged by PrintApply.cpp's get_create_region
        // (i.e. gradient_volume_id().valid()) are considered, so this loop is a strict no-op
        // unless per_part_gradient is enabled for at least one slot AND the corresponding
        // ModelObject has >=2 model-part volumes using that slot. The per-object pass above is
        // unaffected — both run the same layer's data through orthogonal containers.
        if (!per_part_slots_1based.empty()) {
            size_t layer_idx = static_cast<size_t>(&layer_tools - m_layer_tools.data());
            std::set<std::pair<unsigned int, ObjectID>> vol_seen;
            for (const LayerRegion *layerm : layer->regions()) {
                if (layerm->slices.empty())
                    continue;
                const PrintRegion &region = layerm->region();
                ObjectID vol_id = region.gradient_volume_id();
                if (! vol_id.valid())
                    continue;
                const PrintRegionConfig &rcfg = region.config();
                // Orca splits BBS's three role slots into five; cover them all so a mixed
                // slot used by any role is tracked.
                const unsigned int role_slots[5] = {
                    static_cast<unsigned int>(rcfg.outer_wall_filament_id.value),
                    static_cast<unsigned int>(rcfg.inner_wall_filament_id.value),
                    static_cast<unsigned int>(rcfg.sparse_infill_filament_id.value),
                    static_cast<unsigned int>(rcfg.top_surface_filament_id.value),
                    static_cast<unsigned int>(rcfg.bottom_surface_filament_id.value),
                };
                for (unsigned int ext_1based : role_slots) {
                    if (ext_1based >= 1
                        && per_part_slots_1based.count(ext_1based)
                        && vol_seen.insert({ext_1based, vol_id}).second)
                        m_gradient_volume_layers[ext_1based - 1][{&object, vol_id}].push_back(layer_idx);
                }
            }
        }
        layerCount++;
    }

    sort_remove_duplicates(firstLayerExtruders);
    const_cast<PrintObject&>(object).object_first_layer_wall_extruders = firstLayerExtruders;

    // Collect the support extruders.
    for (auto support_layer : object.support_layers()) {
        LayerTools   &layer_tools   = this->tools_for_layer(support_layer->print_z);
        ExtrusionRole role          = support_layer->support_fills.role();
        bool          has_support   = false;
        bool          has_interface = false;
        for (const ExtrusionEntity *ee : support_layer->support_fills.entities) {
            ExtrusionRole er = ee->role();
            if (er == erSupportMaterial || er == erSupportTransition) has_support = true;
            if (er == erSupportMaterialInterface) has_interface = true;
            if (has_support && has_interface) break;
        }
        unsigned int extruder_support   = object.config().support_filament.value;
        unsigned int extruder_interface = object.config().support_interface_filament.value;
        // A mixed-nozzle layer whose support body the interface nozzle lays prints it with the body
        // filament picked for the object on that nozzle.
        if (has_support && support_layer->base_on_interface_nozzle)
            extruder_support = support_layer->interface_nozzle_body_filament + 1;
        if (has_support) {
            if (extruder_support > 0 || !has_interface || extruder_interface == 0 || layer_tools.has_object)
                layer_tools.extruders.push_back(extruder_support);
            else {
                auto all_extruders     = object.print()->extruders();
                auto get_next_extruder = [&](int current_extruder, const std::vector<unsigned int> &extruders) {
                    std::vector<float> flush_matrix(
                        cast<float>(get_flush_volumes_matrix(object.print()->config().flush_volumes_matrix.values, 0, object.print()->config().nozzle_diameter.values.size())));
                    const unsigned int number_of_extruders = (unsigned int) (sqrt(flush_matrix.size()) + EPSILON);
                    // Extract purging volumes for each extruder pair:
                    std::vector<std::vector<float>> wipe_volumes;
                    for (unsigned int i = 0; i < number_of_extruders; ++i)
                        wipe_volumes.push_back(std::vector<float>(flush_matrix.begin() + i * number_of_extruders, flush_matrix.begin() + (i + 1) * number_of_extruders));
                    int   next_extruder = current_extruder;
                    float min_flush     = std::numeric_limits<float>::max();
                    for (auto extruder_id : extruders) {
                        if (object.print()->config().filament_soluble.get_at(extruder_id) || extruder_id == current_extruder) continue;
                        if (wipe_volumes[extruder_interface - 1][extruder_id] < min_flush) {
                            next_extruder = extruder_id;
                            min_flush     = wipe_volumes[extruder_interface - 1][extruder_id];
                        }
                    }
                    return next_extruder;
                };
                bool interface_not_for_body = object.config().support_interface_not_for_body;
                layer_tools.extruders.push_back(get_next_extruder(interface_not_for_body ? extruder_interface - 1 : -1, all_extruders) + 1);
            }
        }
        if (has_interface) layer_tools.extruders.push_back(extruder_interface);
        // Body the interface nozzle lays beside coarse body roads has its own filament on this layer.
        if (! support_layer->fine_body_fills.empty()) {
            layer_tools.extruders.push_back(support_layer->interface_nozzle_body_filament + 1);
            has_support = true;
        }
        if (has_support || has_interface) {
            layer_tools.has_support = true;
            layer_tools.wiping_extrusions().is_support_overriddable_and_mark(role, object);
        }
    }

    // Belt brim apron bands own their layers outright: they print below the
    // object's first layer, so no object or support layer claims an extruder there
    // and process_layer() would bail out at "Nothing to extrude".  Claim the
    // object's outer wall filament, in the same raw 1-based domain the loops above
    // push.  Deliberately not layer_tools.has_object, which drives skirt marking
    // and wiping overrides.
    if (! object.belt_brim_prologue().empty()) {
        // 1-based, same domain the object/support pushes above use; reindexed to 0-based
        // with the rest of the list later.
        const unsigned int brim_filament = object.belt_brim_filament();
        for (const BeltBrimBand &band : object.belt_brim_prologue()) {
            if (band.fills.empty())
                continue;
            LayerTools &layer_tools = this->tools_for_layer(band.print_z);
            layer_tools.extruders.push_back(brim_filament);
        }
    }

    // Coincident brim bands (belt_brim_by_layer) print ON an object layer rather than
    // below it, but that layer can produce no InstanceVisit in process_layer - a
    // zero-extrusion lead-in slice with no coinciding support - and the band would then
    // be silently dropped.  Register the brim filament on every layer that carries a
    // coincident band, in the same 1-based domain as the prologue push above, so a brim
    // pass always exists there.
    if (object.has_belt_brim()) {
        const unsigned int brim_filament = object.belt_brim_filament();
        const auto        &by_layer      = object.belt_brim_by_layer();
        const size_t       n             = std::min(by_layer.size(), object.layers().size());
        for (size_t i = 0; i < n; ++ i) {
            if (by_layer[i].empty())
                continue;
            LayerTools &layer_tools = this->tools_for_layer(object.layers()[i]->print_z);
            layer_tools.extruders.push_back(brim_filament);
        }
    }

    for (auto& layer : m_layer_tools) {
        // Sort and remove duplicates
        sort_remove_duplicates(layer.extruders);

        // make sure that there are some tools for each object layer (e.g. tall wiping object will result in empty extruders vector)
        if (layer.extruders.empty() && layer.has_object)
            layer.extruders.emplace_back(0); // 0="dontcare" extruder - it will be taken care of in reorder_extruders
    }
}


// Resolved max layer height of the nozzle a logical filament prints on. In mixed-nozzle modes an
// unresolved tool returns zero (the caller rejects it); otherwise it falls back to the largest nozzle.
static coordf_t wipe_tower_tool_max_layer_height(const PrintConfig &config, unsigned int tool)
{
    if (tool < config.filament_map.values.size()) {
        const int nozzle_idx = config.filament_map.values[tool] - 1;
        if (nozzle_idx >= 0 && size_t(nozzle_idx) < config.nozzle_diameter.values.size()) {
            const coordf_t resolved = resolved_max_layer_height(config, size_t(nozzle_idx));
            if (!is_mixed_nozzle_slicing_enabled(config) ||
                (std::isfinite(resolved) && resolved > 0.))
                return resolved;
            return 0.;
        }
    }
    if (is_mixed_nozzle_slicing_enabled(config))
        return 0.;
    coordf_t best = 0.;
    for (size_t i = 0; i < config.nozzle_diameter.values.size(); ++i)
        best = std::max(best, resolved_max_layer_height(config, i));
    return best;
}

// The tool that lays the first tower block on a switch layer: the first entry in lt.extruders
// that differs from the active tool. The accumulated gap must fit that tool's limit, which is not
// necessarily lt.extruders.back() when a layer has several switches.
static unsigned int wipe_tower_arriving_tool(const LayerTools &lt, unsigned int active_tool)
{
    for (unsigned int tool : lt.extruders)
        if (tool != active_tool)
            return tool;
    // Not expected on a switch layer; fall back to the layer's last tool.
    return lt.extruders.empty() ? active_tool : lt.extruders.back();
}

bool ToolOrdering::has_wipe_tower() const
{
    if (m_layer_tools.empty() || m_first_printing_extruder == (unsigned int)-1)
        return false;
    const auto first_printing = std::find_if(m_layer_tools.begin(), m_layer_tools.end(),
        [](const LayerTools &lt) { return lt.has_object || lt.has_support; });
    return (first_printing != m_layer_tools.end() ? *first_printing : m_layer_tools.front()).has_wipe_tower;
}

void ToolOrdering::fill_wipe_tower_partitions(const PrintConfig &config, coordf_t object_bottom_z, coordf_t max_layer_height)
{
    // A second pass can follow once the wipe-tower filament is inserted: remove only rows the
    // previous pass synthesized; Print::_make_wipe_tower reassigns event identities.
    m_layer_tools.erase(std::remove_if(m_layer_tools.begin(), m_layer_tools.end(),
        [](const LayerTools &lt) { return lt.tower_support_event; }), m_layer_tools.end());
    for (LayerTools &lt : m_layer_tools) {
        lt.wipe_tower_event_index = size_t(-1);
        lt.tower_support_event = false;
        lt.wipe_tower_emit = false;
    }
    if (m_layer_tools.empty())
        return;

    const bool mixed_nozzle_tower = is_mixed_nozzle_feature_split(config) ||
                                    is_mixed_nozzle_body_split(config);
    coordf_t structural_max_height = max_layer_height;
    if (mixed_nozzle_tower) {
        // Every tower tool must resolve to a nozzle with a valid limit; falling back to the
        // largest nozzle could let an over-limit gap through.
        if (config.nozzle_diameter.values.empty() || config.min_layer_height.values.empty() ||
            config.max_layer_height.values.empty())
            throw Slic3r::SlicingError(
                "[SRL-TOWER-STRUCTURE] Mixed-nozzle tower scheduling requires complete effective nozzle layer-height limits.");
        structural_max_height = std::numeric_limits<coordf_t>::max();
        bool found_participating_nozzle = false;
        for (const LayerTools &lt : m_layer_tools)
            for (unsigned int tool : lt.extruders) {
                if (tool >= config.filament_map.values.size())
                    throw Slic3r::SlicingError(
                        "[SRL-TOWER-STRUCTURE] No effective nozzle mapping exists for a scheduled tower tool.");
                const int nozzle_idx = config.filament_map.values[tool] - 1;
                if (nozzle_idx < 0 || size_t(nozzle_idx) >= config.nozzle_diameter.values.size())
                    throw Slic3r::SlicingError(Slic3r::format(
                        "[SRL-TOWER-STRUCTURE] Prime tower filament %1% is not assigned to a nozzle.",
                        tool + 1));
                const coordf_t resolved = resolved_max_layer_height(config, size_t(nozzle_idx));
                if (!std::isfinite(resolved) || resolved <= 0.)
                    throw Slic3r::SlicingError(Slic3r::format(
                        "[SRL-TOWER-STRUCTURE] Nozzle %1% has no valid maximum layer height.",
                        nozzle_idx + 1));
                structural_max_height = std::min(structural_max_height, resolved);
                found_participating_nozzle = true;
            }
        if (!found_participating_nozzle)
            structural_max_height = max_layer_height;
    }

    // Count the minimum number of tool changes per layer, and record each row's own switch before
    // the propagation below overwrites the counts. The raft branch can insert a row, so it
    // recomputes these records afterwards.
    size_t last_extruder = size_t(-1);
    size_t last_tool_change_layer = size_t(-1);
    // Whether each row has its own real switch. The propagation below makes partitions a
    // footprint-continuity signal, but a compact tower deposits only on switch rows, and its block
    // height is the gap since the last emitted row.
    std::vector<bool> layer_has_own_tool_change(m_layer_tools.size(), false);
    for (size_t layer_idx = 0; layer_idx < m_layer_tools.size(); ++layer_idx) {
        LayerTools &lt = m_layer_tools[layer_idx];
        lt.wipe_tower_partitions = lt.extruders.size();
        if (! lt.extruders.empty()) {
            if (last_extruder == size_t(-1) || last_extruder == lt.extruders.front())
                // The first extruder on this layer is equal to the current one, no need to do an initial tool change.
                -- lt.wipe_tower_partitions;
            last_extruder = lt.extruders.back();
        }
        if (lt.wipe_tower_partitions > 0) {
            last_tool_change_layer = layer_idx;
            layer_has_own_tool_change[layer_idx] = true;
        }
    }

    // Propagate the wipe tower partitions down to support the upper partitions by the lower partitions.
    for (int i = int(m_layer_tools.size()) - 2; i >= 0; -- i)
        m_layer_tools[i].wipe_tower_partitions = std::max(m_layer_tools[i + 1].wipe_tower_partitions, m_layer_tools[i].wipe_tower_partitions);


    int wrapping_layer_nums = config.wrapping_detection_layers;
    for (size_t i = 0; i < wrapping_layer_nums; ++i) {
        if (i >= m_layer_tools.size())
            break;
        LayerTools &lt    = m_layer_tools[i];
        lt.has_wipe_tower = config.enable_wrapping_detection;
    }

    //FIXME this is a hack to get the ball rolling.
    // The `print_z < object_bottom_z` clause reads "below the object" as "raft
    // gap".  On a belt printer that is wrong: the brim apron legitimately prints
    // below the object's first layer, and treating those layers as raft would put a
    // wipe tower at negative Z.  A belt printer never prints the classic
    // prime tower (Print::has_wipe_tower()), so simply drop the clause there.
    //
    // Gate on config.belt_printer, NOT on has_belt_brim: every layer below the
    // object bottom on a belt printer is legitimately a sub-object stream - brim
    // apron, belt support printed below Z0, or the object's own lead-in - and none of
    // them is ever raft, because Print::validate() rejects raft_layers>0 on a belt
    // printer outright.  Narrowing this to has_belt_brim would reclassify
    // belt-support-below-floor layers as raft on brim-less belt prints and reintroduce
    // the negative-Z wipe tower, so the broad belt_printer gate is correct.
    const bool belt_no_raft_gap = config.belt_printer.value;
    // Cadence modes, and a Feature Split tower planned as Off (its config says Off), keep Cadence's
    // tower rows: only object layers are marked here, and the continuity pass below does not run.
    // Cadence marks support-only rows itself (first printing row, and rows up to the last switch
    // that change tool), and its compact and lagging towers decide their own levels.
    const bool cadence_tower_rows = is_mixed_nozzle_slicing_enabled(config) || m_tower_planned_as_off;
    m_cadence_tower_rows = cadence_tower_rows;
    for (LayerTools &lt : m_layer_tools)
        lt.has_wipe_tower |= ((lt.has_object || (! cadence_tower_rows && lt.has_support)) && (config.timelapse_type == TimelapseType::tlSmooth || lt.wipe_tower_partitions > 0))
            || (! belt_no_raft_gap && lt.print_z < object_bottom_z + EPSILON);
    // The first layer that prints can hold only support: the support's bed layer under a Body Split
    // body whose nozzle cannot lay the first layer and starts with a taller cell. has_wipe_tower()
    // reads that layer, so without a level there the plate got no tower at all. It is the tower's
    // bed level whenever the plate switches tools.
    if (is_mixed_nozzle_slicing_enabled(config)) {
        const auto first_printing = std::find_if(m_layer_tools.begin(), m_layer_tools.end(),
            [](const LayerTools &lt) { return lt.has_object || lt.has_support; });
        if (first_printing != m_layer_tools.end() && ! first_printing->has_object && first_printing->wipe_tower_partitions > 0)
            first_printing->has_wipe_tower = true;
    }

    // Test for a raft, insert additional wipe tower layer to fill in the raft separation gap.
    // Skipped on belt printers for the same reason as the clause above: layers
    // below the object are brim apron, not raft.
    bool raft_row_inserted = false;
    for (size_t i = 0; ! belt_no_raft_gap && i + 1 < m_layer_tools.size(); ++ i) {
        const LayerTools &lt      = m_layer_tools[i];
        const LayerTools &lt_next = m_layer_tools[i + 1];
        if (lt.print_z < object_bottom_z + EPSILON && lt_next.print_z >= object_bottom_z + EPSILON) {
            // lt is the last raft layer. Find the 1st object layer.
            size_t j = i + 1;
            for (; j < m_layer_tools.size() && ! m_layer_tools[j].has_wipe_tower; ++ j);
            if (j < m_layer_tools.size()) {
                const LayerTools &lt_object = m_layer_tools[j];
                coordf_t gap = lt_object.print_z - lt.print_z;
                assert(gap > 0.f);
                if (gap > max_layer_height + EPSILON) {
                    // Insert one additional wipe tower layer between lh.print_z and lt_object.print_z.
                    LayerTools lt_new(0.5f * (lt.print_z + lt_object.print_z));
                    // Find the 1st layer above lt_new.
                    for (j = i + 1; j < m_layer_tools.size() && m_layer_tools[j].print_z < lt_new.print_z - EPSILON; ++ j);
                    if (std::abs(m_layer_tools[j].print_z - lt_new.print_z) < EPSILON) {
						m_layer_tools[j].has_wipe_tower = true;
					} else {
						LayerTools &lt_extra = *m_layer_tools.insert(m_layer_tools.begin() + j, lt_new);
                        raft_row_inserted = true;
                        //LayerTools &lt_prev  = m_layer_tools[j];
                        LayerTools &lt_next  = m_layer_tools[j + 1];
                        assert(! m_layer_tools[j - 1].extruders.empty() && ! lt_next.extruders.empty());
                        // FIXME: Following assert tripped when running combine_infill.t. I decided to comment it out for now.
                        // If it is a bug, it's likely not critical, because this code is unchanged for a long time. It might
                        // still be worth looking into it more and decide if it is a bug or an obsolete assert.
                        //assert(lt_prev.extruders.back() == lt_next.extruders.front());
                        lt_extra.has_wipe_tower = true;
                        lt_extra.extruders.push_back(lt_next.extruders.front());
                        lt_extra.wipe_tower_partitions = lt_next.wipe_tower_partitions;
                    }
                }
            }
            break;
        }
    }

    // Ensure wipe tower vertical continuity:
    //
    // (1) Any existing LayerTools sandwiched between two has_wipe_tower layers must itself be a
    //     wipe-tower layer. The LayerTools entry already exists, but it has neither object nor
    //     support geometry (has_object == false && has_support == false), so the marking pass
    //     above leaves has_wipe_tower == false. Happens e.g. when one object is fully floating
    //     above another and the support_top_z_distance / support_bottom_z_distance gap leaves an
    //     interior layer with no object and no support (e.g. B top z=20.4, A first layer z=20.8,
    //     the z=20.6 LayerTools entry exists but stays unmarked).
    //
    // (2) When two adjacent has_wipe_tower layers are farther apart than max_layer_height and no
    //     LayerTools entry exists between them, insert virtual wipe-tower-only layers to bridge
    //     the gap. Happens with raft: BambuStudio's raft contact layer can be thicker than
    //     max_layer_height (e.g. raft base top z=0.2, raft contact top z=0.5 — gap 0.3 > 0.28),
    //     and there is no LayerTools entry between those two z values.
    //
    // wipe_tower_partitions has already been max-propagated downward above, so partition counts
    // on the filled-in / inserted layers stay consistent.
    // Cadence modes skip this pass (cadence_tower_rows above), so the per-row switch records, the
    // compact tower and Body Split's lagging tower all read Cadence's rows. Mode Off runs upstream's
    // pass unchanged.
    if (! cadence_tower_rows) {
        int first_wt_idx = -1;
        int last_wt_idx  = -1;
        for (int i = 0; i < (int)m_layer_tools.size(); ++i)
            if (m_layer_tools[i].has_wipe_tower) {
                if (first_wt_idx < 0) first_wt_idx = i;
                last_wt_idx = i;
            }
        for (int i = first_wt_idx + 1; i < last_wt_idx; ++i) {
            LayerTools &lt = m_layer_tools[i];
            lt.has_wipe_tower = true;
            // GCode::process_layer emits wipe-tower G-code inside `for (extruder_id : layer_tools.extruders)`.
            // An empty extruders vector here would silently skip wipe tower output, leaving the tower
            // physically floating. Seed from the nearest non-empty neighbor so the loop actually runs.
            if (lt.extruders.empty()) {
                unsigned int seed_extruder = 0;
                bool         found_seed    = false;
                for (int j = i - 1; j >= 0; --j)
                    if (!m_layer_tools[j].extruders.empty()) {
                        seed_extruder = m_layer_tools[j].extruders.back();
                        found_seed = true;
                        break;
                    }
                if (!found_seed)
                    for (int j = i + 1; j < (int)m_layer_tools.size(); ++j)
                        if (!m_layer_tools[j].extruders.empty()) {
                            seed_extruder = m_layer_tools[j].extruders.front();
                            found_seed = true;
                            break;
                        }
                if (found_seed)
                    lt.extruders.push_back(seed_extruder);
            }
        }

        // Walk adjacent has_wipe_tower pairs and split oversized gaps. Re-evaluate the same i
        // after each insertion so very large gaps get split into multiple layers.
        for (int i = 0; i + 1 < (int)m_layer_tools.size(); ) {
            LayerTools &lt      = m_layer_tools[i];
            LayerTools &lt_next = m_layer_tools[i + 1];
            if (!lt.has_wipe_tower || !lt_next.has_wipe_tower) {
                ++i;
                continue;
            }
            coordf_t gap = lt_next.print_z - lt.print_z;
            if (gap <= max_layer_height + EPSILON) {
                ++i;
                continue;
            }
            LayerTools lt_new(0.5 * (lt.print_z + lt_next.print_z));
            lt_new.has_wipe_tower = true;
            if (!lt_next.extruders.empty())
                lt_new.extruders.push_back(lt_next.extruders.front());
            else if (!lt.extruders.empty())
                lt_new.extruders.push_back(lt.extruders.back());
            lt_new.wipe_tower_partitions = lt_next.wipe_tower_partitions;
            m_layer_tools.insert(m_layer_tools.begin() + i + 1, lt_new);
        }
    }

    // An inserted raft row shifts later indexes, so recompute the per-row switch records by the
    // same rule. They are only read in mixed-nozzle modes.
    if (raft_row_inserted) {
        layer_has_own_tool_change.assign(m_layer_tools.size(), false);
        last_tool_change_layer = size_t(-1);
        size_t previous_extruder = size_t(-1);
        for (size_t layer_idx = 0; layer_idx < m_layer_tools.size(); ++layer_idx) {
            const LayerTools &row = m_layer_tools[layer_idx];
            if (row.extruders.empty())
                continue;
            size_t own_changes = row.extruders.size();
            if (previous_extruder == size_t(-1) || previous_extruder == row.extruders.front())
                -- own_changes;
            previous_extruder = row.extruders.back();
            if (own_changes > 0) {
                last_tool_change_layer = layer_idx;
                layer_has_own_tool_change[layer_idx] = true;
            }
        }
    }

    // If the model contains empty layers (such as https://github.com/prusa3d/Slic3r/issues/1266), there might be layers
    // that were not marked as has_wipe_tower, even when they should have been. This produces a crash with soluble supports
    // and maybe other problems. We will therefore go through layer_tools and detect and fix this.
    // So, if there is a non-object layer starting with different extruder than the last one ended with (or containing more than one extruder),
    // we'll mark it with has_wipe tower.
    const size_t gap_repair_limit = is_mixed_nozzle_slicing_enabled(config)
        ? (last_tool_change_layer == size_t(-1) ? 0 : last_tool_change_layer)
        : m_layer_tools.size() - 1;
    for (size_t i = 0; i < gap_repair_limit; ++i) {
        LayerTools& lt = m_layer_tools[i];
        LayerTools& lt_next = m_layer_tools[i+1];
        if (lt.extruders.empty() || lt_next.extruders.empty())
            break;
        if (!lt_next.has_wipe_tower && (lt_next.extruders.front() != lt.extruders.back() || lt_next.extruders.size() > 1))
            lt_next.has_wipe_tower = true;
        // We should also check that the next wipe tower layer is no further than max_layer_height:
        size_t j = i+1;
        double last_wipe_tower_print_z = lt_next.print_z;
        // '<' keeps the m_layer_tools[j+1] read in bounds. The limit row itself is never marked
        // here; it is already marked by the partition propagation above.
        while (++j < gap_repair_limit && !m_layer_tools[j].has_wipe_tower)
            if (m_layer_tools[j+1].print_z - last_wipe_tower_print_z > max_layer_height + EPSILON) {
                if (!config.enable_wrapping_detection)
                    m_layer_tools[j].has_wipe_tower = true;
                last_wipe_tower_print_z = m_layer_tools[j].print_z;
            }
    }
    // The repair above stops at the first row without tools, which a Body Split plate has where one
    // body ends under another (a fine base under a coarse column). Its support is sliced on layers of
    // its own, and a support-only layer that changes nozzle (a fine piece of the support between two
    // coarse bands) has no object, so nothing above gives it a tower visit: the nozzle would change
    // without a purge, and the tower would expect the wrong tool on its next level. Every row up to
    // the last switch that changes tool gets a visit.
    if (is_mixed_nozzle_slicing_enabled(config) && last_tool_change_layer != size_t(-1)) {
        int previous = -1;
        for (size_t i = 0; i <= last_tool_change_layer; ++i) {
            LayerTools &lt = m_layer_tools[i];
            if (lt.extruders.empty())
                continue;
            if (!lt.has_wipe_tower && previous >= 0 && (int(lt.extruders.front()) != previous || lt.extruders.size() > 1))
                lt.has_wipe_tower = true;
            previous = int(lt.extruders.back());
        }
    }

    // Compact tower (mixed-nozzle modes only): decide which layers deposit a tower block
    // (wipe_tower_emit). A layer with its own switch always emits. Any other layer is a filler
    // candidate: defer when the gap to the next switch fits that switch's arriving tool, otherwise
    // emit greedily against the active tool's limit, and always on the layer just before the switch.
    // The very first deposit is never deferred. A lagging tower grows at coarse arrivals, with
    // maintenance visits before long gaps and a solid base laid layer by layer.
    // A Body Split plate whose coarse tool cannot lay the base gets the same lagging tower.
    // Off mode never reads wipe_tower_emit and keeps upstream's ladder.
    const bool body_tower_lags = m_mixed_nozzle_body_coarse_height > 0. &&
                                 Slic3r::mixed_nozzle_body_tower_lags(config, m_mixed_nozzle_base_height);
    const bool compact_tower = is_mixed_nozzle_slicing_enabled(config) &&
                               (mixed_nozzle_compact_tower(config) || body_tower_lags);
    const bool lagging_tower = mixed_nozzle_tower_lagging(config) || body_tower_lags;
    const coordf_t lag_coarse_height = body_tower_lags ? m_mixed_nozzle_body_coarse_height :
                                                         coordf_t(config.mixed_nozzle_coarse_layer_height.value);
    m_mixed_nozzle_body_tower_lags  = body_tower_lags;
    m_mixed_nozzle_lag_coarse_height = lag_coarse_height;
    // On a lagging tower every deposit must stop at least the coarse minimum below the next coarse
    // arrival, or that arrival gets a step it cannot lay. For each layer, the Z of the next one.
    const coordf_t coarse_minimum = lagging_tower ? coordf_t(mixed_nozzle_tower_coarse_minimum(config)) : 0.;
    const int      coarse_filament = lagging_tower ? mixed_nozzle_tower_coarse_filament(config) : -1;
    const std::optional<size_t> coarse_extruder =
        coarse_filament >= 0 ? physical_extruder_for_filament(config, unsigned(coarse_filament)) : std::nullopt;
    const auto on_coarse_nozzle = [&](unsigned int tool) {
        if (coarse_extruder) {
            const std::optional<size_t> extruder = physical_extruder_for_filament(config, tool);
            return extruder && *extruder == *coarse_extruder;
        }
        return int(tool) == coarse_filament;
    };
    std::vector<coordf_t> next_coarse_arrival(m_layer_tools.size(), std::numeric_limits<coordf_t>::max());
    if (coarse_minimum > 0. && coarse_filament >= 0) {
        std::vector<char> coarse_arrival(m_layer_tools.size(), 0);
        unsigned int active = m_layer_tools.front().extruders.empty() ? 0u : m_layer_tools.front().extruders.front();
        for (size_t i = 0; i < m_layer_tools.size(); ++i) {
            const LayerTools &lt = m_layer_tools[i];
            // Any arrival on the coarse nozzle counts, as in WipeTower::plan_lagging_tower.
            if (layer_has_own_tool_change[i]) {
                unsigned int previous = active;
                for (unsigned int tool : lt.extruders) {
                    if (tool != previous && on_coarse_nozzle(tool))
                        coarse_arrival[i] = 1;
                    previous = tool;
                }
            }
            if (! lt.extruders.empty())
                active = lt.extruders.back();
        }
        coordf_t next = std::numeric_limits<coordf_t>::max();
        for (size_t i = m_layer_tools.size(); i-- > 0; ) {
            next_coarse_arrival[i] = next;
            if (coarse_arrival[i])
                next = m_layer_tools[i].print_z;
        }
    }
    // A switchless layer too close under the next coarse arrival lays nothing; the arrival lays
    // the whole step. Never true when the coarse tool can lay one part layer.
    const auto too_close_under_coarse_arrival = [&](size_t i) {
        return coarse_minimum > 0. && next_coarse_arrival[i] < std::numeric_limits<coordf_t>::max() &&
               next_coarse_arrival[i] - m_layer_tools[i].print_z < coarse_minimum - EPSILON;
    };
    coordf_t   lagging_base_top = 0.;
    if (lagging_tower) {
        coordf_t tower_height = 0.;
        for (const LayerTools &lt : m_layer_tools)
            if (lt.has_wipe_tower)
                tower_height = std::max(tower_height, lt.print_z);
        const coordf_t required = tower_height > 0.
            ? coordf_t(mixed_nozzle_tower_base_height(float(tower_height))) : 0.;
        // Whole layers, as WipeTower::update_tower_base_extent takes them, so both agree where
        // the base ends.
        if (required > 0.)
            for (const LayerTools &lt : m_layer_tools) {
                if (! lt.has_wipe_tower)
                    continue;
                lagging_base_top = lt.print_z;
                if (lt.print_z + EPSILON >= required)
                    break;
            }
    }
    if (compact_tower) {
        // For each layer, the index of the next layer at or after it with its own real switch.
        std::vector<size_t> next_switch(m_layer_tools.size(), size_t(-1));
        for (size_t i = m_layer_tools.size(); i-- > 0; ) {
            next_switch[i] = (i + 1 < m_layer_tools.size()) ? next_switch[i + 1] : size_t(-1);
            if (layer_has_own_tool_change[i])
                next_switch[i] = i;
        }

        coordf_t     last_emitted_z = 0.; // "the bed".
        unsigned int active_tool    = 0;
        bool         any_emitted    = false;
        for (size_t i = 0; i < m_layer_tools.size(); ++i) {
            LayerTools &lt = m_layer_tools[i];
            if (! lt.has_wipe_tower)
                continue;
            if (! lt.extruders.empty())
                active_tool = lt.extruders.back();
            // The tower's first deposit is never deferred; it starts with the object's first layer.
            if (lagging_tower && any_emitted && ! layer_has_own_tool_change[i] && too_close_under_coarse_arrival(i)) {
                lt.wipe_tower_emit = false;
                continue;
            }
            if (! any_emitted || layer_has_own_tool_change[i]) {
                lt.wipe_tower_emit = true;
                last_emitted_z     = lt.print_z;
                any_emitted        = true;
                continue;
            }
            if (lagging_tower) {
                // An empty coarse band can delay the next switch, so maintain the tower with the
                // active tool before a gap longer than one coarse layer.
                const size_t next = next_switch[i];
                const bool long_gap = next != size_t(-1) &&
                    m_layer_tools[next].print_z - last_emitted_z >
                        lag_coarse_height + EPSILON;
                lt.wipe_tower_emit = lt.print_z <= lagging_base_top + EPSILON || long_gap;
                if (lt.wipe_tower_emit)
                    last_emitted_z = lt.print_z;
                continue;
            }
            // i is not a switch here, so next_switch[i] is the next switch after i.
            const size_t j = next_switch[i];
            if (j == size_t(-1)) {
                lt.wipe_tower_emit = false; // no more switches ahead; nothing left to bound.
                continue;
            }
            const LayerTools  &lt_switch     = m_layer_tools[j];
            const unsigned int arriving_tool = wipe_tower_arriving_tool(lt_switch, active_tool);
            const coordf_t     max_h_arriving = wipe_tower_tool_max_layer_height(config, arriving_tool);
            if (mixed_nozzle_tower && (!std::isfinite(max_h_arriving) || max_h_arriving <= 0.))
                throw Slic3r::SlicingError(
                    "[SRL-TOWER-STRUCTURE] Cannot schedule a tower gap without a resolved arriving-tool layer-height limit.");
            const coordf_t switch_gap_limit = mixed_nozzle_tower
                ? std::min(structural_max_height, max_h_arriving) : max_h_arriving;
            if (lt_switch.print_z - last_emitted_z <= switch_gap_limit + EPSILON) {
                lt.wipe_tower_emit = false; // deferring all the way to the switch still fits.
                continue;
            }
            const coordf_t max_h_active = mixed_nozzle_tower
                ? structural_max_height : wipe_tower_tool_max_layer_height(config, active_tool);
            if (mixed_nozzle_tower && (!std::isfinite(max_h_active) || max_h_active <= 0.))
                throw Slic3r::SlicingError(
                    "[SRL-TOWER-STRUCTURE] Cannot schedule a tower gap without a resolved active-tool layer-height limit.");
            const coordf_t next_layer_h = (i + 1 < m_layer_tools.size()) ? (m_layer_tools[i + 1].print_z - lt.print_z) : 0.;
            const bool     would_overflow        = (lt.print_z - last_emitted_z) + next_layer_h > max_h_active + EPSILON;
            const bool     last_chance_before_j   = (i + 1 == j);
            if (would_overflow || last_chance_before_j) {
                lt.wipe_tower_emit = true;
                last_emitted_z     = lt.print_z;
            } else {
                lt.wipe_tower_emit = false;
            }
        }
    }

    // Split any mixed-nozzle tower gap taller than the nozzle limit into tower-only levels at equal
    // subdivisions, laid by the tool already active (no extra filament switch). A lagging tower
    // closes its gaps with catch-up layers instead. Requires a prime tower to put them on.
    const bool plate_has_prime_tower = config.enable_prime_tower.value;
    if (plate_has_prime_tower && mixed_nozzle_tower && !lagging_tower && structural_max_height > 0. &&
        std::isfinite(structural_max_height)) {
        struct Gap {
            size_t target_index;
            coordf_t previous_z;
            coordf_t target_z;
            unsigned int active_tool;
        };
        std::vector<Gap> gaps;
        const bool compact = compact_tower;
        const size_t final_target = last_tool_change_layer;
        coordf_t previous_z = 0.;
        unsigned int active_tool = m_layer_tools.empty() || m_layer_tools.front().extruders.empty()
            ? 0u : m_layer_tools.front().extruders.front();
        bool have_target = false;
        for (size_t i = 0; i < m_layer_tools.size(); ++i) {
            const LayerTools &lt = m_layer_tools[i];
            if (lt.has_wipe_tower && (!compact || lt.wipe_tower_emit) &&
                (final_target == size_t(-1) || i <= final_target)) {
                if (!have_target)
                    active_tool = lt.extruders.empty() ? active_tool : lt.extruders.front();
                const coordf_t gap = lt.print_z - previous_z;
                // A gap every tool at the level can lay whole stays whole, like a coarse body's first
                // cell on the bed: halving it would give that nozzle steps under its minimum.
                const auto can_lay = [&config, gap](unsigned int tool) {
                    const std::optional<size_t> nozzle = physical_extruder_for_filament(config, tool);
                    if (! nozzle)
                        return false;
                    const coordf_t minimum = resolved_min_layer_height(config, *nozzle);
                    const coordf_t maximum = resolved_max_layer_height(config, *nozzle);
                    return std::isfinite(minimum) && std::isfinite(maximum) && gap > minimum - EPSILON && gap < maximum + EPSILON;
                };
                const bool laid_whole = ! lt.extruders.empty() && can_lay(active_tool) &&
                                        std::all_of(lt.extruders.begin(), lt.extruders.end(), can_lay);
                if (gap > structural_max_height + EPSILON && ! laid_whole)
                    gaps.push_back({i, previous_z, lt.print_z, active_tool});
                previous_z = lt.print_z;
                have_target = true;
            }
            if (!lt.extruders.empty())
                active_tool = lt.extruders.back();
        }
        for (auto it = gaps.rbegin(); it != gaps.rend(); ++it) {
            const size_t parts = std::max<size_t>(2, size_t(std::ceil((it->target_z - it->previous_z) /
                                                                    structural_max_height)));
            for (size_t part = parts; part-- > 1; ) {
                const coordf_t filler_z = it->previous_z + (it->target_z - it->previous_z) * coordf_t(part) / coordf_t(parts);
                // Promote a native row already at this Z rather than create a duplicate Z row
                // for tools_for_layer() to choose between.
                auto collision = std::find_if(m_layer_tools.begin(), m_layer_tools.end(),
                    [&](const LayerTools &lt) { return std::abs(lt.print_z - filler_z) <= EPSILON; });
                if (collision != m_layer_tools.end()) {
                    collision->has_wipe_tower = true;
                    collision->wipe_tower_emit = true;
                    // An empty Body Split placeholder row needs the active tower tool too.
                    if (!collision->has_object && !collision->has_support && collision->extruders.empty()) {
                        collision->tower_support_event = true;
                        collision->extruders.push_back(it->active_tool);
                        collision->wipe_tower_partitions = std::max<size_t>(1, collision->wipe_tower_partitions);
                    }
                    continue;
                }
                LayerTools filler(filler_z);
                filler.has_wipe_tower = true;
                filler.wipe_tower_emit = true;
                filler.tower_support_event = true;
                filler.extruders.push_back(it->active_tool);
                filler.wipe_tower_partitions = 1;
                const auto position = std::lower_bound(m_layer_tools.begin(), m_layer_tools.end(), filler.print_z,
                    [](const LayerTools& row, coordf_t z) { return row.print_z < z; });
                m_layer_tools.insert(position, std::move(filler));
            }
        }
    }

    // Below the object bottom (raft and support levels) a lagging tower has no catch-up visits,
    // and a raft step can exceed a tool's limit. Subdivide those gaps so both the laying and the
    // arriving tool can lay every level. No effect without a raft or support.
    if (plate_has_prime_tower && mixed_nozzle_tower && lagging_tower && object_bottom_z > EPSILON) {
        struct BaseGap {
            coordf_t     previous_z;
            coordf_t     target_z;
            unsigned int active_tool;
            coordf_t     ceiling;
        };
        std::vector<BaseGap> gaps;
        coordf_t     previous_z  = 0.;
        unsigned int active_tool = m_layer_tools.front().extruders.empty()
            ? 0u : m_layer_tools.front().extruders.front();
        for (const LayerTools &lt : m_layer_tools) {
            if (lt.has_wipe_tower && (!compact_tower || lt.wipe_tower_emit) &&
                lt.print_z < object_bottom_z + EPSILON) {
                // Inserted levels are laid by the active tool, the target by the arriving one:
                // use the smaller ceiling.
                const unsigned int arriving = wipe_tower_arriving_tool(lt, active_tool);
                const coordf_t max_h_arriving = wipe_tower_tool_max_layer_height(config, arriving);
                const coordf_t max_h_active   = wipe_tower_tool_max_layer_height(config, active_tool);
                if (!std::isfinite(max_h_arriving) || max_h_arriving <= 0. ||
                    !std::isfinite(max_h_active) || max_h_active <= 0.)
                    throw Slic3r::SlicingError(
                        "[SRL-TOWER-STRUCTURE] Cannot schedule a tower level under a raft without a resolved layer-height limit for the tools that lay it.");
                const coordf_t ceiling = std::min(max_h_arriving, max_h_active);
                if (lt.print_z - previous_z > ceiling + EPSILON)
                    gaps.push_back({previous_z, lt.print_z, active_tool, ceiling});
                previous_z = lt.print_z;
            }
            if (!lt.extruders.empty())
                active_tool = lt.extruders.back();
        }
        for (auto it = gaps.rbegin(); it != gaps.rend(); ++it) {
            const size_t parts = std::max<size_t>(2, size_t(std::ceil((it->target_z - it->previous_z) / it->ceiling)));
            for (size_t part = parts; part-- > 1; ) {
                const coordf_t filler_z = it->previous_z + (it->target_z - it->previous_z) * coordf_t(part) / coordf_t(parts);
                // Same collision rule as the subdivision above.
                auto collision = std::find_if(m_layer_tools.begin(), m_layer_tools.end(),
                    [&](const LayerTools &lt) { return std::abs(lt.print_z - filler_z) <= EPSILON; });
                if (collision != m_layer_tools.end()) {
                    collision->has_wipe_tower  = true;
                    collision->wipe_tower_emit = true;
                    if (!collision->has_object && !collision->has_support && collision->extruders.empty()) {
                        collision->tower_support_event = true;
                        collision->extruders.push_back(it->active_tool);
                        collision->wipe_tower_partitions = std::max<size_t>(1, collision->wipe_tower_partitions);
                    }
                    continue;
                }
                LayerTools filler(filler_z);
                filler.has_wipe_tower       = true;
                filler.wipe_tower_emit      = true;
                filler.tower_support_event  = true;
                filler.extruders.push_back(it->active_tool);
                filler.wipe_tower_partitions = 1;
                const auto position = std::lower_bound(m_layer_tools.begin(), m_layer_tools.end(), filler.print_z,
                    [](const LayerTools& row, coordf_t z) { return row.print_z < z; });
                m_layer_tools.insert(position, std::move(filler));
            }
        }
    }

    // Calculate the wipe_tower_layer_height values. A compact tower measures from the last
    // emitted layer, so purge, ramming and wall volumes are sized for the real step. Other prints
    // keep the marked-to-marked ladder.
    coordf_t wipe_tower_print_z_last      = 0.;
    coordf_t wipe_tower_print_z_last_real = 0.;
    for (size_t layer_idx = 0; layer_idx < m_layer_tools.size(); ++layer_idx) {
        LayerTools &lt = m_layer_tools[layer_idx];
        if (! lt.has_wipe_tower)
            continue;
        const bool is_emitted = compact_tower && lt.wipe_tower_emit;
        if (is_emitted) {
            lt.wipe_tower_layer_height   = lt.print_z - wipe_tower_print_z_last_real;
            wipe_tower_print_z_last_real = lt.print_z;
        } else {
            lt.wipe_tower_layer_height = lt.print_z - wipe_tower_print_z_last;
        }
        wipe_tower_print_z_last = lt.print_z;
    }
}

void ToolOrdering::collect_extruder_statistics(bool prime_multi_material)
{
    m_first_printing_extruder = (unsigned int)-1;
    for (const auto &lt : m_layer_tools)
        if (! lt.extruders.empty()) {
            m_first_printing_extruder = lt.extruders.front();
            break;
        }

    m_last_printing_extruder = (unsigned int)-1;
    for (auto lt_it = m_layer_tools.rbegin(); lt_it != m_layer_tools.rend(); ++ lt_it)
        if (! lt_it->extruders.empty()) {
            m_last_printing_extruder = lt_it->extruders.back();
            break;
        }

    m_all_printing_extruders.clear();
    for (const auto &lt : m_layer_tools) {
        append(m_all_printing_extruders, lt.extruders);
        sort_remove_duplicates(m_all_printing_extruders);
    }

    m_last_layer_per_extruder.clear();
    for (const LayerTools& lt : m_layer_tools) {
        for (unsigned int ext : lt.extruders) {
            m_last_layer_per_extruder[ext] = lt.print_z;
        }
    }

    if (prime_multi_material && ! m_all_printing_extruders.empty()) {
        // Reorder m_all_printing_extruders in the sequence they will be primed, the last one will be m_first_printing_extruder.
        // Then set m_first_printing_extruder to the 1st extruder primed.
        m_all_printing_extruders.erase(
            std::remove_if(m_all_printing_extruders.begin(), m_all_printing_extruders.end(),
                [ this ](const unsigned int eid) { return eid == m_first_printing_extruder; }),
            m_all_printing_extruders.end());
        m_all_printing_extruders.emplace_back(m_first_printing_extruder);
        m_first_printing_extruder = m_all_printing_extruders.front();
    }
}

void ToolOrdering::cal_most_used_extruder(const PrintConfig &config)
{
    // record
    std::vector<int> extruder_count;
    extruder_count.resize(config.nozzle_diameter.size(), 0);
    for (LayerTools &layer_tools : m_layer_tools) {
        std::vector<unsigned int> filaments = layer_tools.extruders;
        std::set<int> layer_extruder_count;
        //count once only
        for (unsigned int &filament : filaments) {
            layer_extruder_count.insert(config.filament_map.values[filament] - 1);
        }

        //record
        for (int extruder_id : layer_extruder_count) {
            extruder_count[extruder_id]++;
        }
    }

    // set key for most used extruder
    // count most used extruder
    most_used_extruder = 0;
    for (int extruder_id = 1; extruder_id < extruder_count.size(); extruder_id++) {
        if (extruder_count[extruder_id] >= extruder_count[most_used_extruder])
            most_used_extruder = extruder_id;
    }
}

float ToolOrdering::cal_max_additional_fan(const PrintConfig &config)
{
    std::set<unsigned int> used_filaments;
    for (const LayerTools &layer_tools : m_layer_tools)
        used_filaments.insert(layer_tools.extruders.begin(), layer_tools.extruders.end());
    if (used_filaments.empty())
        return 0;

    // Orca: additional_cooling_fan_speed can hold one value per extruder variant a filament prints with;
    // filament_self_index maps such a column to its filament.
    const std::vector<int> &self_index = config.filament_self_index.values;
    const size_t columns = std::max(config.additional_cooling_fan_speed.size(), size_t(*used_filaments.rbegin()) + 1);
    float max_fan = 0;
    for (size_t column = 0; column < columns; ++column) {
        const unsigned int filament_id = self_index.size() == columns ? self_index[column] - 1 : column;
        if (used_filaments.count(filament_id) && max_fan < config.additional_cooling_fan_speed.get_at(column))
            max_fan = config.additional_cooling_fan_speed.get_at(column);
    }
    return max_fan;
}


//BBS: find first non support filament
bool ToolOrdering::cal_non_support_filaments(const PrintConfig &config,
                                                         unsigned int &     first_non_support_filament,
                                                         std::vector<int> & initial_non_support_filaments,
                                                         std::vector<int> & initial_filaments)
{
    int find_count = 0;
    int find_first_filaments_count = 0;
    bool has_non_support = has_non_support_filament(config);
    // The selector can move a filament between extruders per layer; resolve the extruder from
    // the published result then, so the first filament attributed to an extruder is one it
    // actually prints there. Static results keep the cross-layer filament_map arithmetic.
    const bool use_dynamic_map = m_nozzle_group_result.is_support_dynamic_nozzle_map() && m_nozzle_group_result.get_layer_count() > 0;
    auto extruder_for_filament = [&](unsigned int filament, size_t layer_idx) -> int {
        if (use_dynamic_map)
            return m_nozzle_group_result.get_extruder_id(static_cast<int>(filament), static_cast<int>(layer_idx));
        return config.filament_map.values[filament] - 1;
    };
    for (size_t layer_idx = 0; layer_idx < m_layer_tools.size(); ++layer_idx) {
        for (const unsigned int &filament : m_layer_tools[layer_idx].extruders) {
            //check first filament
            if (!config.filament_map.values.empty()) {
                const int extruder_id = extruder_for_filament(filament, layer_idx);
                if (extruder_id >= 0 && extruder_id < static_cast<int>(initial_filaments.size()) && initial_filaments[extruder_id] == -1) {
                    initial_filaments[extruder_id] = filament;
                    find_first_filaments_count++;
                }
            }

            if (has_non_support) {
                // check first non support filaments
                if (config.filament_is_support.get_at(filament))
                    continue;

                if (first_non_support_filament == (unsigned int) -1) first_non_support_filament = filament;

                // params missing, add protection
                // filament map missing means single nozzle, no need to set initial_non_support_filaments
                if (config.filament_map.values.empty())
                    return true;

                const int extruder_id = extruder_for_filament(filament, layer_idx);
                if (extruder_id >= 0 && extruder_id < static_cast<int>(initial_non_support_filaments.size()) && initial_non_support_filaments[extruder_id] == -1) {
                    initial_non_support_filaments[extruder_id] = filament;
                    find_count++;
                }

                if (find_count == initial_non_support_filaments.size())
                    return true;
            } else if (find_first_filaments_count == initial_filaments.size() || config.filament_map.values.empty()){
                    return false;
            }

        }
    }

    return false;
}

bool ToolOrdering::has_non_support_filament(const PrintConfig &config) {
    for (const unsigned int &filament : m_all_printing_extruders) {
        if (!config.filament_is_support.get_at(filament)) {
            return true;
        }
    }

    return false;
}

std::set<std::pair<std::vector<unsigned int>, std::vector<unsigned int>>> generate_combinations(const std::vector<unsigned int> &extruders)
{
    int                                                                       n = extruders.size();
    std::vector<bool>                                                         flags(n);
    std::set<std::pair<std::vector<unsigned int>, std::vector<unsigned int>>> unique_combinations;

    if (extruders.empty())
        return unique_combinations;

    for (int i = 1; i <= n / 2; ++i) {
        std::fill(flags.begin(), flags.begin() + i, true);
        std::fill(flags.begin() + i, flags.end(), false);

        do {
            std::vector<unsigned int> group1, group2;
            for (int j = 0; j < n; ++j) {
                if (flags[j]) {
                    group1.push_back(extruders[j]);
                } else {
                    group2.push_back(extruders[j]);
                }
            }

            if (group1.size() > group2.size()) { std::swap(group1, group2); }

            unique_combinations.insert({group1, group2});

        } while (std::prev_permutation(flags.begin(), flags.end()));
    }

    return unique_combinations;
}

float get_flush_volume(const std::vector<int> &filament_maps, const std::vector<unsigned int> &extruders, const std::vector<FlushMatrix> &matrix, size_t nozzle_nums)
{
    std::vector<std::vector<unsigned int>> nozzle_filaments;
    nozzle_filaments.resize(nozzle_nums);

    for (unsigned int filament_id : extruders) {
        nozzle_filaments[filament_maps[filament_id]].emplace_back(filament_id);
    }

    float flush_volume = 0;
    for (size_t nozzle_id = 0; nozzle_id < nozzle_nums; ++nozzle_id) {
        for (size_t i = 0; i + 1 < nozzle_filaments[nozzle_id].size(); ++i) {
            flush_volume += matrix[nozzle_id][nozzle_filaments[nozzle_id][i]][nozzle_filaments[nozzle_id][i+1]];
        }
    }

    return flush_volume;
}

// Forward declaration — the single-nozzle-per-extruder nozzle list (defined below).
static std::vector<MultiNozzleUtils::NozzleInfo> build_default_nozzle_list(const PrintConfig &print_config, size_t extruder_nums);

// Best-effort readers for the multi-nozzle dev config keys. These are registered in the ConfigDef
// but not (yet) static PrintConfig members, so the slicing PrintConfig reads them as inert defaults,
// which keeps the auto grouping path bit-exact (all these degrade to the flush-only, non-switcher
// case).
static bool cfg_bool(const ConfigBase& c, const char* key, bool def)
{
    if (auto* o = c.option<ConfigOptionBool>(key)) return o->value;
    return def;
}
static double cfg_float(const ConfigBase& c, const char* key, double def)
{
    if (auto* o = c.option<ConfigOptionFloat>(key)) return o->value;
    return def;
}

// The logical filament count. filament_diameter defines it (Preset::normalize() sizes every
// filament option from it); filament_colour is UI-only and may have a single entry.
static size_t filament_domain_size(const PrintConfig &print_config)
{
    return print_config.filament_diameter.values.size();
}

// Prepare per-extruder flush matrices. The prime_volume_mode==pvmFast branch: Default reads
// flush_multiplier, Fast reads flush_multiplier_fast.
static std::vector<FlushMatrix> prepare_flush_matrices(const PrintConfig& print_config)
{
    size_t extruder_nums = print_config.nozzle_diameter.values.size();
    size_t filament_nums = filament_domain_size(print_config);
    std::vector<FlushMatrix> nozzle_flush_mtx;
    for (size_t nozzle_id = 0; nozzle_id < extruder_nums; ++nozzle_id) {
        std::vector<float> flush_matrix(cast<float>(get_flush_volumes_matrix(print_config.flush_volumes_matrix.values, nozzle_id, extruder_nums)));
        // The rows below are sliced unchecked, so refuse a matrix smaller than
        // filament_nums x filament_nums per nozzle (a larger one is fine).
        if (flush_matrix.size() < filament_nums * filament_nums)
            throw Slic3r::RuntimeError(
                _L("The purging volume matrix is too small for the number of filaments. Nozzle ")
                + std::to_string(nozzle_id + 1) + _L(" has ") + std::to_string(flush_matrix.size())
                + _L(" entries, but ") + std::to_string(filament_nums) + _L(" filaments need ")
                + std::to_string(filament_nums * filament_nums)
                + _L(". Check the printer's purging volumes."));
        std::vector<std::vector<float>> wipe_volumes;
        for (unsigned int i = 0; i < filament_nums; ++i)
            wipe_volumes.push_back(std::vector<float>(flush_matrix.begin() + i * filament_nums, flush_matrix.begin() + (i + 1) * filament_nums));
        nozzle_flush_mtx.emplace_back(wipe_volumes);
    }

    // Fast purge mode uses flush_multiplier_fast; Default is inert.
    auto flush_multiplies = (print_config.prime_volume_mode == PrimeVolumeMode::pvmFast) ? print_config.flush_multiplier_fast.values
                                                                                         : print_config.flush_multiplier.values;
    flush_multiplies.resize(extruder_nums, 1);
    for (size_t nozzle_id = 0; nozzle_id < extruder_nums; ++nozzle_id) {
        for (auto& vec : nozzle_flush_mtx[nozzle_id]) {
            for (auto& v : vec)
                v *= flush_multiplies[nozzle_id];
        }
    }
    return nozzle_flush_mtx;
}

// Per-extruder physical nozzle groups.
// Orca: bounds-guards the nozzle_volume_type / extruder_max_nozzle_count arrays, which may be
// shorter than the extruder count on some profiles.
static std::vector<MultiNozzleUtils::NozzleGroupInfo> build_nozzle_groups(const PrintConfig& print_config, size_t extruder_nums)
{
    std::vector<MultiNozzleUtils::NozzleGroupInfo> nozzle_groups;
    auto extruder_nozzle_counts = get_extruder_nozzle_stats(print_config.extruder_nozzle_stats.values);
    const auto& nozzle_volume_types = print_config.nozzle_volume_type.values;
    for (size_t idx = 0; idx < extruder_nums; ++idx) {
        std::string diameter = format_diameter_to_str(print_config.nozzle_diameter.values[idx]);
        NozzleVolumeType vt = idx < nozzle_volume_types.size() ? NozzleVolumeType(nozzle_volume_types[idx]) : nvtStandard;
        int max_count = idx < print_config.extruder_max_nozzle_count.values.size() ? print_config.extruder_max_nozzle_count.values[idx] : 1;
        if (idx >= extruder_nozzle_counts.size() || extruder_nozzle_counts[idx].empty()) {
            nozzle_groups.emplace_back(diameter, vt, (int)idx, max_count);
        } else {
            if (vt == nvtHybrid) {
                for (auto& [volume_type, count] : extruder_nozzle_counts[idx])
                    nozzle_groups.emplace_back(diameter, volume_type, (int)idx, count);
            } else {
                nozzle_groups.emplace_back(diameter, vt, (int)idx, extruder_nozzle_counts[idx][vt]);
            }
        }
    }
    return nozzle_groups;
}

// Build the nozzle-centric FilamentGroupContext.
// Orca deviations, all inert for the shipping fleet:
//   * no print->get_filament_usage_type() → FilamentInfo::usage_type stays ModelOnly (the default);
//   * no print->get_filament_print_time() → speed_info.filament_print_time empty (TimeEvaluator → 0);
//   * the fmmAutoForQuality and Bowden-PA-calibration limit blocks are omitted (Orca has no
//     fmmAutoForQuality mode and no Calib_Params::has_bowden_extruder).
// prefer_non_model_filament (Bowden extruders) is all-false for the Direct-Drive BBL fleet, so the
// support-preference reward path stays dormant.
static FilamentGroupContext build_filament_group_context(
    const Print*                                     print,
    const std::vector<std::vector<unsigned int>>&    layer_filaments,
    const std::vector<std::set<int>>&                physical_unprintables,
    const std::vector<std::set<int>>&                geometric_unprintables,
    const std::map<int, std::set<NozzleVolumeType>>& unprintable_volumes,
    FilamentMapMode                                  mode,
    const std::unordered_map<int, int>&              nozzle_status)
{
    using namespace FilamentGroupUtils;

    FilamentGroupContext context;

    const auto&  print_config  = print->config();
    const size_t filament_nums = filament_domain_size(print_config);
    const size_t extruder_nums = print_config.nozzle_diameter.values.size();
    bool         has_multiple_nozzle = std::any_of(print_config.extruder_max_nozzle_count.values.begin(), print_config.extruder_max_nozzle_count.values.end(),
                                                   [](int v) { return v > 1; });

    auto nozzle_flush_mtx = prepare_flush_matrices(print_config);
    auto nozzle_groups    = build_nozzle_groups(print_config, extruder_nums);

    std::vector<std::set<int>> ext_unprintable_filaments;
    collect_unprintable_limits(physical_unprintables, geometric_unprintables, ext_unprintable_filaments);

    bool ignore_ext_filament = false;
    auto extruder_ams_counts = get_extruder_ams_count(print_config.extruder_ams_count.values);
    std::vector<int> group_size = calc_max_group_size(extruder_ams_counts, ignore_ext_filament);

    // When a filament switcher is connected, disable the AMS capacity limit for grouping.
    const bool has_filament_switcher = cfg_bool(print_config, "has_filament_switcher", false);
    if (has_filament_switcher) {
        int total_filaments = (int)filament_nums;
        for (auto& s : group_size)
            s = std::max(s, total_filaments);
    }

    std::vector<bool> prefer_non_model_filament(extruder_nums, false);
    for (size_t idx = 0; idx < extruder_nums; ++idx)
        if (idx < print_config.extruder_type.values.size())
            prefer_non_model_filament[idx] = (print_config.extruder_type.values[idx] == ExtruderType::etBowden);

    auto machine_filament_info = build_machine_filaments(print->get_extruder_filament_info(), extruder_ams_counts, ignore_ext_filament);

    // The grouping code walks filament_ids and indexes filament_info by the same position.
    std::vector<std::string> filament_ids = print_config.filament_ids.values;
    if (filament_ids.size() > filament_nums)
        filament_ids.resize(filament_nums);

    FGMode fg_mode = mode == FilamentMapMode::fmmAutoForMatch ? FGMode::MatchMode : FGMode::FlushMode;
    context.model_info.flush_matrix          = std::move(nozzle_flush_mtx);
    context.model_info.unprintable_filaments = ext_unprintable_filaments;
    context.model_info.layer_filaments       = layer_filaments;
    context.model_info.filament_ids          = filament_ids;
    context.model_info.unprintable_volumes   = unprintable_volumes;

    // Consumers index filament_info by filament id, so it must span the filament count: a partial
    // or legacy config can leave any of these arrays short, and get_at clamps.
    context.model_info.filament_info.reserve(filament_nums);
    for (size_t idx = 0; idx < filament_nums; ++idx) {
        FilamentGroupUtils::FilamentInfo info;
        info.color      = print_config.filament_colour.get_at(idx);
        info.type       = print_config.filament_type.get_at(idx);
        info.is_support = print_config.filament_is_support.get_at(idx);
        context.model_info.filament_info.emplace_back(std::move(info));
    }

    context.speed_info.group_with_time     = cfg_bool(print_config, "group_algo_with_time", false);
    context.speed_info.filament_change_time = print_config.machine_load_filament_time + print_config.machine_unload_filament_time;
    context.speed_info.extruder_change_time = cfg_float(print_config, "machine_switch_extruder_time", 0.0);
    {
        double load_time   = print_config.machine_load_filament_time;
        double unload_time = print_config.machine_unload_filament_time;
        context.speed_info.change_time_params.standard_load_time   = static_cast<float>(load_time);
        context.speed_info.change_time_params.standard_unload_time = static_cast<float>(unload_time);
        context.speed_info.change_time_params.selector_load_time   = static_cast<float>(load_time / 2);
        context.speed_info.change_time_params.selector_unload_time = static_cast<float>(unload_time / 2);
    }

    context.machine_info.machine_filament_info    = machine_filament_info;
    context.machine_info.max_group_size           = std::move(group_size);
    context.machine_info.master_extruder_id       = print_config.master_extruder_id.value - 1;
    context.machine_info.prefer_non_model_filament = prefer_non_model_filament;

    context.group_info.total_filament_num  = (int)(filament_nums);
    context.group_info.max_gap_threshold   = 0.01;
    context.group_info.strategy            = FGStrategy::BestCost;
    context.group_info.mode                = fg_mode;
    context.group_info.ignore_ext_filament = ignore_ext_filament;
    context.group_info.has_filament_switcher = has_filament_switcher;

    // hybrid flow means no special per-filament nozzle-volume request.
    // Orca: honour the config's per-filament volume map only when it is sized to the filament
    // count. The full-config producers (PresetBundle injection, engine write-back) always size
    // it; a mis-sized map (stale project value, CLI runs until the per-filament synthesis lands
    // there) must not displace the hybrid fallback rebuild_nozzle_unprintables relies on, nor be
    // indexed out of bounds.
    if (mode == FilamentMapMode::fmmManual &&
        print_config.filament_volume_map.values.size() == filament_nums)
        context.group_info.filament_volume_map = print_config.filament_volume_map.values;
    else
        context.group_info.filament_volume_map = std::vector<int>(filament_nums, (int)(NozzleVolumeType::nvtHybrid));

    context.nozzle_info.nozzle_list          = build_nozzle_list(nozzle_groups);
    context.nozzle_info.extruder_nozzle_list = build_extruder_nozzle_list(context.nozzle_info.nozzle_list);

    if (context.nozzle_info.nozzle_list.empty())
        throw Slic3r::RuntimeError(_L("No valid nozzle found. Please check nozzle count."));

    if (!nozzle_status.empty())
        context.nozzle_info.nozzle_status = nozzle_status;

    auto used_filaments = collect_sorted_used_filaments(layer_filaments);

    // add_volume_type_limits: only for single-nozzle-per-extruder machines (H2D and the like). A
    // filament whose forbidden nozzle-volume-type matches an extruder's (only) nozzle becomes
    // unprintable on that extruder; conflicts printable nowhere are dropped.
    if (!has_multiple_nozzle) {
        std::vector<std::set<int>> ext_unprintable_filaments_with_volume = ext_unprintable_filaments;
        for (auto& nozzle : context.nozzle_info.nozzle_list) {
            for (auto fil_id : used_filaments) {
                auto unprintable_vols = context.model_info.unprintable_volumes[fil_id];
                if (unprintable_vols.count(nozzle.volume_type) && nozzle.extruder_id >= 0 && nozzle.extruder_id < (int)ext_unprintable_filaments_with_volume.size())
                    ext_unprintable_filaments_with_volume[nozzle.extruder_id].insert(fil_id);
            }
        }
        for (auto fil_id : used_filaments) {
            if (ext_unprintable_filaments_with_volume[0].count(fil_id) && ext_unprintable_filaments_with_volume[1].count(fil_id)) {
                ext_unprintable_filaments_with_volume[0].erase(fil_id);
                ext_unprintable_filaments_with_volume[1].erase(fil_id);
            }
        }
        context.model_info.unprintable_filaments = ext_unprintable_filaments_with_volume;
    }

    return context;
}

// Orca: the master-extruder preference for flush-mode grouping on two-extruder machines. It keeps a
// light-filament print on the master extruder, which the engine's own in-enum penalty is too weak to
// do against a non-zero right-extruder self-flush term in the flush matrix (that term pulls a lone
// filament onto the non-master extruder). It lives in this slicing wrapper, not in the FilamentGroup
// engine, so the engine and its golden tests stay comparable with BambuStudio's. It only breaks ties
// against the filaments loaded in the printer: get_recommended_filament_maps keeps the engine's
// grouping when that matches them better.
static bool can_swap_extruder_groups(int extruder_id_0, const std::set<int>& group_0, int extruder_id_1, const std::set<int>& group_1, const FilamentGroupContext& ctx)
{
    using namespace FilamentGroupUtils;
    std::vector<std::set<int>> extruder_unprintables(2);
    {
        std::vector<std::set<int>> unprintable_filaments = ctx.model_info.unprintable_filaments;
        if (unprintable_filaments.size() > 1)
            remove_intersection(unprintable_filaments[0], unprintable_filaments[1]);
        std::map<int, std::vector<int>> unplaceable_limits;
        for (int group_id : {extruder_id_0, extruder_id_1})
            if (group_id >= 0 && group_id < (int)unprintable_filaments.size())
                for (auto f : unprintable_filaments[group_id])
                    unplaceable_limits[f].emplace_back(group_id);
        for (auto& elem : unplaceable_limits) sort_remove_duplicates(elem.second);
        for (auto& elem : unplaceable_limits)
            for (auto& eid : elem.second) {
                if (eid == extruder_id_0) extruder_unprintables[0].insert(elem.first);
                if (eid == extruder_id_1) extruder_unprintables[1].insert(elem.first);
            }
    }
    for (auto fid : group_0) if (extruder_unprintables[1].count(fid) > 0) return false;
    for (auto fid : group_1) if (extruder_unprintables[0].count(fid) > 0) return false;
    const auto& mgs = ctx.machine_info.max_group_size;
    if (extruder_id_0 < (int)mgs.size() && extruder_id_1 < (int)mgs.size() &&
        mgs[extruder_id_0] >= (int)group_0.size() && mgs[extruder_id_1] >= (int)group_1.size() &&
        (mgs[extruder_id_0] < (int)group_1.size() || mgs[extruder_id_1] < (int)group_0.size()))
        return false;
    return true;
}

// Balance a filament->nozzle map toward the master extruder (2-extruder machines only). If the
// non-master extruder holds strictly more used filaments than the master and the swap is valid, move
// each group's filaments onto nozzles of the opposite extruder. The exact nozzle within an extruder
// does not affect static g-code (which emits H-1), so re-nozzling round-robin is byte-safe.
static std::vector<int> apply_master_extruder_preference(const FilamentGroupContext& ctx, const std::vector<unsigned int>& used_filaments, std::vector<int> nozzle_ret)
{
    const auto& extruder_nozzle_list = ctx.nozzle_info.extruder_nozzle_list;
    int master = ctx.machine_info.master_extruder_id;
    if (extruder_nozzle_list.size() != 2 || master < 0 || master > 1) return nozzle_ret;
    int other = 1 - master;
    if (extruder_nozzle_list.count(master) == 0 || extruder_nozzle_list.count(other) == 0) return nozzle_ret;
    auto ext_of_nozzle = [&](int nid) -> int {
        return (nid >= 0 && nid < (int)ctx.nozzle_info.nozzle_list.size()) ? ctx.nozzle_info.nozzle_list[nid].extruder_id : -1;
    };
    std::set<int> group_master, group_other;
    for (auto fu : used_filaments) {
        int f = (int)fu;
        if (f >= (int)nozzle_ret.size()) continue;
        int e = ext_of_nozzle(nozzle_ret[f]);
        if (e == master) group_master.insert(f);
        else if (e == other) group_other.insert(f);
    }
    if (group_other.size() > group_master.size() &&
        can_swap_extruder_groups(other, group_other, master, group_master, ctx)) {
        const auto& master_nozzles = extruder_nozzle_list.at(master);
        const auto& other_nozzles  = extruder_nozzle_list.at(other);
        if (!master_nozzles.empty() && !other_nozzles.empty()) {
            int mi = 0, oi = 0;
            for (auto f : group_other)  nozzle_ret[f] = master_nozzles[(mi++) % master_nozzles.size()];
            for (auto f : group_master) nozzle_ret[f] = other_nozzles[(oi++) % other_nozzles.size()];
        }
    }
    return nozzle_ret;
}

// Nozzle-centric grouping. Dispatches by FilamentMapMode and returns a nozzle-aware
// LayeredNozzleGroupResult. For single-extruder printers (X1/P1/A1/H2S) the grouping engine is not
// invoked — the trivial all-master map is wrapped in a single-nozzle result, so their g-code is
// unaffected. Multi-extruder (H2D) grouping runs the nozzle-centric FilamentGroup engine;
// multi-nozzle (H2C/A2L) resolves to a nozzle-granular result.
MultiNozzleUtils::LayeredNozzleGroupResult ToolOrdering::get_recommended_filament_maps(const std::vector<std::vector<unsigned int>>& layer_filaments, const Print* print, const FilamentMapMode mode, const std::vector<std::set<int>>& physical_unprintables, const std::vector<std::set<int>>& geometric_unprintables, const std::map<int, std::set<NozzleVolumeType>>& unprintable_volumes, const std::unordered_map<int, int>& nozzle_status)
{
    using namespace MultiNozzleUtils;

    if (!print || layer_filaments.empty())
        return LayeredNozzleGroupResult();

    const auto&  print_config  = print->config();
    size_t       filament_nums = filament_domain_size(print_config);
    size_t       extruder_nums = print_config.nozzle_diameter.values.size();
    auto         used_filaments = collect_sorted_used_filaments(layer_filaments);
    bool         has_multiple_nozzle = std::any_of(print_config.extruder_max_nozzle_count.values.begin(), print_config.extruder_max_nozzle_count.values.end(),
                                                   [](int v) { return v > 1; });
    bool         has_multiple_extruder = extruder_nums > 1;

    auto nozzle_list = build_default_nozzle_list(print_config, extruder_nums);

    // Manual mode: build directly from the user's filament->extruder map.
    if (mode == FilamentMapMode::fmmManual && !has_multiple_nozzle) {
        auto manual_filament_map = print_config.filament_map.values;
        std::transform(manual_filament_map.begin(), manual_filament_map.end(), manual_filament_map.begin(), [](int v) { return v - 1; });
        auto result = LayeredNozzleGroupResult::create(manual_filament_map, nozzle_list, used_filaments);
        return result ? *result : LayeredNozzleGroupResult();
    }

    // Fully-manual mode: build the nozzle-granular result from the config nozzle map.
    if (mode == FilamentMapMode::fmmNozzleManual) {
        auto manual_filament_map = print_config.filament_map.values;
        std::transform(manual_filament_map.begin(), manual_filament_map.end(), manual_filament_map.begin(), [](int v) { return v - 1; });
        // Each extruder's own diameter: on a mixed pair the rack's hotends are not the other tool's size.
        const std::vector<double> &diameters = print_config.nozzle_diameter.values;
        // Orca: create() indexes the volume/nozzle maps per used filament with no bounds check, so
        // pass them only when a producer sized them to the filament count (mis-sized maps can
        // arrive from stale projects or CLI runs until the per-filament synthesis lands there).
        // Without valid maps the fully-manual request cannot be honoured; return the empty result,
        // the same failure an unsatisfiable create() yields.
        std::optional<LayeredNozzleGroupResult> nozzle_result;
        if (print_config.filament_volume_map.values.size() == filament_nums &&
            print_config.filament_nozzle_map.values.size() == filament_nums)
            nozzle_result = LayeredNozzleGroupResult::create(used_filaments, manual_filament_map, print_config.filament_volume_map.values, print_config.filament_nozzle_map.values, get_extruder_nozzle_stats(print_config.extruder_nozzle_stats.values), diameters);
        if (!nozzle_result)
            BOOST_LOG_TRIVIAL(error) << "Failed to build nozzle group result from filament nozzle map!";
        return nozzle_result ? *nozzle_result : LayeredNozzleGroupResult();
    }

    int              master_extruder_id = print_config.master_extruder_id.value - 1;
    std::vector<int> ret(filament_nums, master_extruder_id);

    // Non-BBL multi-extruder printers do not support filament grouping: filament id == extruder id.
    if (has_multiple_extruder && !print->is_BBL_printer()) {
        for (size_t i = 0; i < filament_nums && i < extruder_nums; i++)
            ret[i] = (int)i;
        auto result_opt = LayeredNozzleGroupResult::create(ret, nozzle_list, used_filaments);
        return result_opt ? *result_opt : LayeredNozzleGroupResult();
    }

    if (has_multiple_extruder || has_multiple_nozzle) {
        auto context = build_filament_group_context(print, layer_filaments, physical_unprintables, geometric_unprintables, unprintable_volumes, mode, nozzle_status);

        // other_layers_seq custom-sequence lambda (1-based layer/extruder). Only threaded for the
        // single-nozzle-per-extruder engine.
        std::vector<LayerPrintSequence> other_layers_seqs = get_other_layers_print_sequence(print_config.other_layers_print_sequence_nums.value, print_config.other_layers_print_sequence.values);
        auto get_custom_seq = [other_layers_seqs](int layer_idx, std::vector<int>& out_seq) -> bool {
            for (size_t idx = other_layers_seqs.size() - 1; idx != size_t(-1); --idx) {
                const auto& other_layers_seq = other_layers_seqs[idx];
                if (layer_idx + 1 >= other_layers_seq.first.first && layer_idx + 1 <= other_layers_seq.first.second) {
                    out_seq = other_layers_seq.second;
                    return true;
                }
            }
            return false;
        };

        if (has_multiple_nozzle && mode == FilamentMapMode::fmmManual) {
            auto manual_filament_map = print_config.filament_map.values;
            std::transform(manual_filament_map.begin(), manual_filament_map.end(), manual_filament_map.begin(), [](int v) { return v - 1; });
            ret = calc_filament_group_for_manual_multi_nozzle(manual_filament_map, context);
        } else if (has_multiple_nozzle && mode == FilamentMapMode::fmmAutoForMatch) {
            ret = calc_filament_group_for_match_multi_nozzle(context);
        } else {
            // TPU: keep the dedicated TPU split for single-nozzle-per-extruder printers.
            auto tpu_filaments = get_filament_by_type(used_filaments, &print_config, "TPU");
            if (!has_multiple_nozzle && !tpu_filaments.empty()) {
                ret = std::vector<int>(context.group_info.total_filament_num, context.machine_info.master_extruder_id);
                for (size_t fidx = 0; fidx < (size_t)context.group_info.total_filament_num; ++fidx)
                    ret[fidx] = tpu_filaments.count((int)fidx) ? context.machine_info.master_extruder_id : (1 - context.machine_info.master_extruder_id);
            } else {
                FilamentGroup fg(context);
                if (!has_multiple_nozzle)
                    fg.get_custom_seq = get_custom_seq;
                ret = fg.calc_filament_group();
                // Flush mode prefers the master extruder; match mode groups by the AMS colours alone.
                // select_best_group_for_ams keeps the first candidate on a tie and when the printer
                // reports no filaments, so the preferred grouping loses only to one that matches the
                // loaded filaments strictly better.
                if (context.group_info.mode == FGMode::FlushMode) {
                    std::vector<int> preferred = apply_master_extruder_preference(context, used_filaments, ret);
                    std::vector<FilamentGroupUtils::FilamentInfo> used_filament_info;
                    for (auto f : used_filaments)
                        used_filament_info.emplace_back(context.model_info.filament_info[f]);
                    // The context's nozzle list, which the nozzle ids in ret index (one entry per
                    // physical nozzle), not the one-per-extruder nozzle_list above.
                    ret = select_best_group_for_ams({preferred, ret}, context.nozzle_info.nozzle_list, used_filaments, used_filament_info,
                                                    context.machine_info.machine_filament_info, context.group_info.has_filament_switcher);
                }
            }
        }

        if (has_multiple_nozzle) {
            auto result_opt = LayeredNozzleGroupResult::create(ret, context.nozzle_info.nozzle_list, used_filaments);
            if (!result_opt)
                return LayeredNozzleGroupResult();
            auto result = *result_opt;
            if (mode == FilamentMapMode::fmmManual) {
                // Manual grouping must reproduce the user's filament->extruder map exactly; a
                // deviation means the requested assignment cannot be satisfied by the nozzle
                // inventory, which must surface as a slicing error instead of silently regrouping.
                auto result_map = result.get_extruder_map();
                for (auto fid : used_filaments) {
                    if (result_map[fid] != print_config.filament_map.values[fid] - 1) {
                        throw Slic3r::RuntimeError(_L("Group error in manual mode. Please check nozzle count or regroup."));
                    }
                }
            }
            return result;
        }
    }

    auto result_opt = LayeredNozzleGroupResult::create(ret, nozzle_list, used_filaments);
    return result_opt ? *result_opt : LayeredNozzleGroupResult();
}

FilamentChangeStats ToolOrdering::get_filament_change_stats(FilamentChangeMode mode)
{
    switch (mode)
    {
    case Slic3r::ToolOrdering::SingleExt:
        return m_stats_by_single_extruder;
    case Slic3r::ToolOrdering::MultiExtBest:
        return m_stats_by_multi_extruder_best;
    case Slic3r::ToolOrdering::MultiExtCurr:
        return m_stats_by_multi_extruder_curr;
    default:
        break;
    }
    return m_stats_by_single_extruder;
}

// Build one logical nozzle per extruder. This is the single-nozzle grouping:
// nozzle group_id == extruder_id. Kept file-local.
static std::vector<MultiNozzleUtils::NozzleInfo> build_default_nozzle_list(const PrintConfig &print_config, size_t extruder_nums)
{
    using namespace MultiNozzleUtils;
    std::vector<NozzleInfo> nozzle_list;
    for (size_t idx = 0; idx < extruder_nums; ++idx) {
        NozzleInfo tmp;
        tmp.diameter    = format_diameter_to_str(print_config.nozzle_diameter.values[idx]);
        tmp.group_id    = static_cast<int>(idx);
        tmp.extruder_id = static_cast<int>(idx);
        // nozzle_volume_type may be shorter than nozzle_diameter on some Orca profiles; default to Standard.
        tmp.volume_type = idx < print_config.nozzle_volume_type.values.size()
                              ? NozzleVolumeType(print_config.nozzle_volume_type.values[idx])
                              : nvtStandard;
        nozzle_list.emplace_back(std::move(tmp));
    }
    return nozzle_list;
}

// Build a LayeredNozzleGroupResult from an already-resolved 0-based
// filament->extruder map. Used by the by-object (sequential) path, whose grouping is decided
// earlier in Print.cpp — here we only wrap the config map. For single-nozzle-per-extruder printers
// (the common case incl. H2D) each filament resolves to its extruder's one logical nozzle
// (nozzle_id == extruder_id). For a multi-nozzle printer the config's per-filament nozzle/volume
// choice is resolved via the 6-argument create (the per-layer engine owns the auto
// sequential path proper).
static MultiNozzleUtils::LayeredNozzleGroupResult build_group_result_from_map(
    const PrintConfig&               print_config,
    const std::vector<int>&          filament_map_0based,
    const std::vector<unsigned int>& used_filaments)
{
    using namespace MultiNozzleUtils;
    const size_t extruder_nums = print_config.nozzle_diameter.values.size();
    const size_t filament_nums = filament_domain_size(print_config);
    const bool   has_multiple_nozzle = std::any_of(print_config.extruder_max_nozzle_count.values.begin(), print_config.extruder_max_nozzle_count.values.end(),
                                                   [](int v) { return v > 1; });
    // Orca: same sizing guard as the manual grouping paths — create() indexes the volume/nozzle
    // maps per used filament with no bounds check, so only maps sized to the filament count are
    // trusted (mis-sized maps can arrive from stale projects or CLI runs until the per-filament
    // synthesis lands there). Unsized maps fall through to the extruder-level wrap below.
    if (has_multiple_nozzle &&
        print_config.filament_volume_map.values.size() == filament_nums &&
        print_config.filament_nozzle_map.values.size() == filament_nums) {
        // Each extruder's own diameter, as in the fully manual path above.
        if (auto g = LayeredNozzleGroupResult::create(used_filaments, filament_map_0based, print_config.filament_volume_map.values, print_config.filament_nozzle_map.values, get_extruder_nozzle_stats(print_config.extruder_nozzle_stats.values), print_config.nozzle_diameter.values))
            return *g;
    }
    auto nozzle_list = build_default_nozzle_list(print_config, extruder_nums);
    if (auto group = LayeredNozzleGroupResult::create(filament_map_0based, nozzle_list, used_filaments))
        return *group;
    return LayeredNozzleGroupResult();
}

// Per-layer nozzle-state refinement. Given a per-range grouping result and the
// physical nozzle occupancy (nozzles_state: nozzle_id -> filament currently loaded), it re-matches
// each logical nozzle to a physical nozzle *within its extruder* by a MinFlushFlowSolver that
// rewards keeping an already-loaded filament (cost -1) and otherwise charges the averaged flush.
// Returns a new result carrying the remapped default filament->nozzle map (falls back to the input
// result if the remap cannot be built). File-local: only the per-layer engine below calls it.
static MultiNozzleUtils::LayeredNozzleGroupResult refine_groups_by_Nozzle_State(
    const FilamentGroupContext&                       ctx,
    const MultiNozzleUtils::LayeredNozzleGroupResult& group,
    const std::unordered_map<int, int>&               nozzles_state)
{
    std::vector<std::vector<int>> nozzle_fils(ctx.nozzle_info.nozzle_list.size());
    auto fils        = group.get_used_filaments(0);
    auto fil_noz_map = group.get_layer_filament_nozzle_map(0);

    for (auto fil : fils)
        nozzle_fils[fil_noz_map[fil]].emplace_back(fil);

    // 1. Collect the nozzles each filament may NOT use.
    std::map<int, std::set<int>> fil_unplaceable_nozs;
    for (auto fil : fils) {
        std::set<NozzleVolumeType> unprintable_volumes;
        if (ctx.model_info.unprintable_volumes.count(fil))
            unprintable_volumes = ctx.model_info.unprintable_volumes.at(fil);
        auto expected_volume = ctx.group_info.filament_volume_map[fil];

        for (int noz = 0; noz < (int) ctx.nozzle_info.nozzle_list.size(); noz++) {
            auto noz_info             = ctx.nozzle_info.nozzle_list[noz];
            int  ext_id               = noz_info.extruder_id;
            auto ext_unprintable_fils = ctx.model_info.unprintable_filaments[ext_id];
            if (ext_unprintable_fils.count(fil) > 0 ||
                (expected_volume != nvtHybrid && expected_volume != noz_info.volume_type) || (unprintable_volumes.count(noz_info.volume_type) != 0))
                fil_unplaceable_nozs[fil].insert(noz);
        }
    }

    // 2. Global nozzle-match result.
    std::unordered_map<int, int> global_uv_match;

    // 3. Solve one min-cost flow per extruder.
    for (const auto& [ext_id, ext_nozzles] : ctx.nozzle_info.extruder_nozzle_list) {
        if (ext_nozzles.empty()) continue;

        // 3.1. u_nodes / v_nodes for this extruder.
        std::vector<int> u_nodes = ext_nozzles;
        std::vector<int> v_nodes = ext_nozzles;

        // 3.2. global nozzle id -> local index.
        std::unordered_map<int, int> global_to_local;
        for (size_t i = 0; i < ext_nozzles.size(); ++i)
            global_to_local[ext_nozzles[i]] = static_cast<int>(i);

        // 3.3. cost matrix for this extruder.
        std::vector<std::vector<float>>           cost_matrix(u_nodes.size(), std::vector<float>(v_nodes.size(), std::numeric_limits<float>::max()));
        std::unordered_map<int, std::vector<int>> uv_unlink_limits;

        for (size_t local_u = 0; local_u < u_nodes.size(); ++local_u) {
            int           u_node = u_nodes[local_u];
            std::set<int> unlink_v_local;
            auto          u_fils = nozzle_fils[u_node];

            // Collect the v_nodes this u_node may NOT connect to (as local indices).
            for (auto fil : u_fils) {
                for (auto unplaceable_noz : fil_unplaceable_nozs[fil]) {
                    if (global_to_local.count(unplaceable_noz))
                        unlink_v_local.insert(global_to_local[unplaceable_noz]);
                }
            }
            uv_unlink_limits[static_cast<int>(local_u)].assign(unlink_v_local.begin(), unlink_v_local.end());

            // 3.4. compute costs.
            for (size_t local_v = 0; local_v < v_nodes.size(); ++local_v) {
                int   v_node = v_nodes[local_v];
                float cost   = 0;
                if (unlink_v_local.count(static_cast<int>(local_v))) continue;

                std::optional<unsigned int> v_fil_opt = std::nullopt;
                if (nozzles_state.count(v_node))
                    v_fil_opt = nozzles_state.at(v_node);

                if (!v_fil_opt.has_value() || v_fil_opt.value() >= ctx.model_info.filament_info.size()) {
                    cost = 0;
                } else {
                    int v_fil = v_fil_opt.value();
                    if (std::find(u_fils.begin(), u_fils.end(), v_fil) != u_fils.end())
                        cost = -1;
                    else {
                        for (auto u_fil : u_fils)
                            cost += ctx.model_info.flush_matrix[ext_id][u_fil][v_fil];
                        if (u_fils.size() > 0)
                            cost /= u_fils.size();
                    }
                }

                cost_matrix[local_u][local_v] = cost;
            }
        }

        // 3.5. min-cost flow -> nozzle match for this extruder.
        std::vector<int> local_u_nodes(u_nodes.size());
        std::vector<int> local_v_nodes(v_nodes.size());
        std::iota(local_u_nodes.begin(), local_u_nodes.end(), 0);
        std::iota(local_v_nodes.begin(), local_v_nodes.end(), 0);

        MinFlushFlowSolver solver(cost_matrix, local_u_nodes, local_v_nodes, {}, uv_unlink_limits);
        auto               local_match = solver.solve();

        // 3.6. local match -> global match.
        for (size_t local_u = 0; local_u < u_nodes.size(); ++local_u) {
            int global_u = u_nodes[local_u];
            int local_v  = local_match[static_cast<int>(local_u)];
            if (local_v == MaxFlowGraph::INVALID_ID || local_v < 0 || local_v >= static_cast<int>(v_nodes.size()))
                continue;
            int global_v = v_nodes[local_v];
            global_uv_match[global_u] = global_v;
        }
    }

    // 4. Build the new group_result.
    std::vector<int> new_default_filament_nozzle_maps = group.get_layer_filament_nozzle_map(-1);

    for (auto fil : fils) {
        int ori_noz = new_default_filament_nozzle_maps[fil];
        if (global_uv_match.count(ori_noz))
            new_default_filament_nozzle_maps[fil] = global_uv_match[ori_noz];
    }

    auto new_group = MultiNozzleUtils::LayeredNozzleGroupResult::create(new_default_filament_nozzle_maps, ctx.nozzle_info.nozzle_list, fils);
    if (!new_group.has_value()) new_group = group;

    return *new_group;
}

// Used as an unordered_map key over a filament-set (the per-layer filament combo).
struct VectorHash
{
    size_t operator()(const std::vector<unsigned int>& v) const
    {
        size_t seed = v.size();
        for (auto& elem : v)
            seed ^= std::hash<unsigned int>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

// The per-layer regroup engine. Layers are grouped into
// contiguous runs sharing the same filament set ("combo ranges"); a NozzleStatusRecorder carries the
// physical nozzle occupancy across ranges so the selector rewards keeping an already-loaded filament.
// Per range: get_recommended_filament_maps -> refine_groups_by_Nozzle_State (nozzle re-match) ->
// reorder_filaments_for_multi_nozzle_extruder (in-range ordering). Emits a per-layer
// filament->nozzle match + filament order. Orca: there is no ToolOrdering::OrderingContext here, so
// the custom-sequence function is passed directly instead. Only the dynamic branch
// (H2C selector, is_dynamic_group_reorder) calls this.
static std::vector<FilamentPlanRes> plan_filament_mapping_and_order_by_combo_ranges(
    Print*                                            print,
    const FilamentGroupContext&                       ctx,
    const std::function<bool(int, std::vector<int>&)> get_custom_seq,
    const FilamentMapMode                             mode,
    const std::vector<std::set<int>>&                 physical_unprintables,
    const std::vector<std::set<int>>&                 geometric_unprintables,
    const std::map<int, std::set<NozzleVolumeType>>&  unprintable_volumes,
    MultiNozzleUtils::NozzleStatusRecorder*           io_nozzle_status)
{
    std::vector<FilamentPlanRes> results;

    const auto& layer_fils = ctx.model_info.layer_filaments;
    if (layer_fils.empty())
        return results;

    results.resize(layer_fils.size());

    // key: the sorted+deduped filament set used by a layer; value: the contiguous [start,end] runs.
    std::unordered_map<std::vector<unsigned int>, std::vector<std::pair<int, int>>, VectorHash> filament_combo_ranges;
    for (int layer_idx = 0; layer_idx < static_cast<int>(layer_fils.size()); ++layer_idx) {
        std::vector<unsigned int> cur_combo = layer_fils[layer_idx];
        std::sort(cur_combo.begin(), cur_combo.end());
        cur_combo.erase(std::unique(cur_combo.begin(), cur_combo.end()), cur_combo.end());
        if (cur_combo.empty())
            continue;

        auto& ranges = filament_combo_ranges[cur_combo];
        if (ranges.empty() || ranges.back().second != layer_idx - 1)
            ranges.emplace_back(layer_idx, layer_idx);
        else
            ranges.back().second = layer_idx;
    }

    std::map<std::pair<int, int>, std::vector<unsigned int>> range_filas_map;
    for (auto& [combo, ranges] : filament_combo_ranges)
        for (auto& range : ranges)
            range_filas_map[range] = combo;

    std::set<int> used_filaments;

    // Per combo range: build the range's layer_filaments, group + refine + reorder.
    MultiNozzleUtils::NozzleStatusRecorder tool_status;
    if (io_nozzle_status) tool_status = *io_nozzle_status;

    std::vector<int>             fil_noz_map(ctx.group_info.total_filament_num, -1); // global filament -> nozzle map
    std::unordered_map<int, int> fil_first_nozzle_map;                              // filament -> first nozzle it used
    for (auto& [range, combo] : range_filas_map) {
        auto [start_layer, end_layer] = range;
        // 1. layer_filaments for this range.
        std::vector<std::vector<unsigned int>> range_layer_fils;
        range_layer_fils.reserve(end_layer - start_layer + 1);
        for (int layer_idx = start_layer; layer_idx <= end_layer; ++layer_idx)
            range_layer_fils.push_back(layer_fils[layer_idx]);
        used_filaments.insert(combo.begin(), combo.end());

        // 2. group the range.
        auto nozzle_filament_map = tool_status.get_nozzle_filament_map();
        auto group_result        = ToolOrdering::get_recommended_filament_maps(range_layer_fils, print, mode, physical_unprintables, geometric_unprintables, unprintable_volumes, nozzle_filament_map);

        // 3. re-match logical nozzles to physical nozzles by the current nozzle state.
        auto new_group_result = refine_groups_by_Nozzle_State(ctx, group_result, nozzle_filament_map);

        auto range_seq_function = [&get_custom_seq, start_layer_ = start_layer, end_layer_ = end_layer](int layer_idx, std::vector<int>& out_seq) -> bool {
            if (layer_idx <= end_layer_ - start_layer_) {
                int global_idx = start_layer_ + layer_idx;
                return get_custom_seq ? get_custom_seq(global_idx, out_seq) : false;
            }
            return false;
        };

        // 4. order the filaments within the range.
        std::vector<std::vector<unsigned int>> fils_sequences;
        reorder_filaments_for_multi_nozzle_extruder(range_layer_fils.front(), new_group_result, range_layer_fils, ctx.model_info.flush_matrix, range_seq_function,
                                                    &fils_sequences, tool_status);

        // 5. store the range result + advance the nozzle state.
        for (auto fil_id : fils_sequences.back()) {
            auto noz = new_group_result.get_nozzle_for_filament(fil_id);
            if (noz.has_value()) {
                int noz_id = noz->group_id;
                int ext_id = noz->extruder_id;

                fil_noz_map[fil_id] = noz_id;
                fil_first_nozzle_map.emplace(static_cast<int>(fil_id), noz_id);

                tool_status.set_current_extruder_id(ext_id);
                tool_status.set_nozzle_status(noz_id, fil_id, ext_id);
            }
        }

        assert(fils_sequences.size() == range_layer_fils.size());
        for (size_t layer_id = 0; layer_id < fils_sequences.size(); ++layer_id) {
            int g_layer_id                       = start_layer + static_cast<int>(layer_id);
            results[g_layer_id].fil_nozzle_match = fil_noz_map;
            results[g_layer_id].fil_order        = std::vector<int>(fils_sequences[layer_id].begin(), fils_sequences[layer_id].end());
        }
    }

    // Fill any never-assigned slot with the filament's first nozzle (or nozzle 0).
    for (auto& res : results) {
        for (int fil_id = 0; fil_id < (int) res.fil_nozzle_match.size(); fil_id++) {
            auto& noz_id = res.fil_nozzle_match[fil_id];
            if (noz_id == -1)
                noz_id = (used_filaments.count(fil_id) && fil_first_nozzle_map.count(fil_id)) ? fil_first_nozzle_map[fil_id] : 0;
        }
    }

    if (io_nozzle_status) *io_nozzle_status = tool_status;

    return results;
}

MultiNozzleUtils::LayeredNozzleGroupResult ToolOrdering::build_sequential_group_result(
    Print*                                            print,
    std::vector<std::vector<int>>                     nozzle_map_per_layer,
    const std::vector<std::vector<unsigned int>>&     layer_filaments,
    const std::vector<std::vector<unsigned int>>&     layer_sequences,
    const std::vector<unsigned int>&                  used_filaments,
    const std::vector<std::set<int>>&                 physical_unprintables,
    const std::vector<std::set<int>>&                 geometric_unprintables,
    const std::map<int, std::set<NozzleVolumeType>>&  unprintable_volumes)
{
    MultiNozzleUtils::normalize_nozzle_map_per_layer(nozzle_map_per_layer, layer_filaments);
    auto context = build_filament_group_context(print, layer_filaments, physical_unprintables, geometric_unprintables,
                                                unprintable_volumes, FilamentMapMode::fmmAutoForFlush, {});
    auto result = MultiNozzleUtils::LayeredNozzleGroupResult::create(nozzle_map_per_layer, context.nozzle_info.nozzle_list,
                                                                     used_filaments, layer_sequences);
    return result ? *result : MultiNozzleUtils::LayeredNozzleGroupResult();
}

static double snap_to_simple_fraction(double r, int max_denom = 10)
{
    double best_r = r;
    double best_err = 1.0;
    for (int q = 1; q <= max_denom; ++q) {
        int p = (int)std::round(r * q);
        if (p < 0) p = 0;
        if (p > q) p = q;
        double candidate = (double)p / q;
        double err = std::abs(candidate - r);
        if (err < best_err) {
            best_err = err;
            best_r = candidate;
        }
    }
    return best_r;
}

void ToolOrdering::resolve_mixed_filaments(const PrintConfig &config)
{
    const auto &is_mixed = config.filament_is_mixed.values;
    const auto &comp_strs = config.filament_mixed_components.values;
    const auto &ratio_strs = config.filament_mixed_sublayer_ratios.values;

    // Capture mixed slots that actually appear on layers before they are expanded to
    // physical components. Assigned-but-unused mixed slots never enter layer_tools.
    m_used_mixed_filaments.clear();
    if (has_any_mixed_filament(is_mixed)) {
        std::set<unsigned int> used;
        for (const LayerTools &lt : m_layer_tools)
            for (unsigned int ext : lt.extruders)
                if (ext < is_mixed.size() && is_mixed[ext])
                    used.insert(ext);
        m_used_mixed_filaments.assign(used.begin(), used.end());
    }

    if (!has_any_mixed_filament(is_mixed))
        return;

    const bool sublayer_enabled = config.enable_mixed_color_sublayer.value;

    struct SlotInfo {
        std::vector<unsigned int> components; // 1-based
        std::vector<double>       ratios;
        std::vector<long long>    accum;     // deficit accumulator (integer, unit: 1e-6 mm)
    };
    std::vector<SlotInfo> slots(is_mixed.size());
    for (size_t i = 0; i < is_mixed.size(); ++i) {
        if (!is_mixed[i])
            continue;
        slots[i].components = parse_mixed_components(i < comp_strs.size() ? comp_strs[i] : "");
        if (slots[i].components.size() < 2) {
            slots[i].components.clear();
            continue;
        }
        for (unsigned int cid : slots[i].components) {
            unsigned int idx0 = cid - 1;
            if (idx0 >= is_mixed.size() || (idx0 < is_mixed.size() && is_mixed[idx0])) {
                slots[i].components.clear();
                break;
            }
        }
        if (slots[i].components.empty())
            continue;
        slots[i].ratios = parse_mixed_ratios(
            i < ratio_strs.size() ? ratio_strs[i] : "", slots[i].components.size());
        if (!sublayer_enabled) {
            for (double &r : slots[i].ratios)
                r = snap_to_simple_fraction(r);
            double sum = 0;
            for (double r : slots[i].ratios) sum += r;
            if (sum > 0)
                for (double &r : slots[i].ratios) r /= sum;
        }
        slots[i].accum.assign(slots[i].components.size(), 0LL);
    }

    // Parse gradient settings per slot
    const auto &gradient_flags = config.filament_mixed_gradient.values;
    const auto &gradient_range_strs = config.filament_mixed_gradient_range.values;
    const auto &gradient_curve_strs = config.filament_mixed_gradient_curve.values;
    struct GradientInfo {
        double        start = 0.10;
        double        end_val = 0.90;
        GradientCurve curve;  // empty -> use linear (start, end_val); non-empty wins
    };
    std::vector<bool> is_gradient(is_mixed.size(), false);
    std::vector<GradientInfo> gradient_info(is_mixed.size());
    for (size_t i = 0; i < is_mixed.size(); ++i) {
        if (!is_mixed[i] || slots[i].components.size() != 2)
            continue;
        if (i >= gradient_flags.size() || !gradient_flags[i])
            continue;
        is_gradient[i] = true;
        if (i < gradient_range_strs.size() && !gradient_range_strs[i].empty()) {
            CNumericLocalesSetter c_locale_setter;
            float v0 = 0, v1 = 0;
            if (std::sscanf(gradient_range_strs[i].c_str(), "%f,%f", &v0, &v1) == 2 &&
                v0 > 0 && v0 < 1.0 && v1 > 0 && v1 < 1.0) {
                gradient_info[i].start   = v0;
                gradient_info[i].end_val = v1;
            }
        }
        if (i < gradient_curve_strs.size() && !gradient_curve_strs[i].empty())
            gradient_info[i].curve = parse_gradient_curve(gradient_curve_strs[i]);
    }

    // Pass 1: identify continuous runs for each gradient slot (Per-Run).
    // A "run" is a maximal sequence of consecutive layers where the slot appears.
    struct GradientRunInfo {
        std::vector<size_t> run_lengths;
        int                 current_run = -1;
        size_t              current_idx = 0;
        bool                prev_appeared = false;
        bool                last_absent_was_relevant = false;
    };
    std::map<unsigned int, GradientRunInfo> gradient_runs;
    for (size_t i = 0; i < is_mixed.size(); ++i)
        if (is_gradient[i]) gradient_runs[static_cast<unsigned int>(i)] = {};

    // Build per-slot sets of all layer indices where any slot-owning object has a
    // layer.  Used by gradient run detection (a gap is real only if the slot is
    // absent at a layer belonging to one of its own objects) and by calc_slot_lh
    // to keep prev_relevant_z_for_slot current even when a slot skips many layers.
    std::map<unsigned int, std::set<size_t>> slot_relevant_layers;
    for (auto &[slot_idx, obj_map] : m_mixed_object_layers) {
        for (auto &[obj, _] : obj_map) {
            auto it = m_object_all_layer_indices.find(obj);
            if (it != m_object_all_layer_indices.end())
                slot_relevant_layers[slot_idx].insert(it->second.begin(), it->second.end());
        }
    }

    if (!gradient_runs.empty()) {
        for (size_t li = 0; li < m_layer_tools.size(); ++li) {
            if (li == 0) continue;
            const auto &lt = m_layer_tools[li];
            for (auto &[slot, run] : gradient_runs) {
                bool here = std::find(lt.extruders.begin(), lt.extruders.end(), slot) != lt.extruders.end();
                if (here) {
                    bool real_gap = false;
                    if (!run.prev_appeared && !run.run_lengths.empty()) {
                        real_gap = run.last_absent_was_relevant;
                    }
                    if (run.run_lengths.empty() || real_gap)
                        run.run_lengths.push_back(0);
                    run.run_lengths.back()++;
                    run.last_absent_was_relevant = false;
                } else if (!run.run_lengths.empty()) {
                    auto rel_it = slot_relevant_layers.find(slot);
                    if (rel_it != slot_relevant_layers.end() && rel_it->second.count(li))
                        run.last_absent_was_relevant = true;
                }
                run.prev_appeared = here;
            }
        }
        for (auto &[slot, run] : gradient_runs) {
            run.current_run = -1;
            run.current_idx = 0;
            run.prev_appeared = false;
            run.last_absent_was_relevant = false;
        }
    }

    // Per-object gradient: pre-compute per-object runs (respecting Z gaps within each object).
    struct PerObjRunState {
        std::vector<size_t> run_start_offsets; // index into layer_indices where each run starts
        std::vector<size_t> run_lengths;
        int                 current_run = -1;
        size_t              current_idx = 0;
    };

    // Detect whether a gap between two consecutive gradient-slot appearances is a
    // real run break.  A gap is real only if the object has its own layer inside the
    // gap that does NOT use the gradient slot (i.e. the slot was genuinely absent).
    // Uses lower_bound to skip global indices that don't belong to the object.
    auto has_real_gap = [](size_t prev_idx, size_t cur_idx,
                           const std::set<size_t>& obj_set,
                           const std::set<size_t>& slot_set) -> bool {
        for (auto it = obj_set.lower_bound(prev_idx + 1);
             it != obj_set.end() && *it < cur_idx; ++it) {
            if (!slot_set.count(*it))
                return true;
        }
        return false;
    };

    // Segment a sorted list of layer indices into runs, using has_real_gap to decide
    // where to break.  Shared by the per-object and per-volume paths below.
    auto segment_runs = [&](const std::vector<size_t>& layer_indices,
                            const std::set<size_t>& obj_set,
                            const std::set<size_t>& slot_set) -> PerObjRunState {
        PerObjRunState st;
        for (size_t i = 0; i < layer_indices.size(); ++i) {
            bool new_run = (i == 0) ||
                           has_real_gap(layer_indices[i - 1], layer_indices[i], obj_set, slot_set);
            if (new_run) {
                st.run_start_offsets.push_back(i);
                st.run_lengths.push_back(0);
            }
            st.run_lengths.back()++;
        }
        return st;
    };

    std::map<unsigned int, std::map<const PrintObject*, PerObjRunState>> per_obj_runs;
    for (auto &[slot, obj_map] : m_mixed_object_layers) {
        if (slot >= is_gradient.size() || !is_gradient[slot])
            continue;
        for (auto &[obj, layer_indices] : obj_map) {
            sort_remove_duplicates(layer_indices);
            // Erase layer 0 — this mutation is also relied upon by the Pass 2 binary_search below.
            if (!layer_indices.empty() && layer_indices.front() == 0)
                layer_indices.erase(layer_indices.begin());

            const auto &all_obj_layers = m_object_all_layer_indices[obj];
            std::set<size_t> all_obj_set(all_obj_layers.begin(), all_obj_layers.end());
            std::set<size_t> grad_set(layer_indices.begin(), layer_indices.end());

            per_obj_runs[slot][obj] = segment_runs(layer_indices, all_obj_set, grad_set);
        }
    }

    // Per-volume gradient: mirror the per-object run-segmentation logic above for
    // m_gradient_volume_layers. When per_part_gradient is off (or no qualifying volume exists),
    // m_gradient_volume_layers is empty and per_vol_runs ends up empty too — so all subsequent
    // checks of `per_vol_runs.find(slot) != end()` will fail and the legacy per-object path
    // remains the only path taken.
    using VolumeKey = LayerTools::MixedSubLayerGroup::VolumeKey;
    std::map<unsigned int, std::map<VolumeKey, PerObjRunState>> per_vol_runs;
    for (auto &[slot, vol_map] : m_gradient_volume_layers) {
        if (slot >= is_gradient.size() || !is_gradient[slot])
            continue;
        for (auto &[vkey, layer_indices] : vol_map) {
            sort_remove_duplicates(layer_indices);
            if (!layer_indices.empty() && layer_indices.front() == 0)
                layer_indices.erase(layer_indices.begin());

            const auto &all_obj_layers = m_object_all_layer_indices[vkey.obj];
            std::set<size_t> all_obj_set(all_obj_layers.begin(), all_obj_layers.end());
            std::set<size_t> vol_grad_set(layer_indices.begin(), layer_indices.end());

            per_vol_runs[slot][vkey] = segment_runs(layer_indices, all_obj_set, vol_grad_set);
        }
    }
    // Pass 2: resolve per layer
    coordf_t prev_print_z = 0.;
    // Track last print_z per mixed slot so that layer height is computed from the
    // slot's own previous appearance, not from a global Z that may include layers
    // belonging only to other objects with different layer heights.
    std::map<unsigned int, coordf_t> prev_print_z_for_slot;
    // Track the last Z where a slot-owning object had ANY layer (regardless of
    // whether the slot was present). Used to detect genuine gaps: if the slot was
    // absent but its owner objects had layers, prev_relevant_z advances while
    // prev_print_z_for_slot stays stale. Taking the max of both gives correct lh.
    std::map<unsigned int, coordf_t> prev_relevant_z_for_slot;

    // Compute the effective layer height for a mixed slot by choosing the best
    // reference Z among: (1) the slot's own last Z, (2) the last Z where the
    // slot's owning object had any layer, (3) the global previous Z as fallback
    // when the slot appears for the first time.
    auto calc_slot_lh = [&](unsigned int ext, coordf_t print_z) -> double {
        auto slot_pz_it = prev_print_z_for_slot.find(ext);
        auto rel_pz_it  = prev_relevant_z_for_slot.find(ext);
        coordf_t base_z = prev_print_z;
        if (slot_pz_it != prev_print_z_for_slot.end()) {
            base_z = slot_pz_it->second;
            if (rel_pz_it != prev_relevant_z_for_slot.end())
                base_z = std::max(base_z, rel_pz_it->second);
        }
        double lh = print_z - base_z;
        return (lh > 0.) ? lh : 0.2;  // 0.2mm safety fallback; should not trigger in normal operation
    };

    for (LayerTools &lt : m_layer_tools) {
        size_t layer_idx = static_cast<size_t>(&lt - m_layer_tools.data());

        // Update gradient run state (skip first layer to match counting).
        if (layer_idx > 0) {
            for (auto &[slot, run] : gradient_runs) {
                bool here = std::find(lt.extruders.begin(), lt.extruders.end(), slot) != lt.extruders.end();
                if (here) {
                    if (!run.prev_appeared) {
                        if (run.last_absent_was_relevant || run.current_run < 0) {
                            run.current_run++;
                            run.current_idx = 0;
                        }
                    }
                    run.last_absent_was_relevant = false;
                } else {
                    auto rel_it = slot_relevant_layers.find(slot);
                    if (rel_it != slot_relevant_layers.end() && rel_it->second.count(layer_idx))
                        run.last_absent_was_relevant = true;
                }
                run.prev_appeared = here;
            }
        }

        std::vector<unsigned int> new_extruders;
        for (unsigned int ext : lt.extruders) {
            if (ext >= slots.size() || slots[ext].components.empty()) {
                new_extruders.push_back(ext);
                continue;
            }
            auto &s = slots[ext];

            // Skip sublayer splitting for the first layer to preserve bed adhesion.
            if (sublayer_enabled && layer_idx > 0) {
                double lh = calc_slot_lh(ext, lt.print_z);
                size_t n = s.components.size();

                std::vector<double> sub_heights;
                bool gradient_last_no_split = false;
                unsigned int gradient_last_dominant_0b = 0;
                if (is_gradient[ext] && n == 2) {
                    auto gr_it = gradient_runs.find(ext);
                    if (gr_it != gradient_runs.end() && gr_it->second.current_run >= 0 &&
                        static_cast<size_t>(gr_it->second.current_run) < gr_it->second.run_lengths.size()) {
                        auto &run  = gr_it->second;
                        size_t N   = run.run_lengths[run.current_run];
                        size_t idx = run.current_idx++;
                        double t   = (N > 0) ? (2.0 * idx + 1.0) / (2.0 * N) : 0.5;
                        // Custom curve wins over linear range when present; OFF path stays bit-identical.
                        double r1  = gradient_info[ext].curve.empty()
                                     ? (gradient_info[ext].start + (gradient_info[ext].end_val - gradient_info[ext].start) * t)
                                     : sample_gradient_curve(gradient_info[ext].curve, t);
                        double r2  = 1.0 - r1;
                        sub_heights.push_back(r1 * lh);
                        sub_heights.push_back(r2 * lh);
                        // The sublayer split path sorts components by physical ID ascending;
                        // the higher-ID component ends up on top (visible surface). If the
                        // gradient's dominant component has the lower physical ID, splitting
                        // would put the non-dominant color on the visible top surface. In
                        // that case, skip the split and print this final run-layer as pure
                        // dominant color to preserve the gradient appearance.
                        if (idx == N - 1) {
                            // When r1 == r2 (exactly 50/50), component[0] is treated as dominant.
                            size_t dominant = (r1 >= r2) ? 0 : 1;
                            unsigned int dom_0b = s.components[dominant] - 1;
                            unsigned int oth_0b = s.components[1 - dominant] - 1;
                            if (dom_0b < oth_0b) {
                                gradient_last_no_split = true;
                                gradient_last_dominant_0b = dom_0b;
                            }
                        }
                    } else {
                        for (double r : s.ratios)
                            sub_heights.push_back(r * lh);
                    }
                } else {
                    for (double r : s.ratios)
                        sub_heights.push_back(r * lh);
                }

                // Per-part gradient: when this slot has any qualifying volume, the global
                // no-split short-circuit must NOT bypass MixedSubLayerGroup creation — each
                // volume needs its own no-split decision in GCode.cpp (a per-volume "last
                // run-layer" can occur on a different layer index than the per-object one). We
                // still keep the per-object short-circuit when per_vol_runs[ext] is empty, which
                // covers the legacy path bit-identically.
                bool per_vol_active_for_slot = per_vol_runs.find(ext) != per_vol_runs.end()
                                                && !per_vol_runs[ext].empty();

                if (gradient_last_no_split && !per_vol_active_for_slot) {
                    lt.mixed_filament_resolution[ext] = gradient_last_dominant_0b;
                    new_extruders.push_back(gradient_last_dominant_0b);
                    prev_print_z_for_slot[ext] = lt.print_z;
                    continue;
                }

                LayerTools::MixedSubLayerGroup grp;
                grp.mixed_slot_0based = ext;
                grp.layer_height = lh;
                grp.is_gradient = is_gradient[ext];
                for (size_t k = 0; k < s.components.size(); ++k) {
                    unsigned int comp_0based = s.components[k] - 1;
                    grp.components_0based.push_back(comp_0based);
                }
                grp.sub_heights = sub_heights;

                // Write gradient metadata (run-aware). Both per_object_gradient and
                // per_volume_gradient are populated independently from their own run-state
                // machines; the GCode emitter chooses per-region:
                //  - tagged region (gradient_volume_id valid)  -> per_volume_gradient[{obj, vol}]
                //  - untagged region (modifier / painted / etc.) -> per_object_gradient[obj]
                // Populating both keeps the per-object run state correct even when per-volume
                // takes over for the same (slot, obj), and lets untagged geometry (which is
                // never split per-volume) keep its per-object gradient ratios.
                if (grp.is_gradient) {
                    auto vol_runs_slot_it = per_vol_runs.find(ext);
                    if (vol_runs_slot_it != per_vol_runs.end()) {
                        auto vol_slot_it = m_gradient_volume_layers.find(ext);
                        for (auto &[vkey, st] : vol_runs_slot_it->second) {
                            auto &layer_indices = vol_slot_it->second[vkey];
                            if (!std::binary_search(layer_indices.begin(), layer_indices.end(), layer_idx))
                                continue;
                            if (st.current_run < 0 ||
                                st.current_idx >= st.run_lengths[st.current_run]) {
                                st.current_run++;
                                st.current_idx = 0;
                            }
                            size_t run_N   = st.run_lengths[st.current_run];
                            size_t run_idx = st.current_idx++;
                            grp.per_volume_gradient[vkey] = {
                                run_N,
                                run_idx,
                                gradient_info[ext].start,
                                gradient_info[ext].end_val,
                                gradient_info[ext].curve,
                            };
                        }
                    }

                    auto runs_slot_it = per_obj_runs.find(ext);
                    if (runs_slot_it != per_obj_runs.end()) {
                        auto slot_it = m_mixed_object_layers.find(ext);
                        for (auto &[obj, st] : runs_slot_it->second) {
                            auto &layer_indices = slot_it->second[obj];
                            if (!std::binary_search(layer_indices.begin(), layer_indices.end(), layer_idx))
                                continue;
                            if (st.current_run < 0 ||
                                st.current_idx >= st.run_lengths[st.current_run]) {
                                st.current_run++;
                                st.current_idx = 0;
                            }
                            size_t run_N  = st.run_lengths[st.current_run];
                            size_t run_idx = st.current_idx++;
                            grp.per_object_gradient[obj] = {
                                run_N,
                                run_idx,
                                gradient_info[ext].start,
                                gradient_info[ext].end_val,
                                gradient_info[ext].curve,
                            };
                        }
                    }
                }

                if (grp.components_0based.size() > 1) {
                    unsigned int first_comp_0based = s.components[0] - 1;
                    std::vector<size_t> idx(grp.components_0based.size());
                    std::iota(idx.begin(), idx.end(), 0);
                    std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b) {
                        return grp.components_0based[a] < grp.components_0based[b];
                    });
                    std::vector<unsigned int> sorted_comps;
                    std::vector<double> sorted_heights;
                    for (size_t i : idx) {
                        sorted_comps.push_back(grp.components_0based[i]);
                        sorted_heights.push_back(grp.sub_heights[i]);
                    }
                    grp.components_0based = std::move(sorted_comps);
                    grp.sub_heights = std::move(sorted_heights);
                    if (grp.is_gradient) {
                        for (size_t i = 0; i < grp.components_0based.size(); ++i) {
                            if (grp.components_0based[i] == first_comp_0based) {
                                grp.gradient_first_sorted_idx = static_cast<int>(i);
                                break;
                            }
                        }
                    }
                }

                for (unsigned int comp : grp.components_0based)
                    new_extruders.push_back(comp);
                lt.mixed_sub_layer_groups.push_back(std::move(grp));
                prev_print_z_for_slot[ext] = lt.print_z;
            } else {
                // Deficit Round-Robin: pick one component per layer.
                // Weight by layer height so volume ratios stay accurate
                // even with adaptive layer heights.
                double lh = calc_slot_lh(ext, lt.print_z);
                long long lh_i = std::llround(lh * 1e6);

                // For 2-component gradient on the first layer, use the gradient's
                // starting ratio instead of the configured mixing ratio so the
                // selected filament matches the gradient's "from" end.
                // Only affects the first layer; when sublayer splitting is enabled
                // (required for gradient), layers 1+ take the sublayer path and
                // do not touch the DRR accumulator.
                if (layer_idx == 0 && is_gradient[ext] && s.components.size() == 2) {
                    double r0 = gradient_info[ext].start;
                    s.accum[0] += std::llround(r0 * lh_i);
                    s.accum[1] += std::llround((1.0 - r0) * lh_i);
                } else {
                    for (size_t k = 0; k < s.ratios.size(); ++k)
                        s.accum[k] += std::llround(s.ratios[k] * lh_i);
                }
                size_t sel = 0;
                for (size_t k = 1; k < s.accum.size(); ++k)
                    if (s.accum[k] > s.accum[sel])
                        sel = k;
                s.accum[sel] -= lh_i;
                unsigned int resolved = s.components[sel] - 1;
                lt.mixed_filament_resolution[ext] = resolved;
                new_extruders.push_back(resolved);
                prev_print_z_for_slot[ext] = lt.print_z;
            }
        }
        lt.extruders = new_extruders;
        sort_remove_duplicates(lt.extruders);

        // Update prev_relevant_z: for each slot that has relevant-layer tracking,
        // advance if the current layer belongs to a slot-owning object.
        for (auto &[slot, rel_set] : slot_relevant_layers) {
            if (rel_set.count(layer_idx))
                prev_relevant_z_for_slot[slot] = lt.print_z;
        }

        prev_print_z = lt.print_z;
    }
}

void ToolOrdering::enforce_mixed_component_order()
{
    for (LayerTools &lt : m_layer_tools) {
        if (lt.mixed_sub_layer_groups.empty())
            continue;

        // Build a set of extruders present in lt.extruders for fast lookup.
        std::set<unsigned int> ext_set(lt.extruders.begin(), lt.extruders.end());

        // 1. Build DAG from mixed group constraints.
        //    For each group [c0, c1, c2, ...], add edges c0->c1, c1->c2, ...
        //    Only between components that are both present in lt.extruders.
        //    Use an edge set to avoid duplicate edges inflating in-degree.
        std::map<unsigned int, std::vector<unsigned int>> adj;
        std::map<unsigned int, int> in_degree;
        std::set<std::pair<unsigned int, unsigned int>> edge_set;

        for (unsigned int ext : lt.extruders)
            in_degree[ext] = 0;

        for (const auto &grp : lt.mixed_sub_layer_groups) {
            for (size_t i = 0; i + 1 < grp.components_0based.size(); ++i) {
                unsigned int a = grp.components_0based[i];
                unsigned int b = grp.components_0based[i + 1];
                if (!ext_set.count(a) || !ext_set.count(b))
                    continue;
                if (edge_set.insert({a, b}).second) {
                    adj[a].push_back(b);
                    in_degree[b] += 1;
                }
            }
        }

        // 2. Record original position (from flush optimizer) as priority.
        std::map<unsigned int, size_t> orig_pos;
        for (size_t i = 0; i < lt.extruders.size(); ++i)
            orig_pos[lt.extruders[i]] = i;

        // 3. Kahn's topological sort with priority queue (prefer original position).
        auto cmp = [&orig_pos](unsigned int lhs, unsigned int rhs) {
            return orig_pos[lhs] > orig_pos[rhs]; // min-heap by orig_pos
        };
        std::priority_queue<unsigned int, std::vector<unsigned int>, decltype(cmp)> pq(cmp);

        for (unsigned int ext : lt.extruders) {
            if (in_degree[ext] == 0)
                pq.push(ext);
        }

        std::vector<unsigned int> ordered;
        ordered.reserve(lt.extruders.size());
        while (!pq.empty()) {
            unsigned int ext = pq.top();
            pq.pop();
            ordered.push_back(ext);
            if (auto it = adj.find(ext); it != adj.end()) {
                for (unsigned int next : it->second) {
                    if (--in_degree[next] == 0)
                        pq.push(next);
                }
            }
        }

        // Safety: if topological sort didn't produce all elements, keep original order.
        if (ordered.size() != lt.extruders.size())
            ordered = lt.extruders;

        // 4. Verify: every mixed group's component order is preserved as subsequence.
        for (const auto &grp : lt.mixed_sub_layer_groups) {
            size_t prev_pos = 0;
            bool valid = true;
            for (unsigned int c : grp.components_0based) {
                if (!ext_set.count(c))
                    continue;
                auto it = std::find(ordered.begin() + prev_pos, ordered.end(), c);
                if (it == ordered.end()) { valid = false; break; }
                prev_pos = (it - ordered.begin()) + 1;
            }
            assert(valid && "enforce_mixed_component_order: mixed group subsequence violated");
            (void)valid;
        }

        lt.extruders = ordered;
    }
}

// Declared in ToolOrdering.hpp (exposed for unit testing).
std::vector<unsigned int> parse_cyclic_order(const std::string& str, unsigned int number_of_extruders)
{
    std::vector<unsigned int> order;
    for (const std::string& token : split_string(str, ',')) {
        try {
            size_t pos      = 0;
            int    filament = std::stoi(token, &pos); // stoi skips leading whitespace by itself
            // stoi stops at the first non-digit, so "2x" would parse as 2. Require the whole token to be
            // consumed (bar trailing whitespace) to drop it like any other garbage.
            if (token.find_first_not_of(" \t\r\n", pos) != std::string::npos)
                continue;
            if (filament >= 1 && (unsigned int)filament <= number_of_extruders
                && std::find(order.begin(), order.end(), (unsigned int)(filament - 1)) == order.end())
                order.emplace_back((unsigned int)(filament - 1));
        } catch (const std::exception&) {
            // Not a number, ignore it.
        }
    }
    return order;
}

void ToolOrdering::reorder_extruders_for_minimum_flush_volume(bool reorder_first_layer)
{
    const PrintConfig* print_config = m_print_config_ptr;
    if (!print_config && m_print_object_ptr) {
        print_config = &(m_print_object_ptr->print()->config());
    }

    if (!print_config || m_layer_tools.empty())
        return;

    const unsigned int number_of_extruders = (unsigned int) filament_domain_size(*print_config);

    using FlushMatrix = std::vector<std::vector<float>>;
    size_t             nozzle_nums = print_config->nozzle_diameter.values.size();
    const auto wipe_tower_type = m_print->wipe_tower_type();

    std::vector<FlushMatrix> nozzle_flush_mtx;
    for (size_t nozzle_id = 0; nozzle_id < nozzle_nums; ++nozzle_id) {
        std::vector<float> flush_matrix(cast<float>(get_flush_volumes_matrix(print_config->flush_volumes_matrix.values, nozzle_id, nozzle_nums)));
        std::vector<std::vector<float>> wipe_volumes;
        if ((print_config->purge_in_prime_tower && print_config->single_extruder_multi_material) || wipe_tower_type == WipeTowerType::Type1) {
            for (unsigned int i = 0; i < number_of_extruders; ++i)
                wipe_volumes.push_back(std::vector<float>(flush_matrix.begin() + i * number_of_extruders, flush_matrix.begin() + (i + 1) * number_of_extruders));
        } else {
            // populate wipe_volumes with prime_volume
            for (unsigned int i = 0; i < number_of_extruders; ++i)
                wipe_volumes.push_back(std::vector<float>(number_of_extruders, print_config->prime_volume));
        }
        nozzle_flush_mtx.emplace_back(wipe_volumes);
    }

    // Fast purge mode uses flush_multiplier_fast; Default is inert.
    auto flush_multiplies = (print_config->prime_volume_mode == PrimeVolumeMode::pvmFast) ? print_config->flush_multiplier_fast.values
                                                                                          : print_config->flush_multiplier.values;
    flush_multiplies.resize(nozzle_nums, 1);
    for (size_t nozzle_id = 0; nozzle_id < nozzle_nums; ++nozzle_id) {
        for (auto& vec : nozzle_flush_mtx[nozzle_id]) {
            for (auto& v : vec)
                v *= flush_multiplies[nozzle_id];
        }
    }

    std::vector<int>filament_maps(number_of_extruders, 0);
    FilamentMapMode map_mode = FilamentMapMode::fmmAutoForFlush;

    std::vector<std::vector<unsigned int>> layer_filaments;
    for (auto& lt : m_layer_tools) {
        layer_filaments.emplace_back(lt.extruders);
    }

    std::vector<unsigned int> used_filaments = collect_sorted_used_filaments(layer_filaments);

    std::vector<std::set<int>>geometric_unprintables = m_print->get_geometric_unprintable_filaments();

    // Unprintable sets are keyed by filament id, but a mixed-color slot is virtual: what actually
    // reaches the nozzle are its components. Expand the slot to those components so a geometric
    // restriction is applied to the filaments really being printed. No-op without mixed filaments.
    {
        const auto &is_mixed  = m_print->config().filament_is_mixed.values;
        const auto &comp_strs = m_print->config().filament_mixed_components.values;
        if (has_any_mixed_filament(is_mixed))
            expand_mixed_slots_in_unprintables(geometric_unprintables, is_mixed, comp_strs);
    }

    std::vector<std::set<int>>physical_unprintables = m_print->get_physical_unprintable_filaments(used_filaments);
    auto filament_unprintable_volumes = m_print->get_filament_unprintable_flow(used_filaments);

    filament_maps = m_print->get_filament_maps();
    map_mode = m_print->get_filament_map_mode();

    // Grouping now yields a nozzle-aware LayeredNozzleGroupResult; the
    // extruder-level filament_maps that feeds the ordering/stats below is derived from it.
    MultiNozzleUtils::LayeredNozzleGroupResult grouping_result;

    // The custom-sequence machinery is built before the grouping decision so both the static reorder
    // and the dynamic per-layer plan can share it. Pure local setup (no dependency on the
    // grouping result), so hoisting it above the branch does not change the static path's output.
    std::vector<std::vector<unsigned int>>filament_sequences;
    std::vector<unsigned int>filament_lists(number_of_extruders);
    std::iota(filament_lists.begin(), filament_lists.end(), 0);

    std::vector<LayerPrintSequence> other_layers_seqs;
    const ConfigOptionInts* other_layers_print_sequence_op = print_config->option<ConfigOptionInts>("other_layers_print_sequence");
    const ConfigOptionInt* other_layers_print_sequence_nums_op = print_config->option<ConfigOptionInt>("other_layers_print_sequence_nums");
    if (other_layers_print_sequence_op && other_layers_print_sequence_nums_op) {
        const std::vector<int>& print_sequence = other_layers_print_sequence_op->values;
        int                     sequence_nums = other_layers_print_sequence_nums_op->value;
        other_layers_seqs = get_other_layers_print_sequence(sequence_nums, print_sequence);
    }

    std::vector<unsigned int>first_layer_filaments;
    if (!m_layer_tools.empty())
        first_layer_filaments = m_layer_tools[0].extruders;

    // Cadence modes never take the cyclic order: their layer order is the flush order plus
    // Cadence's own fine-first and interface rules. Both modes refuse cyclic order in
    // Print::validate(), before a plate reaches this guard.
    const bool use_cyclic_ordering = ! is_mixed_nozzle_slicing_enabled(*print_config) &&
        (print_config->toolchange_ordering == ToolChangeOrderingType::Cyclic);

    // By default the first layer keeps its adhesion-optimized order (and any custom first layer
    // sequence); the cyclic sequence is only forced onto it when the user opts in.
    const bool cyclic_first_layer = use_cyclic_ordering && print_config->toolchange_cyclic_first_layer.value;

    // Optional user defined cyclic sequence, given as 1-based filament numbers ("3,2,1,4"). Filaments
    // missing from it keep their ascending order after the listed ones, so a partial or bogus entry
    // still yields the default cyclic order.
    const std::vector<unsigned int> cyclic_order =
        use_cyclic_ordering ? parse_cyclic_order(print_config->toolchange_cyclic_order.value, number_of_extruders)
                            : std::vector<unsigned int>();

    // Reorder a layer's filaments (0-based) for cyclic ordering: ascending by default, or following the
    // user defined sequence when one was given. Filaments absent from the sequence keep ascending order
    // after the listed ones.
    auto apply_cyclic_order = [&cyclic_order](std::vector<unsigned int>& filaments) {
        std::sort(filaments.begin(), filaments.end());
        if (!cyclic_order.empty())
            std::stable_sort(filaments.begin(), filaments.end(), [&cyclic_order](unsigned int lhs, unsigned int rhs) {
                auto rank = [&cyclic_order](unsigned int filament) {
                    return size_t(std::find(cyclic_order.begin(), cyclic_order.end(), filament) - cyclic_order.begin());
                };
                return rank(lhs) < rank(rhs);
            });
    };

    // other_layers_seq: the layer_idx and extruder_idx are base on 1
    auto get_custom_seq = [&other_layers_seqs, &reorder_first_layer, &first_layer_filaments, &layer_filaments, use_cyclic_ordering, cyclic_first_layer, &apply_cyclic_order](int layer_idx, std::vector<int>& out_seq) -> bool {
        if (!reorder_first_layer && layer_idx == 0) {
            // The first layer tool order is already decided (adhesion-optimized, plus any custom first
            // layer sequence). Only override it with the cyclic sequence when the user opted in.
            std::vector<unsigned int> ordered = first_layer_filaments;
            if (cyclic_first_layer)
                apply_cyclic_order(ordered);
            out_seq.resize(ordered.size());
            std::transform(ordered.begin(), ordered.end(), out_seq.begin(), [](auto item) {return int(item) + 1; });
            return true;
        }
        for (size_t idx = other_layers_seqs.size() - 1; idx != size_t(-1); --idx) {
            const auto& other_layers_seq = other_layers_seqs[idx];
            if (layer_idx + 1 >= other_layers_seq.first.first && layer_idx + 1 <= other_layers_seq.first.second) {
                out_seq = other_layers_seq.second;
                return true;
            }
        }

        // Skip the first layer here (layer_idx == 0 only reaches this point on the reorder_first_layer
        // path) unless the user asked for cyclic order on it, so it keeps the default flush ordering.
        if (use_cyclic_ordering && layer_idx >= 0 && (layer_idx != 0 || cyclic_first_layer)
            && size_t(layer_idx) < layer_filaments.size()) {
            std::vector<unsigned int> ordered = layer_filaments[size_t(layer_idx)];
            apply_cyclic_order(ordered);
            out_seq.resize(ordered.size());
            std::transform(ordered.begin(), ordered.end(), out_seq.begin(), [](auto item) { return int(item) + 1; });
            return true;
        }

        return false;
        };

    // Dynamic (per-layer filament-selector) regroup. is_dynamic_group_reorder()
    // gates on enable_filament_dynamic_map (unset on every current profile), so this is closed for the
    // whole shipping fleet AND H2C static mode — the static branch below is the only one they take, so
    // their g-code is byte-identical. Only an H2C profile that enables the selector opens this branch.
    const bool dynamic_reorder = m_print && m_print->is_dynamic_group_reorder();
    // Orca: there is no is_sequential_print() helper, so the not-sequential check is mirrored with
    // the same predicate the static by-object gate below uses. Sequential prints (with more than
    // one object) publish and write back from the by-object branch in Print::process instead of
    // from each per-object ordering.
    const bool not_sequential = print_config->print_sequence != PrintSequence::ByObject ||
                                (m_print && m_print->objects().size() == 1);

    if (dynamic_reorder) {
        // Build the grouping context, plan per-combo-range nozzle maps + filament orders, then wrap the
        // per-layer maps in a selector (4-arg create) result — which sets support_dynamic_nozzle_map and
        // lights the GCode per-layer hotend/nozzle placeholders. filament_sequences is produced here, so
        // the static reorder below is skipped for this branch.
        auto grouping_context = build_filament_group_context(m_print, layer_filaments, physical_unprintables, geometric_unprintables, filament_unprintable_volumes, FilamentMapMode::fmmAutoForFlush,
                                                             m_initial_nozzle_status.get_nozzle_filament_map());
        // The time estimator's per-extruder print times are global and do not apply to per-range
        // grouping.
        grouping_context.speed_info.group_with_time = false;

        m_nozzle_status = m_initial_nozzle_status;
        auto dynamic_plan_res = plan_filament_mapping_and_order_by_combo_ranges(m_print, grouping_context, get_custom_seq, FilamentMapMode::fmmAutoForFlush,
                                                                                physical_unprintables, geometric_unprintables, filament_unprintable_volumes, &m_nozzle_status);

        std::vector<std::vector<int>> nozzle_map_per_layer;
        for (auto& res : dynamic_plan_res) {
            filament_sequences.emplace_back(cast<unsigned int>(res.fil_order));
            nozzle_map_per_layer.emplace_back(res.fil_nozzle_match);
        }

        auto result   = MultiNozzleUtils::LayeredNozzleGroupResult::create(nozzle_map_per_layer, grouping_context.nozzle_info.nozzle_list, used_filaments, filament_sequences);
        grouping_result = result ? *result : MultiNozzleUtils::LayeredNozzleGroupResult();

        // Derive the extruder-level map for the stats path; the write-back resolves the
        // per-variant slots from the full grouping result itself.
        std::vector<int> derived_maps = grouping_result.get_extruder_map(false); // 1-based
        if (!derived_maps.empty()) {
            filament_maps = derived_maps;
            // A sequential per-object plan must not write its own map: the objects' plans are
            // stitched print-wide afterwards and written back once from there.
            if (not_sequential)
                m_print->update_to_config_by_nozzle_group_result(grouping_result);
        }
        std::transform(filament_maps.begin(), filament_maps.end(), filament_maps.begin(), [](int value) { return value - 1; });
    }
    // only check and map in sequence mode, in by object mode, we check the map in print.cpp
    else if (print_config->print_sequence != PrintSequence::ByObject || m_print->objects().size() == 1) {
        grouping_result = ToolOrdering::get_recommended_filament_maps(layer_filaments, m_print, map_mode, physical_unprintables, geometric_unprintables, filament_unprintable_volumes);
        std::vector<int> derived_maps = grouping_result.get_extruder_map(false); // 1-based extruder map

        if (map_mode < FilamentMapMode::fmmManual) {
            if (derived_maps.empty())
                return;
            filament_maps = derived_maps;
        } else if (!derived_maps.empty()) {
            // Manual modes: the result mirrors the user's config map; adopt it for consistency.
            filament_maps = derived_maps;
        }
        // Write the maps back for every mode: used filaments adopt the engine's extruder/nozzle
        // choice, unused ones keep their config assignment. In manual modes the extruder map
        // mirrors the user's map (a deviation throws in get_recommended_filament_maps).
        if (!derived_maps.empty()) {
            // Orca: the config maps are the merge base; fall back to a synthesized base when no
            // producer sized them to the filament count (CLI runs until the per-filament
            // synthesis lands there), where indexing per filament would run out of bounds.
            std::vector<int> base_filament_map = print_config->filament_map.values;
            if (base_filament_map.size() != derived_maps.size())
                base_filament_map.assign(derived_maps.size(), 1);
            std::vector<int> base_volume_map = print_config->filament_volume_map.values;
            if (base_volume_map.size() != derived_maps.size())
                base_volume_map.assign(derived_maps.size(), (int)nvtStandard);
            m_print->update_filament_maps_to_config(FilamentGroupUtils::update_used_filament_values(base_filament_map, derived_maps, used_filaments),
                                                    FilamentGroupUtils::update_used_filament_values(base_volume_map, grouping_result.get_volume_map(), used_filaments),
                                                    grouping_result.get_nozzle_map());
        }
        std::transform(filament_maps.begin(), filament_maps.end(), filament_maps.begin(), [](int value) { return value - 1; });

        if (m_print->is_BBL_printer())
        check_filament_printable_after_group(used_filaments, filament_maps, print_config);
    }
    else {
        // by-object: grouping was decided in Print.cpp; just wrap the (0-based) config map.
        std::transform(filament_maps.begin(), filament_maps.end(), filament_maps.begin(), [](int value) {return value - 1; });
        grouping_result = build_group_result_from_map(*print_config, filament_maps, used_filaments);
    }

    // The grouping result comes from the nozzle-centric engine. For single-nozzle-per-extruder printers
    // (incl. H2D) each filament resolves to nozzle_id == extruder_id, so GCode's static hotend/nozzle
    // placeholders are unchanged; H2C/A2L resolve to a nozzle-granular result (dynamic mode
    // resolves per-layer). GCode consumes this via Print::get_layered_nozzle_group_result().
    m_nozzle_group_result = grouping_result;
    // Orca: the ToolOrdering member is stored unconditionally, but the Print-level store is gated
    // behind the not-sequential check hoisted above.
    if (m_print != nullptr && not_sequential)
        m_print->set_nozzle_group_result(std::make_shared<MultiNozzleUtils::LayeredNozzleGroupResult>(m_nozzle_group_result));

    auto maps_without_group = filament_maps;
    for (auto& item : maps_without_group)
        item = 0;

    // The dynamic branch produced filament_sequences itself (per-layer selector plan); only the static
    // path needs the extruder-map flush reorder here.
    if (!dynamic_reorder) {
        reorder_filaments_for_minimum_flush_volume(
            filament_lists,
            m_print->is_BBL_printer() ? filament_maps : maps_without_group, // non-bbl printers do not support filament group yet
            layer_filaments,
            nozzle_flush_mtx,
            get_custom_seq,
            &filament_sequences
        );
    }

    // The three-mode flush-stat caches are now computed from the nozzle-aware grouping result via the
    // nozzle-aware calc_filament_change_info_by_toolorder. Stats are GUI-only (surfaced by
    // get_filament_change_stats for the mode comparison); they never feed g-code, so this block is
    // byte-inert. For single-nozzle-per-extruder printers (H2D/X1/...) nozzle_id == extruder_id, so
    // every cached value equals the extruder-level stats.
    auto curr_flush_info = calc_filament_change_info_by_toolorder(print_config, grouping_result, nozzle_flush_mtx, filament_sequences);
    if (nozzle_nums <= 1)
        m_stats_by_single_extruder = curr_flush_info;
    else {
        m_stats_by_multi_extruder_curr = curr_flush_info;
        if (map_mode == fmmAutoForFlush)
            m_stats_by_multi_extruder_best = curr_flush_info;
    }

    // in multi extruder mode, collect data under the other modes (for the GUI mode comparison)
    if (nozzle_nums > 1) {
        // always calculate the info as if a single extruder were used
        {
            std::vector<std::vector<unsigned int>> single_extruder_sequences;
            reorder_filaments_for_minimum_flush_volume(
                filament_lists,
                maps_without_group,
                layer_filaments,
                nozzle_flush_mtx,
                get_custom_seq,
                &single_extruder_sequences
            );
            // One logical nozzle (extruder 0, nozzle 0); every filament resolves to it.
            // diameter/volume_type are unused by the stat calc.
            MultiNozzleUtils::NozzleInfo single_nozzle;
            single_nozzle.volume_type = NozzleVolumeType::nvtStandard;
            single_nozzle.extruder_id = 0;
            single_nozzle.group_id    = 0;
            auto single_result = MultiNozzleUtils::LayeredNozzleGroupResult::create(maps_without_group, {single_nozzle}, used_filaments);
            if (single_result)
                m_stats_by_single_extruder = calc_filament_change_info_by_toolorder(print_config, *single_result, nozzle_flush_mtx, single_extruder_sequences);
        }
        // if not already in best-for-flush mode, also calculate the info under best-for-flush grouping
        if (map_mode != fmmAutoForFlush)
        {
            std::vector<std::vector<unsigned int>> best_sequences;
            if (dynamic_reorder) {
                // When the filament selector is active the "best" plan
                // is the per-combo-range dynamic regroup, computed over a *copy* of the initial nozzle
                // status so it cannot perturb the chosen m_nozzle_status / primary result. NOTE:
                // is_dynamic_group_reorder() implies filament_map_mode == fmmAutoForFlush, contradicting
                // this map_mode != fmmAutoForFlush guard, so this sub-branch is unreachable under the
                // current predicate. It is provably inert.
                auto best_context = build_filament_group_context(m_print, layer_filaments, physical_unprintables, geometric_unprintables, filament_unprintable_volumes, FilamentMapMode::fmmAutoForFlush,
                                                                 m_initial_nozzle_status.get_nozzle_filament_map());
                best_context.speed_info.group_with_time = false;
                MultiNozzleUtils::NozzleStatusRecorder best_nozzle_status = m_initial_nozzle_status;
                auto best_plan = plan_filament_mapping_and_order_by_combo_ranges(m_print, best_context, get_custom_seq, FilamentMapMode::fmmAutoForFlush,
                                                                                 physical_unprintables, geometric_unprintables, filament_unprintable_volumes, &best_nozzle_status);
                std::vector<std::vector<int>> best_nozzle_map_per_layer;
                for (auto& res : best_plan) {
                    best_sequences.emplace_back(cast<unsigned int>(res.fil_order));
                    best_nozzle_map_per_layer.emplace_back(res.fil_nozzle_match);
                }
                auto best_result = MultiNozzleUtils::LayeredNozzleGroupResult::create(best_nozzle_map_per_layer, best_context.nozzle_info.nozzle_list, used_filaments, best_sequences);
                if (best_result)
                    m_stats_by_multi_extruder_best = calc_filament_change_info_by_toolorder(print_config, *best_result, nozzle_flush_mtx, best_sequences);
            }
            else {
                // Best-for-flush grouping (nozzle-aware result). The extruder-level map fed to the flush
                // reorder is derived exactly as before, so best_sequences (and the flush weight) are
                // identical; only flush_filament_change_count is now charged per physical nozzle.
                auto best_group_result = get_recommended_filament_maps(layer_filaments, m_print, fmmAutoForFlush, physical_unprintables, geometric_unprintables, filament_unprintable_volumes);
                std::vector<int> best_maps = best_group_result.get_extruder_map();
                reorder_filaments_for_minimum_flush_volume(
                    filament_lists,
                    best_maps,
                    layer_filaments,
                    nozzle_flush_mtx,
                    get_custom_seq,
                    &best_sequences
                );
                m_stats_by_multi_extruder_best = calc_filament_change_info_by_toolorder(print_config, best_group_result, nozzle_flush_mtx, best_sequences);
            }
        }
    }

    const MixedNozzleFineFirstPlan fine_first = mixed_nozzle_fine_first_plan(*print_config, used_filaments);

    for (size_t i = 0; i < filament_sequences.size(); ++i) {
        m_layer_tools[i].extruders = std::move(filament_sequences[i]);
        if (fine_first.applies)
            order_fine_nozzle_first(fine_first, m_layer_tools[i].extruders);
    }

    // The first tool change lays the prime tower's bottom on the bed with the arriving filament.
    // Under a mixed-nozzle mode a support interface filament that differs from the support base goes
    // last on that layer, after the other nozzle's arrival, so the tower starts in the part and base
    // filaments, as stock's does. WipeTower::plan_lagging_tower() primes it on the new top.
    if (is_mixed_nozzle_slicing_enabled(*print_config)) {
        std::vector<unsigned int> interface_only;
        const auto note_object = [&interface_only](const PrintObject &object) {
            const int base = object.config().support_filament.value, interface_filament = object.config().support_interface_filament.value;
            if (object.has_support_material() && interface_filament > 0 && interface_filament != base)
                interface_only.push_back(unsigned(interface_filament - 1));
        };
        if (m_print != nullptr)
            for (const PrintObject *object : m_print->objects())
                note_object(*object);
        else if (m_print_object_ptr != nullptr)
            note_object(*m_print_object_ptr);
        for (LayerTools &layer_tools : m_layer_tools)
            if (layer_tools.extruders.size() > 1) {
                std::stable_partition(layer_tools.extruders.begin() + 1, layer_tools.extruders.end(), [&interface_only](unsigned int filament) {
                    return std::find(interface_only.begin(), interface_only.end(), filament) == interface_only.end();
                });
                break;
            }
    }

    if (fine_first.applies) {
        std::vector<std::vector<unsigned int>> rotated_sequences;
        rotated_sequences.reserve(m_layer_tools.size());
        size_t band_count = 0;
        for (const LayerTools &layer_tools : m_layer_tools) {
            rotated_sequences.push_back(layer_tools.extruders);
            if (is_mixed_nozzle_band_layer(fine_first, layer_tools.extruders))
                ++band_count;
        }
        m_mixed_nozzle_toolchange_stats =
            calc_filament_change_info_by_toolorder(print_config, grouping_result, nozzle_flush_mtx, rotated_sequences);
        m_mixed_nozzle_band_count = band_count;
    }
}

ToolOrdering::MixedNozzleFineFirstPlan ToolOrdering::mixed_nozzle_fine_first_plan(const PrintConfig              &config,
                                                                                 const std::vector<unsigned int> &used_filaments)
{
    // Sort used filaments by nozzle, using the same coarse extruder as the lagging tower. Applies
    // only when every used filament has a static nozzle, both sides are used, and every non-coarse
    // filament is on a narrower nozzle.
    // Feature Split takes it too on a tool changer, where several fine tools share a layer with the coarse
    // one: the flush order alone can put a fine tool after the coarse infill. Two-nozzle printers keep
    // their order.
    MixedNozzleFineFirstPlan plan;
    if (! is_mixed_nozzle_body_split(config) &&
        ! (is_mixed_nozzle_feature_split(config) && config.nozzle_diameter.values.size() > 2))
        return plan;
    const int coarse_extruder = mixed_nozzle_tower_coarse_extruder(config);
    if (coarse_extruder < 0)
        return plan;
    const double coarse_diameter = config.nozzle_diameter.get_at(size_t(coarse_extruder));
    for (const unsigned int filament : used_filaments) {
        const std::optional<size_t> physical = physical_extruder_for_filament(config, filament);
        if (! physical)
            return {};
        if (int(*physical) == coarse_extruder)
            plan.coarse.push_back(filament);
        else if (config.nozzle_diameter.get_at(*physical) < coarse_diameter)
            plan.fine.push_back(filament);
        else
            return {};
    }
    plan.applies = ! plan.fine.empty() && ! plan.coarse.empty();
    if (! plan.applies)
        return {};
    return plan;
}

void ToolOrdering::order_fine_nozzle_first(const MixedNozzleFineFirstPlan &plan, std::vector<unsigned int> &layer_filaments)
{
    if (! plan.applies || plan.fine.empty() || plan.coarse.empty())
        return;
    // Coarse filaments go last; each side keeps the flush planner's order.
    std::stable_partition(layer_filaments.begin(), layer_filaments.end(), [&plan](unsigned int filament) {
        return std::find(plan.coarse.begin(), plan.coarse.end(), filament) == plan.coarse.end();
    });
}

bool ToolOrdering::is_mixed_nozzle_band_layer(const MixedNozzleFineFirstPlan &plan, const std::vector<unsigned int> &layer_filaments)
{
    if (! plan.applies || plan.fine.empty() || plan.coarse.empty())
        return false;
    // A band is any layer that visits both nozzles, whichever filaments it carries on each.
    const auto carries_any = [&layer_filaments](const std::vector<unsigned int> &side) {
        return std::any_of(side.begin(), side.end(), [&layer_filaments](unsigned int filament) {
            return std::find(layer_filaments.begin(), layer_filaments.end(), filament) != layer_filaments.end();
        });
    };
    return carries_any(plan.fine) && carries_any(plan.coarse);
}

// Layers are marked for infinite skirt aka draft shield. Not all the layers have to be printed.
void ToolOrdering::mark_skirt_layers(const PrintConfig &config, coordf_t max_layer_height)
{
    if (m_layer_tools.empty())
        return;

    size_t i = 0;
    if (m_layer_tools.front().extruders.empty()) {
        // Empty first layer, no skirt will be printed.
        //FIXME throw an exception?
        // Unless a Body Split first cell prints on a later event layer: then the skirt starts on
        // the first layer that prints anything (see GCode::generate_skirt).
        if (!m_first_layer_band)
            return;
        while (i < m_layer_tools.size() && m_layer_tools[i].extruders.empty())
            ++ i;
        if (i == m_layer_tools.size())
            return;
    }

    for (;;) {
        m_layer_tools[i].has_skirt = true;
        size_t j = i + 1;
        for (; j < m_layer_tools.size() && ! m_layer_tools[j].has_object; ++ j);
        // i and j are two successive layers printing an object.
        if (j == m_layer_tools.size())
            // Don't print skirt above the last object layer.
            break;
        // Mark some printing intermediate layers as having skirt.
        double last_z = m_layer_tools[i].print_z;
        for (size_t k = i + 1; k < j; ++ k) {
            if (m_layer_tools[k + 1].print_z - last_z > max_layer_height + EPSILON) {
                // Layer k is the last one not violating the maximum layer height.
                // Don't extrude skirt on empty layers.
                while (m_layer_tools[k].extruders.empty())
                    -- k;
                if (m_layer_tools[k].has_skirt) {
                    // Skirt cannot be generated due to empty layers, there would be a missing layer in the skirt.
                    //FIXME throw an exception?
                    break;
                }
                m_layer_tools[k].has_skirt = true;
                last_z = m_layer_tools[k].print_z;
            }
        }
        i = j;
    }
}

// Assign a pointer to a custom G-code to the respective ToolOrdering::LayerTools.
// Ignore color changes, which are performed on a layer and for such an extruder, that the extruder will not be printing above that layer.
// If multiple events are planned over a span of a single layer, use the last one.

// BBS: replace model custom gcode with current plate custom gcode
static CustomGCode::Info custom_gcode_per_print_z;
void ToolOrdering::assign_custom_gcodes(const Print &print)
{
	// Only valid for non-sequential print.
	assert(print.config().print_sequence == PrintSequence::ByLayer);

    custom_gcode_per_print_z = print.model().get_curr_plate_custom_gcodes();
	if (custom_gcode_per_print_z.gcodes.empty())
		return;

    // BBS
	auto 						num_filaments = unsigned(print.config().filament_diameter.size());
	CustomGCode::Mode 			mode          =
		(num_filaments == 1) ? CustomGCode::SingleExtruder :
		print.object_extruders().size() == 1 ? CustomGCode::MultiAsSingle : CustomGCode::MultiExtruder;
    CustomGCode::Mode           model_mode    = print.model().get_curr_plate_custom_gcodes().mode;
	std::vector<unsigned char> 	extruder_printing_above(num_filaments, false);
	auto 						custom_gcode_it = custom_gcode_per_print_z.gcodes.rbegin();
	// Tool changes and color changes will be ignored, if the model's tool/color changes were entered in mm mode and the print is in non mm mode
	// or vice versa.
	bool 						ignore_tool_and_color_changes = (mode == CustomGCode::MultiExtruder) != (model_mode == CustomGCode::MultiExtruder);
	// If printing on a single extruder machine, make the tool changes trigger color change (M600) events.
	bool 						tool_changes_as_color_changes = mode == CustomGCode::SingleExtruder && model_mode == CustomGCode::MultiAsSingle;

	// From the last layer to the first one:
    coordf_t print_z_above = std::numeric_limits<coordf_t>::lowest();
	for (auto it_lt = m_layer_tools.rbegin(); it_lt != m_layer_tools.rend(); ++ it_lt) {
		LayerTools &lt = *it_lt;
		// Add the extruders of the current layer to the set of extruders printing at and above this print_z.
		for (unsigned int i : lt.extruders)
			extruder_printing_above[i] = true;
		// Skip all custom G-codes above this layer and skip all extruder switches.
		for (; custom_gcode_it != custom_gcode_per_print_z.gcodes.rend() && (
            (print_z_above > lt.print_z && custom_gcode_it->print_z > 0.5 * (lt.print_z + print_z_above))
            || custom_gcode_it->type == CustomGCode::ToolChange); ++ custom_gcode_it) {}
        print_z_above = lt.print_z;
		if (custom_gcode_it == custom_gcode_per_print_z.gcodes.rend())
			// Custom G-codes were processed.
			break;
		// Some custom G-code is configured for this layer or a layer below.
		const CustomGCode::Item &custom_gcode = *custom_gcode_it;
		// print_z of the layer below the current layer.
		coordf_t print_z_below = 0.;
		if (auto it_lt_below = it_lt; ++ it_lt_below != m_layer_tools.rend())
			print_z_below = it_lt_below->print_z;
        if (custom_gcode.print_z > 0.5 * (print_z_below + lt.print_z)) {
			// The custom G-code applies to the current layer.
			bool color_change = custom_gcode.type == CustomGCode::ColorChange;
			bool tool_change  = custom_gcode.type == CustomGCode::ToolChange;
			bool pause_or_custom_gcode = ! color_change && ! tool_change;
			bool apply_color_change = ! ignore_tool_and_color_changes &&
				// If it is color change, it will actually be useful as the exturder above will print.
                // BBS
				(color_change ? 
					mode == CustomGCode::SingleExtruder || 
						(custom_gcode.extruder <= int(num_filaments) && extruder_printing_above[unsigned(custom_gcode.extruder - 1)]) :
				 	tool_change && tool_changes_as_color_changes);
			if (pause_or_custom_gcode || apply_color_change)
        		lt.custom_gcode = &custom_gcode;
			// Consume that custom G-code event.
			++ custom_gcode_it;
		}
	}
}

const LayerTools& ToolOrdering::tools_for_layer(coordf_t print_z) const
{
    auto it_layer_tools = std::lower_bound(m_layer_tools.begin(), m_layer_tools.end(), LayerTools(print_z - EPSILON));
    assert(it_layer_tools != m_layer_tools.end());
    coordf_t dist_min = std::abs(it_layer_tools->print_z - print_z);
    for (++ it_layer_tools; it_layer_tools != m_layer_tools.end(); ++ it_layer_tools) {
        coordf_t d = std::abs(it_layer_tools->print_z - print_z);
        if (d >= dist_min)
            break;
        dist_min = d;
    }
    -- it_layer_tools;
    assert(dist_min < EPSILON);
    return *it_layer_tools;
}

// This function is called from Print::mark_wiping_extrusions and sets extruder this entity should be printed with (-1 .. as usual)
void WipingExtrusions::set_extruder_override(const ExtrusionEntity* entity, const PrintObject* object, size_t copy_id, int extruder, size_t num_of_copies)
{
    something_overridden = true;

    auto entity_map_it = (entity_map.emplace(std::make_tuple(entity, object), ExtruderPerCopy())).first; // (add and) return iterator
    ExtruderPerCopy& copies_vector = entity_map_it->second;
    copies_vector.resize(num_of_copies, -1);

    if (copies_vector[copy_id] != -1)
        std::cout << "ERROR: Entity extruder overriden multiple times!!!\n";    // A debugging message - this must never happen.

    copies_vector[copy_id] = extruder;
}

// BBS
void WipingExtrusions::set_support_extruder_override(const PrintObject* object, size_t copy_id, int extruder, size_t num_of_copies)
{
    something_overridden = true;
    support_map.emplace(object, extruder);
}

void WipingExtrusions::set_support_interface_extruder_override(const PrintObject* object, size_t copy_id, int extruder, size_t num_of_copies)
{
    something_overridden = true;
    support_intf_map.emplace(object, extruder);
}

// Finds first non-soluble extruder on the layer
int WipingExtrusions::first_nonsoluble_extruder_on_layer(const PrintConfig& print_config) const
{
    const LayerTools& lt = *m_layer_tools;
    for (auto extruders_it = lt.extruders.begin(); extruders_it != lt.extruders.end(); ++extruders_it)
        if (!print_config.filament_soluble.get_at(*extruders_it) && !print_config.filament_is_support.get_at(*extruders_it))
            return (*extruders_it);

    return (-1);
}

// Finds last non-soluble extruder on the layer
int WipingExtrusions::last_nonsoluble_extruder_on_layer(const PrintConfig& print_config) const
{
    const LayerTools& lt = *m_layer_tools;
    for (auto extruders_it = lt.extruders.rbegin(); extruders_it != lt.extruders.rend(); ++extruders_it)
        if (!print_config.filament_soluble.get_at(*extruders_it) && !print_config.filament_is_support.get_at(*extruders_it))
            return (*extruders_it);

    return (-1);
}

// Decides whether this entity could be overridden
bool WipingExtrusions::is_overriddable(const ExtrusionEntityCollection& eec, const PrintConfig& print_config, const PrintObject& object, const PrintRegion& region) const
{
    if (print_config.filament_soluble.get_at(m_layer_tools->extruder(eec, region)))
        return false;

    if (object.config().flush_into_objects)
        return true;

    if (!object.config().flush_into_infill || eec.role() != erInternalInfill)
        return false;

    return true;
}

// BBS
bool WipingExtrusions::is_support_overriddable(const ExtrusionRole role, const PrintObject& object) const
{
    if (!object.config().flush_into_support)
        return false;

    if (role == erMixed) {
        return object.config().support_filament == 0 || object.config().support_interface_filament == 0;
    }
    else if (role == erSupportMaterial || role == erSupportTransition) {
        return object.config().support_filament == 0;
    }
    else if (role == erSupportMaterialInterface) {
        return object.config().support_interface_filament == 0;
    }

    return false;
}

// Following function iterates through all extrusions on the layer, remembers those that could be used for wiping after toolchange
// and returns volume that is left to be wiped on the wipe tower.
float WipingExtrusions::mark_wiping_extrusions(const Print& print, unsigned int old_extruder, unsigned int new_extruder, float volume_to_wipe)
{
    const LayerTools& lt = *m_layer_tools;
    const float min_infill_volume = 0.f; // ignore infill with smaller volume than this

    if (! this->something_overridable || volume_to_wipe <= 0. || print.config().filament_soluble.get_at(old_extruder) || print.config().filament_soluble.get_at(new_extruder))
        return std::max(0.f, volume_to_wipe); // Soluble filament cannot be wiped in a random infill, neither the filament after it

    // BBS
    if (print.config().filament_is_support.get_at(old_extruder) || print.config().filament_is_support.get_at(new_extruder))
        return std::max(0.f, volume_to_wipe); // Support filament cannot be used to print support, infill, wipe_tower, etc.

    // we will sort objects so that dedicated for wiping are at the beginning:
    ConstPrintObjectPtrs object_list = print.objects().vector();
    // BBS: fix the exception caused by not fixed order between different objects
    std::sort(object_list.begin(), object_list.end(), [object_list](const PrintObject* a, const PrintObject* b) {
        if (a->config().flush_into_objects != b->config().flush_into_objects) {
            return a->config().flush_into_objects.getBool();
        }
        else {
            return a->id() < b->id();
        }
    });

    // We will now iterate through
    //  - first the dedicated objects to mark perimeters or infills (depending on infill_first)
    //  - second through the dedicated ones again to mark infills or perimeters (depending on infill_first)
    //  - then all the others to mark infills (in case that !infill_first, we must also check that the perimeter is finished already
    // this is controlled by the following variable:
    bool perimeters_done = false;

    for (int i=0 ; i<(int)object_list.size() + (perimeters_done ? 0 : 1); ++i) {
        if (!perimeters_done && (i==(int)object_list.size() || !object_list[i]->config().flush_into_objects)) { // we passed the last dedicated object in list
            perimeters_done = true;
            i=-1;   // let's go from the start again
            continue;
        }

        const PrintObject* object = object_list[i];

        // Finds this layer:
        const Layer* this_layer = object->get_layer_at_printz(lt.print_z, EPSILON);
        if (this_layer == nullptr)
        	continue;

        size_t num_of_copies = object->instances().size();

        // iterate through copies (aka PrintObject instances) first, so that we mark neighbouring infills to minimize travel moves
        for (unsigned int copy = 0; copy < num_of_copies; ++copy) {
            for (const LayerRegion *layerm : this_layer->regions()) {
                const auto &region = layerm->region();

                if (!object->config().flush_into_infill && !object->config().flush_into_objects && !object->config().flush_into_support)
                    continue;
                bool wipe_into_infill_only = !object->config().flush_into_objects && object->config().flush_into_infill;
                bool is_infill_first = region.config().is_infill_first;
                if (is_infill_first != perimeters_done || wipe_into_infill_only) {
                    for (const ExtrusionEntity* ee : layerm->fills.entities) {                      // iterate through all infill Collections
                        auto* fill = dynamic_cast<const ExtrusionEntityCollection*>(ee);

                        if (!is_overriddable(*fill, print.config(), *object, region))
                            continue;

                        if (wipe_into_infill_only && ! is_infill_first)
                            // In this case we must check that the original extruder is used on this layer before the one we are overridding
                            // (and the perimeters will be finished before the infill is printed):
                            if (!lt.is_extruder_order(lt.wall_extruder_id(region), new_extruder))
                                continue;

                        if ((!is_entity_overridden(fill, object, copy) && fill->total_volume() > min_infill_volume))
                        {     // this infill will be used to wipe this extruder
                            set_extruder_override(fill, object, copy, new_extruder, num_of_copies);
                            if ((volume_to_wipe -= float(fill->total_volume())) <= 0.f)
                            	// More material was purged already than asked for.
	                            return 0.f;
                        }
                    }
                }

                // Now the same for perimeters - see comments above for explanation:
                if (object->config().flush_into_objects && is_infill_first == perimeters_done)
                {
                    for (const ExtrusionEntity* ee : layerm->perimeters.entities) {
                        auto* fill = dynamic_cast<const ExtrusionEntityCollection*>(ee);
                        if (is_overriddable(*fill, print.config(), *object, region) && !is_entity_overridden(fill, object, copy) && fill->total_volume() > min_infill_volume) {
                            set_extruder_override(fill, object, copy, new_extruder, num_of_copies);
                            if ((volume_to_wipe -= float(fill->total_volume())) <= 0.f)
                            	// More material was purged already than asked for.
	                            return 0.f;
                        }
                    }
                }
            }

            // BBS
            if (object->config().flush_into_support) {
                auto& object_config = object->config();
                const SupportLayer* this_support_layer = object->get_support_layer_at_printz(lt.print_z, EPSILON);

                do {
                    if (this_support_layer == nullptr)
                        break;

                    bool support_overriddable = object_config.support_filament == 0;
                    bool support_intf_overriddable = object_config.support_interface_filament == 0;
                    if (!support_overriddable && !support_intf_overriddable)
                        break;

                    auto &entities = this_support_layer->support_fills.entities;
                    if (support_overriddable && !is_support_overridden(object) && !(object_config.support_interface_not_for_body.value && !support_intf_overriddable &&(new_extruder==object_config.support_interface_filament-1||old_extruder==object_config.support_interface_filament-1))) {
                        set_support_extruder_override(object, copy, new_extruder, num_of_copies);
                        for (const ExtrusionEntity* ee : entities) {
                            if (ee->role() == erSupportMaterial || ee->role() == erSupportTransition)
                                volume_to_wipe -= ee->total_volume();

                            if (volume_to_wipe <= 0.f)
                                return 0.f;
                        }
                    }

                    if (support_intf_overriddable && !is_support_interface_overridden(object)) {
                        set_support_interface_extruder_override(object, copy, new_extruder, num_of_copies);
                        for (const ExtrusionEntity* ee : entities) {
                            if (ee->role() == erSupportMaterialInterface)
                                volume_to_wipe -= ee->total_volume();

                            if (volume_to_wipe <= 0.f)
                                return 0.f;
                        }
                    }
                } while (0);
            }
        }
    }
	// Some purge remains to be done on the Wipe Tower.
    assert(volume_to_wipe > 0.);
    return volume_to_wipe;
}



// Called after all toolchanges on a layer were mark_infill_overridden. There might still be overridable entities,
// that were not actually overridden. If they are part of a dedicated object, printing them with the extruder
// they were initially assigned to might mean violating the perimeter-infill order. We will therefore go through
// them again and make sure we override it.
void WipingExtrusions::ensure_perimeters_infills_order(const Print& print)
{
	if (! this->something_overridable)
		return;

    const LayerTools& lt = *m_layer_tools;
    unsigned int first_nonsoluble_extruder = first_nonsoluble_extruder_on_layer(print.config());
    unsigned int last_nonsoluble_extruder = last_nonsoluble_extruder_on_layer(print.config());

    for (const PrintObject* object : print.objects()) {
        // Finds this layer:
        const Layer* this_layer = object->get_layer_at_printz(lt.print_z, EPSILON);
        if (this_layer == nullptr)
        	continue;
        size_t num_of_copies = object->instances().size();

        for (size_t copy = 0; copy < num_of_copies; ++copy) {    // iterate through copies first, so that we mark neighbouring infills to minimize travel moves
            for (const LayerRegion *layerm : this_layer->regions()) {
                const auto &region = layerm->region();
                //BBS
                if (!object->config().flush_into_infill && !object->config().flush_into_objects)
                    continue;

                bool is_infill_first = region.config().is_infill_first;
                for (const ExtrusionEntity* ee : layerm->fills.entities) {                      // iterate through all infill Collections
                    auto* fill = dynamic_cast<const ExtrusionEntityCollection*>(ee);

                    if (!is_overriddable(*fill, print.config(), *object, region)
                     || is_entity_overridden(fill, object, copy) )
                        continue;

                    // This infill could have been overridden but was not - unless we do something, it could be
                    // printed before its perimeter, or not be printed at all (in case its original extruder has
                    // not been added to LayerTools
                    // Either way, we will now force-override it with something suitable:
                    //BBS
                    if (is_infill_first
                    //BBS
                    //|| object->config().flush_into_objects  // in this case the perimeter is overridden, so we can override by the last one safely
                    || lt.is_extruder_order(lt.wall_extruder_id(region), last_nonsoluble_extruder    // !infill_first, but perimeter is already printed when last extruder prints
                    || ! lt.has_extruder(lt.sparse_infill_filament_id(region)))) // we have to force override - this could violate infill_first (FIXME)
                        set_extruder_override(fill, object, copy, (is_infill_first ? first_nonsoluble_extruder : last_nonsoluble_extruder), num_of_copies);
                    else {
                        // In this case we can (and should) leave it to be printed normally.
                        // Force overriding would mean it gets printed before its perimeter.
                    }
                }

                // Now the same for perimeters - see comments above for explanation:
                for (const ExtrusionEntity* ee : layerm->perimeters.entities) {                      // iterate through all perimeter Collections
                    auto* fill = dynamic_cast<const ExtrusionEntityCollection*>(ee);
                    if (is_overriddable(*fill, print.config(), *object, region) && ! is_entity_overridden(fill, object, copy))
                        set_extruder_override(fill, object, copy, (is_infill_first ? last_nonsoluble_extruder : first_nonsoluble_extruder), num_of_copies);
                }
            }
        }
    }
}

// Following function is called from GCode::process_layer and returns pointer to vector with information about which extruders should be used for given copy of this entity.
// If this extrusion does not have any override, nullptr is returned.
// Otherwise it modifies the vector in place and changes all -1 to correct_extruder_id (at the time the overrides were created, correct extruders were not known,
// so -1 was used as "print as usual").
// The resulting vector therefore keeps track of which extrusions are the ones that were overridden and which were not. If the extruder used is overridden,
// its number is saved as is (zero-based index). Regular extrusions are saved as -number-1 (unfortunately there is no negative zero).
const WipingExtrusions::ExtruderPerCopy* WipingExtrusions::get_extruder_overrides(const ExtrusionEntity* entity, const PrintObject* object, int correct_extruder_id, size_t num_of_copies)
{
	ExtruderPerCopy *overrides = nullptr;
    auto entity_map_it = entity_map.find(std::make_tuple(entity, object));
    if (entity_map_it != entity_map.end()) {
        overrides = &entity_map_it->second;
    	overrides->resize(num_of_copies, -1);
	    // Each -1 now means "print as usual" - we will replace it with actual extruder id (shifted it so we don't lose that information):
	    std::replace(overrides->begin(), overrides->end(), -1, -correct_extruder_id-1);
	}
    return overrides;
}

// BBS
int WipingExtrusions::get_support_extruder_overrides(const PrintObject* object)
{
    auto iter = support_map.find(object);
    if (iter != support_map.end())
        return iter->second;

    return -1;
}

int WipingExtrusions::get_support_interface_extruder_overrides(const PrintObject* object)
{
    auto iter = support_intf_map.find(object);
    if (iter != support_intf_map.end())
        return iter->second;

    return -1;
}

bool ToolOrdering::is_last_extrusion_layer(coordf_t print_z, unsigned int extruder_id) const
{
    auto it = m_last_layer_per_extruder.find(extruder_id);
    if (it == m_last_layer_per_extruder.end())
        return true;
    return print_z >= it->second - EPSILON;
}
} // namespace Slic3r
