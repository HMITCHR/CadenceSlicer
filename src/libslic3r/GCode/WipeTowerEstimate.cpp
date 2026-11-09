#include "WipeTowerEstimate.hpp"

#include "libslic3r/Polygon.hpp"
#include "WipeTower.hpp"
#include "WipeTower2.hpp"
#include "../ClipperUtils.hpp"
#include "../Config.hpp"
#include "../ExPolygon.hpp"
#include "../Geometry/ConvexHull.hpp"
#include "../Line.hpp"
#include "../FilamentMixer.hpp"
#include "../MixedNozzleConfig.hpp"
#include "../Point.hpp"
#include "../PrintConfig.hpp"
#include "../libslic3r.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <set>
#include <utility>
#include <vector>
#include "libslic3r/clonable_ptr.hpp"

namespace Slic3r {

// Every caller today declares all these keys, but the signature accepts any ConfigBase: fall
// back to the key's declared default, never to a hand-copied constant.
static const ConfigOption *option_of(const ConfigBase &config, const char *key)
{
    if (const ConfigOption *opt = config.option(key); opt != nullptr)
        return opt;
    if (const ConfigDef *def = config.def(); def != nullptr)
        if (const ConfigOptionDef *opt_def = def->get(key); opt_def != nullptr)
            return opt_def->default_value.get();
    return nullptr;
}

bool prime_tower_is_printed(const ConfigBase &config, int used_filaments, int num_objects, bool has_mixed_filament)
{
    const auto *ept = config.option<ConfigOptionBool>("enable_prime_tower");
    if (ept == nullptr || !ept->value)
        return false;
    // normalize_fdm_2 only reconsiders the option when the plate uses a filament; below that it
    // leaves the user's choice alone, and there is no tower to reason about either way.
    if (used_filaments <= 0)
        return true;

    const ConfigOption *timelapse = option_of(config, "timelapse_type");
    const bool smooth_timelapse = timelapse != nullptr && timelapse->getInt() == int(TimelapseType::tlSmooth);
    const auto *wrapping = config.option<ConfigOptionBool>("enable_wrapping_detection");
    const bool enable_wrapping = wrapping != nullptr && wrapping->value;
    if (smooth_timelapse || enable_wrapping)
        return true;

    const ConfigOption *sequence = option_of(config, "print_sequence");
    const bool by_object = sequence != nullptr && sequence->getInt() == int(PrintSequence::ByObject);
    return !((used_filaments == 1 && !has_mixed_filament) || (by_object && num_objects > 1));
}

WipeTowerType resolve_wipe_tower_type(const ConfigBase &config)
{
    // printer_model is what the CLI keys its Bambu Lab detection on; the GUI's vendor flag
    // agrees for every shipped profile.
    if (const auto *model = dynamic_cast<const ConfigOptionString *>(config.option("printer_model"));
        model != nullptr && model->value.compare(0, 9, "Bambu Lab") == 0)
        return WipeTowerType::Type1;
    // By value, not by concrete type: a static PrintConfig holds ConfigOptionEnum<T>, a
    // DynamicConfig built from presets holds ConfigOptionEnumGeneric, and both answer getInt().
    const ConfigOption *type = option_of(config, "wipe_tower_type");
    return type != nullptr ? WipeTowerType(type->getInt()) : WipeTowerType::Type2;
}

Polygon estimate_wipe_tower_first_layer_outline(const ConfigBase &config, WipeTowerType tower_type, double width, double depth, double height)
{
    // Type1 ignores the cone option. The wall type is read by value: a preset-shaped config
    // holds it as ConfigOptionEnumGeneric, which a cast to ConfigOptionEnum<T> cannot see.
    const ConfigOption *wall_type  = option_of(config, "wipe_tower_wall_type");
    const ConfigOption *cone_angle = option_of(config, "wipe_tower_cone_angle");
    const bool          cone       = tower_type == WipeTowerType::Type2 && wall_type != nullptr &&
                         wall_type->getInt() == int(WipeTowerWallType::wtwCone) && cone_angle != nullptr;
    return WipeTower2::cone_base_polygon(width, depth, height, cone ? cone_angle->getFloat() : 0.);
}

WipeTowerFootprint estimate_wipe_tower_footprint(const ConfigBase &config, WipeTowerType tower_type, const std::vector<unsigned int> &filament_slots, double layer_height, double max_object_height)
{
    WipeTowerFootprint footprint;
    footprint.height = max_object_height;
    // A mixed slot is virtual: the tool ordering resolves it to one of its physical components
    // on every layer, with or without colour sublayers, so the tower only ever purges those.
    // Validation passes the slots as Print::extruders() lists them; the GUI and CLI placement
    // expand them beforehand, which repeating here leaves unchanged.
    std::vector<unsigned int> filament_ids = filament_slots;
    const auto *is_mixed_opt   = dynamic_cast<const ConfigOptionBools *>(option_of(config, "filament_is_mixed"));
    const auto *components_opt = dynamic_cast<const ConfigOptionStrings *>(option_of(config, "filament_mixed_components"));
    if (is_mixed_opt != nullptr && components_opt != nullptr && has_any_mixed_filament(is_mixed_opt->values))
        filament_ids = expand_mixed_filaments(filament_slots, is_mixed_opt->values, components_opt->values);
    const size_t filaments_cnt = filament_ids.size();
    if (filaments_cnt == 0 || layer_height < EPSILON)
        return footprint;

    auto opt_float = [&config](const char *key) {
        const ConfigOption *opt = option_of(config, key);
        return opt != nullptr ? opt->getFloat() : 0.;
    };
    auto opt_bool = [&config](const char *key) {
        const ConfigOption *opt = option_of(config, key);
        return opt != nullptr && opt->getBool();
    };
    auto opt_enum = [&config](const char *key, int fallback) {
        const ConfigOption *opt = option_of(config, key);
        return opt != nullptr ? opt->getInt() : fallback;
    };
    auto floats_of = [&config](const char *key) { return dynamic_cast<const ConfigOptionFloats *>(option_of(config, key)); };
    auto max_of = [&floats_of](const char *key, double fallback) {
        const auto *opt = floats_of(key);
        return (opt != nullptr && !opt->values.empty()) ? *std::max_element(opt->values.begin(), opt->values.end()) : fallback;
    };
    auto float_at = [&floats_of](const char *key, unsigned int id, double fallback) {
        const auto *opt = floats_of(key);
        return (opt != nullptr && !opt->values.empty()) ? opt->get_at(id) : fallback;
    };
    auto int_at = [&config](const char *key, unsigned int id, int fallback) {
        const auto *opt = dynamic_cast<const ConfigOptionInts *>(option_of(config, key));
        return (opt != nullptr && !opt->values.empty()) ? opt->get_at(id) : fallback;
    };

    // Both planners size every layer, so the tower has to fit its thinnest one: the first layer
    // when it is printed thinner than the rest.
    const double first_layer_height = opt_float("initial_layer_print_height");
    if (first_layer_height > EPSILON)
        layer_height = std::min(layer_height, first_layer_height);

    const bool   type1            = tower_type == WipeTowerType::Type1;
    const double width            = opt_float("prime_tower_width");
    const double prime_volume     = opt_float("prime_volume");
    // Type1 spaces its purge lines by prime_tower_infill_gap, Type2 by wipe_tower_extra_spacing.
    // Type2's extra flow cancels out of the depth: the line length is divided by it and the row
    // pitch multiplied by it (WipeTower2::get_wipe_depth).
    const double extra_spacing    = opt_float(type1 ? "prime_tower_infill_gap" : "wipe_tower_extra_spacing") / 100.;
    const double rib_width        = opt_float("wipe_tower_rib_width");
    const double extra_rib_length = opt_float("wipe_tower_extra_rib_length");
    const auto  *nozzle_opt       = floats_of("nozzle_diameter");
    const double nozzle_diameter  = (nozzle_opt != nullptr && !nozzle_opt->values.empty()) ? nozzle_opt->values.front() : 0.4;
    const bool   dual_nozzle      = nozzle_opt != nullptr && nozzle_opt->values.size() == 2;
    const bool   rib_wall         = opt_enum("wipe_tower_wall_type", int(WipeTowerWallType::wtwRectangle)) == int(WipeTowerWallType::wtwRib);
    const bool   smooth_timelapse = opt_enum("timelapse_type", int(TimelapseType::tlTraditional)) == int(TimelapseType::tlSmooth);
    const bool   wrapping         = opt_bool("enable_wrapping_detection");
    // Reasons a tower is printed with no tool change to purge for: the ones that stop
    // normalize_fdm_2 clearing enable_prime_tower. Its mixed-filament case is not modelled.
    const bool   need_wipe_tower  = smooth_timelapse || wrapping;
    // Cadence: a mixed-nozzle tower resolves Auto brim through height tiers and, on the Type1
    // planner, keeps a footprint floor for tall towers (WipeTower::plan_tower_new() and
    // tower_footprint_floor()). Off reads neither.
    const bool   mixed_nozzle     = opt_enum("mixed_nozzle_slicing_mode", int(MixedNozzleSlicingMode::Off)) != int(MixedNozzleSlicingMode::Off);
    // The generator takes the floor at the tower's top print_z, which sits above the model by the
    // raft and can pass the model top by up to one layer. The estimate only knows the model height,
    // so it adds an upper bound for both from the config: raft_layers levels, the raft contact gap
    // rounded up one level, and one level of slack, each level at the tallest height any nozzle can
    // lay (max_layer_height, or 0.75 x nozzle when 0, as Slicing.cpp). This keeps the floor height at
    // or above the generator's, so the floor is never smaller than the generated one. It can be
    // larger: a compact or lagging tower stops below the model top, and the bound is generous. A
    // per-object raft_layers above the config's value is not seen here.
    double floor_height = max_object_height;
    if (mixed_nozzle && type1) {
        const auto *nozzles    = floats_of("nozzle_diameter");
        const auto *max_layers = floats_of("max_layer_height");
        double      level      = std::max(opt_float("layer_height"), first_layer_height);
        for (size_t i = 0; nozzles != nullptr && i < nozzles->values.size(); ++i) {
            const double configured = (max_layers != nullptr && !max_layers->values.empty()) ? max_layers->get_at(i) : 0.;
            level = std::max(level, configured > 0. ? configured : 0.75 * nozzles->values[i]);
        }
        const int raft_layers = std::max(0, opt_enum("raft_layers", 0));
        if (raft_layers > 0)
            floor_height += double(raft_layers) * level + std::max(0., opt_float("raft_contact_distance")) + level;
        floor_height += level;
    }
    const double pad_floor        = (mixed_nozzle && type1) ? double(mixed_nozzle_tower_footprint_floor(float(floor_height))) : 0.;

    // Fewer than two filaments cannot make a tool change, so only wrapping detection or smooth
    // timelapse print a tower then. The flush volume is no proof of one: it is read from the
    // matrix of every configured slot, nonzero even when a single one of them is used.
    if (filaments_cnt < 2 && !need_wipe_tower)
        return footprint;

    // A tower printed for one of the reasons above has no tool change to purge for; both
    // planners give it the idle depth below and nothing more.
    const size_t purge_count = filaments_cnt > 1 ? (dual_nozzle ? filaments_cnt : filaments_cnt - 1) : 0;

    // Type2 purges one volume per tool change. Type1 plans per filament below; here the volume
    // only decides whether a tower exists.
    double volume = prime_volume * double(purge_count);
    if (dual_nozzle) {
        // Dual-nozzle printers also purge the filament change length on the tower.
        const double length   = max_of("filament_change_length", 0.);
        const double diameter = max_of("filament_diameter", 1.75);
        volume += length * PI * diameter * diameter / 4. * double(filaments_cnt / 2);
    }
    // Single-extruder multi-material purges the flush matrix instead of the prime volume.
    const bool semm_flush = opt_bool("purge_in_prime_tower") && opt_bool("single_extruder_multi_material");
    if (semm_flush)
        volume = WipeTower2::estimate_semm_flush_volume(config, filaments_cnt);

    // The Type1 planner wipes each filament's own prime volume after changing to it, in a block
    // per adhesiveness category. On a two-nozzle printer the leaving filament is also rammed at
    // every nozzle change; the tool order groups filaments by nozzle, so a layer crosses
    // (nozzles used - 1) times, charged here to the longest ramming.
    std::vector<WipeTower::PurgeEstimate> purges;
    if (type1 && filaments_cnt > 1) {
        const bool    saving_mode = opt_enum("prime_volume_mode", int(PrimeVolumeMode::pvmDefault)) == int(PrimeVolumeMode::pvmSaving);
        std::set<int> nozzles;
        size_t        longest_ramming = 0;
        for (size_t i = 0; i < filaments_cnt; ++i) {
            const unsigned int         id = filament_ids[i];
            WipeTower::PurgeEstimate purge;
            purge.prime_volume      = saving_mode ? 15.f : float(float_at("filament_prime_volume", id, prime_volume));
            purge.category          = int_at("filament_adhesiveness_category", id, 0);
            purge.filament_diameter = float(float_at("filament_diameter", id, 1.75));
            purges.push_back(purge);
            if (dual_nozzle) {
                nozzles.insert(int_at("filament_map", id, 1));
                if (float_at("filament_change_length", id, 0.) > float_at("filament_change_length", filament_ids[longest_ramming], 0.))
                    longest_ramming = i;
            }
        }
        if (nozzles.size() > 1)
            purges[longest_ramming].filament_change_length = float(float_at("filament_change_length", filament_ids[longest_ramming], 0.) * double(nozzles.size() - 1));
    }

    const double min_depth      = WipeTower::get_limit_depth_by_height(float(max_object_height));
    const float  perimeter_width = float(nozzle_diameter) * 1.25f; // Width_To_Nozzle_Ratio
    // With nothing to purge, plan_tower_new sizes the tower for wrapping detection or the
    // stability minimum; WipeTower2 only knows the latter.
    const double idle_depth = (type1 && wrapping && !smooth_timelapse) ? WipeTower::get_wrapping_detection_depth() : min_depth;
    if (rib_wall) {
        // Both planners square the tower to the purge area and extend the ribs, not the body,
        // below the stability minimum.
        double side;
        if (!purges.empty())
            side = WipeTower::estimate_rib_tower_bbox_side(purges, float(width), float(layer_height), float(nozzle_diameter), float(extra_spacing), float(rib_width), float(extra_rib_length), float(max_object_height));
        else {
            // Type2 squares the tower from its purge volume; Type1 with no purge list (a lone
            // filament kept for timelapse or wrapping) sizes for the idle depth.
            const bool   has_purge = !type1 && volume > EPSILON;
            const double square    = has_purge ? std::sqrt(volume / layer_height * extra_spacing) : idle_depth;
            side = WipeTower::rib_footprint_side(float(square), float(square), float(rib_width), float(extra_rib_length), float(max_object_height));
        }
        // The floor is on the printed pad, ribs included (WipeTower::rect_floor_for_pad_extent()).
        if (pad_floor > 0.)
            side = std::max(side, pad_floor);
        footprint.width = footprint.depth = side;
    } else {
        double depth;
        // Cadence: the floor widens the rectangle before the purge blocks are laid out, rounded up
        // to whole perimeter widths, and is a floor on the depth as well (plan_tower_new()).
        const double rect_width = pad_floor > 0. ? std::max(width, std::ceil(pad_floor / perimeter_width) * perimeter_width) : width;
        if (type1) {
            // plan_tower_new stretches a short purge stack to the stability minimum behind its
            // leading perimeter width.
            depth = purges.empty() ? idle_depth : std::max(min_depth + perimeter_width, double(WipeTower::estimate_tower_blocks_depth(purges, float(rect_width), float(layer_height), float(nozzle_diameter), float(extra_spacing))));
            if (pad_floor > 0.)
                depth = std::max(depth, purges.empty() ? pad_floor : pad_floor + perimeter_width);
        } else {
            depth = volume / (layer_height * width);
            // The flush volumes already hold the spacing between wipes.
            if (!semm_flush)
                depth *= extra_spacing;
            depth = std::max(min_depth, depth);
        }
        footprint.width = rect_width;
        footprint.depth = depth;
    }

    footprint.brim_width = opt_float("prime_tower_brim_width");
    if (footprint.brim_width < 0)
        footprint.brim_width = mixed_nozzle ? mixed_nozzle_tower_brim_width(float(max_object_height))
                                            : WipeTower::get_auto_brim_by_height(float(max_object_height));
    footprint.brim_width = WipeTower::estimate_brim_real_width(float(footprint.brim_width), float(nozzle_diameter), float(first_layer_height > EPSILON ? first_layer_height : layer_height), !type1);
    return footprint;
}

std::optional<Point> nearest_clear_wipe_tower_anchor(const BoundingBox &anchor_bounds, const Polygon &footprint,
                                                     const Polygons &obstacles, const Point &target)
{
    // The anchors that put the footprint on a convex obstacle form the obstacle grown by the
    // reflected footprint. For two convex shapes that Minkowski sum is the hull of the
    // differences of their vertices.
    const Polygon tower = Geometry::convex_hull(footprint.points);
    Polygons      blocked;
    blocked.reserve(obstacles.size());
    for (const Polygon &obstacle : obstacles) {
        Points differences;
        differences.reserve(obstacle.size() * tower.size());
        for (const Point &o : obstacle.points)
            for (const Point &f : tower.points)
                differences.emplace_back(o - f);
        blocked.emplace_back(Geometry::convex_hull(std::move(differences)));
    }

    // Grown by a hair so bounds pinned to one coordinate on a cramped plate still have an area.
    BoundingBox bounds = anchor_bounds;
    bounds.offset(SCALED_EPSILON);
    const ExPolygons clear = diff_ex(Polygons{ bounds.polygon() }, blocked);

    std::optional<Point> nearest;
    double               nearest_distance = std::numeric_limits<double>::max();
    for (const ExPolygon &region : clear) {
        if (region.contains(target))
            return target;
        for (const Line &edge : to_lines(region)) {
            Point        closest;
            const double distance = edge.distance_to_squared(target, &closest);
            if (distance < nearest_distance) {
                nearest_distance = distance;
                nearest          = closest;
            }
        }
    }
    return nearest;
}

} // namespace Slic3r
