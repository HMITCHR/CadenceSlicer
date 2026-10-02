#include "Exception.hpp"
#include "MixedNozzleConfig.hpp"
#include "Print.hpp"

namespace Slic3r {

// 1-based extruder identifier for this region and role.
unsigned int PrintRegion::extruder(FlowRole role) const
{
    size_t extruder = 0;
    if (role == frPerimeter)
        extruder = m_config.inner_wall_filament_id;
    else if (role == frExternalPerimeter)
        extruder = m_config.outer_wall_filament_id;
    else if (role == frInfill)
        extruder = m_config.sparse_infill_filament_id;
    else if (role == frSolidInfill)
        extruder = m_config.internal_solid_filament_id;
    else if (role == frTopSolidInfill)
        extruder = m_config.top_surface_filament_id;
    else
        throw Slic3r::InvalidArgument("Unknown role");
    return extruder;
}

Flow PrintRegion::flow(const PrintObject &object, FlowRole role, double layer_height, bool first_layer,
                       std::optional<unsigned int> physical_tool_filament_override) const
{
    const PrintConfig          &print_config = object.print()->config();
    ConfigOptionFloatOrPercent config_width;
    // Get extrusion width from configuration.
    // (might be an absolute value, or a percent value, or zero for auto)
    if (first_layer && print_config.initial_layer_line_width.value > 0) {
        config_width = print_config.initial_layer_line_width;
    } else if (role == frExternalPerimeter) {
        config_width = m_config.outer_wall_line_width;
    } else if (role == frPerimeter) {
        config_width = m_config.inner_wall_line_width;
    } else if (role == frInfill) {
        config_width = m_config.sparse_infill_line_width;
    } else if (role == frSolidInfill) {
        config_width = m_config.internal_solid_infill_line_width;
    } else if (role == frTopSolidInfill) {
        config_width = m_config.top_surface_line_width;
    } else {
        throw Slic3r::InvalidArgument("Unknown role");
    }

    if (config_width.value == 0)
        config_width = object.config().line_width;
    
    // Off mode keeps the logical-filament lookup. Every active Mixed-Nozzle mode resolves the logical
    // feature owner through the physical-tool resolver; a missing map is an invalid bypass of
    // validation, not a fallback. A caller that knows which cell it is flowing overrides the region's
    // per-role owner here, and the whole cell is then resolved, width scaled and flowed as that tool.
    const unsigned int logical_filament = physical_tool_filament_override ?
        *physical_tool_filament_override : this->extruder(role) - 1;
    size_t nozzle_idx = logical_filament;
    if (is_mixed_nozzle_slicing_enabled(print_config)) {
        const MixedNozzleToolResolution resolution = resolve_mixed_nozzle_tool(
            print_config, logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
        if (!resolution)
            throw Slic3r::SlicingError("Mixed-Nozzle flow requires a resolved physical extruder.", object.model_object()->id().id);
        nozzle_idx = resolution.tool->physical_extruder;

        // A plain object on a Body Split plate keeps its configured widths, as with the mode off. Only a
        // Body Split object's absolute widths are carried over from the fine nozzle to the nozzle that
        // prints them.
        // The reference nozzle and the Body Split answer depend only on the object. PrintObject takes them
        // once when its slice or perimeter step starts, on the processing thread, and clears them when
        // either step is invalidated. Every input (print map and nozzles, layer height, each volume's
        // binding, cadence and paint) invalidates one of those steps, and every step that calls flow()
        // runs after them, so inside a step, including on TBB workers, flow() only reads the taken value.
        // That keeps ModelVolume::get_extruders() and its mutable painted-colour cache off worker threads.
        // A caller outside a step (validation-time tests) has no value taken yet and derives it here.
        const PrintObject::BodySplitFlowReference &taken = object.body_split_flow_reference();
        const bool body_split_object = is_mixed_nozzle_body_split(print_config) &&
            (taken.taken ? taken.body_split_object : is_body_split_object(print_config, *object.model_object()));
        if (body_split_object) {
            const std::optional<double> reference_nozzle = taken.taken ? taken.nozzle :
                body_split_object_reference_nozzle(print_config, *object.model_object(),
                                                   object.config().layer_height.value);
            const double resolved_nozzle = resolution.tool->nozzle_diameter;
            const std::optional<ConfigOptionFloatOrPercent> effective_width = reference_nozzle ?
                body_split_effective_width(config_width, resolved_nozzle, *reference_nozzle) : std::nullopt;
            if (!effective_width)
                throw Slic3r::SlicingError("Synchronized multi-nozzle layering could not derive an effective extrusion width.", object.model_object()->id().id);
            config_width = *effective_width;
        } else if (is_mixed_nozzle_feature_split(print_config)) {
            // Absolute role widths express a ratio to this region's fine-shell nozzle,
            // including first-layer and object-fallback widths selected above. Reuse Body's
            // rule so percent/auto widths retain Flow's native physical-nozzle semantics.
            // A cell override changes the road's tool, never its shell reference.
            const MixedNozzleToolResolution reference = resolve_mixed_nozzle_tool(
                print_config, size_t(m_config.outer_wall_filament_id.value) - 1,
                MixedNozzleResolveScope::PhysicalToolOnly);
            const std::optional<ConfigOptionFloatOrPercent> effective_width = reference ?
                body_split_effective_width(config_width, resolution.tool->nozzle_diameter,
                                           reference.tool->nozzle_diameter) : std::nullopt;
            if (!effective_width)
                throw Slic3r::SlicingError("Feature Split flow requires a resolved fine-shell nozzle and valid extrusion width.", object.model_object()->id().id);
            config_width = *effective_width;
        }
    }
    auto nozzle_diameter = float(print_config.nozzle_diameter.get_at(nozzle_idx));
    return Flow::new_from_config_width(role, config_width, nozzle_diameter, float(layer_height));
}

coordf_t PrintRegion::nozzle_dmr_avg(const PrintConfig &print_config) const
{
    const std::optional<double> average = mixed_nozzle_nozzle_diameter_average(
        print_config,
        {size_t(m_config.outer_wall_filament_id.value - 1),
         size_t(m_config.inner_wall_filament_id.value - 1),
         size_t(m_config.sparse_infill_filament_id.value - 1),
         size_t(m_config.internal_solid_filament_id.value - 1),
         size_t(m_config.top_surface_filament_id.value - 1),
         size_t(m_config.bottom_surface_filament_id.value - 1)});
    if (!average)
        throw Slic3r::SlicingError("Mixed-Nozzle nozzle averaging requires every logical feature owner to resolve to a valid physical tool.");
    return *average;
}

coordf_t PrintRegion::bridging_height_avg(const PrintConfig &print_config) const
{
    return this->nozzle_dmr_avg(print_config) * sqrt(m_config.bridge_flow.value);
}

void PrintRegion::collect_object_printing_extruders(const PrintConfig &print_config, const PrintRegionConfig &region_config, const bool has_brim, std::vector<unsigned int> &object_extruders)
{
    // These checks reflect the same logic used in the GUI for enabling/disabling extruder selection fields.
    // BBS
    auto num_extruders = (int)print_config.filament_diameter.size();
    auto emplace_extruder = [num_extruders, &object_extruders](int extruder_id) {
    	int i = std::max(0, extruder_id - 1);
        object_extruders.emplace_back((i >= num_extruders) ? 0 : i);
    };
    if (region_config.wall_loops.value > 0 || has_brim) {
    	emplace_extruder(region_config.outer_wall_filament_id);
                if (region_config.wall_loops.value > 1)
			emplace_extruder(region_config.inner_wall_filament_id);
    }
    if (region_config.sparse_infill_density.value > 0)
    	emplace_extruder(region_config.sparse_infill_filament_id);
    if (region_config.sparse_infill_density.value > 0 || region_config.top_shell_layers.value > 0 || region_config.bottom_shell_layers.value > 0)
    	emplace_extruder(region_config.internal_solid_filament_id);
    if (region_config.top_shell_layers.value > 0)
    	emplace_extruder(region_config.top_surface_filament_id);
    if (region_config.bottom_shell_layers.value > 0)
    	emplace_extruder(region_config.bottom_surface_filament_id);
}

void PrintRegion::collect_object_printing_extruders(const Print &print, std::vector<unsigned int> &object_extruders) const
{
    // PrintRegion, if used by some PrintObject, shall have all the extruders set to an existing printer extruder.
    // If not, then there must be something wrong with the Print::apply() function.
#ifndef NDEBUG
    // BBS
    auto num_extruders = int(print.config().filament_diameter.size());
    assert(this->config().outer_wall_filament_id    <= num_extruders);
    assert(this->config().inner_wall_filament_id    <= num_extruders);
    assert(this->config().sparse_infill_filament_id       <= num_extruders);
    assert(this->config().internal_solid_filament_id <= num_extruders);
    assert(this->config().top_surface_filament_id <= num_extruders);
    assert(this->config().bottom_surface_filament_id <= num_extruders);
#endif
    collect_object_printing_extruders(print.config(), this->config(), print.has_brim(), object_extruders);
}

}
