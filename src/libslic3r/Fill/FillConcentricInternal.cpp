#include "../ClipperUtils.hpp"
#include "../ExPolygon.hpp"
#include "../Surface.hpp"
#include "../PrintConfig.hpp"
#include "../VariableWidth.hpp"
#include "Arachne/WallToolPaths.hpp"

#include "FillConcentricInternal.hpp"
#include <libslic3r/ShortestPath.hpp>

namespace Slic3r {

double concentric_internal_bead_nozzle(const FillParams &params, const PrintConfig &print_config)
{
    // Which nozzle this fill's Arachne beading is sized from.
    //
    // The bead floor belongs to the nozzle that will lay the fill. `params.flow` is the region's own
    // solid-infill flow (LayerRegion::flow() -> PrintRegion::flow()), which already carries that
    // region's resolved physical nozzle; Fill.cpp reads the same accessor for seam_gap.
    //
    // The printer-wide minimum is a machine quantity: using it beads every region to the narrowest
    // nozzle on the printer, so on a machine with mixed nozzles a wide-nozzle region gets roads far
    // below anything it can lay cleanly.
    //
    // Deliberately not conditioned on synchronized_multi_nozzle_layering: the error is in a shared
    // upstream path on any machine with mixed nozzles, and a condition would make the equal-height
    // feature-off/feature-on comparison bead from different nozzles. Where the machine's nozzles
    // agree, both readings are the same number and this is inert.
    const double region_nozzle = params.flow.nozzle_diameter();
    if (region_nozzle > 0.)
        return region_nozzle;

    // A fill that reaches here without a resolved flow must not bead from zero; the machine quantity
    // is the safe answer, as before.
    assert(! print_config.nozzle_diameter.values.empty());
    if (print_config.nozzle_diameter.values.empty())
        return 0.;
    return *std::min_element(print_config.nozzle_diameter.values.begin(),
                             print_config.nozzle_diameter.values.end());
}

void FillConcentricInternal::fill_surface_extrusion(const Surface* surface, const FillParams& params, ExtrusionEntitiesPtr& out)
{
    assert(this->print_config != nullptr && this->print_object_config != nullptr);

    ThickPolylines thick_polylines_out;

    for (size_t i = 0; i < this->no_overlap_expolygons.size(); ++i) {
        ExPolygon &expolygon = this->no_overlap_expolygons[i];

        // no rotation is supported for this infill pattern
        Point   bbox_size = expolygon.contour.bounding_box().size();
        coord_t min_spacing = params.flow.scaled_spacing();

        coord_t                loops_count = std::max(bbox_size.x(), bbox_size.y()) / min_spacing + 1;
        Polygons               polygons = to_polygons(expolygon);

        double min_nozzle_diameter = concentric_internal_bead_nozzle(params, *print_config);
        Arachne::WallToolPathsParams input_params;
        input_params.min_bead_width = 0.85 * min_nozzle_diameter;
        input_params.min_feature_size = 0.25 * min_nozzle_diameter;
        input_params.wall_transition_length = 0.4;
        input_params.wall_transition_angle = 10;
        input_params.wall_transition_filter_deviation = 0.25 * min_nozzle_diameter;
        input_params.wall_distribution_count = 1;

        Arachne::WallToolPaths wallToolPaths(polygons, min_spacing, min_spacing, loops_count, 0, params.layer_height, input_params);

        std::vector<Arachne::VariableWidthLines>    loops = wallToolPaths.getToolPaths();
        std::vector<const Arachne::ExtrusionLine*> all_extrusions;
        for (Arachne::VariableWidthLines& loop : loops) {
            if (loop.empty())
                continue;
            for (const Arachne::ExtrusionLine& wall : loop)
                all_extrusions.emplace_back(&wall);
        }

        // Split paths using a nearest neighbor search.
        size_t firts_poly_idx = thick_polylines_out.size();
        Point  last_pos(0, 0);
        for (const Arachne::ExtrusionLine* extrusion : all_extrusions) {
            if (extrusion->empty())
                continue;

            ThickPolyline thick_polyline = Arachne::to_thick_polyline(*extrusion);
            if (extrusion->is_closed && thick_polyline.points.front() == thick_polyline.points.back() && thick_polyline.width.front() == thick_polyline.width.back()) {
                thick_polyline.points.pop_back();
                assert(thick_polyline.points.size() * 2 == thick_polyline.width.size());
                int nearest_idx = last_pos.nearest_point_index(thick_polyline.points);
                std::rotate(thick_polyline.points.begin(), thick_polyline.points.begin() + nearest_idx, thick_polyline.points.end());
                std::rotate(thick_polyline.width.begin(), thick_polyline.width.begin() + 2 * nearest_idx, thick_polyline.width.end());
                thick_polyline.points.emplace_back(thick_polyline.points.front());
            }
            thick_polylines_out.emplace_back(std::move(thick_polyline));
        }

        // clip the paths to prevent the extruder from getting exactly on the first point of the loop
        // Keep valid paths only.
        size_t j = firts_poly_idx;
        for (size_t i = firts_poly_idx; i < thick_polylines_out.size(); ++i) {
            thick_polylines_out[i].clip_end(this->loop_clipping);
            if (thick_polylines_out[i].is_valid()) {
                if (j < i)
                    thick_polylines_out[j] = std::move(thick_polylines_out[i]);
                ++j;
            }
        }
        if (j < thick_polylines_out.size())
            thick_polylines_out.erase(thick_polylines_out.begin() + int(j), thick_polylines_out.end());

        reorder_by_shortest_traverse(thick_polylines_out);
    }

    ExtrusionEntityCollection *coll_nosort = new ExtrusionEntityCollection();
    coll_nosort->no_sort = this->no_sort(); //can be sorted inside the pass

    if (!thick_polylines_out.empty()) {
        Flow new_flow = params.flow.with_spacing(float(this->spacing));
        ExtrusionEntityCollection gap_fill;
        variable_width(thick_polylines_out, params.extrusion_role, new_flow, gap_fill.entities);
        coll_nosort->append(std::move(gap_fill.entities));
    }

    if (!coll_nosort->entities.empty())
        out.push_back(coll_nosort);
    else
        delete coll_nosort;

}


} // namespace Slic3r
