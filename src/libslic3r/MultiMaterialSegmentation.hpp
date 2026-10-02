#ifndef slic3r_MultiMaterialSegmentation_hpp_
#define slic3r_MultiMaterialSegmentation_hpp_

#include <utility>
#include <vector>

namespace Slic3r {

class ExPolygon;
class ModelVolume;
class Layer;
class PrintObject;
class PrintConfig;
class PrintObjectConfig;
class PrintRegionConfig;
class FacetsAnnotation;

using ExPolygons = std::vector<ExPolygon>;

struct ColoredLine
{
    Line line;
    int  color;
    int  poly_idx       = -1;
    int  local_line_idx = -1;
};

using ColoredLines = std::vector<ColoredLine>;

enum class IncludeTopAndBottomLayers {
    Yes,
    No
};

struct ModelVolumeFacetsInfo {
    const FacetsAnnotation &facets_annotation;
    // Indicate if model volume is painted.
    const bool              is_painted;
    // Indicate if the default extruder (TriangleStateType::NONE) should be replaced with the volume extruder.
    const bool              replace_default_extruder;
};

// Returns segmentation based on painting in segmentation gizmos.
std::vector<std::vector<ExPolygons>> segmentation_by_painting(const PrintObject                                               &print_object,
                                                              const std::function<ModelVolumeFacetsInfo(const ModelVolume &)> &extract_facets_info,
                                                              size_t                                                           num_facets_states,
                                                              float                                                            segmentation_max_width,
                                                              float                                                            segmentation_interlocking_depth,
                                                              bool                                                             segmentation_interlocking_beam,
                                                              IncludeTopAndBottomLayers                                        include_top_and_bottom_layers,
                                                              const std::function<void()>                                     &throw_on_cancel_callback);

// Returns multi-material segmentation based on painting in multi-material segmentation gizmo
std::vector<std::vector<ExPolygons>> multi_material_segmentation_by_painting(const PrintObject &print_object, const std::function<void()> &throw_on_cancel_callback);

// Native regional planning may need the same facet projection over a staged layer view before
// the final event layers are committed.  The regular wrapper above keeps the legacy object-layer
// behavior; this overload is the explicit read-only view used by that pre-grid path.
// `interlocking_phases`, when given, holds one entry per layer and replaces the layer index in
// the even/odd alternation of mmu_segmented_region_interlocking_depth, so a view that mixes
// layer heights can keep the alternation on one shared cadence.
std::vector<std::vector<ExPolygons>> multi_material_segmentation_by_painting(
    const PrintObject &print_object, const std::vector<Layer*> &layers,
    const std::function<void()> &throw_on_cancel_callback,
    const std::vector<size_t> *interlocking_phases = nullptr);

// Returns fuzzy skin segmentation based on painting in fuzzy skin segmentation gizmo
std::vector<std::vector<ExPolygons>> fuzzy_skin_segmentation_by_painting(const PrintObject &print_object, const std::function<void()> &throw_on_cancel_callback);

// Effective outer-wall line width for a region, resolved against its own nozzle with PrintRegion::flow's fallback.
double resolve_outer_wall_line_width(const PrintRegionConfig &region_config, const PrintObjectConfig &object_config, const PrintConfig &print_config);

} // namespace Slic3r

namespace boost::polygon {
template<> struct geometry_concept<Slic3r::ColoredLine>
{
    typedef segment_concept type;
};

template<> struct segment_traits<Slic3r::ColoredLine>
{
    typedef coord_t       coordinate_type;
    typedef Slic3r::Point point_type;

    static inline point_type get(const Slic3r::ColoredLine &line, const direction_1d &dir)
    {
        return dir.to_int() ? line.line.b : line.line.a;
    }
};
} // namespace boost::polygon

#endif // slic3r_MultiMaterialSegmentation_hpp_
