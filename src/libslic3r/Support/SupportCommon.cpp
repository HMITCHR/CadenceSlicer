#include "../ClipperUtils.hpp"
// #include "../ClipperZUtils.hpp"
#include "../ExtrusionEntityCollection.hpp"
#include "../Layer.hpp"
#include "../Print.hpp"
#include "../Fill/FillBase.hpp"
#include "../MutablePolygon.hpp"
#include "../Geometry.hpp"
#include "../Point.hpp"
#include "clipper/clipper_z.hpp"

#include <cmath>
#include <boost/container/static_vector.hpp>
#include <boost/log/trivial.hpp>

#include <algorithm>
#include <array>
#include <functional>
#include <map>
#include <iomanip>
#include <set>
#include <sstream>
#include <tbb/parallel_for.h>

#include "SupportCommon.hpp"
#include "SupportLayer.hpp"
#include "SupportParameters.hpp"

// #define SLIC3R_DEBUG

// Make assert active if SLIC3R_DEBUG
#ifdef SLIC3R_DEBUG
    #define DEBUG
    #define _DEBUG
    #undef NDEBUG
    #include "../utils.hpp"
    #include "../SVG.hpp"
#endif

#include <cassert>

namespace Slic3r {

// how much we extend support around the actual contact area
//FIXME this should be dependent on the nozzle diameter!
#define SUPPORT_MATERIAL_MARGIN 1.5

//#define SUPPORT_SURFACES_OFFSET_PARAMETERS ClipperLib::jtMiter, 3.
//#define SUPPORT_SURFACES_OFFSET_PARAMETERS ClipperLib::jtMiter, 1.5
#define SUPPORT_SURFACES_OFFSET_PARAMETERS ClipperLib::jtSquare, 0.

// Convert some of the intermediate layers into top/bottom interface layers as well as base interface layers.
std::pair<SupportGeneratorLayersPtr, SupportGeneratorLayersPtr> generate_interface_layers(
    const PrintObjectConfig           &config,
    const SupportParameters           &support_params,
    const SupportGeneratorLayersPtr   &bottom_contacts,
    const SupportGeneratorLayersPtr   &top_contacts,
    // Input / output, will be merged with output. Only provided for Organic supports.
    SupportGeneratorLayersPtr         &top_interface_layers,
    SupportGeneratorLayersPtr         &top_base_interface_layers,
    // Input, will be trimmed with the newly created interface layers.
    SupportGeneratorLayersPtr         &intermediate_layers,
    SupportGeneratorLayerStorage      &layer_storage)
{
    std::pair<SupportGeneratorLayersPtr, SupportGeneratorLayersPtr> base_and_interface_layers;

    if (! intermediate_layers.empty() && support_params.has_interfaces()) {
        // For all intermediate layers, collect top contact surfaces, which are not further than support_material_interface_layers.
        BOOST_LOG_TRIVIAL(debug) << "PrintObjectSupportMaterial::generate_interface_layers() in parallel - start";
        const bool                 snug_supports          = support_params.support_style == smsSnug;
        const bool                 smooth_supports        = support_params.support_style != smsGrid;
        SupportGeneratorLayersPtr &interface_layers       = base_and_interface_layers.first;
        SupportGeneratorLayersPtr &base_interface_layers  = base_and_interface_layers.second;
        // The user-facing interface layer counts include the contact layer. Internally,
        // contact layers are generated separately, so only the remaining layers are
        // projected into intermediate interface/base-interface layers here.
        const size_t num_top_interface_layers    = support_params.has_top_contacts    ? support_params.num_top_interface_layers    - 1 : 0;
        const size_t num_bottom_interface_layers = support_params.has_bottom_contacts ? support_params.num_bottom_interface_layers - 1 : 0;
        const size_t num_top_base_interface_layers    = std::min(support_params.num_top_base_interface_layers,    num_top_interface_layers);
        const size_t num_bottom_base_interface_layers = std::min(support_params.num_bottom_base_interface_layers, num_bottom_interface_layers);
        const size_t num_top_interface_layers_only    = num_top_interface_layers    - num_top_base_interface_layers;
        const size_t num_bottom_interface_layers_only = num_bottom_interface_layers - num_bottom_base_interface_layers;

        interface_layers.assign(intermediate_layers.size(), nullptr);
        if (support_params.has_base_interfaces())
            base_interface_layers.assign(intermediate_layers.size(), nullptr);
        const auto smoothing_distance    = support_params.support_material_interface_flow.scaled_spacing() * 1.5;
        // ORCA: use top/bottom interface densities for smoothing.
        const auto minimum_island_radius_top = support_params.support_material_interface_flow.scaled_spacing() / support_params.top_interface_density;
        const auto minimum_island_radius_bottom = support_params.support_material_interface_flow.scaled_spacing() / support_params.bottom_interface_density;
        const auto closing_distance      = smoothing_distance; // scaled<float>(config.support_material_closing_radius.value);
        // Insert a new layer into base_interface_layers, if intersection with base exists.
        // ORCA: regularize top and bottom interfaces with separate minimum island radii.
        auto insert_layer = [&layer_storage, smooth_supports, closing_distance, smoothing_distance, minimum_island_radius_top, minimum_island_radius_bottom](
                SupportGeneratorLayer &intermediate_layer, Polygons &bottom, Polygons &&top, SupportGeneratorLayer *top_interface_layer,
                const Polygons *subtract, SupporLayerType type) -> SupportGeneratorLayer* {
            bool has_top_interface = top_interface_layer && ! top_interface_layer->polygons.empty();
            assert(! bottom.empty() || ! top.empty() || has_top_interface);
            // ORCA: regularize interfaces using the top/bottom radii.
            auto regularize = [&](Polygons polys, coordf_t minimum_island_radius) -> Polygons {
                if (polys.empty())
                    return polys;
                return smooth_supports ?
                    smooth_outward(closing(std::move(polys), closing_distance + minimum_island_radius, closing_distance, SUPPORT_SURFACES_OFFSET_PARAMETERS), smoothing_distance) :
                    union_safety_offset(std::move(polys));
            };
            // ORCA: apply independent smoothing to bottom vs top.
            Polygons bottom_polys = regularize(std::move(bottom), minimum_island_radius_bottom);
            Polygons top_polys = regularize(std::move(top), minimum_island_radius_top);
            append(bottom_polys, std::move(top_polys));
            bottom = intersection(std::move(bottom_polys), intermediate_layer.polygons);
            if (has_top_interface) {
                // Don't trim the precomputed Organic supports top interface with base layer
                // as the precomputed top interface likely expands over multiple tree tips.
                bottom = union_(std::move(top_interface_layer->polygons), bottom);
                top_interface_layer->polygons.clear();
            }
            if (! bottom.empty()) {
                //FIXME Remove non-printable tiny islands, let them be printed using the base support.
                //bottom = opening(std::move(bottom), minimum_island_radius);
                if (! bottom.empty()) {
                    SupportGeneratorLayer &layer_new = top_interface_layer ? *top_interface_layer : layer_storage.allocate(type);
                    layer_new.polygons   = std::move(bottom);
                    layer_new.print_z    = intermediate_layer.print_z;
                    layer_new.bottom_z   = intermediate_layer.bottom_z;
                    layer_new.height     = intermediate_layer.height;
                    layer_new.bridging   = intermediate_layer.bridging;
                    // Subtract the interface from the base regions.
                    intermediate_layer.polygons = diff(intermediate_layer.polygons, layer_new.polygons);
                    if (subtract)
                        // Trim the base interface layer with the interface layer.
                        layer_new.polygons = diff(std::move(layer_new.polygons), *subtract);
                    //FIXME filter layer_new.polygons islands by a minimum area?
        //                  $interface_area = [ grep abs($_->area) >= $area_threshold, @$interface_area ];
                    return &layer_new;
                }
            }
            return nullptr;
        };
        tbb::parallel_for(tbb::blocked_range<int>(0, int(intermediate_layers.size())),
            [&bottom_contacts, &top_contacts, &top_interface_layers, &top_base_interface_layers, &intermediate_layers, &insert_layer, &support_params,
             num_top_interface_layers, num_bottom_interface_layers, num_top_base_interface_layers, num_bottom_base_interface_layers,
             num_top_interface_layers_only, num_bottom_interface_layers_only,
             snug_supports, &interface_layers, &base_interface_layers](const tbb::blocked_range<int>& range) {
                // Gather the top / bottom contact layers intersecting with num_interface_layers resp. num_interface_layers_only intermediate layers above / below
                // this intermediate layer.
                // Index of the first top contact layer intersecting the current intermediate layer.
                auto idx_top_contact_first        = -1;
                // Index of the first bottom contact layer intersecting the current intermediate layer.
                auto idx_bottom_contact_first     = -1;
                // Index of the first top interface layer intersecting the current intermediate layer.
                auto idx_top_interface_first      = -1;
                // Index of the first top contact interface layer intersecting the current intermediate layer.
                auto idx_top_base_interface_first = -1;
                auto num_intermediate = int(intermediate_layers.size());
                for (int idx_intermediate_layer = range.begin(); idx_intermediate_layer < range.end(); ++ idx_intermediate_layer) {
                    SupportGeneratorLayer &intermediate_layer = *intermediate_layers[idx_intermediate_layer];
                    Polygons polygons_top_contact_projected_interface;
                    Polygons polygons_top_contact_projected_base;
                    Polygons polygons_bottom_contact_projected_interface;
                    Polygons polygons_bottom_contact_projected_base;
                    if (num_top_interface_layers > 0) {
                        // Top Z coordinate of a slab, over which we are collecting the top / bottom contact surfaces
                        coordf_t top_z              = intermediate_layers[std::min(num_intermediate - 1, idx_intermediate_layer + int(num_top_interface_layers) - 1)]->print_z;
                        coordf_t top_interface_z     = std::numeric_limits<coordf_t>::max();
                        if (num_top_base_interface_layers > 0)
                            // Some top base interface layers will be generated.
                            top_interface_z = num_top_interface_layers_only == 0 ?
                                // Only base interface layers to generate.
                                - std::numeric_limits<coordf_t>::max() :
                                intermediate_layers[std::min(num_intermediate - 1, idx_intermediate_layer + int(num_top_interface_layers_only) - 1)]->print_z;
                        // Move idx_top_contact_first up until above the current print_z.
                        idx_top_contact_first = idx_higher_or_equal(top_contacts, idx_top_contact_first, [&intermediate_layer](const SupportGeneratorLayer *layer){ return layer->print_z >= intermediate_layer.print_z; }); //  - EPSILON
                        // Collect the top contact areas above this intermediate layer, below top_z.
                        for (int idx_top_contact = idx_top_contact_first; idx_top_contact < int(top_contacts.size()); ++ idx_top_contact) {
                            const SupportGeneratorLayer &top_contact_layer = *top_contacts[idx_top_contact];
                            //FIXME maybe this adds one interface layer in excess?
                            if (top_contact_layer.bottom_z - EPSILON > top_z)
                                break;
                            polygons_append(top_contact_layer.bottom_z - EPSILON > top_interface_z ? polygons_top_contact_projected_base : polygons_top_contact_projected_interface,
                                // For snug supports, project the overhang polygons covering the whole overhang, so that they will merge without a gap with support polygons of the other layers.
                                // For grid supports, merging of support regions will be performed by the projection into grid.
                                snug_supports ? *top_contact_layer.overhang_polygons : top_contact_layer.polygons);
                        }
                    }
                    if (num_bottom_interface_layers > 0) {
                        // Bottom Z coordinate of a slab, over which we are collecting the top / bottom contact surfaces
                        coordf_t bottom_z           = intermediate_layers[std::max(0, idx_intermediate_layer - int(num_bottom_interface_layers) + 1)]->bottom_z;
                        coordf_t bottom_interface_z = - std::numeric_limits<coordf_t>::max();
                        if (num_bottom_base_interface_layers > 0)
                            // Some bottom base interface layers will be generated.
                            bottom_interface_z = num_bottom_interface_layers_only == 0 ?
                                // Only base interface layers to generate.
                                std::numeric_limits<coordf_t>::max() :
                                intermediate_layers[std::max(0, idx_intermediate_layer - int(num_bottom_interface_layers_only))]->bottom_z;
                        // Move idx_bottom_contact_first up until touching bottom_z.
                        idx_bottom_contact_first = idx_higher_or_equal(bottom_contacts, idx_bottom_contact_first, [bottom_z](const SupportGeneratorLayer *layer){ return layer->print_z >= bottom_z - EPSILON; });
                        // Collect the top contact areas above this intermediate layer, below top_z.
                        for (int idx_bottom_contact = idx_bottom_contact_first; idx_bottom_contact < int(bottom_contacts.size()); ++ idx_bottom_contact) {
                            const SupportGeneratorLayer &bottom_contact_layer = *bottom_contacts[idx_bottom_contact];
                            if (bottom_contact_layer.print_z - EPSILON > intermediate_layer.bottom_z)
                                break;
                            polygons_append(bottom_contact_layer.print_z - EPSILON > bottom_interface_z ? polygons_bottom_contact_projected_interface : polygons_bottom_contact_projected_base, bottom_contact_layer.polygons);
                        }
                    }
                    auto resolve_same_layer = [](SupportGeneratorLayersPtr &layers, int &idx, coordf_t print_z) -> SupportGeneratorLayer* {
                        if (! layers.empty()) {
                            idx = idx_higher_or_equal(layers, idx, [print_z](const SupportGeneratorLayer *layer) { return layer->print_z > print_z - EPSILON; });
                            if (idx < int(layers.size()) && layers[idx]->print_z < print_z + EPSILON)
                                return layers[idx];
                        }
                        return nullptr;
                    };
                    SupportGeneratorLayer *top_interface_layer      = resolve_same_layer(top_interface_layers, idx_top_interface_first, intermediate_layer.print_z);
                    SupportGeneratorLayer *top_base_interface_layer = resolve_same_layer(top_base_interface_layers, idx_top_base_interface_first, intermediate_layer.print_z);
                    SupportGeneratorLayer *interface_layer          = nullptr;
                    if (! polygons_bottom_contact_projected_interface.empty() || ! polygons_top_contact_projected_interface.empty() ||
                        (top_interface_layer && ! top_interface_layer->polygons.empty())) {
                        interface_layer = insert_layer(
                            intermediate_layer, polygons_bottom_contact_projected_interface, std::move(polygons_top_contact_projected_interface), top_interface_layer,
                            nullptr, polygons_top_contact_projected_interface.empty() ? SupporLayerType::BottomInterface : SupporLayerType::TopInterface);
                        interface_layers[idx_intermediate_layer] = interface_layer;
                    }
                    if (! polygons_bottom_contact_projected_base.empty() || ! polygons_top_contact_projected_base.empty() ||
                        (top_base_interface_layer && ! top_base_interface_layer->polygons.empty()))
                        base_interface_layers[idx_intermediate_layer] = insert_layer(
                            intermediate_layer, polygons_bottom_contact_projected_base, std::move(polygons_top_contact_projected_base), top_base_interface_layer,
                            interface_layer ? &interface_layer->polygons : nullptr, SupporLayerType::Base);
                }
            });

        // Compress contact_out, remove the nullptr items.
        // The parallel_for above may not have merged all the interface and base_interface layers
        // generated by the Organic supports code, do it here.
        auto merge_remove_empty = [](SupportGeneratorLayersPtr &in1, SupportGeneratorLayersPtr &in2) {
            auto remove_empty = [](SupportGeneratorLayersPtr &vec) {
                vec.erase(
                    std::remove_if(vec.begin(), vec.end(), [](const SupportGeneratorLayer *ptr) { return ptr == nullptr || ptr->polygons.empty(); }),
                    vec.end());
            };
            remove_empty(in1);
            remove_empty(in2);
            if (in2.empty())
                return std::move(in1);
            else if (in1.empty())
                return std::move(in2);
            else {
                SupportGeneratorLayersPtr out(in1.size() + in2.size(), nullptr);
                std::merge(in1.begin(), in1.end(), in2.begin(), in2.end(), out.begin(), [](auto* l, auto* r) { return l->print_z < r->print_z; });
                return out;
            }
        };
        interface_layers      = merge_remove_empty(interface_layers,      top_interface_layers);
        base_interface_layers = merge_remove_empty(base_interface_layers, top_base_interface_layers);
        BOOST_LOG_TRIVIAL(debug) << "PrintObjectSupportMaterial::generate_interface_layers() in parallel - end";
    }

    return base_and_interface_layers;
}

SupportGeneratorLayersPtr generate_raft_base(
    const PrintObject                 &object,
    const SupportParameters           &support_params,
	const SlicingParameters			  &slicing_params,
    const SupportGeneratorLayersPtr   &top_contacts,
    const SupportGeneratorLayersPtr   &interface_layers,
    const SupportGeneratorLayersPtr   &base_interface_layers,
    const SupportGeneratorLayersPtr   &base_layers,
    SupportGeneratorLayerStorage      &layer_storage)
{
    // If there is brim to be generated, calculate the trimming regions.
    Polygons brim;
    if (object.has_brim()) {
        // The object does not have a raft.
        // Calculate the area covered by the brim.
        const BrimType brim_type       = object.config().brim_type;
        const bool     brim_outer      = brim_type == btOuterOnly || brim_type == btOuterAndInner;
        const bool     brim_inner      = brim_type == btInnerOnly || brim_type == btOuterAndInner;
        // BBS: the pattern of raft and brim are the same, thus the brim can be serpated by support raft.
        const auto     brim_object_gap = scaled<float>(object.config().brim_object_gap.value);
        //const auto     brim_object_gap = scaled<float>(object.config().brim_object_gap.value + object.config().brim_width.value);
        for (const ExPolygon &ex : object.layers().front()->lslices) {
            if (brim_outer && brim_inner)
                polygons_append(brim, offset(ex, brim_object_gap));
            else {
                if (brim_outer)
                    polygons_append(brim, offset(ex.contour, brim_object_gap, ClipperLib::jtRound, float(scale_(0.1))));
                else
                    brim.emplace_back(ex.contour);
                if (brim_inner) {
                    Polygons holes = ex.holes;
                    polygons_reverse(holes);
                    holes = shrink(holes, brim_object_gap, ClipperLib::jtRound, float(scale_(0.1)));
                    polygons_reverse(holes);
                    polygons_append(brim, std::move(holes));
                } else
                    polygons_append(brim, ex.holes);
            }
        }
        brim = union_(brim);
    }

    // How much to inflate the support columns to be stable. This also applies to the 1st layer, if no raft layers are to be printed.
    const float inflate_factor_fine      = float(scale_((slicing_params.raft_layers() > 1) ? 0.5 : EPSILON));
    const float inflate_factor_1st_layer = std::max(0.f, float(scale_(object.config().raft_first_layer_expansion)) - inflate_factor_fine);
    SupportGeneratorLayer       *contacts         = top_contacts         .empty() ? nullptr : top_contacts         .front();
    SupportGeneratorLayer       *interfaces       = interface_layers     .empty() ? nullptr : interface_layers     .front();
    SupportGeneratorLayer       *base_interfaces  = base_interface_layers.empty() ? nullptr : base_interface_layers.front();
    SupportGeneratorLayer       *columns_base     = base_layers          .empty() ? nullptr : base_layers          .front();
    if (contacts != nullptr && contacts->print_z > std::max(slicing_params.first_print_layer_height, slicing_params.raft_contact_top_z) + EPSILON)
        // This is not the raft contact layer.
        contacts = nullptr;
    if (interfaces != nullptr && interfaces->bottom_print_z() > slicing_params.raft_interface_top_z + EPSILON)
        // This is not the raft column base layer.
        interfaces = nullptr;
    if (base_interfaces != nullptr && base_interfaces->bottom_print_z() > slicing_params.raft_interface_top_z + EPSILON)
        // This is not the raft column base layer.
        base_interfaces = nullptr;
    if (columns_base != nullptr && columns_base->bottom_print_z() > slicing_params.raft_interface_top_z + EPSILON)
        // This is not the raft interface layer.
        columns_base = nullptr;

    Polygons interface_polygons;
    if (contacts != nullptr && ! contacts->polygons.empty())
        polygons_append(interface_polygons, expand(contacts->polygons, inflate_factor_fine, SUPPORT_SURFACES_OFFSET_PARAMETERS));
    if (interfaces != nullptr && ! interfaces->polygons.empty())
        polygons_append(interface_polygons, expand(interfaces->polygons, inflate_factor_fine, SUPPORT_SURFACES_OFFSET_PARAMETERS));
    if (base_interfaces != nullptr && ! base_interfaces->polygons.empty())
        polygons_append(interface_polygons, expand(base_interfaces->polygons, inflate_factor_fine, SUPPORT_SURFACES_OFFSET_PARAMETERS));

    // Output vector.
    SupportGeneratorLayersPtr raft_layers;

    if (slicing_params.raft_layers() > 1) {
        Polygons base;
        Polygons columns;
        Polygons first_layer;
        if (columns_base != nullptr) {
            if (columns_base->bottom_print_z() > slicing_params.raft_interface_top_z - EPSILON) {
                // Classic supports with colums above the raft interface.
                base = columns_base->polygons;
                columns = base;
                if (! interface_polygons.empty())
                    // Trim the 1st layer columns with the inflated interface polygons.
                    columns = diff(columns, interface_polygons);
            } else {
                // Organic supports with raft on print bed.
                assert(is_approx(columns_base->print_z, slicing_params.first_print_layer_height));
                first_layer = columns_base->polygons;
            }
        }
        if (! interface_polygons.empty()) {
            // Merge the untrimmed columns base with the expanded raft interface, to be used for the support base and interface.
            base = union_(base, interface_polygons);
        }
        // Do not add the raft contact layer, only add the raft layers below the contact layer.
        // Insert the 1st layer.
        {
            SupportGeneratorLayer &new_layer = layer_storage.allocate(slicing_params.base_raft_layers > 0 ? SupporLayerType::RaftBase : SupporLayerType::RaftInterface);
            raft_layers.push_back(&new_layer);
            new_layer.print_z = slicing_params.first_print_layer_height;
            new_layer.height  = slicing_params.first_print_layer_height;
            new_layer.bottom_z = 0.;
            first_layer = union_(std::move(first_layer), base);
            new_layer.polygons = inflate_factor_1st_layer > 0 ? expand(first_layer, inflate_factor_1st_layer) : first_layer;
        }
        // Insert the base layers.
        for (size_t i = 1; i < slicing_params.base_raft_layers; ++ i) {
            coordf_t print_z = raft_layers.back()->print_z;
            SupportGeneratorLayer &new_layer  = layer_storage.allocate_unguarded(SupporLayerType::RaftBase);
            raft_layers.push_back(&new_layer);
            new_layer.print_z  = print_z + slicing_params.base_raft_layer_height;
            new_layer.height   = slicing_params.base_raft_layer_height;
            new_layer.bottom_z = print_z;
            new_layer.polygons = base;
        }
        // Insert the interface layers.
        for (size_t i = 1; i < slicing_params.interface_raft_layers; ++ i) {
            coordf_t print_z = raft_layers.back()->print_z;
            SupportGeneratorLayer &new_layer = layer_storage.allocate_unguarded(SupporLayerType::RaftInterface);
            raft_layers.push_back(&new_layer);
            new_layer.print_z = print_z + slicing_params.interface_raft_layer_height;
            new_layer.height  = slicing_params.interface_raft_layer_height;
            new_layer.bottom_z = print_z;
            new_layer.polygons = interface_polygons;
            //FIXME misusing contact_polygons for support columns.
            new_layer.contact_polygons = std::make_unique<Polygons>(columns);
        }
    } else {
        if (columns_base != nullptr) {
            // Expand the bases of the support columns in the 1st layer.
            Polygons &raft     = columns_base->polygons;
            Polygons  trimming;
            // BBS: if first layer of support is intersected with object island, it must have the same function as brim unless in nobrim mode.
            // brim_object_gap is changed to 0 by default, it's no longer appropriate to use it to determine the gap of first layer support.
            trimming = offset(object.layers().front()->lslices, (float) scale_(support_params.gap_xy_first_layer), SUPPORT_SURFACES_OFFSET_PARAMETERS);
            if (inflate_factor_1st_layer > SCALED_EPSILON) {
                // Inflate in multiple steps to avoid leaking of the support 1st layer through object walls.
                auto  nsteps = std::max(5, int(ceil(inflate_factor_1st_layer / support_params.first_layer_flow.scaled_width())));
                float step   = inflate_factor_1st_layer / nsteps;
                for (int i = 0; i < nsteps; ++ i)
                    raft = diff(expand(raft, step), trimming);
            } else
                raft = diff(raft, trimming);
            if (! interface_polygons.empty())
                columns_base->polygons = diff(columns_base->polygons, interface_polygons);
        }
        if (! brim.empty()) {
            if (columns_base)
                columns_base->polygons = diff(columns_base->polygons, brim);
            if (contacts)
                contacts->polygons = diff(contacts->polygons, brim);
            if (interfaces)
                interfaces->polygons = diff(interfaces->polygons, brim);
            if (base_interfaces)
                base_interfaces->polygons = diff(base_interfaces->polygons, brim);
        }
    }

    return raft_layers;
}

static inline void fill_expolygon_generate_paths(
    ExtrusionEntitiesPtr    &dst,
    ExPolygon              &&expolygon,
    Fill                    *filler,
    const FillParams        &fill_params,
    float                    density,
    ExtrusionRole            role,
    const Flow              &flow)
{
    Surface surface(stInternal, std::move(expolygon));
    Polylines polylines;
    try {
        assert(!fill_params.use_arachne);
        polylines = filler->fill_surface(&surface, fill_params);
    } catch (InfillFailedException &) {
    }
    extrusion_entities_append_paths(
        dst,
        std::move(polylines),
        role,
        flow.mm3_per_mm(), flow.width(), flow.height());
}

static inline void fill_expolygons_generate_paths(
    ExtrusionEntitiesPtr    &dst,
    ExPolygons             &&expolygons,
    Fill                    *filler,
    const FillParams        &fill_params,
    float                    density,
    ExtrusionRole            role,
    const Flow              &flow)
{
    for (ExPolygon &expoly : expolygons)
        fill_expolygon_generate_paths(dst, std::move(expoly), filler, fill_params, density, role, flow);
}

static inline void fill_expolygons_generate_paths(
    ExtrusionEntitiesPtr    &dst,
    ExPolygons             &&expolygons,
    Fill                    *filler,
    float                    density,
    ExtrusionRole            role,
    const Flow              &flow)
{
    FillParams fill_params;
    fill_params.density     = density;
    fill_params.dont_adjust = true;
    fill_expolygons_generate_paths(dst, std::move(expolygons), filler, fill_params, density, role, flow);
}

static Polylines draw_perimeters(const ExPolygon &expoly, double clip_length)
{
    // Draw the perimeters.
    Polylines polylines;
    polylines.reserve(expoly.holes.size() + 1);
    for (size_t i = 0; i <= expoly.holes.size();  ++ i) {
        Polyline pl(i == 0 ? expoly.contour.points : expoly.holes[i - 1].points);
        pl.points.emplace_back(pl.points.front());
        if (i > 0)
            // It is a hole, reverse it.
            pl.reverse();
        // so that all contours are CCW oriented.
        pl.clip_end(clip_length);
        polylines.emplace_back(std::move(pl));
    }
    return polylines;
}

void tree_supports_generate_paths(
    ExtrusionEntitiesPtr    &dst,
    const Polygons          &polygons,
    const Flow              &flow,
    const SupportParameters &support_params)
{
    // Offset expolygon inside, returns number of expolygons collected (0 or 1).
    // Vertices of output paths are marked with Z = source contour index of the expoly.
    // Vertices at the intersection of source contours are marked with Z = -1.
    auto shrink_expolygon_with_contour_idx = [](const Slic3r::ExPolygon &expoly, const float delta, ClipperLib::JoinType joinType, double miterLimit, ClipperLib_Z::Paths &out) -> int
    {
        assert(delta > 0);
        auto append_paths_with_z = [](ClipperLib::Paths &src, coord_t contour_idx, ClipperLib_Z::Paths &dst) {
            dst.reserve(next_highest_power_of_2(dst.size() + src.size()));
            for (const ClipperLib::Path &contour : src) {
                ClipperLib_Z::Path tmp;
                tmp.reserve(contour.size());
                for (const Point &p : contour)
                    tmp.emplace_back(p.x(), p.y(), contour_idx);
                dst.emplace_back(std::move(tmp));
            }
        };

        // 1) Offset the outer contour.
        ClipperLib_Z::Paths contours;
        {
            ClipperLib::ClipperOffset co;
            if (joinType == jtRound)
                co.ArcTolerance = miterLimit;
            else
                co.MiterLimit = miterLimit;
            co.ShortestEdgeLength = double(delta * 0.005);
            co.AddPath(expoly.contour.points, joinType, ClipperLib::etClosedPolygon);
            ClipperLib::Paths contours_raw;
            co.Execute(contours_raw, - delta);
            if (contours_raw.empty())
                // No need to try to offset the holes.
                return 0;
            append_paths_with_z(contours_raw, 0, contours);
        }

        if (expoly.holes.empty()) {
            // No need to subtract holes from the offsetted expolygon, we are done.
            append(out, std::move(contours));
        } else {
            // 2) Offset the holes one by one, collect the offsetted holes.
            ClipperLib_Z::Paths holes;
            {
                for (const Polygon &hole : expoly.holes) {
                    ClipperLib::ClipperOffset co;
                    if (joinType == jtRound)
                        co.ArcTolerance = miterLimit;
                    else
                        co.MiterLimit = miterLimit;
                    co.ShortestEdgeLength = double(delta * 0.005);
                    co.AddPath(hole.points, joinType, ClipperLib::etClosedPolygon);
                    ClipperLib::Paths out2;
                    // Execute reorients the contours so that the outer most contour has a positive area. Thus the output
                    // contours will be CCW oriented even though the input paths are CW oriented.
                    // Offset is applied after contour reorientation, thus the signum of the offset value is reversed.
                    co.Execute(out2, delta);
                    append_paths_with_z(out2, 1 + (&hole - expoly.holes.data()), holes);
                }
            }

            // 3) Subtract holes from the contours.
            if (holes.empty()) {
                // No hole remaining after an offset. Just copy the outer contour.
                append(out, std::move(contours));
            } else {
                // Negative offset. There is a chance, that the offsetted hole intersects the outer contour.
                // Subtract the offsetted holes from the offsetted contours.
                ClipperLib_Z::Clipper clipper;
                clipper.ZFillFunction([](const ClipperLib_Z::IntPoint &e1bot, const ClipperLib_Z::IntPoint &e1top, const ClipperLib_Z::IntPoint &e2bot, const ClipperLib_Z::IntPoint &e2top, ClipperLib_Z::IntPoint &pt) {
                        //pt.z() = std::max(std::max(e1bot.z(), e1top.z()), std::max(e2bot.z(), e2top.z()));
                        // Just mark the intersection.
                        pt.z() = -1;
                    });
                clipper.AddPaths(contours, ClipperLib_Z::ptSubject, true);
                clipper.AddPaths(holes,    ClipperLib_Z::ptClip,    true);
                ClipperLib_Z::Paths output;
                clipper.Execute(ClipperLib_Z::ctDifference, output, ClipperLib_Z::pftNonZero, ClipperLib_Z::pftNonZero);
                if (! output.empty()) {
                    append(out, std::move(output));
                } else {
                    // The offsetted holes have eaten up the offsetted outer contour.
                    return 0;
                }
            }
        }

        return 1;
    };

    const double spacing = flow.scaled_spacing();
    // Clip the sheath path to avoid the extruder to get exactly on the first point of the loop.
    const double clip_length = spacing * 0.15;
    const double anchor_length = spacing * 6.;
    ClipperLib_Z::Paths anchor_candidates;
    for (ExPolygon& expoly : closing_ex(polygons, float(SCALED_EPSILON), float(SCALED_EPSILON + 0.5 * flow.scaled_width()))) {
        std::unique_ptr<ExtrusionEntityCollection> eec;
        ExPolygons                                 regions_to_draw_inner_wall{expoly};
        if (support_params.tree_branch_diameter_double_wall_area_scaled > 0)
            if (double area = expoly.area(); area > support_params.tree_branch_diameter_double_wall_area_scaled) {
                BOOST_LOG_TRIVIAL(debug)<< "TreeSupports: double wall area: " << area<< " > " << support_params.tree_branch_diameter_double_wall_area_scaled;
                eec = std::make_unique<ExtrusionEntityCollection>();
                // Don't reorder internal / external loops of the same island, always start with the internal loop.
                eec->no_sort = true;
                // Make the tree branch stable by adding another perimeter.
                ExPolygons level2 = offset2_ex({expoly}, -1.5 * flow.scaled_width(), 0.5 * flow.scaled_width());
                if (level2.size() > 0) {
                    regions_to_draw_inner_wall = level2;
                    extrusion_entities_append_paths(eec->entities, draw_perimeters(expoly, clip_length), ExtrusionRole::erSupportMaterial, flow.mm3_per_mm(), flow.width(), flow.height(),
                            // Disable reversal of the path, always start with the anchor, always print CCW.
                            false);
                    expoly = level2.front();
                }
            }
        for (ExPolygon &expoly : regions_to_draw_inner_wall)
        {
            // Try to produce one more perimeter to place the seam anchor.
            // First genrate a 2nd perimeter loop as a source for anchor candidates.
            // The anchor candidate points are annotated with an index of the source contour or with -1 if on intersection.
            anchor_candidates.clear();
            shrink_expolygon_with_contour_idx(expoly, flow.scaled_width(), DefaultJoinType, 1.2, anchor_candidates);
            // Orient all contours CW.
            for (auto &path : anchor_candidates)
                if (ClipperLib_Z::Area(path) > 0) std::reverse(path.begin(), path.end());

            // Draw the perimeters.
            Polylines polylines;
            polylines.reserve(expoly.holes.size() + 1);
            for (int idx_loop = 0; idx_loop < int(expoly.num_contours()); ++idx_loop) {
                // Open the loop with a seam.
                const Polygon &loop = expoly.contour_or_hole(idx_loop);
                Polyline       pl(loop.points);
                // Orient all contours CW, because the anchor will be added to the end of polyline while we want to start a loop with the anchor.
                if (idx_loop == 0)
                    // It is an outer contour.
                    pl.reverse();
                pl.points.emplace_back(pl.points.front());
                pl.clip_end(clip_length);
                if (pl.size() < 2) continue;
                // Find the foot of the seam point on anchor_candidates. Only pick an anchor point that was created by offsetting the source contour.
                ClipperLib_Z::Path *closest_contour = nullptr;
                Vec2d               closest_point;
                int                 closest_point_idx = -1;
                double              closest_point_t   = 0.;
                double              d2min             = std::numeric_limits<double>::max();
                Vec2d               seam_pt           = pl.back().cast<double>();
                for (ClipperLib_Z::Path &path : anchor_candidates)
                    for (int i = 0; i < int(path.size()); ++i) {
                        int j = next_idx_modulo(i, path);
                        if (path[i].z() == idx_loop || path[j].z() == idx_loop) {
                            Vec2d pi(path[i].x(), path[i].y());
                            Vec2d pj(path[j].x(), path[j].y());
                            Vec2d v  = pj - pi;
                            Vec2d w  = seam_pt - pi;
                            auto  l2 = v.squaredNorm();
                            auto  t  = std::clamp((l2 == 0) ? 0 : v.dot(w) / l2, 0., 1.);
                            if ((path[i].z() == idx_loop || t > EPSILON) && (path[j].z() == idx_loop || t < 1. - EPSILON)) {
                                // Closest point.
                                Vec2d  fp = pi + v * t;
                                double d2 = (fp - seam_pt).squaredNorm();
                                if (d2 < d2min) {
                                    d2min             = d2;
                                    closest_contour   = &path;
                                    closest_point     = fp;
                                    closest_point_idx = i;
                                    closest_point_t   = t;
                                }
                            }
                        }
                    }
                if (d2min < sqr(flow.scaled_width() * 3.)) {
                    // Try to cut an anchor from the closest_contour.
                    // Both closest_contour and pl are CW oriented.
                    pl.points.emplace_back(closest_point.cast<coord_t>());
                    const ClipperLib_Z::Path &path             = *closest_contour;
                    double                    remaining_length = anchor_length - (seam_pt - closest_point).norm();
                    int                       i                = closest_point_idx;
                    int                       j                = next_idx_modulo(i, *closest_contour);
                    Vec2d                     pi(path[i].x(), path[i].y());
                    Vec2d                     pj(path[j].x(), path[j].y());
                    Vec2d                     v = pj - pi;
                    double                    l = v.norm();
                    if (remaining_length < (1. - closest_point_t) * l) {
                        // Just trim the current line.
                        pl.points.emplace_back((closest_point + v * (remaining_length / l)).cast<coord_t>());
                    } else {
                        // Take the rest of the current line, continue with the other lines.
                        pl.points.emplace_back(path[j].x(), path[j].y());
                        pi = pj;
                        for (i = j; path[i].z() == idx_loop && remaining_length > 0; i = j, pi = pj) {
                            j  = next_idx_modulo(i, path);
                            pj = Vec2d(path[j].x(), path[j].y());
                            v  = pj - pi;
                            l  = v.norm();
                            if (i == closest_point_idx) {
                                // Back at the first segment. Most likely this should not happen and we may end the anchor.
                                break;
                            }
                            if (remaining_length <= l) {
                                pl.points.emplace_back((pi + v * (remaining_length / l)).cast<coord_t>());
                                break;
                            }
                            pl.points.emplace_back(path[j].x(), path[j].y());
                            remaining_length -= l;
                        }
                    }
                }
                // Start with the anchor.
                pl.reverse();
                polylines.emplace_back(std::move(pl));
            }

            ExtrusionEntitiesPtr &out = eec ? eec->entities : dst;
            extrusion_entities_append_paths(out, std::move(polylines), ExtrusionRole::erSupportMaterial, flow.mm3_per_mm(), flow.width(), flow.height(),
                                            // Disable reversal of the path, always start with the anchor, always print CCW.
                                            false);
        }
        if (eec) {
            std::reverse(eec->entities.begin(), eec->entities.end());
            dst.emplace_back(eec.release());
        }
    }
}

void fill_expolygons_with_sheath_generate_paths(
    ExtrusionEntitiesPtr    &dst,
    const Polygons          &polygons,
    Fill                    *filler,
    float                    density,
    ExtrusionRole            role,
    const Flow              &flow,
    const SupportParameters& support_params,
    bool                     with_sheath,
    bool                     no_sort)
{
    if (polygons.empty())
        return;

    if (with_sheath) {
        if (density == 0) {
            tree_supports_generate_paths(dst, polygons, flow, support_params);
            return;
        }
    }
    else {
        fill_expolygons_generate_paths(dst, closing_ex(polygons, float(SCALED_EPSILON)), filler, density, role, flow);
        return;
    }

    FillParams fill_params;
    fill_params.density     = density;
    fill_params.dont_adjust = true;

    const double spacing = flow.scaled_spacing();
    // Clip the sheath path to avoid the extruder to get exactly on the first point of the loop.
    const double clip_length = spacing * 0.15;

    for (ExPolygon &expoly : closing_ex(polygons, float(SCALED_EPSILON), float(SCALED_EPSILON + 0.5*flow.scaled_width()))) {
        // Don't reorder the skirt and its infills.
        std::unique_ptr<ExtrusionEntityCollection> eec;
        if (no_sort) {
            eec = std::make_unique<ExtrusionEntityCollection>();
            eec->no_sort = true;
        }
        ExtrusionEntitiesPtr &out = no_sort ? eec->entities : dst;
        extrusion_entities_append_paths(out, draw_perimeters(expoly, clip_length), ExtrusionRole::erSupportMaterial, flow.mm3_per_mm(), flow.width(), flow.height());
        // Fill in the rest.
        fill_expolygons_generate_paths(out, offset_ex(expoly, float(-0.4 * spacing)), filler, fill_params, density, role, flow);
        if (no_sort && ! eec->empty())
            dst.emplace_back(eec.release());
    }
}

// Support layers, partially processed.
struct SupportGeneratorLayerExtruded
{
    SupportGeneratorLayerExtruded& operator=(SupportGeneratorLayerExtruded &&rhs) {
        this->layer = rhs.layer;
        this->extrusions = std::move(rhs.extrusions);
        m_polygons_to_extrude = std::move(rhs.m_polygons_to_extrude);
        rhs.layer = nullptr;
        return *this;
    }

    bool empty() const {
        return layer == nullptr || layer->polygons.empty();
    }

    void set_polygons_to_extrude(Polygons &&polygons) {
        if (m_polygons_to_extrude == nullptr)
            m_polygons_to_extrude = std::make_unique<Polygons>(std::move(polygons));
        else
            *m_polygons_to_extrude = std::move(polygons);
    }
    Polygons& polygons_to_extrude() { return (m_polygons_to_extrude == nullptr) ? layer->polygons : *m_polygons_to_extrude; }
    const Polygons& polygons_to_extrude() const { return (m_polygons_to_extrude == nullptr) ? layer->polygons : *m_polygons_to_extrude; }

    bool could_merge(const SupportGeneratorLayerExtruded &other) const {
        return ! this->empty() && ! other.empty() &&
            std::abs(this->layer->height - other.layer->height) < EPSILON &&
            this->layer->bridging == other.layer->bridging;
    }

    // Merge regions, perform boolean union over the merged polygons.
    void merge(SupportGeneratorLayerExtruded &&other) {
        assert(this->could_merge(other));
        // 1) Merge the rest polygons to extrude, if there are any.
        if (other.m_polygons_to_extrude != nullptr) {
            if (m_polygons_to_extrude == nullptr) {
                // This layer has no extrusions generated yet, if it has no m_polygons_to_extrude (its area to extrude was not reduced yet).
                assert(this->extrusions.empty());
                m_polygons_to_extrude = std::make_unique<Polygons>(this->layer->polygons);
            }
            Slic3r::polygons_append(*m_polygons_to_extrude, std::move(*other.m_polygons_to_extrude));
            *m_polygons_to_extrude = union_safety_offset(*m_polygons_to_extrude);
            other.m_polygons_to_extrude.reset();
        } else if (m_polygons_to_extrude != nullptr) {
            assert(other.m_polygons_to_extrude == nullptr);
            // The other layer has no extrusions generated yet, if it has no m_polygons_to_extrude (its area to extrude was not reduced yet).
            assert(other.extrusions.empty());
            Slic3r::polygons_append(*m_polygons_to_extrude, other.layer->polygons);
            *m_polygons_to_extrude = union_safety_offset(*m_polygons_to_extrude);
        }
        // 2) Merge the extrusions.
        this->extrusions.insert(this->extrusions.end(), other.extrusions.begin(), other.extrusions.end());
        other.extrusions.clear();
        // 3) Merge the infill polygons.
        Slic3r::polygons_append(this->layer->polygons, std::move(other.layer->polygons));
        this->layer->polygons = union_safety_offset(this->layer->polygons);
        other.layer->polygons.clear();
    }

    void polygons_append(Polygons &dst) const {
        if (layer != NULL && ! layer->polygons.empty())
            Slic3r::polygons_append(dst, layer->polygons);
    }

    // The source layer. It carries the height and extrusion type (bridging / non bridging, extrusion height).
    SupportGeneratorLayer  *layer { nullptr };
    // Collect extrusions. They will be exported sorted by the bottom height.
    ExtrusionEntitiesPtr                  extrusions;

private:
    // In case the extrusions are non-empty, m_polygons_to_extrude may contain the rest areas yet to be filled by additional support.
    // This is useful mainly for the loop interfaces, which are generated before the zig-zag infills.
    std::unique_ptr<Polygons>             m_polygons_to_extrude;
};

typedef std::vector<SupportGeneratorLayerExtruded*> SupportGeneratorLayerExtrudedPtrs;

struct LoopInterfaceProcessor
{
    LoopInterfaceProcessor(coordf_t circle_r) :
        n_contact_loops(0),
        circle_radius(circle_r),
        circle_distance(circle_r * 3.)
    {
        // Shape of the top contact area.
        circle.points.reserve(6);
        for (size_t i = 0; i < 6; ++ i) {
            double angle = double(i) * M_PI / 3.;
            circle.points.push_back(Point(circle_radius * cos(angle), circle_radius * sin(angle)));
        }
    }

    // Generate loop contacts at the top_contact_layer,
    // trim the top_contact_layer->polygons with the areas covered by the loops.
    void generate(SupportGeneratorLayerExtruded &top_contact_layer, const Flow &interface_flow_src) const;

    int         n_contact_loops;
    coordf_t    circle_radius;
    coordf_t    circle_distance;
    Polygon     circle;
};

void LoopInterfaceProcessor::generate(SupportGeneratorLayerExtruded &top_contact_layer, const Flow &interface_flow_src) const
{
    if (n_contact_loops == 0 || top_contact_layer.empty())
        return;

    Flow flow = interface_flow_src.with_height(top_contact_layer.layer->height);

    Polygons overhang_polygons;
    if (top_contact_layer.layer->overhang_polygons != nullptr)
        overhang_polygons = std::move(*top_contact_layer.layer->overhang_polygons);

    // Generate the outermost loop.
    // Find centerline of the external loop (or any other kind of extrusions should the loop be skipped)
    ExPolygons top_contact_expolygons = offset_ex(union_ex(top_contact_layer.layer->polygons), - 0.5f * flow.scaled_width());

    // Grid size and bit shifts for quick and exact to/from grid coordinates manipulation.
    coord_t circle_grid_resolution = 1;
    coord_t circle_grid_powerof2 = 0;
    {
        // epsilon to account for rounding errors
        coord_t circle_grid_resolution_non_powerof2 = coord_t(2. * circle_distance + 3.);
        while (circle_grid_resolution < circle_grid_resolution_non_powerof2) {
            circle_grid_resolution <<= 1;
            ++ circle_grid_powerof2;
        }
    }

    struct PointAccessor {
        const Point* operator()(const Point &pt) const { return &pt; }
    };
    typedef ClosestPointInRadiusLookup<Point, PointAccessor> ClosestPointLookupType;

    Polygons loops0;
    {
        // find centerline of the external loop of the contours
        // Only consider the loops facing the overhang.
        Polygons external_loops;
        // Holes in the external loops.
        Polygons circles;
        Polygons overhang_with_margin = offset(union_ex(overhang_polygons), 0.5f * flow.scaled_width());
        for (ExPolygons::iterator it_contact_expoly = top_contact_expolygons.begin(); it_contact_expoly != top_contact_expolygons.end(); ++ it_contact_expoly) {
            // Store the circle centers placed for an expolygon into a regular grid, hashed by the circle centers.
            ClosestPointLookupType circle_centers_lookup(coord_t(circle_distance - SCALED_EPSILON));
            Points circle_centers;
            Point  center_last;
            // For each contour of the expolygon, start with the outer contour, continue with the holes.
            for (size_t i_contour = 0; i_contour <= it_contact_expoly->holes.size(); ++ i_contour) {
                Polygon     &contour = (i_contour == 0) ? it_contact_expoly->contour : it_contact_expoly->holes[i_contour - 1];
                const Point *seg_current_pt = nullptr;
                coordf_t     seg_current_t  = 0.;
                if (! intersection_pl(contour.split_at_first_point(), overhang_with_margin).empty()) {
                    // The contour is below the overhang at least to some extent.
                    //FIXME ideally one would place the circles below the overhang only.
                    // Walk around the contour and place circles so their centers are not closer than circle_distance from each other.
                    if (circle_centers.empty()) {
                        // Place the first circle.
                        seg_current_pt = &contour.points.front();
                        seg_current_t  = 0.;
                        center_last    = *seg_current_pt;
                        circle_centers_lookup.insert(center_last);
                        circle_centers.push_back(center_last);
                    }
                    for (Points::const_iterator it = contour.points.begin() + 1; it != contour.points.end(); ++it) {
                        // Is it possible to place a circle on this segment? Is it not too close to any of the circles already placed on this contour?
                        const Point &p1 = *(it-1);
                        const Point &p2 = *it;
                        // Intersection of a ray (p1, p2) with a circle placed at center_last, with radius of circle_distance.
                        const Vec2d v_seg(coordf_t(p2(0)) - coordf_t(p1(0)), coordf_t(p2(1)) - coordf_t(p1(1)));
                        const Vec2d v_cntr(coordf_t(p1(0) - center_last(0)), coordf_t(p1(1) - center_last(1)));
                        coordf_t a = v_seg.squaredNorm();
                        coordf_t b = 2. * v_seg.dot(v_cntr);
                        coordf_t c = v_cntr.squaredNorm() - circle_distance * circle_distance;
                        coordf_t disc = b * b - 4. * a * c;
                        if (disc > 0.) {
                            // The circle intersects a ray. Avoid the parts of the segment inside the circle.
                            coordf_t t1 = (-b - sqrt(disc)) / (2. * a);
                            coordf_t t2 = (-b + sqrt(disc)) / (2. * a);
                            coordf_t t0 = (seg_current_pt == &p1) ? seg_current_t : 0.;
                            // Take the lowest t in <t0, 1.>, excluding <t1, t2>.
                            coordf_t t;
                            if (t0 <= t1)
                                t = t0;
                            else if (t2 <= 1.)
                                t = t2;
                            else {
                                // Try the following segment.
                                seg_current_pt = nullptr;
                                continue;
                            }
                            seg_current_pt = &p1;
                            seg_current_t  = t;
                            center_last    = Point(p1(0) + coord_t(v_seg(0) * t), p1(1) + coord_t(v_seg(1) * t));
                            // It has been verified that the new point is far enough from center_last.
                            // Ensure, that it is far enough from all the centers.
                            std::pair<const Point*, coordf_t> circle_closest = circle_centers_lookup.find(center_last);
                            if (circle_closest.first != nullptr) {
                                -- it;
                                continue;
                            }
                        } else {
                            // All of the segment is outside the circle. Take the first point.
                            seg_current_pt = &p1;
                            seg_current_t  = 0.;
                            center_last    = p1;
                        }
                        // Place the first circle.
                        circle_centers_lookup.insert(center_last);
                        circle_centers.push_back(center_last);
                    }
                    external_loops.push_back(std::move(contour));
                    for (const Point &center : circle_centers) {
                        circles.push_back(circle);
                        circles.back().translate(center);
                    }
                }
            }
        }
        // Apply a pattern to the external loops.
        loops0 = diff(external_loops, circles);
    }

    Polylines loop_lines;
    {
        // make more loops
        Polygons loop_polygons = loops0;
        for (int i = 1; i < n_contact_loops; ++ i)
            polygons_append(loop_polygons,
                opening(
                    loops0,
                    i * flow.scaled_spacing() + 0.5f * flow.scaled_spacing(),
                    0.5f * flow.scaled_spacing()));
        // Clip such loops to the side oriented towards the object.
        // Collect split points, so they will be recognized after the clipping.
        // At the split points the clipped pieces will be stitched back together.
        loop_lines.reserve(loop_polygons.size());
        std::unordered_map<Point, int, PointHash> map_split_points;
        for (Polygons::const_iterator it = loop_polygons.begin(); it != loop_polygons.end(); ++ it) {
            assert(map_split_points.find(it->first_point()) == map_split_points.end());
            map_split_points[it->first_point()] = -1;
            loop_lines.push_back(it->split_at_first_point());
        }
        loop_lines = intersection_pl(loop_lines, expand(overhang_polygons, scale_(SUPPORT_MATERIAL_MARGIN)));
        // Because a closed loop has been split to a line, loop_lines may contain continuous segments split to 2 pieces.
        // Try to connect them.
        for (int i_line = 0; i_line < int(loop_lines.size()); ++ i_line) {
            Polyline &polyline = loop_lines[i_line];
            auto it = map_split_points.find(polyline.first_point());
            if (it != map_split_points.end()) {
                // This is a stitching point.
                // If this assert triggers, multiple source polygons likely intersected at this point.
                assert(it->second != -2);
                if (it->second < 0) {
                    // First occurence.
                    it->second = i_line;
                } else {
                    // Second occurence. Join the lines.
                    Polyline &polyline_1st = loop_lines[it->second];
                    assert(polyline_1st.first_point() == it->first || polyline_1st.last_point() == it->first);
                    if (polyline_1st.first_point() == it->first)
                        polyline_1st.reverse();
                    polyline_1st.append(std::move(polyline));
                    it->second = -2;
                }
                continue;
            }
            it = map_split_points.find(polyline.last_point());
            if (it != map_split_points.end()) {
                // This is a stitching point.
                // If this assert triggers, multiple source polygons likely intersected at this point.
                assert(it->second != -2);
                if (it->second < 0) {
                    // First occurence.
                    it->second = i_line;
                } else {
                    // Second occurence. Join the lines.
                    Polyline &polyline_1st = loop_lines[it->second];
                    assert(polyline_1st.first_point() == it->first || polyline_1st.last_point() == it->first);
                    if (polyline_1st.first_point() == it->first)
                        polyline_1st.reverse();
                    polyline.reverse();
                    polyline_1st.append(std::move(polyline));
                    it->second = -2;
                }
            }
        }
        // Remove empty lines.
        remove_degenerate(loop_lines);
    }

    // add the contact infill area to the interface area
    // note that growing loops by $circle_radius ensures no tiny
    // extrusions are left inside the circles; however it creates
    // a very large gap between loops and contact_infill_polygons, so maybe another
    // solution should be found to achieve both goals
    // Store the trimmed polygons into a separate polygon set, so the original infill area remains intact for
    // "modulate by layer thickness".
    top_contact_layer.set_polygons_to_extrude(diff(top_contact_layer.layer->polygons, offset(loop_lines, float(circle_radius * 1.1))));

    // Transform loops into ExtrusionPath objects.
    extrusion_entities_append_paths(
        top_contact_layer.extrusions,
        std::move(loop_lines),
        ExtrusionRole::erSupportMaterialInterface, flow.mm3_per_mm(), flow.width(), flow.height());
}

#ifdef SLIC3R_DEBUG
static std::string dbg_index_to_color(int idx)
{
    if (idx < 0)
        return "yellow";
    idx = idx % 3;
    switch (idx) {
        case 0: return "red";
        case 1: return "green";
        default: return "blue";
    }
}
#endif /* SLIC3R_DEBUG */

// When extruding a bottom interface layer over an object, the bottom interface layer is extruded in a thin air, therefore
// it is being extruded with a bridging flow to not shrink excessively (the die swell effect).
// Tiny extrusions are better avoided and it is always better to anchor the thread to an existing support structure if possible.
// Therefore the bottom interface spots are expanded a bit. The expanded regions may overlap with another bottom interface layers,
// leading to over extrusion, where they overlap. The over extrusion is better avoided as it often makes the interface layers
// to stick too firmly to the object.
//
// Modulate thickness (increase bottom_z) of extrusions_in_out generated for this_layer
// if they overlap with overlapping_layers, whose print_z is above this_layer.bottom_z() and below this_layer.print_z.
static void modulate_extrusion_by_overlapping_layers(
    // Extrusions generated for this_layer.
    ExtrusionEntitiesPtr                               &extrusions_in_out,
    const SupportGeneratorLayer          &this_layer,
    // Multiple layers overlapping with this_layer, sorted bottom up.
    const SupportGeneratorLayersPtr      &overlapping_layers)
{
    size_t n_overlapping_layers = overlapping_layers.size();
    if (n_overlapping_layers == 0 || extrusions_in_out.empty())
        // The extrusions do not overlap with any other extrusion.
        return;
    // Only plain paths can be split by height. Tree and sheathed support write loops and collections,
    // which a coarse support band can bring here when another island's contact sits inside the band.
    // Leave those at their own height.
    if (std::any_of(extrusions_in_out.begin(), extrusions_in_out.end(),
                    [](const ExtrusionEntity *entity) { return dynamic_cast<const ExtrusionPath*>(entity) == nullptr; }))
        return;

    // Get the initial extrusion parameters.
    ExtrusionPath *extrusion_path_template = dynamic_cast<ExtrusionPath*>(extrusions_in_out.front());
    assert(extrusion_path_template != nullptr);
    ExtrusionRole extrusion_role  = extrusion_path_template->role();
    float         extrusion_width = extrusion_path_template->width;

    struct ExtrusionPathFragment
    {
        ExtrusionPathFragment() : mm3_per_mm(-1), width(-1), height(-1) {};
        ExtrusionPathFragment(double mm3_per_mm, float width, float height) : mm3_per_mm(mm3_per_mm), width(width), height(height) {};

        Polylines       polylines;
        double          mm3_per_mm;
        float           width;
        float           height;
    };

    // Split the extrusions by the overlapping layers, reduce their extrusion rate.
    // The last path_fragment is from this_layer.
    std::vector<ExtrusionPathFragment> path_fragments(
        n_overlapping_layers + 1,
        ExtrusionPathFragment(extrusion_path_template->mm3_per_mm, extrusion_path_template->width, extrusion_path_template->height));
    // Don't use it, it will be released.
    extrusion_path_template = nullptr;

#ifdef SLIC3R_DEBUG
    static int iRun = 0;
    ++ iRun;
    BoundingBox bbox;
    for (size_t i_overlapping_layer = 0; i_overlapping_layer < n_overlapping_layers; ++ i_overlapping_layer) {
        const SupportGeneratorLayer &overlapping_layer = *overlapping_layers[i_overlapping_layer];
        bbox.merge(get_extents(overlapping_layer.polygons));
    }
    for (ExtrusionEntitiesPtr::const_iterator it = extrusions_in_out.begin(); it != extrusions_in_out.end(); ++ it) {
        ExtrusionPath *path = dynamic_cast<ExtrusionPath*>(*it);
        assert(path != nullptr);
        bbox.merge(get_extents(path->polyline));
    }
    SVG svg(debug_out_path("support-fragments-%d-%lf.svg", iRun, this_layer.print_z).c_str(), bbox);
    const float transparency = 0.5f;
    // Filled polygons for the overlapping regions.
    svg.draw(union_ex(this_layer.polygons), dbg_index_to_color(-1), transparency);
    for (size_t i_overlapping_layer = 0; i_overlapping_layer < n_overlapping_layers; ++ i_overlapping_layer) {
        const SupportGeneratorLayer &overlapping_layer = *overlapping_layers[i_overlapping_layer];
        svg.draw(union_ex(overlapping_layer.polygons), dbg_index_to_color(int(i_overlapping_layer)), transparency);
    }
    // Contours of the overlapping regions.
    svg.draw(to_polylines(this_layer.polygons), dbg_index_to_color(-1), scale_(0.2));
    for (size_t i_overlapping_layer = 0; i_overlapping_layer < n_overlapping_layers; ++ i_overlapping_layer) {
        const SupportGeneratorLayer &overlapping_layer = *overlapping_layers[i_overlapping_layer];
        svg.draw(to_polylines(overlapping_layer.polygons), dbg_index_to_color(int(i_overlapping_layer)), scale_(0.1));
    }
    // Fill extrusion, the source.
    for (ExtrusionEntitiesPtr::const_iterator it = extrusions_in_out.begin(); it != extrusions_in_out.end(); ++ it) {
        ExtrusionPath *path = dynamic_cast<ExtrusionPath*>(*it);
        std::string color_name;
        switch ((it - extrusions_in_out.begin()) % 9) {
            case 0: color_name = "magenta"; break;
            case 1: color_name = "deepskyblue"; break;
            case 2: color_name = "coral"; break;
            case 3: color_name = "goldenrod"; break;
            case 4: color_name = "orange"; break;
            case 5: color_name = "olivedrab"; break;
            case 6: color_name = "blueviolet"; break;
            case 7: color_name = "brown"; break;
            default: color_name = "orchid"; break;
        }
        svg.draw(path->polyline, color_name, scale_(0.2));
    }
#endif /* SLIC3R_DEBUG */

    // End points of the original paths.
    std::vector<std::pair<Point, Point>> path_ends;
    // Collect the paths of this_layer.
    {
        Polylines &polylines = path_fragments.back().polylines;
        for (ExtrusionEntity *ee : extrusions_in_out) {
            ExtrusionPath *path = dynamic_cast<ExtrusionPath*>(ee);
            assert(path != nullptr);
            polylines.emplace_back(path->polyline.to_polyline());
            path_ends.emplace_back(std::pair<Point, Point>(polylines.back().points.front(), polylines.back().points.back()));
            delete path;
        }
    }
    // Destroy the original extrusion paths, their polylines were moved to path_fragments already.
    // This will be the destination for the new paths.
    extrusions_in_out.clear();

    // Fragment the path segments by overlapping layers. The overlapping layers are sorted by an increasing print_z.
    // Trim by the highest overlapping layer first.
    for (int i_overlapping_layer = int(n_overlapping_layers) - 1; i_overlapping_layer >= 0; -- i_overlapping_layer) {
        const SupportGeneratorLayer &overlapping_layer = *overlapping_layers[i_overlapping_layer];
        ExtrusionPathFragment &frag = path_fragments[i_overlapping_layer];
        Polygons polygons_trimming = offset(union_ex(overlapping_layer.polygons), float(scale_(0.5*extrusion_width)));
        frag.polylines = intersection_pl(path_fragments.back().polylines, polygons_trimming);
        path_fragments.back().polylines = diff_pl(path_fragments.back().polylines, polygons_trimming);
        // Adjust the extrusion parameters for a reduced layer height and a non-bridging flow (nozzle_dmr = -1, does not matter).
        assert(this_layer.print_z > overlapping_layer.print_z);
        frag.height = float(this_layer.print_z - overlapping_layer.print_z);
        frag.mm3_per_mm = Flow(frag.width, frag.height, -1.f).mm3_per_mm();
#ifdef SLIC3R_DEBUG
        svg.draw(frag.polylines, dbg_index_to_color(i_overlapping_layer), scale_(0.1));
#endif /* SLIC3R_DEBUG */
    }

#ifdef SLIC3R_DEBUG
    svg.draw(path_fragments.back().polylines, dbg_index_to_color(-1), scale_(0.1));
    svg.Close();
#endif /* SLIC3R_DEBUG */

    // Now chain the split segments using hashing and a nearly exact match, maintaining the order of segments.
    // Create a single ExtrusionPath or ExtrusionEntityCollection per source ExtrusionPath.
    // Map of fragment start/end points to a pair of <i_overlapping_layer, i_polyline_in_layer>
    // Because a non-exact matching is used for the end points, a multi-map is used.
    // As the clipper library may reverse the order of some clipped paths, store both ends into the map.
    struct ExtrusionPathFragmentEnd
    {
        ExtrusionPathFragmentEnd(size_t alayer_idx, size_t apolyline_idx, bool ais_start) :
            layer_idx(alayer_idx), polyline_idx(apolyline_idx), is_start(ais_start) {}
        size_t layer_idx;
        size_t polyline_idx;
        bool   is_start;
    };
    class ExtrusionPathFragmentEndPointAccessor {
    public:
        ExtrusionPathFragmentEndPointAccessor(const std::vector<ExtrusionPathFragment> &path_fragments) : m_path_fragments(path_fragments) {}
        // Return an end point of a fragment, or nullptr if the fragment has been consumed already.
        const Point* operator()(const ExtrusionPathFragmentEnd &fragment_end) const {
            const Polyline &polyline = m_path_fragments[fragment_end.layer_idx].polylines[fragment_end.polyline_idx];
            return polyline.points.empty() ? nullptr :
                (fragment_end.is_start ? &polyline.points.front() : &polyline.points.back());
        }
    private:
        ExtrusionPathFragmentEndPointAccessor& operator=(const ExtrusionPathFragmentEndPointAccessor&) {
            return *this;
        }

        const std::vector<ExtrusionPathFragment> &m_path_fragments;
    };
    const coord_t search_radius = 7;
    ClosestPointInRadiusLookup<ExtrusionPathFragmentEnd, ExtrusionPathFragmentEndPointAccessor> map_fragment_starts(
        search_radius, ExtrusionPathFragmentEndPointAccessor(path_fragments));
    for (size_t i_overlapping_layer = 0; i_overlapping_layer <= n_overlapping_layers; ++ i_overlapping_layer) {
        const Polylines &polylines = path_fragments[i_overlapping_layer].polylines;
        for (size_t i_polyline = 0; i_polyline < polylines.size(); ++ i_polyline) {
            // Map a starting point of a polyline to a pair of <layer, polyline>
            if (polylines[i_polyline].points.size() >= 2) {
                map_fragment_starts.insert(ExtrusionPathFragmentEnd(i_overlapping_layer, i_polyline, true));
                map_fragment_starts.insert(ExtrusionPathFragmentEnd(i_overlapping_layer, i_polyline, false));
            }
        }
    }

    // For each source path:
    for (size_t i_path = 0; i_path < path_ends.size(); ++ i_path) {
        const Point &pt_start = path_ends[i_path].first;
        const Point &pt_end   = path_ends[i_path].second;
        Point pt_current = pt_start;
        // Find a chain of fragments with the original / reduced print height.
        ExtrusionMultiPath multipath;
        for (;;) {
            // Find a closest end point to pt_current.
            std::pair<const ExtrusionPathFragmentEnd*, coordf_t> end_and_dist2 = map_fragment_starts.find(pt_current);
            // There may be a bug in Clipper flipping the order of two last points in a fragment?
            // assert(end_and_dist2.first != nullptr);
            assert(end_and_dist2.first == nullptr || end_and_dist2.second < search_radius * search_radius);
            if (end_and_dist2.first == nullptr) {
                // New fragment connecting to pt_current was not found.
                // Verify that the last point found is close to the original end point of the unfragmented path.
                //const double d2 = (pt_end - pt_current).cast<double>.squaredNorm();
                //assert(d2 < coordf_t(search_radius * search_radius));
                // End of the path.
                break;
            }
            const ExtrusionPathFragmentEnd &fragment_end_min = *end_and_dist2.first;
            // Fragment to consume.
            ExtrusionPathFragment &frag = path_fragments[fragment_end_min.layer_idx];
            Polyline              &frag_polyline = frag.polylines[fragment_end_min.polyline_idx];
            // Path to append the fragment to.
            ExtrusionPath         *path = multipath.paths.empty() ? nullptr : &multipath.paths.back();
            if (path != nullptr) {
                // Verify whether the path is compatible with the current fragment.
                assert(this_layer.layer_type == SupporLayerType::BottomContact || path->height != frag.height || path->mm3_per_mm != frag.mm3_per_mm);
                if (path->height != frag.height || path->mm3_per_mm != frag.mm3_per_mm) {
                    path = nullptr;
                }
                // Merging with the previous path. This can only happen if the current layer was reduced by a base layer, which was split into a base and interface layer.
            }
            if (path == nullptr) {
                // Allocate a new path.
                multipath.paths.push_back(ExtrusionPath(extrusion_role, frag.mm3_per_mm, frag.width, frag.height));
                path = &multipath.paths.back();
            }
            // The Clipper library may flip the order of the clipped polylines arbitrarily.
            // Reverse the source polyline, if connecting to the end.
            if (! fragment_end_min.is_start)
                frag_polyline.reverse();
            // Enforce exact overlap of the end points of successive fragments.
            assert(frag_polyline.points.front() == pt_current);
            frag_polyline.points.front() = pt_current;
            // Don't repeat the first point.
            if (! path->polyline.points.empty())
                path->polyline.points.pop_back();
            // Consume the fragment's polyline, remove it from the input fragments, so it will be ignored the next time.
            path->polyline.append(Polyline3(std::move(frag_polyline)));
            frag_polyline.points.clear();
            const Point3 &pt_back3 = path->polyline.points.back();
            pt_current = Point(pt_back3.x(), pt_back3.y());
            if (pt_current == pt_end) {
                // End of the path.
                break;
            }
        }
        if (!multipath.paths.empty()) {
            if (multipath.paths.size() == 1) {
                // This path was not fragmented.
                extrusions_in_out.push_back(new ExtrusionPath(std::move(multipath.paths.front())));
            } else {
                // This path was fragmented. Copy the collection as a whole object, so the order inside the collection will not be changed
                // during the chaining of extrusions_in_out.
                extrusions_in_out.push_back(new ExtrusionMultiPath(std::move(multipath)));
            }
        }
    }
    // If there are any non-consumed fragments, add them separately.
    //FIXME this shall not happen, if the Clipper works as expected and all paths split to fragments could be re-connected.
    for (auto it_fragment = path_fragments.begin(); it_fragment != path_fragments.end(); ++ it_fragment)
        extrusion_entities_append_paths(extrusions_in_out, std::move(it_fragment->polylines), extrusion_role, it_fragment->mm3_per_mm, it_fragment->width, it_fragment->height);
}

// How organic tree support on a coarse base is laid in bands, for one run of base layers that stack
// without a gap. The tree is planned and sliced on the object's layers; each slice is a cut through
// the branches' tubes at that layer's middle.
struct TreeBandRules {
    std::function<bool(coordf_t)>                     legal;
    coordf_t                                          body_min = 0.;
    coordf_t                                          body_max = 0.;
    coordf_t                                          target   = 0.;
    // The fine nozzle has a filament of the base's material for the bed layer. A tree's bed layer then always goes
    // to it: roots on the bed are often thinner than a coarse road, and one nozzle lays the support's first layer.
    bool                                              bed_layer_on_interface_nozzle = false;
    // The part, grown by the XY distance, and the contacts and interfaces, grown by a road's width,
    // anywhere inside a band's Z span.
    std::function<Polygons(coordf_t, coordf_t)>       part_between;
    std::function<Polygons(coordf_t, coordf_t)>       interface_areas_between;
    // Half the sideways move a branch makes per mm of height at the steepest branch angle, and its cap.
    double                                            lean_per_mm = 0.;
    float                                             lean_cap    = 0.;
    // Parts of a band road narrower than twice this go to the interface nozzle.
    float                                             thin_radius = 0.;
    // Pieces narrower than twice this are too thin for the interface nozzle's road.
    float                                             sliver      = 0.;
};

// A band is one coarse layer, so it takes the cut nearest its own middle, as stock does at that layer
// height, and keeps only what also stands on the band's bottom slice and holds up its top slice,
// within the distance a branch leans over the band. It keeps clear of the part and of every contact
// and interface over its whole height, and leaves anything thinner than the coarse road can draw.
// What no band takes is laid by the interface nozzle at its own layer's height, or, where enough of
// it stacks up, in a smaller band of its own. It never joins a band road.
static void band_tree_run(std::vector<SupportGeneratorLayer*> &run, const TreeBandRules &rules, size_t &num_bands, size_t &num_fine_layers)
{
    // A morphological opening that stays inside its input: the mitred dilation of an opening can poke out
    // past a rounded outline.
    const auto open_inside = [](const Polygons &polygons, float radius) { return intersection(opening(polygons, radius), polygons); };
    const size_t n = run.size();
    std::vector<coordf_t> height(n);
    // What no band has taken yet, per layer.
    std::vector<Polygons> left(n);
    for (size_t k = 0; k < n; ++ k) {
        height[k] = run[k]->height;
        left[k]   = union_(run[k]->polygons);
        run[k]->polygons.clear();
    }
    std::vector<bool> hosts_band(n, false);

    const auto make_band = [&](size_t first, size_t last) {
        if (left[first].empty() || left[last].empty())
            return;
        const coordf_t bottom_z = run[first]->bottom_z;
        const coordf_t top_z    = run[last]->print_z;
        const coordf_t mid      = 0.5 * (bottom_z + top_z);
        const auto     mid_of   = [&](size_t k) { return run[k]->print_z - 0.5 * height[k]; };
        size_t cut = first;
        for (size_t k = first + 1; k <= last; ++ k)
            if (std::abs(mid_of(k) - mid) < std::abs(mid_of(cut) - mid))
                cut = k;
        const float lean = std::min(rules.lean_cap, float(scale_((top_z - bottom_z) * rules.lean_per_mm)) + float(SCALED_EPSILON));
        Polygons road = intersection(left[cut], expand(left[first], lean));
        if (! road.empty())
            road = intersection(road, expand(left[last], lean));
        if (! road.empty())
            if (Polygons part = rules.part_between(bottom_z, top_z); ! part.empty())
                road = diff(road, part);
        if (! road.empty())
            if (Polygons keepout = rules.interface_areas_between(bottom_z, top_z); ! keepout.empty())
                road = diff(road, keepout);
        if (! road.empty())
            road = open_inside(road, rules.thin_radius);
        if (road.empty())
            return;
        const Polygons taken = expand(road, lean);
        for (size_t k = first; k <= last; ++ k)
            left[k] = diff(left[k], taken);
        run[last]->polygons = std::move(road);
        run[last]->bottom_z = bottom_z;
        run[last]->height   = top_z - bottom_z;
        hosts_band[last]    = true;
        ++ num_bands;
    };

    for (size_t i = 0; i < n;) {
        if (run[i]->bottom_z < EPSILON && rules.bed_layer_on_interface_nozzle) {
            ++ i;
            continue;
        }
        // Band heights as for normal support: the smallest legal band on the bed, above it as close to
        // the coarse cadence as whole layers get.
        size_t   j   = i;
        coordf_t sum = height[i];
        if (run[i]->bottom_z < EPSILON) {
            while (sum < rules.body_min - EPSILON && j + 1 < n && sum + height[j + 1] < rules.body_max + EPSILON)
                sum += height[++ j];
        } else {
            while (j + 1 < n && sum + height[j + 1] < rules.target + EPSILON)
                sum += height[++ j];
        }
        if (j > i && rules.legal(sum))
            make_band(i, j);
        // What that band left, where enough of it stacks up, in the smallest bands the nozzle can lay.
        // The band's own top layer is taken.
        const size_t end_free = hosts_band[j] ? j : j + 1;
        for (size_t k = i; k < end_free;) {
            size_t   m = k;
            coordf_t s = height[k];
            while (s < rules.body_min - EPSILON && m + 1 < end_free)
                s += height[++ m];
            if (m > k && rules.legal(s))
                make_band(k, m);
            k = m + 1;
        }
        i = j + 1;
    }

    for (size_t k = 0; k < n; ++ k) {
        SupportGeneratorLayer &layer = *run[k];
        Polygons fine = std::move(left[k]);
        // A layer the coarse nozzle can lay at its own height keeps what is wide enough for its road.
        if (! hosts_band[k] && rules.legal(height[k]) && ! fine.empty() &&
            ! (layer.bottom_z < EPSILON && rules.bed_layer_on_interface_nozzle)) {
            Polygons road = open_inside(fine, rules.thin_radius);
            if (! road.empty()) {
                fine           = diff(fine, road);
                layer.polygons = std::move(road);
            }
        }
        if (! fine.empty())
            fine = open_inside(fine, rules.sliver);
        if (! fine.empty()) {
            layer.fine_body_polygons = std::move(fine);
            layer.fine_body_height   = height[k];
            ++ num_fine_layers;
        }
    }
}

size_t mixed_nozzle_thin_coarse_body_to_fine(SupportGeneratorLayersPtr &base_layers, coordf_t body_min, const SlicingParameters &slicing_params)
{
    size_t moved = 0;
    for (SupportGeneratorLayer *layer : base_layers)
        if (layer != nullptr && ! layer->polygons.empty() && layer->bottom_z > EPSILON && layer->height < body_min - EPSILON &&
            ! (slicing_params.has_raft() && layer->print_z < slicing_params.raft_contact_top_z + EPSILON)) {
            // Only a layer that hosts no band can be this thin, and its fine body already has its own height.
            polygons_append(layer->fine_body_polygons, std::move(layer->polygons));
            layer->fine_body_polygons = union_(layer->fine_body_polygons);
            layer->fine_body_height   = layer->height;
            layer->polygons.clear();
            ++ moved;
        }
    return moved;
}

void mixed_nozzle_band_support_body(
    const PrintObject               &object,
    const SupportParameters         &support_params,
    const SlicingParameters         &slicing_params,
    const SupportGeneratorLayersPtr &bottom_contacts,
    const SupportGeneratorLayersPtr &top_contacts,
    SupportGeneratorLayersPtr       &interface_layers,
    const SupportGeneratorLayersPtr &base_interface_layers,
    SupportGeneratorLayersPtr       &base_layers)
{
    if (! support_params.mixed_nozzle_banded_body || base_layers.empty())
        return;

    const PrintConfig       &print_config  = object.print()->config();
    const PrintObjectConfig &object_config = object.config();
    const int body_idx = resolved_support_filament_nozzle_idx(print_config, object_config.support_filament.value);
    if (body_idx <= 0 || size_t(body_idx) > print_config.nozzle_diameter.values.size())
        return;
    const size_t   body_tool = size_t(body_idx - 1);
    const coordf_t body_min  = resolved_min_layer_height(print_config, body_tool);
    const coordf_t body_max  = resolved_max_layer_height(print_config, body_tool);
    if (! std::isfinite(body_min) || ! std::isfinite(body_max) || body_min <= 0. || body_max < body_min)
        return;

    // The band height aimed for is the plan's coarse cadence. Under Feature Split that is the
    // support cap Slicing.cpp already resolved from the coarse cadence. Under Body Split the
    // coarse height belongs to each body, so take the cadence of the bodies the support nozzle
    // lays, and fall back to the same cap when it lays none of them.
    coordf_t target = slicing_params.max_suport_layer_height;
    if (is_mixed_nozzle_body_split(print_config)) {
        const std::vector<size_t> &grid_tools = object.region_grid_physical_extruders();
        coordf_t body_cadence = 0.;
        for (size_t region_id = 0; region_id < object.num_printing_regions() && region_id < grid_tools.size(); ++ region_id)
            if (grid_tools[region_id] == body_tool)
                body_cadence = std::max(body_cadence, coordf_t(object.printing_region(region_id).config().regional_layer_height.value));
        if (body_cadence > 0.)
            target = body_cadence;
    }
    if (! std::isfinite(target) || target <= 0.)
        target = body_max;
    target = std::clamp(target, body_min, body_max);

    const auto legal = [body_min, body_max](coordf_t height) {
        return height > body_min - EPSILON && height < body_max + EPSILON;
    };
    // A bed layer thinner than the coarse nozzle's minimum stays on the bed at the first layer's height
    // and goes to the interface nozzle in a filament of the base's material, the way the part's own
    // first layer and the tower's bed level are laid, so the support's first layer prints on layer 1 as
    // stock prints it. Only when that nozzle has no such filament does the first band start on the bed.
    const std::optional<unsigned int> bed_filament = slicing_params.has_raft() || object_config.support_filament.value <= 0 ?
        std::nullopt : mixed_nozzle_interface_nozzle_body_filament(print_config, object_config, object.object_extruders());
    const bool bed_layer_on_interface_nozzle = bed_filament &&
        print_config.filament_type.get_at(*bed_filament) == print_config.filament_type.get_at(object_config.support_filament.value - 1);

    // A road joining a band is laid through the band's whole height, so it keeps a flow's width away
    // from the contacts and interfaces inside the band.
    const float clearance = float(support_params.support_material_flow.scaled_width());

    // A band road may not run through a top contact that sits inside the band's Z span, nor come within
    // a flow's width of it: generate_support_toolpaths() trims a road over a contact down to the height
    // left above it, thinner than the band's nozzle can lay. What is cut away stays on the band's own
    // layers, and the pass below hands it to the interface nozzle.
    const auto contacts_between = [&top_contacts, clearance](coordf_t bottom_z, coordf_t top_z) {
        Polygons out;
        for (const SupportGeneratorLayer *contact : top_contacts)
            if (contact != nullptr && contact->print_z > bottom_z + EPSILON && contact->print_z < top_z - EPSILON)
                polygons_append(out, contact->polygons);
        return out.empty() ? out : expand(out, clearance);
    };

    // Contacts and interfaces a band's Z span reaches, with that clearance.
    const auto interface_areas_between = [&](coordf_t bottom_z, coordf_t top_z) {
        Polygons out;
        for (const SupportGeneratorLayersPtr *list : {&bottom_contacts, &top_contacts, const_cast<const SupportGeneratorLayersPtr*>(&interface_layers)})
            for (const SupportGeneratorLayer *layer : *list)
                if (layer != nullptr && layer->print_z > bottom_z + EPSILON && layer->bottom_z < top_z - EPSILON)
                    polygons_append(out, layer->polygons);
        return out.empty() ? out : expand(out, clearance);
    };

    // An interface layer on the bed under a contact is where stock lays its base interface in the base
    // filament. The coarse nozzle cannot lay it at this height, so it joins the first layer's body,
    // which the interface nozzle lays with the part's own filament (see generate_support_toolpaths()).
    if (! slicing_params.has_raft())
        for (SupportGeneratorLayer *base : base_layers)
            if (base != nullptr && base->bottom_z < EPSILON && ! legal(base->height)) {
                for (SupportGeneratorLayer *layer : interface_layers)
                    if (layer != nullptr && layer->bottom_z < EPSILON && std::abs(layer->print_z - base->print_z) < EPSILON &&
                        ! layer->polygons.empty()) {
                        base->polygons = union_(base->polygons, layer->polygons);
                        layer->polygons.clear();
                    }
                break;
            }

    // The dense base-material layer under or over an interface (the base interface) is laid by the
    // interface nozzle in the body filament, so no coarse band may end at its height: the body there goes
    // to the interface nozzle with it.
    std::vector<coordf_t> base_interface_z;
    for (const SupportGeneratorLayer *layer : base_interface_layers)
        if (layer != nullptr && ! layer->polygons.empty())
            base_interface_z.push_back(layer->print_z);

    // Runs of base layers that stack without a gap, above the raft.
    std::vector<std::vector<SupportGeneratorLayer*>> runs;
    for (SupportGeneratorLayer *layer : base_layers) {
        if (layer == nullptr || layer->bridging)
            continue;
        if (slicing_params.has_raft() && layer->print_z < slicing_params.raft_contact_top_z + EPSILON)
            continue;
        if (std::any_of(base_interface_z.begin(), base_interface_z.end(),
                        [layer](coordf_t z) { return std::abs(z - layer->print_z) < EPSILON; })) {
            if (! layer->polygons.empty()) {
                layer->fine_body_polygons = union_(layer->polygons);
                layer->fine_body_height   = layer->height;
                layer->polygons.clear();
            }
            continue;
        }
        if (runs.empty() || std::abs(layer->bottom_z - runs.back().back()->print_z) > EPSILON)
            runs.emplace_back();
        runs.back().push_back(layer);
    }

    size_t num_bands       = 0;
    size_t num_fine_layers = 0;
    const bool organic_tree = support_params.support_style == smsTreeOrganic;
    TreeBandRules tree_rules;
    const auto object_layers = object.layers();
    std::vector<Polygons> part_grown;
    if (organic_tree) {
        // Each object layer's outline, grown by the distance the tree keeps from the part (as TreeSupportSettings
        // has it: the XY distance, or under an overhang half the outer wall's width when that is less), for
        // the band keepout.
        coordf_t external_perimeter_width = 0.;
        for (size_t region_id = 0; region_id < object.num_printing_regions(); ++ region_id)
            external_perimeter_width = std::max<coordf_t>(external_perimeter_width,
                object.printing_region(region_id).flow(object, frExternalPerimeter, object_config.layer_height.value).width());
        coordf_t tree_gap = std::min(support_params.gap_xy, 0.5 * external_perimeter_width);
        if (slicing_params.gap_support_object < EPSILON)
            tree_gap = std::max(tree_gap, 0.1);
        part_grown.assign(object_layers.size(), Polygons());
        const float gap_xy = float(scale_(tree_gap));
        tbb::parallel_for(tbb::blocked_range<size_t>(0, object_layers.size()), [&](const tbb::blocked_range<size_t> &range) {
            for (size_t l = range.begin(); l < range.end(); ++ l)
                part_grown[l] = expand(object_layers[l]->lslices, gap_xy);
        });
        const coordf_t road_width = support_params.support_material_flow.width();
        tree_rules.legal        = legal;
        tree_rules.body_min     = body_min;
        tree_rules.body_max     = body_max;
        tree_rules.target       = target;
        tree_rules.bed_layer_on_interface_nozzle = bed_layer_on_interface_nozzle;
        tree_rules.part_between = [&object_layers, &part_grown](coordf_t bottom_z, coordf_t top_z) {
            auto it = std::lower_bound(object_layers.begin(), object_layers.end(), bottom_z + EPSILON,
                [](const Layer *layer, coordf_t z) { return layer->print_z < z; });
            Polygons out;
            for (; it != object_layers.end() && (*it)->bottom_z() < top_z - EPSILON; ++ it)
                polygons_append(out, part_grown[it - object_layers.begin()]);
            return out;
        };
        tree_rules.interface_areas_between = interface_areas_between;
        tree_rules.lean_per_mm  = 0.5 * std::tan(std::clamp(object_config.tree_support_branch_angle_organic.value, 0., 85.) * M_PI / 180.);
        tree_rules.lean_cap     = float(scale_(0.5 * road_width));
        tree_rules.thin_radius  = float(scale_(0.75 * road_width));
        tree_rules.sliver       = float(scale_(0.5 * support_params.mixed_nozzle_fine_body_flow.width()));
    }
    for (std::vector<SupportGeneratorLayer*> &run : runs) {
        if (organic_tree) {
            band_tree_run(run, tree_rules, num_bands, num_fine_layers);
            continue;
        }
        const size_t n = run.size();
        std::vector<coordf_t> height(n);
        for (size_t k = 0; k < n; ++ k)
            height[k] = run[k]->height;
        std::vector<bool>     hosts_band(n, false);
        std::vector<Polygons> left_on_host(n);
        // The band laid over each layer, by the index of its top layer.
        std::vector<size_t>   host_of(n, n);

        // Lay what layers first..last all share as one road on top of the last one.
        const auto make_band = [&](size_t first, size_t last) {
            Polygons shared = run[first]->polygons;
            for (size_t k = first + 1; k <= last && ! shared.empty(); ++ k)
                shared = intersection(shared, run[k]->polygons);
            if (shared.empty())
                return;
            const coordf_t bottom_z = run[first]->bottom_z;
            const coordf_t top_z    = run[last]->print_z;
            if (Polygons blocked = contacts_between(bottom_z, top_z); ! blocked.empty())
                shared = diff(shared, blocked);
            if (shared.empty())
                return;
            for (size_t k = first; k < last; ++ k)
                run[k]->polygons = diff(run[k]->polygons, shared);
            left_on_host[last]  = diff(run[last]->polygons, shared);
            run[last]->polygons = std::move(shared);
            run[last]->bottom_z = bottom_z;
            run[last]->height   = top_z - bottom_z;
            hosts_band[last]    = true;
            for (size_t k = first; k <= last; ++ k)
                host_of[k] = last;
            ++ num_bands;
        };

        for (size_t i = 0; i < n;) {
            // The bed layer the coarse nozzle cannot lay is left for the interface nozzle below.
            if (run[i]->bottom_z < EPSILON && bed_layer_on_interface_nozzle && ! legal(height[i])) {
                ++ i;
                continue;
            }
            // Where this band ends. On the bed the band is the smallest one the nozzle can lay, so
            // the first road is no thicker than it has to be; above it, as close to the plan's
            // coarse cadence as whole layers get without going over.
            size_t   j   = i;
            coordf_t sum = height[i];
            if (run[i]->bottom_z < EPSILON) {
                while (sum < body_min - EPSILON && j + 1 < n && sum + height[j + 1] < body_max + EPSILON)
                    sum += height[++ j];
            } else {
                while (j + 1 < n && sum + height[j + 1] < target + EPSILON)
                    sum += height[++ j];
            }
            if (j > i && legal(sum))
                make_band(i, j);
            // What that band could not hold, where the shape changes inside it, in the smallest
            // groups the nozzle can lay, from the bottom up. The band's own top layer is taken.
            const size_t last_free = hosts_band[j] ? j - 1 : j;
            for (size_t k = i; k <= last_free;) {
                size_t   m = k;
                coordf_t s = height[k];
                while (s < body_min - EPSILON && m < last_free)
                    s += height[++ m];
                if (m > k && legal(s))
                    make_band(k, m);
                k = m + 1;
            }
            i = j + 1;
        }

        // Whatever is left the coarse nozzle cannot lay at its own layer's height joins the band laid
        // over that layer, so the base filament prints it, like the expanded first layer on the bed or
        // where a tree narrows or moves. The band is taller than the layer, so what would run into a
        // contact or an interface inside the band stays behind and goes to the interface nozzle, at that
        // height. A layer the nozzle can lay as it is stays as it is.
        for (size_t k = 0; k < n; ++ k) {
            SupportGeneratorLayer &layer = *run[k];
            Polygons fine = std::move(left_on_host[k]);
            if (! hosts_band[k] && ! legal(height[k]) && ! layer.polygons.empty()) {
                polygons_append(fine, std::move(layer.polygons));
                layer.polygons.clear();
            }
            if (! fine.empty() && host_of[k] < n) {
                SupportGeneratorLayer &host = *run[host_of[k]];
                Polygons keepout = interface_areas_between(host.bottom_z, host.print_z);
                Polygons joined;
                if (keepout.empty()) {
                    joined = std::move(fine);
                    fine.clear();
                } else {
                    joined = diff(fine, keepout);
                    fine   = intersection(fine, keepout);
                }
                if (! joined.empty())
                    host.polygons = union_(host.polygons, joined);
            }
            if (! fine.empty()) {
                layer.fine_body_polygons = union_(fine);
                layer.fine_body_height   = height[k];
                ++ num_fine_layers;
            }
        }
    }

    if (const size_t num_guarded = mixed_nozzle_thin_coarse_body_to_fine(base_layers, body_min, slicing_params); num_guarded > 0)
        BOOST_LOG_TRIVIAL(warning) << "SupportCadence: object " << object.id().id << ": " << num_guarded
                                   << " support layers thinner than the coarse nozzle's minimum went to the interface nozzle";

    BOOST_LOG_TRIVIAL(info) << "SupportCadence: object " << object.id().id << ": support body laid in " << num_bands
                            << " bands of " << body_min << " to " << body_max << " mm on nozzle " << body_tool + 1
                            << " (aiming for " << target << " mm); " << num_fine_layers
                            << " layers hand part of the body to the interface nozzle because no band can hold it";
}

void mixed_nozzle_coarsen_shared_support(
    const PrintObject               &object,
    const SupportParameters         &support_params,
    const SlicingParameters         &slicing_params,
    SupportGeneratorLayersPtr       &bottom_contacts,
    SupportGeneratorLayersPtr       &top_contacts,
    SupportGeneratorLayersPtr       &interface_layers,
    SupportGeneratorLayersPtr       &base_interface_layers,
    SupportGeneratorLayersPtr       &base_layers)
{
    if (! support_params.mixed_nozzle_coarse_support)
        return;
    const PrintConfig &print_config = object.print()->config();
    const int nozzle_idx = resolved_support_filament_nozzle_idx(print_config, object.config().support_filament.value);
    if (nozzle_idx <= 0 || size_t(nozzle_idx) > print_config.nozzle_diameter.values.size())
        return;
    const size_t   tool    = size_t(nozzle_idx - 1);
    const coordf_t minimum = resolved_min_layer_height(print_config, tool);
    const coordf_t maximum = resolved_max_layer_height(print_config, tool);
    if (! std::isfinite(minimum) || ! std::isfinite(maximum) || minimum <= 0. || maximum < minimum)
        return;

    // Rows aim for the coarse cadence: Feature Split's coarse layer height, or under Body Split the
    // cadence of the bodies this nozzle lays.
    coordf_t target = is_mixed_nozzle_feature_split(print_config) ? coordf_t(print_config.mixed_nozzle_coarse_layer_height.value) : 0.;
    if (is_mixed_nozzle_body_split(print_config)) {
        const std::vector<size_t> &grid_tools = object.region_grid_physical_extruders();
        for (size_t region_id = 0; region_id < object.num_printing_regions() && region_id < grid_tools.size(); ++ region_id)
            if (grid_tools[region_id] == tool)
                target = std::max(target, coordf_t(object.printing_region(region_id).config().regional_layer_height.value));
    }
    if (! std::isfinite(target) || target <= 0.)
        target = slicing_params.max_suport_layer_height;
    if (! std::isfinite(target) || target <= 0.)
        target = maximum;
    target = std::clamp(target, minimum, maximum);

    const coordf_t floor_z = slicing_params.has_raft() ? slicing_params.raft_contact_top_z : 0.;
    const auto movable = [floor_z](const SupportGeneratorLayer *layer) {
        return layer != nullptr && ! layer->polygons.empty() && layer->print_z > floor_z + EPSILON;
    };
    const std::array<SupportGeneratorLayersPtr*, 5> lists { &bottom_contacts, &top_contacts, &interface_layers,
                                                            &base_interface_layers, &base_layers };

    // Z that must be a row: each top contact, and the bottom of each bottom contact.
    std::vector<std::pair<coordf_t, bool>> marks; // Z, is a top contact
    for (const SupportGeneratorLayer *layer : top_contacts)
        if (movable(layer))
            marks.emplace_back(layer->print_z, true);
    for (const SupportGeneratorLayer *layer : bottom_contacts)
        if (movable(layer) && layer->print_z - layer->height > floor_z + EPSILON)
            marks.emplace_back(layer->print_z - layer->height, false);
    std::sort(marks.begin(), marks.end());

    // A mark too close above the last kept one is dropped. A dropped top contact goes down to that
    // row, never up towards the part, and so does everything between. A dropped contact bottom keeps
    // its own lower boundary, so the first row above that kept one is at least the nozzle's thinnest
    // layer above it.
    const auto key = [](coordf_t z) { return (long long)std::llround(z * 1e5); };
    std::vector<coordf_t> kept;
    std::vector<std::pair<coordf_t, coordf_t>> lowered; // row, dropped Z
    std::map<long long, coordf_t> first_row_floor;     // kept row, lowest legal next row
    for (const auto &[z, top_contact] : marks) {
        if (! kept.empty() && z < kept.back() + EPSILON)
            continue;
        if (! kept.empty() && z - kept.back() < minimum - EPSILON) {
            if (top_contact)
                lowered.emplace_back(kept.back(), z);
            else {
                coordf_t &floor = first_row_floor[key(kept.back())];
                floor = std::max(floor, z + minimum);
            }
            continue;
        }
        kept.push_back(z);
    }

    // Rows land on object layers or support layers.
    std::vector<coordf_t> candidates;
    coordf_t top_z = floor_z;
    for (const Layer *layer : object.layers())
        candidates.push_back(layer->print_z);
    for (const SupportGeneratorLayersPtr *list : lists)
        for (const SupportGeneratorLayer *layer : *list)
            if (movable(layer)) {
                candidates.push_back(layer->print_z);
                top_z = std::max(top_z, layer->print_z);
            }
    if (top_z <= floor_z + EPSILON)
        return;
    std::sort(candidates.begin(), candidates.end());

    std::vector<coordf_t> rows;
    coordf_t last = floor_z;
    // On the bed the first row is the thinnest the nozzle can lay.
    if (! slicing_params.has_raft()) {
        const coordf_t first_end = kept.empty() ? top_z : kept.front();
        auto it = std::lower_bound(candidates.begin(), candidates.end(), floor_z + minimum - EPSILON);
        if (it != candidates.end() && (*it < first_end - minimum + EPSILON || std::abs(*it - first_end) < EPSILON)) {
            last = *it;
            rows.push_back(last);
        }
    }
    const auto fill_to = [&](coordf_t end) {
        // A contact bottom dropped just above the last row needs the next row high enough over it.
        coordf_t floor = 0.;
        if (auto it = first_row_floor.find(key(last)); it != first_row_floor.end())
            floor = it->second;
        while (end - last > target + EPSILON) {
            // The highest candidate within the cadence that leaves a legal rest, else the lowest legal one.
            const coordf_t low  = std::max(last + minimum, floor) - EPSILON;
            coordf_t       high = std::min(last + target, end - minimum) + EPSILON;
            // Above a dropped contact bottom the row may go past the cadence, up to the nozzle's maximum.
            const bool raised = high < low;
            if (raised)
                high = std::min(last + maximum, end - minimum) + EPSILON;
            if (high < low)
                break;
            auto it = std::upper_bound(candidates.begin(), candidates.end(), high);
            coordf_t next;
            if (! raised && it != candidates.begin() && *(it - 1) >= low)
                next = *(it - 1);
            else if (it = std::lower_bound(candidates.begin(), candidates.end(), low);
                     it != candidates.end() && *it <= (raised ? high : end - minimum + EPSILON))
                next = *it;
            else
                break;
            last  = next;
            floor = 0.;
            rows.push_back(last);
        }
        rows.push_back(end);
        last = end;
    };
    for (coordf_t z : kept)
        if (z > last + EPSILON)
            fill_to(z);
    if (top_z > last + EPSILON)
        fill_to(top_z);

    const auto row_of = [&](coordf_t z) {
        for (const auto &[row, dropped] : lowered)
            if (z > row + EPSILON && z < dropped + EPSILON)
                return row;
        auto it = std::lower_bound(rows.begin(), rows.end(), z - EPSILON);
        return it == rows.end() ? rows.back() : *it;
    };
    const auto row_below = [&](coordf_t row) {
        auto it = std::lower_bound(rows.begin(), rows.end(), row - EPSILON);
        return it == rows.begin() ? floor_z : *(it - 1);
    };

    // Each list's layers that land on one row become one layer at that row, with the row's full height.
    std::map<long long, Polygons> interface_at_row;
    // Organic trees can list one layer in more than one list. Each layer moves once, and a layer merged
    // into another is reset, so it leaves every list.
    std::set<const SupportGeneratorLayer*> merged_away;
    std::set<const SupportGeneratorLayer*> moved;
    // The surface each contact bottom stands on. Contacts merged onto one row keep the highest, so no
    // part of the merged contact reaches down into a part under it.
    std::map<const SupportGeneratorLayer*, coordf_t> stands_on;
    for (const SupportGeneratorLayer *layer : bottom_contacts)
        if (movable(layer))
            stands_on[layer] = layer->print_z - layer->height;
    // A contact bottom moves up to the next row, never down with a lowered top contact.
    const auto row_for = [&](const SupportGeneratorLayer *layer) {
        if (stands_on.count(layer) == 0)
            return row_of(layer->print_z);
        auto it = std::lower_bound(rows.begin(), rows.end(), layer->print_z - EPSILON);
        return it == rows.end() ? rows.back() : *it;
    };
    // A base interface is the dense base-material row between an interface and the body. Where an interface
    // or contact already took its row, it moves to the row below a top contact's interface, or to the row
    // above a bottom contact's.
    std::map<long long, bool> interface_row_is_bottom;
    std::map<long long, bool> interface_rows;
    const auto list_row = [&](const SupportGeneratorLayersPtr *list, const SupportGeneratorLayer *layer) {
        const coordf_t row = row_for(layer);
        auto it = list == &base_interface_layers ? interface_rows.find(key(row)) : interface_rows.end();
        if (it == interface_rows.end())
            return row;
        if (it->second) {
            auto above = std::upper_bound(rows.begin(), rows.end(), row + EPSILON);
            return above == rows.end() ? row : *above;
        }
        return row_below(row) > floor_z + EPSILON ? row_below(row) : row;
    };
    for (SupportGeneratorLayersPtr *list : lists) {
        const bool is_base = list == &base_layers;
        if (list == &base_interface_layers)
            interface_rows = interface_row_is_bottom;
        SupportGeneratorLayersPtr out;
        out.reserve(list->size());
        SupportGeneratorLayer *keeper = nullptr;
        coordf_t keeper_row = 0.;
        for (SupportGeneratorLayer *layer : *list) {
            if (layer != nullptr && merged_away.count(layer) != 0)
                continue;
            if (layer == nullptr || moved.count(layer) != 0 || layer->print_z <= floor_z + EPSILON) {
                out.push_back(layer);
                continue;
            }
            if (layer->polygons.empty())
                continue;
            const coordf_t row = list_row(list, layer);
            if (keeper != nullptr && keeper != layer && std::abs(row - keeper_row) < EPSILON) {
                const bool bridging = keeper->bridging && layer->bridging;
                if (auto it = stands_on.find(layer); it != stands_on.end()) {
                    coordf_t &surface = stands_on[keeper];
                    surface = std::max(surface, it->second);
                }
                keeper->merge(std::move(*layer));
                keeper->bridging = bridging;
                merged_away.insert(layer);
                continue;
            }
            if (keeper == layer)
                continue;
            keeper     = layer;
            keeper_row = row;
            out.push_back(layer);
        }
        for (SupportGeneratorLayer *layer : out) {
            if (layer == nullptr || layer->print_z <= floor_z + EPSILON || ! moved.insert(layer).second)
                continue;
            const coordf_t row    = list_row(list, layer);
            const coordf_t bottom = row_below(row);
            layer->print_z = row;
            layer->height  = row - bottom;
            if (auto it = stands_on.find(layer); it != stands_on.end() && it->second > bottom) {
                // A contact bottom never reaches below the surface it stands on.
                layer->height = row - it->second;
                // Organic trees do not print their contact bottoms.
                if (layer->height < minimum - EPSILON && support_params.support_style != smsTreeOrganic) {
                    std::ostringstream message;
                    message << std::fixed << std::setprecision(2) << "Support cannot stand on the part at "
                            << it->second << " mm: the " << print_config.nozzle_diameter.get_at(tool)
                            << " mm nozzle cannot lay a support layer that thin there. Set a larger Support bottom Z "
                               "distance, or put the support interface on the finer nozzle.";
                    throw Slic3r::SlicingError(message.str());
                }
            } else if (it == stands_on.end()) {
                // Nothing else reaches into the part either, as an organic tree standing on the part
                // would on a row that starts below that surface. That part of the layer is left out,
                // so the tree starts on the next row up.
                Polygons part;
                for (const Layer *object_layer : object.layers())
                    if (object_layer->print_z > bottom + EPSILON && object_layer->print_z - object_layer->height < row - EPSILON)
                        polygons_append(part, to_polygons(object_layer->lslices));
                if (! part.empty())
                    layer->polygons = diff(layer->polygons, part);
            }
            if (layer->layer_type != SupporLayerType::BottomContact)
                layer->bottom_z = bottom;
            if (! is_base) {
                polygons_append(interface_at_row[key(row)], layer->polygons);
                const bool on_part = layer->layer_type == SupporLayerType::BottomContact || layer->layer_type == SupporLayerType::BottomInterface;
                auto [it, fresh] = interface_row_is_bottom.emplace(key(row), on_part);
                if (! fresh)
                    it->second = it->second && on_part;
            }
        }
        *list = std::move(out);
    }
    for (SupportGeneratorLayersPtr *list : lists)
        list->erase(std::remove_if(list->begin(), list->end(),
                                   [&merged_away](const SupportGeneratorLayer *layer) { return merged_away.count(layer) != 0; }),
                    list->end());
    // Where an interface or contact landed on a row, the body there gives way.
    for (SupportGeneratorLayer *layer : base_layers)
        if (movable(layer))
            if (auto it = interface_at_row.find(key(layer->print_z)); it != interface_at_row.end())
                layer->polygons = diff(layer->polygons, it->second);

    BOOST_LOG_TRIVIAL(info) << "SupportCadence: object " << object.id().id << ": support laid on " << rows.size()
                            << " rows of " << minimum << " to " << maximum << " mm on nozzle " << tool + 1
                            << " (aiming for " << target << " mm); " << lowered.size() << " contacts lowered to the row below";
}

// Support layer that is covered by some form of dense interface.
static constexpr const std::initializer_list<SupporLayerType> support_types_interface{
    SupporLayerType::RaftInterface, SupporLayerType::BottomContact, SupporLayerType::BottomInterface, SupporLayerType::TopContact, SupporLayerType::TopInterface
};

SupportGeneratorLayersPtr generate_support_layers(
    PrintObject                         &object,
    const SupportGeneratorLayersPtr     &raft_layers,
    const SupportGeneratorLayersPtr     &bottom_contacts,
    const SupportGeneratorLayersPtr     &top_contacts,
    const SupportGeneratorLayersPtr     &intermediate_layers,
    const SupportGeneratorLayersPtr     &interface_layers,
    const SupportGeneratorLayersPtr     &base_interface_layers)
{
    // Install support layers into the object.
    // A support layer installed on a PrintObject has a unique print_z.
    SupportGeneratorLayersPtr layers_sorted;
    layers_sorted.reserve(raft_layers.size() + bottom_contacts.size() + top_contacts.size() + intermediate_layers.size() + interface_layers.size() + base_interface_layers.size());
    append(layers_sorted, raft_layers);
    append(layers_sorted, bottom_contacts);
    append(layers_sorted, top_contacts);
    append(layers_sorted, intermediate_layers);
    append(layers_sorted, interface_layers);
    append(layers_sorted, base_interface_layers);
    // remove dupliated layers
    std::sort(layers_sorted.begin(), layers_sorted.end());
    layers_sorted.erase(std::unique(layers_sorted.begin(), layers_sorted.end()), layers_sorted.end());

    // Sort the layers lexicographically by a raising print_z and a decreasing height.
    std::sort(layers_sorted.begin(), layers_sorted.end(), [](auto *l1, auto *l2) { return *l1 < *l2; });
    int layer_id = 0;
    int layer_id_interface = 0;
    assert(object.support_layers().empty());
    for (size_t i = 0; i < layers_sorted.size();) {
        // Find the last layer with roughly the same print_z, find the minimum layer height of all.
        // Due to the floating point inaccuracies, the print_z may not be the same even if in theory they should.
        size_t j = i + 1;
        coordf_t zmax = layers_sorted[i]->print_z + EPSILON;
        for (; j < layers_sorted.size() && layers_sorted[j]->print_z <= zmax; ++j) ;
        // Assign an average print_z to the set of layers with nearly equal print_z.
        coordf_t zavg = 0.5 * (layers_sorted[i]->print_z + layers_sorted[j - 1]->print_z);
        coordf_t height_min = layers_sorted[i]->height;
        bool     empty = true;
        // For snug supports, layers where the direction of the support interface shall change are accounted for.
        size_t   num_interfaces = 0;
        size_t   num_top_contacts = 0;
        double   top_contact_bottom_z = 0;
        for (size_t u = i; u < j; ++u) {
            SupportGeneratorLayer &layer = *layers_sorted[u];
            if (! layer.polygons.empty()) {
                empty             = false;
                const bool is_base_interface = std::find(base_interface_layers.begin(), base_interface_layers.end(), &layer) != base_interface_layers.end();
                num_interfaces += one_of(layer.layer_type, support_types_interface) || is_base_interface;
                if (layer.layer_type == SupporLayerType::TopContact) {
                    ++ num_top_contacts;
                    assert(num_top_contacts <= 1);
                    // All top contact layers sharing this print_z shall also share bottom_z.
                    //assert(num_top_contacts == 1 || (top_contact_bottom_z - layer.bottom_z) < EPSILON);
                    top_contact_bottom_z = layer.bottom_z;
                }
            }
            // A base layer whose body went entirely to the interface nozzle still has something to print at
            // this height.
            if (! layer.fine_body_polygons.empty()) {
                empty      = false;
                height_min = std::min(height_min, layer.fine_body_height);
            }
            layer.print_z = zavg;
            height_min = std::min(height_min, layer.height);
        }
        if (! empty) {
            // Here the upper_layer and lower_layer pointers are left to null at the support layers,
            // as they are never used. These pointers are candidates for removal.
            bool   this_layer_contacts_only = num_top_contacts > 0 && num_top_contacts == num_interfaces;
            size_t this_layer_id_interface  = layer_id_interface;
            if (this_layer_contacts_only) {
                // Find a supporting layer for its interface ID.
                for (auto it = object.support_layers().rbegin(); it != object.support_layers().rend(); ++ it)
                    if (const SupportLayer &other_layer = **it; std::abs(other_layer.print_z - top_contact_bottom_z) < EPSILON) {
                        // other_layer supports this top contact layer. Assign a different support interface direction to this layer
                        // from the layer that supports it.
                        this_layer_id_interface = other_layer.interface_id() + 1;
                    }
            }
            object.add_support_layer(layer_id ++, this_layer_id_interface, height_min, zavg);
            if (num_interfaces && ! this_layer_contacts_only)
                ++ layer_id_interface;
        }
        i = j;
    }
    return layers_sorted;
}

void generate_support_toolpaths(
    SupportLayerPtrs                    &support_layers,
    const PrintObjectConfig             &config,
    const SupportParameters             &support_params,
    const SlicingParameters             &slicing_params,
    const SupportGeneratorLayersPtr     &raft_layers,
    const SupportGeneratorLayersPtr     &bottom_contacts,
    const SupportGeneratorLayersPtr     &top_contacts,
    const SupportGeneratorLayersPtr     &intermediate_layers,
    const SupportGeneratorLayersPtr     &interface_layers,
    const SupportGeneratorLayersPtr     &base_interface_layers)
{
    // loop_interface_processor with a given circle radius.
    LoopInterfaceProcessor loop_interface_processor(1.5 * support_params.support_material_interface_flow.scaled_width());
    loop_interface_processor.n_contact_loops = config.support_interface_loop_pattern.value ? 1 : 0;

    std::vector<float>      angles { support_params.base_angle };
    if (config.support_base_pattern == smpRectilinearGrid)
        angles.push_back(support_params.interface_angle);

    BoundingBox bbox_object(Point(-scale_(1.), -scale_(1.0)), Point(scale_(1.), scale_(1.)));

//    const coordf_t link_max_length_factor = 3.;
    const coordf_t link_max_length_factor = 0.;

    // Insert the raft base layers.
    auto n_raft_layers = std::min<size_t>(support_layers.size(), std::max(0, int(slicing_params.raft_layers()) - 1));

    tbb::parallel_for(tbb::blocked_range<size_t>(0, n_raft_layers),
        [&support_layers, &raft_layers, &intermediate_layers, &config, &support_params, &slicing_params,
            &bbox_object, link_max_length_factor]
            (const tbb::blocked_range<size_t>& range) {
        for (size_t support_layer_id = range.begin(); support_layer_id < range.end(); ++ support_layer_id)
        {
            assert(support_layer_id < raft_layers.size());
            SupportLayer               &support_layer = *support_layers[support_layer_id];
            assert(support_layer.support_fills.entities.empty());
            SupportGeneratorLayer      &raft_layer    = *raft_layers[support_layer_id];

            std::unique_ptr<Fill> filler_interface = std::unique_ptr<Fill>(Fill::new_from_type(support_params.raft_interface_fill_pattern));
            std::unique_ptr<Fill> filler_support   = std::unique_ptr<Fill>(Fill::new_from_type(support_params.base_fill_pattern));
            filler_interface->set_bounding_box(bbox_object);
            filler_support->set_bounding_box(bbox_object);

            // Print the tree supports cutting through the raft with the exception of the 1st layer, where a full support layer will be printed below
            // both the raft and the trees.
            // Trim the raft layers with the tree polygons.
            const Polygons &tree_polygons =
                support_layer_id > 0 && support_layer_id < intermediate_layers.size() && is_approx(intermediate_layers[support_layer_id]->print_z, support_layer.print_z) ?
                intermediate_layers[support_layer_id]->polygons : Polygons();

            // Print the support base below the support columns, or the support base for the support columns plus the contacts.
            if (support_layer_id > 0) {
                const Polygons &to_infill_polygons = (support_layer_id < slicing_params.base_raft_layers) ?
                    raft_layer.polygons :
                    //FIXME misusing contact_polygons for support columns.
                    ((raft_layer.contact_polygons == nullptr) ? Polygons() : *raft_layer.contact_polygons);
                // Trees may cut through the raft layers down to a print bed.
                Flow flow(float(support_params.support_material_flow.width()), float(raft_layer.height), support_params.support_material_flow.nozzle_diameter());
                assert(!raft_layer.bridging);
                if (! to_infill_polygons.empty()) {
                    Fill *filler = filler_support.get();
                    filler->angle = support_params.raft_angle_base;
                    filler->spacing = support_params.support_material_flow.spacing();
                    filler->link_max_length = coord_t(scale_(filler->spacing * link_max_length_factor / support_params.support_density));
                    fill_expolygons_with_sheath_generate_paths(
                        // Destination
                        support_layer.support_fills.entities,
                        // Regions to fill
                        tree_polygons.empty() ? to_infill_polygons : diff(to_infill_polygons, tree_polygons),
                        // Filler and its parameters
                        filler, float(support_params.support_density),
                        // Extrusion parameters
                        ExtrusionRole::erSupportMaterial, flow,
                        support_params, support_params.with_sheath, false);
                }
                if (! tree_polygons.empty())
                    tree_supports_generate_paths(support_layer.support_fills.entities, tree_polygons, flow, support_params);
            }

            Fill *filler = filler_interface.get();
            Flow  flow = support_params.first_layer_flow;
            float density = 0.f;
            if (support_layer_id == 0) {
                // Base flange.
                filler->angle = support_params.raft_angle_1st_layer;
                filler->spacing = support_params.first_layer_flow.spacing();
                density       = float(config.raft_first_layer_density.value * 0.01);
            } else if (support_layer_id >= slicing_params.base_raft_layers) {
                filler->angle = support_params.raft_interface_angle(support_layer.interface_id());
                // We don't use $base_flow->spacing because we need a constant spacing
                // value that guarantees that all layers are correctly aligned.
                filler->spacing = support_params.support_material_flow.spacing();
                assert(! raft_layer.bridging);
                flow          = Flow(float(support_params.raft_interface_flow.width()), float(raft_layer.height), support_params.raft_interface_flow.nozzle_diameter());
                density       = float(support_params.raft_interface_density);
            } else
                continue;
            filler->link_max_length = coord_t(scale_(filler->spacing * link_max_length_factor / density));
            fill_expolygons_with_sheath_generate_paths(
                // Destination
                support_layer.support_fills.entities,
                // Regions to fill
                tree_polygons.empty() ? raft_layer.polygons : diff(raft_layer.polygons, tree_polygons),
                // Filler and its parameters
                filler, density,
                // Extrusion parameters
                (support_layer_id < slicing_params.base_raft_layers) ? ExtrusionRole::erSupportMaterial : ExtrusionRole::erSupportMaterialInterface, flow,
                // sheath at first layer
                support_params, support_layer_id == 0, support_layer_id == 0);
        }
    });

    struct LayerCacheItem {
        LayerCacheItem(SupportGeneratorLayerExtruded *layer_extruded = nullptr) : layer_extruded(layer_extruded) {}
        SupportGeneratorLayerExtruded         *layer_extruded;
        std::vector<SupportGeneratorLayer*>    overlapping;
    };
    struct LayerCache {
        SupportGeneratorLayerExtruded                                     bottom_contact_layer;
        SupportGeneratorLayerExtruded                                     top_contact_layer;
        SupportGeneratorLayerExtruded                                     base_layer;
        SupportGeneratorLayerExtruded                                     interface_layer;
        SupportGeneratorLayerExtruded                                     base_interface_layer;
        boost::container::static_vector<LayerCacheItem, 5>  nonempty;
        // Body roads the interface nozzle lays, see mixed_nozzle_band_support_body().
        ExtrusionEntitiesPtr                                fine_body_extrusions;
        // They share the layer with coarse body roads, so they go to SupportLayer::fine_body_fills.
        bool                                                fine_body_beside_coarse = false;

        float    ironing_angle;
        Polygons polys_to_iron;

        void add_nonempty_and_sort() {
            for (SupportGeneratorLayerExtruded *item : { &bottom_contact_layer, &top_contact_layer, &interface_layer, &base_interface_layer, &base_layer })
                if (! item->empty())
                    this->nonempty.emplace_back(item);
            // Sort the layers with the same print_z coordinate by their heights, thickest first.
            std::stable_sort(this->nonempty.begin(), this->nonempty.end(), [](const LayerCacheItem &lc1, const LayerCacheItem &lc2) { return lc1.layer_extruded->layer->height > lc2.layer_extruded->layer->height; });
        }
    };
    std::vector<LayerCache>             layer_caches(support_layers.size());

    tbb::parallel_for(tbb::blocked_range<size_t>(n_raft_layers, support_layers.size()),
        [&config, &slicing_params, &support_params, &support_layers, &bottom_contacts, &top_contacts, &intermediate_layers, &interface_layers, &base_interface_layers, &layer_caches, &loop_interface_processor,
            &bbox_object, &angles, n_raft_layers, link_max_length_factor]
            (const tbb::blocked_range<size_t>& range) {
        // Indices of the 1st layer in their respective container at the support layer height.
        size_t idx_layer_bottom_contact   = size_t(-1);
        size_t idx_layer_top_contact      = size_t(-1);
        size_t idx_layer_intermediate     = size_t(-1);
        size_t idx_layer_interface        = size_t(-1);
        size_t idx_layer_base_interface   = size_t(-1);
        const auto fill_type_first_layer  = ipRectilinear;
        auto filler_interface       = std::unique_ptr<Fill>(Fill::new_from_type(support_params.contact_fill_pattern));
        // Filler for the 1st layer interface, if different from filler_interface.
        auto filler_first_layer_ptr = std::unique_ptr<Fill>(range.begin() == 0 && support_params.contact_fill_pattern != fill_type_first_layer ? Fill::new_from_type(fill_type_first_layer) : nullptr);
        // Pointer to the 1st layer interface filler.
        auto filler_first_layer     = filler_first_layer_ptr ? filler_first_layer_ptr.get() : filler_interface.get();
        // Filler for the 1st layer interface, if different from filler_interface.
        const bool top_interfaces_enabled    = support_params.num_top_interface_layers > 0;
        const bool bottom_interfaces_enabled = support_params.num_bottom_interface_layers > 0;
        const coordf_t base_interface_density = top_interfaces_enabled || !bottom_interfaces_enabled ?
            support_params.top_interface_density : support_params.bottom_interface_density;
        auto filler_raft_contact_ptr = std::unique_ptr<Fill>(range.begin() == n_raft_layers && !top_interfaces_enabled ?
            Fill::new_from_type(support_params.raft_interface_fill_pattern) : nullptr);
        // Pointer to the 1st layer interface filler.
        auto filler_raft_contact     = filler_raft_contact_ptr ? filler_raft_contact_ptr.get() : filler_interface.get();
        // Filler for the base interface (to be used for soluble interface / non soluble base, to produce non soluble interface layer below soluble interface layer).
        auto filler_base_interface  = std::unique_ptr<Fill>(base_interface_layers.empty() ? nullptr :
            Fill::new_from_type(base_interface_density > 0.95 || support_params.with_sheath ? ipRectilinear : ipSupportBase));
        auto filler_support         = std::unique_ptr<Fill>(Fill::new_from_type(support_params.base_fill_pattern));
        filler_interface->set_bounding_box(bbox_object);
        if (filler_first_layer_ptr)
            filler_first_layer_ptr->set_bounding_box(bbox_object);
        if (filler_raft_contact_ptr)
            filler_raft_contact_ptr->set_bounding_box(bbox_object);
        if (filler_base_interface)
            filler_base_interface->set_bounding_box(bbox_object);
        filler_support->set_bounding_box(bbox_object);
        for (size_t support_layer_id = range.begin(); support_layer_id < range.end(); ++ support_layer_id)
        {
            SupportLayer &support_layer = *support_layers[support_layer_id];
            LayerCache   &layer_cache   = layer_caches[support_layer_id];
            const float   support_interface_angle = support_params.support_interface_angle(support_layer.interface_id());

            // Find polygons with the same print_z.
            SupportGeneratorLayerExtruded &bottom_contact_layer = layer_cache.bottom_contact_layer;
            SupportGeneratorLayerExtruded &top_contact_layer    = layer_cache.top_contact_layer;
            SupportGeneratorLayerExtruded &base_layer           = layer_cache.base_layer;
            SupportGeneratorLayerExtruded &interface_layer      = layer_cache.interface_layer;
            SupportGeneratorLayerExtruded &base_interface_layer = layer_cache.base_interface_layer;
            // Increment the layer indices to find a layer at support_layer.print_z.
            {
                auto fun = [&support_layer](const SupportGeneratorLayer *l){ return l->print_z >= support_layer.print_z - EPSILON; };
                idx_layer_bottom_contact  = idx_higher_or_equal(bottom_contacts,     idx_layer_bottom_contact,  fun);
                idx_layer_top_contact     = idx_higher_or_equal(top_contacts,        idx_layer_top_contact,     fun);
                idx_layer_intermediate    = idx_higher_or_equal(intermediate_layers, idx_layer_intermediate,    fun);
                idx_layer_interface       = idx_higher_or_equal(interface_layers,    idx_layer_interface,       fun);
                idx_layer_base_interface  = idx_higher_or_equal(base_interface_layers, idx_layer_base_interface,fun);
            }
            // Copy polygons from the layers.
            if (idx_layer_bottom_contact < bottom_contacts.size() && bottom_contacts[idx_layer_bottom_contact]->print_z < support_layer.print_z + EPSILON)
                bottom_contact_layer.layer = bottom_contacts[idx_layer_bottom_contact];
            if (idx_layer_top_contact < top_contacts.size() && top_contacts[idx_layer_top_contact]->print_z < support_layer.print_z + EPSILON)
                top_contact_layer.layer = top_contacts[idx_layer_top_contact];
            if (idx_layer_interface < interface_layers.size() && interface_layers[idx_layer_interface]->print_z < support_layer.print_z + EPSILON)
                interface_layer.layer = interface_layers[idx_layer_interface];
            if (idx_layer_base_interface < base_interface_layers.size() && base_interface_layers[idx_layer_base_interface]->print_z < support_layer.print_z + EPSILON)
                base_interface_layer.layer = base_interface_layers[idx_layer_base_interface];
            if (idx_layer_intermediate < intermediate_layers.size() && intermediate_layers[idx_layer_intermediate]->print_z < support_layer.print_z + EPSILON)
                base_layer.layer = intermediate_layers[idx_layer_intermediate];
            // The part of the body at this height that the interface nozzle lays.
            const SupportGeneratorLayer *fine_body_layer =
                base_layer.layer != nullptr && ! base_layer.layer->fine_body_polygons.empty() ? base_layer.layer : nullptr;

            // This layer is a raft contact layer. Any contact polygons at this layer are raft contacts.
            bool raft_layer = slicing_params.interface_raft_layers && top_contact_layer.layer && is_approx(top_contact_layer.layer->print_z, slicing_params.raft_contact_top_z);
            // ORCA: Organic tree uses projected contacts to build the interface stack; avoid extra bottom-contact extrusion.
            const bool organic_tree = support_params.support_style == SupportMaterialStyle::smsTreeOrganic;
            const bool top_interfaces = support_params.num_top_interface_layers > 0;
            const bool bottom_interfaces = support_params.num_bottom_interface_layers > 0;
            if (!top_interfaces) {
                // If no top interface layers were requested, we treat the contact layer exactly as a generic base layer.
                // Don't merge the raft contact layer though.
                if (support_params.can_merge_support_regions && ! raft_layer) {
                    if (base_layer.could_merge(top_contact_layer))
                        base_layer.merge(std::move(top_contact_layer));
                    else if (base_layer.empty())
                        base_layer = std::move(top_contact_layer);
                }
            } else {
                if (support_params.ironing && !top_contact_layer.empty()) {
                    // Orca: save the top surface to be ironed later
                    layer_cache.ironing_angle = support_interface_angle; // TODO: should we rotate 90 degrees?
                    layer_cache.polys_to_iron = top_contact_layer.polygons_to_extrude();
                }

                loop_interface_processor.generate(top_contact_layer, support_params.support_material_interface_flow);
                // If no loops are allowed, we treat the contact layer exactly as a generic interface layer.
                // Merge interface_layer into top_contact_layer, as the top_contact_layer is not synchronized and therefore it will be used
                // to trim other layers.
                if (top_contact_layer.could_merge(interface_layer) && ! raft_layer)
                    top_contact_layer.merge(std::move(interface_layer));
            }
            if (!bottom_interfaces && support_params.can_merge_support_regions) {
                if (base_layer.could_merge(bottom_contact_layer))
                    base_layer.merge(std::move(bottom_contact_layer));
                else if (base_layer.empty() && ! bottom_contact_layer.empty() && ! bottom_contact_layer.layer->bridging)
                    base_layer = std::move(bottom_contact_layer);
            } else if (bottom_contact_layer.could_merge(top_contact_layer) && ! raft_layer) {
                if (top_interfaces && bottom_interfaces) {
                    top_contact_layer.merge(std::move(bottom_contact_layer));
                } else if (bottom_interfaces) {
                    top_contact_layer.set_polygons_to_extrude(
                        diff(top_contact_layer.polygons_to_extrude(), bottom_contact_layer.polygons_to_extrude()));
                } else {
                    bottom_contact_layer.set_polygons_to_extrude(
                        diff(bottom_contact_layer.polygons_to_extrude(), top_contact_layer.polygons_to_extrude()));
                }
            } else if (bottom_contact_layer.could_merge(interface_layer) && ! organic_tree) {
                const bool interface_layer_is_bottom = interface_layer.layer->layer_type == SupporLayerType::BottomInterface;
                if (bottom_interfaces && interface_layer_is_bottom) {
                    bottom_contact_layer.merge(std::move(interface_layer));
                } else {
                    bottom_contact_layer.set_polygons_to_extrude(
                        diff(bottom_contact_layer.polygons_to_extrude(), interface_layer.polygons_to_extrude()));
                }
            }

            // Orca: For organic trees the support-material regions are generated from
            // expanded wall polygons. With zero top Z gap and separate interface material,
            // that expansion can overlap same-layer interface-material regions, so trim
            // the support-material regions from those interface footprints here.
            if (organic_tree && support_params.zero_gap_interface_top && !support_params.can_merge_support_regions &&
                (!base_layer.empty() || !base_interface_layer.empty())) {
                Polygons interface_polygons;
                if (!top_contact_layer.empty())
                    polygons_append(interface_polygons, top_contact_layer.polygons_to_extrude());
                if (!interface_layer.empty())
                    polygons_append(interface_polygons, interface_layer.polygons_to_extrude());
                if (!interface_polygons.empty()) {
                    const coord_t trim_margin = std::max(
                        support_params.support_material_flow.scaled_width(),
                        support_params.support_material_interface_flow.scaled_width());
                    Polygons interface_keepout = offset(interface_polygons, trim_margin);
                    if (!base_layer.empty())
                        base_layer.set_polygons_to_extrude(diff(base_layer.polygons_to_extrude(), interface_keepout));
                    if (!base_interface_layer.empty())
                        base_interface_layer.set_polygons_to_extrude(diff(base_interface_layer.polygons_to_extrude(), interface_keepout));
                }
            }

#if 0
            if ( ! interface_layer.empty() && ! base_layer.empty()) {
                // turn base support into interface when it's contained in our holes
                // (this way we get wider interface anchoring)
                //FIXME The intention of the code below is unclear. One likely wanted to just merge small islands of base layers filling in the holes
                // inside interface layers, but the code below fills just too much, see GH #4570
                Polygons islands = top_level_islands(interface_layer.layer->polygons);
                polygons_append(interface_layer.layer->polygons, intersection(base_layer.layer->polygons, islands));
                base_layer.layer->polygons = diff(base_layer.layer->polygons, islands);
            }
#endif

            // Top and bottom contacts, interface layers.
            enum class InterfaceLayerType { TopContact, BottomContact, RaftContact, Interface, InterfaceAsBase };
            auto extrude_interface = [&](SupportGeneratorLayerExtruded &layer_ex, InterfaceLayerType interface_layer_type) {
                if (! layer_ex.empty() && ! layer_ex.polygons_to_extrude().empty()) {
                    bool interface_as_base = interface_layer_type == InterfaceLayerType::InterfaceAsBase;
                    bool raft_contact      = interface_layer_type == InterfaceLayerType::RaftContact;
                    // A contact printed with the base pattern is one object layer tall and sits against the
                    // part, so it cannot be banded. When the base nozzle is the coarser one it is laid by the
                    // interface nozzle, at that nozzle's own width, still with the base pattern and density.
                    const bool base_on_fine_tool = interface_as_base && support_params.mixed_nozzle_banded_body;
                    const Flow &as_base_flow = base_on_fine_tool ? support_params.mixed_nozzle_fine_body_flow :
                                                                   support_params.support_material_flow;
                    // ORCA: detect bottom interface layers for density selection.
                    bool bottom_interface  = interface_layer_type == InterfaceLayerType::BottomContact ||
                        (interface_layer_type == InterfaceLayerType::Interface && layer_ex.layer->layer_type == SupporLayerType::BottomInterface);
                    //FIXME Bottom interfaces are extruded with the briding flow. Some bridging layers have its height slightly reduced, therefore
                    // the bridging flow does not quite apply. Reduce the flow to area of an ellipse? (A = pi * a * b)
                    auto *filler = raft_contact ? filler_raft_contact : filler_interface.get();
                    auto interface_flow = layer_ex.layer->bridging ?
                        Flow::bridging_flow(layer_ex.layer->height, support_params.support_material_bottom_interface_flow.nozzle_diameter()) :
                        (raft_contact ? &support_params.raft_interface_flow :
                         interface_as_base ? &as_base_flow : &support_params.support_material_interface_flow)
                            ->with_height(float(layer_ex.layer->height));
                    filler->angle = interface_as_base ?
                            // If zero interface layers are configured, use the same angle as for the base layers.
                            angles[support_layer_id % angles.size()] :
                            // Use interface angle for the interface layers.
                            raft_contact ?
                                support_params.raft_interface_angle(support_layer.interface_id()) :
                                support_interface_angle;
                    // ORCA: pick density based on interface type.
                    double density = raft_contact ? support_params.raft_interface_density :
                        interface_as_base ? support_params.support_density :
                        bottom_interface ? support_params.bottom_interface_density : support_params.top_interface_density;
                    filler->spacing = raft_contact ? support_params.raft_interface_flow.spacing() :
                        interface_as_base ? as_base_flow.spacing() : support_params.support_material_interface_flow.spacing();
                    filler->link_max_length = coord_t(scale_(filler->spacing * link_max_length_factor / density));
                    // On the fine tool it joins the body that nozzle lays (fine_body_extrusions), so it is
                    // printed with the body filament there, as stock prints it with the base filament, and
                    // not with an interface filament of another material. Only next to a coarse band road
                    // on the same layer does it go to the interface filament (see below).
                    fill_expolygons_generate_paths(
                        // Destination
                        base_on_fine_tool ? layer_cache.fine_body_extrusions : layer_ex.extrusions,
                        // Regions to fill
                        union_safety_offset_ex(layer_ex.polygons_to_extrude()),
                        // Filler and its parameters
                        filler, float(density),
                        // Extrusion parameters
                        interface_as_base ? ExtrusionRole::erSupportMaterial : ExtrusionRole::erSupportMaterialInterface,
                        interface_flow);
                }
            };
            extrude_interface(top_contact_layer,    raft_layer ? InterfaceLayerType::RaftContact : top_interfaces ? InterfaceLayerType::TopContact : InterfaceLayerType::InterfaceAsBase);
            if (!organic_tree)
                extrude_interface(bottom_contact_layer, bottom_interfaces ? InterfaceLayerType::BottomContact : InterfaceLayerType::InterfaceAsBase);
            const bool interface_layer_enabled = !interface_layer.empty() &&
                (interface_layer.layer->layer_type == SupporLayerType::BottomInterface ? bottom_interfaces : top_interfaces);
            extrude_interface(interface_layer,      interface_layer_enabled ? InterfaceLayerType::Interface : InterfaceLayerType::InterfaceAsBase);
            // Base interface layers under soluble interfaces
            if ( ! base_interface_layer.empty() && ! base_interface_layer.polygons_to_extrude().empty()) {
                Fill *filler = filler_base_interface.get();
                //FIXME Bottom interfaces are extruded with the briding flow. Some bridging layers have its height slightly reduced, therefore
                // the bridging flow does not quite apply. Reduce the flow to area of an ellipse? (A = pi * a * b)
                assert(! base_interface_layer.layer->bridging);
                // Under a banded body this layer is one object layer tall, so the interface nozzle lays it.
                const Flow &base_interface_flow = support_params.mixed_nozzle_banded_body ?
                    support_params.mixed_nozzle_fine_body_flow : support_params.support_material_flow;
                Flow interface_flow = base_interface_flow.with_height(float(base_interface_layer.layer->height));
                filler->angle   = support_interface_angle;
                filler->spacing = support_params.mixed_nozzle_banded_body ? base_interface_flow.spacing() :
                                                                            support_params.support_material_interface_flow.spacing();
                filler->link_max_length = coord_t(scale_(filler->spacing * link_max_length_factor / base_interface_density));
                fill_expolygons_generate_paths(
                    // Destination
                    base_interface_layer.extrusions,
                    //base_layer_interface.extrusions,
                    // Regions to fill
                    union_safety_offset_ex(base_interface_layer.polygons_to_extrude()),
                    // Filler and its parameters
                    filler, float(base_interface_density),
                    // Extrusion parameters
                    ExtrusionRole::erSupportMaterial, interface_flow);
            }

            // Base support or flange.
            if (! base_layer.empty() && ! base_layer.polygons_to_extrude().empty()) {
                // A band road the nozzle cannot confine, narrower than its face or taller than it is wide,
                // drops a loose strand instead of a road, so it never reaches the G-code.
                const auto check_band_road = [&support_params](const Flow &flow) {
                    if ((support_params.mixed_nozzle_banded_body || support_params.mixed_nozzle_coarse_support) &&
                        (flow.width() < 0.75f * flow.nozzle_diameter() - EPSILON || flow.height() > flow.width() + EPSILON)) {
                        std::ostringstream message;
                        message << std::fixed << std::setprecision(2) << "The " << flow.nozzle_diameter()
                                << " mm nozzle cannot lay a support road " << flow.width() << " mm wide and " << flow.height()
                                << " mm tall. Raise the support line width.";
                        throw Slic3r::SlicingError(message.str());
                    }
                };
                Fill             *filler          = filler_support.get();
                filler->angle = angles[support_layer_id % angles.size()];
                // We don't use $base_flow->spacing because we need a constant spacing
                // value that guarantees that all layers are correctly aligned.
                assert(! base_layer.layer->bridging);
                auto flow = support_params.support_material_flow.with_height(float(base_layer.layer->height));
                if (base_layer.layer->bottom_z > EPSILON)
                    check_band_road(flow);
                filler->spacing = support_params.support_material_flow.spacing();
                filler->link_max_length = coord_t(scale_(filler->spacing * link_max_length_factor / support_params.support_density));
                float density = float(support_params.support_density);
                bool  sheath  = support_params.with_sheath;
                bool  no_sort = false;
                bool  done    = false;
                if (base_layer.layer->bottom_z < EPSILON) {
                    // Base flange (the 1st layer).
                    filler = filler_first_layer;
                    filler->angle = Geometry::deg2rad(float(config.support_angle.value + 90.));
                    density = float(config.raft_first_layer_density.value * 0.01);
                    flow = support_params.first_layer_flow;
                    // A band that starts on the bed is laid in one pass as tall as the band, still with the
                    // first layer's width.
                    if ((support_params.mixed_nozzle_banded_body || support_params.mixed_nozzle_coarse_support) &&
                        std::abs(base_layer.layer->height - flow.height()) > EPSILON)
                        flow = flow.with_height(float(base_layer.layer->height));
                    check_band_road(flow);
                    // use the proper spacing for first layer as we don't need to align
                    // its pattern to the other layers
                    //FIXME When paralellizing, each thread shall have its own copy of the fillers.
                    filler->spacing = flow.spacing();
                    filler->link_max_length = coord_t(scale_(filler->spacing * link_max_length_factor / density));
                    sheath  = true;
                    no_sort = true;
                } else if (support_params.support_style == SupportMaterialStyle::smsTreeOrganic &&
                           (config.support_base_pattern == smpNone || config.support_base_pattern == smpDefault)) {
                    // Orca: A special case for the hollow Organic supports
                    // Orca: If the tree supports are too tall, use a double wall to make it stronger
                    SupportParameters support_params2 = support_params;
                    if (support_layer.print_z > 100.0)
                        support_params2.tree_branch_diameter_double_wall_area_scaled = 0.1;
                    tree_supports_generate_paths(base_layer.extrusions, base_layer.polygons_to_extrude(), flow, support_params2);
                    done = true;
                }
                if (! done)
                    fill_expolygons_with_sheath_generate_paths(
                        // Destination
                        base_layer.extrusions,
                        // Regions to fill
                        base_layer.polygons_to_extrude(),
                        // Filler and its parameters
                        filler, density,
                        // Extrusion parameters
                        ExtrusionRole::erSupportMaterial, flow,
                        support_params, sheath, no_sort);
            }

            // The body no band could hold, laid by the interface nozzle at this layer's own height with the
            // base pattern and density.
            if (fine_body_layer != nullptr) {
                const bool on_bed = fine_body_layer->print_z - fine_body_layer->fine_body_height < EPSILON;
                Flow  flow    = (on_bed ? support_params.mixed_nozzle_fine_first_layer_flow : support_params.mixed_nozzle_fine_body_flow)
                                    .with_height(float(fine_body_layer->fine_body_height));
                Fill *filler  = on_bed ? filler_first_layer : filler_support.get();
                filler->angle = on_bed ? Geometry::deg2rad(float(config.support_angle.value + 90.)) : angles[support_layer_id % angles.size()];
                filler->spacing = flow.spacing();
                const float density = on_bed ? float(config.raft_first_layer_density.value * 0.01) : float(support_params.support_density);
                filler->link_max_length = coord_t(scale_(filler->spacing * link_max_length_factor / density));
                if (! on_bed && support_params.support_style == SupportMaterialStyle::smsTreeOrganic &&
                    (config.support_base_pattern == smpNone || config.support_base_pattern == smpDefault))
                    tree_supports_generate_paths(layer_cache.fine_body_extrusions, fine_body_layer->fine_body_polygons, flow, support_params);
                else
                    fill_expolygons_with_sheath_generate_paths(layer_cache.fine_body_extrusions, fine_body_layer->fine_body_polygons,
                        filler, density, ExtrusionRole::erSupportMaterial, flow, support_params,
                        on_bed || support_params.with_sheath, on_bed);
            }
            // The body the interface nozzle lays (the fine body above and a contact printed with the base
            // pattern) keeps the base role and the G-code gives it a body filament on the interface nozzle
            // (PrintObject::assign_interface_nozzle_body_filament()), so an interface filament of another
            // material never prints body, on the bed, under a contact or beside a coarse band road. Where
            // the coarse nozzle lays no body at this height that filament prints the layer's whole body;
            // beside a coarse road, on the bed too, the fine body is kept apart in fine_body_fills, so the coarse
            // road keeps its own nozzle.
            if (! layer_cache.fine_body_extrusions.empty()) {
                if (base_layer.extrusions.empty())
                    support_layer.base_on_interface_nozzle = true;
                else
                    layer_cache.fine_body_beside_coarse = true;
            }
            // The dense base-material layer under a dissimilar interface goes to the interface nozzle the same way.
            // Banding keeps coarse roads off its height, so the second branch is only a fallback.
            if (support_params.mixed_nozzle_banded_body && ! base_interface_layer.extrusions.empty()) {
                if (base_layer.extrusions.empty())
                    support_layer.base_on_interface_nozzle = true;
                else {
                    append(layer_cache.fine_body_extrusions, std::move(base_interface_layer.extrusions));
                    base_interface_layer.extrusions.clear();
                    layer_cache.fine_body_beside_coarse = true;
                }
            }

            // Merge base_interface_layers to base_layers to avoid unneccessary retractions
            if (! base_layer.empty() && ! base_interface_layer.empty() && ! base_layer.polygons_to_extrude().empty() && ! base_interface_layer.polygons_to_extrude().empty() &&
                base_layer.could_merge(base_interface_layer))
                base_layer.merge(std::move(base_interface_layer));

            layer_cache.add_nonempty_and_sort();

            // Collect the support areas with this print_z into islands, as there is no need
            // for retraction over these islands.
            Polygons polys;
            // Collect the extrusions, sorted by the bottom extrusion height.
            if (fine_body_layer != nullptr)
                polygons_append(polys, fine_body_layer->fine_body_polygons);
            for (LayerCacheItem &layer_cache_item : layer_cache.nonempty) {
                // Collect islands to polys.
                layer_cache_item.layer_extruded->polygons_append(polys);
                // The print_z of the top contact surfaces and bottom_z of the bottom contact surfaces are "free"
                // in a sense that they are not synchronized with other support layers. As the top and bottom contact surfaces
                // are inflated to achieve a better anchoring, it may happen, that these surfaces will at least partially
                // overlap in Z with another support layers, leading to over-extrusion.
                // Mitigate the over-extrusion by modulating the extrusion rate over these regions.
                // The print head will follow the same print_z, but the layer thickness will be reduced
                // where it overlaps with another support layer.
                //FIXME When printing a briging path, what is an equivalent height of the squished extrudate of the same width?
                // Collect overlapping top/bottom surfaces.
                layer_cache_item.overlapping.reserve(20);
                coordf_t bottom_z = layer_cache_item.layer_extruded->layer->bottom_print_z() + EPSILON;
                auto add_overlapping = [&layer_cache_item, bottom_z](const SupportGeneratorLayersPtr &layers, size_t idx_top) {
                    for (int i = int(idx_top) - 1; i >= 0 && layers[i]->print_z > bottom_z; -- i)
                        layer_cache_item.overlapping.push_back(layers[i]);
                };
                add_overlapping(top_contacts, idx_layer_top_contact);
                if (layer_cache_item.layer_extruded->layer->layer_type == SupporLayerType::BottomContact) {
                    // Bottom contact layer may overlap with a base layer, which may be changed to interface layer.
                    add_overlapping(intermediate_layers,   idx_layer_intermediate);
                    add_overlapping(interface_layers,      idx_layer_interface);
                    add_overlapping(base_interface_layers, idx_layer_base_interface);
                }
                // Order the layers by lexicographically by an increasing print_z and a decreasing layer height.
                std::stable_sort(layer_cache_item.overlapping.begin(), layer_cache_item.overlapping.end(), [](auto *l1, auto *l2) { return *l1 < *l2; });
            }
            assert(support_layer.support_islands.empty());
            if (! polys.empty()) {
                support_layer.support_islands = union_ex(polys);
                // support_layer.support_islands_bboxes.reserve(support_layer.support_islands.size());
                // for (const ExPolygon &expoly : support_layer.support_islands)
                //     support_layer.support_islands_bboxes.emplace_back(get_extents(expoly).inflated(SCALED_EPSILON));
            }
        } // for each support_layer_id
    });

    // Now modulate the support layer height in parallel.
    tbb::parallel_for(tbb::blocked_range<size_t>(n_raft_layers, support_layers.size()),
        [&support_layers, &layer_caches, &support_params, &bbox_object]
            (const tbb::blocked_range<size_t>& range) {
        for (size_t support_layer_id = range.begin(); support_layer_id < range.end(); ++ support_layer_id) {
            SupportLayer &support_layer = *support_layers[support_layer_id];
            LayerCache   &layer_cache   = layer_caches[support_layer_id];
            // For all extrusion types at this print_z, ordered by decreasing layer height:
            for (LayerCacheItem &layer_cache_item : layer_cache.nonempty) {
                // Trim the extrusion height from the bottom by the overlapping layers.
                modulate_extrusion_by_overlapping_layers(layer_cache_item.layer_extruded->extrusions, *layer_cache_item.layer_extruded->layer, layer_cache_item.overlapping);
                support_layer.support_fills.append(std::move(layer_cache_item.layer_extruded->extrusions));
            }
            if (! layer_cache.fine_body_extrusions.empty())
                (layer_cache.fine_body_beside_coarse ? support_layer.fine_body_fills : support_layer.support_fills)
                    .append(std::move(layer_cache.fine_body_extrusions));

            // Orca: Generate iron toolpath for contact layer
            if (!layer_cache.polys_to_iron.empty()) {
                auto f = std::unique_ptr<Fill>(Fill::new_from_type(support_params.ironing_pattern));
                f->set_bounding_box(bbox_object);
                f->layer_id        = support_layer.id();
                f->z               = support_layer.print_z;
                f->overlap         = 0;
                f->angle           = layer_cache.ironing_angle;
                f->spacing         = support_params.ironing_spacing;
                f->link_max_length = (coord_t) scale_(3. * f->spacing);

                ExPolygons polys_to_iron = union_safety_offset_ex(layer_cache.polys_to_iron);
                layer_cache.polys_to_iron.clear();

                // Find the layer above that directly overlaps current layer, clip the overlapped part
                if (support_layer_id < support_layers.size() - 1) {
                    const auto& upper_layer = support_layers[support_layer_id + 1];
                    if (!upper_layer->support_islands.empty() && upper_layer->bottom_z() <= support_layer.print_z + EPSILON) {
                        polys_to_iron = diff_ex(polys_to_iron, upper_layer->support_islands);
                    }
                }

                fill_expolygons_generate_paths(
                    // Destination
                    support_layer.support_fills.entities,
                    // Regions to fill
                    std::move(polys_to_iron),
                    // Filler and its parameters
                    f.get(), 1.f,
                    // Extrusion parameters
                    ExtrusionRole::erIroning, support_params.ironing_flow);
            }
        }
    });

#ifndef NDEBUG
    struct Test {
        static bool verify_nonempty(const ExtrusionEntityCollection *collection) {
            for (const ExtrusionEntity *ee : collection->entities) {
                if (const ExtrusionPath *path = dynamic_cast<const ExtrusionPath*>(ee))
                    assert(! path->empty());
                else if (const ExtrusionMultiPath *multipath = dynamic_cast<const ExtrusionMultiPath*>(ee))
                    assert(! multipath->empty());
                else if (const ExtrusionEntityCollection *eecol = dynamic_cast<const ExtrusionEntityCollection*>(ee)) {
                    assert(! eecol->empty());
                    return verify_nonempty(eecol);
                } else
                    assert(false);
            }
            return true;
        }
    };
    for (const SupportLayer *support_layer : support_layers)
        assert(Test::verify_nonempty(&support_layer->support_fills));
#endif // NDEBUG
}

/*
void PrintObjectSupportMaterial::clip_by_pillars(
    const PrintObject   &object,
    LayersPtr           &bottom_contacts,
    LayersPtr           &top_contacts,
    LayersPtr           &intermediate_contacts);

{
    // this prevents supplying an empty point set to BoundingBox constructor
    if (top_contacts.empty())
        return;

    coord_t pillar_size    = scale_(PILLAR_SIZE);
    coord_t pillar_spacing = scale_(PILLAR_SPACING);

    // A regular grid of pillars, filling the 2D bounding box.
    Polygons grid;
    {
        // Rectangle with a side of 2.5x2.5mm.
        Polygon pillar;
        pillar.points.push_back(Point(0, 0));
        pillar.points.push_back(Point(pillar_size, 0));
        pillar.points.push_back(Point(pillar_size, pillar_size));
        pillar.points.push_back(Point(0, pillar_size));

        // 2D bounding box of the projection of all contact polygons.
        BoundingBox bbox;
        for (LayersPtr::const_iterator it = top_contacts.begin(); it != top_contacts.end(); ++ it)
            bbox.merge(get_extents((*it)->polygons));
        grid.reserve(size_t(ceil(bb.size()(0) / pillar_spacing)) * size_t(ceil(bb.size()(1) / pillar_spacing)));
        for (coord_t x = bb.min(0); x <= bb.max(0) - pillar_size; x += pillar_spacing) {
            for (coord_t y = bb.min(1); y <= bb.max(1) - pillar_size; y += pillar_spacing) {
                grid.push_back(pillar);
                for (size_t i = 0; i < pillar.points.size(); ++ i)
                    grid.back().points[i].translate(Point(x, y));
            }
        }
    }

    // add pillars to every layer
    for my $i (0..n_support_z) {
        $shape->[$i] = [ @$grid ];
    }

    // build capitals
    for my $i (0..n_support_z) {
        my $z = $support_z->[$i];

        my $capitals = intersection(
            $grid,
            $contact->{$z} // [],
        );

        // work on one pillar at time (if any) to prevent the capitals from being merged
        // but store the contact area supported by the capital because we need to make
        // sure nothing is left
        my $contact_supported_by_capitals = [];
        foreach my $capital (@$capitals) {
            // enlarge capital tops
            $capital = offset([$capital], +($pillar_spacing - $pillar_size)/2);
            push @$contact_supported_by_capitals, @$capital;

            for (my $j = $i-1; $j >= 0; $j--) {
                my $jz = $support_z->[$j];
                $capital = offset($capital, -$self->interface_flow->scaled_width/2);
                last if !@$capitals;
                push @{ $shape->[$j] }, @$capital;
            }
        }

        // Capitals will not generally cover the whole contact area because there will be
        // remainders. For now we handle this situation by projecting such unsupported
        // areas to the ground, just like we would do with a normal support.
        my $contact_not_supported_by_capitals = diff(
            $contact->{$z} // [],
            $contact_supported_by_capitals,
        );
        if (@$contact_not_supported_by_capitals) {
            for (my $j = $i-1; $j >= 0; $j--) {
                push @{ $shape->[$j] }, @$contact_not_supported_by_capitals;
            }
        }
    }
}

sub clip_with_shape {
    my ($self, $support, $shape) = @_;

    foreach my $i (keys %$support) {
        // don't clip bottom layer with shape so that we
        // can generate a continuous base flange
        // also don't clip raft layers
        next if $i == 0;
        next if $i < $self->object_config->raft_layers;
        $support->{$i} = intersection(
            $support->{$i},
            $shape->[$i],
        );
    }
}
*/

} // namespace Slic3r
