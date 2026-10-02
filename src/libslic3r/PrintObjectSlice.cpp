#include <boost/log/trivial.hpp>

#include <cmath>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <tuple>

#include <tbb/parallel_for.h>

#include "ClipperUtils.hpp"
#include "ElephantFootCompensation.hpp"
#include "Exception.hpp"
#include "I18N.hpp"
#include "Layer.hpp"
#include "MixedNozzleConfig.hpp"
#include "MultiMaterialSegmentation.hpp"
#include "NativePaintPartition.hpp"
#include "Print.hpp"
//BBS
#include "ShortestPath.hpp"
#include "format.hpp"
#include "libslic3r/Feature/Interlocking/InterlockingGenerator.hpp"

//! macro used to mark string used at localization, return same string
#define L(s) Slic3r::I18N::translate(s)

namespace Slic3r {

bool PrintObject::clip_multipart_objects = true;
bool PrintObject::infill_only_where_needed = false;

// Z-contouring slices every layer but the first at its bottom plus the smallest zaa_min_z of the
// regions that ask for it. Nothing when no region asks for it. Body Split applies the same offset
// to each of its cells (NativeRegionalPlanningInput::contour_slice_offset).
static std::optional<coordf_t> z_contour_slice_offset(const PrintObject &print_object)
{
    std::optional<coordf_t> z_offset;
    size_t num_regions = print_object.num_printing_regions();
    for (size_t rid = 0; rid < num_regions; ++rid) {
        const auto& rcfg = print_object.printing_region(rid).config();
        if (rcfg.zaa_enabled) {
            if (!z_offset || rcfg.zaa_min_z < *z_offset)
                z_offset = rcfg.zaa_min_z;
        }
    }
    return z_offset;
}

static coordf_t compute_slice_z(PrintObject* print_object, size_t i_layer, coordf_t lo, coordf_t hi)
{
    const std::optional<coordf_t> z_offset = z_contour_slice_offset(*print_object);

    if (!z_offset || i_layer == 0) {
        return 0.5 * (lo + hi);
    }

    coordf_t slice_z = lo + *z_offset;
    if ((slice_z < lo && !is_approx(slice_z, lo)) || (slice_z > hi && !is_approx(slice_z, hi))) {
        throw RuntimeError("Bad min Z value");
    }
    return slice_z;
}

LayerPtrs new_layers(
    PrintObject                 *print_object,
    // Object layers (pairs of bottom/top Z coordinate), without the raft.
    const std::vector<coordf_t> &object_layers)
{
    LayerPtrs out;
    out.reserve(object_layers.size());
    auto     id   = int(print_object->slicing_parameters().raft_layers());
    coordf_t zmin = print_object->slicing_parameters().object_print_z_min;
    Layer   *prev = nullptr;
    for (size_t i_layer = 0; i_layer < object_layers.size(); i_layer += 2) {
        coordf_t lo = object_layers[i_layer];
        coordf_t hi = object_layers[i_layer + 1];
        coordf_t slice_z = compute_slice_z(print_object, i_layer, lo, hi);

        Layer *layer = new Layer(id ++, print_object, hi - lo, hi + zmin, slice_z);
        out.emplace_back(layer);
        if (prev != nullptr) {
            prev->upper_layer = layer;
            layer->lower_layer = prev;
        }
        prev = layer;
    }
    return out;
}

// Slice single triangle mesh.
static std::vector<ExPolygons> slice_volume(
    const ModelVolume             &volume,
    const std::vector<float>      &zs,
    const MeshSlicingParamsEx     &params,
    const std::function<void()>   &throw_on_cancel_callback)
{
    std::vector<ExPolygons> layers;
    if (! zs.empty()) {
        indexed_triangle_set its = volume.mesh().its;
        if (its.indices.size() > 0) {
            MeshSlicingParamsEx params2 { params };
            params2.trafo = params2.trafo * volume.get_matrix();
            if (params2.trafo.rotation().determinant() < 0.)
                its_flip_triangles(its);
            layers = slice_mesh_ex(its, zs, params2, throw_on_cancel_callback);
            throw_on_cancel_callback();
        }
    }
    return layers;
}

// Slice single triangle mesh.
// Filter the zs not inside the ranges. The ranges are closed at the bottom and open at the top, they are sorted lexicographically and non overlapping.
static std::vector<ExPolygons> slice_volume(
    const ModelVolume                           &volume,
    const std::vector<float>                    &z,
    const std::vector<t_layer_height_range>     &ranges,
    const MeshSlicingParamsEx                   &params,
    const std::function<void()>                 &throw_on_cancel_callback)
{
    std::vector<ExPolygons> out;
    if (! z.empty() && ! ranges.empty()) {
        if (ranges.size() == 1 && z.front() >= ranges.front().first && z.back() < ranges.front().second) {
            // All layers fit into a single range.
            out = slice_volume(volume, z, params, throw_on_cancel_callback);
        } else {
            std::vector<float>                     z_filtered;
            std::vector<std::pair<size_t, size_t>> n_filtered;
            z_filtered.reserve(z.size());
            n_filtered.reserve(2 * ranges.size());
            size_t i = 0;
            for (const t_layer_height_range &range : ranges) {
                for (; i < z.size() && z[i] < range.first; ++ i) ;
                size_t first = i;
                for (; i < z.size() && z[i] < range.second; ++ i)
                    z_filtered.emplace_back(z[i]);
                if (i > first)
                    n_filtered.emplace_back(std::make_pair(first, i));
            }
            if (! n_filtered.empty()) {
                std::vector<ExPolygons> layers = slice_volume(volume, z_filtered, params, throw_on_cancel_callback);
                out.assign(z.size(), ExPolygons());
                i = 0;
                for (const std::pair<size_t, size_t> &span : n_filtered)
                    for (size_t j = span.first; j < span.second; ++ j)
                        out[j] = std::move(layers[i ++]);
            }
        }
    }
    return out;
}
static inline bool model_volume_needs_slicing(const ModelVolume &mv)
{
    ModelVolumeType type = mv.type();
    return type == ModelVolumeType::MODEL_PART || type == ModelVolumeType::NEGATIVE_VOLUME || type == ModelVolumeType::PARAMETER_MODIFIER;
}

// Slice printable volumes, negative volumes and modifier volumes, sorted by ModelVolume::id().
// Apply closing radius.
// Apply positive XY compensation to ModelVolumeType::MODEL_PART and ModelVolumeType::PARAMETER_MODIFIER, not to ModelVolumeType::NEGATIVE_VOLUME.
// Apply contour simplification.
static std::vector<VolumeSlices> slice_volumes_inner(
    const PrintConfig                                        &print_config,
    const PrintObjectConfig                                  &print_object_config,
    const Transform3d                                        &object_trafo,
    ModelVolumePtrs                                           model_volumes,
    const std::vector<PrintObjectRegions::LayerRangeRegions> &layer_ranges,
    const std::vector<float>                                 &zs,
    const std::function<void()>                              &throw_on_cancel_callback)
{
    model_volumes_sort_by_id(model_volumes);

    std::vector<VolumeSlices> out;
    out.reserve(model_volumes.size());

    std::vector<t_layer_height_range> slicing_ranges;
    if (layer_ranges.size() > 1)
        slicing_ranges.reserve(layer_ranges.size());

    MeshSlicingParamsEx params_base;
    params_base.closing_radius = print_object_config.slice_closing_radius.value;
    params_base.extra_offset   = 0;
    params_base.trafo          = object_trafo;
    //BBS: 0.0025mm is safe enough to simplify the data to speed slicing up for high-resolution model.
    //Also has on influence on arc fitting which has default resolution 0.0125mm.
    params_base.resolution = print_config.resolution <= 0.001 ? 0.0f : 0.0025;
    switch (print_object_config.slicing_mode.value) {
    case SlicingMode::Regular:    params_base.mode = MeshSlicingParams::SlicingMode::Regular; break;
    case SlicingMode::EvenOdd:    params_base.mode = MeshSlicingParams::SlicingMode::EvenOdd; break;
    case SlicingMode::CloseHoles: params_base.mode = MeshSlicingParams::SlicingMode::Positive; break;
    }

    params_base.mode_below     = params_base.mode;

    // BBS
    const size_t num_extruders = print_config.filament_diameter.size();
    const bool   is_mm_painted = num_extruders > 1 && std::any_of(model_volumes.cbegin(), model_volumes.cend(), [](const ModelVolume *mv) { return mv->is_mm_painted(); });
    // BBS: don't do size compensation when slice volume.
    // Will handle contour and hole size compensation seperately later.
    //const auto   extra_offset  = is_mm_painted ? 0.f : std::max(0.f, float(print_object_config.xy_contour_compensation.value));
    const auto   extra_offset = 0.f;

    for (const ModelVolume *model_volume : model_volumes)
        if (model_volume_needs_slicing(*model_volume)) {
            MeshSlicingParamsEx params { params_base };
            if (! model_volume->is_negative_volume())
                params.extra_offset = extra_offset;
            if (layer_ranges.size() == 1) {
                if (const PrintObjectRegions::LayerRangeRegions &layer_range = layer_ranges.front(); layer_range.has_volume(model_volume->id())) {
                    if (model_volume->is_model_part() && print_config.spiral_mode) {
                        auto it = std::find_if(layer_range.volume_regions.begin(), layer_range.volume_regions.end(),
                            [model_volume](const auto &slice){ return model_volume == slice.model_volume; });
                        params.mode = MeshSlicingParams::SlicingMode::PositiveLargestContour;
                        // Slice the bottom layers with SlicingMode::Regular.
                        // This needs to be in sync with LayerRegion::make_perimeters() spiral_mode!
                        const PrintRegionConfig &region_config = it->region->config();
                        params.slicing_mode_normal_below_layer = size_t(region_config.bottom_shell_layers.value);
                        for (; params.slicing_mode_normal_below_layer < zs.size() && zs[params.slicing_mode_normal_below_layer] < region_config.bottom_shell_thickness - EPSILON;
                            ++ params.slicing_mode_normal_below_layer);
                    }
                    out.push_back({
                        model_volume->id(),
                        slice_volume(*model_volume, zs, params, throw_on_cancel_callback)
                    });
                }
            } else {
                assert(! print_config.spiral_mode);
                slicing_ranges.clear();
                for (const PrintObjectRegions::LayerRangeRegions &layer_range : layer_ranges)
                    if (layer_range.has_volume(model_volume->id()))
                        slicing_ranges.emplace_back(layer_range.layer_height_range);
                if (! slicing_ranges.empty())
                    out.push_back({
                        model_volume->id(),
                        slice_volume(*model_volume, zs, slicing_ranges, params, throw_on_cancel_callback)
                    });
            }
            if (! out.empty() && out.back().slices.empty())
                out.pop_back();
        }

    return out;
}

static inline VolumeSlices& volume_slices_find_by_id(std::vector<VolumeSlices> &volume_slices, const ObjectID id)
{
    auto it = lower_bound_by_predicate(volume_slices.begin(), volume_slices.end(), [id](const VolumeSlices &vs) { return vs.volume_id < id; });
    assert(it != volume_slices.end() && it->volume_id == id);
    return *it;
}

static inline bool overlap_in_xy(const PrintObjectRegions::BoundingBox &l, const PrintObjectRegions::BoundingBox &r)
{
    return ! (l.max().x() < r.min().x() || l.min().x() > r.max().x() ||
              l.max().y() < r.min().y() || l.min().y() > r.max().y());
}

static std::vector<PrintObjectRegions::LayerRangeRegions>::const_iterator layer_range_first(const std::vector<PrintObjectRegions::LayerRangeRegions> &layer_ranges, double z)
{
    auto  it = lower_bound_by_predicate(layer_ranges.begin(), layer_ranges.end(),
        [z](const PrintObjectRegions::LayerRangeRegions &lr) {
            return lr.layer_height_range.second < z && abs(lr.layer_height_range.second - z) > EPSILON;
        });
    assert(it != layer_ranges.end() && it->layer_height_range.first <= z && z <= it->layer_height_range.second);
    if (z == it->layer_height_range.second)
        if (auto it_next = it; ++ it_next != layer_ranges.end() && it_next->layer_height_range.first == z)
            it = it_next;
    assert(it != layer_ranges.end() && it->layer_height_range.first <= z && z <= it->layer_height_range.second);
    return it;
}

static std::vector<PrintObjectRegions::LayerRangeRegions>::const_iterator layer_range_next(
    const std::vector<PrintObjectRegions::LayerRangeRegions>            &layer_ranges,
    std::vector<PrintObjectRegions::LayerRangeRegions>::const_iterator   it,
    double                                                               z)
{
    for (; it->layer_height_range.second <= z + EPSILON; ++ it)
        assert(it != layer_ranges.end());
    assert(it != layer_ranges.end() && it->layer_height_range.first <= z && z < it->layer_height_range.second);
    return it;
}

static std::vector<std::vector<ExPolygons>> slices_to_regions(
    const PrintConfig                                        &print_config,
    const PrintObject                                        &print_object,
    ModelVolumePtrs                                           model_volumes,
    const PrintObjectRegions                                 &print_object_regions,
    const std::vector<float>                                 &zs,
    std::vector<VolumeSlices>                               &&volume_slices,
    // If clipping is disabled, then ExPolygons produced by different volumes will never be merged, thus they will be allowed to overlap.
    // It is up to the model designer to handle these overlaps.
    const bool                                                clip_multipart_objects,
    const std::function<void()>                              &throw_on_cancel_callback)
{
    model_volumes_sort_by_id(model_volumes);

    std::vector<std::vector<ExPolygons>> slices_by_region(print_object_regions.all_regions.size(), std::vector<ExPolygons>(zs.size(), ExPolygons()));

    // First shuffle slices into regions if there is no overlap with another region possible, collect zs of the complex cases.
    std::vector<std::pair<size_t, float>> zs_complex;
    {
        size_t z_idx = 0;
        for (const PrintObjectRegions::LayerRangeRegions &layer_range : print_object_regions.layer_ranges) {
            for (; z_idx < zs.size() && zs[z_idx] < layer_range.layer_height_range.first; ++ z_idx) ;
            if (layer_range.volume_regions.empty()) {
            } else if (layer_range.volume_regions.size() == 1) {
                const ModelVolume *model_volume = layer_range.volume_regions.front().model_volume;
                assert(model_volume != nullptr);
                if (model_volume->is_model_part()) {
                    VolumeSlices &slices_src = volume_slices_find_by_id(volume_slices, model_volume->id());
                    auto         &slices_dst = slices_by_region[layer_range.volume_regions.front().region->print_object_region_id()];
                    for (; z_idx < zs.size() && zs[z_idx] < layer_range.layer_height_range.second; ++ z_idx)
                        slices_dst[z_idx] = std::move(slices_src.slices[z_idx]);
                }
            } else {
                zs_complex.reserve(zs.size());
                for (; z_idx < zs.size() && zs[z_idx] < layer_range.layer_height_range.second; ++ z_idx) {
                    float z                          = zs[z_idx];
                    int   idx_first_printable_region = -1;
                    bool  complex                    = false;
                    // Every model-part region whose Z range spans z is a candidate, not just the first one
                    // found: several can coexist with disjoint XY footprints (for example a Body Split
                    // object's bodies). Only a real XY overlap between some pair needs the boolean-clipping
                    // path below.
                    std::vector<int> printable_region_indices;
                    for (int idx_region = 0; idx_region < int(layer_range.volume_regions.size()); ++ idx_region) {
                        const PrintObjectRegions::VolumeRegion &region = layer_range.volume_regions[idx_region];
                        if (region.bbox->min().z() <= z && region.bbox->max().z() >= z) {
                            if (idx_first_printable_region == -1 && region.model_volume->is_model_part())
                                idx_first_printable_region = idx_region;
                            else if (idx_first_printable_region != -1) {
                                // Test for overlap with some other region.
                                for (int idx_region2 = idx_first_printable_region; idx_region2 < idx_region; ++ idx_region2) {
                                    const PrintObjectRegions::VolumeRegion &region2 = layer_range.volume_regions[idx_region2];
                                    if (region2.bbox->min().z() <= z && region2.bbox->max().z() >= z && overlap_in_xy(*region.bbox, *region2.bbox)) {
                                        complex = true;
                                        break;
                                    }
                                }
                            }
                            if (region.model_volume->is_model_part())
                                printable_region_indices.push_back(idx_region);
                        }
                    }
                    if (complex)
                        zs_complex.push_back({ z_idx, z });
                    else
                        for (int idx_region : printable_region_indices) {
                            const PrintObjectRegions::VolumeRegion &region = layer_range.volume_regions[idx_region];
                            slices_by_region[region.region->print_object_region_id()][z_idx] = std::move(volume_slices_find_by_id(volume_slices, region.model_volume->id()).slices[z_idx]);
                        }
                }
            }
            throw_on_cancel_callback();
        }
    }

    // Second perform region clipping and assignment in parallel.
    if (! zs_complex.empty()) {
        std::vector<std::vector<VolumeSlices*>> layer_ranges_regions_to_slices(print_object_regions.layer_ranges.size(), std::vector<VolumeSlices*>());
        for (const PrintObjectRegions::LayerRangeRegions &layer_range : print_object_regions.layer_ranges) {
            std::vector<VolumeSlices*> &layer_range_regions_to_slices = layer_ranges_regions_to_slices[&layer_range - print_object_regions.layer_ranges.data()];
            layer_range_regions_to_slices.reserve(layer_range.volume_regions.size());
            for (const PrintObjectRegions::VolumeRegion &region : layer_range.volume_regions)
                layer_range_regions_to_slices.push_back(&volume_slices_find_by_id(volume_slices, region.model_volume->id()));
        }
        tbb::parallel_for(
            tbb::blocked_range<size_t>(0, zs_complex.size()),
            [&slices_by_region, &print_object_regions, &zs_complex, &layer_ranges_regions_to_slices, clip_multipart_objects, &throw_on_cancel_callback]
                (const tbb::blocked_range<size_t> &range) {
                float z              = zs_complex[range.begin()].second;
                auto  it_layer_range = layer_range_first(print_object_regions.layer_ranges, z);
                // Per volume_regions slices at this Z height.
                struct RegionSlice {
                    ExPolygons  expolygons;
                    // Identifier of this region in PrintObjectRegions::all_regions
                    int         region_id;
                    ObjectID    volume_id;
                    bool operator<(const RegionSlice &rhs) const {
                        bool this_empty = this->region_id < 0 || this->expolygons.empty();
                        bool rhs_empty  = rhs.region_id < 0 || rhs.expolygons.empty();
                        // Sort the empty items to the end of the list.
                        // Sort by region_id & volume_id lexicographically.
                        return ! this_empty && (rhs_empty || (this->region_id < rhs.region_id || (this->region_id == rhs.region_id && volume_id < rhs.volume_id)));
                    }
                };

                // BBS
                auto trim_overlap = [](ExPolygons& expolys_a, ExPolygons& expolys_b) {
                    ExPolygons trimming_a;
                    ExPolygons trimming_b;

                    for (ExPolygon& expoly_a : expolys_a) {
                        BoundingBox bbox_a = get_extents(expoly_a);
                        ExPolygons expolys_new;
                        for (ExPolygon& expoly_b : expolys_b) {
                            BoundingBox bbox_b = get_extents(expoly_b);
                            if (!bbox_a.overlap(bbox_b))
                                continue;

                            ExPolygons temp = intersection_ex(expoly_b, expoly_a, ApplySafetyOffset::Yes);
                            if (temp.empty())
                                continue;

                            if (expoly_a.contour.length() > expoly_b.contour.length())
                                trimming_a.insert(trimming_a.end(), temp.begin(), temp.end());
                            else
                                trimming_b.insert(trimming_b.end(), temp.begin(), temp.end());
                        }
                    }

                    expolys_a = diff_ex(expolys_a, trimming_a);
                    expolys_b = diff_ex(expolys_b, trimming_b);
                };

                std::vector<RegionSlice> temp_slices;
                for (size_t zs_complex_idx = range.begin(); zs_complex_idx < range.end(); ++ zs_complex_idx) {
                    auto [z_idx, z] = zs_complex[zs_complex_idx];
                    it_layer_range = layer_range_next(print_object_regions.layer_ranges, it_layer_range, z);
                    const PrintObjectRegions::LayerRangeRegions &layer_range = *it_layer_range;
                    {
                        std::vector<VolumeSlices*> &layer_range_regions_to_slices = layer_ranges_regions_to_slices[it_layer_range - print_object_regions.layer_ranges.begin()];
                        // Per volume_regions slices at thiz Z height.
                        temp_slices.clear();
                        temp_slices.reserve(layer_range.volume_regions.size());
                        for (VolumeSlices* &slices : layer_range_regions_to_slices) {
                            const PrintObjectRegions::VolumeRegion &volume_region = layer_range.volume_regions[&slices - layer_range_regions_to_slices.data()];
                            temp_slices.push_back({ std::move(slices->slices[z_idx]), volume_region.region ? volume_region.region->print_object_region_id() : -1, volume_region.model_volume->id() });
                        }
                    }
                    for (int idx_region = 0; idx_region < int(layer_range.volume_regions.size()); ++ idx_region)
                        if (! temp_slices[idx_region].expolygons.empty()) {
                            const PrintObjectRegions::VolumeRegion &region = layer_range.volume_regions[idx_region];
                            if (region.model_volume->is_modifier()) {
                                assert(region.parent > -1);
                                bool next_region_same_modifier = idx_region + 1 < int(temp_slices.size()) && layer_range.volume_regions[idx_region + 1].model_volume == region.model_volume;
                                RegionSlice &parent_slice = temp_slices[region.parent];
                                RegionSlice &this_slice   = temp_slices[idx_region];
                                ExPolygons   source       = std::move(this_slice.expolygons);
                                if (parent_slice.expolygons.empty()) {
                                    this_slice  .expolygons.clear();
                                } else {
                                    this_slice  .expolygons = intersection_ex(parent_slice.expolygons, source);
                                    parent_slice.expolygons = diff_ex        (parent_slice.expolygons, source);
                                }
                                if (next_region_same_modifier)
                                    // To be used in the following iteration.
                                    temp_slices[idx_region + 1].expolygons = std::move(source);
                            } else if ((region.model_volume->is_model_part() && clip_multipart_objects) || region.model_volume->is_negative_volume()) {
                                // Clip every non-zero region preceding it.
                                for (int idx_region2 = 0; idx_region2 < idx_region; ++ idx_region2)
                                    if (! temp_slices[idx_region2].expolygons.empty()) {
                                        // Skip trim_overlap for now, because it slow down the performace so much for some special cases
#if 1
                                        if (const PrintObjectRegions::VolumeRegion& region2 = layer_range.volume_regions[idx_region2];
                                            !region2.model_volume->is_negative_volume() && overlap_in_xy(*region.bbox, *region2.bbox))
                                            temp_slices[idx_region2].expolygons = diff_ex(temp_slices[idx_region2].expolygons, temp_slices[idx_region].expolygons);
#else
                                        const PrintObjectRegions::VolumeRegion& region2 = layer_range.volume_regions[idx_region2];
                                        if (!region2.model_volume->is_negative_volume() && overlap_in_xy(*region.bbox, *region2.bbox))
                                            //BBS: handle negative_volume seperately, always minus the negative volume and don't need to trim overlap
                                            if (!region.model_volume->is_negative_volume())
                                                trim_overlap(temp_slices[idx_region2].expolygons, temp_slices[idx_region].expolygons);
                                            else
                                                temp_slices[idx_region2].expolygons = diff_ex(temp_slices[idx_region2].expolygons, temp_slices[idx_region].expolygons);
#endif
                                    }
                            }
                        }
                    // Sort by region_id, push empty slices to the end.
                    std::sort(temp_slices.begin(), temp_slices.end());
                    // Remove the empty slices.
                    temp_slices.erase(std::find_if(temp_slices.begin(), temp_slices.end(), [](const auto &slice) { return slice.region_id == -1 || slice.expolygons.empty(); }), temp_slices.end());
                    // Merge slices and store them to the output.
                    for (int i = 0; i < int(temp_slices.size());) {
                        // Find a range of temp_slices with the same region_id.
                        int j = i;
                        bool merged = false;
                        ExPolygons &expolygons = temp_slices[i].expolygons;
                        for (++ j; j < int(temp_slices.size()) && temp_slices[i].region_id == temp_slices[j].region_id; ++ j)
                            if (ExPolygons &expolygons2 = temp_slices[j].expolygons; ! expolygons2.empty()) {
                                if (expolygons.empty()) {
                                    expolygons = std::move(expolygons2);
                                } else {
                                    append(expolygons, std::move(expolygons2));
                                    merged = true;
                                }
                            }
                        // Don't unite the regions if ! clip_multipart_objects. In that case it is user's responsibility
                        // to handle region overlaps. Indeed, one may intentionally let the regions overlap to produce crossing perimeters
                        // for example.
                        if (merged && clip_multipart_objects)
                            expolygons = closing_ex(expolygons, float(scale_(EPSILON)));
                        slices_by_region[temp_slices[i].region_id][z_idx] = std::move(expolygons);
                        i = j;
                    }
                    throw_on_cancel_callback();
                }
            });
    }

    return slices_by_region;
}

//BBS: justify whether a volume is connected to another one
bool doesVolumeIntersect(VolumeSlices& vs1, VolumeSlices& vs2)
{
    if (vs1.volume_id == vs2.volume_id) return true;
    // two volumes in the same object should have same number of layers, otherwise the slicing is incorrect.
    if (vs1.slices.size() != vs2.slices.size()) return false;

    auto& vs1s = vs1.slices;
    auto& vs2s = vs2.slices;
    bool is_intersect = false;

    tbb::parallel_for(tbb::blocked_range<int>(0, vs1s.size()),
        [&vs1s, &vs2s, &is_intersect](const tbb::blocked_range<int>& range) {
            for (auto i = range.begin(); i != range.end(); ++i) {
                if (vs1s[i].empty()) continue;

                if (overlaps(vs1s[i], vs2s[i])) {
                    is_intersect = true;
                    break;
                }
                if (i + 1 != vs2s.size() && overlaps(vs1s[i], vs2s[i + 1])) {
                    is_intersect = true;
                    break;
                }
                if (i - 1 >= 0 && overlaps(vs1s[i], vs2s[i - 1])) {
                    is_intersect = true;
                    break;
                }
            }
        });
    return is_intersect;
}

//BBS: grouping the volumes of an object according to their connection relationship
bool groupingVolumes(std::vector<VolumeSlices> objSliceByVolume, std::vector<groupedVolumeSlices>& groups, double resolution, int firstLayerReplacedBy)
{
    std::vector<int> groupIndex(objSliceByVolume.size(), -1);
    double offsetValue = 0.05 / SCALING_FACTOR;

    std::vector<std::vector<int>> osvIndex;
    for (int i = 0; i != objSliceByVolume.size(); ++i) {
        for (int j = 0; j != objSliceByVolume[i].slices.size(); ++j) {
            osvIndex.push_back({ i,j });
        }
    }

    tbb::parallel_for(tbb::blocked_range<int>(0, osvIndex.size()),
        [&osvIndex, &objSliceByVolume, &offsetValue, &resolution](const tbb::blocked_range<int>& range) {
            for (auto k = range.begin(); k != range.end(); ++k) {
                for (ExPolygon& poly_ex : objSliceByVolume[osvIndex[k][0]].slices[osvIndex[k][1]])
                    poly_ex.douglas_peucker(resolution);
            }
        });

    tbb::parallel_for(tbb::blocked_range<int>(0, osvIndex.size()),
        [&osvIndex, &objSliceByVolume,&offsetValue, &resolution](const tbb::blocked_range<int>& range) {
            for (auto k = range.begin(); k != range.end(); ++k) {
                objSliceByVolume[osvIndex[k][0]].slices[osvIndex[k][1]] = offset_ex(objSliceByVolume[osvIndex[k][0]].slices[osvIndex[k][1]], offsetValue);
            }
        });

    for (int i = 0; i != objSliceByVolume.size(); ++i) {
        if (groupIndex[i] < 0) {
            groupIndex[i] = i;
        }
        for (int j = i + 1; j != objSliceByVolume.size(); ++j) {
            if (doesVolumeIntersect(objSliceByVolume[i], objSliceByVolume[j])) {
                if (groupIndex[j] < 0) groupIndex[j] = groupIndex[i];
                if (groupIndex[j] != groupIndex[i]) {
                    int retain = std::min(groupIndex[i], groupIndex[j]);
                    int cover = std::max(groupIndex[i], groupIndex[j]);
                    for (int k = 0; k != objSliceByVolume.size(); ++k) {
                        if (groupIndex[k] == cover) groupIndex[k] = retain;
                    }
                }
            }

        }
    }

    std::vector<int> groupVector{};
    for (int gi : groupIndex) {
        bool exist = false;
        for (int gv : groupVector) {
            if (gv == gi) {
                exist = true;
                break;
            }
        }
        if (!exist) groupVector.push_back(gi);
    }

    // group volumes and their slices according to the grouping Vector
    groups.clear();

    for (int gv : groupVector) {
        groupedVolumeSlices gvs;
        gvs.groupId = gv;
        for (int i = 0; i != objSliceByVolume.size(); ++i) {
            if (groupIndex[i] == gv) {
                gvs.volume_ids.push_back(objSliceByVolume[i].volume_id);
                append(gvs.slices, objSliceByVolume[i].slices[firstLayerReplacedBy]);
            }
        }

        // the slices of a group should be unioned
        gvs.slices = offset_ex(union_ex(gvs.slices), -offsetValue);
        for (ExPolygon& poly_ex : gvs.slices)
            poly_ex.douglas_peucker(resolution);

        groups.push_back(gvs);
    }
    return true;
}

//BBS: filter the members of "objSliceByVolume" such that only "model_part" are included
std::vector<VolumeSlices> findPartVolumes(const std::vector<VolumeSlices>& objSliceByVolume, ModelVolumePtrs model_volumes) {
    std::vector<VolumeSlices> outPut;
    for (const auto& vs : objSliceByVolume) {
        for (const auto& mv : model_volumes) {
            if (vs.volume_id == mv->id() && mv->is_model_part()) outPut.push_back(vs);
        }
    }
    return outPut;
}

void applyNegtiveVolumes(ModelVolumePtrs model_volumes, const std::vector<VolumeSlices>& objSliceByVolume, std::vector<groupedVolumeSlices>& groups, double resolution) {
    ExPolygons negTotal;
    for (const auto& vs : objSliceByVolume) {
        for (const auto& mv : model_volumes) {
            if (vs.volume_id == mv->id() && mv->is_negative_volume()) {
                if (vs.slices.size() > 0) {
                    append(negTotal, vs.slices.front());
                }
            }
        }
    }

    for (auto& g : groups) {
        g.slices = diff_ex(g.slices, negTotal);
        for (ExPolygon& poly_ex : g.slices)
            poly_ex.douglas_peucker(resolution);
    }
}

void reGroupingLayerPolygons(std::vector<groupedVolumeSlices>& gvss, ExPolygons &eps, double resolution)
{
    std::vector<int> epsIndex;
    epsIndex.resize(eps.size(), -1);

    auto gvssc = gvss;
    auto epsc = eps;

    for (ExPolygon& poly_ex : epsc)
        poly_ex.douglas_peucker(resolution);

    for (int i = 0; i != gvssc.size(); ++i) {
        for (ExPolygon& poly_ex : gvssc[i].slices)
            poly_ex.douglas_peucker(resolution);
    }

    tbb::parallel_for(tbb::blocked_range<int>(0, epsc.size()),
        [&epsc, &gvssc, &epsIndex](const tbb::blocked_range<int>& range) {
            for (auto ie = range.begin(); ie != range.end(); ++ie) {
                if (epsc[ie].area() <= 0)
                    continue;

                double minArea = epsc[ie].area();
                for (int iv = 0; iv != gvssc.size(); iv++) {
                    auto clipedExPolys = diff_ex(epsc[ie], gvssc[iv].slices);
                    double area = 0;
                    for (const auto& ce : clipedExPolys) {
                        area += ce.area();
                    }
                    if (area < minArea) {
                        minArea = area;
                        epsIndex[ie] = iv;
                    }
                }
            }
        });

    for (int iv = 0; iv != gvss.size(); iv++)
        gvss[iv].slices.clear();

    for (int ie = 0; ie != eps.size(); ie++) {
        if (epsIndex[ie] >= 0)
            gvss[epsIndex[ie]].slices.push_back(eps[ie]);
    }
}

/*
std::string fix_slicing_errors(PrintObject* object, LayerPtrs &layers, const std::function<void()> &throw_if_canceled, int &firstLayerReplacedBy)
{
    std::string error_msg;//BBS

    if (layers.size() == 0) return error_msg;

    // Collect layers with slicing errors.
    // These layers will be fixed in parallel.
    std::vector<size_t> buggy_layers;
    buggy_layers.reserve(layers.size());
    // BBS: get largest external perimenter width of all layers
    auto get_ext_peri_width = [](Layer* layer) {return layer->m_regions.empty() ? 0 : layer->m_regions[0]->flow(frExternalPerimeter).scaled_width(); };
    auto it = std::max_element(layers.begin(), layers.end(), [get_ext_peri_width](auto& a, auto& b) {return get_ext_peri_width(a) < get_ext_peri_width(b); });
    coord_t thresh = get_ext_peri_width(*it) * 0.5;// half of external perimeter width  // 0.5 * scale_(this->config().line_width);
    for (size_t idx_layer = 0; idx_layer < layers.size(); ++idx_layer) {
        // BBS: detect empty layers (layers with very small regions) and mark them as problematic, then these layers will copy the nearest good layer
        auto layer = layers[idx_layer];
        ExPolygons lslices;
        for (size_t region_id = 0; region_id < layer->m_regions.size(); ++region_id) {
            LayerRegion* layerm = layer->m_regions[region_id];
            for (auto& surface : layerm->slices.surfaces) {
                auto expoly = offset_ex(surface.expolygon, -thresh);
                lslices.insert(lslices.begin(), expoly.begin(), expoly.end());
            }
        }
        if (lslices.empty()) {
            layer->slicing_errors = true;
        }

        if (layers[idx_layer]->slicing_errors) {
            buggy_layers.push_back(idx_layer);
        }
        else
            break; // only detect empty layers near bed
    }

    BOOST_LOG_TRIVIAL(debug) << "Slicing objects - fixing slicing errors in parallel - begin";
    std::atomic<bool> is_replaced = false;
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, buggy_layers.size()),
        [&layers, &throw_if_canceled, &buggy_layers, &is_replaced](const tbb::blocked_range<size_t>& range) {
            for (size_t buggy_layer_idx = range.begin(); buggy_layer_idx < range.end(); ++ buggy_layer_idx) {
                throw_if_canceled();
                size_t idx_layer = buggy_layers[buggy_layer_idx];
                // BBS: only replace empty layers lower than 1mm
                const coordf_t thresh_empty_layer_height = 1;
                Layer* layer = layers[idx_layer];
                if (layer->print_z>= thresh_empty_layer_height)
                    continue;
                assert(layer->slicing_errors);
                // Try to repair the layer surfaces by merging all contours and all holes from neighbor layers.
                // BOOST_LOG_TRIVIAL(trace) << "Attempting to repair layer" << idx_layer;
                for (size_t region_id = 0; region_id < layer->region_count(); ++ region_id) {
                    LayerRegion *layerm = layer->get_region(region_id);
                    // Find the first valid layer below / above the current layer.
                    const Surfaces *upper_surfaces = nullptr;
                    const Surfaces *lower_surfaces = nullptr;
                    //BBS: only repair empty layers lowers than 1mm
                    for (size_t j = idx_layer + 1; j < layers.size(); ++j) {
                        if (!layers[j]->slicing_errors) {
                            upper_surfaces = &layers[j]->regions()[region_id]->slices.surfaces;
                            break;
                        }
                        if (layers[j]->print_z >= thresh_empty_layer_height) break;
                    }
                    for (int j = int(idx_layer) - 1; j >= 0; --j) {
                        if (layers[j]->print_z >= thresh_empty_layer_height) continue;
                        if (!layers[j]->slicing_errors) {
                            lower_surfaces = &layers[j]->regions()[region_id]->slices.surfaces;
                            break;
                        }
                    }
                    // Collect outer contours and holes from the valid layers above & below.
                    ExPolygons expolys;
                    expolys.reserve(
                        ((upper_surfaces == nullptr) ? 0 : upper_surfaces->size()) +
                        ((lower_surfaces == nullptr) ? 0 : lower_surfaces->size()));
                    if (upper_surfaces)
                        for (const auto &surface : *upper_surfaces) {
                            expolys.emplace_back(surface.expolygon);
                        }
                    if (lower_surfaces)
                        for (const auto &surface : *lower_surfaces) {
                            expolys.emplace_back(surface.expolygon);
                        }
                    if (!expolys.empty()) {
                        //BBS
                        is_replaced = true;
                        layerm->slices.set(union_ex(expolys), stInternal);
                    }
                }
                // Update layer slices after repairing the single regions.
                layer->make_slices();
            }
        });
    throw_if_canceled();
    BOOST_LOG_TRIVIAL(debug) << "Slicing objects - fixing slicing errors in parallel - end";

    if(is_replaced)
        error_msg = L("Empty layers around bottom are replaced by nearest normal layers.");

    // remove empty layers from bottom
    while (! layers.empty() && (layers.front()->lslices.empty() || layers.front()->empty())) {
        delete layers.front();
        layers.erase(layers.begin());
        layers.front()->lower_layer = nullptr;
        for (size_t i = 0; i < layers.size(); ++ i)
            layers[i]->set_id(layers[i]->id() - 1);
    }

    //BBS
    if(error_msg.empty() && !buggy_layers.empty())
        error_msg = L("The model has too many empty layers.");

    // BBS: first layer slices are sorted by volume group, if the first layer is empty and replaced by the 2nd layer
// the later will be stored in "object->firstLayerObjGroupsMod()"
    if (!buggy_layers.empty() && buggy_layers.front() == 0 && layers.size() > 1)
        firstLayerReplacedBy = 1;

    return error_msg;
}
*/

void groupingVolumesForBrim(PrintObject* object, LayerPtrs& layers, int firstLayerReplacedBy)
{
    const auto           scaled_resolution = scaled<double>(object->print()->config().resolution.value);
    auto partsObjSliceByVolume = findPartVolumes(object->firstLayerObjSliceMod(), object->model_object()->volumes);
    groupingVolumes(partsObjSliceByVolume, object->firstLayerObjGroupsMod(), scaled_resolution, firstLayerReplacedBy);
    applyNegtiveVolumes(object->model_object()->volumes, object->firstLayerObjSliceMod(), object->firstLayerObjGroupsMod(), scaled_resolution);

    // BBS: the actual first layer slices stored in layers are re-sorted by volume group and will be used to generate brim
    // A Body Split coarse body whose first cell stands on the bed but prints on a later event layer
    // leaves layer 0 without slices of its own, and re-grouping would leave the object with no brim.
    // Keep the groups the volumes were sliced into at the first layer. With any slices on layer 0
    // this is the upstream re-grouping.
    if (!layers.empty() && !layers.front()->lslices.empty())
        reGroupingLayerPolygons(object->firstLayerObjGroupsMod(), layers.front()->lslices, scaled_resolution);
}

// Called by make_perimeters()
// Body Split counterpart of apply_fuzzy_skin_segmentation(). The grid is already fixed and a fuzzy
// painted sibling has no cells, so the painted band is recorded on the host cell instead
// (LayerRegion::fuzzy_skin_painted_masks) and Layer::make_perimeters() passes it to the perimeter
// generator as a companion region, the same merge stock Orca does. Only the external wall across
// the painted band is roughened.
template<typename ThrowOnCancel>
void apply_body_split_fuzzy_skin_masks(PrintObject &print_object, ThrowOnCancel throw_on_cancel)
{
    // Projected on the event layers. Each cell is hosted on the event layer at its top, whose
    // slice plane lies inside the cell.
    std::vector<std::vector<ExPolygons>> segmentation = fuzzy_skin_segmentation_by_painting(print_object, throw_on_cancel);
    assert(segmentation.size() == print_object.layer_count());

    tbb::parallel_for(tbb::blocked_range<size_t>(0, segmentation.size()), [&print_object, &segmentation, throw_on_cancel](const tbb::blocked_range<size_t> &range) {
        const auto &layer_ranges = print_object.shared_regions()->layer_ranges;
        for (size_t layer_idx = range.begin(); layer_idx < range.end(); ++layer_idx) {
            throw_on_cancel();
            if (segmentation[layer_idx].empty() || segmentation[layer_idx].front().empty())
                continue;
            const ExPolygons &painted = segmentation[layer_idx].front();
            Layer &layer = *print_object.get_layer(int(layer_idx));
            for (int region_idx = 0; region_idx < layer.region_count(); ++region_idx) {
                LayerRegion &host = *layer.get_region(region_idx);
                if (host.slices.empty())
                    continue;
                const double z = host.slice_z();
                const auto layer_range = std::find_if(layer_ranges.begin(), layer_ranges.end(),
                    [z](const PrintObjectRegions::LayerRangeRegions &candidate) {
                        return z >= candidate.layer_height_range.first - EPSILON &&
                               z <= candidate.layer_height_range.second + EPSILON;
                    });
                if (layer_range == layer_ranges.end())
                    continue;
                const auto fuzzy = std::find_if(layer_range->fuzzy_skin_painted_regions.begin(),
                                                layer_range->fuzzy_skin_painted_regions.end(),
                    [&](const PrintObjectRegions::FuzzySkinPaintedRegion &candidate) {
                        return candidate.parent_print_object_region_id(*layer_range) == region_idx;
                    });
                // Native Orca skips a sibling identical to its parent (fuzzy skin disabled).
                if (fuzzy == layer_range->fuzzy_skin_painted_regions.end() || fuzzy->region == &host.region())
                    continue;
                ExPolygons mask = intersection_ex(host.slices.surfaces, painted);
                if (!mask.empty())
                    host.fuzzy_skin_painted_masks.emplace_back(fuzzy->region, std::move(mask));
            }
        }
    });
}

// 1) Decides Z positions of the layers,
// 2) Initializes layers and their regions
// 3) Slices the object meshes
// 4) Slices the modifier meshes and reclassifies the slices of the object meshes by the slices of the modifier meshes
// 5) Applies size compensation (offsets the slices in XY plane)
// 6) Replaces bad slices by the slices reconstructed from the upper/lower layer
// Resulting expolygons of layer regions are marked as Internal.
void PrintObject::slice()
{
    if (! this->set_started(posSlice))
        return;
    //BBS: add flag to reload scene for shell rendering
    m_print->set_status(5, L("Slicing mesh"), PrintBase::SlicingStatus::RELOAD_SCENE);
    std::vector<coordf_t> layer_height_profile;
    this->update_layer_height_profile(*this->model_object(), m_slicing_params, layer_height_profile);
    m_print->throw_if_canceled();
    m_typed_slices = false;
    this->clear_layers();
    m_layers = new_layers(this, generate_object_layers(m_slicing_params, layer_height_profile, m_config.precise_z_height.value));
    this->slice_volumes();
    m_print->throw_if_canceled();
    int firstLayerReplacedBy = 0;

#if 0
    // Fix the model.
    //FIXME is this the right place to do? It is done repeateadly at the UI and now here at the backend.
    std::string warning = fix_slicing_errors(this, m_layers, [this](){ m_print->throw_if_canceled(); }, firstLayerReplacedBy);
    m_print->throw_if_canceled();
    //BBS: send warning message to slicing callback
    // This warning is inaccurate, because the empty layers may have been replaced, or the model has supports.
    //if (!warning.empty()) {
    //    BOOST_LOG_TRIVIAL(info) << warning;
    //    this->active_step_add_warning(PrintStateBase::WarningLevel::CRITICAL, warning, PrintStateBase::SlicingReplaceInitEmptyLayers);
    //}
#endif

    // Detect and process holes that should be converted to polyholes
    this->_transform_hole_to_polyholes();

    // BBS: the actual first layer slices stored in layers are re-sorted by volume group and will be used to generate brim
    groupingVolumesForBrim(this, m_layers, firstLayerReplacedBy);

    // Update bounding boxes, back up raw slices of complex models.
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, m_layers.size()),
        [this](const tbb::blocked_range<size_t>& range) {
            for (size_t layer_idx = range.begin(); layer_idx < range.end(); ++ layer_idx) {
                m_print->throw_if_canceled();
                Layer &layer = *m_layers[layer_idx];
                layer.lslices_bboxes.clear();
                layer.lslices_bboxes.reserve(layer.lslices.size());
                for (const ExPolygon &expoly : layer.lslices)
                	layer.lslices_bboxes.emplace_back(get_extents(expoly));
                layer.backup_untyped_slices();
            }
        });
    if (m_layers.empty())
        throw Slic3r::SlicingError(L("No layers were detected. You might want to repair your STL file(s) or check their size or thickness and retry.\n"));

    // BBS
    this->set_done(posSlice);
}

template<typename ThrowOnCancel>
static inline void apply_mm_segmentation(PrintObject &print_object, ThrowOnCancel throw_on_cancel)
{
    // Returns MM segmentation based on painting in MM segmentation gizmo
    std::vector<std::vector<ExPolygons>> segmentation = multi_material_segmentation_by_painting(print_object, throw_on_cancel);
    assert(segmentation.size() == print_object.layer_count());
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, segmentation.size(), std::max(segmentation.size() / 128, size_t(1))),
        [&print_object, &segmentation, throw_on_cancel](const tbb::blocked_range<size_t> &range) {
            const auto  &layer_ranges   = print_object.shared_regions()->layer_ranges;
            double       z              = print_object.get_layer(int(range.begin()))->slice_z;
            auto         it_layer_range = layer_range_first(layer_ranges, z);
            // BBS
            const size_t num_extruders = print_object.print()->config().filament_diameter.size();

            struct ByExtruder {
                ExPolygons  expolygons;
                BoundingBox bbox;
            };

            struct ByRegion {
                ExPolygons expolygons;
                bool       needs_merge { false };
            };

            std::vector<ByExtruder> by_extruder;
            std::vector<ByRegion>   by_region;
            for (size_t layer_id = range.begin(); layer_id < range.end(); ++layer_id) {
                throw_on_cancel();
                Layer &layer = *print_object.get_layer(int(layer_id));
                it_layer_range = layer_range_next(layer_ranges, it_layer_range, layer.slice_z);
                const PrintObjectRegions::LayerRangeRegions &layer_range = *it_layer_range;
                // Gather per extruder expolygons.
                by_extruder.assign(num_extruders, ByExtruder());
                by_region.assign(layer.region_count(), ByRegion());
                bool layer_split = false;
                for (size_t extruder_id = 0; extruder_id < num_extruders; ++ extruder_id) {
                    ByExtruder &region = by_extruder[extruder_id];
                    append(region.expolygons, std::move(segmentation[layer_id][extruder_id]));
                    if (! region.expolygons.empty()) {
                        region.bbox = get_extents(region.expolygons);
                        layer_split = true;
                    }
                }

                if (!layer_split)
                    continue;

                // Split LayerRegions by by_extruder regions.
                // layer_range.painted_regions are sorted by extruder ID and parent PrintObject region ID.
                auto it_painted_region_begin = layer_range.painted_regions.cbegin();
                for (int parent_layer_region_idx = 0; parent_layer_region_idx < layer.region_count(); ++parent_layer_region_idx) {
                    if (it_painted_region_begin == layer_range.painted_regions.cend())
                        continue;

                    const LayerRegion &parent_layer_region = *layer.get_region(parent_layer_region_idx);
                    const PrintRegion &parent_print_region = parent_layer_region.region();
                    assert(parent_print_region.print_object_region_id() == parent_layer_region_idx);
                    if (parent_layer_region.slices.empty())
                        continue;

                    // Find the first PaintedRegion, which overrides the parent PrintRegion.
                    auto it_first_painted_region = std::find_if(it_painted_region_begin, layer_range.painted_regions.cend(), [&layer_range, &parent_print_region](const auto &painted_region) {
                        return layer_range.volume_regions[painted_region.parent].region->print_object_region_id() == parent_print_region.print_object_region_id();
                    });

                    if (it_first_painted_region == layer_range.painted_regions.cend())
                        continue; // This LayerRegion isn't overrides by any PaintedRegion.

                    assert(&parent_print_region == layer_range.volume_regions[it_first_painted_region->parent].region);

                    // Update the beginning PaintedRegion iterator for the next iteration.
                    it_painted_region_begin = it_first_painted_region;

                    const BoundingBox parent_layer_region_bbox = get_extents(parent_layer_region.slices.surfaces);
                    bool              self_trimmed             = false;
                    int               self_extruder_id         = -1; // 1-based extruder ID
                    for (int extruder_id = 1; extruder_id <= int(by_extruder.size()); ++extruder_id) {
                        const ByExtruder &segmented = by_extruder[extruder_id - 1];
                        if (!segmented.bbox.defined || !parent_layer_region_bbox.overlap(segmented.bbox))
                            continue;

                        // Find the first target region iterator.
                        auto it_target_region = std::find_if(it_painted_region_begin, layer_range.painted_regions.cend(), [extruder_id](const auto &painted_region) {
                            return int(painted_region.extruder_id) >= extruder_id;
                        });

                        assert(it_target_region != layer_range.painted_regions.end());
                        assert(layer_range.volume_regions[it_target_region->parent].region == &parent_print_region && int(it_target_region->extruder_id) == extruder_id);

                        // Update the beginning PaintedRegion iterator for the next iteration.
                        it_painted_region_begin = it_target_region;

                        // FIXME: Don't trim by self, it is not reliable.
                        if (it_target_region->region == &parent_print_region) {
                            self_extruder_id = extruder_id;
                            continue;
                        }

                        // Steal from this region.
                        int        target_region_id = it_target_region->region->print_object_region_id();
                        ExPolygons stolen           = intersection_ex(parent_layer_region.slices.surfaces, segmented.expolygons);
                        if (!stolen.empty()) {
                            ByRegion &dst = by_region[target_region_id];
                            if (dst.expolygons.empty()) {
                                dst.expolygons = std::move(stolen);
                            } else {
                                append(dst.expolygons, std::move(stolen));
                                dst.needs_merge = true;
                            }
                        }
                    }

                    if (!self_trimmed) {
                        // Trim slices of this LayerRegion with all the MM regions.
                        Polygons mine = to_polygons(parent_layer_region.slices.surfaces);
                        for (auto &segmented : by_extruder) {
                            if (&segmented - by_extruder.data() + 1 != self_extruder_id && segmented.bbox.defined && parent_layer_region_bbox.overlap(segmented.bbox)) {
                                mine = diff(mine, segmented.expolygons);
                                if (mine.empty())
                                    break;
                            }
                        }

                        // Filter out unprintable polygons produced by subtraction multi-material painted regions from layerm.region().
                        // ExPolygon returned from multi-material segmentation does not precisely match ExPolygons in layerm.region()
                        // (because of preprocessing of the input regions in multi-material segmentation). Therefore, subtraction from
                        // layerm.region() could produce a huge number of small unprintable regions for the model's base extruder.
                        // This could, on some models, produce bulges with the model's base color (#7109).
                        if (!mine.empty()) {
                            mine = opening(union_ex(mine), scaled<float>(5. * EPSILON), scaled<float>(5. * EPSILON));
                        }

                        if (!mine.empty()) {
                            ByRegion &dst = by_region[parent_print_region.print_object_region_id()];
                            if (dst.expolygons.empty()) {
                                dst.expolygons = union_ex(mine);
                            } else {
                                append(dst.expolygons, union_ex(mine));
                                dst.needs_merge = true;
                            }
                        }
                    }
                }

                // Re-create Surfaces of LayerRegions.
                for (int region_id = 0; region_id < layer.region_count(); ++region_id) {
                    ByRegion &src = by_region[region_id];
                    if (src.needs_merge) {
                        // Multiple regions were merged into one.
                        src.expolygons = closing_ex(src.expolygons, scaled<float>(10. * EPSILON));
                    }

                    layer.get_region(region_id)->slices.set(std::move(src.expolygons), stInternal);
                }
            }
        });
}

template<typename ThrowOnCancel>
void apply_fuzzy_skin_segmentation(PrintObject &print_object, ThrowOnCancel throw_on_cancel)
{
    // Returns fuzzy skin segmentation based on painting in the fuzzy skin painting gizmo.
    std::vector<std::vector<ExPolygons>> segmentation = fuzzy_skin_segmentation_by_painting(print_object, throw_on_cancel);
    assert(segmentation.size() == print_object.layer_count());

    struct ByRegion
    {
        ExPolygons expolygons;
        bool       needs_merge { false };
    };

    tbb::parallel_for(tbb::blocked_range<size_t>(0, segmentation.size(), std::max(segmentation.size() / 128, size_t(1))), [&print_object, &segmentation, throw_on_cancel](const tbb::blocked_range<size_t> &range) {
        const auto &layer_ranges   = print_object.shared_regions()->layer_ranges;
        auto        it_layer_range = layer_range_first(layer_ranges, print_object.get_layer(int(range.begin()))->slice_z);

        for (size_t layer_idx = range.begin(); layer_idx < range.end(); ++layer_idx) {
            throw_on_cancel();

            Layer &layer = *print_object.get_layer(int(layer_idx));
            it_layer_range = layer_range_next(layer_ranges, it_layer_range, layer.slice_z);
            const PrintObjectRegions::LayerRangeRegions &layer_range = *it_layer_range;

            assert(segmentation[layer_idx].size() == 1);
            const ExPolygons &fuzzy_skin_segmentation      = segmentation[layer_idx][0];
            const BoundingBox fuzzy_skin_segmentation_bbox = get_extents(fuzzy_skin_segmentation);
            if (fuzzy_skin_segmentation.empty())
                continue;

            // Split LayerRegions by painted fuzzy skin regions.
            // layer_range.fuzzy_skin_painted_regions are sorted by parent PrintObject region ID.
            std::vector<ByRegion> by_region(layer.region_count());
            auto                  it_fuzzy_skin_region_begin = layer_range.fuzzy_skin_painted_regions.cbegin();
            for (int parent_layer_region_idx = 0; parent_layer_region_idx < layer.region_count(); ++parent_layer_region_idx) {
                if (it_fuzzy_skin_region_begin == layer_range.fuzzy_skin_painted_regions.cend())
                    continue;

                const LayerRegion &parent_layer_region = *layer.get_region(parent_layer_region_idx);
                const PrintRegion &parent_print_region = parent_layer_region.region();
                assert(parent_print_region.print_object_region_id() == parent_layer_region_idx);
                if (parent_layer_region.slices.empty())
                    continue;

                // Find the first FuzzySkinPaintedRegion, which overrides the parent PrintRegion.
                auto it_fuzzy_skin_region = std::find_if(it_fuzzy_skin_region_begin, layer_range.fuzzy_skin_painted_regions.cend(), [&layer_range, &parent_print_region](const auto &fuzzy_skin_region) {
                    return fuzzy_skin_region.parent_print_object_region_id(layer_range) == parent_print_region.print_object_region_id();
                });

                if (it_fuzzy_skin_region == layer_range.fuzzy_skin_painted_regions.cend())
                    continue; // This LayerRegion isn't overrides by any FuzzySkinPaintedRegion.

                assert(it_fuzzy_skin_region->parent_print_object_region(layer_range) == &parent_print_region);

                // Update the beginning FuzzySkinPaintedRegion iterator for the next iteration.
                it_fuzzy_skin_region_begin = std::next(it_fuzzy_skin_region);

                const BoundingBox parent_layer_region_bbox        = get_extents(parent_layer_region.slices.surfaces);
                Polygons          layer_region_remaining_polygons = to_polygons(parent_layer_region.slices.surfaces);
                // Don't trim by self, it is not reliable.
                if (parent_layer_region_bbox.overlap(fuzzy_skin_segmentation_bbox) && it_fuzzy_skin_region->region != &parent_print_region) {
                    // Steal from this region.
                    const int  target_region_id = it_fuzzy_skin_region->region->print_object_region_id();
                    ExPolygons stolen           = intersection_ex(parent_layer_region.slices.surfaces, fuzzy_skin_segmentation);
                    if (!stolen.empty()) {
                        ByRegion &dst = by_region[target_region_id];
                        if (dst.expolygons.empty()) {
                            dst.expolygons = std::move(stolen);
                        } else {
                            append(dst.expolygons, std::move(stolen));
                            dst.needs_merge = true;
                        }
                    }

                    // Trim slices of this LayerRegion by the fuzzy skin region.
                    layer_region_remaining_polygons = diff(layer_region_remaining_polygons, fuzzy_skin_segmentation);

                    // Filter out unprintable polygons. Detailed explanation is inside apply_mm_segmentation.
                    if (!layer_region_remaining_polygons.empty()) {
                        layer_region_remaining_polygons = opening(union_ex(layer_region_remaining_polygons), scaled<float>(5. * EPSILON), scaled<float>(5. * EPSILON));
                    }
                }

                if (!layer_region_remaining_polygons.empty()) {
                    ByRegion &dst = by_region[parent_print_region.print_object_region_id()];
                    if (dst.expolygons.empty()) {
                        dst.expolygons = union_ex(layer_region_remaining_polygons);
                    } else {
                        append(dst.expolygons, union_ex(layer_region_remaining_polygons));
                        dst.needs_merge = true;
                    }
                }
            }

            // Re-create Surfaces of LayerRegions.
            for (int region_id = 0; region_id < layer.region_count(); ++region_id) {
                ByRegion &src = by_region[region_id];
                if (src.needs_merge) {
                    // Multiple regions were merged into one.
                    src.expolygons = closing_ex(src.expolygons, scaled<float>(10. * EPSILON));
                }

                layer.get_region(region_id)->slices.set(std::move(src.expolygons), stInternal);
            }
        }
    }); // end of parallel_for
}

// 1) Decides Z positions of the layers,
// 2) Initializes layers and their regions
// 3) Slices the object meshes
// 4) Slices the modifier meshes and reclassifies the slices of the object meshes by the slices of the modifier meshes
// 5) Applies size compensation (offsets the slices in XY plane)
// 6) Replaces bad slices by the slices reconstructed from the upper/lower layer
// Resulting expolygons of layer regions are marked as Internal.
//
// this should be idempotent
void PrintObject::slice_volumes()
{
    BOOST_LOG_TRIVIAL(info) << "Slicing volumes..." << log_memory_info();
    const Print *print                      = this->print();
    const auto   throw_on_cancel_callback   = std::function<void()>([print](){ print->throw_if_canceled(); });

    // Clear old LayerRegions, allocate for new PrintRegions.
    for (Layer* layer : m_layers) {
        //BBS: should delete all LayerRegionPtr to avoid memory leak
        while (!layer->m_regions.empty()) {
            if (layer->m_regions.back())
                delete layer->m_regions.back();
            layer->m_regions.pop_back();
        }
        layer->m_regions.reserve(m_shared_regions->all_regions.size());
        for (const std::unique_ptr<PrintRegion> &pr : m_shared_regions->all_regions)
            layer->m_regions.emplace_back(new LayerRegion(layer, pr.get()));
    }

    std::vector<float>                   slice_zs      = zs_from_layers(m_layers);
    std::vector<VolumeSlices> objSliceByVolume;
    if (!slice_zs.empty()) {
        objSliceByVolume = slice_volumes_inner(
            print->config(), this->config(), this->trafo_centered(),
            this->model_object()->volumes, m_shared_regions->layer_ranges, slice_zs, throw_on_cancel_callback);
    }

    //BBS: "model_part" volumes are grouded according to their connections
    //const auto           scaled_resolution = scaled<double>(print->config().resolution.value);
    //firstLayerObjSliceByVolume = findPartVolumes(objSliceByVolume, this->model_object()->volumes);
    //groupingVolumes(objSliceByVolumeParts, firstLayerObjSliceByGroups, scaled_resolution);
    //applyNegtiveVolumes(this->model_object()->volumes, objSliceByVolume, firstLayerObjSliceByGroups, scaled_resolution);
    firstLayerObjSliceByVolume = objSliceByVolume;

    // Overlap record. The native Body Split pass below measures it on the ownership lattice; every
    // other mode leaves it clear.
    m_regional_volume_overlap_layer = size_t(-1);
    m_regional_volume_overlap_mm2   = 0.;
    // Only a Body Split object is sliced on native grids. A plain object on a Body Split plate takes
    // the ordinary path below, as with the mode off.
    const bool body_split_object = is_mixed_nozzle_body_split(print->config()) &&
                                   is_body_split_object(print->config(), *this->model_object());
    // Measured only for an object staged because of a modifier or a negative part: its staged rows
    // are clipped part against part and can no longer show an overlap. For every other Body object
    // the ownership lattice measures it.
    const bool body_split_region_volumes = body_split_object &&
        std::any_of(this->model_object()->volumes.begin(), this->model_object()->volumes.end(),
                    [](const ModelVolume *volume) { return volume->is_modifier() || volume->is_negative_volume(); });
    if (body_split_region_volumes) {
        std::vector<const VolumeSlices*> part_slices;
        for (const ModelVolume *volume : this->model_object()->volumes)
            if (volume->is_model_part())
                for (const VolumeSlices &volume_slices : objSliceByVolume)
                    if (volume_slices.volume_id == volume->id())
                        part_slices.emplace_back(&volume_slices);
        if (part_slices.size() == 2) {
            const size_t shared_layers = std::min(part_slices[0]->slices.size(), part_slices[1]->slices.size());
            for (size_t layer_idx = 0; layer_idx < shared_layers; ++ layer_idx) {
                // Opened by 10 um first. Two bodies exported from one CAD split meet on a shared face, and
                // single-precision slicing turns that into hairline slivers that are not a double deposition.
                const ExPolygons shared = opening_ex(intersection_ex(part_slices[0]->slices[layer_idx],
                                                                     part_slices[1]->slices[layer_idx]),
                                                     scaled<float>(0.01));
                if (! shared.empty()) {
                    m_regional_volume_overlap_layer = layer_idx;
                    m_regional_volume_overlap_mm2   = unscale<double>(unscale<double>(area(shared)));
                    break;
                }
            }
        }
    }

    const bool native_grids = body_split_object;
    bool       conical_overhang_applied = false;
    if (native_grids) {
        const size_t region_count = num_printing_regions();
        const coordf_t base_h = config().layer_height.value;
        const bool has_painted_region = std::any_of(
            m_shared_regions->layer_ranges.begin(), m_shared_regions->layer_ranges.end(),
            [](const PrintObjectRegions::LayerRangeRegions &layer_range) {
                return !layer_range.painted_regions.empty();
            });
        // A modifier or a negative part changes which region owns what, so the ownership rows come from
        // the same region slicing the ordinary path uses rather than from the raw part slices.
        const bool has_region_volumes = std::any_of(
            this->model_object()->volumes.begin(), this->model_object()->volumes.end(),
            [](const ModelVolume *volume) { return volume->is_modifier() || volume->is_negative_volume(); });
        const bool staged_rows = has_painted_region || has_region_volumes;
        std::vector<std::vector<coordf_t>> nominal_planes(region_count);
        m_region_slicing.resize(region_count);
        m_region_grid_physical_extruders.assign(region_count, size_t(-1));

        // Native paint must be present before the ownership lattice is resolved. Stage the regular
        // per-region rows from the sliced model volumes, apply the same conical overhang normalization as
        // the ordinary path, then derive painted children from that view. Modifiers and negative parts
        // are resolved the same way. The raw rows move into slices_to_regions() here, so the native
        // planner uses these resolved parent rows.
        if (staged_rows) {
            std::vector<std::vector<ExPolygons>> staged_region_slices = slices_to_regions(
                print->config(), *this, this->model_object()->volumes, *m_shared_regions, slice_zs,
                std::move(objSliceByVolume), PrintObject::clip_multipart_objects, throw_on_cancel_callback);
            assert(staged_region_slices.size() == region_count);
            for (size_t region_id = 0; region_id < staged_region_slices.size(); ++region_id)
                for (size_t layer_id = 0; layer_id < staged_region_slices[region_id].size(); ++layer_id)
                    m_layers[layer_id]->get_region(int(region_id))->slices.set(
                        staged_region_slices[region_id][layer_id], stInternal);
            this->apply_conical_overhang(true);
            conical_overhang_applied = true;
        }

        // A region owns real per-body geometry only when it is the default (own-filament) region of one of
        // the object's model-part volumes, that is, it appears in some layer_range's volume_regions.
        // PrintApply.cpp creates a painted region for every parent volume and paint colour, including
        // ghosts with none of that colour's facets. Those would reach the geometry loop below with empty
        // per-region arrays, and indexing them there crashes.
        std::vector<bool>   is_body_region(region_count, false);
        // Which non-body regions are colour children and which are modifier regions. A modifier region is
        // a child of its body and, on a painted object, also a parent that colours are split off.
        std::vector<bool>   is_painted_region(region_count, false);
        std::vector<bool>   is_modifier_region(region_count, false);
        std::vector<size_t> painted_parent_region(region_count, size_t(-1));
        std::vector<const ModelVolume *> painted_parent_volume(region_count, nullptr);
        for (const PrintObjectRegions::LayerRangeRegions &layer_range : m_shared_regions->layer_ranges) {
            for (const PrintObjectRegions::VolumeRegion &volume_region : layer_range.volume_regions)
                if (volume_region.region != nullptr && volume_region.model_volume != nullptr &&
                    volume_region.model_volume->is_model_part())
                    is_body_region[size_t(volume_region.region->print_object_region_id())] = true;
            for (const PrintObjectRegions::PaintedRegion &painted_region : layer_range.painted_regions)
                if (painted_region.region != nullptr && painted_region.parent >= 0 &&
                    size_t(painted_region.parent) < layer_range.volume_regions.size()) {
                    const PrintObjectRegions::VolumeRegion &parent = layer_range.volume_regions[size_t(painted_region.parent)];
                    if (parent.region != nullptr) {
                        const size_t painted_region_id = size_t(painted_region.region->print_object_region_id());
                        // A colour equal to the parent's own filament resolves to the parent's own
                        // region; it is not a child of itself.
                        if (painted_region_id == size_t(parent.region->print_object_region_id()))
                            continue;
                        painted_parent_region[painted_region_id] =
                            size_t(parent.region->print_object_region_id());
                        painted_parent_volume[painted_region_id] = parent.model_volume;
                        is_painted_region[painted_region_id] = true;
                    }
                }
        }
        // A modifier's region is a child of the body it sits in, planned like a painted child: the body's
        // cadence while its filament stays on the body's tool, the other tool's cadence when it moves
        // across. Follow the parent chain through nested modifiers to the model part. A modifier region
        // shared by two bodies would need two cadences at once, so it is refused.
        for (const PrintObjectRegions::LayerRangeRegions &layer_range : m_shared_regions->layer_ranges)
            for (const PrintObjectRegions::VolumeRegion &volume_region : layer_range.volume_regions) {
                if (volume_region.region == nullptr || volume_region.model_volume == nullptr ||
                    ! volume_region.model_volume->is_modifier())
                    continue;
                const size_t modifier_region_id = size_t(volume_region.region->print_object_region_id());
                if (is_body_region[modifier_region_id])
                    continue;   // the modifier changed nothing, so its area stays with the body
                int parent_index = volume_region.parent;
                while (parent_index >= 0 && size_t(parent_index) < layer_range.volume_regions.size() &&
                       layer_range.volume_regions[size_t(parent_index)].model_volume != nullptr &&
                       ! layer_range.volume_regions[size_t(parent_index)].model_volume->is_model_part())
                    parent_index = layer_range.volume_regions[size_t(parent_index)].parent;
                if (parent_index < 0 || size_t(parent_index) >= layer_range.volume_regions.size() ||
                    layer_range.volume_regions[size_t(parent_index)].region == nullptr)
                    continue;
                const PrintObjectRegions::VolumeRegion &body = layer_range.volume_regions[size_t(parent_index)];
                const size_t body_region_id = size_t(body.region->print_object_region_id());
                if (painted_parent_region[modifier_region_id] != size_t(-1) &&
                    painted_parent_region[modifier_region_id] != body_region_id)
                    throw Slic3r::SlicingError(
                        "A modifier in a Body Split object covers two bodies with the same settings, so its area cannot follow "
                        "both bodies' layer heights. Keep each modifier inside one body, or give the two parts different settings.",
                        this->model_object()->id().id);
                painted_parent_region[modifier_region_id] = body_region_id;
                painted_parent_volume[modifier_region_id] = body.model_volume;
                is_modifier_region[modifier_region_id] = true;
            }

        // Resolve native facet painting against the staged parent rows. The helper keeps empty painted
        // ghosts, and only non-empty rows enter precedence below. This is the pre-grid counterpart of
        // apply_mm_segmentation(), which is skipped after native planning so cells and LayerRegion slices
        // cannot diverge.
        NativePaintSliceRows lattice_segmentation;
        // mmu_segmented_region_interlocking_depth cuts the painted band to the depth on even layers and to
        // the maximum width on odd ones. On a Body grid the fine painted rows and the coarse cell around
        // them have to flip together, or the shallow rows of each coarse cell leave a void and every row
        // becomes a contact plane. So the phase follows the object's coarsest cadence: 0 on the shared
        // first layer, then one phase per coarse layer. With no coarser body it is the lattice row index,
        // stock Orca's own phase. Both segmentation views below use this phase.
        const bool paint_interlocking_phased = has_painted_region &&
            config().mmu_segmented_region_interlocking_depth.value != 0. && !config().interlocking_beam.value;
        coordf_t paint_phase_cadence = base_h;
        if (paint_interlocking_phased)
            for (const BodySplitRegionAssignment &assignment : collect_body_split_body_assignments(*model_object(), base_h))
                paint_phase_cadence = std::max(paint_phase_cadence, coordf_t(assignment.cadence));
        const coordf_t paint_phase_first_top = m_layers.empty() ? 0. : m_layers.front()->print_z;
        const auto paint_interlocking_phases = [paint_phase_cadence, paint_phase_first_top](const LayerPtrs &layers) {
            std::vector<size_t> phases(layers.size(), 0);
            for (size_t layer_id = 0; layer_id < layers.size(); ++layer_id) {
                const coordf_t z = layers[layer_id]->slice_z;
                if (z > paint_phase_first_top + EPSILON)
                    phases[layer_id] = 1 + size_t(std::floor((z - paint_phase_first_top) / paint_phase_cadence + EPSILON));
            }
            return phases;
        };
        size_t parent_region_count = 0;
        for (size_t region_id = 0; region_id < region_count; ++region_id)
            if (is_body_region[region_id] || is_modifier_region[region_id])
                parent_region_count = std::max(parent_region_count, region_id + 1);
        if (has_painted_region && parent_region_count > 0) {
            NativePaintSliceRows staged_parent(m_layers.size(),
                                                std::vector<ExPolygons>(parent_region_count));
            for (size_t layer_id = 0; layer_id < m_layers.size(); ++layer_id)
                for (size_t parent_region_id = 0; parent_region_id < parent_region_count; ++parent_region_id)
                    staged_parent[layer_id][parent_region_id] = to_expolygons(
                        m_layers[layer_id]->get_region(int(parent_region_id))->slices.surfaces);

            const std::vector<size_t> lattice_paint_phases = paint_interlocking_phased ?
                paint_interlocking_phases(m_layers) : std::vector<size_t>();
            const NativePaintSliceRows segmentation = paint_interlocking_phased ?
                multi_material_segmentation_by_painting(*this, m_layers, [print]() { print->throw_if_canceled(); },
                                                        &lattice_paint_phases) :
                multi_material_segmentation_by_painting(*this, [print]() { print->throw_if_canceled(); });
            lattice_segmentation = segmentation;
            NativePaintRegionMappings mappings(m_layers.size());
            for (size_t layer_id = 0; layer_id < m_layers.size(); ++layer_id) {
                const double z = m_layers[layer_id]->slice_z;
                for (size_t parent_region_id = 0; parent_region_id < parent_region_count; ++parent_region_id)
                    mappings[layer_id].push_back({parent_region_id, 0, parent_region_id, true});

                std::set<std::tuple<size_t, size_t, size_t>> seen;
                for (const PrintObjectRegions::LayerRangeRegions &layer_range : m_shared_regions->layer_ranges) {
                    if (z < layer_range.layer_height_range.first - EPSILON ||
                        z > layer_range.layer_height_range.second + EPSILON)
                        continue;
                    for (const PrintObjectRegions::PaintedRegion &painted_region : layer_range.painted_regions) {
                        if (painted_region.parent < 0 || painted_region.region == nullptr || painted_region.extruder_id == 0 ||
                            size_t(painted_region.region->print_object_region_id()) >= region_count)
                            continue;
                        const size_t parent_index = size_t(painted_region.parent);
                        if (parent_index >= layer_range.volume_regions.size())
                            continue;
                        const PrintObjectRegions::VolumeRegion &parent = layer_range.volume_regions[parent_index];
                        if (parent.region == nullptr)
                            continue;
                        const size_t parent_region_id = size_t(parent.region->print_object_region_id());
                        const size_t target_region_id = size_t(painted_region.region->print_object_region_id());
                        if (parent_region_id >= parent_region_count ||
                            !(is_body_region[parent_region_id] || is_modifier_region[parent_region_id]))
                            continue;
                        const NativePaintRegionMapping mapping {
                            parent_region_id, size_t(painted_region.extruder_id - 1), target_region_id, false
                        };
                        if (seen.emplace(mapping.parent_region_id, mapping.logical_filament,
                                         mapping.target_region_id).second)
                            mappings[layer_id].push_back(mapping);
                    }
                }
            }

            const std::optional<NativePaintSliceRows> partition = partition_native_painted_slices(
                staged_parent, segmentation, region_count, mappings);
            if (!partition)
                throw Slic3r::SlicingError(
                    "Body Split could not work out which nozzle prints each painted area.",
                    this->model_object()->id().id);
            for (size_t layer_id = 0; layer_id < m_layers.size(); ++layer_id)
                for (size_t region_id = 0; region_id < region_count; ++region_id)
                    m_layers[layer_id]->get_region(int(region_id))->slices.set(
                        (*partition)[layer_id][region_id], stInternal);
        }

        // Real (non-ghost) per-body facts, taken from each body's own ModelVolume rather than the
        // ghost-inflated region loop, as PrintRegion::flow() and Print::validate() do. A painted colour is
        // evidence for admission but not a body cadence owner: on a single coarse body its fine painted
        // child resolves to the base cadence even though get_extruders() reports that colour on the coarse
        // volume. Cadence follows the volume's explicit body binding, and a cross-tool paint is qualified
        // through the same physical resolver admission uses.
        const std::vector<BodySplitRegionAssignment> body_owner_assignments =
            collect_body_split_body_assignments(*model_object(), base_h);

        const auto body_owner_cadence = [base_h](const ModelVolume &volume) {
            const double configured_height = volume.config.has("regional_layer_height") ?
                volume.config.opt_float("regional_layer_height") : 0.;
            return coordf_t(configured_height != 0. ? configured_height : base_h);
        };

        std::vector<coordf_t> planned_cadence(region_count, 0.);
        const auto resolve_painted_cadence = [&](size_t painted_region_id, size_t target_logical)
            -> std::optional<coordf_t> {
            std::optional<coordf_t> result;
            if (painted_region_id >= painted_parent_volume.size() ||
                painted_parent_volume[painted_region_id] == nullptr)
                return result;

            const ModelVolume &parent_volume = *painted_parent_volume[painted_region_id];
            // A colour painted over a modifier's area is a child of the modifier region, whose filament and
            // cadence are already planned (modifier regions have lower ids than any colour child, so they are
            // planned first below).
            const size_t parent_region_id = painted_parent_region[painted_region_id];
            const bool parent_is_modifier = parent_region_id < region_count && is_modifier_region[parent_region_id];
            const int own_filament = parent_is_modifier ?
                printing_region(parent_region_id).config().outer_wall_filament_id.value : parent_volume.extruder_id();
            if (own_filament <= 0)
                return result;
            const size_t own_logical = size_t(own_filament - 1);
            const coordf_t own_cadence = parent_is_modifier ? planned_cadence[parent_region_id] :
                body_owner_cadence(parent_volume);
            if (own_cadence <= 0.)
                return result;
            const MixedNozzleToolResolution own_tool = resolve_mixed_nozzle_tool(
                print->config(), own_logical, MixedNozzleResolveScope::PhysicalToolOnly);
            const MixedNozzleToolResolution target_tool = resolve_mixed_nozzle_tool(
                print->config(), target_logical, MixedNozzleResolveScope::PhysicalToolOnly);
            if (!own_tool || !target_tool)
                return result;

            // A colour that stays on the body's physical tool keeps the body's own cadence.
            if (own_tool.tool->physical_extruder == target_tool.tool->physical_extruder)
                return own_cadence;

            std::optional<coordf_t> target_cadence;
            for (const BodySplitRegionAssignment &assignment : body_owner_assignments)
                if (assignment.logical_filament == target_logical) {
                    if (target_cadence && !is_approx(*target_cadence, assignment.cadence))
                        return result;
                    target_cadence = coordf_t(assignment.cadence);
                }
            // A colour no body is bound to prints at the cadence of the bodies on its physical tool, as a
            // second colour on a nozzle does in Feature Split. Bodies sharing a tool share a cadence; if they
            // ever disagree, fail closed.
            if (!target_cadence)
                for (const BodySplitRegionAssignment &assignment : body_owner_assignments) {
                    const MixedNozzleToolResolution body_tool = resolve_mixed_nozzle_tool(
                        print->config(), assignment.logical_filament, MixedNozzleResolveScope::PhysicalToolOnly);
                    if (!body_tool || body_tool.tool->physical_extruder != target_tool.tool->physical_extruder)
                        continue;
                    if (target_cadence && !is_approx(*target_cadence, assignment.cadence))
                        return result;
                    target_cadence = coordf_t(assignment.cadence);
                }
            if (target_cadence) {
                if (is_approx(*target_cadence, own_cadence))
                    return result; // two physical tools at one cadence are not qualified.

                const bool target_is_fine = *target_cadence < own_cadence;
                const MixedNozzleCadenceResolution resolved = target_is_fine ?
                    resolve_mixed_nozzle_cadence(print->config(), *target_cadence, own_cadence,
                                                 target_logical, own_logical) :
                    resolve_mixed_nozzle_cadence(print->config(), own_cadence, *target_cadence,
                                                 own_logical, target_logical);
                if (!resolved)
                    return result;
                return *target_cadence;
            }

            // A single coarse body may have no separate fine body at all.  In that case the
            // target is still qualified when the body's own coarse cadence and physical nozzle
            // form a valid pair with the configured base cadence; otherwise fail closed instead
            // of inheriting the parent's regional_layer_height.
            if (own_cadence <= base_h || !is_approx(own_cadence / base_h,
                                                      std::round(own_cadence / base_h)))
                return result;
            const MixedNozzleCadenceResolution resolved = resolve_mixed_nozzle_cadence(
                print->config(), base_h, own_cadence, target_logical, own_logical);
            if (!resolved)
                return result;
            return coordf_t(resolved.cadence->fine_height);
        };
        // The live lattice slices are final here: nothing below writes them before the lattice ownership
        // is committed (the conical-overhang pass on the midpoint view swaps m_layers out and restores
        // it). So each region's "has any geometry" answer is taken once, not per layer.
        std::vector<char> region_sliced_geometry(region_count, 0);
        for (size_t region_id = 0; region_id < region_count; ++region_id)
            region_sliced_geometry[region_id] = std::any_of(m_layers.begin(), m_layers.end(),
                [region_id](const Layer *layer) {
                    const LayerRegion *region = layer->get_region(int(region_id));
                    return region != nullptr && !region->slices.empty();
                });
        const auto region_has_sliced_geometry = [&region_sliced_geometry](size_t region_id) {
            return region_id < region_sliced_geometry.size() && region_sliced_geometry[region_id] != 0;
        };
        for (size_t region_id = 0; region_id < region_count; ++region_id) {
            if (! is_body_region[region_id])
                continue;   // a painted sibling (real or ghost): handled by the pass below.
            const PrintRegionConfig &region = printing_region(region_id).config();
            const unsigned int logical_filament = unsigned(region.outer_wall_filament_id.value - 1);
            const auto assignment_it = std::find_if(body_owner_assignments.begin(), body_owner_assignments.end(),
                [logical_filament](const BodySplitRegionAssignment &assignment) {
                    return assignment.logical_filament == size_t(logical_filament);
                });
            assert(assignment_it != body_owner_assignments.end());
            const coordf_t cadence = assignment_it != body_owner_assignments.end() ? assignment_it->cadence :
                (region.regional_layer_height.value == 0. ? base_h : region.regional_layer_height.value);
            if (const std::optional<size_t> physical = physical_extruder_for_filament(print->config(), logical_filament))
                m_region_grid_physical_extruders[region_id] = *physical;
            planned_cadence[region_id] = cadence;
            RegionalSlicingPlan regional = make_regional_slicing_plan(
                print->config(), config(), model_object()->max_z(),
                logical_filament, print->shrinkage_compensation(), cadence);
            m_region_slicing[region_id] = regional.parameters;
            nominal_planes[region_id] = std::move(regional.nominal_planes);
        }
        // Painted siblings (including empty ghosts) still need a valid plan because the native
        // planner constructs a RegionalGridPlan for every region.  Resolve each child from its
        // target PrintRegion's logical filament; copying the parent would silently route a
        // cross-tool paint to the wrong cadence and physical owner.  Empty ghosts get metadata
        // but never enter precedence, so they remain cell-free.
        for (size_t region_id = 0; region_id < region_count; ++region_id) {
            if (is_body_region[region_id] || painted_parent_region[region_id] == size_t(-1))
                continue;
            const PrintRegionConfig &region = printing_region(region_id).config();
            if (region.outer_wall_filament_id.value <= 0)
                throw Slic3r::SlicingError(
                    "A painted area has no filament.",
                    this->model_object()->id().id);
            const size_t parent_region_id = painted_parent_region[region_id];
            if (!region_has_sliced_geometry(region_id)) {
                // PrintApply creates one painted PrintRegion for every (parent, colour) pair,
                // including ghosts whose colour has no facet on that parent.  Such a region has
                // no native ownership to project: mirror the parent's plan and tool metadata,
                // then leave it out of precedence below rather than resolving its colour as a
                // second body cadence.
                if (parent_region_id >= region_count ||
                    m_region_grid_physical_extruders[parent_region_id] == size_t(-1))
                    throw Slic3r::SlicingError(
                        "A painted area has no body to take its layer plan from.",
                        this->model_object()->id().id);
                m_region_grid_physical_extruders[region_id] =
                    m_region_grid_physical_extruders[parent_region_id];
                m_region_slicing[region_id] = m_region_slicing[parent_region_id];
                nominal_planes[region_id] = nominal_planes[parent_region_id];
                planned_cadence[region_id] = planned_cadence[parent_region_id];
                continue;
            }
            const size_t logical_filament = size_t(region.outer_wall_filament_id.value - 1);
            const std::optional<size_t> physical = physical_extruder_for_filament(
                print->config(), logical_filament);
            if (!physical)
                throw Slic3r::SlicingError(
                    "A painted area's filament is not assigned to a nozzle.",
                    this->model_object()->id().id);
            m_region_grid_physical_extruders[region_id] = *physical;

            const std::optional<coordf_t> painted_cadence = resolve_painted_cadence(region_id, logical_filament);
            if (!painted_cadence)
                throw Slic3r::SlicingError(
                    "A painted area has no layer height its nozzle can print.",
                    this->model_object()->id().id);
            const coordf_t cadence = *painted_cadence;
            planned_cadence[region_id] = cadence;
            RegionalSlicingPlan regional = make_regional_slicing_plan(
                print->config(), config(), model_object()->max_z(), logical_filament,
                print->shrinkage_compensation(), cadence);
            m_region_slicing[region_id] = regional.parameters;
            nominal_planes[region_id] = std::move(regional.nominal_planes);
        }

        // Fuzzy-skin painted siblings never own grid geometry under Body Split; their band stays on the
        // host cell as a perimeter mask (apply_body_split_fuzzy_skin_masks). Give each its parent's plan
        // and tool so it is a valid, cell-free region, like an empty painted ghost.
        for (const PrintObjectRegions::LayerRangeRegions &layer_range : m_shared_regions->layer_ranges)
            for (const PrintObjectRegions::FuzzySkinPaintedRegion &fuzzy : layer_range.fuzzy_skin_painted_regions) {
                if (fuzzy.region == nullptr)
                    continue;
                const size_t region_id = size_t(fuzzy.region->print_object_region_id());
                const size_t parent_region_id = size_t(fuzzy.parent_print_object_region_id(layer_range));
                if (region_id >= region_count || parent_region_id >= region_count || region_id == parent_region_id ||
                    is_body_region[region_id] || painted_parent_region[region_id] != size_t(-1) ||
                    m_region_grid_physical_extruders[region_id] != size_t(-1))
                    continue;
                if (m_region_grid_physical_extruders[parent_region_id] == size_t(-1))
                    throw Slic3r::SlicingError(
                        "A fuzzy skin painted area has no body to take its layer plan from.",
                        this->model_object()->id().id);
                m_region_grid_physical_extruders[region_id] = m_region_grid_physical_extruders[parent_region_id];
                m_region_slicing[region_id] = m_region_slicing[parent_region_id];
                nominal_planes[region_id] = nominal_planes[parent_region_id];
            }

        // The sliced object grid is exactly the ownership lattice, so reuse it; never slice these rows a
        // second time. With Z-contouring new_layers() sliced each row but the first at its bottom plus
        // the minimum Z, and every cell follows it.
        const std::optional<coordf_t> contour_slice_offset = z_contour_slice_offset(*this);
        std::vector<coordf_t> lattice_planes;
        lattice_planes.reserve(m_layers.size());
        coordf_t lattice_bottom = 0.;
        for (size_t row = 0; row < m_layers.size(); ++row) {
            const Layer *layer = m_layers[row];
            lattice_planes.push_back(layer->print_z);
            assert(std::abs(layer->slice_z - (contour_slice_offset && row > 0 ?
                lattice_bottom + *contour_slice_offset : 0.5 * (lattice_bottom + layer->print_z))) <= 1e-7);
            assert(std::abs(layer->height - (layer->print_z - lattice_bottom)) <= 1e-7);
            lattice_bottom = layer->print_z;
        }

        // A body is laid by one physical nozzle, and each of its cells has to be a height that nozzle can
        // lay. The lattice rows are the base cadence, so a nozzle whose minimum is above one row needs that
        // many rows per cell, and a nozzle whose minimum is above the shared first layer starts on its own
        // first plane: the first lattice plane at or above its minimum. On a pair that shares a height,
        // every region gets one row and the shared first plane.
        std::vector<size_t>   minimum_cell_rows(region_count, 1);
        std::vector<coordf_t> maximum_first_cell_height(region_count, 0.);
        bool envelope_grids = false;
        for (size_t region_id = 0; region_id < region_count && !lattice_planes.empty(); ++region_id) {
            const size_t tool = m_region_grid_physical_extruders[region_id];
            if (tool == size_t(-1) || tool >= print->config().nozzle_diameter.size())
                continue;
            const coordf_t minimum = resolved_min_layer_height(print->config(), tool);
            maximum_first_cell_height[region_id] = std::min<coordf_t>(
                resolved_max_layer_height(print->config(), tool), print->config().nozzle_diameter.get_at(tool));
            if (base_h > EPSILON && minimum > base_h + EPSILON)
                minimum_cell_rows[region_id] = size_t(std::ceil((minimum - EPSILON) / base_h));
            envelope_grids = envelope_grids || minimum_cell_rows[region_id] > 1;
            std::vector<coordf_t> &nominal = nominal_planes[region_id];
            if (nominal.empty() || minimum <= lattice_planes.front() + EPSILON)
                continue;
            const auto first = std::find_if(lattice_planes.begin(), lattice_planes.end(),
                [minimum](coordf_t z) { return z >= minimum - EPSILON; });
            // An object shorter than the nozzle's minimum keeps its nominal planes; the envelope
            // check after planning refuses it.
            if (first == lattice_planes.end())
                continue;
            const coordf_t cadence = nominal.size() > 1 ? nominal[1] - nominal[0] : 0.;
            std::vector<coordf_t> shifted{*first};
            // Each plane is snapped onto the lattice it is a whole number of rows above, so the
            // planner's plane lookup never sees accumulated rounding.
            while (cadence > EPSILON && shifted.back() < lattice_planes.back() - EPSILON) {
                const coordf_t target = shifted.back() + cadence;
                const auto snapped = std::lower_bound(lattice_planes.begin(), lattice_planes.end(), target - 1e-6);
                shifted.push_back(snapped != lattice_planes.end() && std::abs(*snapped - target) <= 1e-6 ? *snapped : target);
            }
            nominal = std::move(shifted);
            envelope_grids = true;
        }
        // A pair that shares a height plans on the plain lattice.
        if (!envelope_grids)
            maximum_first_cell_height.assign(region_count, 0.);

        // Every region gets a validly sized sample: a painted sibling (real or ghost) that claims no
        // lattice geometry stays empty ExPolygons at every row, since it is never written below.
        std::vector<std::vector<ExPolygons>> raw_lattice_samples(
            region_count, std::vector<ExPolygons>(lattice_planes.size()));
        std::vector<size_t> precedence;
        ModelVolumePtrs declared_volumes = model_object()->volumes;
        model_volumes_sort_by_id(declared_volumes);
        for (const ModelVolume *volume : declared_volumes) {
            if (!volume->is_model_part())
                continue;
            const auto raw_it = std::find_if(objSliceByVolume.begin(), objSliceByVolume.end(), [volume](const VolumeSlices &slices) {
                return slices.volume_id == volume->id();
            });
            assert(staged_rows || raw_it != objSliceByVolume.end());
            const PrintObjectRegions::VolumeRegion *mapped = nullptr;
            for (const auto &layer_range : m_shared_regions->layer_ranges) {
                const auto it = std::find_if(layer_range.volume_regions.begin(), layer_range.volume_regions.end(),
                    [volume](const PrintObjectRegions::VolumeRegion &candidate) {
                        return candidate.model_volume == volume && candidate.region != nullptr;
                    });
                if (it != layer_range.volume_regions.end()) { mapped = &*it; break; }
            }
            assert(mapped != nullptr);
            const size_t region_id = size_t(mapped->region->print_object_region_id());
            if (staged_rows) {
                for (size_t row = 0; row < lattice_planes.size(); ++row)
                    raw_lattice_samples[region_id][row] = to_expolygons(
                        m_layers[row]->get_region(int(region_id))->slices.surfaces);
            } else {
                raw_lattice_samples[region_id] = raw_it->slices;
            }
            if (std::find(precedence.begin(), precedence.end(), region_id) == precedence.end())
                precedence.push_back(region_id);
        }
        assert(precedence.size() == size_t(std::count(is_body_region.begin(), is_body_region.end(), true)));

        // Non-empty painted children take part after their model-part parents. The partition has already
        // removed each child's polygons from its parent; this resolver still does the cross-body
        // precedence and overlap accounting.
        for (size_t region_id = 0; region_id < region_count; ++region_id) {
            if (is_body_region[region_id] || painted_parent_region[region_id] == size_t(-1))
                continue;
            bool nonempty = false;
            for (size_t row = 0; row < lattice_planes.size(); ++row) {
                raw_lattice_samples[region_id][row] = to_expolygons(
                    m_layers[row]->get_region(int(region_id))->slices.surfaces);
                nonempty = nonempty || !raw_lattice_samples[region_id][row].empty();
            }
            if (nonempty)
                precedence.push_back(region_id);
        }

        // make_overhang_printable on an unpainted object: normalize the raw rows here, where the
        // painted path normalizes its staged rows, so the cone becomes regional ownership and the
        // cells are planned around it. Normalizing event layers after placement instead can hand
        // one body's cone to a slab another body's taller cell already covers. The overlap record
        // is taken from the rows before the cone, which never adds an overlap but can hide one.
        size_t   pre_cone_overlap_layer = size_t(-1);
        double   pre_cone_overlap_mm2   = 0.;
        const bool conical_on_native_rows = !conical_overhang_applied &&
            std::any_of(m_shared_regions->all_regions.begin(), m_shared_regions->all_regions.end(),
                        [](const std::unique_ptr<PrintRegion> &region) { return region->config().make_overhang_printable.value; });
        if (conical_on_native_rows) {
            for (size_t row = 0; row < lattice_planes.size() && pre_cone_overlap_layer == size_t(-1); ++row) {
                std::vector<ExPolygons> raw_row(region_count);
                for (size_t region = 0; region < region_count; ++region)
                    raw_row[region] = raw_lattice_samples[region][row];
                const RegionalOwnershipResolution resolved = resolve_regional_ownership(raw_row, precedence);
                if (resolved.b01_overlap) {
                    pre_cone_overlap_layer = row;
                    pre_cone_overlap_mm2   = resolved.b01_overlap_mm2;
                }
            }
            std::vector<coordf_t> lattice_slice_zs;
            lattice_slice_zs.reserve(m_layers.size());
            for (const Layer *layer : m_layers)
                lattice_slice_zs.push_back(layer->slice_z);
            this->apply_conical_overhang_to_rows(lattice_slice_zs, raw_lattice_samples);
            conical_overhang_applied = true;
        }

        std::vector<std::vector<ExPolygons>> lattice_ownership(region_count,
            std::vector<ExPolygons>(lattice_planes.size()));
        // Staged rows are already clipped part against part, so they cannot show an overlap. For an
        // object staged only because of a modifier or negative part, keep the record measured on the raw
        // part slices above.
        const size_t raw_overlap_layer = m_regional_volume_overlap_layer;
        const double raw_overlap_mm2   = m_regional_volume_overlap_mm2;
        m_regional_volume_overlap_layer = size_t(-1);
        m_regional_volume_overlap_mm2 = 0.;
        for (size_t row = 0; row < lattice_planes.size(); ++row) {
            std::vector<ExPolygons> raw_row(region_count);
            for (size_t region = 0; region < region_count; ++region)
                raw_row[region] = raw_lattice_samples[region][row];
            RegionalOwnershipResolution resolved = resolve_regional_ownership(raw_row, precedence);
            if (resolved.b01_overlap && m_regional_volume_overlap_layer == size_t(-1)) {
                m_regional_volume_overlap_layer = row;
                m_regional_volume_overlap_mm2 = resolved.b01_overlap_mm2;
            }
            for (size_t region = 0; region < region_count; ++region)
                lattice_ownership[region][row] = std::move(resolved.resolved[region]);
        }
        if (m_regional_volume_overlap_layer == size_t(-1) && pre_cone_overlap_layer != size_t(-1)) {
            m_regional_volume_overlap_layer = pre_cone_overlap_layer;
            m_regional_volume_overlap_mm2   = pre_cone_overlap_mm2;
        }
        if (has_region_volumes && ! has_painted_region && m_regional_volume_overlap_layer == size_t(-1)) {
            m_regional_volume_overlap_layer = raw_overlap_layer;
            m_regional_volume_overlap_mm2   = raw_overlap_mm2;
        }

        NativeRegionalPlanningInput planning;
        planning.lattice_planes = lattice_planes;
        planning.nominal_planes = nominal_planes;
        planning.lattice_ownership = lattice_ownership;
        planning.precedence = precedence;
        planning.interface_tolerance = print->config().regional_interface_tolerance.value;
        planning.phase_rule = print->config().regional_grid_phase_rule.value;
        planning.interlocking = {config().interlocking_beam.value,
                                 size_t(config().interlocking_beam_layer_count.value)};
        planning.sample_native_ownership = [](size_t, coordf_t) { return ExPolygons{}; };
        planning.minimum_cell_rows = minimum_cell_rows;
        planning.maximum_first_cell_height = maximum_first_cell_height;
        // Each cell is sampled, sliced and later contoured on this one plane.
        planning.contour_slice_offset = contour_slice_offset;

        // The fine-skins opt-in is the explicit per-object/per-region key mixed_nozzle_body_fine_skins.
        // Divergent skin filament ids are not used as the signal: PrintRegion::flow() resolves
        // frSolidInfill through internal_solid_filament_id, so every solid surface of the region, coarse
        // cells included, would get the fine nozzle's width. The cell decides, not the role routing.
        planning.fine_skin_enabled.assign(region_count, false);
        planning.fine_skin_layers.assign(region_count, 0);
        planning.fine_skin_coarse_line_width.assign(region_count, 0.);
        for (size_t region = 0; region < region_count; ++region) {
            const PrintRegionConfig &rc = printing_region(region).config();
            // A painted region inherits its parent body's opt-in, but its colour is what the user painted,
            // so only bodies convert. In a two-region object the painted child is always at the base cadence
            // and never converts anyway.
            if (rc.mixed_nozzle_body_fine_skins.value && (region_count == 2 || is_body_region[region])) {
                planning.fine_skin_enabled[region] = true;
                // The window is mixed_nozzle_body_fine_skin_layers (fine rows converted at each exposed
                // face), not the region's single-nozzle shell thickness.
                planning.fine_skin_layers[region] = rc.mixed_nozzle_body_fine_skin_layers.value;
                // The seed filter erodes by the coarse physical tool's nozzle width, read back off the
                // region's flow rather than re-derived from the tool map. The planner treats 0 as "no
                // filter", which every region that did not opt in keeps.
                const double region_cadence = rc.regional_layer_height.value == 0. ?
                    config().layer_height.value : rc.regional_layer_height.value;
                planning.fine_skin_coarse_line_width[region] = double(
                    printing_region(region).flow(*this, frSolidInfill, region_cadence, false).nozzle_diameter());
            }
        }

        std::vector<RegionalGridPlan> native_sample_plans;
        const std::vector<coordf_t> native_midplanes = native_regional_cell_midplanes(planning, &native_sample_plans);
        std::vector<float> native_zs;
        native_zs.reserve(native_midplanes.size());
        for (coordf_t z : native_midplanes)
            native_zs.push_back(float(z));
        std::vector<VolumeSlices> native_raw = slice_volumes_inner(
            print->config(), config(), trafo_centered(), model_object()->volumes,
            m_shared_regions->layer_ranges, native_zs, throw_on_cancel_callback);
        std::vector<std::vector<ExPolygons>> native_ownership = slices_to_regions(
            print->config(), *this, model_object()->volumes, *m_shared_regions, native_zs,
            std::move(native_raw), PrintObject::clip_multipart_objects, throw_on_cancel_callback);
        // The cells take their footprints from these midplane samples, so they carry the same cone.
        if (conical_on_native_rows)
            this->apply_conical_overhang_to_rows(native_midplanes, native_ownership);

        if (has_painted_region && parent_region_count > 0) {
        // The projector counts shell depth over one Layer view, so each painted cadence gets its own view
        // and every painted colour is projected on the view of the cadence its target region prints at.
        // There is no limit on painted colours per object. One painted cadence, including a fine painted
        // patch on one coarse body, builds exactly one view.
        std::vector<std::optional<coordf_t>> painted_view_cadences;
        for (size_t region_id = 0; region_id < region_count; ++region_id) {
            if (is_body_region[region_id] || !is_painted_region[region_id] ||
                !region_has_sliced_geometry(region_id))
                continue;
            const coordf_t cadence = m_region_slicing[region_id].layer_height;
            if (std::none_of(painted_view_cadences.begin(), painted_view_cadences.end(),
                    [cadence](const std::optional<coordf_t> &view) { return is_approx(*view, cadence); }))
                painted_view_cadences.emplace_back(cadence);
        }
        if (painted_view_cadences.empty())
            painted_view_cadences.emplace_back(std::nullopt);

        // Which view each segmentation slot (one painted colour) is read from. A slot that only
        // paints its own parent, or only reaches empty ghosts, moves no geometry and keeps view 0.
        std::vector<size_t> painted_slot_view;
        if (painted_view_cadences.size() > 1)
            for (const PrintObjectRegions::LayerRangeRegions &layer_range : m_shared_regions->layer_ranges)
                for (const PrintObjectRegions::PaintedRegion &painted_region : layer_range.painted_regions) {
                    if (painted_region.parent < 0 || painted_region.region == nullptr || painted_region.extruder_id == 0 ||
                        size_t(painted_region.parent) >= layer_range.volume_regions.size())
                        continue;
                    const PrintObjectRegions::VolumeRegion &parent = layer_range.volume_regions[size_t(painted_region.parent)];
                    if (parent.region == nullptr)
                        continue;
                    const size_t parent_region_id = size_t(parent.region->print_object_region_id());
                    const size_t target_region_id = size_t(painted_region.region->print_object_region_id());
                    // A colour painted over a modifier region has that region as its parent, as the lattice
                    // and midpoint partitions already allow.
                    if (parent_region_id >= parent_region_count || target_region_id >= region_count ||
                        !(is_body_region[parent_region_id] || is_modifier_region[parent_region_id]) ||
                        is_body_region[target_region_id] ||
                        target_region_id == parent_region_id || !region_has_sliced_geometry(target_region_id))
                        continue;
                    const coordf_t cadence = m_region_slicing[target_region_id].layer_height;
                    const auto view_it = std::find_if(painted_view_cadences.begin(), painted_view_cadences.end(),
                        [cadence](const std::optional<coordf_t> &candidate) { return is_approx(*candidate, cadence); });
                    if (view_it == painted_view_cadences.end())
                        continue;
                    const size_t view = size_t(view_it - painted_view_cadences.begin());
                    const size_t slot = size_t(painted_region.extruder_id - 1);
                    if (painted_slot_view.size() <= slot)
                        painted_slot_view.resize(slot + 1, size_t(-1));
                    if (painted_slot_view[slot] == size_t(-1))
                        painted_slot_view[slot] = view;
                    else if (painted_slot_view[slot] != view)
                        throw Slic3r::SlicingError(
                            "One painted colour resolves to two different layer heights in this object, so its paint cannot be placed.",
                            this->model_object()->id().id);
                }

        // Reuse the native facet projector on the exact cell-midpoint view.  A nearest base-row
        // lookup is unsafe for sloped paint boundaries and thin roof patches, so construct a
        // read-only layer view over the fresh midpoint slices and partition that result with the
        // same parent/target mapping used for the lattice rows.
            struct OwnedNativePaintLayers {
                LayerPtrs layers;
                ~OwnedNativePaintLayers()
                {
                    for (Layer *layer : layers)
                        delete layer;
                }
                OwnedNativePaintLayers() = default;
                OwnedNativePaintLayers(const OwnedNativePaintLayers &) = delete;
                OwnedNativePaintLayers &operator=(const OwnedNativePaintLayers &) = delete;
            };
            const auto build_native_paint_view = [&](const std::optional<coordf_t> &painted_view_cadence,
                                                     OwnedNativePaintLayers &native_paint_layers) {
        std::vector<coordf_t> native_view_heights(native_midplanes.size(), base_h);
        std::vector<coordf_t> native_view_print_z(native_midplanes.size());
        std::vector<size_t> preferred_painted_regions;
        for (size_t region_id = 0; region_id < region_count; ++region_id)
            if (!is_body_region[region_id] && is_painted_region[region_id] &&
                region_has_sliced_geometry(region_id) &&
                (!painted_view_cadence || is_approx(m_region_slicing[region_id].layer_height,
                                                    *painted_view_cadence)))
                preferred_painted_regions.push_back(region_id);
        // Midpoint inventory includes empty cells too. Their interval comes from the native
        // tiling plan, not the geometry-pruned list of cells that will actually deposit material.
        for (size_t layer_id = 0; layer_id < native_midplanes.size(); ++layer_id) {
            const auto find_shape_cell = [z = native_midplanes[layer_id]](const RegionalGridPlan &plan)
                -> const RegionCell * {
                const auto cell_it = std::find_if(plan.cells.begin(), plan.cells.end(),
                    [z](const RegionCell &cell) { return std::abs(cell.slice_z - z) <= 1e-7; });
                return cell_it == plan.cells.end() ? nullptr : &*cell_it;
            };
            // Prefer the painted target's own cell; otherwise take the first region that has one.
            const RegionCell *shape_cell = nullptr;
            for (size_t region_id : preferred_painted_regions)
                if ((shape_cell = find_shape_cell(native_sample_plans[region_id])) != nullptr)
                    break;
            if (shape_cell == nullptr)
                for (const RegionalGridPlan &region_plan : native_sample_plans)
                    if ((shape_cell = find_shape_cell(region_plan)) != nullptr)
                        break;
            if (shape_cell == nullptr)
                throw Slic3r::SlicingError(
                    "A painted area has no matching body layer at Z=" +
                        std::to_string(native_midplanes[layer_id]) + ".",
                    this->model_object()->id().id);
            native_view_heights[layer_id] = shape_cell->height;
            native_view_print_z[layer_id] = shape_cell->top_z;
        }

            native_paint_layers.layers.reserve(native_zs.size());
            for (size_t layer_id = 0; layer_id < native_zs.size(); ++layer_id) {
                Layer *layer = new Layer(layer_id, this, native_view_heights[layer_id],
                                         native_view_print_z[layer_id], native_zs[layer_id]);
                layer->m_regions.reserve(m_shared_regions->all_regions.size());
                for (const std::unique_ptr<PrintRegion> &pr : m_shared_regions->all_regions)
                    layer->m_regions.emplace_back(new LayerRegion(layer, pr.get()));
                for (size_t region_id = 0; region_id < region_count; ++region_id)
                    layer->get_region(int(region_id))->slices.set(
                        native_ownership[region_id][layer_id], stInternal);
                if (!native_paint_layers.layers.empty()) {
                    native_paint_layers.layers.back()->upper_layer = layer;
                    layer->lower_layer = native_paint_layers.layers.back();
                }
                native_paint_layers.layers.push_back(layer);
            }
            };
            OwnedNativePaintLayers native_paint_layers;
            build_native_paint_view(painted_view_cadences.front(), native_paint_layers);

            // The base lattice was normalized for overhangs before its paint partition. Apply
            // the same mutation to the exact midpoint view while it is owned by this scratch
            // scope, then restore the live layer vector before continuing the native planner.
            LayerPtrs saved_layers = std::move(m_layers);
            try {
                m_layers = native_paint_layers.layers;
                this->apply_conical_overhang(true);
            } catch (...) {
                m_layers = std::move(saved_layers);
                throw;
            }
            m_layers = std::move(saved_layers);
            for (size_t layer_id = 0; layer_id < native_paint_layers.layers.size(); ++layer_id)
                for (size_t region_id = 0; region_id < region_count; ++region_id)
                    native_ownership[region_id][layer_id] = to_expolygons(
                        native_paint_layers.layers[layer_id]->get_region(int(region_id))->slices.surfaces);

            NativePaintSliceRows native_segmentation;
            const std::vector<size_t> native_paint_phases = paint_interlocking_phased ?
                paint_interlocking_phases(native_paint_layers.layers) : std::vector<size_t>();
            native_segmentation = multi_material_segmentation_by_painting(
                *this, native_paint_layers.layers, [print]() { print->throw_if_canceled(); },
                paint_interlocking_phased ? &native_paint_phases : nullptr);

            // Every further painted cadence is projected on its own view, built from the
            // overhang-normalized slices above (the overhang pass reads the object layer height,
            // never the view heights, so it is not applied twice), and hands over only the slots
            // that print at that cadence.
            for (size_t view = 1; view < painted_view_cadences.size(); ++view) {
                OwnedNativePaintLayers view_layers;
                build_native_paint_view(painted_view_cadences[view], view_layers);
                NativePaintSliceRows view_segmentation = multi_material_segmentation_by_painting(
                    *this, view_layers.layers, [print]() { print->throw_if_canceled(); });
                if (view_segmentation.size() != native_segmentation.size())
                    throw Slic3r::SlicingError(
                        "Painted areas disagree on their layer count.",
                        this->model_object()->id().id);
                for (size_t slot = 0; slot < painted_slot_view.size(); ++slot) {
                    if (painted_slot_view[slot] != view)
                        continue;
                    for (size_t row = 0; row < native_segmentation.size(); ++row)
                        if (slot < native_segmentation[row].size() && slot < view_segmentation[row].size())
                            native_segmentation[row][slot] = std::move(view_segmentation[row][slot]);
                }
            }

            // At shared sample planes the existing native base-cadence projection is
            // authoritative. Re-projecting these on a union of cadences changes native shell
            // layer counting even though the requested Z and source mesh are identical.
            for (size_t row = 0; row < native_paint_layers.layers.size(); ++row) {
                const double z = native_paint_layers.layers[row]->slice_z;
                const auto original = std::lower_bound(m_layers.begin(), m_layers.end(), z - 1e-7,
                    [](const Layer *layer, double height) { return layer->slice_z < height; });
                if (original != m_layers.end() && std::abs((*original)->slice_z - z) <= 1e-7)
                    native_segmentation[row] = lattice_segmentation[size_t(original - m_layers.begin())];
            }

            NativePaintRegionMappings native_mappings(native_paint_layers.layers.size());
            for (size_t layer_id = 0; layer_id < native_paint_layers.layers.size(); ++layer_id) {
                const double z = native_paint_layers.layers[layer_id]->slice_z;
                for (size_t parent_region_id = 0; parent_region_id < parent_region_count; ++parent_region_id)
                    native_mappings[layer_id].push_back({parent_region_id, 0, parent_region_id, true});
                std::set<std::tuple<size_t, size_t, size_t>> seen;
                for (const PrintObjectRegions::LayerRangeRegions &layer_range : m_shared_regions->layer_ranges) {
                    if (z < layer_range.layer_height_range.first - EPSILON ||
                        z > layer_range.layer_height_range.second + EPSILON)
                        continue;
                    for (const PrintObjectRegions::PaintedRegion &painted_region : layer_range.painted_regions) {
                        if (painted_region.parent < 0 || painted_region.region == nullptr || painted_region.extruder_id == 0)
                            continue;
                        const size_t parent_index = size_t(painted_region.parent);
                        if (parent_index >= layer_range.volume_regions.size())
                            continue;
                        const PrintObjectRegions::VolumeRegion &parent = layer_range.volume_regions[parent_index];
                        if (parent.region == nullptr)
                            continue;
                        const size_t parent_region_id = size_t(parent.region->print_object_region_id());
                        const size_t target_region_id = size_t(painted_region.region->print_object_region_id());
                        if (parent_region_id >= parent_region_count || target_region_id >= region_count ||
                            !(is_body_region[parent_region_id] || is_modifier_region[parent_region_id]))
                            continue;
                        const NativePaintRegionMapping mapping {
                            parent_region_id, size_t(painted_region.extruder_id - 1),
                            region_has_sliced_geometry(target_region_id) ? target_region_id : parent_region_id, false
                        };
                        if (seen.emplace(mapping.parent_region_id, mapping.logical_filament,
                                         mapping.target_region_id).second)
                            native_mappings[layer_id].push_back(mapping);
                    }
                }
            }

            NativePaintSliceRows native_staged(native_paint_layers.layers.size(),
                                                std::vector<ExPolygons>(parent_region_count));
            for (size_t layer_id = 0; layer_id < native_paint_layers.layers.size(); ++layer_id)
                for (size_t parent_region_id = 0; parent_region_id < parent_region_count; ++parent_region_id)
                    native_staged[layer_id][parent_region_id] =
                        native_ownership[parent_region_id][layer_id];
            const std::optional<NativePaintSliceRows> native_partition =
                partition_native_painted_slices(native_staged, native_segmentation,
                                                region_count, native_mappings);
            if (!native_partition)
                throw Slic3r::SlicingError(
                    "Body Split could not work out which nozzle prints a painted area on one of its layers.",
                    this->model_object()->id().id);
            native_ownership.assign(region_count, std::vector<ExPolygons>(native_paint_layers.layers.size()));
            for (size_t layer_id = 0; layer_id < native_partition->size(); ++layer_id)
                for (size_t region_id = 0; region_id < region_count; ++region_id)
                    native_ownership[region_id][layer_id] = (*native_partition)[layer_id][region_id];
        }
        planning.sample_native_ownership = [&native_midplanes, &native_ownership](size_t region, coordf_t z) {
            const auto it = std::lower_bound(native_midplanes.begin(), native_midplanes.end(), z - 1e-8);
            assert(it != native_midplanes.end() && std::abs(*it - z) <= 1e-7);
            return native_ownership[region][size_t(it - native_midplanes.begin())];
        };
        m_native_regional_grid_state = plan_native_regional_grids(planning);
        // Last use of the local samples: hand them over instead of copying all lattice geometry.
        m_native_regional_grid_state.raw_lattice_samples = std::move(raw_lattice_samples);

        // Fail closed: every cell a region keeps is laid by its own nozzle, or by the fine nozzle for a
        // fine-skin row, and has to be a height that nozzle can lay. A grid the planner could not hold to
        // that is refused here rather than exported.
        for (size_t region_id = 0; envelope_grids && region_id < region_count; ++region_id) {
            const size_t tool = m_region_grid_physical_extruders[region_id];
            if (tool == size_t(-1) || tool >= print->config().nozzle_diameter.size())
                continue;
            const coordf_t minimum = resolved_min_layer_height(print->config(), tool);
            for (const NativeRegionCellState &native : m_native_regional_grid_state.cells[region_id]) {
                if (native.cell.fine_skin)
                    continue;
                if (native.cell.height < minimum - EPSILON ||
                    native.cell.height > maximum_first_cell_height[region_id] + EPSILON)
                    throw Slic3r::SlicingError(Slic3r::format(
                        "[SRL-A38] A %1% mm layer from z %2% mm to %3% mm falls outside nozzle %4%'s "
                        "layer-height limits (%5% mm to %6% mm), and no whole number of base layers fixes it here.",
                        native.cell.height, native.cell.bottom_z, native.cell.top_z, tool + 1, minimum,
                        maximum_first_cell_height[region_id]),
                        this->model_object()->id().id);
            }
        }

        m_pre_interlocking_slices.assign(lattice_planes.size(), std::vector<ExPolygons>(region_count));
        for (size_t region = 0; region < region_count; ++region)
            for (size_t row = 0; row < lattice_planes.size(); ++row) {
                m_pre_interlocking_slices[row][region] = lattice_ownership[region][row];
                m_layers[row]->get_region(int(region))->slices.set(lattice_ownership[region][row], stInternal);
            }

        // Interlocking remains on its native uniform lattice. The explicit one-row
        // origin phases beams off the shared first layer.
        if (config().interlocking_beam.value)
        {
            // The beams start on the first plane every body has met at. When all bodies share the first
            // layer that is lattice row 0, so the origin is 1.
            size_t first_met_row = 0;
            for (const RegionalGridPlan &plan : m_native_regional_grid_state.region_plans)
                if (!plan.cells.empty())
                    first_met_row = std::max(first_met_row, plan.cells.front().last_lattice_row);
            // Body beams: the planner phases its voxels from the first coarse plane where the
            // bodies touch. Start the beam lattice on the row above that plane so every deposited
            // beam block is one of the planned voxels' coarse cells.
            {
                const std::vector<coordf_t> &rows = m_native_regional_grid_state.lattice_planes;
                const coordf_t origin_z = m_native_regional_grid_state.interlocking_origin_z;
                const auto origin = std::lower_bound(rows.begin(), rows.end(), origin_z - 1e-7);
                if (origin != rows.end() && std::abs(*origin - origin_z) <= 1e-7)
                    first_met_row = std::max(first_met_row, size_t(origin - rows.begin()));
            }
            InterlockingGenerator::generate_interlocking_structure(this, [print]() { print->throw_if_canceled(); },
                                                                   coord_t(first_met_row + 1));
        }
        std::vector<std::vector<ExPolygons>> beam_ownership(region_count,
            std::vector<ExPolygons>(lattice_planes.size()));
        for (size_t row = 0; row < lattice_planes.size(); ++row)
            for (size_t region = 0; region < region_count; ++region)
                beam_ownership[region][row] = union_ex(to_expolygons(m_layers[row]->get_region(int(region))->slices.surfaces));
        if (!config().interlocking_beam.value)
            beam_ownership = lattice_ownership;

        const auto row_overlaps_decision = [this](size_t row, const RegionalInterlockingDecision &decision) {
            const coordf_t row_top = m_native_regional_grid_state.lattice_planes[row];
            const coordf_t row_bottom = row == 0 ? 0. : m_native_regional_grid_state.lattice_planes[row - 1];
            return row_top > decision.bottom_z + 1e-8 && row_bottom < decision.top_z - 1e-8;
        };
        const auto row_in_applied_voxel = [this, &row_overlaps_decision](size_t row) {
            return std::any_of(m_native_regional_grid_state.interlocking_decisions.begin(),
                               m_native_regional_grid_state.interlocking_decisions.end(),
                [row, &row_overlaps_decision](const RegionalInterlockingDecision &decision) {
                    return decision.apply && row_overlaps_decision(row, decision);
                });
        };
        const auto same_transfer = [](const ExPolygons &lhs, const ExPolygons &rhs) {
            return diff_ex(lhs, rhs, ApplySafetyOffset::Yes).empty() &&
                   diff_ex(rhs, lhs, ApplySafetyOffset::Yes).empty();
        };
        const auto row_has_generated_transfer = [&lattice_ownership, &beam_ownership, region_count](size_t row) {
            for (size_t region = 0; region < region_count; ++region)
                if (!diff_ex(beam_ownership[region][row], lattice_ownership[region][row],
                             ApplySafetyOffset::Yes).empty() ||
                    !diff_ex(lattice_ownership[region][row], beam_ownership[region][row],
                             ApplySafetyOffset::Yes).empty())
                    return true;
            return false;
        };

        std::vector<bool> rollback_rows(lattice_planes.size(), false);
        // Body beam trace (CADENCE_BEAM_TRACE=1): per planned unit, per lattice row and region,
        // the transfer area after each pass, and why a unit was dropped. Off costs one bool.
        const bool beam_trace = config().interlocking_beam.value && interlocking_beam_trace();
        auto &trace_decisions = m_native_regional_grid_state.interlocking_decisions;
        std::vector<std::string> drop_reason(trace_decisions.size());
        for (size_t index = 0; index < trace_decisions.size(); ++index)
            if (!trace_decisions[index].apply)
                drop_reason[index] = "planned as partial contact across a forced plane";
        const auto trace_units = [&](const char *pass) {
            if (!beam_trace)
                return;
            for (size_t index = 0; index < trace_decisions.size(); ++index) {
                const RegionalInterlockingDecision &decision = trace_decisions[index];
                std::ostringstream out;
                out << "[BEAM-TRACE] " << this->model_object()->name << " pass '" << pass << "' unit " << index << " ["
                    << decision.bottom_z << ", " << decision.top_z << "] apply " << decision.apply;
                for (size_t row = 0; row < lattice_planes.size(); ++row) {
                    if (!row_overlaps_decision(row, decision))
                        continue;
                    out << "\n    row " << row << " z " << lattice_planes[row] << (rollback_rows[row] ? " (rolled back)" : "");
                    for (size_t region = 0; region < region_count; ++region)
                        out << " | region " << region << " in "
                            << unscale<double>(unscale<double>(area(diff_ex(beam_ownership[region][row],
                                   lattice_ownership[region][row], ApplySafetyOffset::Yes))))
                            << " out "
                            << unscale<double>(unscale<double>(area(diff_ex(lattice_ownership[region][row],
                                   beam_ownership[region][row], ApplySafetyOffset::Yes))));
                }
                BOOST_LOG_TRIVIAL(warning) << out.str();
            }
        };
        if (beam_trace)
            BOOST_LOG_TRIVIAL(warning) << "[BEAM-TRACE] " << this->model_object()->name << " origin_z "
                                       << m_native_regional_grid_state.interlocking_origin_z << ", "
                                       << trace_decisions.size() << " planned units";
        trace_units("generator output");
        std::vector<RendezvousPlane> dropped;
        for (const RendezvousPlane &plane : m_native_regional_grid_state.rendezvous)
            if (plane.reason == RegionalRendezvousReason::InterlockingDropped)
                dropped.push_back(plane);
        const auto record_drop = [&dropped, &precedence](coordf_t z) {
            dropped.push_back({z, RegionalRendezvousReason::InterlockingDropped, precedence, 0.});
        };

        // Geometry outside an applied legal decision is never retained, including row zero.
        for (size_t row = 0; row < lattice_planes.size(); ++row)
            if (row == 0 || !row_in_applied_voxel(row)) {
                rollback_rows[row] = true;
                if (row_has_generated_transfer(row))
                    record_drop(lattice_planes[row]);
            }
        trace_units("rows outside applied units marked for rollback");

        // Body beams: one transfer per coarse cell. The generator works row by row, and its thin-area and
        // closing passes react to any difference between the rows of one coarse cell, even far from the
        // joint or a hundredth of a millimetre at an outer edge, which fails the uniformity check below
        // for the whole voxel. A coarse cell prints one footprint for all its rows, so for every multirow
        // cell whose rows all lie in applied voxels keep only the transfer every row agrees on, and hand
        // whatever a single row took or gave back to the other regions on that row. The uniformity pass
        // still drops anything this cannot make uniform, such as two multirow cells that meet.
        for (size_t row = 0; row < rollback_rows.size(); ++row)
            if (rollback_rows[row])
                for (size_t region = 0; region < region_count; ++region)
                    beam_ownership[region][row] = lattice_ownership[region][row];
        for (size_t region = 0; region < region_count; ++region)
            for (const NativeRegionCellState &cell : m_native_regional_grid_state.cells[region]) {
                const size_t first_row = cell.cell.first_lattice_row;
                const size_t last_row  = cell.cell.last_lattice_row;
                if (first_row == last_row || last_row >= rollback_rows.size())
                    continue;
                bool live = true;
                for (size_t row = first_row; row <= last_row && live; ++row)
                    live = !rollback_rows[row];
                if (!live)
                    continue;
                ExPolygons common_in = diff_ex(beam_ownership[region][first_row],
                    lattice_ownership[region][first_row], ApplySafetyOffset::Yes);
                ExPolygons common_out = diff_ex(lattice_ownership[region][first_row],
                    beam_ownership[region][first_row], ApplySafetyOffset::Yes);
                for (size_t row = first_row + 1; row <= last_row; ++row) {
                    common_in = intersection_ex(common_in, diff_ex(beam_ownership[region][row],
                        lattice_ownership[region][row], ApplySafetyOffset::Yes));
                    common_out = intersection_ex(common_out, diff_ex(lattice_ownership[region][row],
                        beam_ownership[region][row], ApplySafetyOffset::Yes));
                }
                // Rebuild every row of the cell from the common transfer as soon as any row differs.
                // Rewriting only the differing rows leaves the intersection's rounding on the others, which
                // the uniformity check rejects.
                bool any_extra = false;
                std::vector<ExPolygons> extras_in(last_row - first_row + 1), extras_out(last_row - first_row + 1);
                for (size_t row = first_row; row <= last_row; ++row) {
                    extras_in[row - first_row] = diff_ex(diff_ex(beam_ownership[region][row],
                        lattice_ownership[region][row], ApplySafetyOffset::Yes), common_in, ApplySafetyOffset::Yes);
                    extras_out[row - first_row] = diff_ex(diff_ex(lattice_ownership[region][row],
                        beam_ownership[region][row], ApplySafetyOffset::Yes), common_out, ApplySafetyOffset::Yes);
                    any_extra = any_extra || !extras_in[row - first_row].empty() || !extras_out[row - first_row].empty();
                }
                if (!any_extra)
                    continue;
                for (size_t row = first_row; row <= last_row; ++row) {
                    const ExPolygons &extra_in  = extras_in[row - first_row];
                    const ExPolygons &extra_out = extras_out[row - first_row];
                    beam_ownership[region][row] =
                        union_ex(diff_ex(lattice_ownership[region][row], common_out), common_in);
                    for (size_t other = 0; other < region_count; ++other) {
                        if (other == region)
                            continue;
                        ExPolygons &owned = beam_ownership[other][row];
                        if (!extra_out.empty())
                            owned = diff_ex(owned, extra_out);
                        if (!extra_in.empty())
                            owned = union_ex(owned, intersection_ex(extra_in, lattice_ownership[other][row]));
                        owned = diff_ex(owned, beam_ownership[region][row]);
                    }
                }
            }

        trace_units("after per-cell common transfer");

        bool retry_uniformity;
        do {
            retry_uniformity = false;

            // A decision is indivisible evidence. If any overlapping row is reverted,
            // revoke the decision and revert its complete Z range as well.
            for (RegionalInterlockingDecision &decision : m_native_regional_grid_state.interlocking_decisions) {
                if (!decision.apply)
                    continue;
                bool overlaps_rollback = false;
                for (size_t row = 0; row < rollback_rows.size() && !overlaps_rollback; ++row)
                    overlaps_rollback = rollback_rows[row] && row_overlaps_decision(row, decision);
                if (!overlaps_rollback)
                    continue;
                decision.apply = false;
                record_drop(decision.top_z);
                if (beam_trace) {
                    const size_t index = size_t(&decision - trace_decisions.data());
                    for (size_t row = 0; row < rollback_rows.size(); ++row)
                        if (rollback_rows[row] && row_overlaps_decision(row, decision)) {
                            if (drop_reason[index].empty())
                                drop_reason[index] = "row " + std::to_string(row) + " in it was rolled back";
                            break;
                        }
                }
                for (size_t row = 0; row < rollback_rows.size(); ++row)
                    if (row_overlaps_decision(row, decision) && !rollback_rows[row]) {
                        rollback_rows[row] = true;
                        retry_uniformity = true;
                    }
            }

            for (size_t row = 0; row < rollback_rows.size(); ++row)
                if (rollback_rows[row])
                    for (size_t region = 0; region < region_count; ++region)
                        beam_ownership[region][row] = lattice_ownership[region][row];

            // Every receiving multirow cell in every region must have identical signed
            // incoming and outgoing transfer geometry on every row. This is a release-
            // path check; the assertions below merely document the established invariant.
            for (size_t region = 0; region < region_count; ++region)
                for (const NativeRegionCellState &cell : m_native_regional_grid_state.cells[region]) {
                    if (cell.cell.first_lattice_row == cell.cell.last_lattice_row)
                        continue;
                    const size_t first_row = cell.cell.first_lattice_row;
                    const ExPolygons transfer_in = diff_ex(beam_ownership[region][first_row],
                        lattice_ownership[region][first_row], ApplySafetyOffset::Yes);
                    const ExPolygons transfer_out = diff_ex(lattice_ownership[region][first_row],
                        beam_ownership[region][first_row], ApplySafetyOffset::Yes);
                    bool uniform = true;
                    for (size_t row = first_row + 1; row <= cell.cell.last_lattice_row; ++row) {
                        const ExPolygons row_in = diff_ex(beam_ownership[region][row], lattice_ownership[region][row],
                                                         ApplySafetyOffset::Yes);
                        const ExPolygons row_out = diff_ex(lattice_ownership[region][row], beam_ownership[region][row],
                                                          ApplySafetyOffset::Yes);
                        if (!same_transfer(transfer_in, row_in) || !same_transfer(transfer_out, row_out)) {
                            uniform = false;
                            break;
                        }
                    }
                    if (uniform)
                        continue;
                    record_drop(cell.cell.top_z);
                    if (beam_trace)
                        BOOST_LOG_TRIVIAL(warning) << "[BEAM-TRACE] " << this->model_object()->name << " region " << region
                                                   << " cell [" << cell.cell.bottom_z << ", " << cell.cell.top_z
                                                   << "] rows " << first_row << "-" << cell.cell.last_lattice_row
                                                   << " not row-uniform; its rows are rolled back";
                    for (size_t row = first_row; row <= cell.cell.last_lattice_row; ++row)
                        if (!rollback_rows[row]) {
                            rollback_rows[row] = true;
                            retry_uniformity = true;
                        }
                }

            // Body beams: a unit is an interlock only when at least two bodies reach into each other inside
            // it. A unit where only one body gains is a one-way bite, so it is revoked and its rows go back
            // like any other dropped unit. A unit with no transfer at all is left to the publish pass below.
            if (!retry_uniformity)
                for (size_t index = 0; index < trace_decisions.size(); ++index) {
                    RegionalInterlockingDecision &decision = trace_decisions[index];
                    if (!decision.apply)
                        continue;
                    size_t gaining = 0;
                    for (size_t region = 0; region < region_count; ++region) {
                        bool gains = false;
                        for (size_t row = 0; row < rollback_rows.size() && !gains; ++row)
                            gains = row_overlaps_decision(row, decision) &&
                                    !diff_ex(beam_ownership[region][row], lattice_ownership[region][row],
                                             ApplySafetyOffset::Yes).empty();
                        gaining += gains ? 1 : 0;
                    }
                    if (gaining != 1)
                        continue;
                    decision.apply = false;
                    record_drop(decision.top_z);
                    if (drop_reason[index].empty())
                        drop_reason[index] = "one-way: only one body gains in it";
                    for (size_t row = 0; row < rollback_rows.size(); ++row)
                        if (row_overlaps_decision(row, decision) && !rollback_rows[row]) {
                            rollback_rows[row] = true;
                            retry_uniformity = true;
                        }
                }
        } while (retry_uniformity);
        trace_units("after row-uniformity and rollback");

        m_native_regional_grid_state.lattice_ownership = beam_ownership;

        // Publish only decisions that still own real transfer geometry after every rollback has
        // converged. A nominal decision with no surviving transfer is an auditable drop, not an
        // applied mutation.
        for (RegionalInterlockingDecision &decision : m_native_regional_grid_state.interlocking_decisions) {
            if (!decision.apply)
                continue;
            bool generated_transfer = false;
            for (size_t row = 0; row < m_pre_interlocking_slices.size() && !generated_transfer; ++row) {
                if (!row_overlaps_decision(row, decision))
                    continue;
                for (size_t region = 0; region < region_count && !generated_transfer; ++region)
                    generated_transfer =
                        !diff_ex(m_native_regional_grid_state.lattice_ownership[region][row],
                                 m_pre_interlocking_slices[row][region], ApplySafetyOffset::Yes).empty() ||
                        !diff_ex(m_pre_interlocking_slices[row][region],
                                 m_native_regional_grid_state.lattice_ownership[region][row], ApplySafetyOffset::Yes).empty();
            }
            if (!generated_transfer) {
                decision.apply = false;
                record_drop(decision.top_z);
                if (beam_trace)
                    drop_reason[size_t(&decision - trace_decisions.data())] = "no transfer survived in it";
            }
        }
        if (beam_trace)
            for (size_t index = 0; index < trace_decisions.size(); ++index)
                BOOST_LOG_TRIVIAL(warning) << "[BEAM-TRACE] " << this->model_object()->name << " final unit " << index << " ["
                                           << trace_decisions[index].bottom_z << ", " << trace_decisions[index].top_z << "] "
                                           << (trace_decisions[index].apply ? std::string("applied") :
                                               "dropped: " + (drop_reason[index].empty() ? std::string("unknown") : drop_reason[index]));

        // Report when every recorded interlock decision was dropped, rather than leaving that only in
        // the plan records. This is a non-critical warning: the slice remains valid.
        if (config().interlocking_beam.value && !m_native_regional_grid_state.interlocking_decisions.empty()) {
            const size_t voxels = m_native_regional_grid_state.interlocking_decisions.size();
            const size_t applied = size_t(std::count_if(
                m_native_regional_grid_state.interlocking_decisions.begin(),
                m_native_regional_grid_state.interlocking_decisions.end(),
                [](const RegionalInterlockingDecision &decision) { return decision.apply; }));
            if (applied == 0)
                this->active_step_add_warning(PrintStateBase::WarningLevel::NON_CRITICAL,
                    _u8L("Interlocking beams were requested but none could be applied: every beam "
                         "voxel at the body interface was dropped (SRL-R04), so the bodies will "
                         "meet in a plain butt joint.") +
                    "\n" + _u8L("Object name") + ": " + this->model_object()->name +
                    "\n" + _u8L("Dropped beam voxels") + ": " + std::to_string(voxels));
        }

        m_native_regional_grid_state.rendezvous.erase(
            std::remove_if(m_native_regional_grid_state.rendezvous.begin(),
                           m_native_regional_grid_state.rendezvous.end(),
                [](const RendezvousPlane &plane) {
                    return plane.reason == RegionalRendezvousReason::InterlockingDropped;
                }),
            m_native_regional_grid_state.rendezvous.end());
        std::sort(dropped.begin(), dropped.end(), [](const RendezvousPlane &lhs, const RendezvousPlane &rhs) {
            return lhs.z < rhs.z;
        });
        dropped.erase(std::unique(dropped.begin(), dropped.end(), [](const RendezvousPlane &lhs, const RendezvousPlane &rhs) {
            return std::abs(lhs.z - rhs.z) <= 1e-8;
        }), dropped.end());
        dropped.erase(std::remove_if(dropped.begin(), dropped.end(), [this](const RendezvousPlane &plane) {
            return std::any_of(plane.regions.begin(), plane.regions.end(), [this, &plane](size_t region) {
                return region >= m_native_regional_grid_state.cells.size() ||
                       std::none_of(m_native_regional_grid_state.cells[region].begin(),
                                    m_native_regional_grid_state.cells[region].end(),
                                    [&plane](const NativeRegionCellState &cell) {
                                        return std::abs(cell.cell.top_z - plane.z) <= 1e-8;
                                    });
            });
        }), dropped.end());
        m_native_regional_grid_state.rendezvous.insert(m_native_regional_grid_state.rendezvous.end(),
                                                       dropped.begin(), dropped.end());

        for (size_t region = 0; region < region_count; ++region)
            for (NativeRegionCellState &cell : m_native_regional_grid_state.cells[region]) {
                const size_t first_row = cell.cell.first_lattice_row;
                const ExPolygons transfer_in = diff_ex(beam_ownership[region][first_row], lattice_ownership[region][first_row],
                                                        ApplySafetyOffset::Yes);
                const ExPolygons transfer_out = diff_ex(lattice_ownership[region][first_row], beam_ownership[region][first_row],
                                                         ApplySafetyOffset::Yes);
                for (size_t row = first_row + 1; row <= cell.cell.last_lattice_row; ++row) {
                    assert(same_transfer(transfer_in, diff_ex(beam_ownership[region][row], lattice_ownership[region][row],
                                                             ApplySafetyOffset::Yes)));
                    assert(same_transfer(transfer_out, diff_ex(lattice_ownership[region][row], beam_ownership[region][row],
                                                              ApplySafetyOffset::Yes)));
                }
                cell.footprint = union_ex(diff_ex(cell.footprint, transfer_out, ApplySafetyOffset::Yes), transfer_in);
            }
        for (Layer *layer : m_layers)
            delete layer;
        std::vector<coordf_t> event_ranges;
        event_ranges.reserve(2 * m_native_regional_grid_state.event_planes.size());
        coordf_t event_bottom = 0.;
        for (coordf_t event_top : m_native_regional_grid_state.event_planes) {
            event_ranges.push_back(event_bottom);
            event_ranges.push_back(event_top);
            event_bottom = event_top;
        }
        m_layers = new_layers(this, event_ranges);
        for (Layer *layer : m_layers) {
            layer->m_regions.reserve(m_shared_regions->all_regions.size());
            for (const std::unique_ptr<PrintRegion> &pr : m_shared_regions->all_regions)
                layer->m_regions.emplace_back(new LayerRegion(layer, pr.get()));
        }

        const coordf_t object_z_offset = slicing_parameters().object_print_z_min;
        const auto event_index = [this, object_z_offset](coordf_t z) {
            z += object_z_offset;
            const auto it = std::lower_bound(m_layers.begin(), m_layers.end(), z - 1e-8,
                [](const Layer *layer, coordf_t value) { return layer->print_z < value; });
            assert(it != m_layers.end() && std::abs((*it)->print_z - z) <= 1e-7);
            return size_t(it - m_layers.begin());
        };
        // Which logical filament deposits each cell of an opted-in region. A converted (fine-skin) cell
        // takes its fine partner's body filament; every other cell keeps its own. Regions that did not opt
        // in stamp nothing, so the stream is unchanged with the key off.
        std::vector<int>  body_filament(region_count, 0);
        std::vector<char> fine_skin_active(region_count, 0);
        std::vector<size_t> fine_skin_partner(region_count, size_t(-1));
        for (size_t region = 0; region < region_count; ++region)
            body_filament[region] = printing_region(region).config().outer_wall_filament_id.value;
        // With two regions the partner is the other region. With more (painting, or a second coarse body)
        // it is the fine body: the lowest base-cadence body region with cells, else the lowest
        // base-cadence painted region with cells. Only coarse body regions convert, so only they are
        // stamped.
        size_t fine_body_partner = size_t(-1);
        size_t fine_paint_partner = size_t(-1);
        for (size_t region = 0; region < region_count; ++region) {
            if (m_native_regional_grid_state.cells[region].empty() ||
                !is_approx(m_region_slicing[region].layer_height, base_h))
                continue;
            size_t &slot = is_body_region[region] ? fine_body_partner : fine_paint_partner;
            if (slot == size_t(-1))
                slot = region;
        }
        const size_t fine_partner = fine_body_partner != size_t(-1) ? fine_body_partner : fine_paint_partner;
        for (size_t region = 0; region < region_count; ++region) {
            const PrintRegionConfig &rc = printing_region(region).config();
            if (!rc.mixed_nozzle_body_fine_skins.value)
                continue;
            if (region_count == 2) {
                fine_skin_active[region] = 1;
                fine_skin_partner[region] = 1 - region;
            } else if (is_body_region[region] && m_region_slicing[region].layer_height > base_h + EPSILON) {
                const bool converts = std::any_of(m_native_regional_grid_state.cells[region].begin(),
                    m_native_regional_grid_state.cells[region].end(),
                    [](const NativeRegionCellState &cell) { return cell.cell.fine_skin; });
                if (converts && fine_partner == size_t(-1))
                    throw Slic3r::SlicingError(
                        "Fine skins were planned on a coarse body, but the object has no fine body to print them.",
                        this->model_object()->id().id);
                fine_skin_active[region] = 1;
                fine_skin_partner[region] = fine_partner;
            }
        }

        m_region_grids.assign(region_count, {});
        for (size_t region = 0; region < region_count; ++region) {
            auto &grid = m_region_grids[region];
            grid.reserve(m_native_regional_grid_state.cells[region].size());
            for (const NativeRegionCellState &cell : m_native_regional_grid_state.cells[region]) {
                LayerRegion *host = m_layers[event_index(cell.cell.top_z)]->get_region(int(region));
                host->slices.append(cell.footprint, stInternal);
                grid.push_back(host);
            }
            for (size_t index = 0; index < grid.size(); ++index) {
                const RegionCell &cell = m_native_regional_grid_state.cells[region][index].cell;
                Layer *lower_event = cell.bottom_z <= 1e-8 ? nullptr : m_layers[event_index(cell.bottom_z)];
                const size_t host_index = event_index(cell.top_z);
                Layer *upper_event = host_index + 1 < m_layers.size() ? m_layers[host_index + 1] : nullptr;
                grid[index]->set_cell_metadata(cell.bottom_z + object_z_offset, cell.height, cell.slice_z, cell.cell_index,
                    index == 0 ? nullptr : grid[index - 1], index + 1 < grid.size() ? grid[index + 1] : nullptr,
                    lower_event, upper_event);
                if (fine_skin_active[region])
                    grid[index]->set_cell_filament_id(cell.fine_skin && fine_skin_partner[region] != size_t(-1) ?
                                                          body_filament[fine_skin_partner[region]] :
                                                          body_filament[region]);
            }
        }

        coordf_t previous_event = object_z_offset;
        for (Layer *layer : m_layers) {
            LayerRegionPtrs slab_cells(region_count, nullptr);
            ExPolygons coverage;
            size_t ending_cells = 0;
            for (size_t region = 0; region < region_count; ++region) {
                const auto &cells = m_native_regional_grid_state.cells[region];
                const auto containing = std::find_if(cells.begin(), cells.end(), [layer, object_z_offset](const NativeRegionCellState &cell) {
                    const coordf_t local_z = layer->print_z - object_z_offset;
                    return cell.cell.bottom_z < local_z - 1e-8 && cell.cell.top_z >= local_z - 1e-8;
                });
                if (containing != cells.end()) {
                    const size_t index = size_t(containing - cells.begin());
                    slab_cells[region] = m_region_grids[region][index];
                    coverage.insert(coverage.end(), containing->footprint.begin(), containing->footprint.end());
                }
                if (layer->get_region(int(region))->has_cell())
                    ++ending_cells;
            }
            layer->set_slab_coverage(union_ex(coverage), std::move(slab_cells));
            assert(ending_cells > 0);
            assert(std::abs(layer->height - (layer->print_z - previous_event)) <= 1e-7);
            previous_event = layer->print_z;
        }
        for (size_t region = 0; region < region_count; ++region) {
            assert(!m_region_grids[region].empty());
            coordf_t bottom = object_z_offset;
            for (const LayerRegion *cell : m_region_grids[region]) {
                assert(cell->has_cell() && cell->height() > 0.);
                assert(std::abs(cell->bottom_z() - bottom) <= 1e-7);
                // Cell 0 carries the first-layer height and need not be a base multiple.
                assert(cell->bottom_z() <= object_z_offset + 1e-8 ||
                       std::abs(cell->height() / base_h - std::round(cell->height() / base_h)) <= 1e-7);
                bottom = cell->layer()->print_z;
            }
        }
        m_rendezvous = m_native_regional_grid_state.rendezvous;
    } else {
        std::vector<std::vector<ExPolygons>> region_slices =
            slices_to_regions(print->config(), *this, this->model_object()->volumes, *m_shared_regions, slice_zs,
                              std::move(objSliceByVolume), PrintObject::clip_multipart_objects, throw_on_cancel_callback);
        for (size_t region_id = 0; region_id < region_slices.size(); ++region_id)
            for (size_t layer_id = 0; layer_id < region_slices[region_id].size(); ++layer_id)
                m_layers[layer_id]->regions()[region_id]->slices.append(std::move(region_slices[region_id][layer_id]), stInternal);
    }

    BOOST_LOG_TRIVIAL(debug) << "Slicing volumes - removing top empty layers";
    while (! m_layers.empty()) {
        const Layer *layer = m_layers.back();
        if (! layer->empty())
            break;
        delete layer;
        m_layers.pop_back();
    }
    if (! m_layers.empty())
        m_layers.back()->upper_layer = nullptr;
    m_print->throw_if_canceled();

    if (!conical_overhang_applied)
        this->apply_conical_overhang();

    // Is any ModelVolume multi-material painted?
    if (const auto& volumes = this->model_object()->volumes;
        m_print->config().filament_diameter.size() > 1 && // BBS
        std::find_if(volumes.begin(), volumes.end(), [](const ModelVolume* v) { return !v->mmu_segmentation_facets.empty(); }) != volumes.end()) {

        // If XY Size compensation is also enabled, notify the user that XY Size compensation
        // would not be used because the object is multi-material painted.
        if (m_config.xy_hole_compensation.value != 0.f || m_config.xy_contour_compensation.value != 0.f) {
            this->active_step_add_warning(
                PrintStateBase::WarningLevel::CRITICAL,
                L("An object's XY size compensation will not be used because it is also color-painted.\nXY Size "
                  "compensation cannot be combined with color-painting."));
            BOOST_LOG_TRIVIAL(info) << "xy compensation will not work for object " << this->model_object()->name << " for multi filament.";
        }

        BOOST_LOG_TRIVIAL(debug) << "Slicing volumes - MMU segmentation";
        if (!native_grids)
            apply_mm_segmentation(*this, [print]() { print->throw_if_canceled(); });
    }

    // Is any ModelVolume fuzzy skin painted?
    if (this->model_object()->is_fuzzy_skin_painted()) {
        // If XY Size compensation is also enabled, notify the user that XY Size compensation
        // would not be used because the object has custom fuzzy skin painted.
        if (m_config.xy_hole_compensation.value != 0.f || m_config.xy_contour_compensation.value != 0.f) {
            this->active_step_add_warning(
                PrintStateBase::WarningLevel::CRITICAL,
                _u8L("An object has enabled XY Size compensation which will not be used because it is also fuzzy skin painted.\nXY Size "
                     "compensation cannot be combined with fuzzy skin painting.") +
                    "\n" + (_u8L("Object name")) + ": " + this->model_object()->name);
        }

        BOOST_LOG_TRIVIAL(debug) << "Slicing volumes - Fuzzy skin segmentation";
        if (native_grids)
            apply_body_split_fuzzy_skin_masks(*this, [print]() { print->throw_if_canceled(); });
        else
            apply_fuzzy_skin_segmentation(*this, [print]() { print->throw_if_canceled(); });
    }

    if (!native_grids)
        InterlockingGenerator::generate_interlocking_structure(this, [print]() { print->throw_if_canceled(); });
    m_print->throw_if_canceled();

    BOOST_LOG_TRIVIAL(debug) << "Slicing volumes - make_slices in parallel - begin";
    {
        // Compensation value, scaled. Only applying the negative scaling here, as the positive scaling has already been applied during slicing.
        const size_t num_extruders = print->config().filament_diameter.size();
        const auto   xy_hole_scaled = (num_extruders > 1 && this->is_mm_painted()) ? scaled<float>(0.f) : scaled<float>(m_config.xy_hole_compensation.value);
        const auto   xy_contour_scaled            = (num_extruders > 1 && this->is_mm_painted()) ? scaled<float>(0.f) : scaled<float>(m_config.xy_contour_compensation.value);
        const float  elephant_foot_compensation_scaled = (m_config.raft_layers == 0) ?
        	// Only enable Elephant foot compensation if printing directly on the print bed.
            float(scale_(m_config.elefant_foot_compensation.value)) :
        	0.f;
        // Uncompensated slices for the layers in case the Elephant foot compensation is applied.
        std::vector<ExPolygons> lslices_elfoot_uncompensated;
        lslices_elfoot_uncompensated.resize(elephant_foot_compensation_scaled > 0 ? std::min(m_config.elefant_foot_compensation_layers.value, (int)m_layers.size()) : 0);
        //BBS: this part has been changed a lot to support seperated contour and hole size compensation
	    tbb::parallel_for(
	        tbb::blocked_range<size_t>(0, m_layers.size()),
			[this, native_grids, xy_hole_scaled, xy_contour_scaled, elephant_foot_compensation_scaled, &lslices_elfoot_uncompensated](const tbb::blocked_range<size_t>& range) {
	            for (size_t layer_id = range.begin(); layer_id < range.end(); ++ layer_id) {
	                m_print->throw_if_canceled();
	                Layer *layer = m_layers[layer_id];
	                // Apply size compensation and perform clipping of multi-part objects.
	                float elfoot = elephant_foot_compensation_scaled > 0 && layer_id < m_config.elefant_foot_compensation_layers.value ? 
                        elephant_foot_compensation_scaled - (elephant_foot_compensation_scaled / m_config.elefant_foot_compensation_layers.value) * layer_id : 
                        0.f;
	                if (layer->m_regions.size() == 1) {
	                    // Optimized version for a single region layer.
	                    // Single region, growing or shrinking.
	                    LayerRegion *layerm = layer->m_regions.front();
                        if (elfoot > 0) {
		                    // Apply the elephant foot compensation and store the original layer slices without the Elephant foot compensation applied.
                            ExPolygons expolygons_to_compensate = to_expolygons(std::move(layerm->slices.surfaces));
                            if (xy_contour_scaled > 0 || xy_hole_scaled > 0) {
                                expolygons_to_compensate = _shrink_contour_holes(std::max(0.f, xy_contour_scaled),
                                                                   std::max(0.f, xy_hole_scaled),
                                                                   expolygons_to_compensate);
                            }
                            if (xy_contour_scaled < 0 || xy_hole_scaled < 0) {
                                expolygons_to_compensate = _shrink_contour_holes(std::min(0.f, xy_contour_scaled),
                                                                   std::min(0.f, xy_hole_scaled),
                                                                   expolygons_to_compensate);
                            }
                            lslices_elfoot_uncompensated[layer_id] = expolygons_to_compensate;
							layerm->slices.set(
								union_ex(
									Slic3r::elephant_foot_compensation(expolygons_to_compensate,
	                            		layerm->flow(frExternalPerimeter), unscale<double>(elfoot))),
								stInternal);
	                    } else {
	                        // Apply the XY contour and hole size compensation.
                            if (xy_contour_scaled != 0.0f || xy_hole_scaled != 0.0f) {
                                ExPolygons expolygons = to_expolygons(std::move(layerm->slices.surfaces));
                                if (xy_contour_scaled > 0 || xy_hole_scaled > 0) {
                                    expolygons = _shrink_contour_holes(std::max(0.f, xy_contour_scaled),
                                                                       std::max(0.f, xy_hole_scaled),
                                                                       expolygons);
                                }
                                if (xy_contour_scaled < 0 || xy_hole_scaled < 0) {
                                    expolygons = _shrink_contour_holes(std::min(0.f, xy_contour_scaled),
                                                                       std::min(0.f, xy_hole_scaled),
                                                                       expolygons);
                                }
                                layerm->slices.set(std::move(expolygons), stInternal);
                            }
	                    }
	                } else {
                        float max_growth = std::max(xy_hole_scaled, xy_contour_scaled);
                        float min_growth = std::min(xy_hole_scaled, xy_contour_scaled);
                        ExPolygons merged_poly_for_holes_growing;
                        if (max_growth > 0) {
                            //BBS: merge polygons because region can cut "holes".
                            //Then, cut them to give them again later to their region
                            merged_poly_for_holes_growing = layer->merged(float(SCALED_EPSILON));
                            merged_poly_for_holes_growing = _shrink_contour_holes(std::max(0.f, xy_contour_scaled),
                                                                                  std::max(0.f, xy_hole_scaled),
                                                                                  union_ex(merged_poly_for_holes_growing));

                            // BBS: clipping regions, priority is given to the first regions.
                            Polygons processed;
                            for (size_t region_id = 0; region_id < layer->regions().size(); ++region_id) {
                                // Only a native grid has regions without a cell on a layer. With the mode
                                // off, or for any other object, every region is trimmed as upstream trims it.
                                if (native_grids && !layer->m_regions[region_id]->has_cell())
                                    continue;
                                ExPolygons slices = to_expolygons(std::move(layer->m_regions[region_id]->slices.surfaces));
                                if (max_growth > 0.f) {
                                    slices = intersection_ex(offset_ex(slices, max_growth), merged_poly_for_holes_growing);
                                }

                                //BBS: Trim by the slices of already processed regions.
                                if (region_id > 0)
                                    slices = diff_ex(to_polygons(std::move(slices)), processed);
                                if (region_id + 1 < layer->regions().size())
                                    // Collect the already processed regions to trim the to be processed regions.
                                    polygons_append(processed, slices);
                                layer->m_regions[region_id]->slices.set(std::move(slices), stInternal);
                            }
                        }
                        if (min_growth < 0.f || elfoot > 0.f) {
                            // Apply the negative XY compensation. (the ones that is <0)
                            ExPolygons trimming;
                            static const float eps = float(scale_(m_config.slice_closing_radius.value) * 1.5);
                            if (elfoot > 0.f) {
                                ExPolygons expolygons_to_compensate = offset_ex(layer->merged(eps), -eps);
                                lslices_elfoot_uncompensated[layer_id] = expolygons_to_compensate;
                                trimming = Slic3r::elephant_foot_compensation(expolygons_to_compensate,
                                    layer->m_regions.front()->flow(frExternalPerimeter), unscale<double>(elfoot));
                            } else {
                                trimming = layer->merged(float(SCALED_EPSILON));
                            }
                            if (min_growth < 0.0f)
                                trimming = _shrink_contour_holes(std::min(0.f, xy_contour_scaled),
                                                                 std::min(0.f, xy_hole_scaled),
                                                                 trimming);
                            //BBS: trim surfaces
                            for (size_t region_id = 0; region_id < layer->regions().size(); ++region_id) {
                                if (native_grids && !layer->regions()[region_id]->has_cell())
                                    continue;
                                // BBS: split trimming result by region
                                ExPolygons contour_exp = to_expolygons(std::move(layer->regions()[region_id]->slices.surfaces));

                                layer->regions()[region_id]->slices.set(intersection_ex(contour_exp, to_polygons(trimming)), stInternal);
                            }
                        }
	                }
	                // Merge all regions' slices to get islands, chain them by a shortest path.
	                layer->make_slices();
	            }
	        });
	    if (elephant_foot_compensation_scaled > 0.f && ! m_layers.empty()) {
	    	// The Elephant foot has been compensated, therefore the elefant_foot_compensation_layers layer's lslices are shrank with the Elephant foot compensation value.
	    	// Store the uncompensated value there.
	    	assert(m_layers.front()->id() == 0);
            //BBS: sort the lslices_elfoot_uncompensated according to shortest path before saving
            //Otherwise the travel of the layer layer would be mess.
            for (int i = 0; i < lslices_elfoot_uncompensated.size(); i++) {
                ExPolygons &expolygons_uncompensated = lslices_elfoot_uncompensated[i];
                Points ordering_points;
                ordering_points.reserve(expolygons_uncompensated.size());
                for (const ExPolygon &ex : expolygons_uncompensated)
                    ordering_points.push_back(ex.contour.first_point());
                std::vector<Points::size_type> order = chain_points(ordering_points);
                ExPolygons lslices_sorted;
                lslices_sorted.reserve(expolygons_uncompensated.size());
                for (size_t i : order)
                    lslices_sorted.emplace_back(std::move(expolygons_uncompensated[i]));
                m_layers[i]->lslices = std::move(lslices_sorted);
            }
		}
	}

    m_print->throw_if_canceled();
    BOOST_LOG_TRIVIAL(debug) << "Slicing volumes - make_slices in parallel - end";
    this->capture_mixed_nozzle_body_observations();
}

void PrintObject::apply_conical_overhang(bool use_row_spacing) {
    BOOST_LOG_TRIVIAL(info) << "Make overhang printable...";

    if (m_layers.empty()) {
        return;
    }
    
    const double conical_overhang_angle = this->config().make_overhang_printable_angle;
    if (conical_overhang_angle == 90.0) {
        return;
    }
    const double angle_radians = conical_overhang_angle * M_PI / 180.;
    const double max_hole_area = this->config().make_overhang_printable_hole_size; // in MM^2
    const double tan_angle = tan(angle_radians); // the XY-component of the angle
    BOOST_LOG_TRIVIAL(info) << "angle " << angle_radians << " maxHoleArea " << max_hole_area << " tan_angle "
                            << tan_angle;
    const coordf_t layer_thickness = m_config.layer_height.value;
    const coordf_t max_dist_from_lower_layer = tan_angle * layer_thickness; // max dist which can be bridged, in MM
    BOOST_LOG_TRIVIAL(info) << "layer_thickness " << layer_thickness << " max_dist_from_lower_layer "
                            << max_dist_from_lower_layer;

    // Pre-scale config
    const coordf_t scaled_max_dist_from_lower_layer = -float(scale_(max_dist_from_lower_layer));
    const coordf_t scaled_max_hole_area = float(scale_(scale_(max_hole_area)));


    for (auto i = m_layers.rbegin() + 1; i != m_layers.rend(); ++i) {
        m_print->throw_if_canceled();
        Layer *layer = *i;
        Layer *upper_layer = layer->upper_layer;

        if (upper_layer->empty()) {
          continue;
        }

        // Skip if entire layer has this disabled
        if (std::all_of(layer->m_regions.begin(), layer->m_regions.end(),
                        [](const LayerRegion *r) { return  r->slices.empty() || !r->region().config().make_overhang_printable; })) {
            continue;
        }

        //layer->export_region_slices_to_svg_debug("layer_before_conical_overhang");
        //upper_layer->export_region_slices_to_svg_debug("upper_layer_before_conical_overhang");


        // Merge the upper layer because we want to offset the entire layer uniformly, otherwise
        // the model could break at the region boundary.
        auto upper_poly = upper_layer->merged(float(SCALED_EPSILON));
        upper_poly = union_ex(upper_poly);

        // Merge layer for the same reason
        auto current_poly = layer->merged(float(SCALED_EPSILON));
        current_poly = union_ex(current_poly);

        // Avoid closing up of recessed holes in the base of a model.
        // Detects when a hole is completely covered by the layer above and removes the hole from the layer above before
        // adding it in.
        // This should have no effect any time a hole in a layer interacts with any polygon in the layer above
        if (scaled_max_hole_area > 0.0) {

            // Now go through all the holes in the current layer and check if they intersect anything in the layer above
            // If not, then they're the top of a hole and should be cut from the layer above before the union
            for (auto layer_polygon : current_poly) {
                for (auto hole : layer_polygon.holes) {
                    if (std::abs(hole.area()) < scaled_max_hole_area) {
                        ExPolygon hole_poly(hole);
                        auto hole_with_above = intersection_ex(upper_poly, hole_poly);
                        if (!hole_with_above.empty()) {
                            // The hole had some intersection with the above layer, check if it's a complete overlap
                            auto hole_difference = xor_ex(hole_with_above, hole_poly);
                            if (hole_difference.empty()) {
                                // The layer above completely cover it, remove it from the layer above
                                upper_poly = diff_ex(upper_poly, hole_poly);
                            }
                        }
                    }
                }
            }
        }

        // Now offset the upper layer to be added into current layer
        if (use_row_spacing)
            upper_poly = offset_ex(upper_poly, -float(scale_(tan_angle * std::max(0., upper_layer->slice_z - layer->slice_z))));
        else
            upper_poly = offset_ex(upper_poly, scaled_max_dist_from_lower_layer);

        for (size_t region_id = 0; region_id < this->num_printing_regions(); ++region_id) {
            // export_to_svg(debug_out_path("Surface-obj-%d-layer-%d-region-%d.svg", id().id, layer->id(), region_id).c_str(),
            //               layer->m_regions[region_id]->slices.surfaces);

            // Disable on given region
            if (!upper_layer->m_regions[region_id]->region().config().make_overhang_printable) {
                continue;
            }

            // Calculate the scaled upper poly that belongs to current region
            auto p = union_ex(intersection_ex(upper_layer->m_regions[region_id]->slices.surfaces, upper_poly));

            // Remove all islands that have already been fully covered by current layer
            p.erase(std::remove_if(p.begin(), p.end(), [&current_poly](const ExPolygon& ex) {
                return diff_ex(ex, current_poly).empty();
            }), p.end());
            // On Body Split rows (the only caller of the row spacing form) the cone fills air only. Left
            // whole, a coarse lip's island would take every other body under it, so a fine body beneath the
            // lip would lose its cells. Native objects keep upstream's rule.
            if (use_row_spacing)
                p = diff_ex(p, current_poly);

            // And now union it with current region
            ExPolygons layer_polygons = to_expolygons(layer->m_regions[region_id]->slices.surfaces);
            layer->m_regions[region_id]->slices.set(union_ex(layer_polygons, p), stInternal);

            // Then remove it from all other regions, to avoid overlapping regions
            for (size_t other_region = 0; other_region < this->num_printing_regions(); ++other_region) {
                if (other_region == region_id) {
                    continue;
                }
                ExPolygons s = to_expolygons(layer->m_regions[other_region]->slices.surfaces);
                layer->m_regions[other_region]->slices.set(diff_ex(s, p, ApplySafetyOffset::Yes), stInternal);
            }
        }
        //layer->export_region_slices_to_svg_debug("layer_after_conical_overhang");
    }
}

void PrintObject::apply_conical_overhang_to_rows(const std::vector<coordf_t> &slice_zs,
                                                 std::vector<std::vector<ExPolygons>> &rows)
{
    struct OwnedRowLayers {
        LayerPtrs layers;
        ~OwnedRowLayers()
        {
            for (Layer *layer : layers)
                delete layer;
        }
        OwnedRowLayers() = default;
        OwnedRowLayers(const OwnedRowLayers &) = delete;
        OwnedRowLayers &operator=(const OwnedRowLayers &) = delete;
    } scratch;
    const size_t region_count = rows.size();
    const coordf_t base_h = config().layer_height.value;
    scratch.layers.reserve(slice_zs.size());
    for (size_t row = 0; row < slice_zs.size(); ++row) {
        Layer *layer = new Layer(row, this, base_h, slice_zs[row], slice_zs[row]);
        layer->m_regions.reserve(m_shared_regions->all_regions.size());
        for (const std::unique_ptr<PrintRegion> &pr : m_shared_regions->all_regions)
            layer->m_regions.emplace_back(new LayerRegion(layer, pr.get()));
        for (size_t region_id = 0; region_id < region_count && region_id < layer->m_regions.size(); ++region_id)
            layer->get_region(int(region_id))->slices.set(rows[region_id][row], stInternal);
        if (!scratch.layers.empty()) {
            scratch.layers.back()->upper_layer = layer;
            layer->lower_layer = scratch.layers.back();
        }
        scratch.layers.push_back(layer);
    }

    LayerPtrs saved_layers = std::move(m_layers);
    try {
        m_layers = scratch.layers;
        this->apply_conical_overhang(true);
    } catch (...) {
        m_layers = std::move(saved_layers);
        throw;
    }
    m_layers = std::move(saved_layers);
    for (size_t row = 0; row < scratch.layers.size(); ++row)
        for (size_t region_id = 0; region_id < region_count && region_id < scratch.layers[row]->m_regions.size(); ++region_id)
            rows[region_id][row] = to_expolygons(scratch.layers[row]->get_region(int(region_id))->slices.surfaces);
}

//BBS: this function is used to offset contour and holes of expolygons seperately by different value
ExPolygons PrintObject::_shrink_contour_holes(double contour_delta, double hole_delta, const ExPolygons& polys) const
{
    ExPolygons new_ex_polys;
    for (const ExPolygon& ex_poly : polys) {
        Polygons contours;
        Polygons holes;
        //BBS: modify hole
        for (const Polygon& hole : ex_poly.holes) {
            if (hole_delta != 0) {
                for (Polygon& newHole : offset(hole, -hole_delta)) {
                    newHole.make_counter_clockwise();
                    holes.emplace_back(std::move(newHole));
                }
            } else {
                holes.push_back(hole);
                holes.back().make_counter_clockwise();
            }
        }
        //BBS: modify contour
        if (contour_delta != 0) {
            Polygons new_contours = offset(ex_poly.contour, contour_delta);
            if (new_contours.size() == 0)
                continue;
            contours.insert(contours.end(), std::make_move_iterator(new_contours.begin()), std::make_move_iterator(new_contours.end()));
        } else {
            contours.push_back(ex_poly.contour);
        }
        ExPolygons temp = diff_ex(union_(contours), union_(holes));
        new_ex_polys.insert(new_ex_polys.end(), std::make_move_iterator(temp.begin()), std::make_move_iterator(temp.end()));
    }
    return union_ex(new_ex_polys);
}

std::vector<Polygons> PrintObject::slice_support_volumes(const ModelVolumeType model_volume_type) const
{
    auto it_volume     = this->model_object()->volumes.begin();
    auto it_volume_end = this->model_object()->volumes.end();
    for (; it_volume != it_volume_end && (*it_volume)->type() != model_volume_type; ++ it_volume) ;
    std::vector<Polygons> slices;
    if (it_volume != it_volume_end) {
        // Found at least a single support volume of model_volume_type.
        std::vector<float> zs = zs_from_layers(this->layers());
        std::vector<char>  merge_layers;
        bool               merge = false;
        const Print       *print = this->print();
        auto               throw_on_cancel_callback = std::function<void()>([print](){ print->throw_if_canceled(); });
        MeshSlicingParamsEx params;
        params.trafo = this->trafo_centered();
        for (; it_volume != it_volume_end; ++ it_volume)
            if ((*it_volume)->type() == model_volume_type) {
                std::vector<ExPolygons> slices2 = slice_volume(*(*it_volume), zs, params, throw_on_cancel_callback);
                if (slices.empty()) {
                    slices.reserve(slices2.size());
                    for (ExPolygons &src : slices2)
                        slices.emplace_back(to_polygons(std::move(src)));
                } else if (!slices2.empty()) {
                    if (merge_layers.empty())
                        merge_layers.assign(zs.size(), false);
                    for (size_t i = 0; i < zs.size(); ++ i) {
                        if (slices[i].empty())
                            slices[i] = to_polygons(std::move(slices2[i]));
                        else if (! slices2[i].empty()) {
                            append(slices[i], to_polygons(std::move(slices2[i])));
                            merge_layers[i] = true;
                            merge = true;
                        }
                    }
                }
            }
        if (merge) {
            std::vector<Polygons*> to_merge;
            to_merge.reserve(zs.size());
            for (size_t i = 0; i < zs.size(); ++ i)
                if (merge_layers[i])
                    to_merge.emplace_back(&slices[i]);
            tbb::parallel_for(
                tbb::blocked_range<size_t>(0, to_merge.size()),
                [&to_merge](const tbb::blocked_range<size_t> &range) {
                    for (size_t i = range.begin(); i < range.end(); ++ i)
                        *to_merge[i] = union_(*to_merge[i]);
            });
        }
    }
    return slices;
}

} // namespace Slic3r
