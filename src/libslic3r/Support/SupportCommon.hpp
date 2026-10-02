#ifndef slic3r_SupportCommon_hpp_
#define slic3r_SupportCommon_hpp_

#include "../Layer.hpp"
#include "../Polygon.hpp"
#include "../Print.hpp"
#include "SupportLayer.hpp"
#include "SupportParameters.hpp"

namespace Slic3r {

class PrintObject;
class SupportLayer;

// Turn some of the base layers into base interface layers.
// For soluble interfaces with non-soluble bases, print maximum two first interface layers with the base
// extruder to improve adhesion of the soluble filament to the base.
// For Organic supports, merge top_interface_layers & top_base_interface_layers with the interfaces
// produced by this function.
std::pair<SupportGeneratorLayersPtr, SupportGeneratorLayersPtr> generate_interface_layers(
    const PrintObjectConfig           &config,
    const SupportParameters           &support_params,
    const SupportGeneratorLayersPtr   &bottom_contacts,
    const SupportGeneratorLayersPtr   &top_contacts,
    // Input / output, will be merged with output
    SupportGeneratorLayersPtr         &top_interface_layers,
    SupportGeneratorLayersPtr         &top_base_interface_layers,
    // Input, will be trimmed with the newly created interface layers.
    SupportGeneratorLayersPtr         &intermediate_layers,
    SupportGeneratorLayerStorage      &layer_storage);

// Generate raft layers, also expand the 1st support layer
// in case there is no raft layer to improve support adhesion.
SupportGeneratorLayersPtr generate_raft_base(
	const PrintObject				&object,
	const SupportParameters			&support_params,
	const SlicingParameters			&slicing_params,
	const SupportGeneratorLayersPtr &top_contacts,
	const SupportGeneratorLayersPtr &interface_layers,
	const SupportGeneratorLayersPtr &base_interface_layers,
	const SupportGeneratorLayersPtr &base_layers,
	SupportGeneratorLayerStorage    &layer_storage);

// When SupportParameters::mixed_nozzle_banded_body is set, the support body is laid by a coarser
// nozzle than the object's layers are cut for. Group the base layers (the support body left after
// the interfaces were carved out) into bands of whole object layers that nozzle can lay, the way
// coarse sparse infill is combined: the area every layer of a band shares moves to the band's top
// layer, which takes the band's full height. Bands aim for the plan's coarse cadence; a bed layer the
// coarse nozzle cannot lay stays at the first layer's height for the interface nozzle, and the first
// band starts on top of it (only without a body filament on that nozzle is the first band on the bed
// the smallest legal one); what a band leaves behind is tried again in the smallest
// legal groups; anything still left joins the band laid over its layer, clear of contacts and
// interfaces, so the base filament prints it; and only what cannot join a band goes to
// fine_body_polygons, for the interface nozzle. So does the body at the height of a base interface,
// which that nozzle lays too. On the first layer that body, and an interface layer there, is printed
// with the part's own filament on that nozzle, never the interface filament.
// Raft layers are left alone.
// Call after generate_interface_layers() and generate_raft_base(), before generate_support_layers().
void mixed_nozzle_band_support_body(
	const PrintObject				&object,
	const SupportParameters			&support_params,
	const SlicingParameters			&slicing_params,
	const SupportGeneratorLayersPtr &bottom_contacts,
	const SupportGeneratorLayersPtr &top_contacts,
	SupportGeneratorLayersPtr       &interface_layers,
	const SupportGeneratorLayersPtr &base_interface_layers,
	SupportGeneratorLayersPtr       &base_layers);

// When SupportParameters::mixed_nozzle_coarse_support is set, body and interface share a nozzle that
// cannot lay the object's layer height. Every support layer is moved onto rows that nozzle can lay:
// each top contact keeps its own Z (so the top Z distance holds), the bottom of each bottom contact
// is a row too, and rows between aim for the coarse cadence. A layer moves up to the next row, or
// down to the row below when a contact too close above had to be dropped, and takes the row's full
// height. A contact bottom only moves up and keeps its lower boundary, so it never reaches into the
// part it stands on; the row over one dropped too close above a kept row is raised to stay legal, and
// slicing stops with a message when no legal row fits. Raft layers are left alone. Call where
// mixed_nozzle_band_support_body() is called.
void mixed_nozzle_coarsen_shared_support(
	const PrintObject				&object,
	const SupportParameters			&support_params,
	const SlicingParameters			&slicing_params,
	SupportGeneratorLayersPtr       &bottom_contacts,
	SupportGeneratorLayersPtr       &top_contacts,
	SupportGeneratorLayersPtr       &interface_layers,
	SupportGeneratorLayersPtr       &base_interface_layers,
	SupportGeneratorLayersPtr       &base_layers);

void tree_supports_generate_paths(ExtrusionEntitiesPtr &dst, const Polygons &polygons, const Flow &flow, const SupportParameters &support_params);

void fill_expolygons_with_sheath_generate_paths(
    ExtrusionEntitiesPtr &dst, const Polygons &polygons, Fill *filler, float density, ExtrusionRole role, const Flow &flow, const SupportParameters& support_params, bool with_sheath, bool no_sort);

// returns sorted layers
SupportGeneratorLayersPtr generate_support_layers(
	PrintObject							&object,
    const SupportGeneratorLayersPtr     &raft_layers,
    const SupportGeneratorLayersPtr     &bottom_contacts,
    const SupportGeneratorLayersPtr     &top_contacts,
    const SupportGeneratorLayersPtr     &intermediate_layers,
    const SupportGeneratorLayersPtr     &interface_layers,
    const SupportGeneratorLayersPtr     &base_interface_layers);

// Produce the support G-code.
// Used by both classic and tree supports.
void generate_support_toolpaths(
	SupportLayerPtrs    				&support_layers,
	const PrintObjectConfig 			&config,
	const SupportParameters 			&support_params,
	const SlicingParameters 			&slicing_params,
    const SupportGeneratorLayersPtr 	&raft_layers,
    const SupportGeneratorLayersPtr   	&bottom_contacts,
    const SupportGeneratorLayersPtr   	&top_contacts,
    const SupportGeneratorLayersPtr   	&intermediate_layers,
	const SupportGeneratorLayersPtr   	&interface_layers,
    const SupportGeneratorLayersPtr   	&base_interface_layers);

// FN_HIGHER_EQUAL: the provided object pointer has a Z value >= of an internal threshold.
// Find the first item with Z value >= of an internal threshold of fn_higher_equal.
// If no vec item with Z value >= of an internal threshold of fn_higher_equal is found, return vec.size()
// If the initial idx is size_t(-1), then use binary search.
// Otherwise search linearly upwards.
template<typename IteratorType, typename IndexType, typename FN_HIGHER_EQUAL>
IndexType idx_higher_or_equal(IteratorType begin, IteratorType end, IndexType idx, FN_HIGHER_EQUAL fn_higher_equal)
{
    auto size = int(end - begin);
    if (size == 0) {
        idx = 0;
    } else if (idx == IndexType(-1)) {
        // First of the batch of layers per thread pool invocation. Use binary search.
        int idx_low  = 0;
        int idx_high = std::max(0, size - 1);
        while (idx_low + 1 < idx_high) {
            int idx_mid  = (idx_low + idx_high) / 2;
            if (fn_higher_equal(begin[idx_mid]))
                idx_high = idx_mid;
            else
                idx_low  = idx_mid;
        }
        idx =  fn_higher_equal(begin[idx_low])  ? idx_low  :
              (fn_higher_equal(begin[idx_high]) ? idx_high : size);
    } else {
        // For the other layers of this batch of layers, search incrementally, which is cheaper than the binary search.
        while (int(idx) < size && ! fn_higher_equal(begin[idx]))
            ++ idx;
    }
    return idx;
}
template<typename T, typename IndexType, typename FN_HIGHER_EQUAL>
IndexType idx_higher_or_equal(const std::vector<T>& vec, IndexType idx, FN_HIGHER_EQUAL fn_higher_equal)
{
    return idx_higher_or_equal(vec.begin(), vec.end(), idx, fn_higher_equal);
}

// FN_LOWER_EQUAL: the provided object pointer has a Z value <= of an internal threshold.
// Find the first item with Z value <= of an internal threshold of fn_lower_equal.
// If no vec item with Z value <= of an internal threshold of fn_lower_equal is found, return -1.
// If the initial idx is < -1, then use binary search.
// Otherwise search linearly downwards.
template<typename IT, typename FN_LOWER_EQUAL>
int idx_lower_or_equal(IT begin, IT end, int idx, FN_LOWER_EQUAL fn_lower_equal)
{
    auto size = int(end - begin);
    if (size == 0) {
        idx = -1;
    } else if (idx < -1) {
        // First of the batch of layers per thread pool invocation. Use binary search.
        int idx_low  = 0;
        int idx_high = std::max(0, size - 1);
        while (idx_low + 1 < idx_high) {
            int idx_mid  = (idx_low + idx_high) / 2;
            if (fn_lower_equal(begin[idx_mid]))
                idx_low  = idx_mid;
            else
                idx_high = idx_mid;
        }
        idx =  fn_lower_equal(begin[idx_high]) ? idx_high :
              (fn_lower_equal(begin[idx_low ]) ? idx_low  : -1);
    } else {
        // For the other layers of this batch of layers, search incrementally, which is cheaper than the binary search.
        while (idx >= 0 && ! fn_lower_equal(begin[idx]))
            -- idx;
    }
    return idx;
}
template<typename T, typename FN_LOWER_EQUAL>
int idx_lower_or_equal(const std::vector<T*> &vec, int idx, FN_LOWER_EQUAL fn_lower_equal)
{
    return idx_lower_or_equal(vec.begin(), vec.end(), idx, fn_lower_equal);
}

} // namespace Slic3r

#endif /* slic3r_SupportCommon_hpp_ */
