#ifndef slic3r_FillConcentricInternal_hpp_
#define slic3r_FillConcentricInternal_hpp_

#include "FillBase.hpp"

namespace Slic3r {

class PrintConfig;

// The nozzle diameter the narrow-internal-solid beading is sized from, as a named, testable
// decision. Fill.cpp re-patterns a narrow internal-solid area onto this fill, and the Arachne bead
// floor derived here decides how narrow a road that region may lay.
double concentric_internal_bead_nozzle(const FillParams &params, const PrintConfig &print_config);

class FillConcentricInternal : public Fill
{
public:
    ~FillConcentricInternal() override = default;
    void fill_surface_extrusion(const Surface *surface, const FillParams &params, ExtrusionEntitiesPtr &out) override;
    bool is_self_crossing() override { return false; }

protected:
    Fill* clone() const override { return new FillConcentricInternal(*this); };
    bool no_sort() const override { return true; }

    friend class Layer;
};

} // namespace Slic3r

#endif // slic3r_FillConcentricInternal_hpp_
