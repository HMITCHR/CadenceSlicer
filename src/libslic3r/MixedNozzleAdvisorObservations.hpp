#ifndef slic3r_MixedNozzleAdvisorObservations_hpp_
#define slic3r_MixedNozzleAdvisorObservations_hpp_

#include "ExtrusionEntity.hpp"
#include "ObjectID.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r {

// Value-only evidence published by the native mixed-nozzle pipeline. Geometry workers must not
// hand out LayerRegion/PrintRegion pointers or references into temporary stage storage through
// this interface.
enum class AdvisorMode {
    FeatureSplit,
    BodySplit,
};

enum class AdvisorFeatureOutcome {
    Committed,
    Vetoed,
    FineFallback,
    Unavailable,
};

enum class AdvisorExtrusionSource {
    NativeLayerRegion,
    FinalMoveStream,
};

struct AdvisorObservationStamp {
    static constexpr std::uint32_t current_schema_version = 3;

    std::uint32_t schema_version { current_schema_version };
    ObjectID      object_id;
    // ModelObject/ModelVolume inherit ObjectBase::timestamp(), which is always zero. Keep
    // native config and geometry identity separate so a consumer can reject stale evidence
    // without mistaking an object ID for a content revision.
    std::uint64_t source_config_timestamp { 0 };
    std::uint64_t source_transform_signature { 0 };
    std::vector<std::uint64_t> source_volume_config_timestamps;
    std::vector<std::uint64_t> source_volume_facet_timestamps;
    std::vector<std::uint64_t> source_volume_mesh_identities;
    std::vector<std::uint64_t> source_volume_transform_signatures;
    std::vector<std::size_t>   region_config_hashes;
    std::uint64_t process_step_timestamp { 0 };

    bool operator==(const AdvisorObservationStamp &rhs) const
    {
        return schema_version == rhs.schema_version && object_id == rhs.object_id &&
               source_config_timestamp == rhs.source_config_timestamp &&
               source_transform_signature == rhs.source_transform_signature &&
               source_volume_config_timestamps == rhs.source_volume_config_timestamps &&
               source_volume_facet_timestamps == rhs.source_volume_facet_timestamps &&
               source_volume_mesh_identities == rhs.source_volume_mesh_identities &&
               source_volume_transform_signatures == rhs.source_volume_transform_signatures &&
               region_config_hashes == rhs.region_config_hashes &&
               process_step_timestamp == rhs.process_step_timestamp;
    }
    bool operator!=(const AdvisorObservationStamp &rhs) const { return !(*this == rhs); }
};

struct AdvisorFeatureBand {
    std::size_t region_id { 0 };
    // 1-based position of this band in AdvisorSliceObservations::feature_bands, stamped at
    // publication, so a report row can name the band.
    std::size_t band_index { 0 };
    double       z_low { 0. };
    double       z_high { 0. };
    int          effective_ratio { 1 };
    AdvisorMode  mode { AdvisorMode::FeatureSplit };
    std::optional<std::size_t> fine_physical_tool;
    std::optional<std::size_t> coarse_physical_tool;
    std::optional<double> candidate_area_mm2;
    // Area-derived candidate work estimate; it is deliberately distinct from final extrusion
    // volume and must be absent when the stage cannot provide the bounded estimate.
    std::optional<double> estimated_candidate_volume_mm3;
    // Nominal native path-volume/rate and regenerated tower/switch estimates, seconds.
    // These do not claim firmware, cooling or acceleration-exact elapsed time.
    std::optional<double> fine_model_seconds;
    std::optional<double> coarse_model_seconds;
    // Cost of handing this band to the coarse tool: the sum of the two terms below.
    std::optional<double> incremental_handoff_seconds;
    // The same cost split into nozzle switching (NativeFilamentChangeTransition seconds) and prime
    // tower work (ToolChangeResult elapsed time). Both are coarse-minus-fine trial deltas, absent
    // whenever incremental_handoff_seconds is.
    std::optional<double> incremental_switch_seconds;
    std::optional<double> incremental_tower_seconds;
    // Incremental generated tower material for this band, mm3 (the same coarse-trial minus
    // fine-trial delta over WipeTower::ToolChangeResult::purge_volume). This is trial tower
    // work, not the final tower's real material: do not sum it across bands to report a total.
    std::optional<double> incremental_tower_volume_mm3;
    AdvisorFeatureOutcome outcome { AdvisorFeatureOutcome::Unavailable };
    std::string reason;
};

struct AdvisorBodyCell {
    std::size_t cell_index { 0 };
    std::size_t region_id { 0 };
    double       z_low { 0. };
    double       z_high { 0. };
    double       slice_z { 0. };
    double       height { 0. };
    bool         fine_skin { false };
    std::optional<std::size_t> logical_filament;
    std::optional<std::size_t> physical_tool;
    std::optional<double> footprint_area_mm2;
};

struct AdvisorExtrusionFact {
    std::optional<ObjectID> object_id;
    ExtrusionRole           role { erNone };
    std::optional<std::size_t> physical_tool;
    // Native LayerRegion facts carry the actual logical filament selected by LayerTools,
    // including a Body Split cell override. Final move-stream facts may leave region identity
    // unavailable because the export stream has no stable LayerRegion owner.
    std::optional<std::size_t> logical_filament;
    std::optional<std::size_t> region_id;
    std::optional<double>      print_z;
    double                  volume_mm3 { 0. };
    // Physical path length in millimetres, copied from the native extrusion entity. This is
    // independent of volume and must not be reconstructed from area or a flow estimate.
    double                  path_length_mm { 0. };
    AdvisorExtrusionSource  source { AdvisorExtrusionSource::NativeLayerRegion };
};

struct AdvisorSliceObservations {
    AdvisorObservationStamp stamp;
    AdvisorMode              mode { AdvisorMode::FeatureSplit };
    std::vector<AdvisorFeatureBand>   feature_bands;
    std::vector<AdvisorBodyCell>     body_cells;
    std::vector<AdvisorExtrusionFact> native_extrusions;
    std::optional<std::size_t> total_physical_tool_changes;
};

// A completed snapshot is immutable once published. A null pointer means that no successful,
// stamp-matching native process pass is currently available (including after invalidation or
// cancellation); callers must not infer zero-valued metrics from absence.
using AdvisorSliceObservationsPtr = std::shared_ptr<const AdvisorSliceObservations>;

} // namespace Slic3r

#endif // slic3r_MixedNozzleAdvisorObservations_hpp_
