#ifndef slic3r_MixedNozzleRanking_hpp_
#define slic3r_MixedNozzleRanking_hpp_

#include "Model.hpp"
#include "Point.hpp"
#include "PrintConfig.hpp"

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

namespace Slic3r {

class Print;

// Cancels one headless ranking slice from any thread. The slice attaches its own Print
// while it runs, so a cancel reaches the Print's own checkpoints, and a cancel that arrives
// before the Print exists stops the slice as soon as it attaches.
class MixedNozzleRankingCancel
{
public:
    void cancel();
    bool cancelled() const { return m_cancelled.load(std::memory_order_acquire); }
    // Called by the slice on its own thread: the Print once it exists, nullptr when it is done.
    void attach(Print *print);

private:
    mutable std::mutex m_mutex;
    Print *m_print {nullptr};
    std::atomic<bool> m_cancelled {false};
};

// What one headless ranking slice runs on. Composed on the main thread from the staged Apply
// result; owned by the slice once handed over, so nothing in it points at live project state.
struct MixedNozzleRankingSlice {
    std::shared_ptr<Model> model;
    DynamicPrintConfig config;
    bool is_bbl_printer {false};
    Vec3d plate_origin {Vec3d::Zero()};
    std::size_t plate_id {0};
    // Plate id, model hash and config hash. Equal keys slice the same way.
    std::string key;
    // Plain words. Set when this row has nothing that can be sliced.
    std::string diagnostic;
};

enum class MixedNozzleSliceTimeStatus {
    Estimated,
    Refused,
    Cancelled,
    Failed,
};

struct MixedNozzleSliceTime {
    MixedNozzleSliceTimeStatus status {MixedNozzleSliceTimeStatus::Failed};
    // The G-code processor's Normal mode print time, in seconds.
    double seconds {0.};
    // Physical tool changes the processor counted.
    unsigned int switches {0};
    // Tower volume over every extruder, mm3.
    double tower_mm3 {0.};
    std::string diagnostic;
};

// Slices `model` with `config` the way a real slice of the plate runs (apply, validate, process,
// G-code export) and reads the time from the G-code processor, so the tower, re-prime and tool
// changes are priced exactly as the real slice prices them. The G-code goes to a temporary file
// under `temp_dir` (the system temporary directory when empty) and is removed afterwards.
MixedNozzleSliceTime mixed_nozzle_slice_time(const Model &model, const DynamicPrintConfig &config,
                                             bool is_bbl_printer, const Vec3d &plate_origin,
                                             const std::string &temp_dir,
                                             MixedNozzleRankingCancel &cancel);

// The all-fine single-nozzle baseline, made from a composed mixed-nozzle slice in place: mode
// Off, `fine_height` everywhere, every material slot on the fine material's nozzle, every feature
// and body on the fine material, Body layer heights and colour painting cleared.
// `fine_logical_filament` is zero based.
void mixed_nozzle_single_nozzle_baseline(Model &model, DynamicPrintConfig &config,
                                         std::size_t fine_logical_filament, double fine_height);

// Plate id, a hash of what the model prints and a hash of the config, as one string.
std::string mixed_nozzle_ranking_key(std::size_t plate_id, const Model &model,
                                     const DynamicPrintConfig &config);

} // namespace Slic3r

#endif // slic3r_MixedNozzleRanking_hpp_
