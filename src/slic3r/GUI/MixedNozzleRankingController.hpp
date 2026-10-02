#ifndef slic3r_GUI_MixedNozzleRankingController_hpp_
#define slic3r_GUI_MixedNozzleRankingController_hpp_

#include "libslic3r/MixedNozzleRanking.hpp"
#include "slic3r/GUI/MixedNozzleWizardModel.hpp"

#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r::GUI {

// The cadence ranking job behind the wizard, with no wx in it. One job at a time; its rows are
// sliced one after another, and every result is checked against the job that asked for it. There is
// no time budget: a job ends when every row has its answer or on cancel.
//
// A new request waits this long before its job starts, so a burst of input changes starts one job.
inline constexpr double kWizardRankingDebounceSeconds = 0.25;
// The row id of the all-fine single-nozzle baseline.
inline constexpr const char *WIZARD_RANKING_BASELINE_ROW = "single-nozzle-baseline";

// The order a job slices rows in: the project's current coarse row first, then the rest from the
// thickest coarse layer to the thinnest (usually fastest, fewest nozzle changes), with the
// one-nozzle baseline after the first two so the saving appears early. Empty when the page has no
// row.
std::vector<std::string> wizard_ranking_order(const WizardCadencePage &page,
                                              const std::vector<WizardCandidate> &candidates,
                                              std::optional<std::size_t> current_row);

// Finished estimates by slice key, so an unchanged row is not sliced again, including after the
// wizard is reopened in the same app run.
using WizardEstimateCache = std::map<std::string, WizardEstimate>;

struct WizardRankingHooks {
    // Main thread. The slice input for one row; a non-empty diagnostic means nothing to slice.
    std::function<MixedNozzleRankingSlice(const std::string &row_id)> compose;
    // Starts the slice and returns. The runner reports back on the main thread through
    // MixedNozzleRankingController::deliver() with the same job and row.
    std::function<void(std::uint64_t job, const std::string &row_id, MixedNozzleRankingSlice slice,
                       std::shared_ptr<MixedNozzleRankingCancel> cancel)> run;
    // True while the plater's own background slice runs. Ranking never competes with it.
    std::function<bool()> real_slice_running;
    // Seconds on any steady clock.
    std::function<double()> now;
    // Estimates changed; the page should be rebuilt.
    std::function<void()> changed;
};

class MixedNozzleRankingController
{
public:
    explicit MixedNozzleRankingController(WizardRankingHooks hooks,
                                          std::shared_ptr<WizardEstimateCache> cache = {});
    ~MixedNozzleRankingController();
    MixedNozzleRankingController(const MixedNozzleRankingController &) = delete;
    MixedNozzleRankingController &operator=(const MixedNozzleRankingController &) = delete;

    // The rows changed. Cancels the running job and queues these rows, all "estimating"; the new
    // job starts from tick() once the debounce has passed.
    void request(std::vector<std::string> row_ids);
    // Drives the job: starts it after the debounce and pauses it while a real slice runs.
    void tick();
    // A finished slice. Dropped unless it is the running row of the current job.
    void deliver(std::uint64_t job, const std::string &row_id, const WizardEstimate &estimate);
    // The input is gone (a new fine height, the dialog closing). Cancels the running slice; rows
    // not estimated say so.
    void cancel();
    // cancel() without telling the page, for a dialog that is being destroyed.
    void shutdown();

    std::uint64_t job() const { return m_job; }
    bool running() const { return m_running_row.has_value(); }
    // The row whose slice is running now.
    const std::optional<std::string> &running_row() const { return m_running_row; }
    std::optional<WizardEstimate> estimate(const std::string &row_id) const;

private:
    void start_next();
    void finish_job_with(WizardEstimateStatus status, const std::string &note);
    void notify();

    WizardRankingHooks m_hooks;
    std::shared_ptr<WizardEstimateCache> m_cache;
    std::uint64_t m_job {0};
    std::deque<std::string> m_queue;
    std::map<std::string, WizardEstimate> m_estimates;
    std::optional<std::string> m_running_row;
    std::string m_running_key;
    std::shared_ptr<MixedNozzleRankingCancel> m_running_cancel;
    double m_requested_at {0.};
    bool m_started {false};
};

} // namespace Slic3r::GUI

#endif
