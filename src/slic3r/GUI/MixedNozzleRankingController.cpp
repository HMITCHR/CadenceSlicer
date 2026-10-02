#include "MixedNozzleRankingController.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>

namespace Slic3r::GUI {

namespace {

WizardEstimate estimate_with(WizardEstimateStatus status, const std::string &note)
{
    WizardEstimate estimate;
    estimate.status = status;
    estimate.note = note;
    return estimate;
}

} // namespace

std::vector<std::string> wizard_ranking_order(const WizardCadencePage &page,
                                              const std::vector<WizardCandidate> &candidates,
                                              std::optional<std::size_t> current_row)
{
    std::vector<std::size_t> rows;
    for (std::size_t index = 0; index < page.rows.size(); ++index)
        if (page.rows[index].candidate_index < candidates.size())
            rows.push_back(index);
    if (rows.empty())
        return {};
    std::stable_sort(rows.begin(), rows.end(), [&page, &current_row](std::size_t lhs, std::size_t rhs) {
        const bool lhs_current = current_row && lhs == *current_row;
        const bool rhs_current = current_row && rhs == *current_row;
        if (lhs_current != rhs_current)
            return lhs_current;
        return page.rows[lhs].coarse_height > page.rows[rhs].coarse_height;
    });
    std::vector<std::string> order;
    for (const std::size_t index : rows)
        order.push_back(candidates[page.rows[index].candidate_index].stable_id);
    order.insert(order.begin() + std::ptrdiff_t(std::min<std::size_t>(2, order.size())),
                 WIZARD_RANKING_BASELINE_ROW);
    return order;
}

MixedNozzleRankingController::MixedNozzleRankingController(WizardRankingHooks hooks,
                                                           std::shared_ptr<WizardEstimateCache> cache)
    : m_hooks(std::move(hooks)), m_cache(std::move(cache))
{
}

MixedNozzleRankingController::~MixedNozzleRankingController()
{
    shutdown();
}

void MixedNozzleRankingController::request(std::vector<std::string> row_ids)
{
    // Whatever was running belongs to input that no longer exists.
    if (m_running_cancel)
        m_running_cancel->cancel();
    m_running_cancel.reset();
    m_running_row.reset();
    m_running_key.clear();
    ++m_job;
    m_queue.assign(row_ids.begin(), row_ids.end());
    m_estimates.clear();
    for (const std::string &row : row_ids)
        m_estimates[row] = WizardEstimate{};
    m_requested_at = m_hooks.now ? m_hooks.now() : 0.;
    m_started = false;
    notify();
}

void MixedNozzleRankingController::tick()
{
    const double now = m_hooks.now ? m_hooks.now() : 0.;
    const bool real_slice = m_hooks.real_slice_running && m_hooks.real_slice_running();
    if (m_started) {
        if (real_slice) {
            // Never compete with the plater's own slice. The running row goes back to the front of
            // the queue under a new job id, so its late result is dropped.
            if (m_running_cancel)
                m_running_cancel->cancel();
            if (m_running_row)
                m_queue.push_front(*m_running_row);
            m_running_cancel.reset();
            m_running_row.reset();
            m_running_key.clear();
            ++m_job;
            m_started = false;
            notify();
        }
        return;
    }
    if (m_queue.empty() || real_slice || now - m_requested_at < kWizardRankingDebounceSeconds)
        return;
    m_started = true;
    start_next();
}

void MixedNozzleRankingController::deliver(std::uint64_t job, const std::string &row_id,
                                           const WizardEstimate &estimate)
{
    if (job != m_job || !m_running_row || *m_running_row != row_id)
        return;
    m_estimates[row_id] = estimate;
    if (estimate.status == WizardEstimateStatus::Estimated && m_cache && !m_running_key.empty())
        (*m_cache)[m_running_key] = estimate;
    m_running_cancel.reset();
    m_running_row.reset();
    m_running_key.clear();
    start_next();
}

void MixedNozzleRankingController::cancel()
{
    shutdown();
    notify();
}

void MixedNozzleRankingController::shutdown()
{
    finish_job_with(WizardEstimateStatus::Unavailable, "not estimated");
}

std::optional<WizardEstimate> MixedNozzleRankingController::estimate(const std::string &row_id) const
{
    const auto found = m_estimates.find(row_id);
    if (found == m_estimates.end())
        return std::nullopt;
    return found->second;
}

void MixedNozzleRankingController::start_next()
{
    while (!m_queue.empty()) {
        const std::string row = m_queue.front();
        m_queue.pop_front();
        MixedNozzleRankingSlice slice = m_hooks.compose ? m_hooks.compose(row) : MixedNozzleRankingSlice{};
        if (!slice.diagnostic.empty() || !slice.model) {
            m_estimates[row] = estimate_with(WizardEstimateStatus::Unavailable,
                slice.diagnostic.empty() ? std::string("nothing to slice") : slice.diagnostic);
            continue;
        }
        if (m_cache && !slice.key.empty()) {
            const auto cached = m_cache->find(slice.key);
            if (cached != m_cache->end()) {
                m_estimates[row] = cached->second;
                continue;
            }
        }
        if (!m_hooks.run) {
            m_estimates[row] = estimate_with(WizardEstimateStatus::Unavailable, "nothing to slice with");
            continue;
        }
        m_running_row = row;
        m_running_key = slice.key;
        m_running_cancel = std::make_shared<MixedNozzleRankingCancel>();
        const std::uint64_t job = m_job;
        std::shared_ptr<MixedNozzleRankingCancel> cancel = m_running_cancel;
        notify();
        // The runner may report back before it returns; nothing here runs after it.
        m_hooks.run(job, row, std::move(slice), std::move(cancel));
        return;
    }
    // Every row has its answer.
    m_started = false;
    notify();
}

void MixedNozzleRankingController::finish_job_with(WizardEstimateStatus status, const std::string &note)
{
    if (m_running_cancel)
        m_running_cancel->cancel();
    if (m_running_row)
        m_estimates[*m_running_row] = estimate_with(status, note);
    for (const std::string &row : m_queue)
        m_estimates[row] = estimate_with(status, note);
    m_queue.clear();
    m_running_cancel.reset();
    m_running_row.reset();
    m_running_key.clear();
    ++m_job;
    m_started = false;
}

void MixedNozzleRankingController::notify()
{
    if (m_hooks.changed)
        m_hooks.changed();
}

} // namespace Slic3r::GUI
