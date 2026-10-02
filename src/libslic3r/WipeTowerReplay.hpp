#pragma once

#include "BoundingBox.hpp"
#include "GCode/WipeTower.hpp"

#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace Slic3r {

class PrintConfig;

namespace MultiNozzleUtils {
class LayeredNozzleGroupResult;
}

// A Type1 candidate's immutable schedule input. The replay seam consumes these native tower
// events instead of deriving a second schedule from logical switch counts.
struct WipeTowerPlanEvent {
    float z;
    float layer_height;
    unsigned int old_tool;
    unsigned int new_tool;
    float prime_extruder_change;
    float prime_nozzle_change;
    float purge_volume;
    bool force_emit;
    bool disable_last_layer_fill;
    size_t outgoing_filament;
    size_t incoming_filament;
    int outgoing_extruder;
    int incoming_extruder;
    bool automatic_ram;
    bool automatic_prime;
    float ram_override_mm;
    // Seconds the incoming nozzle sat parked before this return (planner estimate); negative when
    // unknown. Automatic primes are raised for it via mixed_nozzle_idle_reprime_volume.
    float idle_seconds { -1.f };
};

struct WipeTowerReplayTiming {
    // WipeTowerWriter's native nominal motion accounting. This remains available when the
    // caller does not request a parser replay.
    double writer_motion_seconds { 0. };
    // GCodeProcessor's native replay of the supplied tower command stream, when requested.
    double native_parser_seconds { 0. };
    bool native_parser_replayed { false };
    bool writer_motion_complete { true };
    // Unknown firmware/custom commands are not silently treated as timed. Callers must inspect
    // this bit before using measured_seconds for an economic comparison.
    bool interval_uncertain { false };
    // This adapter receives tower snippets only; native T/filament transition time is supplied by
    // the stateful transition adapter and is never folded into this value.
    static constexpr bool includes_native_transition_overhead = false;

    double measured_seconds() const {
        return native_parser_replayed ? native_parser_seconds : writer_motion_seconds;
    }
};

struct WipeTowerReplayContext {
    WipeTower &tower;
    const PrintConfig &config;
    const std::vector<int> &filament_categories;
    const std::set<int> &used_filament_ids;
    float requested_width;
    std::shared_ptr<MultiNozzleUtils::LayeredNozzleGroupResult> nozzle_group_result;
    // Structural eligibility stays owned by the native support-domain audit. The replay adapter
    // invokes it after generation and returns its result with the candidate value.
    std::function<bool(const std::vector<std::vector<WipeTower::ToolChangeResult>> &)> audit_structure;
    // The native maximum width is part of the effective AutoPad context. Infinity leaves the
    // generic replay seam uncapped for callers that are evaluating a pre-cap schedule.
    double width_cap { std::numeric_limits<double>::infinity() };
    // Parser timing is opt-in. AutoPad keeps the cheap native writer provenance path; a schedule
    // cost caller must explicitly provide the complete native command stream, including the
    // active tool, start position and E-mode transitions surrounding each event.
    std::string native_command_stream;
    bool request_native_timing { false };
    bool complete_native_command_context { false };
    // Share of each automatic return's idle re-prime (above its floor) that this replay lays.
    // Auto pad lowers it only when the full re-prime does not fit inside the width cap.
    float reprime_share { 1.f };
};

struct WipeTowerReplayResult {
    std::vector<std::vector<WipeTower::ToolChangeResult>> tool_changes;
    std::vector<float> automatic_ram_lengths;
    std::vector<float> automatic_prime_volumes;
    WipeTowerReplayTiming timing;
    BoundingBoxf bbx;
    double support_height { 0. };
    double support_depth { std::numeric_limits<double>::infinity() };
    double footprint { std::numeric_limits<double>::infinity() };
    double depth_residual { std::numeric_limits<double>::infinity() };
    double bbox_residual { std::numeric_limits<double>::infinity() };
    size_t expected_switches { 0 };
    size_t structural_emission_count { 0 };
    bool replay_ok { false };
    // Geometry generation alone cannot certify unsupported gaps between supplied events.
    bool structure_qualified { false };
};

// Replays the supplied native Type1 tower events and returns the generated support geometry plus
// native timing provenance. The implementation lives in Print.cpp so it uses the same factory and
// effective PrintConfig context as production tower generation.
WipeTowerReplayResult replay_wipe_tower_plan(const std::vector<WipeTowerPlanEvent> &plan_events,
                                             WipeTowerReplayContext &context);

// AutoPad's width search, one seed at a time. The rib planner squares the tower against the
// depth its blocks need, so the target is a width the planner hands back unchanged: each walk
// keeps asking for the width it was last given. The caller builds and reports each candidate.
struct AutoPadWidthBuild {
    bool  replay_ok { false };
    // The width the planner came back with. Also filled for a failed replay whenever a tower was
    // planned, because a planned width over the cap is what tells a cap failure from any other.
    float generated_width { 0.f };
};
using AutoPadWidthBuildFn = std::function<AutoPadWidthBuild(float requested_width)>;

enum class AutoPadWalkEnd {
    Settled,      // the last build came back at the width it asked for; width says which
    Cycle,        // the walk came back to a width it had already built; cycle lists them
    Merged,       // the walk reached a width an earlier walk had built
    Unsettled,    // out of iterations; width is the one it would have asked for next
    ReplayFailed,
    OutOfRange    // the planner came back below the minimum or above the cap
};

struct AutoPadWidthWalk {
    AutoPadWalkEnd     end { AutoPadWalkEnd::ReplayFailed };
    float              width { 0.f };
    std::vector<float> path;    // every width built, in order
    std::vector<float> cycle;   // Cycle only
    // The narrowest width the planner asked for above the cap, or 0 when it never did.
    float              above_cap_width { 0.f };
};

// One seed's walk. replayed_widths is shared by every walk of one search, keyed to the micron, so
// no width is replayed twice. The callback is called once per build.
AutoPadWidthWalk walk_auto_pad_width(float seed, float minimum_width, float width_cap,
                                     size_t max_iterations, std::set<long> &replayed_widths,
                                     const AutoPadWidthBuildFn &build);

// The widths AutoPad falls back to when no walk settled, ascending, each once. The caller builds
// each at exactly that width, without squaring it again, and keeps every other check.
std::vector<float> auto_pad_fallback_widths(const std::vector<AutoPadWidthWalk> &walks);

} // namespace Slic3r
