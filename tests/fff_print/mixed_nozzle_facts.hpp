#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "libslic3r/GCode/GCodeProcessor.hpp"

namespace Slic3r::Emitted {

// One physical-tool/role/height/width bucket observed on one layer.
struct EmittedRoleFact
{
    unsigned int layer_id{ 0 };
    ExtrusionRole role{ erNone };
    unsigned char logical_filament{ 0 };
    unsigned char physical_tool{ 0 };
    float height{ 0.0f };
    float width{ 0.0f };
    float mm3_per_mm{ 0.0f };
    size_t move_count{ 0 };
    // Print Z of the bucket's first move, and the XY box every move of the bucket ends inside.
    float z{ 0.0f };
    float min_x{ 0.0f }, min_y{ 0.0f }, max_x{ 0.0f }, max_y{ 0.0f };
};

// One physical-tool change observed in the move stream, in emission order.
struct EmittedToolChange
{
    unsigned char from_physical_tool{ 0 };
    unsigned char to_physical_tool{ 0 };
    float z{ 0.0f };
};

// A parsed "; SRL_SUPPORT ..." record, emitted at most once per plan block. Tool indices are
// 0-based physical extruder ids; heights are coordf_t values printed with eight decimal places.
struct EmittedPlanSupport
{
    unsigned int base_tool{ 0 };
    double       base_height{ 0. };
    unsigned int interface_tool{ 0 };
    double       interface_height{ 0. };
};

// A parsed "; SRL_GRID ..." record from the emitted plan block.
struct EmittedPlanGrid
{
    size_t       region{ 0 };
    unsigned int tool{ 0 };
    double       nozzle{ 0. };
    double       cadence{ 0. };
    size_t       cells{ 0 };
    double       top_z{ 0. };
    std::string  phase;
};

// What GCodeProcessor plus the SRL_PLAN block say about one emitted G-code stream.
struct EmittedFacts
{
    std::vector<EmittedRoleFact>   role_facts;
    std::vector<EmittedToolChange> tool_changes;
    std::vector<EmittedPlanGrid>   plan_grids;
    std::optional<EmittedPlanSupport> support;
    // The "; SRL_TOOLCHANGES total=<n> per_band=<n>" record, when the plan carries one.
    std::optional<int>             plan_toolchanges_total;
    std::optional<int>             plan_toolchanges_per_band;
    size_t                         plan_start_markers{ 0 };
    size_t                         plan_end_markers{ 0 };
    bool                           plan_malformed{ false };
};

// Runs the string through GCodeProcessor::process_file (via a ScopedTemporaryFile) and
// extracts EmittedFacts from GCodeProcessorResult::MoveVertex plus the "; SRL_GRID" records
// in an "; SRL_PLAN_START" / "; SRL_PLAN_END" block, if present. Throws std::runtime_error
// if the gcode string cannot be written to a temporary file.
EmittedFacts extract_emitted_facts(const std::string& gcode);

// Every move GCodeProcessor reads from the string, in stream order: position, role, width,
// height and the physical tool that made it. For checks that need to know where a road is.
std::vector<GCodeProcessorResult::MoveVertex> emitted_moves(const std::string& gcode);

} // namespace Slic3r::Emitted
