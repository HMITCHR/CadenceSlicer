#include "mixed_nozzle_facts.hpp"

#include "test_utils.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace Slic3r::Emitted {

namespace {

// Parse space-delimited key=value fields, flagging empty fields and duplicate keys.
std::map<std::string, std::string> plan_fields(const std::string &rest, bool &malformed)
{
    std::map<std::string, std::string> fields;
    std::istringstream words(rest);
    std::string word;
    while (words >> word) {
        const size_t split = word.find('=');
        if (split == std::string::npos || split == 0 || split + 1 == word.size())
            malformed = true;
        else if (! fields.emplace(word.substr(0, split), word.substr(split + 1)).second)
            malformed = true;
    }
    return fields;
}

// A record must contain every required field and may contain only the allowed optional fields.
bool plan_fields_match(const std::map<std::string, std::string> &fields,
                       const std::set<std::string> &required, const std::set<std::string> &optional)
{
    for (const std::string &key : required)
        if (fields.find(key) == fields.end())
            return false;
    for (const auto &field : fields)
        if (required.count(field.first) == 0 && optional.count(field.first) == 0)
            return false;
    return true;
}

// Validate the comma-separated region ids on a "; SRL_RENDEZVOUS" record. The ids are used only
// for syntax validation; EmittedFacts records the malformed side effect, not the parsed list.
void validate_region_list(const std::string &value, bool &malformed)
{
    std::istringstream stream(value);
    std::string item;
    size_t count = 0;
    while (std::getline(stream, item, ',')) {
        try {
            std::stoul(item);
            ++count;
        } catch (...) {
            malformed = true;
        }
    }
    if (count == 0)
        malformed = true;
}

// Parse SRL plan records directly from the G-code stream, independently of the processed moves.
// Validate every supported record type (GRID, CELL, RENDEZVOUS, TOLERANCE, TOOLCHANGES, SUPPORT,
// BAND, FEATURE), but store only GRID and SUPPORT in EmittedFacts. This keeps other well-formed
// records from being flagged malformed merely because they have no corresponding stored type.
void parse_srl_plan(const std::string &gcode, EmittedFacts &facts)
{
    bool inside = false;
    std::istringstream stream(gcode);
    std::string line;
    while (std::getline(stream, line)) {
        while (! line.empty() && (line.back() == '\r' || line.back() == ' '))
            line.pop_back();

        if (line == "; SRL_PLAN_START") {
            ++facts.plan_start_markers;
            inside = true;
            continue;
        }
        if (line == "; SRL_PLAN_END") {
            ++facts.plan_end_markers;
            inside = false;
            continue;
        }
        if (! inside)
            continue;

        const std::string grid_prefix = "; SRL_GRID ", cell_prefix = "; SRL_CELL ",
                          rendezvous_prefix = "; SRL_RENDEZVOUS ", tolerance_prefix = "; SRL_TOLERANCE ",
                          toolchanges_prefix = "; SRL_TOOLCHANGES ", support_prefix = "; SRL_SUPPORT ",
                          band_prefix = "; SRL_BAND ",
                          feature_prefix = "; SRL_FEATURE ";
        try {
            if (line.rfind(grid_prefix, 0) == 0) {
                std::map<std::string, std::string> fields = plan_fields(line.substr(grid_prefix.size()), facts.plan_malformed);
                if (! plan_fields_match(fields, {"region", "tool", "nozzle", "cadence", "cells", "top_z"}, {"phase"})) {
                    facts.plan_malformed = true;
                    continue;
                }
                facts.plan_grids.push_back({size_t(std::stoul(fields["region"])), unsigned(std::stoul(fields["tool"])),
                                            std::stod(fields["nozzle"]), std::stod(fields["cadence"]),
                                            size_t(std::stoul(fields["cells"])), std::stod(fields["top_z"]),
                                            fields.count("phase") ? fields["phase"] : std::string()});
            } else if (line.rfind(cell_prefix, 0) == 0) {
                std::map<std::string, std::string> fields = plan_fields(line.substr(cell_prefix.size()), facts.plan_malformed);
                if (! plan_fields_match(fields, {"region", "tool", "z_lo", "z_hi", "height"}, {}))
                    facts.plan_malformed = true;
            } else if (line.rfind(rendezvous_prefix, 0) == 0) {
                std::map<std::string, std::string> fields = plan_fields(line.substr(rendezvous_prefix.size()), facts.plan_malformed);
                if (! plan_fields_match(fields, {"z", "reason", "regions"}, {"contact_w"}))
                    facts.plan_malformed = true;
                else
                    validate_region_list(fields["regions"], facts.plan_malformed);
            } else if (line.rfind(tolerance_prefix, 0) == 0) {
                std::map<std::string, std::string> fields = plan_fields(line.substr(tolerance_prefix.size()), facts.plan_malformed);
                if (! plan_fields_match(fields, {"value"}, {}))
                    facts.plan_malformed = true;
            } else if (line.rfind(toolchanges_prefix, 0) == 0) {
                // The Body plan block carries total and per-band tool-change counts.
                std::map<std::string, std::string> fields = plan_fields(line.substr(toolchanges_prefix.size()), facts.plan_malformed);
                if (! plan_fields_match(fields, {"total", "per_band"}, {})) {
                    facts.plan_malformed = true;
                    continue;
                }
                facts.plan_toolchanges_total    = std::stoi(fields["total"]);
                facts.plan_toolchanges_per_band = std::stoi(fields["per_band"]);
            } else if (line.rfind(support_prefix, 0) == 0) {
                // A Body plan block carries at most one support record, emitted only when the
                // project has support.
                std::map<std::string, std::string> fields = plan_fields(line.substr(support_prefix.size()), facts.plan_malformed);
                if (! plan_fields_match(fields, {"base_tool", "base_height", "interface_tool", "interface_height"}, {})) {
                    facts.plan_malformed = true;
                    continue;
                }
                facts.support = EmittedPlanSupport{unsigned(std::stoul(fields["base_tool"])), std::stod(fields["base_height"]),
                                                   unsigned(std::stoul(fields["interface_tool"])), std::stod(fields["interface_height"])};
            } else if (line.rfind(band_prefix, 0) == 0) {
                // The Feature plan block carries a band record with optional vertical bounds.
                std::map<std::string, std::string> fields = plan_fields(line.substr(band_prefix.size()), facts.plan_malformed);
                if (! plan_fields_match(fields, {"mode", "region", "tool", "nozzle", "cadence"}, {"z_lo", "z_hi", "height"}))
                    facts.plan_malformed = true;
            } else if (line.rfind(feature_prefix, 0) == 0) {
                // The Feature plan header is the first record after SRL_PLAN_START; bands=0 is a
                // valid empty plan.
                std::map<std::string, std::string> fields = plan_fields(line.substr(feature_prefix.size()), facts.plan_malformed);
                if (! plan_fields_match(fields, {"bands"}, {}))
                    facts.plan_malformed = true;
            } else {
                facts.plan_malformed = true;
            }
        } catch (...) {
            facts.plan_malformed = true;
        }
    }
}

} // namespace

std::vector<GCodeProcessorResult::MoveVertex> emitted_moves(const std::string &gcode)
{
    ScopedTemporaryFile gcode_file(".gcode");
    {
        std::ofstream out(gcode_file.string());
        if (! out)
            throw std::runtime_error("emitted_moves: could not write gcode to " + gcode_file.string());
        out << gcode;
    }

    GCodeProcessor processor;
    processor.process_file(gcode_file.string());
    auto moves = processor.get_result().moves;
    // Speed-preview clones do not represent additional deposited roads.
    moves.erase(std::remove_if(moves.begin(), moves.end(),
        [](const auto &move) { return move.internal_only; }), moves.end());
    return moves;
}

EmittedFacts extract_emitted_facts(const std::string &gcode)
{
    EmittedFacts facts;

    const std::vector<GCodeProcessorResult::MoveVertex> moves = emitted_moves(gcode);

    // Bucket extrude moves by (layer_id, role, physical_tool_id). First-seen height/width/
    // mm3_per_mm are kept -- not averaged -- for determinism: every RED fixture's bucket is
    // uniform by construction (Step 2), so first-seen and average coincide there.
    std::map<std::tuple<unsigned int, ExtrusionRole, unsigned char>, size_t> bucket_index;

    bool have_prev_extrude = false;
    unsigned char prev_physical_tool = 0;

    for (const GCodeProcessorResult::MoveVertex &move : moves) {
        if (move.type != EMoveType::Extrude)
            continue;

        const auto key = std::make_tuple(move.layer_id, move.extrusion_role, move.physical_tool_id);
        const auto found = bucket_index.find(key);
        if (found == bucket_index.end()) {
            EmittedRoleFact fact;
            fact.layer_id = move.layer_id;
            fact.role = move.extrusion_role;
            fact.logical_filament = move.extruder_id;
            fact.physical_tool = move.physical_tool_id;
            fact.height = move.height;
            fact.width = move.width;
            fact.mm3_per_mm = move.mm3_per_mm;
            fact.move_count = 1;
            fact.z = move.position.z();
            fact.min_x = fact.max_x = move.position.x();
            fact.min_y = fact.max_y = move.position.y();
            facts.role_facts.push_back(fact);
            bucket_index.emplace(key, facts.role_facts.size() - 1);
        } else {
            EmittedRoleFact &fact = facts.role_facts[found->second];
            ++fact.move_count;
            fact.min_x = std::min(fact.min_x, move.position.x());
            fact.max_x = std::max(fact.max_x, move.position.x());
            fact.min_y = std::min(fact.min_y, move.position.y());
            fact.max_y = std::max(fact.max_y, move.position.y());
        }

        if (have_prev_extrude && move.physical_tool_id != prev_physical_tool)
            facts.tool_changes.push_back({prev_physical_tool, move.physical_tool_id, move.position.z()});
        prev_physical_tool = move.physical_tool_id;
        have_prev_extrude = true;
    }

    parse_srl_plan(gcode, facts);

    return facts;
}

} // namespace Slic3r::Emitted
