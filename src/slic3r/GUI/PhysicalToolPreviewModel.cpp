#include "PhysicalToolPreviewModel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <set>
#include <utility>

namespace Slic3r::GUI {
namespace {
constexpr std::array<libvgcode::Color, 8> FOUNDATION_COLORS {{
    {0x00, 0x72, 0xB2},
    {0xE6, 0x9F, 0x00},
    {0x00, 0x9E, 0x73},
    {0xCC, 0x79, 0xA7},
    {0x56, 0xB4, 0xE9},
    {0xD5, 0x5E, 0x00},
    {0xF0, 0xE4, 0x42},
    {0x00, 0x00, 0x00}
}};

libvgcode::Color generated_color(std::size_t index)
{
    uint32_t value = static_cast<uint32_t>(index) * 0x9E3779B9u + 0x85EBCA6Bu;
    value ^= value >> 16;
    value *= 0x7FEB352Du;
    value ^= value >> 15;
    return {
        static_cast<uint8_t>(48u + (value & 0x9Fu)),
        static_cast<uint8_t>(48u + ((value >> 8) & 0x9Fu)),
        static_cast<uint8_t>(48u + ((value >> 16) & 0x9Fu))
    };
}
} // namespace

std::vector<std::string> physical_tool_snapshot_filament_names(
    const std::vector<std::string>& result_names, const std::vector<std::string>& sliced_names,
    bool allow_sliced_fallback)
{
    auto names = result_names;
    if (allow_sliced_fallback) {
        names.resize(std::max(names.size(), sliced_names.size()));
        for (std::size_t logical = 0; logical < sliced_names.size(); ++logical)
            if (names[logical].empty())
                names[logical] = sliced_names[logical];
    }
    return names;
}

PhysicalToolPreviewSnapshot::PhysicalToolPreviewSnapshot(PhysicalToolPreviewSnapshotInput input)
    : m_result_identity(input.result_identity)
    , m_mode(input.mode)
    , m_filament_map_mode(std::move(input.filament_map_mode))
    , m_filament_map(std::move(input.filament_map))
    , m_nozzle_diameters_mm(std::move(input.nozzle_diameters_mm))
    , m_filament_materials(std::move(input.filament_materials))
    , m_filament_preset_ids(std::move(input.filament_preset_ids))
    , m_configured_bindings(std::move(input.configured_bindings))
    , m_logical_usage(std::move(input.logical_usage))
    , m_parsed_vertex_facts(std::move(input.parsed_vertex_facts))
    , m_plan_published(input.plan_published)
{
    std::set<std::size_t> logical_bindings;
    std::vector<PhysicalToolBindingFact> unique_bindings;
    unique_bindings.reserve(m_configured_bindings.size());
    for (PhysicalToolBindingFact& binding : m_configured_bindings) {
        if (!logical_bindings.insert(binding.logical_filament).second) {
            m_diagnostics.push_back({"duplicate_configured_binding",
                                     "Duplicate configured binding; first logical fact retained",
                                     binding.logical_filament});
        } else {
            unique_bindings.push_back(std::move(binding));
        }
    }
    m_configured_bindings = std::move(unique_bindings);

    std::set<std::size_t> logical_usage;
    std::vector<PhysicalToolLogicalUsage> unique_usage;
    unique_usage.reserve(m_logical_usage.size());
    for (const PhysicalToolLogicalUsage& usage : m_logical_usage) {
        if (!logical_usage.insert(usage.logical_filament).second) {
            m_diagnostics.push_back({"duplicate_logical_usage",
                                     "Duplicate logical usage; first metres/grams fact retained",
                                     usage.logical_filament});
        } else {
            unique_usage.push_back(usage);
        }
    }
    m_logical_usage = std::move(unique_usage);

    for (const PhysicalToolVertexFact& fact : m_parsed_vertex_facts) {
        if (fact.physical_tool != libvgcode::UNKNOWN_PHYSICAL_TOOL_ID)
            m_parsed_used_physical_tools.push_back(fact.physical_tool);
    }
    std::sort(m_parsed_used_physical_tools.begin(), m_parsed_used_physical_tools.end());
    m_parsed_used_physical_tools.erase(
        std::unique(m_parsed_used_physical_tools.begin(), m_parsed_used_physical_tools.end()),
        m_parsed_used_physical_tools.end());
}

PhysicalToolPreviewSnapshot make_physical_tool_preview_snapshot(PhysicalToolPreviewSnapshotInput input)
{
    return PhysicalToolPreviewSnapshot(std::move(input));
}

std::vector<PhysicalToolLegendRow>
build_physical_tool_legend_rows(const PhysicalToolPreviewSnapshot& snapshot)
{
    std::set<std::size_t> configured_logical_filaments;
    for (std::size_t logical = 0; logical < snapshot.filament_map().size(); ++logical)
        configured_logical_filaments.insert(logical);
    for (const PhysicalToolBindingFact& binding : snapshot.configured_bindings())
        configured_logical_filaments.insert(binding.logical_filament);

    std::vector<PhysicalToolLegendRow> rows;
    rows.reserve(configured_logical_filaments.size());
    for (std::size_t logical : configured_logical_filaments) {
        PhysicalToolLegendRow row;
        row.logical_filament = logical;
        row.filament_preset = logical < snapshot.filament_preset_ids().size() &&
                                      !snapshot.filament_preset_ids()[logical].empty()
                                  ? snapshot.filament_preset_ids()[logical]
                                  : "preset unknown";
        row.material = logical < snapshot.filament_materials().size() &&
                               !snapshot.filament_materials()[logical].empty()
                           ? snapshot.filament_materials()[logical]
                           : "material unknown";

        std::set<uint8_t> parsed_for_logical;
        for (const PhysicalToolVertexFact& fact : snapshot.parsed_vertex_facts()) {
            if (fact.logical_filament == logical &&
                fact.physical_tool != libvgcode::UNKNOWN_PHYSICAL_TOOL_ID)
                parsed_for_logical.insert(fact.physical_tool);
        }
        const bool duplicate_binding = std::any_of(
            snapshot.diagnostics().begin(), snapshot.diagnostics().end(),
            [logical](const PhysicalToolSnapshotDiagnostic& diagnostic) {
                return diagnostic.logical_filament == logical &&
                       diagnostic.code == "duplicate_configured_binding";
            });

        const auto binding = std::find_if(snapshot.configured_bindings().begin(),
                                          snapshot.configured_bindings().end(),
                                          [logical](const PhysicalToolBindingFact& fact) {
                                              return fact.logical_filament == logical;
                                          });
        const bool has_configured_tool = binding != snapshot.configured_bindings().end() &&
                                         binding->configured_physical_tool.has_value() &&
                                         *binding->configured_physical_tool != libvgcode::UNKNOWN_PHYSICAL_TOOL_ID;
        if (duplicate_binding) {
            row.status = PhysicalToolBindingStatus::Unresolved;
            row.diagnostic_code = "duplicate_configured_binding";
            row.diagnostic = "Duplicate configured binding - Preview qualification blocked";
        } else if (has_configured_tool) {
            row.physical_tool = binding->configured_physical_tool;
            if (!parsed_for_logical.empty() &&
                (parsed_for_logical.size() != 1 || *parsed_for_logical.begin() != *row.physical_tool)) {
                row.status = PhysicalToolBindingStatus::Mismatch;
                row.diagnostic_code = "parsed_configured_physical_tool_mismatch";
                row.diagnostic = "Parsed/configured physical tool mismatch - Preview qualification blocked";
            } else {
                row.status = PhysicalToolBindingStatus::Resolved;
                row.physical_qualified = true;
            }
        } else if (parsed_for_logical.size() == 1) {
            row.physical_tool = *parsed_for_logical.begin();
            row.status = PhysicalToolBindingStatus::ParsedOnly;
            row.diagnostic_code = "configured_binding_missing";
            row.diagnostic = "Parsed physical tool has no configured binding - Preview qualification blocked";
        } else {
            row.status = PhysicalToolBindingStatus::Unresolved;
            row.diagnostic_code = binding == snapshot.configured_bindings().end()
                                      ? "configured_binding_missing"
                                      : binding->resolver_code;
            row.diagnostic = binding == snapshot.configured_bindings().end()
                                 ? "Configured binding unknown"
                                 : binding->resolver_diagnostic;
        }
        if (row.physical_tool.has_value() && *row.physical_tool < snapshot.nozzle_diameters_mm().size())
            row.nozzle_diameter_mm = snapshot.nozzle_diameters_mm()[*row.physical_tool];

        const auto usage = std::find_if(snapshot.logical_usage().begin(), snapshot.logical_usage().end(),
                                        [logical](const PhysicalToolLogicalUsage& fact) {
                                            return fact.logical_filament == logical;
                                        });
        if (usage != snapshot.logical_usage().end()) {
            row.model_usage_m = usage->model_usage_m;
            row.model_usage_g = usage->model_usage_g;
        }
        rows.push_back(std::move(row));
    }

    std::sort(rows.begin(), rows.end(), [](const PhysicalToolLegendRow& left,
                                           const PhysicalToolLegendRow& right) {
        const bool left_resolved = left.status == PhysicalToolBindingStatus::Resolved;
        const bool right_resolved = right.status == PhysicalToolBindingStatus::Resolved;
        if (left_resolved != right_resolved)
            return left_resolved;
        if (left_resolved && left.physical_tool != right.physical_tool)
            return left.physical_tool < right.physical_tool;
        return left.logical_filament < right.logical_filament;
    });
    return rows;
}

PhysicalToolPalette make_physical_tool_palette(std::size_t tool_count)
{
    PhysicalToolPalette palette;
    tool_count = std::min(tool_count, static_cast<std::size_t>(libvgcode::UNKNOWN_PHYSICAL_TOOL_ID));
    palette.colors.reserve(tool_count);
    for (std::size_t index = 0; index < tool_count; ++index)
        palette.colors.push_back(index < FOUNDATION_COLORS.size() ? FOUNDATION_COLORS[index] : generated_color(index));
    return palette;
}

libvgcode::Color physical_tool_color(const PhysicalToolPalette& palette, uint8_t physical_tool)
{
    if (physical_tool == libvgcode::UNKNOWN_PHYSICAL_TOOL_ID || physical_tool >= palette.colors.size())
        return palette.unknown;
    return palette.colors[physical_tool];
}

namespace {
std::vector<PhysicalToolLegendRow> depositing_legend_rows(const PhysicalToolPreviewSnapshot& snapshot)
{
    std::array<bool, 256> depositing {};
    for (const auto& fact : snapshot.parsed_vertex_facts())
        if (fact.type == libvgcode::EMoveType::Extrude && std::isfinite(fact.height_mm) && fact.height_mm > 0.0f)
            depositing[fact.logical_filament] = true;
    auto rows = build_physical_tool_legend_rows(snapshot);
    rows.erase(std::remove_if(rows.begin(), rows.end(), [&](const auto& row) {
        return row.logical_filament >= depositing.size() || !depositing[row.logical_filament];
    }), rows.end());
    return rows;
}

template<class VisitFacts>
PhysicalToolEventSummary summarize_physical_tool_event(
    const std::vector<PhysicalToolLegendRow>& legend_rows,
    VisitFacts visit_facts, std::optional<float> selected_event_z)
{
    PhysicalToolEventSummary summary;
    for (const PhysicalToolLegendRow& legend : legend_rows) {
        if (!legend.physical_qualified || !legend.physical_tool.has_value())
            continue;
        auto append_contributor = [&](std::vector<PhysicalToolHeightRow>& rows) {
            auto row = std::find_if(rows.begin(), rows.end(), [&](const PhysicalToolHeightRow& candidate) {
                return candidate.physical_tool == *legend.physical_tool;
            });
            if (row == rows.end()) {
                rows.push_back(PhysicalToolHeightRow{});
                rows.back().physical_tool = *legend.physical_tool;
                row = std::prev(rows.end());
            }
            row->logical_contributors.push_back(legend.logical_filament);
        };
        append_contributor(summary.model_rows);
        append_contributor(summary.tower_or_custom_rows);
    }
    auto normalize_rows = [](std::vector<PhysicalToolHeightRow>& rows) {
        std::sort(rows.begin(), rows.end(), [](const auto& left, const auto& right) {
            return left.physical_tool < right.physical_tool;
        });
        for (auto& row : rows) {
            std::sort(row.logical_contributors.begin(), row.logical_contributors.end());
            row.logical_contributors.erase(std::unique(row.logical_contributors.begin(), row.logical_contributors.end()),
                                           row.logical_contributors.end());
        }
    };
    normalize_rows(summary.model_rows);
    normalize_rows(summary.tower_or_custom_rows);

    if (selected_event_z.has_value() && !(std::isfinite(*selected_event_z) && *selected_event_z > 0.0f)) {
        summary.diagnostic_code = "selected_event_z_unknown";
        summary.diagnostic = "Selected event Z is not finite and strictly positive; event Z is unknown";
        return summary;
    }

    std::set<float> observed_event_z;
    visit_facts([&](const PhysicalToolVertexFact& fact) {
        if (selected_event_z.has_value() && fact.event_z != *selected_event_z)
            return;
        if (std::isfinite(fact.event_z) && fact.event_z > 0.0f)
            observed_event_z.insert(fact.event_z);
        if (fact.type != libvgcode::EMoveType::Extrude || !std::isfinite(fact.height_mm) || fact.height_mm <= 0.0f)
            return;
        auto& rows = fact.role == libvgcode::EGCodeExtrusionRole::WipeTower ||
                             fact.role == libvgcode::EGCodeExtrusionRole::Custom
                         ? summary.tower_or_custom_rows
                         : summary.model_rows;
        auto row = std::find_if(rows.begin(), rows.end(), [&](const PhysicalToolHeightRow& candidate) {
            return candidate.physical_tool == fact.physical_tool;
        });
        if (row != rows.end())
            row->effective_heights_mm.push_back(fact.height_mm);
    });
    auto finish_rows = [](std::vector<PhysicalToolHeightRow>& rows) {
        for (auto& row : rows) {
            std::sort(row.effective_heights_mm.begin(), row.effective_heights_mm.end());
            row.effective_heights_mm.erase(std::unique(row.effective_heights_mm.begin(), row.effective_heights_mm.end()),
                                           row.effective_heights_mm.end());
            row.has_deposition = !row.effective_heights_mm.empty();
        }
    };
    finish_rows(summary.model_rows);
    finish_rows(summary.tower_or_custom_rows);

    if (observed_event_z.size() == 1)
        summary.event_z = *observed_event_z.begin();
    else if (observed_event_z.size() > 1) {
        summary.diagnostic_code = "conflicting_event_z";
        summary.diagnostic = "Conflicting nonzero event-Z snapshots; event Z is unknown";
    }
    return summary;
}

} // namespace

PhysicalToolEventSummary build_physical_tool_event_summary(
    const PhysicalToolPreviewSnapshot& snapshot,
    const std::vector<PhysicalToolVertexFact>& final_converted_facts,
    uint32_t selected_layer_id, std::optional<float> selected_event_z, bool depositing_only)
{
    const auto rows = depositing_only ? depositing_legend_rows(snapshot) : build_physical_tool_legend_rows(snapshot);
    return summarize_physical_tool_event(rows, [&](const auto& consume) {
        for (const auto& fact : final_converted_facts)
            if (fact.layer_id == selected_layer_id)
                consume(fact);
    }, selected_event_z);
}

PhysicalToolClassification classify_physical_tool_events(
    const std::vector<PhysicalToolVertexFact>& final_converted_facts)
{
    PhysicalToolClassification result;
    std::optional<uint8_t> prior_valid_tool;
    for (std::size_t index = 0; index < final_converted_facts.size(); ++index) {
        const PhysicalToolVertexFact& fact = final_converted_facts[index];
        if (fact.type != libvgcode::EMoveType::ToolChange)
            continue;
        PhysicalToolEvent event;
        event.vertex_index = index;
        event.gcode_id = fact.gcode_id;
        event.layer_id = fact.layer_id;
        if (std::isfinite(fact.event_z) && fact.event_z > 0.0f)
            event.event_z = fact.event_z;
        event.from_tool = prior_valid_tool;
        if (fact.physical_tool == libvgcode::UNKNOWN_PHYSICAL_TOOL_ID) {
            event.kind = PhysicalToolEventKind::Unknown;
        } else {
            event.to_tool = fact.physical_tool;
            if (!prior_valid_tool.has_value())
                event.kind = PhysicalToolEventKind::InitialSelection;
            else if (*prior_valid_tool == fact.physical_tool)
                event.kind = PhysicalToolEventKind::SamePhysicalFilamentChange;
            else {
                event.kind = PhysicalToolEventKind::PhysicalTransition;
                result.marker_vertex_indices.push_back(index);
            }
            prior_valid_tool = fact.physical_tool;
        }
        result.events.push_back(std::move(event));
    }
    std::sort(result.marker_vertex_indices.begin(), result.marker_vertex_indices.end());
    result.marker_vertex_indices.erase(
        std::unique(result.marker_vertex_indices.begin(), result.marker_vertex_indices.end()),
        result.marker_vertex_indices.end());

    std::optional<std::size_t> first_model_extrusion;
    std::optional<std::size_t> last_model_extrusion;
    for (std::size_t index = 0; index < final_converted_facts.size(); ++index) {
        const auto& fact = final_converted_facts[index];
        const bool model = fact.type == libvgcode::EMoveType::Extrude &&
                           fact.role != libvgcode::EGCodeExtrusionRole::WipeTower &&
                           fact.role != libvgcode::EGCodeExtrusionRole::Custom &&
                           fact.physical_tool != libvgcode::UNKNOWN_PHYSICAL_TOOL_ID &&
                           std::isfinite(fact.height_mm) && fact.height_mm > 0.0f;
        if (model) {
            if (!first_model_extrusion.has_value())
                first_model_extrusion = index;
            last_model_extrusion = index;
        }
    }

    for (std::size_t event_index = 0; event_index < result.events.size(); ++event_index) {
        PhysicalToolEvent& event = result.events[event_index];
        if (event.kind != PhysicalToolEventKind::PhysicalTransition)
            continue;
        std::size_t span_end = final_converted_facts.size();
        for (std::size_t next = event_index + 1; next < result.events.size(); ++next) {
            if (result.events[next].kind == PhysicalToolEventKind::PhysicalTransition) {
                span_end = result.events[next].vertex_index;
                break;
            }
        }
        bool matching_model = false;
        bool matching_tower_or_custom = false;
        bool ambiguous = false;
        for (std::size_t index = event.vertex_index + 1; index < span_end; ++index) {
            const auto& fact = final_converted_facts[index];
            if (fact.type != libvgcode::EMoveType::Extrude || !std::isfinite(fact.height_mm) || fact.height_mm <= 0.0f)
                continue;
            if (fact.physical_tool == libvgcode::UNKNOWN_PHYSICAL_TOOL_ID ||
                !event.to_tool.has_value() || fact.physical_tool != *event.to_tool) {
                ambiguous = true;
                continue;
            }
            const bool event_insufficient = !event.event_z.has_value() || !std::isfinite(fact.event_z) ||
                                            fact.event_z <= 0.0f;
            // A physical tool may stay selected across later fine layers before the next tool
            // transition. Only backwards chronology conflicts with the transition identity.
            const bool event_conflict = !event_insufficient && fact.event_z + 1e-6f < *event.event_z;
            if (event_insufficient || event_conflict) {
                ambiguous = true;
                continue;
            }
            if (fact.role == libvgcode::EGCodeExtrusionRole::WipeTower ||
                fact.role == libvgcode::EGCodeExtrusionRole::Custom)
                matching_tower_or_custom = true;
            else
                matching_model = true;
        }
        if (ambiguous)
            event.scope = PhysicalToolEventScope::Ambiguous;
        else if (matching_model)
            event.scope = PhysicalToolEventScope::LeadsToModelDeposit;
        else if (matching_tower_or_custom)
            event.scope = PhysicalToolEventScope::TowerOrCustomOnly;
        else if (!first_model_extrusion.has_value() || event.vertex_index < *first_model_extrusion ||
                 event.vertex_index > *last_model_extrusion)
            event.scope = PhysicalToolEventScope::StartupOrTerminal;
        else
            event.scope = PhysicalToolEventScope::Ambiguous;

        if (event.scope == PhysicalToolEventScope::LeadsToModelDeposit)
            result.final_model_leading_event_index = event_index;
    }
    return result;
}

PhysicalToolQualification qualify_physical_tool_preview(const PhysicalToolPreviewSnapshot& snapshot)
{
    if (snapshot.mode() == PhysicalPreviewMode::Off)
        return {false, "physical_preview_off", "Physical Tool Preview is disabled"};
    if (! snapshot.plan_published())
        return {false, "plan_block_not_published",
                "Synchronized multi-nozzle layering published no layer plan - Preview qualification blocked"};
    const auto rows = build_physical_tool_legend_rows(snapshot);
    if (rows.empty())
        return {false, "configured_binding_missing", "No configured physical-tool bindings"};
    for (const auto& row : rows) {
        if (!row.physical_qualified)
            return {false,
                    row.diagnostic_code.empty() ? "physical_identity_unresolved" : row.diagnostic_code,
                    row.diagnostic.empty() ? "Physical identity unresolved - Preview qualification blocked" : row.diagnostic};
    }
    const bool has_matching_physical_extrusion = std::any_of(
        snapshot.parsed_vertex_facts().begin(), snapshot.parsed_vertex_facts().end(),
        [&rows](const PhysicalToolVertexFact& fact) {
            if (fact.type != libvgcode::EMoveType::Extrude ||
                fact.physical_tool == libvgcode::UNKNOWN_PHYSICAL_TOOL_ID ||
                !std::isfinite(fact.height_mm) || fact.height_mm <= 0.0f)
                return false;
            return std::any_of(rows.begin(), rows.end(), [&fact](const PhysicalToolLegendRow& row) {
                return row.physical_qualified && row.physical_tool == fact.physical_tool;
            });
        });
    if (!has_matching_physical_extrusion)
        return {false, "physical_extrusion_missing",
                "No valid parsed physical extrusion matches a configured physical tool"};
    return {true, "", ""};
}

std::vector<PhysicalToolViewEntry> build_physical_tool_view_entries(
    PhysicalPreviewMode mode, const PhysicalToolQualification& qualification)
{
    std::vector<PhysicalToolViewEntry> entries {
        {libvgcode::EViewType::Summary, "Summary"},
        {libvgcode::EViewType::FeatureType, "Line Type"},
        {libvgcode::EViewType::ColorPrint, "Filament"},
        {libvgcode::EViewType::Speed, "Speed"},
        {libvgcode::EViewType::ActualSpeed, "Actual Speed"},
        {libvgcode::EViewType::Acceleration, "Acceleration"},
        {libvgcode::EViewType::Jerk, "Jerk"},
        {libvgcode::EViewType::Height, "Layer Height"},
        {libvgcode::EViewType::Width, "Line Width"},
        {libvgcode::EViewType::VolumetricFlowRate, "Flow"},
        {libvgcode::EViewType::ActualVolumetricFlowRate, "Actual Flow"},
        {libvgcode::EViewType::LayerTimeLinear, "Layer Time"},
        {libvgcode::EViewType::LayerTimeLogarithmic, "Layer Time (log)"},
        {libvgcode::EViewType::FanSpeed, "Fan Speed"},
        {libvgcode::EViewType::Temperature, "Temperature"},
        {libvgcode::EViewType::PressureAdvance, "Pressure Advance"}
    };
    if (mode != PhysicalPreviewMode::Off && qualification.qualified)
        entries.push_back({libvgcode::EViewType::Tool, "Physical Tool"});
    return entries;
}

PhysicalToolSelectionDecision decide_physical_tool_selection(const PhysicalToolSelectionInput& input)
{
    PhysicalToolSelectionDecision decision;
    decision.selected = input.current_selection;
    if (input.mode == PhysicalPreviewMode::Off) {
        decision.off_count_category = input.used_logical_count > 1 ? 2 : 1;
        const auto legacy_default = input.used_logical_count > 1
                                        ? libvgcode::EViewType::ColorPrint
                                        : libvgcode::EViewType::FeatureType;
        if (input.prior_off_count_category != decision.off_count_category ||
            input.current_selection == libvgcode::EViewType::Tool) {
            decision.selected = legacy_default;
            decision.apply_selection = true;
        }
        return decision;
    }

    if (!input.qualification.qualified) {
        if (input.current_selection == libvgcode::EViewType::Tool) {
            decision.selected = input.used_logical_count > 1
                                    ? libvgcode::EViewType::ColorPrint
                                    : libvgcode::EViewType::FeatureType;
            decision.apply_selection = true;
        }
        return decision;
    }
    if (!input.consumed_mixed_result_identity.has_value() ||
        *input.consumed_mixed_result_identity != input.result_identity) {
        decision.selected = libvgcode::EViewType::Tool;
        decision.apply_selection = true;
        decision.mark_mixed_default_consumed = true;
    }
    return decision;
}

namespace {
std::vector<PhysicalToolLegendPresentationRow> legend_presentation(
    const std::vector<PhysicalToolLegendRow>& legend_rows, const PhysicalToolPalette& palette)
{
    std::vector<PhysicalToolLegendPresentationRow> rows;
    // Legend rows in snapshot order (physical T then logical F). The swatch is the deterministic
    // physical palette, bounds-safe; unresolved/255/out-of-range is gray.
    rows.reserve(legend_rows.size());
    for (const PhysicalToolLegendRow& row : legend_rows) {
        PhysicalToolLegendPresentationRow out;
        out.physical_tool = row.physical_tool;
        out.nozzle_diameter_mm = row.nozzle_diameter_mm;
        out.logical_filament = row.logical_filament;
        out.filament_preset = row.filament_preset;
        out.material = row.material;
        out.model_usage_m = row.model_usage_m;
        out.model_usage_g = row.model_usage_g;
        out.physical_qualified = row.physical_qualified;
        out.diagnostic = row.diagnostic;
        if (row.physical_tool.has_value()) {
            out.swatch = physical_tool_color(palette, *row.physical_tool);
        } else {
            out.swatch = palette.unknown;
        }
        rows.push_back(std::move(out));
    }

    return rows;
}

PhysicalToolCadencePresentation cadence_presentation(const PhysicalToolEventSummary& event_summary)
{
    PhysicalToolCadencePresentation cadence;
    // Copy the event summary verbatim (event Z separate from heights, model vs tower/custom rows
    // separate, fine-only rows keep has_deposition=false).
    cadence.event_z = event_summary.event_z;
    cadence.diagnostic_code = event_summary.diagnostic_code;
    cadence.diagnostic = event_summary.diagnostic;
    auto copy_rows = [](const std::vector<PhysicalToolHeightRow>& src,
                        std::vector<PhysicalToolCadencePresentationRow>& dst) {
        dst.reserve(src.size());
        for (const PhysicalToolHeightRow& row : src) {
            PhysicalToolCadencePresentationRow out;
            out.physical_tool = row.physical_tool;
            out.logical_contributors = row.logical_contributors;
            out.effective_heights_mm = row.effective_heights_mm;
            out.has_deposition = row.has_deposition;
            dst.push_back(std::move(out));
        }
    };
    copy_rows(event_summary.model_rows, cadence.model_rows);
    copy_rows(event_summary.tower_or_custom_rows, cadence.tower_or_custom_rows);

    return cadence;
}

std::vector<PhysicalToolTransitionPresentationRow> transition_presentation(const PhysicalToolClassification& classification)
{
    std::vector<PhysicalToolTransitionPresentationRow> rows;
    // Proven transitions only: PhysicalTransition events whose vertex index is in the marker
    // sidecar. Raw initial/same/unknown events are never presented as transitions.
    const std::vector<std::size_t>& sidecar = classification.marker_vertex_indices;
    for (const PhysicalToolEvent& event : classification.events) {
        if (event.kind != PhysicalToolEventKind::PhysicalTransition)
            continue;
        if (!std::binary_search(sidecar.begin(), sidecar.end(), event.vertex_index))
            continue;
        PhysicalToolTransitionPresentationRow out;
        out.vertex_index = event.vertex_index;
        out.from_tool = event.from_tool;
        out.to_tool = event.to_tool;
        out.event_z = event.event_z;
        switch (event.scope) {
        case PhysicalToolEventScope::LeadsToModelDeposit: out.scope_label = "model"; break;
        case PhysicalToolEventScope::TowerOrCustomOnly:  out.scope_label = "tower/custom"; break;
        case PhysicalToolEventScope::StartupOrTerminal:  out.scope_label = "startup/terminal"; break;
        case PhysicalToolEventScope::Ambiguous:          out.scope_label = "ambiguous"; break;
        }
        rows.push_back(std::move(out));
    }

    return rows;
}
} // namespace

// Mixed-Nozzle Preview presentation projection, free of wx/ImGui/OpenGL. Copies only the frozen
// snapshot outputs into plain structs; the renderer is a thin formatter over them.
PhysicalToolPresentation build_physical_tool_presentation(
    const PhysicalToolPreviewSnapshot& snapshot,
    const PhysicalToolPalette& palette,
    const PhysicalToolEventSummary& event_summary,
    const PhysicalToolClassification& classification)
{
    PhysicalToolPresentation presentation;

    presentation.legend_rows = legend_presentation(build_physical_tool_legend_rows(snapshot), palette);
    presentation.cadence = cadence_presentation(event_summary);
    presentation.transition_rows = transition_presentation(classification);

    // Diagnostics from the owned snapshot, copied verbatim.
    for (const PhysicalToolSnapshotDiagnostic& diagnostic : snapshot.diagnostics())
        presentation.diagnostics.push_back(diagnostic.diagnostic);

    return presentation;
}

void PhysicalToolPresentationCache::capture(const PhysicalToolPreviewSnapshot& snapshot,
    const PhysicalToolClassification& classification, const PhysicalToolPalette& palette)
{
    const auto start = std::chrono::steady_clock::now();
    m_snapshot = &snapshot;
    ++m_metrics.source_updates;
    m_selection.reset();
    m_layer_facts.clear();
    m_legend_rows = depositing_legend_rows(snapshot);

    // Store separate spans whenever a layer/event reappears later in vertex order.
    // No contiguity assumption, and no duplicate copy of the large fact vector.
    const auto append = [](auto& spans, std::size_t index) {
        if (!spans.empty() && spans.back().end == index)
            ++spans.back().end;
        else
            spans.push_back(FactSpan{index, index + 1});
    };
    const auto& facts = snapshot.parsed_vertex_facts();
    for (std::size_t index = 0; index < facts.size(); ++index) {
        const auto& fact = facts[index];
        auto& layer = m_layer_facts[fact.layer_id];
        append(layer.all, index);
        if (std::isfinite(fact.event_z) && fact.event_z > 0.0f)
            append(layer.events[fact.event_z], index);
    }
    m_presentation = {};
    m_presentation.legend_rows = legend_presentation(m_legend_rows, palette);
    m_presentation.transition_rows = transition_presentation(classification);
    for (const auto& diagnostic : snapshot.diagnostics())
        m_presentation.diagnostics.push_back(diagnostic.diagnostic);
    set_palette(palette);
    // The projection above already copied these exact classification facts.
    ++m_metrics.classification_updates;
    ++m_metrics.snapshot_rebuilds;
    m_metrics.last_rebuild_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    m_metrics.total_rebuild_ms += m_metrics.last_rebuild_ms;
}

void PhysicalToolPresentationCache::clear()
{
    m_snapshot = nullptr;
    m_legend_rows.clear();
    m_layer_facts.clear();
    m_selection.reset();
    m_palette = {};
    m_view_type.reset();
    m_presentation = {};
}

void PhysicalToolPresentationCache::set_palette(const PhysicalToolPalette& palette)
{
    if (m_palette.colors == palette.colors && m_palette.unknown == palette.unknown)
        return;
    m_palette = palette;
    for (auto& row : m_presentation.legend_rows)
        row.swatch = row.physical_tool ? physical_tool_color(palette, *row.physical_tool) : palette.unknown;
    ++m_metrics.palette_updates;
}

void PhysicalToolPresentationCache::set_classification(const PhysicalToolClassification& classification)
{
    m_presentation.transition_rows = transition_presentation(classification);
    ++m_metrics.classification_updates;
}

void PhysicalToolPresentationCache::set_view_type(libvgcode::EViewType view_type)
{
    if (m_view_type == view_type)
        return;
    m_view_type = view_type;
    m_selection.reset();
    ++m_metrics.view_invalidations;
}

const PhysicalToolPresentation& PhysicalToolPresentationCache::presentation(uint32_t layer_id, std::optional<float> event_z)
{
    ++m_metrics.presentation_requests;
    if (!m_snapshot)
        return m_presentation;
    const auto same_event = [](std::optional<float> left, std::optional<float> right) {
        if (left.has_value() != right.has_value()) return false;
        if (!left) return true;
        const bool left_valid = std::isfinite(*left) && *left > 0.0f;
        const bool right_valid = std::isfinite(*right) && *right > 0.0f;
        return (!left_valid && !right_valid) || (left_valid && right_valid && *left == *right);
    };
    if (m_selection && m_selection->layer_id == layer_id && same_event(m_selection->event_z, event_z))
        return m_presentation;
    const auto start = std::chrono::steady_clock::now();
    m_selection = Selection{layer_id, event_z};
    const auto layer = m_layer_facts.find(layer_id);
    const std::vector<FactSpan>* spans = nullptr;
    if (layer != m_layer_facts.end()) {
        if (!event_z)
            spans = &layer->second.all;
        else if (std::isfinite(*event_z) && *event_z > 0.0f) {
            const auto event = layer->second.events.find(*event_z);
            if (event != layer->second.events.end())
                spans = &event->second;
        }
    }
    const auto summary = summarize_physical_tool_event(m_legend_rows, [&](const auto& consume) {
        if (spans)
            for (const auto& span : *spans)
                for (std::size_t index = span.first; index < span.end; ++index) {
                    ++m_metrics.event_fact_visits;
                    consume(m_snapshot->parsed_vertex_facts()[index]);
                }
    }, event_z);
    m_presentation.cadence = cadence_presentation(summary);
    ++m_metrics.event_rebuilds;
    m_metrics.last_event_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    m_metrics.total_event_ms += m_metrics.last_event_ms;
    return m_presentation;
}

} // namespace Slic3r::GUI
