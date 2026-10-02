#ifndef SLIC3R_GUI_PHYSICAL_TOOL_PREVIEW_MODEL_HPP
#define SLIC3R_GUI_PHYSICAL_TOOL_PREVIEW_MODEL_HPP

#include "../../libvgcode/include/Types.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <map>
#include <string>
#include <vector>

namespace Slic3r::GUI {

struct PhysicalToolVertexFact {
    libvgcode::EMoveType type {libvgcode::EMoveType::Noop};
    libvgcode::EGCodeExtrusionRole role {libvgcode::EGCodeExtrusionRole::None};
    uint32_t gcode_id {0};
    uint32_t layer_id {0};
    uint8_t logical_filament {0};
    uint8_t physical_tool {libvgcode::UNKNOWN_PHYSICAL_TOOL_ID};
    float event_z {0.0f};
    float height_mm {0.0f};
};

enum class PhysicalPreviewMode { Off, FeatureSplit, BodySplit };

// The parsed result owns names for imported G-code. Only a locally sliced result
// may fill missing entries from the frozen Print config, never from live presets.
std::vector<std::string> physical_tool_snapshot_filament_names(
    const std::vector<std::string>& result_names, const std::vector<std::string>& sliced_names,
    bool allow_sliced_fallback);

struct PhysicalToolLogicalUsage {
    std::size_t logical_filament {0};
    double model_usage_m {0.0};
    double model_usage_g {0.0};
};

struct PhysicalToolBindingFact {
    std::size_t logical_filament {0};
    std::optional<uint8_t> configured_physical_tool;
    std::string resolver_code;
    std::string resolver_diagnostic;
};

struct PhysicalToolSnapshotDiagnostic {
    std::string code;
    std::string diagnostic;
    std::size_t logical_filament {0};
};

struct PhysicalToolPreviewSnapshotInput {
    uint32_t result_identity {0};
    PhysicalPreviewMode mode {PhysicalPreviewMode::Off};
    std::string filament_map_mode;
    std::vector<int> filament_map;
    std::vector<double> nozzle_diameters_mm;
    std::vector<std::string> filament_materials;
    std::vector<std::string> filament_preset_ids;
    std::vector<PhysicalToolBindingFact> configured_bindings;
    std::vector<PhysicalToolLogicalUsage> logical_usage;
    std::vector<PhysicalToolVertexFact> parsed_vertex_facts;
    bool plan_published {false};
};

class PhysicalToolPreviewSnapshot {
public:
    uint32_t result_identity() const { return m_result_identity; }
    PhysicalPreviewMode mode() const { return m_mode; }
    const std::string& filament_map_mode() const { return m_filament_map_mode; }
    const std::vector<int>& filament_map() const { return m_filament_map; }
    const std::vector<double>& nozzle_diameters_mm() const { return m_nozzle_diameters_mm; }
    const std::vector<std::string>& filament_materials() const { return m_filament_materials; }
    const std::vector<std::string>& filament_preset_ids() const { return m_filament_preset_ids; }
    const std::vector<PhysicalToolBindingFact>& configured_bindings() const { return m_configured_bindings; }
    const std::vector<PhysicalToolLogicalUsage>& logical_usage() const { return m_logical_usage; }
    const std::vector<PhysicalToolVertexFact>& parsed_vertex_facts() const { return m_parsed_vertex_facts; }
    bool plan_published() const { return m_plan_published; }
    const std::vector<uint8_t>& parsed_used_physical_tools() const { return m_parsed_used_physical_tools; }
    const std::vector<PhysicalToolSnapshotDiagnostic>& diagnostics() const { return m_diagnostics; }

private:
    explicit PhysicalToolPreviewSnapshot(PhysicalToolPreviewSnapshotInput input);
    friend PhysicalToolPreviewSnapshot make_physical_tool_preview_snapshot(PhysicalToolPreviewSnapshotInput input);

    uint32_t m_result_identity {0};
    PhysicalPreviewMode m_mode {PhysicalPreviewMode::Off};
    std::string m_filament_map_mode;
    std::vector<int> m_filament_map;
    std::vector<double> m_nozzle_diameters_mm;
    std::vector<std::string> m_filament_materials;
    std::vector<std::string> m_filament_preset_ids;
    std::vector<PhysicalToolBindingFact> m_configured_bindings;
    std::vector<PhysicalToolLogicalUsage> m_logical_usage;
    std::vector<PhysicalToolVertexFact> m_parsed_vertex_facts;
    bool m_plan_published {false};
    std::vector<uint8_t> m_parsed_used_physical_tools;
    std::vector<PhysicalToolSnapshotDiagnostic> m_diagnostics;
};

PhysicalToolPreviewSnapshot make_physical_tool_preview_snapshot(PhysicalToolPreviewSnapshotInput input);

enum class PhysicalToolBindingStatus { Resolved, ParsedOnly, Unresolved, Mismatch };

struct PhysicalToolLegendRow {
    std::optional<uint8_t> physical_tool;
    std::optional<double> nozzle_diameter_mm;
    std::size_t logical_filament {0};
    std::string filament_preset;
    std::string material;
    PhysicalToolBindingStatus status {PhysicalToolBindingStatus::Unresolved};
    bool physical_qualified {false};
    std::string diagnostic_code;
    std::string diagnostic;
    double model_usage_m {0.0};
    double model_usage_g {0.0};
};

std::vector<PhysicalToolLegendRow>
build_physical_tool_legend_rows(const PhysicalToolPreviewSnapshot& snapshot);

struct PhysicalToolPalette {
    std::vector<libvgcode::Color> colors;
    libvgcode::Color unknown {libvgcode::DUMMY_COLOR};
};

PhysicalToolPalette make_physical_tool_palette(std::size_t tool_count);
libvgcode::Color physical_tool_color(const PhysicalToolPalette& palette, uint8_t physical_tool);

struct PhysicalToolHeightRow {
    uint8_t physical_tool {libvgcode::UNKNOWN_PHYSICAL_TOOL_ID};
    std::vector<std::size_t> logical_contributors;
    std::vector<float> effective_heights_mm;
    bool has_deposition {false};
};

struct PhysicalToolEventSummary {
    std::optional<float> event_z;
    std::vector<PhysicalToolHeightRow> model_rows;
    std::vector<PhysicalToolHeightRow> tower_or_custom_rows;
    std::string diagnostic_code;
    std::string diagnostic;
};

PhysicalToolEventSummary build_physical_tool_event_summary(
    const PhysicalToolPreviewSnapshot& snapshot,
    const std::vector<PhysicalToolVertexFact>& final_converted_facts,
    uint32_t selected_layer_id,
    std::optional<float> selected_event_z,
    bool depositing_only = false);

enum class PhysicalToolEventKind {
    InitialSelection,
    SamePhysicalFilamentChange,
    PhysicalTransition,
    Unknown
};

enum class PhysicalToolEventScope {
    LeadsToModelDeposit,
    TowerOrCustomOnly,
    StartupOrTerminal,
    Ambiguous
};

struct PhysicalToolEvent {
    std::size_t vertex_index {0};
    uint32_t gcode_id {0};
    uint32_t layer_id {0};
    std::optional<float> event_z;
    std::optional<uint8_t> from_tool;
    std::optional<uint8_t> to_tool;
    PhysicalToolEventKind kind {PhysicalToolEventKind::Unknown};
    PhysicalToolEventScope scope {PhysicalToolEventScope::Ambiguous};
};

struct PhysicalToolClassification {
    std::vector<PhysicalToolEvent> events;
    std::vector<std::size_t> marker_vertex_indices;
    std::optional<std::size_t> final_model_leading_event_index;
};

PhysicalToolClassification classify_physical_tool_events(
    const std::vector<PhysicalToolVertexFact>& final_converted_facts);

struct PhysicalToolViewEntry {
    libvgcode::EViewType type {libvgcode::EViewType::FeatureType};
    std::string label;

    bool operator==(const PhysicalToolViewEntry& other) const
    {
        return type == other.type && label == other.label;
    }
};

struct PhysicalToolQualification {
    bool qualified {false};
    std::string diagnostic_code;
    std::string diagnostic;
};

PhysicalToolQualification qualify_physical_tool_preview(const PhysicalToolPreviewSnapshot& snapshot);
std::vector<PhysicalToolViewEntry> build_physical_tool_view_entries(
    PhysicalPreviewMode mode, const PhysicalToolQualification& qualification);

struct PhysicalToolSelectionInput {
    PhysicalPreviewMode mode {PhysicalPreviewMode::Off};
    PhysicalToolQualification qualification;
    uint32_t result_identity {0};
    std::optional<uint32_t> consumed_mixed_result_identity;
    std::size_t used_logical_count {0};
    int prior_off_count_category {0};
    libvgcode::EViewType current_selection {libvgcode::EViewType::FeatureType};
};

struct PhysicalToolSelectionDecision {
    libvgcode::EViewType selected {libvgcode::EViewType::FeatureType};
    bool apply_selection {false};
    bool mark_mixed_default_consumed {false};
    bool tool_changes_enabled {false};
    int off_count_category {0};
};

PhysicalToolSelectionDecision decide_physical_tool_selection(const PhysicalToolSelectionInput& input);

// Mixed-Nozzle Preview presentation projection, free of wx/ImGui/OpenGL. Consumes only the frozen
// snapshot, palette, event summary and classification, and returns plain rows for the physical
// legend, cadence and proven-transition list. No mapping or classification happens here.
struct PhysicalToolLegendPresentationRow
{
    std::optional<uint8_t> physical_tool;
    std::optional<double> nozzle_diameter_mm;
    std::size_t logical_filament {0};
    std::string filament_preset;
    std::string material;
    libvgcode::Color swatch;     // deterministic physical palette; unresolved gray
    double model_usage_m {0.0};
    double model_usage_g {0.0};
    std::string diagnostic;      // empty when resolved
    bool physical_qualified {false};
};

struct PhysicalToolCadencePresentationRow
{
    uint8_t physical_tool {libvgcode::UNKNOWN_PHYSICAL_TOOL_ID};
    std::vector<std::size_t> logical_contributors;
    std::vector<float> effective_heights_mm;
    bool has_deposition {false};
};

struct PhysicalToolCadencePresentation
{
    std::optional<float> event_z;
    std::vector<PhysicalToolCadencePresentationRow> model_rows;
    std::vector<PhysicalToolCadencePresentationRow> tower_or_custom_rows;
    std::string diagnostic_code;
    std::string diagnostic;
};

struct PhysicalToolTransitionPresentationRow
{
    std::size_t vertex_index {0};
    std::optional<uint8_t> from_tool;
    std::optional<uint8_t> to_tool;
    std::optional<float> event_z;
    std::string scope_label;     // semantic scope; localized once in the renderer
};

struct PhysicalToolPresentation
{
    std::vector<PhysicalToolLegendPresentationRow> legend_rows;
    PhysicalToolCadencePresentation cadence;
    std::vector<PhysicalToolTransitionPresentationRow> transition_rows;
    std::vector<std::string> diagnostics;
};

// Build the presentation projection from the owned snapshot outputs. Transition rows include
// only proven different-T PhysicalTransition events in the marker sidecar. Logical usage is read
// from each legend row's logical-keyed aggregate, never indexed by physical tool.
PhysicalToolPresentation build_physical_tool_presentation(
    const PhysicalToolPreviewSnapshot& snapshot,
    const PhysicalToolPalette& palette,
    const PhysicalToolEventSummary& event_summary,
    const PhysicalToolClassification& classification);

// Counts and elapsed times from the cache invoked by GCodeViewer, not an independent
// benchmark model. Times are wall-clock CPU call durations, never predicted print time.
struct PhysicalToolPreviewCacheMetrics {
    uint64_t source_updates {0};
    uint64_t snapshot_rebuilds {0};
    uint64_t presentation_requests {0};
    uint64_t event_rebuilds {0};
    uint64_t event_fact_visits {0};
    uint64_t palette_updates {0};
    uint64_t classification_updates {0};
    uint64_t view_invalidations {0};
    uint64_t render_frames {0};
    uint64_t toolpath_frames {0};
    double last_rebuild_ms {0.0};
    double total_rebuild_ms {0.0};
    double last_event_ms {0.0};
    double total_event_ms {0.0};
    double capture_ms {0.0};
    double total_legend_render_ms {0.0};
    double total_toolpath_submit_ms {0.0};
};

// Snapshot lifetime must exceed this cache's use. capture() replaces every derived
// source even when the result id is reused; clear() releases it before owner reset.
// Palette/classification/view changes go through their setters, never mutable aliases.
class PhysicalToolPresentationCache {
public:
    void capture(const PhysicalToolPreviewSnapshot& snapshot,
                 const PhysicalToolClassification& classification, const PhysicalToolPalette& palette);
    void clear();
    void set_palette(const PhysicalToolPalette& palette);
    void set_classification(const PhysicalToolClassification& classification);
    void set_view_type(libvgcode::EViewType view_type);
    const PhysicalToolPresentation& presentation(uint32_t layer_id, std::optional<float> event_z);
    const PhysicalToolPreviewCacheMetrics& metrics() const { return m_metrics; }
    void record_capture(double elapsed_ms) { m_metrics.capture_ms = elapsed_ms; }
    void record_toolpath_submit(double elapsed_ms) {
        ++m_metrics.toolpath_frames;
        m_metrics.total_toolpath_submit_ms += elapsed_ms;
    }
    void record_legend_render(double elapsed_ms) {
        ++m_metrics.render_frames;
        m_metrics.total_legend_render_ms += elapsed_ms;
    }

private:
    const PhysicalToolPreviewSnapshot* m_snapshot {nullptr};
    struct FactSpan { std::size_t first, end; };
    struct LayerFacts {
        std::vector<FactSpan> all;
        std::map<float, std::vector<FactSpan>> events;
    };
    struct Selection { uint32_t layer_id; std::optional<float> event_z; };
    std::map<uint32_t, LayerFacts> m_layer_facts;
    std::vector<PhysicalToolLegendRow> m_legend_rows;
    std::optional<Selection> m_selection;
    PhysicalToolPalette m_palette;
    std::optional<libvgcode::EViewType> m_view_type;
    PhysicalToolPresentation m_presentation;
    PhysicalToolPreviewCacheMetrics m_metrics;
};

} // namespace Slic3r::GUI

#endif // SLIC3R_GUI_PHYSICAL_TOOL_PREVIEW_MODEL_HPP
