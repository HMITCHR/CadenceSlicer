#include <catch2/catch_all.hpp>

#include "fff_print/rebuild_harness.hpp"
#include "libslic3r/MixedNozzleConfig.hpp"
#include "slic3r/GUI/LibVGCode/LibVGCodeWrapper.hpp"
#include "slic3r/GUI/MixedNozzleDecisionReport.hpp"
#include "slic3r/GUI/PhysicalToolPreviewModel.hpp"

#include <algorithm>
#include <map>
#include <set>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::GUI;
using namespace CadenceTest;

namespace {

struct SlicedPreview {
    Model model;
    Print print;
    Facts facts;
    PhysicalToolPreviewSnapshotInput input;

    explicit SlicedPreview(Scene scene)
    {
        srl_fixtures::fill_per_filament_values(scene.config);
        print.is_BBL_printer() = scene.bambu;
        scene.populate(model, print, scene.config);
        print.set_status_silent();
        facts = slice(print);
        INFO(facts.refusal.string);
        REQUIRE(facts.refusal.string.empty());
        REQUIRE_FALSE(facts.gcode.empty());

        const PrintConfig &cfg = print.config();
        input.mode = cfg.mixed_nozzle_slicing_mode.value == MixedNozzleSlicingMode::FeatureSplit
                         ? PhysicalPreviewMode::FeatureSplit
                         : cfg.mixed_nozzle_slicing_mode.value == MixedNozzleSlicingMode::BodySplit
                               ? PhysicalPreviewMode::BodySplit
                               : PhysicalPreviewMode::Off;
        input.filament_map_mode = cfg.filament_map_mode.serialize();
        input.filament_map = cfg.filament_map.values;
        input.nozzle_diameters_mm = cfg.nozzle_diameter.values;
        input.filament_materials = cfg.filament_type.values;
        input.plan_published = input.mode == PhysicalPreviewMode::Off ||
                               !print.regional_layer_plan_export().block.empty();

        GCodeProcessorResult processed;
        processed.moves = facts.moves;
        processed.filament_densities.assign(256, 1.24f);
        libvgcode::Viewer viewer;
        const auto final_vertices = libvgcode::convert(processed, {}, {}, viewer);
        input.parsed_vertex_facts = libvgcode::project_physical_tool_vertex_facts(final_vertices.vertices);
        for (std::size_t logical = 0; logical < input.filament_map.size(); ++logical) {
            PhysicalToolBindingFact binding;
            binding.logical_filament = logical;
            const MixedNozzleToolResolution resolution =
                resolve_mixed_nozzle_tool(cfg, logical, MixedNozzleResolveScope::PhysicalToolOnly);
            if (resolution)
                binding.configured_physical_tool = static_cast<uint8_t>(resolution.tool->physical_extruder);
            input.configured_bindings.push_back(std::move(binding));
        }
    }
};

} // namespace

TEST_CASE("A real Feature Split slice feeds a decision report that agrees with emitted coarse infill",
          "[TestRebuild][DecisionReport][Preview]")
{
    SlicedPreview scene(feature_cube());
    REQUIRE_FALSE(scene.print.objects().empty());
    const auto published = scene.print.objects().front()->mixed_nozzle_advisor_observations();
    REQUIRE(published != nullptr);
    MixedNozzleDecisionInput input;
    input.observations = published.get();
    input.physical_tool_changes = scene.facts.emitted.tool_changes.size();
    input.coarse_nozzle_diameter = scene.print.config().nozzle_diameter.values.back();
    const MixedNozzleDecisionSummary report = build_decision_summary(input);
    const auto rows = build_decision_band_rows(*published);
    REQUIRE(report.available);
    REQUIRE(report.coarse_band_count > 0);
    REQUIRE(report.coarse_z_high.has_value());
    CHECK(report.physical_tool_changes == scene.facts.emitted.tool_changes.size());

    std::size_t coarse_roads = 0;
    double last_coarse_z = 0.;
    for (const auto &move : scene.facts.moves) {
        if (!model_road(move) || move.physical_tool_id != 1 ||
            move.extrusion_role != erInternalInfill)
            continue;
        ++coarse_roads;
        last_coarse_z = std::max(last_coarse_z, double(move.print_z));
        const bool covered = std::any_of(rows.begin(), rows.end(), [&move](const auto &band) {
            return band.coarse && move.print_z >= band.z_low - 0.02 &&
                   move.print_z <= band.z_high + 0.02;
        });
        CHECK(covered);
    }
    REQUIRE(coarse_roads > 0);
    CHECK_THAT(*report.coarse_z_high, Catch::Matchers::WithinAbs(last_coarse_z, 0.37));
    CHECK_FALSE(format_decision_headline(report).empty());
}

TEST_CASE("A real two-nozzle slice reaches the physical preview legend and transition model",
          "[TestRebuild][PhysicalPreview]")
{
    SlicedPreview scene(feature_cube());
    REQUIRE_FALSE(scene.facts.emitted.tool_changes.empty());
    const PhysicalToolPreviewSnapshot snapshot = make_physical_tool_preview_snapshot(std::move(scene.input));
    const PhysicalToolQualification qualification = qualify_physical_tool_preview(snapshot);
    INFO(qualification.diagnostic_code);
    REQUIRE(qualification.qualified);
    const auto entries = build_physical_tool_view_entries(PhysicalPreviewMode::FeatureSplit, qualification);
    CHECK(std::any_of(entries.begin(), entries.end(), [](const auto &entry) {
        return entry.type == libvgcode::EViewType::Tool;
    }));

    const PhysicalToolClassification classification = classify_physical_tool_events(snapshot.parsed_vertex_facts());
    const PhysicalToolPalette palette = make_physical_tool_palette(2);
    PhysicalToolPresentationCache cache;
    cache.capture(snapshot, classification, palette);
    const auto &presentation = cache.presentation(1, std::nullopt);
    std::set<std::size_t> printing_logical;
    std::set<uint8_t> printing_physical;
    for (const auto &move : scene.facts.moves)
        if (model_road(move)) {
            INFO("model role " << int(move.extrusion_role) << " tool " << int(move.physical_tool_id));
            REQUIRE(move.physical_tool_id < scene.print.config().nozzle_diameter.values.size());
            printing_logical.insert(move.extruder_id);
            printing_physical.insert(move.physical_tool_id);
        }
    const std::set<uint8_t> expected_tools {0, 1};
    REQUIRE(printing_physical == expected_tools);
    REQUIRE_FALSE(presentation.legend_rows.empty());
    for (const auto &row : presentation.legend_rows) {
        CHECK(printing_logical.count(row.logical_filament) == 1);
        REQUIRE(row.physical_tool.has_value());
        CHECK(printing_physical.count(*row.physical_tool) == 1);
        CHECK(row.physical_qualified);
    }
    REQUIRE_FALSE(presentation.transition_rows.empty());
    for (const auto &row : presentation.transition_rows) {
        REQUIRE(row.from_tool.has_value());
        REQUIRE(row.to_tool.has_value());
        CHECK(*row.from_tool != *row.to_tool);
        CHECK(std::any_of(scene.facts.emitted.tool_changes.begin(),
                          scene.facts.emitted.tool_changes.end(), [&row](const auto &change) {
            return change.from_physical_tool == *row.from_tool &&
                   change.to_physical_tool == *row.to_tool;
        }));
    }
}

TEST_CASE("An Off slice does not offer the physical tool preview", "[TestRebuild][PhysicalPreview][Off]")
{
    Scene off = feature_cube();
    off.config.set_key_value("mixed_nozzle_slicing_mode",
        new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::Off));
    SlicedPreview scene(std::move(off));
    const PhysicalToolPreviewSnapshot snapshot = make_physical_tool_preview_snapshot(std::move(scene.input));
    const PhysicalToolQualification qualification = qualify_physical_tool_preview(snapshot);
    CHECK_FALSE(qualification.qualified);
    const auto entries = build_physical_tool_view_entries(PhysicalPreviewMode::Off, qualification);
    CHECK_FALSE(std::any_of(entries.begin(), entries.end(), [](const auto &entry) {
        return entry.type == libvgcode::EViewType::Tool;
    }));
}
