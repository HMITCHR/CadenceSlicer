#include <catch2/catch_all.hpp>

#include "mixed_nozzle_harness.hpp"
#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/MixedNozzleConfig.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <sstream>
#include <utility>

using namespace Slic3r;
using namespace mixed_nozzle_fixtures;

namespace {
using Move = GCodeProcessorResult::MoveVertex;

std::pair<double, double> emitted_gap(const CadenceTest::Facts &facts, double x_min, double x_max)
{
    double object_bottom = std::numeric_limits<double>::max();
    for (const Move &move : facts.moves)
        if (CadenceTest::model_road(move) && move.position.x() > x_min && move.position.x() < x_max)
            object_bottom = std::min(object_bottom, double(move.position.z()) - double(move.height));
    double support_top = 0.;
    for (const Move &move : facts.moves)
        if (move.type == EMoveType::Extrude &&
            (move.extrusion_role == erSupportMaterial || move.extrusion_role == erSupportMaterialInterface) &&
            move.position.x() > x_min && move.position.x() < x_max &&
            double(move.position.z()) < object_bottom + 1e-4)
            support_top = std::max(support_top, double(move.position.z()));
    return {support_top, object_bottom};
}

CadenceTest::Facts require_feature_slice(Print &print)
{
    const CadenceTest::Facts facts = CadenceTest::slice(print);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    CHECK_FALSE(facts.emitted.plan_malformed);
    return facts;
}
} // namespace

TEST_CASE("A Feature Split layer tool change owns the shelf overhang and bridge",
          "[TestRebuild][FeatureSupport][ToolOverride]")
{
    DynamicPrintConfig config = coupon_config(false);
    config.set_key_value("mixed_nozzle_slicing_mode",
        new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::FeatureSplit));
    config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(.20));
    config.set_key_value("detect_overhang_wall", new ConfigOptionBool(true));
    config.set_key_value("sparse_infill_density", new ConfigOptionPercent(0.));
    for (const char *role : {"outer_wall_filament_id", "inner_wall_filament_id",
                             "internal_solid_filament_id", "top_surface_filament_id",
                             "bottom_surface_filament_id"})
        config.set_key_value(role, new ConfigOptionInt(1));
    config.set_key_value("sparse_infill_filament_id", new ConfigOptionInt(2));
    fill_per_filament_values(config);

    Model model;
    Print print;
    init_feature_overhang_fixture(model, print, config);
    CustomGCode::Info changes;
    changes.mode = CustomGCode::MultiAsSingle;
    changes.gcodes.push_back({1.0, CustomGCode::ToolChange, 2, {}, {}});
    model.plates_custom_gcodes[model.curr_plate_index] = changes;
    print.apply(model, config);
    const CadenceTest::Facts facts = require_feature_slice(print);

    size_t earlier_fine = 0, overhang = 0, bridge = 0, wrong_owner = 0;
    for (const Move &move : facts.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        if (move.position.z() < 1.0 - 1e-3 && move.physical_tool_id == 0 &&
            CadenceTest::model_road(move))
            ++earlier_fine;
        if (move.extrusion_role == erOverhangPerimeter) {
            ++overhang;
            wrong_owner += move.physical_tool_id != 1;
        } else if (move.extrusion_role == erBridgeInfill) {
            ++bridge;
            wrong_owner += move.physical_tool_id != 1;
        }
    }
    CAPTURE(earlier_fine, overhang, bridge, wrong_owner);
    CHECK(earlier_fine > 0);
    CHECK(overhang > 0);
    CHECK(bridge > 0);
    CHECK(wrong_owner == 0);
    CHECK_FALSE(facts.emitted.tool_changes.empty());
}

TEST_CASE("Feature Split deposits assigned support base and interface at each nozzle width",
          "[TestRebuild][FeatureSupport]")
{
    DynamicPrintConfig config = coupon_config(false);
    config.set_key_value("mixed_nozzle_slicing_mode",
                         new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::FeatureSplit));
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.6, 0.2});
    config.set_key_value("filament_map", new ConfigOptionInts{2, 1});
    config.set_key_value("enable_support", new ConfigOptionBool(true));
    config.set_key_value("support_filament", new ConfigOptionInt(2));
    config.set_key_value("support_interface_filament", new ConfigOptionInt(1));
    config.set_key_value("support_type", new ConfigOptionEnum<SupportType>(stNormalAuto));
    config.set_key_value("support_line_width", new ConfigOptionFloatOrPercent(0., false));
    config.set_key_value("line_width", new ConfigOptionFloatOrPercent(0., false));
    config.set_key_value("independent_support_layer_height", new ConfigOptionBool(false));
    config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(0.20));
    config.set_key_value("outer_wall_filament_id", new ConfigOptionInt(1));
    config.set_key_value("sparse_infill_filament_id", new ConfigOptionInt(1));
    config.set_key_value("sparse_infill_density", new ConfigOptionPercent(0.));
    config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btOuterOnly));
    config.set_key_value("brim_width", new ConfigOptionFloat(4.));
    config.set_key_value("skirt_loops", new ConfigOptionInt(0));
    fill_per_filament_values(config);
    Model model;
    Print print;
    init_feature_overhang_fixture(model, print, config);
    const CadenceTest::Facts facts = require_feature_slice(print);

    size_t base = 0, interface_roads = 0, brim = 0, wrong = 0;
    for (const Move &move : facts.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        if (move.extrusion_role == erBrim) {
            ++brim;
            wrong += move.physical_tool_id != 1;
        } else if (move.extrusion_role == erSupportMaterial) {
            ++base;
            // The dense base-material layer against the interface is one object layer tall, so the 0.2
            // lays it at its own width.
            wrong += (move.physical_tool_id != 0 || std::abs(move.width - 0.6f) > 1e-3f) &&
                     (move.physical_tool_id != 1 || std::abs(move.width - 0.2f) > 1e-3f);
        } else if (move.extrusion_role == erSupportMaterialInterface) {
            ++interface_roads;
            wrong += move.physical_tool_id != 1 || std::abs(move.width - 0.2f) > 1e-3f;
        }
    }
    CAPTURE(base, interface_roads, brim, wrong);
    CHECK(brim > 0);
    CHECK(base > 0);
    CHECK(interface_roads > 0);
    CHECK(wrong == 0);
}

TEST_CASE("Feature Split support and raft retain their recommended object-grid Z gaps",
          "[TestRebuild][FeatureSupport]")
{
    SECTION("recommended PETG interface touches the PLA shelf") {
        DynamicPrintConfig config = coarse_base_support_config(MixedNozzleSlicingMode::FeatureSplit);
        config.set_key_value("filament_diameter", new ConfigOptionFloats{1.75, 1.75, 1.75});
        config.set_key_value("filament_map", new ConfigOptionInts{1, 2, 1});
        config.set_key_value("flush_volumes_matrix", new ConfigOptionFloats(18, 0.));
        // Not resized with the other per-filament values; export needs a colour per filament.
        config.set_key_value("filament_colour", new ConfigOptionStrings{"#FF0000", "#0000FF", "#00FF00"});
        config.set_key_value("filament_self_index", new ConfigOptionInts{1, 2, 3});
        config.set_key_value("filament_volume_map", new ConfigOptionInts{0, 0, 0});
        config.set_key_value("filament_start_gcode", new ConfigOptionStrings(3, ""));
        config.set_key_value("filament_end_gcode", new ConfigOptionStrings(3, ""));
        config.set_key_value("filament_type", new ConfigOptionStrings{"PLA", "PLA", "PETG"});
        // Stock PLA and PETG presets use separate tower adhesion blocks.
        config.set_key_value("filament_adhesiveness_category", new ConfigOptionInts{100, 100, 300});
        config.set_key_value("filament_soluble", new ConfigOptionBools{false, false, false});
        config.set_key_value("filament_is_support", new ConfigOptionBools{false, false, false});
        config.set_key_value("support_interface_filament", new ConfigOptionInt(3));
        config.set_key_value("support_top_z_distance", new ConfigOptionFloat(0.20));
        config.set_key_value("support_interface_spacing", new ConfigOptionFloat(0.5));
        config.set_key_value("support_interface_pattern", new ConfigOptionEnum<SupportMaterialInterfacePattern>(
            SupportMaterialInterfacePattern::smipConcentric));
        config.set_key_value("independent_support_layer_height", new ConfigOptionBool(true));
        REQUIRE(mixed_nozzle_apply_support_interface_recommendation(config));
        fill_per_filament_values(config);
        Model model;
        Print print;
        print.is_BBL_printer() = true;
        init_feature_overhang_fixture(model, print, config);
        const CadenceTest::Facts facts = [&]() {
            try {
                return require_feature_slice(print);
            } catch (...) {
                std::ostringstream detail;
                DynamicPrintConfig probe_config = config;
                probe_config.set_key_value("mixed_nozzle_auto_pad", new ConfigOptionBool(false));
                Model probe_model;
                Print probe;
                probe.is_BBL_printer() = true;
                init_feature_overhang_fixture(probe_model, probe, probe_config);
                try {
                    (void) CadenceTest::slice(probe);
                } catch (const std::exception &error) {
                    detail << "manual tower probe: " << error.what() << '\n';
                }
                struct Prior { WipeTower::StructuralEmission emission; size_t visit; size_t change; };
                std::vector<Prior> prior;
                float previous_z = 0.f;
                bool found = false;
                const auto &visits = probe.wipe_tower_data().tool_changes;
                detail << "manual tower visits " << visits.size() << '\n';
                for (size_t visit = 0; visit < visits.size() && !found; ++visit)
                    for (size_t change = 0; change < visits[visit].size() && !found; ++change)
                        for (const auto &emission : visits[visit][change].structural_emissions) {
                            if (emission.support_domain != 3 || emission.role == WipeTower::StructuralRole::None)
                                continue;
                            if (emission.z < previous_z - 1e-4f) {
                                detail << "domain-2 reverse emission at visit " << visit << ", change " << change
                                       << ": role " << int(emission.role) << ", tool " << emission.logical_tool
                                       << ", z " << emission.z << ", height " << emission.height
                                       << ", footprint " << emission.footprint_min.transpose() << " to "
                                       << emission.footprint_max.transpose() << ", previous z " << previous_z << '\n';
                                size_t overlap = 0;
                                for (const Prior &item : prior) {
                                    const auto &old = item.emission;
                                    const bool intersects = old.footprint_min.x() <= emission.footprint_max.x() &&
                                        emission.footprint_min.x() <= old.footprint_max.x() &&
                                        old.footprint_min.y() <= emission.footprint_max.y() &&
                                        emission.footprint_min.y() <= old.footprint_max.y();
                                    overlap += intersects;
                                    if (prior.size() <= 20 || intersects)
                                        detail << "  prior visit " << item.visit << ", change " << item.change
                                               << ": role " << int(old.role) << ", tool " << old.logical_tool
                                               << ", z " << old.z << ", height " << old.height
                                               << ", intersects " << intersects << ", footprint "
                                               << old.footprint_min.transpose() << " to "
                                               << old.footprint_max.transpose() << '\n';
                                }
                                detail << "prior count " << prior.size() << ", bounding-box intersections " << overlap;
                                found = true;
                                break;
                            }
                            if (!emission.held_above_level)
                                previous_z = std::max(previous_z, emission.z);
                            prior.push_back({emission, visit, change});
                        }
                std::cerr << "Feature support tower diagnosis: " << detail.str() << '\n';
                throw;
            }
        }();
        const double left = model.objects.front()->instance_bounding_box(0).min.x();
        const auto [interface_top, object_bottom] = emitted_gap(facts, left + 13., left + 19.);
        CAPTURE(interface_top, object_bottom);
        REQUIRE(interface_top > 0.);
        size_t petg_interface = 0;
        for (const Move &move : facts.moves)
            if (move.type == EMoveType::Extrude && move.extrusion_role == erSupportMaterialInterface &&
                move.physical_tool_id == 0 && move.extruder_id == 2 &&
                move.position.x() > left + 13. && move.position.x() < left + 19. &&
                std::abs(double(move.position.z()) - interface_top) < 1e-3)
                ++petg_interface;
        CHECK(petg_interface > 0);
        CHECK_THAT(object_bottom - interface_top, Catch::Matchers::WithinAbs(0., 1e-3));
    }
    SECTION("shelf support rounds 0.25 mm to the next 0.10 mm object row") {
        DynamicPrintConfig config = coarse_base_support_config(MixedNozzleSlicingMode::FeatureSplit);
        config.set_key_value("support_top_z_distance", new ConfigOptionFloat(0.25));
        config.set_key_value("support_bottom_z_distance", new ConfigOptionFloat(0.25));
        fill_per_filament_values(config);
        Model model;
        Print print;
        print.is_BBL_printer() = true;
        init_feature_overhang_fixture(model, print, config);
        const CadenceTest::Facts facts = require_feature_slice(print);
        const double left = model.objects.front()->instance_bounding_box(0).min.x();
        const auto [support_top, object_bottom] = emitted_gap(facts, left + 13., left + 19.);
        CAPTURE(support_top, object_bottom);
        REQUIRE(support_top > 0.);
        CHECK_THAT(object_bottom, Catch::Matchers::WithinAbs(2.0, 1e-3));
        CHECK_THAT(object_bottom - support_top, Catch::Matchers::WithinAbs(0.30, 1e-3));
    }
    SECTION("four-layer raft rounds 0.35 mm to the next 0.10 mm object row") {
        DynamicPrintConfig config = coarse_base_support_config(MixedNozzleSlicingMode::FeatureSplit);
        config.set_key_value("raft_layers", new ConfigOptionInt(4));
        // The raft's base on the fine nozzle, so the first layer keeps its 0.10 mm height.
        // The 0.4 mm contact gap is more than one level of the fine nozzle's tower can carry.
        config.set_key_value("support_filament", new ConfigOptionInt(1));
        config.set_key_value("enable_prime_tower", new ConfigOptionBool(false));
        config.set_key_value("raft_contact_distance", new ConfigOptionFloat(0.35));
        config.set_key_value("support_top_z_distance", new ConfigOptionFloat(0.25));
        fill_per_filament_values(config);
        Model model;
        Print print;
        print.is_BBL_printer() = true;
        init_feature_flow_fixture(model, print, config);
        const CadenceTest::Facts facts = require_feature_slice(print);
        const double left = model.objects.front()->instance_bounding_box(0).min.x();
        const auto [raft_top, object_bottom] = emitted_gap(facts, left + 5., left + 15.);
        CAPTURE(raft_top, object_bottom);
        REQUIRE(raft_top > 0.);
        CHECK_THAT(object_bottom - raft_top, Catch::Matchers::WithinAbs(0.40, 1e-3));
    }
}
