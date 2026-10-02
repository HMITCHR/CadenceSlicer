#include <catch2/catch_all.hpp>

#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/TriangleSelector.hpp"
#include "rebuild_harness.hpp"
#include "srl_fixtures.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace srl_fixtures;

namespace {
using Move = GCodeProcessorResult::MoveVertex;

DynamicPrintConfig option_config()
{
    DynamicPrintConfig config = coupon_config(true);
    fill_per_filament_values(config);
    return config;
}

CadenceTest::Facts required(CadenceTest::Scene scene)
{
    CadenceTest::Facts facts = CadenceTest::slice(std::move(scene));
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    CHECK_FALSE(facts.emitted.plan_malformed);
    return facts;
}

CadenceTest::Scene two_bodies(DynamicPrintConfig config, TriangleMesh fine, TriangleMesh coarse)
{
    CadenceTest::Scene scene;
    scene.config = std::move(config);
    scene.populate = [fine = std::move(fine), coarse = std::move(coarse)]
        (Model &model, Print &print, const DynamicPrintConfig &cfg) mutable {
        init_native_coupon(model, print, cfg, std::move(fine), std::move(coarse));
    };
    return scene;
}

size_t roads(const CadenceTest::Facts &facts, ExtrusionRole role, unsigned tool)
{
    return std::count_if(facts.moves.begin(), facts.moves.end(), [=](const Move &move) {
        return move.type == EMoveType::Extrude && move.extrusion_role == role && move.physical_tool_id == tool;
    });
}

struct WallRun {
    unsigned tool = 0;
    double length = 0.;
    double speed = 0.;
};

std::vector<WallRun> wall_runs(const CadenceTest::Facts &facts)
{
    std::vector<WallRun> out;
    bool open = false;
    for (size_t i = 0; i < facts.moves.size(); ++i) {
        const Move &move = facts.moves[i];
        if (move.type != EMoveType::Extrude || move.extrusion_role != erExternalPerimeter ||
            move.position.z() <= .2f) {
            open = false;
            continue;
        }
        if (!open || out.back().tool != move.physical_tool_id) {
            out.push_back(WallRun{move.physical_tool_id, 0., 0.});
            open = true;
        }
        if (i > 0)
            out.back().length += double((move.position.head<2>() - facts.moves[i - 1].position.head<2>()).norm());
        out.back().speed = std::max(out.back().speed, double(move.feedrate));
    }
    return out;
}

std::vector<int> facing(const ModelVolume &volume, const Vec3f &direction)
{
    std::vector<int> facets;
    const indexed_triangle_set &its = volume.mesh().its;
    for (int i = 0; i < int(its.indices.size()); ++i)
        if (its_face_normal(its, i).dot(direction) > .95f)
            facets.push_back(i);
    return facets;
}

void paint(ModelVolume &volume, const std::vector<int> &facets, EnforcerBlockerType state,
           bool seam, bool support)
{
    TriangleSelector selector(volume.mesh());
    for (int facet : facets)
        selector.set_facet(facet, state);
    if (seam)
        volume.seam_facets.set_data(selector.serialize());
    if (support)
        volume.supported_facets.set_data(selector.serialize());
}

CadenceTest::Scene painted_bodies(DynamicPrintConfig config, TriangleMesh coarse,
                                  const std::function<void(ModelVolume &, ModelVolume &)> &paint_bodies)
{
    CadenceTest::Scene scene;
    scene.config = std::move(config);
    scene.populate = [coarse = std::move(coarse), paint_bodies]
        (Model &model, Print &print, const DynamicPrintConfig &cfg) mutable {
        ModelObject *object = model.add_object();
        ModelVolume *fine = object->add_volume(make_cube(10., 20., 2.), ModelVolumeType::MODEL_PART, false);
        fine->config.set_key_value("extruder", new ConfigOptionInt(1));
        fine->config.set_key_value("regional_layer_height", new ConfigOptionFloat(.1));
        ModelVolume *thick = object->add_volume(std::move(coarse), ModelVolumeType::MODEL_PART, false);
        thick->set_offset(Vec3d(20., 0., 0.));
        thick->config.set_key_value("extruder", new ConfigOptionInt(2));
        thick->config.set_key_value("regional_layer_height", new ConfigOptionFloat(.2));
        paint_bodies(*fine, *thick);
        object->add_instance();
        object->ensure_on_bed();
        print.apply(model, cfg);
        print.set_status_silent();
    };
    return scene;
}
} // namespace

TEST_CASE("Body Split slows a small fine loop while retaining long fine and coarse walls",
          "[TestRebuild][BodyOptions][BodySpeed]")
{
    DynamicPrintConfig config = option_config();
    config.set_key_value("outer_wall_speed", new ConfigOptionFloatsNullable{120., 120.});
    config.set_key_value("inner_wall_speed", new ConfigOptionFloatsNullable{150., 150.});
    config.set_key_value("small_perimeter_speed", new ConfigOptionFloatsOrPercentsNullable{
        FloatOrPercent(50., true), FloatOrPercent(50., true)});
    config.set_key_value("small_perimeter_threshold", new ConfigOptionFloatsNullable{0., 0.});
    config.set_key_value("filament_max_volumetric_speed", new ConfigOptionFloats{50., 50.});
    config.set_key_value("slow_down_for_layer_cooling", new ConfigOptionBools{false, false});
    fill_per_filament_values(config);
    TriangleMesh fine = make_cube(10., 20., 4.);
    TriangleMesh pillar = make_cube(2., 2., 4.);
    pillar.translate(4.f, 24.f, 0.f);
    fine.merge(pillar);
    const CadenceTest::Facts facts = required(two_bodies(config, fine, make_cube(20., 20., 4.)));

    double small_fine = 0., long_fine = 0., long_coarse = 0.;
    size_t small_count = 0;
    for (const WallRun &run : wall_runs(facts)) {
        if (run.tool == 0 && run.length > 3. && run.length < 15.) {
            ++small_count;
            small_fine = std::max(small_fine, run.speed);
        } else if (run.tool == 0 && run.length > 40.)
            long_fine = std::max(long_fine, run.speed);
        else if (run.tool == 1 && run.length > 40.)
            long_coarse = std::max(long_coarse, run.speed);
    }
    CAPTURE(small_count, small_fine, long_fine, long_coarse);
    REQUIRE(small_count > 0);
    REQUIRE(long_fine > 0.);
    REQUIRE(long_coarse > 0.);
    CHECK(small_fine < long_fine * .65);
    CHECK(std::abs(long_coarse - 120.) < 1.);
    CHECK(long_fine <= long_coarse + 1.);
}

TEST_CASE("Body Split classifies a real shelf overhang without slowing upright coarse walls",
          "[TestRebuild][BodyOptions][BodyOverhang]")
{
    DynamicPrintConfig config = option_config();
    config.set_key_value("detect_overhang_wall", new ConfigOptionBool(true));
    config.set_key_value("bridge_speed", new ConfigOptionFloatsNullable(2, 3.));
    config.set_key_value("enable_overhang_speed", new ConfigOptionBoolsNullable{true, true});
    config.set_key_value("slowdown_for_curled_perimeters", new ConfigOptionBoolsNullable{false, false});
    for (const char *key : {"overhang_1_4_speed", "overhang_2_4_speed", "overhang_3_4_speed", "overhang_4_4_speed"})
        config.set_key_value(key, new ConfigOptionFloatsOrPercentsNullable(2, FloatOrPercent{3., false}));
    config.set_key_value("outer_wall_speed", new ConfigOptionFloatsNullable(2, 40.));
    config.set_key_value("inner_wall_speed", new ConfigOptionFloatsNullable(2, 40.));
    config.set_key_value("slow_down_for_layer_cooling", new ConfigOptionBools{false, false});
    fill_per_filament_values(config);

    const CadenceTest::Facts upright = required(two_bodies(config, make_cube(10., 20., 2.), make_cube(20., 20., 2.)));
    const CadenceTest::Facts shelf = required(two_bodies(config, make_cube(10., 20., 2.),
                                                        overhang_shelf_mesh(12., 20., .6, 2.)));
    double upright_min = std::numeric_limits<double>::max();
    size_t upright_roads = 0, shelf_overhang = 0;
    double shelf_min = std::numeric_limits<double>::max();
    for (const Move &move : upright.moves)
        if (move.type == EMoveType::Extrude && move.physical_tool_id == 1 &&
            (move.extrusion_role == erExternalPerimeter || move.extrusion_role == erPerimeter) && move.position.z() > .3f) {
            ++upright_roads;
            upright_min = std::min(upright_min, double(move.feedrate));
        }
    for (const Move &move : shelf.moves)
        if (move.type == EMoveType::Extrude && move.physical_tool_id == 1 &&
            move.extrusion_role == erOverhangPerimeter) {
            ++shelf_overhang;
            shelf_min = std::min(shelf_min, double(move.feedrate));
        }
    CAPTURE(upright_roads, upright_min, shelf_overhang, shelf_min);
    CHECK(upright_roads > 0);
    CHECK(upright_min > 10.);
    CHECK(shelf_overhang > 0);
    CHECK(shelf_min < upright_min * .5);
    CHECK(roads(upright, erOverhangPerimeter, 1) == 0);
}

TEST_CASE("Fuzzy external walls and ironing retain each Body Split tool's emitted ownership",
          "[TestRebuild][BodyOptions][BodyParity]")
{
    DynamicPrintConfig plain = option_config();
    DynamicPrintConfig enabled = plain;
    enabled.set_key_value("fuzzy_skin", new ConfigOptionEnum<FuzzySkinType>(FuzzySkinType::External));
    enabled.set_key_value("fuzzy_skin_thickness", new ConfigOptionFloat(.2));
    enabled.set_key_value("fuzzy_skin_point_distance", new ConfigOptionFloat(.5));
    enabled.set_key_value("ironing_type", new ConfigOptionEnum<IroningType>(IroningType::TopSurfaces));
    enabled.set_key_value("filament_ironing_flow", new ConfigOptionPercentsNullable{
        ConfigOptionPercentsNullable::nil_value(), ConfigOptionPercentsNullable::nil_value()});
    for (const char *key : {"filament_ironing_spacing", "filament_ironing_inset", "filament_ironing_speed"})
        enabled.set_key_value(key, new ConfigOptionFloatsNullable{
            ConfigOptionFloatsNullable::nil_value(), ConfigOptionFloatsNullable::nil_value()});
    fill_per_filament_values(enabled);

    const CadenceTest::Facts control = required(two_bodies(plain, make_cube(10., 20., 2.), make_cube(20., 20., 2.)));
    const CadenceTest::Facts active = required(two_bodies(enabled, make_cube(10., 20., 2.), make_cube(20., 20., 2.)));
    for (unsigned tool : {0u, 1u}) {
        CAPTURE(tool);
        CHECK(roads(active, erExternalPerimeter, tool) > roads(control, erExternalPerimeter, tool) * 2);
        CHECK(roads(active, erPerimeter, tool) > 0);
        CHECK(roads(active, erIroning, tool) > 0);
        CHECK(roads(control, erIroning, tool) == 0);
    }
}

TEST_CASE("Scarf seams ramp on both Body Split wall heights in exported motion",
          "[TestRebuild][BodyOptions][BodyParity]")
{
    DynamicPrintConfig config = option_config();
    config.set_key_value("filament_flow_ratio", new ConfigOptionFloats{1., 1.});
    config.set_key_value("seam_slope_type", new ConfigOptionEnum<SeamScarfType>(SeamScarfType::External));
    config.set_key_value("seam_slope_conditional", new ConfigOptionBool(false));
    config.set_key_value("seam_slope_start_height", new ConfigOptionFloatOrPercent(0., false));
    config.set_key_value("seam_slope_entire_loop", new ConfigOptionBool(false));
    config.set_key_value("seam_slope_min_length", new ConfigOptionFloat(10.));
    config.set_key_value("seam_slope_steps", new ConfigOptionInt(10));
    const CadenceTest::Facts facts = required(two_bodies(config, make_cube(10., 20., 2.), make_cube(20., 20., 2.)));

    std::map<unsigned, float> layer_top;
    for (const Move &move : facts.moves)
        if (move.type == EMoveType::Extrude)
            layer_top[move.layer_id] = std::max(layer_top[move.layer_id], move.position.z());
    double depth[2] = {0., 0.};
    for (const Move &move : facts.moves)
        if (move.type == EMoveType::Extrude && move.extrusion_role == erExternalPerimeter && move.physical_tool_id < 2)
            depth[move.physical_tool_id] = std::max(depth[move.physical_tool_id],
                double(layer_top[move.layer_id] - move.position.z()));
    CAPTURE(depth[0], depth[1]);
    CHECK(depth[0] > .05);
    CHECK(depth[0] < .101);
    CHECK(depth[1] > .15);
    CHECK(depth[1] < .201);
}

TEST_CASE("Seam enforcers move both Body Split outer-wall starts to the painted face",
          "[TestRebuild][BodyOptions][PaintParity]")
{
    DynamicPrintConfig config = option_config();
    config.set_key_value("seam_position", new ConfigOptionEnum<SeamPosition>(spRear));
    config.set_key_value("filament_flow_ratio", new ConfigOptionFloats{1., 1.});
    const auto build = [&](bool painted) {
        return required(painted_bodies(config, make_cube(20., 20., 2.), [painted](ModelVolume &fine, ModelVolume &coarse) {
            if (!painted)
                return;
            for (ModelVolume *volume : {&fine, &coarse})
                paint(*volume, facing(*volume, Vec3f(0.f, -1.f, 0.f)),
                      EnforcerBlockerType::ENFORCER, true, false);
        }));
    };
    const CadenceTest::Facts plain = build(false);
    const CadenceTest::Facts marked = build(true);
    const auto starts = [](const std::string &gcode) {
        std::vector<double> y;
        std::string role;
        bool awaiting = false;
        GCodeReader reader;
        reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
            const std::string_view comment = line.comment();
            std::string next = role;
            if (comment.rfind(" FEATURE: ", 0) == 0)
                next.assign(comment.substr(10));
            else if (comment.rfind("TYPE:", 0) == 0)
                next.assign(comment.substr(5));
            if (next != role) {
                role = next;
                awaiting = role == "Outer wall";
            }
            if (awaiting && role == "Outer wall" && line.extruding(self) && line.dist_XY(self) > 0.) {
                y.push_back(self.y());
                awaiting = false;
            }
        });
        return y;
    };
    const std::vector<double> plain_y = starts(plain.gcode);
    const std::vector<double> marked_y = starts(marked.gcode);
    REQUIRE(plain_y.size() > 20);
    REQUIRE(marked_y.size() > 20);
    const double front = *std::min_element(marked_y.begin(), marked_y.end());
    const double back = *std::max_element(plain_y.begin(), plain_y.end());
    size_t marked_front = 0, plain_back = 0;
    for (double y : marked_y)
        marked_front += y < front + 1. ? 1 : 0;
    for (double y : plain_y)
        plain_back += y > back - 1. ? 1 : 0;
    CAPTURE(marked_front, marked_y.size(), plain_back, plain_y.size());
    CHECK(marked_front * 2 > marked_y.size());
    CHECK(plain_back * 2 > plain_y.size());
}

TEST_CASE("Support painting adds and blocks emitted support beneath a Body Split shelf",
          "[TestRebuild][BodyOptions][PaintParity][Support]")
{
    const auto build = [](SupportType type, std::optional<EnforcerBlockerType> state) {
        DynamicPrintConfig config = option_config();
        config.set_key_value("filament_flow_ratio", new ConfigOptionFloats{1., 1.});
        config.set_key_value("enable_support", new ConfigOptionBool(true));
        config.set_key_value("support_filament", new ConfigOptionInt(2));
        config.set_key_value("support_interface_filament", new ConfigOptionInt(1));
        config.set_key_value("support_type", new ConfigOptionEnum<SupportType>(type));
        config.set_key_value("independent_support_layer_height", new ConfigOptionBool(false));
        fill_per_filament_values(config);
        CadenceTest::Facts facts = required(painted_bodies(config, overhang_shelf_mesh(12., 20., .6, 2.),
            [state](ModelVolume &fine, ModelVolume &coarse) {
                if (!state)
                    return;
                if (*state == EnforcerBlockerType::ENFORCER) {
                    std::vector<int> underside;
                    const indexed_triangle_set &its = coarse.mesh().its;
                    for (int facet : facing(coarse, Vec3f(0.f, 0.f, -1.f)))
                        if (its.vertices[its.indices[facet](0)].z() > .3f)
                            underside.push_back(facet);
                    REQUIRE_FALSE(underside.empty());
                    paint(coarse, underside, *state, false, true);
                } else {
                    for (ModelVolume *volume : {&fine, &coarse}) {
                        std::vector<int> all(volume->mesh().its.indices.size());
                        std::iota(all.begin(), all.end(), 0);
                        paint(*volume, all, *state, false, true);
                    }
                }
            }));
        size_t support = 0;
        for (const Move &move : facts.moves)
            if (move.type == EMoveType::Extrude &&
                (move.extrusion_role == erSupportMaterial || move.extrusion_role == erSupportMaterialInterface ||
                 move.extrusion_role == erSupportTransition))
                ++support;
        return support;
    };
    const size_t manual_plain = build(stNormal, std::nullopt);
    const size_t manual_painted = build(stNormal, EnforcerBlockerType::ENFORCER);
    const size_t automatic = build(stNormalAuto, std::nullopt);
    const size_t automatic_blocked = build(stNormalAuto, EnforcerBlockerType::BLOCKER);
    CAPTURE(manual_plain, manual_painted, automatic, automatic_blocked);
    CHECK(manual_plain == 0);
    CHECK(manual_painted > 0);
    CHECK(automatic > 0);
    CHECK(automatic_blocked == 0);
}
