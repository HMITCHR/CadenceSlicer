#include <catch2/catch_all.hpp>

#include "mixed_nozzle_harness.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/Support/SupportLayer.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

using namespace Slic3r;
using namespace mixed_nozzle_fixtures;

namespace {
using Move = GCodeProcessorResult::MoveVertex;

enum class Mode { Off, Feature, Body };

const char *mode_name(Mode mode)
{
    return mode == Mode::Off ? "Off" : mode == Mode::Feature ? "Feature Split" : "Body Split";
}

struct SupportCase {
    const char          *name;
    SupportType          type;
    SupportMaterialStyle style;
};

// Filaments 1 and 3 sit on the left 0.2 nozzle, 2 and 4 on the right 0.8 nozzle.
struct FilamentCase {
    const char *name;
    int         base;
    int         interface_filament; // 0 = Default
    const char *interface_type = "PLA";
};

const std::vector<int> k_filament_map{1, 2, 1, 2};

const SupportCase k_organic{"tree organic", stTreeAuto, smsTreeOrganic};
const SupportCase k_hybrid{"tree hybrid", stTreeAuto, smsTreeHybrid};
const SupportCase k_slim{"tree slim", stTreeAuto, smsTreeSlim};
const SupportCase k_strong{"tree strong", stTreeAuto, smsTreeStrong};
const SupportCase k_normal{"normal auto", stNormalAuto, smsDefault};

struct CoarseNozzle { double coarse, coarse_min, coarse_max; };

bool is_support(const Move &move)
{
    return move.type == EMoveType::Extrude &&
        (move.extrusion_role == erSupportMaterial || move.extrusion_role == erSupportMaterialInterface);
}

double point_to_segment(const Vec2d &p, const Vec2d &a, const Vec2d &b)
{
    const Vec2d d = b - a;
    const double t = d.squaredNorm() > 0. ? std::clamp((p - a).dot(d) / d.squaredNorm(), 0., 1.) : 0.;
    return (p - (a + t * d)).norm();
}

// Checks that every case of a scene passed, listing the failures one per line.
void check_no_failures(const std::vector<std::string> &failures)
{
    std::ostringstream out;
    for (const std::string &failure : failures)
        if (!failure.empty())
            out << failure << '\n';
    INFO(out.str());
    CHECK(out.str().empty());
}

// The first refusal code ([SRL-...] or [MNS-...]) in a message, without the brackets.
std::string code_of(const std::string &message)
{
    size_t at = std::string::npos;
    for (const char *prefix : {"[SRL-", "[MNS-"})
        at = std::min(at, message.find(prefix));
    const size_t end = at == std::string::npos ? at : message.find(']', at);
    return end == std::string::npos ? std::string() : message.substr(at + 1, end - at - 1);
}

void set_nozzles(DynamicPrintConfig &config, double fine_d, double coarse_d, double fine_min, double fine_max,
                 double coarse_min, double coarse_max)
{
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{fine_d, coarse_d});
    config.set_key_value("mixed_nozzle_process_nozzle_diameters", new ConfigOptionFloats{fine_d, coarse_d});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{fine_min, coarse_min});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{fine_max, coarse_max});
}

// Line widths for the 0.2: 0.22 roads, a 0.25 first layer.
void set_fine_widths(DynamicPrintConfig &config)
{
    config.set_key_value("line_width", new ConfigOptionFloatOrPercent(0.22, false));
    config.set_key_value("initial_layer_line_width", new ConfigOptionFloatOrPercent(0.25, false));
    config.set_key_value("support_line_width", new ConfigOptionFloatOrPercent(0.22, false));
}

// Every model role on the object's own filament.
void single_filament_roles(DynamicPrintConfig &config)
{
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id", "internal_solid_filament_id",
                            "top_surface_filament_id", "bottom_surface_filament_id", "sparse_infill_filament_id"})
        config.set_key_value(key, new ConfigOptionInt(0));
}

// What CadenceTest::slice(Scene) does to a config, for scenes sliced on a Print the test keeps.
void bambu_config(DynamicPrintConfig &config)
{
    fill_per_filament_values(config);
    if (config.opt_string("machine_start_gcode").find("M1020") == std::string::npos)
        config.set_key_value("machine_start_gcode", new ConfigOptionString(config.opt_string("machine_start_gcode") + "\nM1020 S[initial_extruder]\n"));
}

ModelObject *add_object(Model &model, const char *name, TriangleMesh mesh, const Vec3d &offset)
{
    ModelObject *object = model.add_object();
    object->name = name;
    object->add_volume(std::move(mesh), ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    object->instances.front()->set_offset(offset);
    object->ensure_on_bed();
    return object;
}

void add_part(ModelObject &object, TriangleMesh mesh, int filament, double height)
{
    ModelVolume *volume = object.add_volume(std::move(mesh), ModelVolumeType::MODEL_PART, false);
    volume->config.set_key_value("extruder", new ConfigOptionInt(filament));
    volume->config.set_key_value("regional_layer_height", new ConfigOptionFloat(height));
}

// A populate step that puts one object on the plate.
std::function<void(Model &, Print &, const DynamicPrintConfig &)> one_object(const char *name, const TriangleMesh &mesh, const Vec3d &offset)
{
    return [name, mesh, offset](Model &model, Print &print, const DynamicPrintConfig &config) {
        add_object(model, name, mesh, offset);
        print.apply(model, config);
        print.set_status_silent();
    };
}

// Slices a Print or a Scene; returns `what` and the refusal or the throw, or an empty string with the facts filled in.
template<class Source> std::string slice_into(Source &source, const std::string &what, CadenceTest::Facts &facts)
{
    try {
        facts = CadenceTest::slice(source);
    } catch (const std::exception &error) {
        return what + "threw " + error.what();
    }
    if (!facts.refusal.string.empty())
        return what + "refused: " + facts.refusal.string;
    return facts.gcode.empty() ? what + "no G-code" : std::string();
}

// A column with a low arm on the right and a high arm on the left. The left arm's support
// rises past the right arm's contact, so its body bands span another island's top contact.
TriangleMesh two_arm_mesh()
{
    return extruded_xz_outline({{0., 0.}, {8., 0.}, {8., 3.}, {16., 3.}, {16., 4.2}, {8., 4.2},
                                {8., 8.}, {-8., 8.}, {-8., 6.6}, {0., 6.6}},
                               12.);
}

// A column with a shelf on the right whose top is 0.1 mm above the underside of an arm on the left,
// and a second arm over the shelf. The left arm's support top and the bottom of the support standing
// on the shelf are closer together than the coarse nozzle's thinnest layer.
TriangleMesh shelf_and_arms_mesh()
{
    return extruded_xz_outline({{0., 0.}, {20., 0.}, {20., 10.1}, {8., 10.1}, {8., 13.}, {20., 13.}, {20., 14.},
                                {0., 14.}, {0., 11.}, {-12., 11.}, {-12., 10.}, {0., 10.}},
                               12.);
}

DynamicPrintConfig h2d_support_config(Mode mode, double fine, double coarse, int ratio, double top_z,
                                      const SupportCase &support, const FilamentCase &filaments)
{
    DynamicPrintConfig config = coarse_base_support_config(MixedNozzleSlicingMode::FeatureSplit);
    config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(
        mode == Mode::Off ? MixedNozzleSlicingMode::Off :
        mode == Mode::Feature ? MixedNozzleSlicingMode::FeatureSplit : MixedNozzleSlicingMode::BodySplit));
    config.set_key_value("printable_area", new ConfigOptionPoints{Vec2d(0., 0.), Vec2d(320., 0.), Vec2d(320., 320.), Vec2d(0., 320.)});
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, 0.8});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.16});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.56});
    config.set_key_value("layer_height", new ConfigOptionFloat(fine));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(fine));
    config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(coarse));
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{ratio});
    config.set_key_value("mixed_nozzle_process_nozzle_diameters", new ConfigOptionFloats{0.2, 0.8});

    config.set_key_value("filament_diameter", new ConfigOptionFloats(4, 1.75));
    config.set_key_value("filament_map", new ConfigOptionInts(k_filament_map));
    config.set_key_value("flush_volumes_matrix", new ConfigOptionFloats(32, 0.));
    config.set_key_value("filament_colour", new ConfigOptionStrings{"#FF0000", "#0000FF", "#00FF00", "#FFFF00"});
    config.set_key_value("filament_self_index", new ConfigOptionInts{1, 2, 3, 4});
    config.set_key_value("filament_volume_map", new ConfigOptionInts(4, 0));
    config.set_key_value("filament_start_gcode", new ConfigOptionStrings(4, ""));
    config.set_key_value("filament_end_gcode", new ConfigOptionStrings(4, ""));
    config.set_key_value("filament_type", new ConfigOptionStrings(4, "PLA"));
    if (filaments.interface_filament > 0)
        config.option<ConfigOptionStrings>("filament_type")->values[size_t(filaments.interface_filament - 1)] = filaments.interface_type;
    config.set_key_value("filament_adhesiveness_category", new ConfigOptionInts(4, 100));
    config.set_key_value("filament_soluble", new ConfigOptionBools(4, false));
    config.set_key_value("filament_is_support", new ConfigOptionBools(4, false));

    // Model roles: fine filament 1; Feature Split sends sparse infill to the coarse nozzle, Body
    // Split takes each part's own filament.
    const int role_filament = mode == Mode::Body ? 0 : 1;
    for (const char *key : {"outer_wall_filament_id", "inner_wall_filament_id", "internal_solid_filament_id",
                            "top_surface_filament_id", "bottom_surface_filament_id"})
        config.set_key_value(key, new ConfigOptionInt(role_filament));
    config.set_key_value("sparse_infill_filament_id", new ConfigOptionInt(mode == Mode::Feature ? 2 : role_filament));
    config.set_key_value("sparse_infill_density", new ConfigOptionPercent(15.));

    config.set_key_value("enable_support", new ConfigOptionBool(true));
    config.set_key_value("support_type", new ConfigOptionEnum<SupportType>(support.type));
    config.set_key_value("support_style", new ConfigOptionEnum<SupportMaterialStyle>(support.style));
    config.set_key_value("support_filament", new ConfigOptionInt(filaments.base));
    config.set_key_value("support_interface_filament", new ConfigOptionInt(filaments.interface_filament));
    config.set_key_value("support_top_z_distance", new ConfigOptionFloat(top_z));
    config.set_key_value("support_bottom_z_distance", new ConfigOptionFloat(top_z));
    config.set_key_value("support_interface_top_layers", new ConfigOptionInt(2));
    config.set_key_value("support_interface_bottom_layers", new ConfigOptionInt(0));
    // Two walls on every tree branch, as tall or thick trees get, so tree support writes collections.
    config.set_key_value("tree_support_wall_count", new ConfigOptionInt(2));
    fill_per_filament_values(config);
    return config;
}

void load_two_arm_model(Model &model, Print &print, const DynamicPrintConfig &config, Mode mode, double fine, double coarse)
{
    ModelObject *object = model.add_object();
    object->name = "two-arm-overhang";
    ModelVolume *part = object->add_volume(two_arm_mesh(), ModelVolumeType::MODEL_PART, false);
    if (mode == Mode::Body) {
        part->config.set_key_value("extruder", new ConfigOptionInt(1));
        part->config.set_key_value("regional_layer_height", new ConfigOptionFloat(fine));
        ModelVolume *body = object->add_volume(make_cube(8., 12., 6.), ModelVolumeType::MODEL_PART, false);
        body->set_offset(Vec3d(24., 0., 0.));
        body->config.set_key_value("extruder", new ConfigOptionInt(2));
        body->config.set_key_value("regional_layer_height", new ConfigOptionFloat(coarse));
    }
    object->add_instance();
    object->instances.front()->set_offset(Vec3d(60., 60., 0.));
    object->ensure_on_bed();
    print.apply(model, config);
    print.set_status_silent();
}

// The prime tower as the automatic tower setup leaves it: auto-sized pad under a 35 mm width cap.
void set_automatic_tower(DynamicPrintConfig &config)
{
    config.set_key_value("enable_prime_tower", new ConfigOptionBool(true));
    config.set_key_value("wipe_tower_no_sparse_layers", new ConfigOptionBool(true));
    config.set_key_value("mixed_nozzle_auto_pad", new ConfigOptionBool(true));
    config.set_key_value("prime_tower_width", new ConfigOptionFloat(35.));
    config.set_key_value("wipe_tower_wall_type", new ConfigOptionEnum<WipeTowerWallType>(wtwRib));
    config.set_key_value("wipe_tower_rib_width", new ConfigOptionFloat(8.));
    config.set_key_value("prime_tower_brim_width", new ConfigOptionFloat(-1.));
    config.set_key_value("prime_volume_mode", new ConfigOptionEnum<PrimeVolumeMode>(pvmDefault));
    config.set_key_value("wipe_tower_x", new ConfigOptionFloats{200.});
    config.set_key_value("wipe_tower_y", new ConfigOptionFloats{200.});
}

// The project as the app saved it (fine PLA 1 and PETG 2 on the 0.2, coarse PLA 3 on the 0.8, support base 3 and
// interface 2), with the automatic tower put back.
DynamicPrintConfig app_three_filament_config(Mode mode, double fine, double coarse, int ratio, const SupportCase &support)
{
    DynamicPrintConfig config;
    config.load_from_ini(std::string(TEST_DATA_DIR) + "/h2d-three-filament.ini", ForwardCompatibilitySubstitutionRule::Enable);
    REQUIRE(config.has("printable_area"));
    config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(
        mode == Mode::Off ? MixedNozzleSlicingMode::Off : MixedNozzleSlicingMode::FeatureSplit));
    config.set_key_value("layer_height", new ConfigOptionFloat(fine));
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(fine));
    config.set_key_value("mixed_nozzle_coarse_layer_height", new ConfigOptionFloat(coarse));
    config.set_key_value("mixed_nozzle_allowed_cadence_ratios", new ConfigOptionInts{ratio});
    config.set_key_value("support_type", new ConfigOptionEnum<SupportType>(support.type));
    config.set_key_value("support_style", new ConfigOptionEnum<SupportMaterialStyle>(support.style));
    config.set_key_value("wipe_tower_wall_type", new ConfigOptionEnum<WipeTowerWallType>(wtwRib));
    config.set_key_value("mixed_nozzle_auto_pad", new ConfigOptionBool(true));
    config.set_key_value("prime_tower_width", new ConfigOptionFloat(35.));
    return config;
}

std::function<void(ModelObject&)> handy_model(const char *file)
{
    return [file](ModelObject &object) {
        Model loaded = Model::read_from_file(std::string(PROFILES_DIR) + "/../handy_models/" + file);
        for (const ModelObject *source : loaded.objects)
            for (const ModelVolume *volume : source->volumes)
                object.add_volume(*volume);
        object.add_instance();
        // The bunny, at the size it imports into the app.
        if (object.instance_bounding_box(0).size().z() > 118.)
            object.scale_to_fit(Vec3d(120., 120., 118.));
    };
}

std::function<void(ModelObject&)> parts_of(std::vector<TriangleMesh> (*make)())
{
    return [make](ModelObject &object) {
        for (TriangleMesh &mesh : make())
            object.add_volume(std::move(mesh), ModelVolumeType::MODEL_PART, false);
    };
}

std::function<void(ModelObject&)> part_of(TriangleMesh (*make)())
{
    return [make](ModelObject &object) { object.add_volume(make(), ModelVolumeType::MODEL_PART, false); };
}

// A plate on the bed, a column, and an arm over the plate: support that stands on the part.
TriangleMesh c_shape_mesh()
{
    return extruded_xz_outline({{0., 0.}, {40., 0.}, {40., 6.}, {15., 6.}, {15., 16.}, {40., 16.}, {40., 20.}, {0., 20.}}, 20.);
}

// A ball on the bed, like the underside of a hull or a bunny: a small first layer and support rising
// from the bed under a curve.
TriangleMesh hull_mesh()
{
    TriangleMesh mesh = make_sphere(15., PI / 24.);
    mesh.scale(Vec3f(1.6f, 1.f, 1.f));
    return mesh;
}

// A bunny-like part: a ball on a foot to one side, its underside curving down to 0.32 mm over the
// bed, so trees stand on the bed and meet the curve a few layers up.
std::vector<TriangleMesh> bunny_like_parts()
{
    TriangleMesh ball = make_sphere(12., PI / 24.);
    ball.scale(Vec3f(1.4f, 1.f, 1.f));
    ball.translate(0.f, 0.f, 12.32f);
    TriangleMesh foot = make_cylinder(3., 3., PI / 24.);
    foot.translate(10.f, 0.f, 0.f);
    return {std::move(foot), std::move(ball)};
}

// A Benchy-like hull bottom: a block whose underside has a 7 x 5 mm pocket 0.3 mm deep, with a deeper
// 3 x 2 mm recess in its middle up to 0.8 mm, so trees stand on the bed inside the pocket and meet a
// contact one layer above the first band.
std::vector<TriangleMesh> benchy_like_parts()
{
    std::vector<TriangleMesh> parts;
    const auto box = [&parts](double x, double y, double z, double dx, double dy, double dz) {
        TriangleMesh mesh = make_cube(dx, dy, dz);
        mesh.translate(float(x), float(y), float(z));
        parts.push_back(std::move(mesh));
    };
    // A 30 x 20 block with a frame around the pocket, then a frame around the recess.
    const auto frame = [&box](double x0, double y0, double w, double h, double outer_w, double outer_h, double z, double dz) {
        box(x0 - (outer_w - w) / 2., y0 - (outer_h - h) / 2., z, outer_w, (outer_h - h) / 2., dz);
        box(x0 - (outer_w - w) / 2., y0 + h, z, outer_w, (outer_h - h) / 2., dz);
        box(x0 - (outer_w - w) / 2., y0, z, (outer_w - w) / 2., h, dz);
        box(x0 + w, y0, z, (outer_w - w) / 2., h, dz);
    };
    frame(11.5, 7.5, 7., 5., 30., 20., 0., 0.8);
    frame(13.5, 9., 3., 2., 7., 5., 0.3, 0.5);
    box(0., 0., 0.8, 30., 20., 8.);
    return parts;
}

// ---- Tower G-code audits -------------------------------------------------------------------------

// One G-code move, or a tool change (T line) after the first layer change: start and end (an arc in pieces
// of about 0.5 mm), Z before and after, E, the tower block and the purge or ramming row it is in, whether it
// is under "; FEATURE: Prime tower", and the part layer (the last "; Z_HEIGHT:").
struct GMove {
    std::vector<Vec2d> points;
    double z0 = 0., z = 0., e = 0., height = 0.2, part_z = 0.;
    bool   tool_change = false, has_xy = false, has_z = false, in_tower = false, in_change = false, prime = false;
    const char *row = nullptr;
    size_t row_id = 0, layer = 0, line = 0;

    double length() const
    {
        double out = 0.;
        for (size_t k = 1; k < points.size(); ++k)
            out += (points[k] - points[k - 1]).norm();
        return out;
    }
};

// Calls visit for every move up to the end G-code.
void walk_gcode(const std::string &gcode, const std::function<void(const GMove &)> &visit)
{
    std::istringstream lines(gcode);
    std::string line;
    double x = 0., y = 0., z = 0., height = 0.2, part_z = 0.;
    bool in_tower = false, in_change = false, prime = false, purge = false, ramming = false, relative = false;
    size_t layer = 0, row_id = 0, number = 0;
    while (std::getline(lines, line)) {
        ++number;
        const auto tag = [&line](const char *prefix) { return line.rfind(prefix, 0) == 0; };
        if (tag("; MACHINE_END_GCODE_START"))
            break;
        if (tag("; CHANGE_LAYER")) {
            ++layer;
            prime = false;
        } else if (tag("; Z_HEIGHT:"))
            part_z = std::atof(line.c_str() + 11);
        else if (tag("; FEATURE:"))
            prime = line.find("Prime tower") != std::string::npos;
        else if (tag("; WIPE_TOWER_START"))
            in_tower = true;
        else if (tag("; WIPE_TOWER_END"))
            in_tower = purge = ramming = false;
        else if (tag("; CP_TOOLCHANGE_START"))
            in_change = true;
        else if (tag("; CP_TOOLCHANGE_END"))
            in_change = false;
        else if (tag("; CP_TOOLCHANGE_WIPE")) {
            purge = true;
            ++row_id;
        } else if (tag("; CP TOOLCHANGE END"))
            purge = false;
        else if (tag("; NOZZLE_CHANGE_START")) {
            ramming = true;
            ++row_id;
        } else if (tag("; NOZZLE_CHANGE_END"))
            ramming = false;
        else if (tag("; LAYER_HEIGHT:"))
            height = std::atof(line.c_str() + 15);
        else if (tag(";HEIGHT:"))
            height = std::atof(line.c_str() + 8);
        GMove move;
        move.height = height;
        move.part_z = part_z;
        move.in_tower = in_tower;
        move.in_change = in_change;
        move.prime = prime;
        move.row = purge ? "purge" : ramming ? "ramming" : nullptr;
        move.row_id = row_id;
        move.layer = layer;
        move.line = number;
        move.z0 = move.z = z;
        if (layer > 0 && line.size() > 1 && line[0] == 'T' && std::isdigit((unsigned char) line[1]) && std::atoi(line.c_str() + 1) < 255) {
            move.tool_change = true;
            move.points = {Vec2d(x, y)};
            visit(move);
            continue;
        }
        const std::string code = line.substr(0, line.find(';'));
        if (code.rfind("G90", 0) == 0 && (code.size() == 3 || code[3] == ' '))
            relative = false;
        if (code.rfind("G91", 0) == 0 && (code.size() == 3 || code[3] == ' '))
            relative = true;
        if (code.size() < 3 || code[0] != 'G' || code[1] < '0' || code[1] > '3' || code[2] != ' ')
            continue;
        double nx = x, ny = y, ci = 0., cj = 0.;
        std::istringstream words(code.substr(3));
        for (std::string word; words >> word;) {
            if (word.size() < 2)
                continue;
            const double value = std::atof(word.c_str() + 1);
            switch (word[0]) {
            case 'X': nx = relative ? x + value : value; move.has_xy = true; break;
            case 'Y': ny = relative ? y + value : value; move.has_xy = true; break;
            case 'Z': move.z = relative ? z + value : value; move.has_z = true; break;
            case 'E': move.e = value; break;
            case 'I': ci = value; break;
            case 'J': cj = value; break;
            default: break;
            }
        }
        move.points = {Vec2d(x, y), Vec2d(nx, ny)};
        // An arc (G2 clockwise, G3 counter-clockwise, the rib wall's corners) in pieces of about 0.5 mm.
        if ((code[1] == '2' || code[1] == '3') && (ci != 0. || cj != 0.)) {
            const Vec2d center(x + ci, y + cj);
            const double radius = std::hypot(ci, cj);
            const double a0 = std::atan2(y - center.y(), x - center.x());
            double a1 = std::atan2(ny - center.y(), nx - center.x());
            if (code[1] == '3')
                while (a1 <= a0) a1 += 2. * M_PI;
            else
                while (a1 >= a0) a1 -= 2. * M_PI;
            const int pieces = std::max(2, int(std::abs(a1 - a0) * radius / 0.5) + 1);
            move.points.clear();
            for (int k = 0; k <= pieces; ++k) {
                const double a = a0 + (a1 - a0) * k / pieces;
                move.points.emplace_back(center.x() + radius * std::cos(a), center.y() + radius * std::sin(a));
            }
        }
        visit(move);
        x = nx;
        y = ny;
        z = move.z;
    }
}

// Empty when the tower never stands more than one coarse minimum layer above the part layer it is laid with. The
// tower may run ahead of the part only by the steps its own rules allow: an outgoing tool rams at no less than its
// nozzle's minimum layer height, and right above the bed level a coarse arrival lays a full minimum step.
std::string tower_ahead_failure(const std::string &gcode, double coarse_min)
{
    double most = 0., part_z = 0., tower_z = 0.;
    walk_gcode(gcode, [&](const GMove &move) {
        if (move.in_tower && move.e > 0. && move.z - move.part_z > most) {
            most    = move.z - move.part_z;
            part_z  = move.part_z;
            tower_z = move.z;
        }
    });
    if (most <= coarse_min + 1e-3)
        return {};
    std::ostringstream out;
    out << "tower extrusion at Z " << tower_z << " for part Z " << part_z << ", " << most << " mm above the part (allowed "
        << coarse_min << ")";
    return out.str();
}

// Empty when no move outside the tower blocks comes down over the tower's footprint more than allowed below the
// highest tower road laid so far: the nozzle going back to the part layer would hit a tower standing above it.
std::string tower_dive_failure(const std::string &gcode, double allowed)
{
    BoundingBoxf footprint;
    walk_gcode(gcode, [&footprint](const GMove &move) {
        if (move.in_tower && move.e > 0.)
            footprint.merge(move.points.back());
    });
    if (!footprint.defined)
        return {};
    double top = 0.;
    std::string failure;
    walk_gcode(gcode, [&](const GMove &move) {
        if (!failure.empty() || move.tool_change)
            return;
        if (move.in_tower) {
            if (move.e > 0.)
                top = std::max(top, move.z);
        } else if (footprint.contains(move.points.back()) && move.z < top - allowed - 1e-3) {
            std::ostringstream out;
            out << "a move at X " << move.points.back().x() << " Y " << move.points.back().y() << " comes down to Z " << move.z
                << " over the tower, whose top is at " << top << " (allowed " << allowed << " below it)";
            failure = out.str();
        }
    });
    return failure;
}

// The tower's roads, each arc piece on its own, with the width its E, length and tagged height give (1.75 mm filament).
struct TowerRoad { Vec2d a, b; double z, width; const char *row; size_t line; };

std::vector<TowerRoad> tower_roads(const std::string &gcode)
{
    const double filament_area = 0.25 * M_PI * 1.75 * 1.75;
    std::vector<TowerRoad> roads;
    walk_gcode(gcode, [&](const GMove &move) {
        const double length = move.length();
        if (!move.in_tower || move.e <= 0. || length <= 0.05 || move.height <= 0.)
            return;
        const double width = move.e * filament_area / (length * move.height) + move.height * (1. - 0.25 * M_PI);
        for (size_t k = 1; k < move.points.size(); ++k)
            roads.push_back({move.points[k - 1], move.points[k], move.z, width, move.row, move.line});
    });
    return roads;
}

// The 2 mm grid cells a road's bounding box, grown by reach, covers.
std::set<std::pair<int, int>> cells_of(const TowerRoad &road, double reach)
{
    std::set<std::pair<int, int>> cells;
    for (int gx = int(std::floor((std::min(road.a.x(), road.b.x()) - reach) / 2.)); gx <= int(std::floor((std::max(road.a.x(), road.b.x()) + reach) / 2.)); ++gx)
        for (int gy = int(std::floor((std::min(road.a.y(), road.b.y()) - reach) / 2.)); gy <= int(std::floor((std::max(road.a.y(), road.b.y()) + reach) / 2.)); ++gy)
            cells.insert({gx, gy});
    return cells;
}

// Empty when no tower road crosses an earlier purge or ramming row that stands more than 0.02 mm higher: the nozzle
// would be pushed through deposited plastic. Roads are compared by their bounding boxes, widened by 0.2 mm. A ramming
// row stands higher when the outgoing tool rams at its own minimum above a lower level.
std::string tower_plough_failure(const std::vector<TowerRoad> &roads)
{
    std::map<std::pair<int, int>, std::vector<size_t>> grid;
    for (size_t i = 0; i < roads.size(); ++i) {
        const TowerRoad &road = roads[i];
        const std::set<std::pair<int, int>> cells = cells_of(road, 0.);
        for (const auto &cell : cells)
            for (size_t j : grid[cell]) {
                const TowerRoad &earlier = roads[j];
                if (earlier.row != nullptr && earlier.z > road.z + 0.02 &&
                    std::max(earlier.a.x(), earlier.b.x()) >= std::min(road.a.x(), road.b.x()) - 0.2 &&
                    std::min(earlier.a.x(), earlier.b.x()) <= std::max(road.a.x(), road.b.x()) + 0.2 &&
                    std::max(earlier.a.y(), earlier.b.y()) >= std::min(road.a.y(), road.b.y()) - 0.2 &&
                    std::min(earlier.a.y(), earlier.b.y()) <= std::max(road.a.y(), road.b.y()) + 0.2) {
                    std::ostringstream out;
                    out << "a tower road at Z " << road.z << " crosses an earlier " << earlier.row << " row at Z " << earlier.z;
                    return out.str();
                }
            }
        for (const auto &cell : cells)
            grid[cell].push_back(i);
    }
    return {};
}

// Empty when no tower road touches an earlier purge or ramming row that stands more than 0.02 mm higher: their edges
// must keep at least min_gap apart, or the nozzle laying the lower road rubs the side of the higher row.
std::string tower_touch_failure(const std::vector<TowerRoad> &roads, double min_gap)
{
    const auto distance = [](const TowerRoad &r, const TowerRoad &s) {
        const auto side = [](const Vec2d &o, const Vec2d &p, const Vec2d &q) {
            return (p.x() - o.x()) * (q.y() - o.y()) - (p.y() - o.y()) * (q.x() - o.x());
        };
        if (side(s.a, s.b, r.a) * side(s.a, s.b, r.b) < 0. && side(r.a, r.b, s.a) * side(r.a, r.b, s.b) < 0.)
            return 0.;
        return std::min({point_to_segment(r.a, s.a, s.b), point_to_segment(r.b, s.a, s.b), point_to_segment(s.a, r.a, r.b),
                         point_to_segment(s.b, r.a, r.b)});
    };
    // Grid of 2 mm cells over the held rows laid so far: only purge and ramming rows count as earlier roads.
    std::map<std::pair<int, int>, std::vector<size_t>> grid;
    for (size_t i = 0; i < roads.size(); ++i) {
        const TowerRoad &road = roads[i];
        for (const auto &cell : cells_of(road, 1.)) {
            const auto found = grid.find(cell);
            if (found == grid.end())
                continue;
            for (size_t j : found->second) {
                const TowerRoad &earlier = roads[j];
                if (earlier.z <= road.z + 0.02)
                    continue;
                if (const double gap = distance(road, earlier) - 0.5 * (road.width + earlier.width); gap < min_gap) {
                    std::ostringstream out;
                    out << "a tower road at Z " << road.z << " (G-code line " << road.line << ") comes within " << gap
                        << " mm of an earlier " << earlier.row << " row at Z " << earlier.z << " (line " << earlier.line << ")";
                    return out.str();
                }
            }
        }
        if (road.row != nullptr)
            for (const auto &cell : cells_of(road, 0.))
                grid[cell].push_back(i);
    }
    return {};
}

// Rows (rammings and purges) held above their level: a Z step of 0.02 to 0.1 mm up inside the row.
size_t held_rows(const std::string &gcode)
{
    size_t held = 0, last = 0;
    walk_gcode(gcode, [&](const GMove &move) {
        if (move.row != nullptr && move.has_z && move.z > move.z0 + 0.02 && move.z < move.z0 + 0.1 && move.row_id != last) {
            ++held;
            last = move.row_id;
        }
    });
    return held;
}

// Empty when the plate has held rows and no tower road runs under or beside one.
std::string held_rows_failure(const std::string &gcode)
{
    const size_t held = held_rows(gcode);
    if (held == 0)
        return "no row held above its level";
    const std::vector<TowerRoad> roads = tower_roads(gcode);
    std::string failure = tower_plough_failure(roads);
    if (failure.empty())
        failure = tower_touch_failure(roads, 0.02);
    return failure.empty() ? failure : std::to_string(held) + " held rows, and " + failure;
}
} // namespace

// ---- Support matrix ---------------------------------------------------------------------------------

namespace {
enum class Shape { TwoArm, Shelf };

struct MatrixCase {
    Mode         mode;
    double       fine, coarse;
    int          ratio;
    double       top_z;
    SupportCase  support;
    FilamentCase filaments;
    Shape        shape = Shape::TwoArm;
    // A 5 mm outer brim, tallied against the first-layer support.
    bool         brim = false;
    // The PETG interface prints only at the arms' contacts, over a dense fine PLA layer.
    bool         petg_contacts = false;
};

struct Outcome { std::string code, opt_key, failure; };

std::string label(const MatrixCase &c)
{
    std::ostringstream out;
    out << mode_name(c.mode) << " / " << c.support.name << " / " << c.filaments.name << " " << c.filaments.interface_type
        << " / " << c.fine << "/" << c.coarse << " gap " << c.top_z << (c.shape == Shape::Shelf ? " / shelf" : "");
    return out.str();
}

// Top of the support under the model inside [x_lo, x_hi], the model's underside there, and the
// role of the topmost support road.
struct Gap {
    double       support_top   = 0.;
    double       object_bottom = std::numeric_limits<double>::max();
    ExtrusionRole top_role     = erNone;
};

Gap support_gap(const std::vector<Move> &moves, double x_lo, double x_hi)
{
    Gap gap;
    for (const Move &move : moves)
        if (CadenceTest::model_road(move) && move.position.x() > x_lo && move.position.x() < x_hi)
            gap.object_bottom = std::min(gap.object_bottom, double(move.position.z()) - double(move.height));
    for (const Move &move : moves)
        if (is_support(move) && move.position.x() > x_lo && move.position.x() < x_hi &&
            double(move.position.z()) < gap.object_bottom + 1e-4 &&
            double(move.position.z()) > gap.support_top + 1e-4) {
            gap.support_top = double(move.position.z());
            gap.top_role    = move.extrusion_role;
        }
    return gap;
}

void collect_paths(const ExtrusionEntity *entity, std::vector<const ExtrusionPath*> &out)
{
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection*>(entity)) {
        for (const ExtrusionEntity *child : collection->entities)
            collect_paths(child, out);
    } else if (const auto *path = dynamic_cast<const ExtrusionPath*>(entity))
        out.push_back(path);
    else if (const auto *multipath = dynamic_cast<const ExtrusionMultiPath*>(entity)) {
        for (const ExtrusionPath &p : multipath->paths)
            out.push_back(&p);
    } else if (const auto *loop = dynamic_cast<const ExtrusionLoop*>(entity)) {
        for (const ExtrusionPath &p : loop->paths)
            out.push_back(&p);
    }
}

// Support roads whose height reaches down (or up) into a layer of the part at the same place.
std::string support_into_part(const PrintObject &object)
{
    std::ostringstream what;
    size_t count = 0;
    for (const SupportLayer *support : object.support_layers()) {
        std::vector<const ExtrusionPath*> paths;
        collect_paths(&support->support_fills, paths);
        for (const ExtrusionPath *path : paths) {
            const double top = support->print_z, bottom = top - double(path->height);
            for (const Layer *layer : object.layers()) {
                if (layer->print_z < bottom + 1e-3 || layer->print_z - layer->height > top - 1e-3)
                    continue;
                const ExPolygons inside = offset_ex(layer->lslices, -float(scale_(0.1)));
                const bool hits = std::any_of(path->polyline.points.begin(), path->polyline.points.end(), [&inside](const auto &road_point) {
                    const Point pt(road_point.x(), road_point.y());
                    return std::any_of(inside.begin(), inside.end(), [&pt](const ExPolygon &island) { return island.contains(pt); });
                });
                if (hits && count++ < 3)
                    what << "[road at Z " << top << ", height " << path->height << ", into the part layer "
                         << layer->print_z - layer->height << ".." << layer->print_z << "] ";
            }
        }
    }
    return count == 0 ? std::string() : std::to_string(count) + " support roads reach into the part " + what.str();
}

// Slices one case to G-code. Returns the refusal's code and setting, and what is wrong with the slice, if anything.
// two_body_layers counts the support layers where both nozzles lay body, on the PETG contact cases.
Outcome run_case(const MatrixCase &c, size_t &two_body_layers)
{
    std::ostringstream what;
    what << label(c) << ": ";
    DynamicPrintConfig config = h2d_support_config(c.mode, c.fine, c.coarse, c.ratio, c.top_z, c.support, c.filaments);
    if (c.shape == Shape::Shelf)
        config.set_key_value("independent_support_layer_height", new ConfigOptionBool(false));
    if (c.brim) {
        config.set_key_value("brim_type", new ConfigOptionEnum<BrimType>(btOuterOnly));
        config.set_key_value("brim_width", new ConfigOptionFloat(5.));
        config.set_key_value("brim_object_gap", new ConfigOptionFloat(0.));
    }
    bambu_config(config);
    Model model;
    Print print;
    print.is_BBL_printer() = true;
    if (c.shape == Shape::Shelf) {
        add_object(model, "shelf and two arms", shelf_and_arms_mesh(), Vec3d(60., 60., 0.));
        print.apply(model, config);
        print.set_status_silent();
    } else
        load_two_arm_model(model, print, config, c.mode, c.fine, c.coarse);
    const double left = model.objects.front()->instance_bounding_box(0).min.x();
    Outcome out;
    CadenceTest::Facts facts;
    if (out.failure = slice_into(print, what.str(), facts); !out.failure.empty()) {
        out.code    = code_of(facts.refusal.string);
        out.opt_key = facts.refusal.opt_key;
        return out;
    }

    const bool mode_on = c.mode != Mode::Off;
    const bool petg    = std::string(c.filaments.interface_type) == "PETG";
    // Under a mode a Default interface prints with the base filament.
    const int interface_filament = c.filaments.interface_filament > 0 ? c.filaments.interface_filament :
                                   mode_on ? c.filaments.base : 0;
    // Each nozzle's layer height range, as h2d_support_config sets it.
    const double nozzle_min[2] = {0.04, 0.16};
    const double nozzle_max[2] = {0.14, 0.56};
    size_t base = 0, interface_roads = 0, base_wrong = 0, interface_wrong = 0, out_of_range = 0;
    for (const Move &move : facts.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        const int nozzle = int(move.physical_tool_id);
        if (move.extrusion_role == erSupportMaterial) {
            ++base;
            // Under a mode, body the coarse base nozzle cannot lay is laid by the interface nozzle in a filament of
            // the base's material there, never an interface filament of another material.
            const bool fine_body = mode_on && c.filaments.interface_filament > 0 &&
                k_filament_map[c.filaments.base - 1] != k_filament_map[c.filaments.interface_filament - 1] &&
                nozzle == k_filament_map[c.filaments.interface_filament - 1] - 1 &&
                (int(move.extruder_id) != c.filaments.interface_filament - 1 || !petg);
            base_wrong += !fine_body && (int(move.extruder_id) != c.filaments.base - 1 ||
                          nozzle != k_filament_map[c.filaments.base - 1] - 1);
        } else if (move.extrusion_role == erSupportMaterialInterface) {
            ++interface_roads;
            if (interface_filament > 0)
                interface_wrong += int(move.extruder_id) != interface_filament - 1 ||
                                   nozzle != k_filament_map[interface_filament - 1] - 1;
        } else if (move.extrusion_role != erInternalInfill || nozzle != 1)
            continue;
        // Under a mode every support road above the first layer is a height its nozzle can lay, and the coarse
        // nozzle lays no sparse infill thinner than its minimum.
        const bool sparse = move.extrusion_role == erInternalInfill;
        if (mode_on && nozzle >= 0 && nozzle < 2 && double(move.position.z()) > c.fine + 1e-3 &&
            (double(move.height) < nozzle_min[nozzle] - (sparse ? 0.005 : 1e-3) ||
             (!sparse && double(move.height) > nozzle_max[nozzle] + 1e-3))) {
            if (out_of_range < 3)
                what << "[Z " << move.position.z() << ", height " << move.height << ", role " << int(move.extrusion_role)
                     << ", nozzle " << nozzle + 1 << "] ";
            ++out_of_range;
        }
    }
    bool ok = true;
    if (base == 0 || interface_roads == 0 || base_wrong != 0 || interface_wrong != 0) {
        what << "base " << base << " (" << base_wrong << " on the wrong filament), interface " << interface_roads
             << " (" << interface_wrong << " on the wrong filament); ";
        ok = false;
    }
    if (out_of_range != 0) {
        what << out_of_range << " support or coarse sparse roads outside their nozzle's layer height range; ";
        ok = false;
    }
    // Under each arm the topmost support road is interface and sits the top Z distance below the
    // model, rounded up to at most one fine layer.
    if (c.shape == Shape::TwoArm)
        for (const auto &[name, lo, hi] : {std::tuple<const char *, double, double>{"left arm", left + 1., left + 7.},
                                            {"right arm", left + 18., left + 23.}}) {
            const Gap gap = support_gap(facts.moves, lo, hi);
            const double distance = gap.object_bottom - gap.support_top;
            if (gap.support_top <= 0. || gap.top_role != erSupportMaterialInterface ||
                distance < c.top_z - 1e-3 || distance > c.top_z + c.fine + 1e-3) {
                what << name << " support top " << gap.support_top << " role " << int(gap.top_role) << " under model at "
                     << gap.object_bottom << "; ";
                ok = false;
            }
        }
    // Support laid on coarse rows never reaches down into the part it stands on.
    if (mode_on)
        if (const std::string into = support_into_part(*print.objects().front()); !into.empty()) {
            what << into << "; ";
            ok = false;
        }

    if (c.petg_contacts) {
        for (const SupportLayer *layer : print.objects().front()->support_layers())
            two_body_layers += !layer->fine_body_fills.empty();
        // The PETG prints only the interface right under each arm (x range and underside from two_arm_mesh()), and
        // the layer under its lowest one is dense PLA on the fine nozzle.
        struct Arm { const char *name; double x_lo, x_hi, underside; };
        const Arm arms[2] = {{"left arm", left, left + 8., 6.6}, {"right arm", left + 16., left + 24., 3.0}};
        const int petg_filament = c.filaments.interface_filament - 1;
        size_t petg_roads = 0, petg_elsewhere = 0;
        double lowest_petg[2] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
        for (const Move &move : facts.moves) {
            if (move.type != EMoveType::Extrude || move.extrusion_role == erWipeTower || int(move.extruder_id) != petg_filament)
                continue;
            ++petg_roads;
            const double x = double(move.position.x()), z = double(move.position.z());
            bool at_contact = false;
            for (size_t i = 0; i < 2; ++i)
                if (move.extrusion_role == erSupportMaterialInterface && x > arms[i].x_lo - 2. && x < arms[i].x_hi + 2. &&
                    z > arms[i].underside - c.top_z - 2. * c.fine - 1e-3 && z < arms[i].underside - c.top_z + 1e-3) {
                    at_contact = true;
                    lowest_petg[i] = std::min(lowest_petg[i], z);
                }
            if (!at_contact && petg_elsewhere++ < 3)
                what << "[PETG role " << int(move.extrusion_role) << " at x " << x << ", z " << z << "] ";
        }
        if (petg_roads == 0 || petg_elsewhere != 0) {
            what << petg_roads << " PETG roads, " << petg_elsewhere << " of them away from the contacts; ";
            ok = false;
        }
        for (size_t i = 0; i < 2; ++i) {
            const double below = lowest_petg[i] - c.fine;
            const bool pla_below = lowest_petg[i] < std::numeric_limits<double>::max() &&
                std::any_of(facts.moves.begin(), facts.moves.end(), [&](const Move &move) {
                    return move.type == EMoveType::Extrude && move.extrusion_role == erSupportMaterial &&
                        int(move.extruder_id) != petg_filament && move.physical_tool_id == 0 &&
                        std::abs(double(move.position.z()) - below) < 1e-3 &&
                        double(move.position.x()) > arms[i].x_lo && double(move.position.x()) < arms[i].x_hi;
                });
            if (!pla_below) {
                what << "no fine PLA body under the " << arms[i].name << "'s PETG; ";
                ok = false;
            }
        }
    }

    if (c.brim) {
        // The brim keeps off the first-layer support: a brim road that ends inside a support road would lay
        // filament on top of it. A few brim ends touch a support road where the two meet.
        double first_z = std::numeric_limits<double>::max();
        for (const Move &move : facts.moves)
            if (CadenceTest::model_road(move))
                first_z = std::min(first_z, double(move.position.z()));
        struct Road { Vec2d a, b; double width; };
        std::vector<Road> support;
        for (size_t i = 1; i < facts.moves.size(); ++i)
            if (const Move &move = facts.moves[i]; is_support(move) && std::abs(double(move.position.z()) - first_z) < 1e-3)
                support.push_back({facts.moves[i - 1].position.head<2>().cast<double>(), move.position.head<2>().cast<double>(),
                                   double(move.width)});
        size_t brim_roads = 0, brim_on_support = 0;
        for (const Move &move : facts.moves)
            if (move.type == EMoveType::Extrude && move.extrusion_role == erBrim && std::abs(double(move.position.z()) - first_z) < 1e-3) {
                ++brim_roads;
                const Vec2d end = move.position.head<2>().cast<double>();
                brim_on_support += std::any_of(support.begin(), support.end(), [&](const Road &road) {
                    return point_to_segment(end, road.a, road.b) < 0.5 * (road.width + double(move.width)) - 0.05;
                });
            }
        if (support.empty() || brim_roads == 0 || brim_on_support * 20 > brim_roads) {
            what << brim_on_support << " of " << brim_roads << " brim roads end on the " << support.size() << " first-layer support roads; ";
            ok = false;
        }
    }
    out.failure = ok ? std::string() : what.str();
    return out;
}

// Slim, Strong and Hybrid trees on a coarse base are refused for their style (an Organic tree never is), and any
// tree with its interface on the base's coarse nozzle for that: SRL-F14 under Feature Split, SRL-A53 under Body Split.
bool tree_refused(const Outcome &outcome, const MatrixCase &c)
{
    if (outcome.code != "SRL-F14" && outcome.code != "SRL-A53")
        return false;
    if (outcome.opt_key == "support_style")
        return c.support.style != smsTreeOrganic;
    const bool interface_on_base_nozzle = c.filaments.interface_filament == 0 ||
        k_filament_map[c.filaments.base - 1] == k_filament_map[c.filaments.interface_filament - 1];
    return outcome.opt_key == "support_interface_filament" && interface_on_base_nozzle;
}

// Every case prints, or is refused as a tree on a coarse base, except an interface on a larger nozzle than the base
// under a mode, which is refused (SRL-F01, SRL-A29): the interface is what touches the model.
std::string judge(const MatrixCase &c, const Outcome &outcome)
{
    if (tree_refused(outcome, c))
        return {};
    const bool interface_larger = c.mode != Mode::Off && c.filaments.interface_filament > 0 &&
        k_filament_map[c.filaments.base - 1] == 1 && k_filament_map[c.filaments.interface_filament - 1] == 2;
    if (!interface_larger)
        return outcome.failure;
    if (outcome.code == (c.mode == Mode::Body ? "SRL-A29" : "SRL-F01") && outcome.opt_key == "support_interface_filament")
        return {};
    return label(c) + ": expected a refusal, got " + (outcome.failure.empty() ? std::string("a slice") : outcome.failure);
}
} // namespace

TEST_CASE("Support matrix on an H2D 0.2/0.8 slices to G-code; Slim and Hybrid trees on a coarse base are refused", "[TestRebuild][SupportMatrix]")
{
    const FilamentCase base_coarse{"base coarse, interface fine", 2, 3};
    const FilamentCase petg{"base coarse, interface fine", 2, 3, "PETG"};
    const std::vector<FilamentCase> filament_cases{
        {"interface Default", 2, 0},
        {"interface = base", 2, 2},
        {"interface on the base nozzle", 2, 4},
        base_coarse,
        {"base fine, interface coarse", 3, 4},
    };
    std::vector<MatrixCase> cases;
    for (Mode mode : {Mode::Off, Mode::Feature, Mode::Body})
        for (const SupportCase &support : {k_normal, k_slim, k_hybrid, k_organic})
            for (const FilamentCase &filaments : filament_cases)
                // With the base on the coarse nozzle the fine nozzle lays the first-layer support, which the brim keeps
                // off in Feature Split as in Off.
                cases.push_back({mode, 0.10, 0.30, 3, 0.20, support, filaments, Shape::TwoArm,
                                 mode != Mode::Body && support.type == stNormalAuto && filaments.base == 2 && filaments.interface_filament == 3});
    // A 0.08 fine layer under 0.56 (ratio 7) and 0.24 (ratio 3) coarse layers, with a 0.24 gap.
    for (const auto &[coarse, ratio] : {std::pair<double, int>{0.56, 7}, {0.24, 3}})
        for (const SupportCase &support : {k_organic, k_hybrid, k_slim, k_normal})
            cases.push_back({Mode::Feature, 0.08, coarse, ratio, 0.24, support, petg});
    for (const SupportCase &support : {k_organic, k_hybrid, k_slim})
        cases.push_back({Mode::Feature, 0.08, 0.56, 7, 0.24, support, base_coarse});
    cases.push_back({Mode::Feature, 0.08, 0.56, 7, 0.24, k_organic, filament_cases[1]});
    for (const FilamentCase &filaments : {base_coarse, filament_cases[4], filament_cases[0]})
        cases.push_back({Mode::Off, 0.08, 0.56, 7, 0.24, k_organic, filaments});
    // At zero gap a PETG interface prints only at the contacts, also where both nozzles lay body on one layer. The
    // shelf's support stands on the part with body and interface sharing the coarse nozzle.
    for (const SupportCase &support : {k_normal, k_organic})
        for (const auto &[coarse, ratio] : {std::pair<double, int>{0.30, 3}, {0.20, 2}}) {
            cases.push_back({Mode::Feature, 0.10, coarse, ratio, 0., support, petg, Shape::TwoArm, false, true});
            if (support.type == stNormalAuto)
                cases.push_back({Mode::Feature, 0.10, coarse, ratio, 0., support, filament_cases[2], Shape::Shelf});
        }
    std::vector<std::string> failures;
    size_t two_body_layers = 0;
    for (const MatrixCase &c : cases)
        failures.push_back(judge(c, run_case(c, two_body_layers)));
    CAPTURE(cases.size());
    check_no_failures(failures);
    // Organic trees leave thin pieces to the fine nozzle beside coarse band roads, so the PETG contact cases have
    // layers where both nozzles lay body.
    CHECK(two_body_layers > 0);
}

TEST_CASE("Feature Split exports a plate of two different objects of different heights, with and without supports",
          "[TestRebuild][Support][MultiObject]")
{
    // An 8 mm column with two arms and a 20 mm box beside it: each object gets its own layer plan
    // block, so the plate exports.
    const double box_height = 20.;
    for (const bool supports : {false, true}) {
        INFO((supports ? "with supports" : "without supports"));
        CadenceTest::Scene scene;
        scene.config = h2d_support_config(Mode::Feature, 0.10, 0.30, 3, 0.20, k_normal, FilamentCase{"base coarse, interface fine", 2, 3});
        scene.config.set_key_value("enable_support", new ConfigOptionBool(supports));
        double column_max_x = 0., box_min_x = 0.;
        scene.populate = [&](Model &model, Print &print, const DynamicPrintConfig &cfg) {
            column_max_x = add_object(model, "two-arm-overhang", two_arm_mesh(), Vec3d(60., 60., 0.))->instance_bounding_box(0).max.x();
            box_min_x = add_object(model, "tall-box", make_cube(14., 14., box_height), Vec3d(110., 60., 0.))->instance_bounding_box(0).min.x();
            print.apply(model, cfg);
            print.set_status_silent();
        };
        const CadenceTest::Facts facts = CadenceTest::slice(scene);
        INFO(facts.refusal.string);
        REQUIRE(facts.refusal.string.empty());
        REQUIRE_FALSE(facts.gcode.empty());
        REQUIRE(column_max_x < box_min_x);
        CHECK(facts.emitted.plan_start_markers == 2);
        CHECK(facts.emitted.plan_end_markers == 2);
        CHECK_FALSE(facts.emitted.plan_malformed);

        // Both objects print in full: walls up to each one's own top, sparse infill inside each.
        double column_top = 0., box_top = 0.;
        size_t column_sparse = 0, box_sparse = 0, support_roads = 0;
        for (const Move &move : facts.moves) {
            support_roads += is_support(move);
            if (!CadenceTest::model_road(move))
                continue;
            const bool on_box = double(move.position.x()) > box_min_x - 1.;
            (on_box ? box_top : column_top) = std::max(on_box ? box_top : column_top, double(move.position.z()));
            if (move.extrusion_role == erInternalInfill)
                ++(on_box ? box_sparse : column_sparse);
        }
        CAPTURE(column_top, box_top, column_sparse, box_sparse, support_roads);
        CHECK_THAT(box_top, Catch::Matchers::WithinAbs(box_height, 0.11));
        CHECK_THAT(column_top, Catch::Matchers::WithinAbs(8., 0.11));
        CHECK(column_sparse > 0);
        CHECK(box_sparse > 0);
        CHECK((support_roads > 0) == supports);
    }
}

// ---- Tower from the first layer -------------------------------------------------------------------

namespace {
struct TowerTally {
    double      seconds = 0.;
    size_t      tower_levels = 0;
    size_t      coarse_roads = 0;
    std::string gcode;
    double      first_layer_height = 0.;
    double      max_layer_height = 0.;
};

// What the G-code does with the prime tower: tower extrusions are the ones under "; FEATURE: Prime tower" or inside
// a tower block, and each move's layer is the last "; Z_HEIGHT:" above it.
struct TowerPath {
    double lowest_tower_z = std::numeric_limits<double>::max();
    // G-code layer of the lowest tower extrusion, and of the first extrusion above the first layer.
    size_t lowest_tower_layer = 0;
    size_t first_layer_above_bed = 0;
    // Tower extrusions laid more than `tolerance` away from their layer's Z.
    size_t off_layer_tower_roads = 0;
    double worst_off_layer = 0.;
    // Z-only descents of more than 1 mm to below the layer being printed, outside the tower footprint.
    std::vector<std::string> plunges;
};

TowerPath tower_path(const std::string &gcode, double first_layer_height, double tolerance)
{
    const auto on_tower = [](const GMove &move) { return move.in_tower || move.in_change || move.prime; };
    // The tower's footprint, grown by 3 mm.
    BoundingBoxf footprint;
    walk_gcode(gcode, [&](const GMove &move) {
        if (move.has_xy && move.e > 0. && on_tower(move))
            footprint.merge(move.points.back());
    });
    const auto over_tower = [&footprint](const Vec2d &p) {
        return footprint.defined && p.x() >= footprint.min.x() - 3. && p.x() <= footprint.max.x() + 3. &&
            p.y() >= footprint.min.y() - 3. && p.y() <= footprint.max.y() + 3.;
    };
    TowerPath path;
    walk_gcode(gcode, [&](const GMove &move) {
        if (move.tool_change)
            return;
        if (move.has_z && !move.has_xy && move.layer > 0 && move.z0 - move.z > 1. && move.z < move.part_z - 0.5 &&
            !over_tower(move.points.front())) {
            std::ostringstream plunge;
            plunge << "line " << move.line << " layer " << move.layer << " at X" << move.points.front().x() << " Y"
                   << move.points.front().y() << ": Z " << move.z0 << " -> " << move.z;
            path.plunges.push_back(plunge.str());
        }
        if (move.layer == 0 || !move.has_xy || move.e <= 0.)
            return;
        // A purge the change template lays away from the tower is not a tower level.
        if (on_tower(move) && over_tower(move.points.back())) {
            if (move.z < path.lowest_tower_z - EPSILON) {
                path.lowest_tower_z = move.z;
                path.lowest_tower_layer = move.layer;
            }
            if (const double off = std::abs(move.z - move.part_z); off > tolerance + EPSILON) {
                ++path.off_layer_tower_roads;
                path.worst_off_layer = std::max(path.worst_off_layer, off);
            }
        } else if (path.first_layer_above_bed == 0 && move.z > first_layer_height + EPSILON)
            path.first_layer_above_bed = move.layer;
    });
    return path;
}

// The L part from the app project with support base (fine PLA 1) and PETG interface (2) both on the fine nozzle; with
// `plain`, one filament and no support or sparse infill, so no layer switches tools. Requires a slice.
TowerTally slice_l_part_fine_support(Mode mode, bool no_sparse_layers = true, bool plain = false)
{
    DynamicPrintConfig config = app_three_filament_config(mode, 0.10, 0.30, 3, k_normal);
    config.set_key_value("support_filament", new ConfigOptionInt(1));
    config.set_key_value("support_interface_filament", new ConfigOptionInt(2));
    // Stock Orca does not offer "No sparse layers" on a Bambu tower; the presets write it on.
    config.set_key_value("wipe_tower_no_sparse_layers", new ConfigOptionBool(no_sparse_layers));
    // Off prints the part in one filament, as stock does.
    if (mode == Mode::Off || plain)
        single_filament_roles(config);
    if (plain) {
        config.set_key_value("enable_support", new ConfigOptionBool(false));
        config.set_key_value("sparse_infill_density", new ConfigOptionPercent(0.));
    }
    CadenceTest::Scene scene;
    scene.config   = config;
    scene.populate = one_object("l-part", overhang_shelf_mesh(20., 60., 40., 50.), Vec3d(60., 60., 0.));
    const CadenceTest::Facts facts = CadenceTest::slice(scene);
    INFO(mode_name(mode) << (plain ? " plain: " : ": ") << facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    TowerTally tally;
    tally.seconds = facts.seconds;
    tally.gcode = facts.gcode;
    tally.first_layer_height = config.opt_float("initial_layer_print_height");
    const std::vector<double> &max_heights = config.option<ConfigOptionFloats>("max_layer_height")->values;
    tally.max_layer_height = max_heights.empty() ? 0. : *std::max_element(max_heights.begin(), max_heights.end());
    std::set<long> levels;
    for (const Move &move : facts.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        if (move.extrusion_role == erWipeTower)
            levels.insert(std::lround(double(move.position.z()) * 1000.));
        tally.coarse_roads += move.physical_tool_id == 1;
    }
    tally.tower_levels = levels.size();
    return tally;
}

// The prime tower stands from the first layer, keeps level with the print, and the nozzle never drops
// through the part to reach it.
void check_tower_from_first_layer(const TowerTally &tally)
{
    const TowerPath path = tower_path(tally.gcode, tally.first_layer_height, tally.max_layer_height);
    CAPTURE(path.lowest_tower_z, path.lowest_tower_layer, path.first_layer_above_bed, path.off_layer_tower_roads,
            path.worst_off_layer, path.plunges.size());
    INFO((path.plunges.empty() ? std::string("no plunge") : "first plunge: " + path.plunges.front()));
    REQUIRE(path.lowest_tower_layer > 0);
    CHECK_THAT(path.lowest_tower_z, Catch::Matchers::WithinAbs(tally.first_layer_height, 0.001));
    REQUIRE(path.first_layer_above_bed > 0);
    CHECK(path.lowest_tower_layer < path.first_layer_above_bed);
    CHECK(path.off_layer_tower_roads == 0);
    CHECK(path.plunges.empty());
}
} // namespace

TEST_CASE("Feature Split with no coarse work left starts the prime tower on the first layer", "[TestRebuild][Tower]")
{
    // Every coarse band goes back fine, so only the PETG interface changes are left and the tower is planned as Off;
    // with "No sparse layers" on it still stands from the first layer as stock lays it.
    const TowerTally split = slice_l_part_fine_support(Mode::Feature);
    REQUIRE(split.coarse_roads == 0);
    check_tower_from_first_layer(split);

    // Off with "No sparse layers" off is stock Orca on a Bambu tower.
    const TowerTally off = slice_l_part_fine_support(Mode::Off, false);
    REQUIRE(off.coarse_roads == 0);
    check_tower_from_first_layer(off);
    CAPTURE(off.seconds, split.seconds, off.tower_levels, split.tower_levels);
    CHECK(split.tower_levels == off.tower_levels);
    CHECK(std::abs(split.seconds - off.seconds) <= 0.01 * off.seconds);

    // With the tower on but no tool change, the conflict check never reads a tower that was not generated.
    const TowerTally plain = slice_l_part_fine_support(Mode::Feature, true, true);
    CHECK_FALSE(plain.gcode.empty());
    CHECK(plain.coarse_roads == 0);
}

// ---- Shapes --------------------------------------------------------------------------------------

namespace {
struct ShapeRun {
    const char                        *name;
    std::function<void(ModelObject&)> parts;
    Mode                               mode;
    double                             fine, coarse;
    int                                ratio;
    SupportCase                        support;
    // Organic coarse support leaves few short scraps (the large bunny).
    bool                               scraps = false;
    // The coarse nozzle rams above fine levels: the held rows keep clear of later tower roads.
    bool                               held_rows = false;
};

// Slices a shape in the app project (Normal support takes the fine PLA as interface) and returns what is wrong: an
// Organic interface filament on the bed or tower bed level, or a coarse support or sparse road outside its heights.
std::string shape_case(const ShapeRun &run, size_t &coarse_roads)
{
    std::ostringstream what;
    what << run.name << " / " << mode_name(run.mode) << " / " << run.support.name << " / " << run.fine << "/" << run.coarse << ": ";
    const bool organic = run.support.style == smsTreeOrganic;
    CadenceTest::Scene scene;
    scene.config = app_three_filament_config(run.mode, run.fine, run.coarse, run.ratio, run.support);
    if (!organic)
        scene.config.set_key_value("support_interface_filament", new ConfigOptionInt(1));
    scene.populate = [&run](Model &model, Print &print, const DynamicPrintConfig &cfg) {
        ModelObject *object = model.add_object();
        object->name = run.name;
        run.parts(*object);
        if (object->instances.empty())
            object->add_instance();
        object->center_around_origin(false);
        object->instances.front()->set_offset(Vec3d(100., 120., 0.));
        object->ensure_on_bed();
        print.apply(model, cfg);
        print.set_status_silent();
    };
    CadenceTest::Facts facts;
    if (std::string failure = slice_into(scene, what.str(), facts); !failure.empty())
        return failure;
    const int    interface_filament = scene.config.opt_int("support_interface_filament");
    const double coarse_min = scene.config.option<ConfigOptionFloats>("min_layer_height")->get_at(1);
    const double coarse_max = scene.config.option<ConfigOptionFloats>("max_layer_height")->get_at(1);
    size_t support = 0, interface_roads = 0, on_bed = 0, tower_on_bed = 0, outside = 0;
    // Coarse support road length per height.
    std::map<long long, double> coarse_support;
    for (size_t i = 1; i < facts.moves.size(); ++i) {
        const Move &move = facts.moves[i];
        if (move.type != EMoveType::Extrude)
            continue;
        const double z = double(move.position.z()), height = double(move.height);
        const bool body = move.extrusion_role == erSupportMaterial;
        const bool in_interface = int(move.extruder_id) == interface_filament - 1;
        support += body;
        interface_roads += in_interface && move.extrusion_role == erSupportMaterialInterface;
        if (organic && in_interface && z < run.fine + 1e-3)
            ++(move.extrusion_role == erWipeTower ? tower_on_bed : on_bed);
        if (run.mode == Mode::Off || move.physical_tool_id != 1 || z < run.fine + 1e-3 ||
            (!body && move.extrusion_role != erInternalInfill))
            continue;
        ++coarse_roads;
        if (height < coarse_min - 0.005 || (body && height > coarse_max + 0.005)) {
            if (outside < 3)
                what << "[coarse " << (body ? "support" : "sparse") << " at Z " << z << ", height " << height << "] ";
            ++outside;
        }
        if (body)
            coarse_support[std::llround(z * 1e4)] += double((move.position - facts.moves[i - 1].position).head<2>().norm());
    }
    size_t scraps = 0;
    for (const auto &[z, length] : coarse_support)
        scraps += length < 5.;
    bool ok = true;
    if (support == 0 || interface_roads == 0) {
        what << "support " << support << ", interface " << interface_roads << "; ";
        ok = false;
    }
    if (on_bed + tower_on_bed != 0) {
        what << on_bed << " interface-filament roads on the first layer, and " << tower_on_bed << " on the tower's; ";
        ok = false;
    }
    if (outside != 0) {
        what << outside << " coarse roads outside its layer heights; ";
        ok = false;
    }
    if (run.scraps && scraps > coarse_support.size() / 20) {
        what << scraps << " of " << coarse_support.size() << " coarse support heights lay less than 5 mm; ";
        ok = false;
    }
    if (run.held_rows)
        if (const std::string failure = held_rows_failure(facts.gcode); !failure.empty()) {
            what << failure << "; ";
            ok = false;
        }
    return ok ? std::string() : what.str();
}
} // namespace

TEST_CASE("Supports on curved and standing shapes keep the interface filament off the bed and the coarse nozzle within its heights",
          "[TestRebuild][SupportMatrix]")
{
    // Support standing on the part (C shape) or on the bed under a curve. Rows the coarse tool rams above a fine level
    // (Benchy at 0.10/0.30, the pocket at 0.08/0.56) keep clear of later tower roads.
    const auto benchy = handy_model("3DBenchy.drc"), bunny = handy_model("Stanford_Bunny.drc");
    const auto ball = parts_of(bunny_like_parts), pocket = parts_of(benchy_like_parts);
    const std::vector<ShapeRun> runs{
        {"bunny-like ball on a foot", ball, Mode::Off, 0.10, 0.30, 3, k_organic},
        {"bunny-like ball on a foot", ball, Mode::Feature, 0.10, 0.30, 3, k_organic},
        {"bunny-like ball on a foot", ball, Mode::Feature, 0.08, 0.56, 7, k_organic},
        {"Benchy-like pocket", pocket, Mode::Off, 0.10, 0.30, 3, k_organic},
        {"Benchy-like pocket", pocket, Mode::Feature, 0.10, 0.30, 3, k_organic},
        {"Benchy-like pocket", pocket, Mode::Feature, 0.08, 0.56, 7, k_organic, false, true},
        {"C shape", part_of(c_shape_mesh), Mode::Feature, 0.10, 0.30, 3, k_organic},
        {"C shape", part_of(c_shape_mesh), Mode::Feature, 0.08, 0.56, 7, k_normal},
        {"hull", part_of(hull_mesh), Mode::Feature, 0.10, 0.30, 3, k_organic},
        {"hull", part_of(hull_mesh), Mode::Feature, 0.08, 0.56, 7, k_normal},
        {"3DBenchy", benchy, Mode::Feature, 0.10, 0.30, 3, k_organic, false, true},
        {"Stanford bunny", bunny, Mode::Feature, 0.10, 0.30, 3, k_organic, true},
        {"Stanford bunny", bunny, Mode::Feature, 0.10, 0.30, 3, k_normal},
    };
    std::vector<std::string> failures;
    size_t coarse_roads = 0;
    for (const ShapeRun &run : runs)
        failures.push_back(shape_case(run, coarse_roads));
    check_no_failures(failures);
    CHECK(coarse_roads > 0);
}

// ---- Body Split support and the tower --------------------------------------------------------------

namespace {
enum class BodyShape { Block, Mushroom, CoarseColumn };

// Body Split in the app project, support base coarse PLA (3). Block: fine base, coarse column, PETG cap overhanging
// both. Mushroom: fine stem under a wide coarse cap. CoarseColumn: coarse column with a fine arm on top. Every nozzle
// change is on the tower, the tower never runs ahead of, under or through the part, and PETG never prints support body.
std::string body_split_support_case(BodyShape shape, const SupportCase &support, int interface_filament, double fine, double coarse)
{
    std::ostringstream what;
    what << (shape == BodyShape::Block ? "Body Split three-part block " : shape == BodyShape::Mushroom ? "Body Split mushroom " :
             "Body Split coarse column, fine arm ") << fine << "/" << coarse << " / " << support.name << " / interface filament "
         << interface_filament << ": ";
    CadenceTest::Scene scene;
    scene.config = app_three_filament_config(Mode::Feature, fine, coarse, int(std::lround(coarse / fine)), support);
    scene.config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::BodySplit));
    single_filament_roles(scene.config);
    scene.config.set_key_value("enable_support", new ConfigOptionBool(true));
    scene.config.set_key_value("support_filament", new ConfigOptionInt(3));
    scene.config.set_key_value("support_interface_filament", new ConfigOptionInt(interface_filament));
    scene.populate = [shape, fine, coarse](Model &model, Print &print, const DynamicPrintConfig &cfg) {
        ModelObject *object = model.add_object();
        object->name = "Body Split support block";
        const auto at = [](TriangleMesh mesh, float x, float y, float z) { mesh.translate(x, y, z); return mesh; };
        if (shape == BodyShape::Mushroom) {
            add_part(*object, at(make_cube(10., 10., 20.), 20.f, 20.f, 0.f), 1, fine);
            add_part(*object, at(make_cube(50., 50., 4.), 0.f, 0.f, 20.f), 3, coarse);
        } else if (shape == BodyShape::Block) {
            add_part(*object, make_cube(40., 30., 4.), 1, fine);
            add_part(*object, at(make_cube(20., 20., 26.), 10.f, 5.f, 4.f), 3, coarse);
            add_part(*object, at(make_cube(50., 30., 4.), 0.f, 0.f, 30.f), 2, fine);
        } else {
            add_part(*object, make_cube(20., 20., 40.), 3, coarse);
            add_part(*object, at(make_cube(40., 20., 10.), 20.f, 0.f, 30.f), 1, fine);
        }
        object->add_instance();
        object->instances.front()->set_offset(Vec3d(100., 120., 0.));
        object->ensure_on_bed();
        print.apply(model, cfg);
        print.set_status_silent();
    };
    CadenceTest::Facts facts;
    if (std::string failure = slice_into(scene, what.str(), facts); !failure.empty())
        return failure;
    size_t changes = 0, off_tower = 0;
    double first_off_tower = 0.;
    walk_gcode(facts.gcode, [&](const GMove &move) {
        if (!move.tool_change)
            return;
        ++changes;
        if (!move.in_tower && off_tower++ == 0)
            first_off_tower = move.part_z;
    });
    if (changes == 0)
        return what.str() + "no tool change";
    if (off_tower > 0)
        return what.str() + std::to_string(off_tower) + " of " + std::to_string(changes) +
               " tool changes off the prime tower, the first at Z " + std::to_string(first_off_tower);
    // Filament 2 is PETG: the support body is PLA, whichever nozzle lays it.
    if (interface_filament == 2)
        if (const auto petg_body = std::count_if(facts.moves.begin(), facts.moves.end(), [](const Move &move) {
                return move.type == EMoveType::Extrude && move.extrusion_role == erSupportMaterial && move.extruder_id == 1; });
            petg_body != 0)
            return what.str() + std::to_string(petg_body) + " support body roads in the PETG interface filament";
    const double coarse_min = scene.config.option<ConfigOptionFloats>("min_layer_height")->get_at(1);
    if (std::string ahead = tower_ahead_failure(facts.gcode, coarse_min); !ahead.empty())
        return what.str() + ahead;
    if (std::string dive = tower_dive_failure(facts.gcode, coarse_min); !dive.empty())
        return what.str() + dive;
    if (std::string plough = tower_plough_failure(tower_roads(facts.gcode)); !plough.empty())
        return what.str() + plough;
    return {};
}
} // namespace

TEST_CASE("Supported plates keep every nozzle change on the prime tower, and no tower road runs into another", "[TestRebuild][Support]")
{
    std::vector<std::string> failures;
    // 0.08/0.56 has coarse support switching nozzle every few fine layers under the cap: a coarse arrival right at the
    // tower top lays no full coarse layer there, so the tower never runs ahead of the part.
    for (const auto &[fine, coarse] : {std::pair<double, double>{0.10, 0.30}, {0.08, 0.56}})
        for (const SupportCase &support : {k_normal, k_organic})
            // PETG on the fine nozzle, and PLA on the fine nozzle (the base's own material).
            for (const int interface_filament : {2, 1})
                failures.push_back(body_split_support_case(BodyShape::Block, support, interface_filament, fine, coarse));
    for (const SupportCase &support : {k_normal, k_organic}) {
        // Coarse prime roads held above the tower top, which the next fine purge must not run under.
        failures.push_back(body_split_support_case(BodyShape::Mushroom, support, 2, 0.08, 0.56));
        // The fine nozzle's support body filament comes from the project, never the PETG interface.
        failures.push_back(body_split_support_case(BodyShape::CoarseColumn, support, 2, 0.10, 0.30));
    }
    // A saved Feature Split L part on 0.2/0.6 at 0.08/0.40 with organic support and a PETG interface: the 0.6 rams at
    // its 0.12 minimum on 0.08 levels, so its rows stand 0.04 mm proud; later roads on that level must keep clear.
    DynamicPrintConfig config;
    Model model = CadenceTest::load_project("feature-split-L-tower-008.3mf", config);
    REQUIRE(model.objects.size() == 1);
    Print print;
    print.is_BBL_printer() = true;
    print.apply(model, config);
    print.set_status_silent();
    CadenceTest::Facts facts;
    const std::string refused = slice_into(print, "Feature Split L tower project: ", facts);
    failures.push_back(refused.empty() ? held_rows_failure(facts.gcode) : refused);
    check_no_failures(failures);
}

TEST_CASE("The fine nozzle's support body filament comes from the project, never the interface filament", "[TestRebuild][Support]")
{
    // Filaments: 1 PLA and 2 PETG (the interface) and 4 PETG on the fine nozzle, 3 PLA (the base) on the coarse one, with
    // the adhesion categories the printer profiles give PLA (100) and PETG (300).
    DynamicPrintConfig config = app_three_filament_config(Mode::Feature, 0.10, 0.30, 3, k_normal);
    REQUIRE(config.opt_int("support_filament") == 3);
    REQUIRE(config.opt_int("support_interface_filament") == 2);
    const auto pick = [&config](const std::vector<unsigned int> &object_filaments) {
        PrintConfig print_config;
        print_config.apply(config, true);
        PrintObjectConfig object_config;
        object_config.apply(config, true);
        return mixed_nozzle_interface_nozzle_body_filament(print_config, object_config, object_filaments);
    };
    // The base's material on the fine nozzle, whether or not this layer or the object prints it.
    CHECK(pick({0}) == std::optional<unsigned int>(0));
    CHECK(pick({}) == std::optional<unsigned int>(0));
    CHECK(pick({2}) == std::optional<unsigned int>(0));
    // Without it, a filament of another material (PETG) is not used: it does not stick to the PLA base.
    config.option<ConfigOptionBools>("filament_soluble")->values[0] = true;
    CHECK_FALSE(pick({}).has_value());
    // Another filament that sticks to the base is: of its type, or of its adhesion category.
    config.option<ConfigOptionStrings>("filament_type")->values[3] = "PLA";
    CHECK(pick({}) == std::optional<unsigned int>(3));
    config.option<ConfigOptionStrings>("filament_type")->values[3] = "PLA-CF";
    config.option<ConfigOptionInts>("filament_adhesiveness_category")->values[3] = 100;
    CHECK(pick({}) == std::optional<unsigned int>(3));
    // With none left, there is none: the interface filament is never the answer.
    config.option<ConfigOptionInts>("filament_map")->values[3] = 2;
    CHECK_FALSE(pick({}).has_value());
    // An interface filament of the base's own material prints the body.
    config.option<ConfigOptionStrings>("filament_type")->values[1] = "PLA";
    CHECK(pick({}) == std::optional<unsigned int>(1));
}

// ---- The tower's first level -----------------------------------------------------------------------

namespace {
// A nozzle pair with its layer height limits, fine, first and coarse layer heights.
struct NozzlePair {
    const char *name;
    double      fine_nozzle, coarse_nozzle;
    double      fine_min, fine_max, coarse_min, coarse_max;
    double      fine, first_layer, coarse;
};

void set_pair(DynamicPrintConfig &config, const NozzlePair &c)
{
    set_nozzles(config, c.fine_nozzle, c.coarse_nozzle, c.fine_min, c.fine_max, c.coarse_min, c.coarse_max);
    config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(c.first_layer));
}

struct TowerBed {
    std::string failure;            // a refusal or a throw
    size_t      interface_on_bed_level = 0;
    size_t      coarse_roads = 0;
};

// A low arm whose interface prints near the bed beside a taller arm: counts the interface-filament roads the first
// tower visit lays on the tower's bed level, and every coarse road.
TowerBed interface_on_tower_bed(const NozzlePair &nozzles, int base, double arm_z, Mode mode)
{
    std::ostringstream what;
    what << mode_name(mode) << " / " << nozzles.name << " / base " << base << " / arm at " << arm_z << ": ";
    const FilamentCase filaments{"PETG interface on fine", base, 3, "PETG"};
    CadenceTest::Scene scene;
    scene.config = h2d_support_config(mode, nozzles.fine, nozzles.coarse, int(std::lround(nozzles.coarse / nozzles.fine)), 0., k_normal, filaments);
    set_pair(scene.config, nozzles);
    scene.config.set_key_value("bottom_shell_layers", new ConfigOptionInt(1));
    scene.config.set_key_value("bottom_shell_thickness", new ConfigOptionFloat(0.));
    set_automatic_tower(scene.config);
    scene.populate = [arm_z](Model &model, Print &print, const DynamicPrintConfig &cfg) {
        // One object: the layer plan export takes one object per plate.
        ModelObject *object = add_object(model, "low and tall arms", overhang_shelf_mesh(10., 30., arm_z, arm_z + 3.), Vec3d(80., 80., 0.));
        TriangleMesh tall = overhang_shelf_mesh(10., 30., 4., 7.);
        tall.translate(0.f, 50.f, 0.f);
        object->add_volume(std::move(tall), ModelVolumeType::MODEL_PART, false);
        object->ensure_on_bed();
        print.apply(model, cfg);
        print.set_status_silent();
    };
    TowerBed out;
    CadenceTest::Facts facts;
    if (out.failure = slice_into(scene, what.str(), facts); !out.failure.empty())
        return out;
    double tower_bottom = std::numeric_limits<double>::max();
    for (const Move &move : facts.moves)
        if (move.type == EMoveType::Extrude) {
            if (move.extrusion_role == erWipeTower)
                tower_bottom = std::min(tower_bottom, double(move.position.z()));
            out.coarse_roads += move.physical_tool_id == 1;
        }
    // The first visit: tower roads from the first one to the next road off the tower.
    bool in_visit = false;
    for (const Move &move : facts.moves) {
        if (move.type != EMoveType::Extrude)
            continue;
        if (move.extrusion_role != erWipeTower) {
            if (in_visit)
                break;
            continue;
        }
        in_visit = true;
        out.interface_on_bed_level += int(move.extruder_id) == filaments.interface_filament - 1 &&
                                      double(move.position.z()) < tower_bottom + 1e-3;
    }
    return out;
}
} // namespace

TEST_CASE("The tower's first level lies on the bed with its brim, and the interface filament primes above it",
          "[TestRebuild][Support][TowerFirstLayer]")
{
    const NozzlePair pairs[] = {
        {"0.4/0.6", 0.4, 0.6, 0.08, 0.28, 0.12, 0.42, 0.20, 0.20, 0.40},
        // The 0.2 nozzle cannot lay a 0.2 mm first layer, so this pair starts at its fine height.
        {"0.2/0.8", 0.2, 0.8, 0.04, 0.14, 0.16, 0.56, 0.10, 0.10, 0.30},
    };
    std::vector<std::string> failures;
    for (const NozzlePair &nozzles : pairs)
        for (int base : {1, 2})
            for (double arm_z : {0.2, 0.4, 0.6, 1.0}) {
                const TowerBed split = interface_on_tower_bed(nozzles, base, arm_z, Mode::Feature);
                if (!split.failure.empty()) {
                    failures.push_back(split.failure);
                    continue;
                }
                // A plate the coarse nozzle never prints on builds Off's tower, so it is held to Off.
                if (split.interface_on_bed_level != 0 &&
                    (split.coarse_roads != 0 || interface_on_tower_bed(nozzles, base, arm_z, Mode::Off).interface_on_bed_level == 0))
                    failures.push_back(std::string(nozzles.name) + " / base " + std::to_string(base) + " / arm at " +
                                       std::to_string(arm_z) + ": " + std::to_string(split.interface_on_bed_level) +
                                       " interface-filament roads on the tower's bed level");
            }

    // Layer 1 has no tool switch: under the ball only the fine nozzle prints it, and the coarse nozzle first arrives one
    // fine layer up, closer than its own minimum. The L part prints three filaments, with the PETG interface under its arm.
    const double fine = 0.10;
    for (const auto &[filaments, ball] : {std::pair<FilamentCase, bool>{{"ball, interface on the part filament", 2, 1}, true},
                                          {{"L part, PETG interface on fine", 2, 3, "PETG"}, false}}) {
        const std::string what = std::string(filaments.name) + ": ";
        CadenceTest::Scene scene;
        scene.config = h2d_support_config(Mode::Feature, fine, 0.30, 3, 0.2, k_normal, filaments);
        set_automatic_tower(scene.config);
        scene.populate = one_object(ball ? "ball" : "l-part", ball ? hull_mesh() : overhang_shelf_mesh(20., 60., 40., 50.), Vec3d(80., 80., 0.));
        CadenceTest::Facts facts;
        if (std::string failure = slice_into(scene, what, facts); !failure.empty()) {
            failures.push_back(failure);
            continue;
        }
        // The brim widens the tower's first layer past the fullest of the next few tower levels.
        std::set<int> first_layer_filaments;
        BoundingBoxf first;
        std::map<double, std::pair<size_t, BoundingBoxf>> above;
        size_t first_roads = 0, first_interface = 0;
        double lowest = std::numeric_limits<double>::max();
        for (const Move &move : facts.moves) {
            if (move.type != EMoveType::Extrude)
                continue;
            if (move.extrusion_role != erWipeTower) {
                if (double(move.position.z()) < fine + 1e-3)
                    first_layer_filaments.insert(int(move.extruder_id));
                continue;
            }
            const double z = double(move.position.z());
            lowest = std::min(lowest, z);
            const Vec2d xy(move.position.x(), move.position.y());
            if (z < fine + 1e-3) {
                ++first_roads;
                // Here the interface filament is not the part's, so it must stay off the first layer.
                first_interface += filaments.interface_filament != 1 && int(move.extruder_id) == filaments.interface_filament - 1;
                first.merge(xy);
            } else if (z < fine + 1.) {
                auto &level = above[std::round(z * 1000.) / 1000.];
                ++level.first;
                level.second.merge(xy);
            }
        }
        const std::pair<size_t, BoundingBoxf> *fullest = nullptr;
        for (const auto &level : above)
            if (fullest == nullptr || level.second.first > fullest->first)
                fullest = &level.second;
        const double brim = first.defined && fullest != nullptr ?
            std::min({fullest->second.min.x() - first.min.x(), fullest->second.min.y() - first.min.y(),
                      first.max.x() - fullest->second.max.x(), first.max.y() - fullest->second.max.y()}) : 0.;
        if ((ball && first_layer_filaments.size() != 1) || first_roads == 0 || brim < 1. || first_interface != 0) {
            std::ostringstream out;
            out << what << first_layer_filaments.size() << " layer 1 filaments, lowest tower road at Z " << lowest
                << ", first-layer tower roads " << first_roads << " (" << first_interface << " in the interface filament), brim " << brim;
            failures.push_back(out.str());
        }
    }
    check_no_failures(failures);
}

// ---- Coarse support base ---------------------------------------------------------------------------

namespace {
// Where the coarse-base cases put the support filaments. Filaments 1 and 3 are on the 0.2, 2 and 4 on the coarse nozzle.
enum class SupportPlacement { PetgOnFine, AllCoarse, PetgOnCoarse };

// The L part (a column with an arm, support from the bed) or a step part: a box, a column on it and a slab
// wider than the box, so the support under the slab stands both on the box (bottom contacts) and on the bed.
enum class PartShape { L, Step };

FilamentCase placement_filaments(SupportPlacement where)
{
    return where == SupportPlacement::AllCoarse ? FilamentCase{"PLA base and interface on coarse", 2, 2} :
           where == SupportPlacement::PetgOnCoarse ? FilamentCase{"PLA base, PETG interface, both on coarse", 2, 4, "PETG"} :
                                                    FilamentCase{"coarse PLA base, PETG interface on fine", 2, 3, "PETG"};
}

// A fine PLA part on the 0.2 with its support base in PLA on the coarse nozzle, 0.10 / 0.40 with a 0.10 first layer
// and widths set for the 0.2. On a Body Split plate a one-part object on one filament is a plain object, so the plate
// gets a Body Split block too, a block with a coarse half that needs no support.
CadenceTest::Scene coarse_base_scene(Mode mode, const CoarseNozzle &nozzles, const SupportCase &support, SupportPlacement where,
                                     int raft_layers = 0, PartShape part_shape = PartShape::L)
{
    CadenceTest::Scene scene;
    scene.config = h2d_support_config(mode, 0.10, 0.40, 4, 0., support, placement_filaments(where));
    set_nozzles(scene.config, 0.2, nozzles.coarse, 0.04, 0.14, nozzles.coarse_min, nozzles.coarse_max);
    scene.config.set_key_value("support_interface_bottom_layers", new ConfigOptionInt(2));
    scene.config.set_key_value("support_bottom_z_distance", new ConfigOptionFloat(0.1));
    set_fine_widths(scene.config);
    scene.config.set_key_value("raft_layers", new ConfigOptionInt(raft_layers));
    set_automatic_tower(scene.config);
    scene.populate = [mode, part_shape](Model &model, Print &print, const DynamicPrintConfig &cfg) {
        TriangleMesh mesh = overhang_shelf_mesh(20., 60., 40., 50.);
        if (part_shape == PartShape::Step) {
            mesh = make_cube(30., 20., 10.);
            TriangleMesh column = make_cube(8., 8., 10.);
            column.translate(11.f, 6.f, 10.f);
            mesh.merge(column);
            TriangleMesh slab = make_cube(44., 20., 2.);
            slab.translate(-7.f, 0.f, 20.f);
            mesh.merge(slab);
        }
        ModelObject *object = add_object(model, "coarse-base-part", std::move(mesh), Vec3d(80., 80., 0.));
        if (mode == Mode::Body) {
            object->volumes.front()->config.set_key_value("extruder", new ConfigOptionInt(1));
            ModelObject *body = model.add_object();
            body->name = "body-block";
            add_part(*body, make_cube(10., 10., 10.), 1, 0.1);
            add_part(*body, make_cube(10., 10., 10.), 2, 0.4);
            body->volumes.back()->set_offset(Vec3d(10., 0., 0.));
            body->add_instance();
            body->instances.front()->set_offset(Vec3d(200., 200., 0.));
            body->ensure_on_bed();
        }
        print.apply(model, cfg);
        print.set_status_silent();
    };
    return scene;
}

// Slices a coarse_base_scene and returns what is wrong, or an empty string.
std::string coarse_base_case(Mode mode, const CoarseNozzle &nozzles, const SupportCase &support, SupportPlacement where,
                             PartShape part_shape)
{
    const double fine = 0.10;
    const FilamentCase filaments = placement_filaments(where);
    const bool coarse_interface = where != SupportPlacement::PetgOnFine;
    std::ostringstream what;
    what << mode_name(mode) << " / " << nozzles.coarse << " / " << support.name << " / " << filaments.name
         << (part_shape == PartShape::Step ? " / step part" : "") << ": ";
    const CadenceTest::Scene scene = coarse_base_scene(mode, nozzles, support, where, 0, part_shape);
    CadenceTest::Facts facts;
    if (std::string failure = slice_into(scene, what.str(), facts); !failure.empty())
        return failure;
    const double nozzle[2] = {0.2, nozzles.coarse};
    const int    petg      = filaments.interface_filament != filaments.base ? filaments.interface_filament - 1 : -1;
    // The first row: layer 1, or on the coarse nozzle the first whole layers it can lay.
    const double bed_z     = coarse_interface ? fine * std::ceil(nozzles.coarse_min / fine - 1e-6) : fine;
    const double bed_speed = scene.config.option<ConfigOptionFloatsNullable>("initial_layer_infill_speed")->get_at(coarse_interface ? 1 : 0);
    double lowest_support = std::numeric_limits<double>::max();
    size_t narrow = 0, first_layer_petg = 0, bed_roads = 0, bed_thin = 0, bed_fast = 0, roads[2] = {0, 0};
    double thinnest[2] = {1e9, 1e9}, widest[2] = {0., 0.};
    // Support body volume per nozzle (width x height x length).
    double body_volume[2] = {0., 0.};
    // Support Z levels.
    std::set<long long> support_z;
    const auto key = [](double z) { return (long long)std::llround(z * 1e4); };
    for (size_t i = 0; i < facts.moves.size(); ++ i) {
        const Move &move = facts.moves[i];
        if (move.type != EMoveType::Extrude)
            continue;
        const double z = double(move.position.z());
        if (z < fine + 1e-3 && int(move.extruder_id) == petg)
            ++first_layer_petg;
        if (!is_support(move))
            continue;
        support_z.insert(key(z));
        lowest_support = std::min(lowest_support, z);
        if (z < bed_z + 1e-3) {
            ++bed_roads;
            bed_thin += coarse_interface && double(move.width) < 1.25 * nozzles.coarse - 1e-3;
            bed_fast += double(move.feedrate) > bed_speed + 1e-3;
        }
        const int tool = k_filament_map[move.extruder_id] - 1;
        ++roads[tool];
        if (move.extrusion_role == erSupportMaterial && i > 0)
            body_volume[tool] += double((move.position - facts.moves[i - 1].position).head<2>().norm()) * double(move.width) * double(move.height);
        thinnest[tool] = std::min(thinnest[tool], double(move.width));
        widest[tool]   = std::max(widest[tool], double(move.width));
        if (double(move.width) < 0.75 * nozzle[tool] - 1e-3 || double(move.height) > double(move.width) + 1e-3) {
            if (narrow < 3)
                what << "[width " << move.width << " on the " << nozzle[tool] << " at Z " << z
                     << ", height " << move.height << ", role " << int(move.extrusion_role) << "] ";
            ++narrow;
        }
    }
    // Stock lays one dense layer in the base's material between a dissimilar interface and the body: under
    // a top contact's interface, over a bottom contact's. The nozzle that lays the interface lays it too.
    // At each interface-filament layer, the next support level up and down that has support over the
    // interface's footprint (more than part there) and no interface filament must be covered densely, in
    // base material on the interface's nozzle; sparse body covers a quarter or less.
    std::map<long long, BoundingBoxf> petg_box;
    std::map<long long, double> petg_area;
    if (petg >= 0)
        for (size_t i = 1; i < facts.moves.size(); ++ i) {
            const Move &move = facts.moves[i];
            if (move.type == EMoveType::Extrude && int(move.extruder_id) == petg && move.extrusion_role == erSupportMaterialInterface) {
                const Vec3f &from = facts.moves[i - 1].position;
                BoundingBoxf &box = petg_box[key(move.position.z())];
                box.merge(Vec2d(from.x(), from.y()));
                box.merge(Vec2d(move.position.x(), move.position.y()));
                petg_area[key(move.position.z())] += double((move.position - from).norm()) * double(move.width);
            }
        }
    // Area of roads at level z over box: support, base material on the interface's nozzle, and part.
    struct Areas { double support = 0., base = 0., part = 0.; };
    const auto areas_at = [&](long long z, const BoundingBoxf &box) {
        Areas out;
        for (size_t i = 1; i < facts.moves.size(); ++ i) {
            const Move &move = facts.moves[i];
            if (move.type != EMoveType::Extrude || key(move.position.z()) != z)
                continue;
            const Vec3f &from = facts.moves[i - 1].position;
            if (!box.contains(Vec2d(0.5 * (from.x() + move.position.x()), 0.5 * (from.y() + move.position.y()))))
                continue;
            const double area = double((move.position - from).norm()) * double(move.width);
            if (CadenceTest::model_road(move))
                out.part += area;
            if (!is_support(move))
                continue;
            out.support += area;
            if (move.extrusion_role == erSupportMaterial && int(move.extruder_id) != petg &&
                k_filament_map[move.extruder_id] == k_filament_map[petg])
                out.base += area;
        }
        return out;
    };
    size_t floors_checked = 0, floors_missing = 0;
    for (const auto &[z, box] : petg_box) {
        const auto it = support_z.find(z);
        for (const long long neighbour : {it == support_z.begin() ? -1LL : *std::prev(it),
                                          std::next(it) == support_z.end() ? -1LL : *std::next(it)}) {
            if (neighbour < 0 || petg_box.count(neighbour) != 0)
                continue;
            const Areas areas = areas_at(neighbour, box);
            if (areas.support < 0.05 * petg_area.at(z) || areas.part > areas.support)
                continue;
            ++floors_checked;
            if (areas.base < 0.6 * petg_area.at(z)) {
                if (floors_missing < 3)
                    what << "[interface at Z " << z * 1e-4 << ": base material at Z " << neighbour * 1e-4 << " covers "
                         << areas.base << " mm2 against the interface's " << petg_area.at(z) << "] ";
                ++floors_missing;
            }
        }
    }
    what << "lowest support road at Z " << lowest_support << ", bed-layer roads " << bed_roads;
    for (int tool : {0, 1})
        what << ", " << roads[tool] << " roads on the " << nozzle[tool] << " " << thinnest[tool] << "-" << widest[tool];
    bool ok = true;
    if (std::abs(lowest_support - bed_z) > 1e-3 || bed_roads == 0) {
        what << ", support's first row is not at Z " << bed_z;
        ok = false;
    }
    if (bed_thin != 0 || bed_fast != 0) {
        what << ", " << bed_thin << " first-row roads under the coarse first-layer width, " << bed_fast
             << " faster than the first-layer speed " << bed_speed;
        ok = false;
    }
    if (narrow != 0) {
        what << ", " << narrow << " support roads narrower than 0.75 x their nozzle or taller than wide";
        ok = false;
    }
    if (first_layer_petg != 0) {
        what << ", " << first_layer_petg << " interface-filament roads on layer 1";
        ok = false;
    }
    // Feature Split, a coarse base under a fine interface: the coarse nozzle lays most of the body.
    if (mode == Mode::Feature && ! coarse_interface && body_volume[1] <= body_volume[0]) {
        what << ", support body " << body_volume[1] << " mm3 on the coarse nozzle against " << body_volume[0] << " mm3 on the fine one";
        ok = false;
    }
    if (petg >= 0 && (floors_checked == 0 || floors_missing != 0)) {
        what << ", " << floors_missing << " of " << floors_checked << " interface layers without a dense base-material layer beside them";
        ok = false;
    }
    return ok ? std::string() : what.str();
}

// The handy Benchy (tree roots thinner than a coarse road) with a coarse PLA base and a PETG interface on the fine
// nozzle, on a first layer the coarse nozzle can lay: every support road is 0.75 to 1.5 times its own nozzle wide.
std::string first_layer_widths(const NozzlePair &c, const SupportCase &support)
{
    std::ostringstream what;
    what << c.name << " / " << support.name << ": ";
    CadenceTest::Scene scene;
    scene.config = h2d_support_config(Mode::Feature, c.fine, c.coarse, int(std::lround(c.coarse / c.fine)), 0., support,
                                      FilamentCase{"coarse PLA base, PETG interface on fine", 2, 3, "PETG"});
    scene.config.set_key_value("support_bottom_z_distance", new ConfigOptionFloat(0.1));
    scene.config.set_key_value("support_interface_bottom_layers", new ConfigOptionInt(2));
    scene.config.set_key_value("raft_first_layer_expansion", new ConfigOptionFloat(2.));
    set_pair(scene.config, c);
    scene.config.set_key_value("line_width", new ConfigOptionFloatOrPercent(1.1 * c.fine_nozzle, false));
    scene.config.set_key_value("initial_layer_line_width", new ConfigOptionFloatOrPercent(1.25 * c.fine_nozzle, false));
    scene.config.set_key_value("support_line_width", new ConfigOptionFloatOrPercent(1.1 * c.fine_nozzle, false));
    scene.config.set_key_value("tree_support_wall_count", new ConfigOptionInt(0));
    set_automatic_tower(scene.config);
    scene.config.set_key_value("prime_tower_width", new ConfigOptionFloat(60.));
    scene.populate = [](Model &model, Print &print, const DynamicPrintConfig &cfg) {
        ModelObject *object = model.add_object();
        object->name = "benchy";
        handy_model("3DBenchy.drc")(*object);
        // A small tab just above the bed beside the hull, whose tree root on the bed is thinner than a coarse road.
        TriangleMesh tab = make_cube(1.2, 1.2, 0.6);
        tab.translate(-1.2f, 0.f, 1.2f);
        TriangleMesh post = make_cube(4., 4., 3.);
        post.translate(0.f, -1.4f, 0.f);
        post.merge(tab);
        post.translate(-12.f, 20.f, 0.f);
        object->add_volume(std::move(post), ModelVolumeType::MODEL_PART, false);
        object->instances.front()->set_offset(Vec3d(80., 80., 0.));
        object->ensure_on_bed();
        print.apply(model, cfg);
        print.set_status_silent();
    };
    CadenceTest::Facts facts;
    if (std::string failure = slice_into(scene, what.str(), facts); !failure.empty())
        return failure;
    const double nozzle[2] = {c.fine_nozzle, c.coarse_nozzle};
    size_t roads = 0, coarse_roads = 0, wrong = 0;
    for (const Move &move : facts.moves) {
        if (!is_support(move))
            continue;
        const int tool = std::clamp(int(move.physical_tool_id), 0, 1);
        ++roads;
        coarse_roads += tool == 1;
        if (double(move.width) > 1.5 * nozzle[tool] + 1e-3 || double(move.width) < 0.75 * nozzle[tool] - 1e-3) {
            if (wrong < 3)
                what << "[width " << move.width << " on the " << nozzle[tool] << " at Z " << move.position.z() << "] ";
            ++wrong;
        }
    }
    if (roads == 0 || coarse_roads == 0 || wrong != 0) {
        what << roads << " support roads, " << coarse_roads << " on the coarse nozzle, " << wrong << " too wide or too narrow for their nozzle";
        return what.str();
    }
    if (std::string ahead = tower_ahead_failure(facts.gcode, c.coarse_min); !ahead.empty())
        return what.str() + ahead;
    return {};
}
} // namespace

TEST_CASE("Every support road stays within its own nozzle's widths, and the support's bed layer prints on layer 1", "[TestRebuild][Support]")
{
    std::vector<std::string> failures;
    // A first layer the coarse nozzle can lay, on the Benchy.
    for (const NozzlePair &c : {NozzlePair{"0.2/0.6, first layer 0.12", 0.2, 0.6, 0.04, 0.14, 0.12, 0.42, 0.10, 0.12, 0.40},
                                NozzlePair{"0.2/0.6, first layer 0.14", 0.2, 0.6, 0.04, 0.14, 0.12, 0.42, 0.10, 0.14, 0.40},
                                NozzlePair{"0.4/0.8, 0.12 layers, first layer 0.20", 0.4, 0.8, 0.08, 0.28, 0.16, 0.56, 0.12, 0.20, 0.36}})
        for (const SupportCase &support : {k_normal, k_organic})
            failures.push_back(first_layer_widths(c, support));
    // A 0.10 first layer the coarse nozzle cannot lay: the support's first layer still prints on layer 1, and no nozzle
    // lays a support road much narrower than its own face. With base and interface both on the coarse nozzle there is
    // no fine support filament: the first row stays the coarse nozzle's first legal height from the bed, laid at its
    // own first-layer width and speed.
    for (const SupportPlacement where : {SupportPlacement::PetgOnFine, SupportPlacement::AllCoarse, SupportPlacement::PetgOnCoarse})
        for (const PartShape part_shape : {PartShape::L, PartShape::Step}) {
            for (const CoarseNozzle &nozzles : {CoarseNozzle{0.6, 0.12, 0.42}, CoarseNozzle{0.8, 0.16, 0.56}}) {
                failures.push_back(coarse_base_case(Mode::Feature, nozzles, k_normal, where, part_shape));
                // Organic trees band their body too when the interface is on the fine nozzle.
                if (where == SupportPlacement::PetgOnFine)
                    failures.push_back(coarse_base_case(Mode::Feature, nozzles, k_organic, where, part_shape));
            }
            // A plain object on a Body Split plate: with the interface on the 0.2 its support body has no coarse
            // cadence to band to, so the 0.2 lays it; with both support filaments on the 0.6 the 0.6 lays all of it.
            failures.push_back(coarse_base_case(Mode::Body, CoarseNozzle{0.6, 0.12, 0.42}, k_normal, where, part_shape));
        }
    check_no_failures(failures);
}

// ---- Rafts ---------------------------------------------------------------------------------------

namespace {
// What a coarse raft case is made of: the support filaments' placement, the support style and the raft layers.
struct RaftCase {
    CoarseNozzle   nozzles;
    SupportCase    support;
    FilamentCase   filaments;
    int            raft_layers = 2;
    bool           support_on = true;
    bool           skirt = false;
    // No top Z distance, as stock suggests for a dissimilar interface: the part stands right on the raft.
    bool           zero_gap = false;
};

// Slices the L part (support from the bed under its arm) on a raft whose base is PLA on the coarse nozzle, under
// Feature Split, 0.10 / 0.40 with a 0.10 first layer, and returns what is wrong, or an empty string.
std::string coarse_raft_case(const RaftCase &rc)
{
    std::ostringstream what;
    what << "0.2/" << rc.nozzles.coarse << " / " << (rc.support_on ? rc.support.name : "no support") << " / "
         << rc.filaments.name << " / raft " << rc.raft_layers << (rc.skirt ? " / skirt" : "") << (rc.zero_gap ? " / no gap" : "") << ": ";
    const double fine = 0.10;
    DynamicPrintConfig config = h2d_support_config(Mode::Feature, fine, 0.40, 4, 0., rc.support, rc.filaments);
    set_nozzles(config, 0.2, rc.nozzles.coarse, 0.04, 0.14, rc.nozzles.coarse_min, rc.nozzles.coarse_max);
    config.set_key_value("support_bottom_z_distance", new ConfigOptionFloat(0.1));
    config.set_key_value("support_top_z_distance", new ConfigOptionFloat(rc.zero_gap ? 0. : 0.1));
    config.set_key_value("raft_contact_distance", new ConfigOptionFloat(0.1));
    set_fine_widths(config);
    config.set_key_value("raft_layers", new ConfigOptionInt(rc.raft_layers));
    config.set_key_value("enable_support", new ConfigOptionBool(rc.support_on));
    config.set_key_value("skirt_loops", new ConfigOptionInt(rc.skirt ? 2 : 0));
    set_automatic_tower(config);
    fill_per_filament_values(config);
    config.set_key_value("machine_start_gcode", new ConfigOptionString("G28\nM1020 S[initial_extruder]\n"));
    Model model;
    Print print;
    print.is_BBL_printer() = true;
    add_object(model, "coarse-raft-part", overhang_shelf_mesh(20., 60., 40., 50.), Vec3d(80., 80., 0.));
    print.apply(model, config);
    print.set_status_silent();
    CadenceTest::Facts facts;
    if (std::string failure = slice_into(print, what.str(), facts); !failure.empty())
        return failure;
    const SlicingParameters &slicing = print.objects().front()->slicing_parameters();
    const double nozzle[2]  = {0.2, rc.nozzles.coarse};
    const double minimum[2] = {0.04, rc.nozzles.coarse_min};
    const double maximum[2] = {0.14, rc.nozzles.coarse_max};
    // The raft's bed layer is laid by the base filament, or by the interface filament when the raft is one layer,
    // at a height its nozzle can lay: the first layer's, or that nozzle's thinnest when that is thicker.
    const int    base_tool  = k_filament_map[(rc.raft_layers == 1 ? rc.filaments.interface_filament : rc.filaments.base) - 1] - 1;
    const int    petg       = std::string(rc.filaments.interface_type) == "PETG" ? rc.filaments.interface_filament - 1 : -1;
    const double bed_z = std::max(fine, minimum[base_tool]);
    double lowest = 1e9, lowest_tower = 1e9, first_model_bottom = 1e9;
    size_t bed_roads = 0, bed_wrong = 0, thin = 0, petg_body = 0, petg_low = 0, contact_roads = 0, contact_wrong = 0;
    size_t skirt_roads = 0, skirt_wrong = 0;
    std::set<int> bed_nozzles;
    for (const Move &move : facts.moves) {
        if (move.type != EMoveType::Extrude || move.delta_extruder <= 0.f)
            continue;
        const double z = double(move.position.z());
        const int tool = k_filament_map[move.extruder_id] - 1;
        lowest = std::min(lowest, z);
        if (move.extrusion_role == erWipeTower)
            lowest_tower = std::min(lowest_tower, z);
        if (CadenceTest::model_road(move))
            first_model_bottom = std::min(first_model_bottom, z - double(move.height));
        if (move.extrusion_role == erSkirt) {
            ++skirt_roads;
            skirt_wrong += tool != base_tool || double(move.width) < nozzle[tool] - 1e-3;
        }
        if (z < bed_z + 1e-3) {
            bed_nozzles.insert(tool);
            if (is_support(move)) {
                ++bed_roads;
                if (tool != base_tool || std::abs(double(move.height) - bed_z) > 1e-3 || double(move.width) < nozzle[tool] - 1e-3)
                    ++bed_wrong;
            }
        }
        // Every road, the tower's included, is a height its own nozzle can lay. The tower's first layer is held to
        // that too: a raised first layer is one the nozzle that lays it can lay.
        if (double(move.height) < minimum[tool] - 1e-3 || double(move.height) > maximum[tool] + 1e-3) {
            if (thin < 3)
                what << "[" << move.height << " mm road on the " << nozzle[tool] << " at Z " << z << ", role " << int(move.extrusion_role) << "] ";
            ++thin;
        }
        if (int(move.extruder_id) == petg) {
            petg_body += move.extrusion_role == erSupportMaterial;
            petg_low  += z < slicing.raft_base_top_z + 1e-3;
        }
        if (std::abs(z - slicing.raft_contact_top_z) < 1e-3 && move.extrusion_role == erSupportMaterialInterface) {
            ++contact_roads;
            contact_wrong += k_filament_map[move.extruder_id] != k_filament_map[rc.filaments.interface_filament - 1];
        }
    }
    what << "first layer Z " << lowest << ", tower from Z " << lowest_tower << ", raft top " << slicing.raft_contact_top_z
         << ", part from Z " << first_model_bottom << ", raft bed roads " << bed_roads;
    bool ok = true;
    if (std::abs(lowest - bed_z) > 1e-3 || bed_roads == 0 || bed_wrong != 0) {
        what << ", raft bed layer not laid by the coarse nozzle at Z " << bed_z << " (" << bed_wrong << " roads otherwise)";
        ok = false;
    }
    if (bed_nozzles.size() != 1) {
        what << ", both nozzles on layer 1";
        ok = false;
    }
    // A raised first layer is said, as a warning on First layer height.
    const bool warned = std::any_of(facts.validation_warnings.begin(), facts.validation_warnings.end(), [](const StringObjectException &w) {
        return w.opt_key == "initial_layer_print_height";
    });
    if (warned != (bed_z > fine + 1e-3)) {
        what << (warned ? ", warns of a raised first layer" : ", no warning of the raised first layer");
        ok = false;
    }
    // The gap above the raft is rounded to a whole fine layer, and the export's empty-layer check allows for it.
    const std::vector<PrintStateBase::Warning> export_warnings = print.step_state_with_warnings(psGCodeExport).warnings;
    if (std::any_of(export_warnings.begin(), export_warnings.end(), [](const PrintStateBase::Warning &w) {
            return w.message_id == PrintStateBase::SlicingEmptyGcodeLayers; })) {
        what << ", empty-layer warning";
        ok = false;
    }
    if (rc.skirt && (skirt_roads == 0 || skirt_wrong != 0)) {
        what << ", " << skirt_wrong << " of " << skirt_roads << " skirt roads not laid by the raft's nozzle at its width";
        ok = false;
    }
    if (std::abs(lowest_tower - lowest) > 1e-3) {
        what << ", tower not on the first printed layer";
        ok = false;
    }
    if (thin != 0) {
        what << ", " << thin << " roads outside their nozzle's layer heights";
        ok = false;
    }
    if (std::abs(first_model_bottom - slicing.raft_contact_top_z - slicing.gap_raft_object) > 1e-3) {
        what << ", part does not stand " << slicing.gap_raft_object << " mm above the raft";
        ok = false;
    }
    if (rc.raft_layers > 1 && (contact_roads == 0 || contact_wrong != 0)) {
        what << ", raft contact layer not on the interface's nozzle (" << contact_wrong << " of " << contact_roads << ")";
        ok = false;
    }
    if (petg_body != 0 || petg_low != 0) {
        what << ", " << petg_body << " PETG body roads, " << petg_low << " PETG roads in the raft base";
        ok = false;
    }
    return ok ? std::string() : what.str();
}

// The L part on a raft, Feature Split 0.2/0.6, 0.10 / 0.40, PETG interface on the 0.2.
DynamicPrintConfig raft_reslice_config(int support_filament)
{
    DynamicPrintConfig config = h2d_support_config(Mode::Feature, 0.10, 0.40, 4, 0., k_normal, FilamentCase{"PETG interface", support_filament, 3, "PETG"});
    set_nozzles(config, 0.2, 0.6, 0.04, 0.14, 0.12, 0.42);
    config.set_key_value("support_top_z_distance", new ConfigOptionFloat(0.1));
    config.set_key_value("raft_contact_distance", new ConfigOptionFloat(0.1));
    set_fine_widths(config);
    config.set_key_value("raft_layers", new ConfigOptionInt(5));
    set_automatic_tower(config);
    fill_per_filament_values(config);
    config.set_key_value("machine_start_gcode", new ConfigOptionString("G28\nM1020 S[initial_extruder]\n"));
    return config;
}

// Where the part's first road starts (bottom of its lowest model road) against the raft top plus gap, after a
// slice of this print as it stands; empty when they agree.
std::string part_on_raft(Print &print, const std::string &what)
{
    CadenceTest::Facts facts;
    if (std::string failure = slice_into(print, what + ": ", facts); !failure.empty())
        return failure;
    const SlicingParameters &slicing = print.objects().front()->slicing_parameters();
    double part = 1e9, lowest = 1e9;
    for (const Move &move : facts.moves)
        if (move.type == EMoveType::Extrude && move.delta_extruder > 0.f) {
            lowest = std::min(lowest, double(move.position.z()));
            if (CadenceTest::model_road(move))
                part = std::min(part, double(move.position.z()) - double(move.height));
        }
    std::ostringstream out;
    if (std::abs(part - slicing.raft_contact_top_z - slicing.gap_raft_object) > 1e-3 ||
        std::abs(lowest - slicing.first_print_layer_height) > 1e-3)
        out << what << ": part from Z " << part << " on a raft whose top is " << slicing.raft_contact_top_z << " (gap "
            << slicing.gap_raft_object << "), first layer at Z " << lowest << " for " << slicing.first_print_layer_height;
    return out.str();
}
} // namespace

TEST_CASE("A raft with its base on the coarse nozzle starts on the bed at a height that nozzle lays", "[TestRebuild][Support][Raft]")
{
    const FilamentCase petg{"coarse PLA base, PETG interface on fine", 2, 3, "PETG"};
    const FilamentCase pla{"coarse PLA base, PLA interface on fine", 2, 3, "PLA"};
    const FilamentCase all_coarse{"PLA base and interface on coarse", 2, 2};
    std::vector<std::string> failures;
    for (const CoarseNozzle &nozzles : {CoarseNozzle{0.6, 0.12, 0.42}, CoarseNozzle{0.8, 0.16, 0.56}}) {
        for (const SupportCase &support : {k_normal, k_organic})
            for (const FilamentCase &filaments : {petg, pla})
                for (const int raft_layers : {2, 3, 5})
                    failures.push_back(coarse_raft_case(RaftCase{nozzles, support, filaments, raft_layers}));
        // A raft without support, a one-layer raft (its bed layer is the interface's), both support filaments on the
        // coarse nozzle, and a skirt, which goes with the raft's bed layer.
        failures.push_back(coarse_raft_case(RaftCase{nozzles, k_normal, petg, 3, false}));
        failures.push_back(coarse_raft_case(RaftCase{nozzles, k_normal, petg, 1}));
        failures.push_back(coarse_raft_case(RaftCase{nozzles, k_normal, all_coarse, 3}));
        failures.push_back(coarse_raft_case(RaftCase{nozzles, k_normal, petg, 2, true, true}));
        failures.push_back(coarse_raft_case(RaftCase{nozzles, k_normal, petg, 4, true, false, true}));
    }

    // Changing what the raft is laid with slices the part again at the raft's new height: the support/raft base on
    // the 0.2 (filament 1), then on the 0.6 (filament 2), then back, and the 0.6's layer height limits, each move the
    // raft's first layer and base levels.
    Model model;
    Print print;
    print.is_BBL_printer() = true;
    add_object(model, "raft-reslice", overhang_shelf_mesh(20., 60., 40., 50.), Vec3d(80., 80., 0.));
    for (const int base : {1, 2, 1}) {
        print.apply(model, raft_reslice_config(base));
        print.set_status_silent();
        failures.push_back(part_on_raft(print, "base on filament " + std::to_string(base)));
    }
    DynamicPrintConfig config = raft_reslice_config(2);
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, 0.16});
    print.apply(model, config);
    failures.push_back(part_on_raft(print, "0.6 minimum 0.16"));
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, 0.40});
    print.apply(model, config);
    failures.push_back(part_on_raft(print, "0.6 maximum 0.40"));
    check_no_failures(failures);
}

// ---- Interface filament against stock -----------------------------------------------------------------

namespace {
// Length of the support interface roads laid in the interface filament: on the first layer and in all.
struct InterfaceLength {
    double      first_layer = 0.;
    double      total       = 0.;
    size_t      layers      = 0;
    // "Z: mm" for every layer that has the interface filament.
    std::string by_layer;
    // Support layers where both nozzles lay body.
    size_t      two_body_layers = 0;
};

// A part overhanging from layer 2 and again 10 mm up, a coarse PLA base and a PETG interface on the fine nozzle at
// zero gap; mode Off is the stock print of the same settings. Requires a slice.
InterfaceLength interface_length_case(Mode mode, const CoarseNozzle &nozzles)
{
    const double fine = 0.10;
    const FilamentCase filaments{"coarse PLA base, PETG interface on fine", 2, 3, "PETG"};
    DynamicPrintConfig config = h2d_support_config(mode, fine, 0.40, 4, 0., k_normal, filaments);
    set_nozzles(config, 0.2, nozzles.coarse, 0.04, 0.14, nozzles.coarse_min, nozzles.coarse_max);
    set_fine_widths(config);
    set_automatic_tower(config);
    bambu_config(config);
    InterfaceLength out;
    CadenceTest::Facts facts;
    Model model;
    Print print;
    print.is_BBL_printer() = true;
    add_object(model, "overhang-from-layer-two",
               extruded_xz_outline({{0., 0.}, {20., 0.}, {20., 0.1}, {60., 0.1}, {60., 3.}, {40., 3.},
                                    {40., 10.}, {70., 10.}, {70., 13.}, {0., 13.}}, 20.),
               Vec3d(80., 80., 0.));
    print.apply(model, config);
    print.set_status_silent();
    const std::string refused = slice_into(print, std::string(mode_name(mode)) + ": ", facts);
    INFO(refused);
    REQUIRE(refused.empty());
    for (const SupportLayer *layer : print.objects().front()->support_layers())
        out.two_body_layers += ! layer->fine_body_fills.empty();
    std::map<int, double> layers;
    for (size_t i = 1; i < facts.moves.size(); ++i) {
        const Move &move = facts.moves[i];
        if (move.type != EMoveType::Extrude || move.extrusion_role != erSupportMaterialInterface ||
            int(move.extruder_id) != filaments.interface_filament - 1)
            continue;
        const double length = (move.position.head<2>() - facts.moves[i - 1].position.head<2>()).norm();
        out.total += length;
        if (double(move.position.z()) < fine + 1e-3)
            out.first_layer += length;
        layers[int(std::lround(double(move.position.z()) * 1000.))] += length;
    }
    out.layers = layers.size();
    std::ostringstream text;
    for (const auto &[z, length] : layers)
        text << "Z " << z / 1000. << ": " << length << " mm; ";
    out.by_layer = text.str();
    return out;
}
} // namespace

// Contact and interface areas are not grown with the coarse base road width: the interface filament covers about
// stock's area, the first layer included, and appears on no layer where stock lays none.
TEST_CASE("A coarse support base lays no more interface filament than the stock print of the same settings",
          "[TestRebuild][Support]")
{
    const CoarseNozzle nozzles = GENERATE(CoarseNozzle{0.6, 0.12, 0.42}, CoarseNozzle{0.8, 0.16, 0.56});
    CAPTURE(nozzles.coarse);
    const InterfaceLength stock = interface_length_case(Mode::Off, nozzles);
    const InterfaceLength split = interface_length_case(Mode::Feature, nozzles);
    CAPTURE(stock.first_layer, split.first_layer, stock.total, split.total, stock.layers, split.layers);
    INFO("stock: " << stock.by_layer);
    INFO("split: " << split.by_layer);
    // The scene: stock lays interface filament on the first layer too.
    REQUIRE(stock.first_layer > 100.);
    REQUIRE(stock.total > stock.first_layer);
    CHECK(split.first_layer <= 1.05 * stock.first_layer);
    // Support standing on the part starts with a body-filament layer from the interface nozzle, also on a band top.
    CHECK(split.total <= 1.05 * stock.total);
    CHECK(split.total >= 0.9 * stock.total);
    CHECK(split.layers <= stock.layers);
    // Normal support alone builds a layer where both nozzles lay body here.
    CAPTURE(split.two_body_layers);
    CHECK(split.two_body_layers > 0);
    CHECK(stock.two_body_layers == 0);
}

namespace {
// An Organic tree scene sliced under Feature Split with its base in PLA on the coarse nozzle and a PETG
// interface on the fine one, and its Off twin. Under a mode the base road is the coarse nozzle's
// (support_line_width 0.22 scaled by the nozzle ratio), and the tree is planned with that road, so the
// twin sets support_line_width to the same road to plan the same tree.
enum class TreeShape { L, Sphere, Bunny };

CadenceTest::Facts organic_tree_scene(Mode mode, const CoarseNozzle &nozzles, TreeShape shape)
{
    const double fine = 0.10;
    const FilamentCase filaments{"coarse PLA base, PETG interface on fine", 2, 3, "PETG"};
    const double coarse = nozzles.coarse > 0.7 ? 0.30 : 0.40;
    DynamicPrintConfig config = h2d_support_config(mode, fine, coarse, int(std::lround(coarse / fine)), 0.1, k_organic, filaments);
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.2, nozzles.coarse});
    config.set_key_value("mixed_nozzle_process_nozzle_diameters", new ConfigOptionFloats{0.2, nozzles.coarse});
    config.set_key_value("min_layer_height", new ConfigOptionFloats{0.04, nozzles.coarse_min});
    config.set_key_value("max_layer_height", new ConfigOptionFloats{0.14, nozzles.coarse_max});
    config.set_key_value("line_width", new ConfigOptionFloatOrPercent(0.22, false));
    config.set_key_value("initial_layer_line_width", new ConfigOptionFloatOrPercent(0.25, false));
    const double road = 0.22 * nozzles.coarse / 0.2;
    config.set_key_value("support_line_width", new ConfigOptionFloatOrPercent(mode == Mode::Off ? road : 0.22, false));
    // Stock asks for a tip at least as wide as the road.
    config.set_key_value("tree_support_tip_diameter", new ConfigOptionFloat(std::max(0.8, road + 0.12)));
    config.set_key_value("tree_support_wall_count", new ConfigOptionInt(0));
    set_automatic_tower(config);
    config.set_key_value("prime_tower_width", new ConfigOptionFloat(60.));
    config.set_key_value("printable_height", new ConfigOptionFloat(325.));
    CadenceTest::Scene scene;
    scene.config   = config;
    scene.populate = [shape](Model &model, Print &print, const DynamicPrintConfig &cfg) {
        ModelObject *object = model.add_object();
        object->name = "organic tree scene";
        if (shape == TreeShape::L)
            object->add_volume(overhang_shelf_mesh(20., 60., 40., 50.), ModelVolumeType::MODEL_PART, false);
        else if (shape == TreeShape::Sphere) {
            // A 15 mm ball resting on the bed, raised 4 mm on a thin stem, so trees stand under its whole underside.
            TriangleMesh ball = make_sphere(7.5, 2. * PI / 64.);
            ball.translate(0.f, 0.f, 11.5f);
            TriangleMesh stem = make_cube(2., 2., 5.);
            stem.translate(-1.f, -1.f, 0.f);
            ball.merge(stem);
            object->add_volume(std::move(ball), ModelVolumeType::MODEL_PART, false);
        } else
            handy_model("Stanford_Bunny.drc")(*object);
        if (object->instances.empty())
            object->add_instance();
        object->instances.front()->set_offset(Vec3d(80., 80., 0.));
        object->ensure_on_bed();
        print.apply(model, cfg);
        print.set_status_silent();
    };
    return CadenceTest::slice(scene);
}

// Support body region per 2 mm of height: the roads of each (height, road height) group drawn at their
// width, holes filled (organic branches are hollow loops), times the part of the road height in the bin.
// It does not depend on the road width.
std::map<int, double> support_region_bins(const std::vector<Move> &moves)
{
    std::map<std::tuple<long long, long long, long long>, Polylines> groups;
    for (size_t i = 1; i < moves.size(); ++ i) {
        const Move &move = moves[i];
        if (move.type != EMoveType::Extrude || move.extrusion_role != erSupportMaterial)
            continue;
        const Vec3f &from = moves[i - 1].position;
        groups[{std::llround(double(move.position.z()) * 1e4), std::llround(double(move.height) * 1e4), std::llround(double(move.width) * 1e4)}]
            .push_back(Polyline(Point::new_scale(from.x(), from.y()), Point::new_scale(move.position.x(), move.position.y())));
    }
    std::map<std::pair<long long, long long>, Polygons> covered;
    for (const auto &[key, lines] : groups)
        polygons_append(covered[{std::get<0>(key), std::get<1>(key)}], offset(lines, float(scale_(0.5e-4 * double(std::get<2>(key))))));
    std::map<int, double> bins;
    for (const auto &[key, polygons] : covered) {
        double area = 0.;
        for (const ExPolygon &expolygon : union_ex(polygons))
            area += expolygon.contour.area();
        // Spread over the 2 mm bins the roads' height span covers, so a band across a bin edge counts in both.
        const double top = key.first * 1e-4, bottom = top - key.second * 1e-4;
        for (int bin = int(std::floor(bottom / 2.)); bin * 2. < top - 1e-9; ++ bin)
            if (const double overlap = std::min(top, (bin + 1) * 2.) - std::max(bottom, bin * 2.); overlap > 0.)
                bins[bin] += area * SCALING_FACTOR * SCALING_FACTOR * overlap;
    }
    return bins;
}

// Share of the length of the given support roads whose plastic comes within 0.05 mm of a part road printed
// inside the support road's own height span.
double support_touching_part_share(const std::vector<Move> &moves, const std::function<bool(const Move &)> &which)
{
    struct Segment { Vec2d a, b; double half_width; };
    const double cell = 2.;
    const auto cell_of = [cell](const Vec2d &p) { return std::make_pair(long(std::floor(p.x() / cell)), long(std::floor(p.y() / cell))); };
    std::map<long long, std::map<std::pair<long, long>, std::vector<Segment>>> part;
    for (size_t i = 1; i < moves.size(); ++ i) {
        const Move &move = moves[i];
        if (! CadenceTest::model_road(move))
            continue;
        const Segment segment{moves[i - 1].position.head<2>().cast<double>(), move.position.head<2>().cast<double>(), 0.5 * double(move.width)};
        auto &grid = part[std::llround(double(move.position.z()) * 1e4)];
        const double length = (segment.b - segment.a).norm();
        const int n = std::max(1, int(length / cell) + 1);
        std::set<std::pair<long, long>> cells;
        for (int k = 0; k <= n; ++ k)
            cells.insert(cell_of(segment.a + (segment.b - segment.a) * (double(k) / n)));
        for (const auto &c : cells)
            grid[c].push_back(segment);
    }
    const auto distance = [](const Vec2d &p, const Segment &s) {
        const Vec2d d = s.b - s.a;
        const double l2 = d.squaredNorm();
        const double t = l2 > 0. ? std::clamp((p - s.a).dot(d) / l2, 0., 1.) : 0.;
        return (p - (s.a + t * d)).norm();
    };
    double bad = 0., total = 0.;
    for (size_t i = 1; i < moves.size(); ++ i) {
        const Move &move = moves[i];
        if (move.type != EMoveType::Extrude || ! which(move))
            continue;
        const Vec2d a = moves[i - 1].position.head<2>().cast<double>(), b = move.position.head<2>().cast<double>();
        const double length = (b - a).norm();
        total += length;
        const long long top = std::llround(double(move.position.z()) * 1e4), bottom = std::llround(double(move.position.z() - move.height) * 1e4);
        const int n = std::max(1, int(length / 0.5));
        int hits = 0;
        for (int k = 0; k <= n; ++ k) {
            const Vec2d p = a + (b - a) * (double(k) / n);
            const auto c = cell_of(p);
            bool hit = false;
            for (auto it = part.upper_bound(bottom); it != part.end() && it->first <= top && ! hit; ++ it)
                for (long dx = -1; dx <= 1 && ! hit; ++ dx)
                    for (long dy = -1; dy <= 1 && ! hit; ++ dy)
                        if (auto found = it->second.find({c.first + dx, c.second + dy}); found != it->second.end())
                            for (const Segment &segment : found->second)
                                if (distance(p, segment) < 0.5 * double(move.width) + segment.half_width + 0.05) {
                                    hit = true;
                                    break;
                                }
            hits += hit;
        }
        bad += length * hits / (n + 1);
    }
    return total > 0. ? bad / total : 0.;
}

size_t interface_layer_count(const std::vector<Move> &moves)
{
    std::set<long long> heights;
    for (const Move &move : moves)
        if (move.type == EMoveType::Extrude && move.extrusion_role == erSupportMaterialInterface)
            heights.insert(std::llround(double(move.position.z()) * 1e4));
    return heights.size();
}
} // namespace

TEST_CASE("Organic trees on a coarse base match the stock tree of the same plan", "[TestRebuild][Support]")
{
    struct Scene { const char *name; CoarseNozzle nozzles; TreeShape shape; };
    for (const Scene &scene : {Scene{"L part, 0.2/0.6", CoarseNozzle{0.6, 0.12, 0.42}, TreeShape::L},
                               Scene{"ball, 0.2/0.6", CoarseNozzle{0.6, 0.12, 0.42}, TreeShape::Sphere},
                               Scene{"bunny, 0.2/0.8", CoarseNozzle{0.8, 0.16, 0.56}, TreeShape::Bunny}}) {
        CAPTURE(scene.name);
        const CadenceTest::Facts split = organic_tree_scene(Mode::Feature, scene.nozzles, scene.shape);
        INFO(split.refusal.string);
        REQUIRE(split.refusal.string.empty());
        const CadenceTest::Facts off = organic_tree_scene(Mode::Off, scene.nozzles, scene.shape);
        INFO(off.refusal.string);
        REQUIRE(off.refusal.string.empty());

        // The coarse nozzle lays support.
        size_t coarse_roads = 0;
        for (const Move &move : split.moves)
            coarse_roads += move.type == EMoveType::Extrude && move.extrusion_role == erSupportMaterial && move.physical_tool_id == 1;
        CHECK(coarse_roads > 0);

        // The same interface layers as stock.
        CHECK(interface_layer_count(split.moves) == interface_layer_count(off.moves));

        // The support region per 2 mm of height, and in all, against stock's.
        const std::map<int, double> ours = support_region_bins(split.moves), theirs = support_region_bins(off.moves);
        double total_ours = 0., total_theirs = 0.;
        std::ostringstream bins;
        size_t off_bins = 0;
        for (const auto &[bin, volume] : theirs) {
            total_theirs += volume;
            const auto it = ours.find(bin);
            const double mine = it == ours.end() ? 0. : it->second;
            bins << bin * 2 << " mm: " << mine << " / " << volume << "; ";
            // Bins with little support are dominated by where a tip or a root happens to fall. Stock plans
            // the tree with its interface road as wide as the base road, so its branches differ a little from
            // ours (the L part's worst bin is 18% over stock); against the same plan without banding the sweep
            // kept every bin within 3%.
            if (volume > 20. && std::abs(mine / volume - 1.) > 0.2)
                ++off_bins;
        }
        for (const auto &[bin, volume] : ours)
            total_ours += volume;
        INFO("region by 2 mm bin (ours / stock): " << bins.str());
        CAPTURE(total_ours, total_theirs);
        CHECK(off_bins == 0);
        CHECK(std::abs(total_ours / total_theirs - 1.) < 0.05);

        // Coarse band roads keep clear of the part over their whole height, as stock's roads do over theirs.
        const double coarse_touch = support_touching_part_share(split.moves, [](const Move &move) {
            return move.extrusion_role == erSupportMaterial && move.physical_tool_id == 1; });
        const double stock_touch = support_touching_part_share(off.moves, [](const Move &move) {
            return move.extrusion_role == erSupportMaterial; });
        CAPTURE(coarse_touch, stock_touch);
        CHECK(coarse_touch <= 1.5 * stock_touch + 0.002);
    }
}

// ---- Refusals ------------------------------------------------------------------------------------

namespace {
enum class Verdict { Refused, Slices, Validates, Clears };

// A plate and what it gets: refused with a code and setting (an empty code: refused while slicing, with no code of
// its own; an empty setting: refused while slicing, where no setting is named), sliced to G-code, past validation, or not refused for the given code.
struct RefusalRow {
    std::string        name;
    CadenceTest::Scene scene;
    Verdict            verdict;
    std::string        code    = {};
    std::string        opt_key = {};
};

struct Refusal { std::string message, code, opt_key; };

Refusal refusal_of(CadenceTest::Scene scene, bool validate_only)
{
    Refusal out;
    if (validate_only) {
        fill_per_filament_values(scene.config);
        Model model;
        Print print;
        print.is_BBL_printer() = scene.bambu;
        scene.populate(model, print, scene.config);
        const StringObjectException refused = print.validate();
        out.message = refused.string;
        out.opt_key = refused.opt_key;
    } else {
        try {
            const CadenceTest::Facts facts = CadenceTest::slice(scene);
            out.message = facts.refusal.string;
            out.opt_key = facts.refusal.opt_key;
        } catch (const std::exception &error) {
            out.message = error.what();
        }
    }
    out.code = code_of(out.message);
    return out;
}

// The coarse-base L part with a coarse PLA base (2) and a PETG interface (3) on the fine nozzle, whose other filament
// (1, the part's own) is soluble, or PETG. With LaysNone a plain cube that needs no support, with support on.
enum class FineBodyCase { OnlyInterface, PetgWalls, LaysNone };

CadenceTest::Scene fine_body_scene(const SupportCase &support, FineBodyCase fine_body)
{
    CadenceTest::Scene scene;
    scene.config = h2d_support_config(Mode::Feature, 0.10, 0.40, 4, 0., support, FilamentCase{"coarse PLA base, PETG interface on fine", 2, 3, "PETG"});
    set_nozzles(scene.config, 0.2, 0.6, 0.04, 0.14, 0.12, 0.42);
    if (fine_body == FineBodyCase::PetgWalls) {
        // Both fine filaments PETG, with the adhesion categories the printer profiles give PLA and PETG.
        scene.config.option<ConfigOptionStrings>("filament_type")->values[0] = "PETG";
        scene.config.set_key_value("filament_adhesiveness_category", new ConfigOptionInts{300, 100, 300, 100});
    } else
        scene.config.option<ConfigOptionBools>("filament_soluble")->values[0] = true;
    scene.populate = one_object("l-part", fine_body == FineBodyCase::LaysNone ? make_cube(20., 20., 20.) : overhang_shelf_mesh(20., 60., 40., 50.),
                                Vec3d(80., 80., 0.));
    return scene;
}

// Two cubes on rafts, Feature Split 0.2/0.6 at 0.10 / 0.40: one raft's base on the 0.6 (it starts on the bed at
// the 0.6's 0.12 mm), the other's on the 0.2.
CadenceTest::Scene two_raft_scene(double first_layer)
{
    CadenceTest::Scene scene;
    scene.config = h2d_support_config(Mode::Feature, 0.10, 0.40, 4, 0., k_normal, FilamentCase{"coarse PLA base", 2, 3});
    set_nozzles(scene.config, 0.2, 0.6, 0.04, 0.14, 0.12, 0.42);
    scene.config.set_key_value("raft_layers", new ConfigOptionInt(2));
    scene.config.set_key_value("initial_layer_print_height", new ConfigOptionFloat(first_layer));
    set_automatic_tower(scene.config);
    scene.populate = [](Model &model, Print &print, const DynamicPrintConfig &cfg) {
        add_object(model, "coarse-raft", make_cube(10., 10., 10.), Vec3d(60., 60., 0.));
        add_object(model, "fine-raft", make_cube(10., 10., 10.), Vec3d(100., 60., 0.))->config.set_key_value("support_filament", new ConfigOptionInt(1));
        print.apply(model, cfg);
        print.set_status_silent();
    };
    return scene;
}

// A 30 mm cube in the app project with no support: PLA walls on the 0.2, and sparse infill in PETG (slot 4, moved to
// the 0.8) or in the project's coarse PLA.
CadenceTest::Scene cube_in_app_project(bool petg_sparse)
{
    CadenceTest::Scene scene;
    scene.config = app_three_filament_config(Mode::Feature, 0.10, 0.30, 3, k_normal);
    scene.config.set_key_value("enable_support", new ConfigOptionBool(false));
    if (petg_sparse) {
        scene.config.set_key_value("filament_map", new ConfigOptionInts{1, 1, 1, 2});
        scene.config.set_key_value("sparse_infill_filament_id", new ConfigOptionInt(4));
    }
    scene.populate = one_object("cube", make_cube(30., 30., 12.), Vec3d(80., 80., 0.));
    return scene;
}
} // namespace

TEST_CASE("Support a plate cannot print is refused by code and setting, and the plate that can is accepted", "[TestRebuild][Support][Refusal]")
{
    std::vector<RefusalRow> rows;
    const CoarseNozzle nozzles{0.6, 0.12, 0.42};
    const auto name = [](Mode mode, const std::string &what) { return std::string(mode_name(mode)) + ": " + what; };
    // Tree support with its base on the coarse nozzle: Slim, Strong and Hybrid for their style, and any tree with its
    // interface on that nozzle too. Organic with a finer interface is accepted, and so are Snug and Grid, which a tree
    // draws as Organic.
    for (const Mode mode : {Mode::Feature, Mode::Body}) {
        const std::string tree_code = mode == Mode::Feature ? "SRL-F14" : "SRL-A53";
        for (const SupportCase &support : {k_slim, k_strong, k_hybrid})
            rows.push_back({name(mode, std::string(support.name) + " on a coarse base"),
                            coarse_base_scene(mode, nozzles, support, SupportPlacement::PetgOnFine), Verdict::Refused, tree_code, "support_style"});
        rows.push_back({name(mode, "organic with its interface on the coarse base's nozzle"),
                        coarse_base_scene(mode, nozzles, k_organic, SupportPlacement::AllCoarse), Verdict::Refused, tree_code,
                        "support_interface_filament"});
        for (const SupportCase &support : {k_organic, SupportCase{"tree snug", stTreeAuto, smsSnug}, SupportCase{"tree grid", stTreeAuto, smsGrid}})
            rows.push_back({name(mode, std::string(support.name) + " on a coarse base, interface on the fine nozzle"),
                            coarse_base_scene(mode, nozzles, support, SupportPlacement::PetgOnFine), Verdict::Validates});
    }
    // A raft whose base the coarse nozzle lays: refused on a Body Split plate, whose first layer the Body Split objects
    // share; Slim, Strong and Hybrid trees keep their own refusal on it. Two rafts that would start on the bed at
    // different heights are refused, and accepted on a first layer that raises neither.
    rows.push_back({name(Mode::Body, "raft on a coarse base"), coarse_base_scene(Mode::Body, nozzles, k_normal, SupportPlacement::PetgOnFine, 2),
                    Verdict::Refused, "SRL-A52", "support_filament"});
    rows.push_back({name(Mode::Feature, "slim tree on a coarse raft base"), coarse_base_scene(Mode::Feature, nozzles, k_slim, SupportPlacement::PetgOnFine, 2),
                    Verdict::Refused, "SRL-F14", "support_style"});
    rows.push_back({name(Mode::Feature, "rafts starting at 0.12 and 0.10"), two_raft_scene(0.10), Verdict::Refused, "SRL-F16", "raft_layers"});
    rows.push_back({name(Mode::Feature, "rafts on a 0.12 mm first layer"), two_raft_scene(0.12), Verdict::Clears, "SRL-F16"});
    // Slim, Strong and Hybrid trees with both support filaments on the fine nozzle, whose thinnest layer (0.12) is above
    // the object's 0.10 layers: an Organic tree is laid on rows that nozzle can lay, the others cannot be.
    {
        DynamicPrintConfig config = h2d_support_config(Mode::Feature, 0.10, 0.30, 3, 0.1, k_slim, FilamentCase{"base and interface on the fine nozzle", 1, 3});
        config.set_key_value("min_layer_height", new ConfigOptionFloats{0.12, 0.16});
        for (const SupportCase &support : {k_slim, k_strong, k_hybrid}) {
            CadenceTest::Scene scene;
            scene.config = config;
            scene.config.set_key_value("support_style", new ConfigOptionEnum<SupportMaterialStyle>(support.style));
            scene.populate = [](Model &model, Print &print, const DynamicPrintConfig &cfg) { load_two_arm_model(model, print, cfg, Mode::Feature, 0.10, 0.30); };
            rows.push_back({name(Mode::Feature, std::string(support.name) + " on a nozzle that cannot lay the layers"), scene,
                            Verdict::Refused, "SRL-F14", "support_style"});
        }
    }
    // A coarse base whose fine nozzle lays body (the dense layer under the PETG, thin tree parts) with no PLA there:
    // only the PETG interface, or PETG walls, which do not stick to the PLA base. Refused while slicing, once the
    // support shows that body; a plate whose support lays no body on the fine nozzle slices.
    for (const SupportCase &support : {k_normal, k_organic})
        for (const FineBodyCase fine_body : {FineBodyCase::OnlyInterface, FineBodyCase::PetgWalls})
            rows.push_back({name(Mode::Feature, std::string(support.name) + (fine_body == FineBodyCase::PetgWalls ? ", PETG walls" : ", only the interface") +
                                 " on the fine nozzle"), fine_body_scene(support, fine_body), Verdict::Refused, "SRL-F15", ""});
    rows.push_back({name(Mode::Feature, "support that lays no fine body"), fine_body_scene(k_normal, FineBodyCase::LaysNone), Verdict::Slices});
    // PLA walls on the 0.2 and PETG sparse infill on the 0.8 share no material, so the tower cannot be built.
    rows.push_back({name(Mode::Feature, "PLA walls, PETG sparse infill"), cube_in_app_project(true), Verdict::Refused});
    rows.push_back({name(Mode::Feature, "PLA walls, PLA sparse infill"), cube_in_app_project(false), Verdict::Slices});

    for (const RefusalRow &row : rows) {
        const Refusal got = refusal_of(row.scene, row.verdict == Verdict::Validates || row.verdict == Verdict::Clears);
        CAPTURE(row.name, got.message, got.code, got.opt_key);
        if (row.verdict == Verdict::Clears)
            CHECK(got.code != row.code);
        else if (row.verdict != Verdict::Refused)
            CHECK(got.message.empty());
        else if (row.code.empty())
            CHECK_FALSE(got.message.empty());
        else {
            CHECK(got.code == row.code);
            CHECK(got.opt_key == row.opt_key);
        }
    }
}

TEST_CASE("The printer's G-code reads the nozzle diameter of the filament's own nozzle", "[TestRebuild][Support][Raft]")
{
    // A raft on the 0.6 makes filament 2 (on the 0.6) the first one; filament 3 (PETG) is on the 0.2.
    const std::string start  = "; ND_IDX {nozzle_diameter[initial_no_support_extruder]}\n; ND_LEGACY [nozzle_diameter]\n"
                               "; ND_BARE {nozzle_diameter}\nG28\nM1020 S[initial_extruder]\n";
    const std::string change = "; TC_NEXT {nozzle_diameter[next_extruder]} TC_LEGACY [nozzle_diameter] TC_BARE {nozzle_diameter}\n";
    const auto lines = [](const std::string &gcode, const std::string &tag) {
        std::vector<std::string> out;
        std::istringstream in(gcode);
        for (std::string line; std::getline(in, line);)
            if (line.rfind("; " + tag, 0) == 0)
                out.push_back(line);
        return out;
    };
    for (const bool mode : {true, false}) {
        DynamicPrintConfig config = raft_reslice_config(2);
        if (! mode)
            config.set_key_value("mixed_nozzle_slicing_mode", new ConfigOptionEnum<MixedNozzleSlicingMode>(MixedNozzleSlicingMode::Off));
        config.set_key_value("machine_start_gcode", new ConfigOptionString(start));
        config.set_key_value("change_filament_gcode", new ConfigOptionString(change));
        Model model;
        Print print;
        print.is_BBL_printer() = true;
        add_object(model, "l-part", overhang_shelf_mesh(20., 60., 40., 50.), Vec3d(80., 80., 0.));
        print.apply(model, config);
        print.set_status_silent();
        const CadenceTest::Facts facts = CadenceTest::slice(print);
        INFO(facts.refusal.string);
        REQUIRE(facts.refusal.string.empty());
        const std::vector<std::string> idx = lines(facts.gcode, "ND_IDX"), legacy = lines(facts.gcode, "ND_LEGACY"),
                                       bare = lines(facts.gcode, "ND_BARE"), change_lines = lines(facts.gcode, "TC_NEXT");
        REQUIRE(idx.size() == 1);
        REQUIRE(legacy.size() == 1);
        REQUIRE(bare.size() == 1);
        REQUIRE(! change_lines.empty());
        // The first filament is on the 0.6. Off keeps the printer's own reading: the nozzle vector indexed by
        // filament, as stock does.
        CHECK(idx.front() == "; ND_IDX 0.6");
        CHECK(legacy.front() == "; ND_LEGACY 0.6");
        if (mode) {
            // Every change names its own nozzle.
            CHECK(bare.front() == "; ND_BARE 0.6");
            size_t to_fine = 0, to_coarse = 0;
            for (const std::string &line : change_lines) {
                INFO(line);
                const bool fine = line == "; TC_NEXT 0.2 TC_LEGACY 0.2 TC_BARE 0.2";
                const bool coarse = line == "; TC_NEXT 0.6 TC_LEGACY 0.6 TC_BARE 0.6";
                CHECK((fine || coarse));
                to_fine += fine;
                to_coarse += coarse;
            }
            CHECK(to_fine > 0);
            CHECK(to_coarse > 0);
        }
    }
}

namespace {
// The tower's stack of roads by adhesion category on a 0.5 mm grid: a category that starts a run, stops for another
// and starts again at the same point stands on a material it does not bond with.
struct TowerStack {
    size_t      carrying    = 0; // points where a category other than the bottom one is laid
    size_t      interrupted = 0; // of those, points where such a category is laid in two or more separate runs
    std::string example;
};

TowerStack tower_material_stack(const std::vector<Move> &moves, const std::vector<int> &categories)
{
    struct Road { Vec2d a, b; long z; double half; int category; };
    std::vector<Road> roads;
    BoundingBoxf box;
    // With no change_filament_gcode (these scenes), the first road after a tool change starts where the last tool
    // stopped, a diagonal across the blocks the printer's own change G-code does not lay. It is left out.
    int last_tool = -1;
    for (size_t i = 1; i < moves.size(); ++i) {
        const Move &move = moves[i];
        if (move.type != EMoveType::Extrude || move.delta_extruder <= 0.f)
            continue;
        const bool first_after_change = last_tool != -1 && int(move.extruder_id) != last_tool;
        last_tool = int(move.extruder_id);
        if (move.extrusion_role != erWipeTower || first_after_change)
            continue;
        const int category = move.extruder_id < categories.size() ? categories[move.extruder_id] : -1;
        roads.push_back({moves[i - 1].position.head<2>().cast<double>(), move.position.head<2>().cast<double>(),
                         std::lround(double(move.position.z()) * 1000.), 0.5 * double(move.width), category});
        box.merge(roads.back().a);
        box.merge(roads.back().b);
    }
    TowerStack out;
    if (roads.empty())
        return out;

    std::map<std::pair<long, long>, std::vector<size_t>> grid;
    for (size_t r = 0; r < roads.size(); ++r) {
        const int steps = std::max(1, int((roads[r].b - roads[r].a).norm()) + 1);
        std::set<std::pair<long, long>> cells;
        for (int k = 0; k <= steps; ++k) {
            const Vec2d p = roads[r].a + (roads[r].b - roads[r].a) * (double(k) / steps);
            cells.insert({long(std::floor(p.x())), long(std::floor(p.y()))});
        }
        for (const auto &cell : cells)
            grid[cell].push_back(r);
    }
    for (double x = box.min.x(); x <= box.max.x(); x += 0.5)
        for (double y = box.min.y(); y <= box.max.y(); y += 0.5) {
            const Vec2d p(x, y);
            const auto found = grid.find({long(std::floor(x)), long(std::floor(y))});
            if (found == grid.end())
                continue;
            std::map<long, int> stack;
            for (size_t r : found->second)
                if (point_to_segment(p, roads[r].a, roads[r].b) <= roads[r].half)
                    stack[roads[r].z] = roads[r].category;
            std::vector<int> runs;
            for (const auto &[z, category] : stack)
                if (runs.empty() || runs.back() != category)
                    runs.push_back(category);
            std::map<int, int> starts;
            for (size_t k = 1; k < runs.size(); ++k)
                ++starts[runs[k]];
            bool other = false, broken = false;
            for (const auto &[category, count] : starts)
                if (category != runs.front()) {
                    other  = true;
                    broken = broken || count >= 2;
                }
            out.carrying += other;
            if (broken && out.interrupted++ == 0) {
                std::ostringstream text;
                text << "at " << x << ", " << y << ":";
                int last = -2;
                for (const auto &[z, category] : stack)
                    if (category != last) {
                        text << " Z " << double(z) / 1000. << " category " << category;
                        last = category;
                    }
                out.example = text.str();
            }
        }
    return out;
}
} // namespace

// A PETG interface (adhesion category 300) beside PLA (100) on the fine nozzle over a coarse PLA base: the lagging
// tower never lays PETG, PLA and PETG again in one block, where it could split. Off the tower PETG prints only the
// interface under the arm, a base filament lays the support on the bed, and the tower stays on the bed.
TEST_CASE("A PETG support interface beside PLA keeps its tower block on its own material, on every pair",
          "[TestRebuild][Tower][MaterialMix][Pairs]")
{
    struct StackPair { const char *name; double fine_d, coarse_d; bool swapped; double fine_h; int ratio; };
    const std::vector<StackPair> pairs{{"0.2/0.4", 0.2, 0.4, false, 0.10, 2}, {"0.2/0.6", 0.2, 0.6, false, 0.10, 4},
                                       {"0.2/0.6 fine right", 0.2, 0.6, true, 0.10, 4}, {"0.2/0.8", 0.2, 0.8, false, 0.10, 5},
                                       {"0.4/0.6", 0.4, 0.6, false, 0.20, 2}, {"0.4/0.8", 0.4, 0.8, false, 0.20, 2}};
    const std::vector<int> categories{100, 100, 300, 100};
    std::vector<std::string> failures;
    size_t sliced = 0;
    for (const Mode mode : {Mode::Feature, Mode::Body})
        for (const SupportCase &support : {k_normal, k_organic})
            for (const StackPair &pair : pairs) {
                // Body Split's tower lags only where the coarse nozzle's minimum is over the fine layer height.
                if (mode == Mode::Body && 0.2 * pair.coarse_d <= pair.fine_h + 1e-9)
                    continue;
                const double coarse_h = pair.fine_h * pair.ratio;
                const FilamentCase filaments{"PETG interface on fine", mode == Mode::Feature ? 2 : 1, 3, "PETG"};
                CadenceTest::Scene scene;
                scene.config = h2d_support_config(mode, pair.fine_h, coarse_h, pair.ratio, 0., support, filaments);
                const double first_d = pair.swapped ? pair.coarse_d : pair.fine_d, second_d = pair.swapped ? pair.fine_d : pair.coarse_d;
                set_nozzles(scene.config, first_d, second_d, 0.2 * first_d, 0.7 * first_d, 0.2 * second_d, 0.7 * second_d);
                if (pair.swapped)
                    scene.config.set_key_value("filament_map", new ConfigOptionInts{2, 1, 2, 1});
                scene.config.set_key_value("filament_adhesiveness_category", new ConfigOptionInts(categories));
                scene.config.set_key_value("support_interface_top_layers", new ConfigOptionInt(3));
                scene.config.set_key_value("support_interface_bottom_layers", new ConfigOptionInt(2));
                set_automatic_tower(scene.config);
                scene.populate = [mode, pair, coarse_h](Model &model, Print &print, const DynamicPrintConfig &cfg) {
                    ModelObject *object = add_object(model, "l-part", overhang_shelf_mesh(20., 60., 40., 50.), Vec3d(80., 80., 0.));
                    if (mode == Mode::Body) {
                        object->volumes.front()->config.set_key_value("extruder", new ConfigOptionInt(1));
                        object->volumes.front()->config.set_key_value("regional_layer_height", new ConfigOptionFloat(pair.fine_h));
                        add_part(*object, make_cube(12., 12., 45.), 2, coarse_h);
                        object->volumes.back()->set_offset(Vec3d(-14., 0., 0.));
                    }
                    print.apply(model, cfg);
                    print.set_status_silent();
                };
                const std::string name = std::string(mode_name(mode)) + " " + support.name + " " + pair.name + ": ";
                CadenceTest::Facts facts;
                if (std::string failure = slice_into(scene, name, facts); !failure.empty()) {
                    failures.push_back(failure);
                    continue;
                }
                ++sliced;
                // The L part's arm spans x 100..140 with its underside at 40 mm.
                const BoundingBoxf bed(scene.config.option<ConfigOptionPoints>("printable_area")->values);
                size_t petg_interface = 0, petg_tower = 0, petg_elsewhere = 0, bed_base = 0, tower_off_bed = 0;
                for (const Move &move : facts.moves) {
                    if (move.type != EMoveType::Extrude)
                        continue;
                    const double x = double(move.position.x()), z = double(move.position.z());
                    if (move.extrusion_role == erWipeTower)
                        tower_off_bed += !bed.contains(Vec2d(x, double(move.position.y())));
                    bed_base += move.extrusion_role == erSupportMaterial && move.extruder_id != 2 && z < pair.fine_h + 1e-3;
                    if (move.extruder_id != 2)
                        continue;
                    if (move.extrusion_role == erWipeTower)
                        ++petg_tower;
                    else if (move.extrusion_role == erSupportMaterialInterface && z > 39. && z < 40. + 1e-3 && x > 97. && x < 143.)
                        ++petg_interface;
                    else
                        ++petg_elsewhere;
                }
                const TowerStack stack = tower_material_stack(facts.moves, categories);
                std::ostringstream what;
                what << name << "PETG interface roads " << petg_interface << ", PETG tower roads " << petg_tower << ", PETG elsewhere "
                     << petg_elsewhere << ", support base roads on the bed " << bed_base << ", tower roads off the bed " << tower_off_bed
                     << ", points carrying another material " << stack.carrying << ", interrupted " << stack.interrupted << " " << stack.example;
                if (petg_interface == 0 || petg_tower == 0 || petg_elsewhere != 0 || bed_base == 0 || tower_off_bed != 0 || stack.interrupted != 0)
                    failures.push_back(what.str());
            }
    check_no_failures(failures);
    CHECK(sliced >= 18);
}
