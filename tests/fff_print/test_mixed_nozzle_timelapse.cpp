#include <catch2/catch_all.hpp>
#include "mixed_nozzle_harness.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Utils.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <regex>
#include <set>
#include <sstream>

using namespace Slic3r;
using namespace CadenceTest;
using namespace mixed_nozzle_fixtures;

namespace {
// Each photo names the nozzle that takes it (E) and the nozzle printing when it is taken (C).
const char *const photo_template = "; PHOTO E{most_used_physical_extruder_id} C{curr_physical_extruder_id}\n";

struct Photo { int photo_nozzle; int printing_nozzle; };

std::vector<Photo> photos_of(const std::string &gcode)
{
    std::vector<Photo> photos;
    static const std::regex line(R"(^; PHOTO E(\d+) C(\d+))");
    std::istringstream in(gcode);
    for (std::string text; std::getline(in, text);) {
        std::smatch match;
        if (std::regex_search(text, match, line))
            photos.push_back({std::stoi(match[1]), std::stoi(match[2])});
    }
    return photos;
}

bool warns_traditional_timelapse(const Facts &facts)
{
    return std::any_of(facts.warnings.begin(), facts.warnings.end(), [](const GCodeProcessorResult::SliceWarning &warning) {
        return warning.msg == NOT_SUPPORT_TRADITIONAL_TIMELAPSE;
    });
}

// A 0.2/0.4 Body Split pair where the coarse body is taller than the fine one: the coarse nozzle prints on
// more layers, so it is the photo head, and the fine layers between coarse layers end on the fine nozzle.
Scene body_split_short_fine_body(bool farthest_point)
{
    Scene scene;
    scene.config = body_split_pad_config(.2, .4, .1, .2, false, true, 60., wtwRib);
    scene.config.set_key_value("time_lapse_gcode", new ConfigOptionString(photo_template));
    scene.config.set_key_value("timelapse_type", new ConfigOptionEnum<TimelapseType>(tlTraditional));
    scene.config.set_key_value("farthest_point_timelapse", new ConfigOptionBool(farthest_point));
    scene.populate = [](Model &model, Print &print, const DynamicPrintConfig &config) {
        print.is_BBL_printer() = true;
        ModelObject *object = model.add_object();
        object->name = "short-fine-body";
        ModelVolume *fine = object->add_volume(make_cube(10., 20., 1.), ModelVolumeType::MODEL_PART, false);
        fine->config.set_key_value("extruder", new ConfigOptionInt(1));
        ModelVolume *coarse = object->add_volume(make_cube(20., 20., 4.), ModelVolumeType::MODEL_PART, false);
        coarse->set_offset(Vec3d(10., 0., 0.));
        coarse->config.set_key_value("extruder", new ConfigOptionInt(2));
        object->add_instance();
        object->instances.front()->set_offset(Vec3d(60., 60., 0.));
        object->ensure_on_bed();
        print.apply(model, config);
        print.set_status_silent();
        declare_native_cadences(model, print, config, .1, .2);
    };
    return scene;
}
// The H2D timelapse template's photo blocks: inline (M971 where the nozzle is) or parked (M9711).
struct PhotoStyles { size_t layers = 0, inline_photos = 0, parked = 0; std::vector<size_t> parked_layers; };

PhotoStyles photo_styles(const std::string &gcode)
{
    PhotoStyles styles;
    std::istringstream in(gcode);
    bool in_photo = false, parked = false;
    for (std::string text; std::getline(in, text);) {
        if (text.rfind("; CHANGE_LAYER", 0) == 0)
            ++styles.layers;
        else if (text.rfind("; SKIPTYPE: timelapse", 0) == 0)
            in_photo = true, parked = false;
        else if (in_photo && text.find("M9711") != std::string::npos)
            parked = true;
        else if (in_photo && text.rfind("; SKIPPABLE_END", 0) == 0) {
            in_photo = false;
            if (parked)
                ++styles.parked, styles.parked_layers.push_back(styles.layers);
            else
                ++styles.inline_photos;
        }
    }
    return styles;
}

// A saved plate, sliced as it is (its own printer, timelapse template and farthest-point setting).
struct PlateSlice { Model model; Print print; Facts facts; };

std::unique_ptr<PlateSlice> slice_saved_plate(const std::string &path, int farthest_point = -1, const std::string &overrides = {})
{
    auto out = std::make_unique<PlateSlice>();
    DynamicPrintConfig config;
    {
        ScopedTemporaryDir unpack("cadence-3mf");
        const std::string previous = temporary_dir();
        set_temporary_dir(unpack.string());
        ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Enable);
        // A project the app saved carries its own presets and plate settings; the importer needs somewhere to put them.
        PlateDataPtrs        plates;
        std::vector<Preset*> project_presets;
        bool                 is_bbl = false, is_orca = false;
        Semver               version;
        const bool loaded = load_bbs_3mf(path.c_str(), &config, &substitutions, &out->model, &plates, &project_presets, &is_bbl, &is_orca,
                                         &version, nullptr, LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
        set_temporary_dir(previous);
        if (!plates.empty())
            config.apply(plates.front()->config, true);
        release_PlateData_list(plates);
        for (Preset *preset : project_presets)
            delete preset;
        REQUIRE(loaded);
    }
    if (farthest_point >= 0)
        config.set_key_value("farthest_point_timelapse", new ConfigOptionBool(farthest_point != 0));
    // "key=value;key=value", measure only.
    for (std::stringstream list(overrides); list.good();) {
        std::string item;
        std::getline(list, item, ';');
        const size_t eq = item.find('=');
        if (eq != std::string::npos) {
            ConfigSubstitutionContext context(ForwardCompatibilitySubstitutionRule::Disable);
            config.set_deserialize(item.substr(0, eq), item.substr(eq + 1), context);
        }
    }
    // Another printer's timelapse template, measure only.
    if (const char *other = std::getenv("CADENCE_TL_TEMPLATE")) {
        std::ifstream file(other, std::ios::binary);
        config.set_key_value("time_lapse_gcode", new ConfigOptionString(std::string(std::istreambuf_iterator<char>(file), {})));
    }
    // Mark each photo with the nozzle it names and the nozzle printing (see photos_of).
    config.set_key_value("time_lapse_gcode", new ConfigOptionString(photo_template + config.opt_string("time_lapse_gcode")));
    out->print.is_BBL_printer() = true;
    out->print.apply(out->model, config);
    out->print.set_status_silent();
    out->facts = slice(out->print);
    return out;
}
} // namespace

TEST_CASE("Body Split takes every timelapse photo the same way, inline at the farthest point with the nozzle printing there",
          "[TestRebuild][Timelapse]")
{
    // Print 2 of the tiny-text fill plate (H2D 0.2/0.6, Traditional, farthest point on): 9 layers took the
    // inline photo with the 0.6 (head in the shot) and 8 parked a nozzle off the parts, so the video jumped
    // between the two. The 3 boxes of that plate are one box here; the same layers alternate nozzles.
    const auto sliced = slice_saved_plate(std::string(TEST_DATA_DIR) + "/body-split-tiny-text.3mf");
    INFO(sliced->facts.refusal.string);
    REQUIRE(sliced->facts.refusal.string.empty());
    REQUIRE(sliced->print.config().farthest_point_timelapse.value);
    REQUIRE(sliced->print.config().timelapse_type.value == tlTraditional);
    const PhotoStyles styles = photo_styles(sliced->facts.gcode);
    std::ostringstream parked;
    for (size_t layer : styles.parked_layers)
        parked << ' ' << layer;
    INFO("layers " << styles.layers << ", inline " << styles.inline_photos << ", parked " << styles.parked << " (layers" << parked.str() << ")");
    REQUIRE(styles.layers > 10);
    // One photo per layer, every one inline.
    CHECK(styles.parked == 0);
    CHECK(styles.inline_photos == styles.layers);
    CHECK_FALSE(warns_traditional_timelapse(sliced->facts));
    // Each photo names the nozzle taking it. The H2C template lifts the head and sends it to the chute
    // around a photo that names another nozzle, even an inline one.
    const std::vector<Photo> photos = photos_of(sliced->facts.gcode);
    CHECK(photos.size() == styles.inline_photos + styles.parked);
    size_t other_nozzle = 0;
    std::set<int> printing;
    for (const Photo &photo : photos) {
        other_nozzle += photo.photo_nozzle != photo.printing_nozzle;
        printing.insert(photo.printing_nozzle);
    }
    CHECK(other_nozzle == 0);
    // Both nozzles take photos on this plate.
    CHECK(printing.size() == 2);
}

TEST_CASE("Timelapse measure: saved plates to G-code files", "[.][TimelapseMeasure]")
{
    // CADENCE_TL_PLATES: 3MF paths separated by ':'. CADENCE_TL_OUT: output folder.
    // CADENCE_TL_FP: unset keeps the plate's farthest_point_timelapse, 0 or 1 overrides it.
    // CADENCE_TL_SET: "key=value;..." set on every plate, CADENCE_TL_TAG: added to the file names.
    // CADENCE_TL_TEMPLATE: a file whose text replaces the plates' timelapse template.
    const char *plates = std::getenv("CADENCE_TL_PLATES");
    const char *dir = std::getenv("CADENCE_TL_OUT");
    const char *fp = std::getenv("CADENCE_TL_FP");
    REQUIRE(plates != nullptr);
    REQUIRE(dir != nullptr);
    for (std::stringstream in(plates); in.good();) {
        std::string path;
        std::getline(in, path, ':');
        if (path.empty())
            continue;
        const char *set = std::getenv("CADENCE_TL_SET");
        const char *tag = std::getenv("CADENCE_TL_TAG");
        const auto sliced = slice_saved_plate(path, fp ? std::atoi(fp) : -1, set ? set : "");
        std::string stem = path.substr(path.find_last_of('/') + 1);
        stem = stem.substr(0, stem.find_last_of('.'));
        if (fp)
            stem += std::string("-fp") + fp;
        if (tag)
            stem += std::string("-") + tag;
        std::ofstream(std::string(dir) + "/" + stem + ".gcode", std::ios::binary) << sliced->facts.gcode;
        const PhotoStyles styles = photo_styles(sliced->facts.gcode);
        std::cout << stem << ": refusal '" << sliced->facts.refusal.string << "' seconds " << sliced->facts.seconds << " layers "
                  << styles.layers << " inline " << styles.inline_photos << " parked " << styles.parked << " warn "
                  << warns_traditional_timelapse(sliced->facts) << std::endl;
    }
}

TEST_CASE("Body Split takes each traditional timelapse photo with the nozzle already printing, so the send dialog has no timelapse warning",
          "[TestRebuild][Timelapse]")
{
    // Bambu's "traditional timelapse may cause surface imperfections" warning fired on every Body Split
    // plate: a fine layer between coarse layers ended on the fine nozzle and its photo called the
    // coarse nozzle (the photo head) in for it. A one-nozzle print on that nozzle takes its photo with it.
    const bool farthest_point = GENERATE(false, true);
    CAPTURE(farthest_point);
    const Facts facts = slice(body_split_short_fine_body(farthest_point));
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    const std::vector<Photo> photos = photos_of(facts.gcode);
    REQUIRE_FALSE(photos.empty());
    CHECK_FALSE(warns_traditional_timelapse(facts));
    if (! farthest_point) {
        // Without the farthest-point photo every photo is a parked one; none calls the other nozzle in.
        size_t other_nozzle = 0, fine_photos = 0;
        for (const Photo &photo : photos) {
            other_nozzle += photo.photo_nozzle != photo.printing_nozzle;
            fine_photos += photo.printing_nozzle == 0;
        }
        CHECK(other_nozzle == 0);
        // The fine layers between coarse layers keep their photo (taken with the fine nozzle).
        CHECK(fine_photos > 0);
    }
}

TEST_CASE("Feature Split with Smooth timelapse on a coarse nozzle that cannot lay the layer height is refused naming Timelapse",
          "[TestRebuild][Timelapse][Admission]")
{
    // Smooth keeps the prime tower level with the part, so the 0.6 nozzle (0.12 mm minimum) would lay
    // 0.1 mm tower layers. The slice used to fail late with an internal SRL-TOWER-STRUCTURE message.
    const auto scene_for = [](double coarse, TimelapseType type) {
        Scene scene;
        scene.config = feature_split_tower_config(.20, coarse, false);
        scene.config.set_key_value("timelapse_type", new ConfigOptionEnum<TimelapseType>(type));
        scene.populate = [](Model &model, Print &print, const DynamicPrintConfig &config) {
            init_tower_coupon(model, print, config, make_cube(30., 30., 2.));
        };
        return scene;
    };
    SECTION("0.2/0.6 with Smooth is refused up front") {
        const Facts facts = slice(scene_for(.60, tlSmooth));
        INFO(facts.refusal.string);
        REQUIRE(facts.refusal.string.find("SRL-F17") != std::string::npos);
        CHECK(facts.refusal.string.find("Timelapse is set to Smooth") != std::string::npos);
        CHECK(facts.refusal.string.find("Set Timelapse to Traditional") != std::string::npos);
        CHECK(facts.refusal.string.find("at least 0.12 mm") != std::string::npos);
        CHECK(facts.refusal.opt_key == "timelapse_type");
        CHECK(facts.refusal.string.find("_type") == std::string::npos);
    }
    SECTION("0.2/0.6 with Traditional still slices") {
        const Facts facts = slice(scene_for(.60, tlTraditional));
        INFO(facts.refusal.string);
        CHECK(facts.refusal.string.empty());
        CHECK_FALSE(facts.gcode.empty());
    }
    SECTION("0.2/0.4 with Smooth still slices: the 0.4 nozzle lays the 0.1 mm tower layers") {
        const Facts facts = slice(scene_for(.40, tlSmooth));
        INFO(facts.refusal.string);
        CHECK(facts.refusal.string.empty());
        CHECK_FALSE(facts.gcode.empty());
    }
}

TEST_CASE("Body Split's Smooth timelapse refusal names the Timelapse setting and the value to pick",
          "[TestRebuild][Timelapse][Admission]")
{
    Scene scene;
    scene.config = body_split_pad_config(.2, .6, .1, .2, false, true, 60., wtwRib);
    scene.config.set_key_value("timelapse_type", new ConfigOptionEnum<TimelapseType>(tlSmooth));
    scene.populate = [](Model &model, Print &print, const DynamicPrintConfig &config) {
        init_body_split_pad_coupon(model, print, config, false, 4., .1, .2);
    };
    const Facts facts = slice(scene);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.find("SRL-A38") != std::string::npos);
    CHECK(facts.refusal.string.find("Timelapse set to Smooth") != std::string::npos);
    CHECK(facts.refusal.string.find("Set Timelapse to Traditional (Others > Special mode)") != std::string::npos);
    CHECK(facts.refusal.opt_key == "timelapse_type");
}
