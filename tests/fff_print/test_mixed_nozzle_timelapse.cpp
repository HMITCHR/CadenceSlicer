#include <catch2/catch_all.hpp>
#include "mixed_nozzle_harness.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/Utils.hpp"

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

std::unique_ptr<PlateSlice> slice_saved_plate(const std::string &path)
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
    SECTION("A saved plate with the farthest-point photo") {
        // The tiny-text fill plate (H2D 0.2/0.6, Traditional, farthest point on): its layers alternate nozzles, and
        // every photo is the same inline one, so the video does not jump between inline and parked shots.
        const auto sliced = slice_saved_plate(std::string(TEST_DATA_DIR) + "/body-split-tiny-text.3mf");
        INFO(sliced->facts.refusal.string);
        REQUIRE(sliced->facts.refusal.string.empty());
        REQUIRE(sliced->print.config().farthest_point_timelapse.value);
        REQUIRE(sliced->print.config().timelapse_type.value == tlTraditional);
        const PhotoStyles styles = photo_styles(sliced->facts.gcode);
        CAPTURE(styles.layers, styles.inline_photos, styles.parked, styles.parked_layers);
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
    SECTION("A fine body shorter than the coarse one, with and without the farthest-point photo") {
        // A fine layer between coarse layers ends on the fine nozzle; its photo is taken with that nozzle rather
        // than calling the coarse nozzle (the photo head) in, so there is no traditional timelapse warning.
        for (const bool farthest_point : {false, true}) {
            CAPTURE(farthest_point);
            const Facts facts = slice(body_split_short_fine_body(farthest_point));
            INFO(facts.refusal.string);
            REQUIRE(facts.refusal.string.empty());
            const std::vector<Photo> photos = photos_of(facts.gcode);
            REQUIRE_FALSE(photos.empty());
            CHECK_FALSE(warns_traditional_timelapse(facts));
            if (farthest_point)
                continue;
            // Without the farthest-point photo every photo is a parked one; none calls the other nozzle in, and
            // the fine layers between coarse layers keep their photo, taken with the fine nozzle.
            size_t other_nozzle = 0, fine_photos = 0;
            for (const Photo &photo : photos) {
                other_nozzle += photo.photo_nozzle != photo.printing_nozzle;
                fine_photos += photo.printing_nozzle == 0;
            }
            CHECK(other_nozzle == 0);
            CHECK(fine_photos > 0);
        }
    }
}
