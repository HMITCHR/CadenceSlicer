// Engine guards for reported bugs.

#include <catch2/catch_all.hpp>

#include "libslic3r/libslic3r.h"
#include "libslic3r/MixedNozzleConfig.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Utils.hpp"

#include "mixed_nozzle_harness.hpp"
#include "mixed_nozzle_fixtures.hpp"
#include "test_utils.hpp"

#include <algorithm>
#include <array>
#include <clocale>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace mixed_nozzle_fixtures;

namespace {

enum class ReprimeFixture { FeatureSplit, FeatureSplitAutoPad, BodySplitAutoPad };

struct ReprimeRun {
    std::vector<ArrivingPurge> purges;
    PrintConfig config;
    double tower_width {0.};
};

// One slice of the 0.2 / 0.4 tower coupon. A wall speed above zero slows every wall, which makes the
// other nozzle wait longer without changing the geometry or the switch plan.
ReprimeRun reprime_run(ReprimeFixture fixture, double wall_speed)
{
    DynamicPrintConfig config = fixture == ReprimeFixture::BodySplitAutoPad
        ? body_split_pad_config(.20, .40, .10, .20, false, true, 60., wtwRib)
        : feature_split_tower_config(.20, .40, false);
    if (fixture == ReprimeFixture::FeatureSplitAutoPad)
        config.set_key_value("mixed_nozzle_auto_pad", new ConfigOptionBool(true));
    if (wall_speed > 0.) {
        config.set_key_value("outer_wall_speed", new ConfigOptionFloatsNullable{wall_speed, wall_speed});
        config.set_key_value("inner_wall_speed", new ConfigOptionFloatsNullable{wall_speed, wall_speed});
    }
    Model model;
    Print print;
    if (fixture == ReprimeFixture::BodySplitAutoPad)
        init_body_split_pad_coupon(model, print, config, false, 4.0, .10, .20);
    else
        init_tower_coupon(model, print, config);
    const CadenceTest::Facts facts = CadenceTest::slice(print);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    return ReprimeRun{scan_arriving_purges(facts.gcode, print.config().filament_diameter.values),
                      print.config(), double(print.wipe_tower_data().width)};
}

// The hand-off prime a warm return starts from, at the final tower width.
double reprime_floor_mm3(const ReprimeRun &run, const ArrivingPurge &purge)
{
    PrintConfig config = run.config;
    if (run.tower_width > 0.)
        config.prime_tower_width.value = run.tower_width;
    const auto handoff = mixed_nozzle_handoff_deposits(config, size_t(purge.new_filament), size_t(purge.new_nozzle));
    REQUIRE(handoff.has_value());
    return double(handoff->prime_volume_mm3);
}

std::vector<ArrivingPurge> warm_returns(const ReprimeRun &run, int nozzle)
{
    std::vector<ArrivingPurge> out;
    for (const ArrivingPurge &purge : run.purges)
        if (purge.warm_return && (nozzle < 0 || purge.new_nozzle == nozzle))
            out.push_back(purge);
    return out;
}

double mean_mm3(const std::vector<ArrivingPurge> &purges)
{
    double sum = 0.;
    for (const ArrivingPurge &purge : purges)
        sum += purge.volume_mm3;
    return purges.empty() ? 0. : sum / double(purges.size());
}

} // namespace

// Each warm return is bounded from above by what the idle re-prime saturates at, whatever the
// wait: the larger of the hand-off floor and the vendor prime, plus one row. That slow walls
// wait longer is checked through what the wait causes: more prime.
TEST_CASE("Reprime: every warm return stays between its floor and the vendor prime, and a long wait gets more",
          "[TestRebuild][BugGuard][Reprime]")
{
    const ReprimeFixture fixture = GENERATE(ReprimeFixture::FeatureSplit, ReprimeFixture::FeatureSplitAutoPad,
                                            ReprimeFixture::BodySplitAutoPad);
    CAPTURE(int(fixture));
    constexpr double rounding_mm3 = 0.02;
    constexpr int coarse_nozzle = 1;
    const ReprimeRun fast = reprime_run(fixture, -1.);
    const ReprimeRun slow = reprime_run(fixture, 2.);

    for (const ReprimeRun *run : {&fast, &slow}) {
        const auto warm = warm_returns(*run, -1);
        REQUIRE_FALSE(warm.empty());
        std::array<size_t, 2> by_nozzle{};
        for (const ArrivingPurge &purge : warm) {
            REQUIRE(purge.new_filament >= 0);
            REQUIRE(purge.new_filament < int(run->config.filament_map.values.size()));
            REQUIRE(purge.new_nozzle >= 0);
            REQUIRE(purge.new_nozzle < 2);
            ++by_nozzle[size_t(purge.new_nozzle)];
            // The arriving nozzle follows the project's map.
            CHECK(purge.new_nozzle == run->config.filament_map.values[size_t(purge.new_filament)] - 1);
            const double floor_mm3 = reprime_floor_mm3(*run, purge);
            const double vendor_mm3 = run->config.filament_prime_volume.get_at(size_t(purge.new_filament));
            const double row = prime_row_allowance_mm3(run->config, purge.new_nozzle,
                                                           std::max(purge.max_emitted_height, .20));
            CAPTURE(run == &slow, purge.print_z, purge.new_filament, purge.new_nozzle, purge.volume_mm3,
                    floor_mm3, vendor_mm3, row);
            CHECK(purge.volume_mm3 + rounding_mm3 >= floor_mm3);
            CHECK(purge.volume_mm3 <= std::max(floor_mm3, vendor_mm3) + row + rounding_mm3);
        }
        CHECK(by_nozzle[0] > 0);
        CHECK(by_nozzle[1] > 0);
    }

    // After a long wait the coarse nozzle gets the vendor's own prime for its filament.
    const auto slow_coarse = warm_returns(slow, coarse_nozzle);
    const auto fast_coarse = warm_returns(fast, coarse_nozzle);
    REQUIRE_FALSE(slow_coarse.empty());
    REQUIRE_FALSE(fast_coarse.empty());
    for (const ArrivingPurge &purge : slow_coarse) {
        const double vendor_mm3 = slow.config.filament_prime_volume.get_at(size_t(purge.new_filament));
        CAPTURE(purge.volume_mm3, vendor_mm3);
        REQUIRE(vendor_mm3 > reprime_floor_mm3(slow, purge));
        CHECK(purge.volume_mm3 + rounding_mm3 >= vendor_mm3);
    }
    // The same returns after short waits get less, and some short wait stays well under the vendor
    // prime: the prime follows the wait, not the switch.
    CAPTURE(mean_mm3(slow_coarse), mean_mm3(fast_coarse));
    CHECK(mean_mm3(slow_coarse) > mean_mm3(fast_coarse) + 1.);
    bool some_short_wait_below_vendor = false;
    for (const ArrivingPurge &purge : warm_returns(fast, -1))
        some_short_wait_below_vendor |=
            purge.volume_mm3 + 1. < fast.config.filament_prime_volume.get_at(size_t(purge.new_filament));
    CHECK(some_short_wait_below_vendor);
}

namespace {
// The app's C locale follows its language (Windows picks it from the regional format), so the
// slicing thread can run with a comma decimal mark. Returns the first comma-decimal locale this
// machine has, or nothing.
const char *comma_decimal_locale()
{
    const std::string previous = std::setlocale(LC_ALL, nullptr);
    const char *found = nullptr;
    for (const char *name : {"de_DE.UTF-8", "de_DE", "German_Germany.1252", "de-DE", "fr_FR.UTF-8", "fr_FR"})
        if (std::setlocale(LC_ALL, name) != nullptr) {
            char text[8] = "";
            std::snprintf(text, sizeof(text), "%.1f", 0.5);
            if (text[1] == ',') {
                found = name;
                break;
            }
        }
    std::setlocale(LC_ALL, previous.c_str());
    return found;
}

// Slices the project in the given C locale and returns its G-code without the line that names the
// time it was made, the progress lines and the estimate: organic trees move the estimate by a second
// or so from run to run, which shifts the M73 lines, in any language.
std::string slice_in_locale(const Model &model, const DynamicPrintConfig &config, const char *locale)
{
    Print print;
    print.is_BBL_printer() = true;
    print.apply(model, config);
    print.set_status_silent();
    ScopedTemporaryFile output(".gcode");
    const std::string previous = std::setlocale(LC_ALL, nullptr);
    if (locale != nullptr)
        std::setlocale(LC_ALL, locale);
    std::string refusal;
    try {
        refusal = print.validate().string;
        if (refusal.empty()) {
            print.process();
            GCodeProcessorResult processed;
            print.export_gcode(output.string(), &processed, nullptr);
        }
    } catch (...) {
        std::setlocale(LC_ALL, previous.c_str());
        throw;
    }
    std::setlocale(LC_ALL, previous.c_str());
    if (!refusal.empty())
        return "refused: " + refusal;
    std::ifstream input(output.string(), std::ios::binary);
    std::string gcode, line;
    while (std::getline(input, line))
        if (line.find("generated by") == std::string::npos && line.rfind("M73 ", 0) != 0 &&
            line.find("estimated time") == std::string::npos)
            gcode += line + '\n';
    return gcode;
}
} // namespace

TEST_CASE("A comma-decimal app language slices the same G-code as English", "[TestRebuild][BugGuard][Locale]")
{
    const char *comma = comma_decimal_locale();
    if (comma == nullptr)
        SKIP("no comma-decimal locale installed");
    // Projects without organic trees, which vary a little from run to run in any language.
    for (const std::string file : {"feature-split-L-tower-008.3mf", "body-split-block-thin-text.3mf"}) {
        CAPTURE(file, comma);
        // Loaded once, so both slices see the same object ids.
        DynamicPrintConfig config;
        const Model model = CadenceTest::load_project(file, config);
        const std::string english = slice_in_locale(model, config, nullptr);
        REQUIRE(english.rfind("refused: ", 0) != 0);
        const std::string german = slice_in_locale(model, config, comma);
        if (german != english) {
            std::istringstream a(english), b(german);
            std::string left, right;
            size_t line = 0, shown = 0;
            std::ostringstream first;
            while (shown < 8) {
                const bool more_a = bool(std::getline(a, left)), more_b = bool(std::getline(b, right));
                if (!more_a && !more_b)
                    break;
                ++line;
                if (left != right) {
                    first << "line " << line << ": \"" << left << "\" vs \"" << right << "\"\n";
                    ++shown;
                }
            }
            INFO(first.str());
            CHECK(german == english);
        }
    }
}

TEST_CASE("A project with both old Feature Split and Body Split switches on is refused when it loads",
          "[TestRebuild][BugGuard][MNS-MIG-A01]")
{
    // Older projects stored the two modes as two booleans. Loading turns one of them into
    // mixed_nozzle_slicing_mode and refuses a project that has both on.
    const auto load = [](const std::string &ini) {
        DynamicPrintConfig config;
        config.load_from_ini_string(ini, ForwardCompatibilitySubstitutionRule::Enable);
        return config;
    };
    SECTION("Both on") {
        CHECK_THROWS_WITH(load("mixed_nozzle_feature_split = 1\nsynchronized_multi_nozzle_layering = 1\n"),
                          Catch::Matchers::ContainsSubstring("MNS-MIG-A01"));
    }
    SECTION("One on loads as that mode and drops the old keys") {
        const auto mode = [](const DynamicPrintConfig &config) {
            const auto *opt = config.option<ConfigOptionEnum<MixedNozzleSlicingMode>>("mixed_nozzle_slicing_mode");
            REQUIRE(opt != nullptr);
            return opt->value;
        };
        const DynamicPrintConfig feature = load("mixed_nozzle_feature_split = 1\nsynchronized_multi_nozzle_layering = 0\n");
        CHECK(mode(feature) == MixedNozzleSlicingMode::FeatureSplit);
        CHECK_FALSE(feature.has("mixed_nozzle_feature_split"));
        CHECK_FALSE(feature.has("synchronized_multi_nozzle_layering"));
        const DynamicPrintConfig body = load("synchronized_multi_nozzle_layering = 1\n");
        CHECK(mode(body) == MixedNozzleSlicingMode::BodySplit);
        const DynamicPrintConfig off = load("mixed_nozzle_feature_split = 0\nsynchronized_multi_nozzle_layering = 0\n");
        CHECK(mode(off) == MixedNozzleSlicingMode::Off);
    }
}
