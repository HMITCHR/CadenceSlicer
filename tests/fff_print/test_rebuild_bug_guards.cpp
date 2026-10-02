// Engine guards for reported bugs the build 29 test rebuild deleted, restored from the build 28 suite as
// adapted to build 31.

#include <catch2/catch_all.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/MixedNozzleConfig.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Semver.hpp"

#include "rebuild_harness.hpp"
#include "srl_fixtures.hpp"
#include "test_utils.hpp"

#include <boost/filesystem.hpp>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace srl_fixtures;

// OB-36 and OB-37. A private two-body Body Split project supplied by env var; skips when unset.
// The case slices it as saved at 0.08/0.24 (N=3) with beams on the object (count 3) and checks that
// beams land in both bodies, and that every applied unit reaches into both bodies.
TEST_CASE("OB-36 OB-37: Body Split beams land in both bodies of every applied unit on a private project",
          "[TestRebuild][OwnerGuard][BodyBeams]")
{
    const char *path = std::getenv("CADENCE_PRIVATE_BODY_SPLIT_3MF");
    if (path == nullptr || *path == '\0')
        SKIP("CADENCE_PRIVATE_BODY_SPLIT_3MF is not set");
    REQUIRE(boost::filesystem::exists(path));

    DynamicPrintConfig config;
    ConfigSubstitutionContext subs(ForwardCompatibilitySubstitutionRule::EnableSilent);
    Model model;
    ScopedTemporaryDir backup_dir("cadence_private_3mf");
    model.set_backup_path(backup_dir.string());
    {
        PlateDataPtrs plate_data;
        std::vector<Preset *> project_presets;
        bool is_bbl_3mf = false, is_orca_3mf = false;
        Semver file_version;
        const bool loaded = load_bbs_3mf(path, &config, &subs, &model, &plate_data, &project_presets,
                                         &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                                         LoadStrategy::AddDefaultInstances | LoadStrategy::LoadModel |
                                             LoadStrategy::LoadConfig);
        release_PlateData_list(plate_data);
        REQUIRE(loaded);
    }
    // The plate's manual map, as the app applies it at slice time: slot 6 on the right nozzle.
    config.set_key_value("filament_map", new ConfigOptionInts{1, 1, 1, 1, 1, 2, 1, 1});
    config.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->config.get().opt_bool("interlocking_beam"));
    REQUIRE(model.objects.front()->config.opt_int("interlocking_beam_layer_count") == 3);

    Print print;
    print.apply(model, config);
    // A Bambu Lab printer model marks the print as a BBL printer, which selects the native tower.
    const std::string printer_model = config.has("printer_model") ? config.opt_string("printer_model") : std::string();
    const std::string printer_name = config.has("printer_settings_id") ? config.opt_string("printer_settings_id") : std::string();
    print.is_BBL_printer() = printer_model.rfind("Bambu Lab", 0) == 0 ||
                             (printer_model.empty() && printer_name.rfind("Bambu Lab", 0) == 0);
    REQUIRE(print.is_BBL_printer());
    REQUIRE(print.wipe_tower_type() == WipeTowerType::Type1);
    print.set_status_silent();
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    REQUIRE_NOTHROW(print.process());

    const PrintObject &object = *print.objects().front();
    const NativeRegionalGridState &state = object.native_regional_grid_state();
    REQUIRE(object.pre_interlocking_slices().size() == state.lattice_planes.size());
    REQUIRE(state.lattice_ownership.size() == 2);
    const auto gains = [&](size_t row, size_t region) {
        return !diff_ex(state.lattice_ownership[region][row], object.pre_interlocking_slices()[row][region],
                        ApplySafetyOffset::Yes).empty();
    };

    size_t applied = 0;
    for (const RegionalInterlockingDecision &decision : state.interlocking_decisions)
        applied += decision.apply ? 1 : 0;
    CHECK(applied > 0);
    std::vector<bool> region_gains(2, false);
    for (size_t row = 0; row < state.lattice_planes.size(); ++row)
        for (size_t region = 0; region < 2; ++region)
            region_gains[region] = region_gains[region] || gains(row, region);
    CHECK(region_gains[0]);
    CHECK(region_gains[1]);

    // OB-37: an applied unit is an interlock only when both bodies reach into each other inside it.
    for (const RegionalInterlockingDecision &decision : state.interlocking_decisions) {
        if (!decision.apply)
            continue;
        INFO("applied unit " << decision.bottom_z << ".." << decision.top_z);
        std::vector<bool> unit_gains(2, false);
        for (size_t row = 0; row < state.lattice_planes.size(); ++row) {
            const coordf_t row_top = state.lattice_planes[row];
            const coordf_t row_bottom = row == 0 ? 0. : state.lattice_planes[row - 1];
            if (row_top <= decision.bottom_z + 1e-8 || row_bottom >= decision.top_z - 1e-8)
                continue;
            for (size_t region = 0; region < 2; ++region)
                unit_gains[region] = unit_gains[region] || gains(row, region);
        }
        CHECK(unit_gains[0]);
        CHECK(unit_gains[1]);
    }
}

namespace {

enum class ReprimeFixture { FeatureSplit, FeatureSplitAutoPad, BodySplitAutoPad };

struct ReprimeRun {
    std::vector<V24PrimePurge> purges;
    PrintConfig config;
    double tower_width {0.};
};

// One slice of the 0.2 / 0.4 tower coupon. A wall speed above zero slows every wall, which makes the
// other nozzle wait longer without changing the geometry or the switch plan.
ReprimeRun reprime_run(ReprimeFixture fixture, double wall_speed)
{
    DynamicPrintConfig config = fixture == ReprimeFixture::BodySplitAutoPad
        ? v24_body_split_pad_config(.20, .40, .10, .20, false, true, 60., wtwRib)
        : v24_prime_tower_config(.20, .40, false);
    if (fixture == ReprimeFixture::FeatureSplitAutoPad)
        config.set_key_value("mixed_nozzle_auto_pad", new ConfigOptionBool(true));
    if (wall_speed > 0.) {
        config.set_key_value("outer_wall_speed", new ConfigOptionFloatsNullable{wall_speed, wall_speed});
        config.set_key_value("inner_wall_speed", new ConfigOptionFloatsNullable{wall_speed, wall_speed});
    }
    Model model;
    Print print;
    if (fixture == ReprimeFixture::BodySplitAutoPad)
        init_v24_body_split_pad_coupon(model, print, config, false, 4.0, .10, .20);
    else
        init_tower_coupon(model, print, config);
    const CadenceTest::Facts facts = CadenceTest::slice(print);
    INFO(facts.refusal.string);
    REQUIRE(facts.refusal.string.empty());
    REQUIRE_FALSE(facts.gcode.empty());
    return ReprimeRun{scan_v24_arriving_purges(facts.gcode, print.config().filament_diameter.values),
                      print.config(), double(print.wipe_tower_data().width)};
}

// The hand-off prime a warm return starts from, at the final tower width.
double reprime_floor_mm3(const ReprimeRun &run, const V24PrimePurge &purge)
{
    PrintConfig config = run.config;
    if (run.tower_width > 0.)
        config.prime_tower_width.value = run.tower_width;
    const auto handoff = mixed_nozzle_handoff_deposits(config, size_t(purge.new_filament), size_t(purge.new_nozzle));
    REQUIRE(handoff.has_value());
    return double(handoff->prime_volume_mm3);
}

std::vector<V24PrimePurge> warm_returns(const ReprimeRun &run, int nozzle)
{
    std::vector<V24PrimePurge> out;
    for (const V24PrimePurge &purge : run.purges)
        if (purge.warm_return && (nozzle < 0 || purge.new_nozzle == nozzle))
            out.push_back(purge);
    return out;
}

double mean_mm3(const std::vector<V24PrimePurge> &purges)
{
    double sum = 0.;
    for (const V24PrimePurge &purge : purges)
        sum += purge.volume_mm3;
    return purges.empty() ? 0. : sum / double(purges.size());
}

} // namespace

// Guard E4. The V24Prime, V24Pad and long-wait Reprime cases bounded each warm return from above
// with the wait the engine used to publish (Print::mixed_nozzle_return_waits, removed in
// 0c7728b1ac). Without the wait, the bound that still holds is the one the idle re-prime saturates
// at, whatever the wait: the larger of the hand-off floor and the vendor prime, plus one row. The
// published-wait comparison (slow walls wait longer) is kept through what it caused: more prime.
TEST_CASE("Reprime: every warm return stays between its floor and the vendor prime, and a long wait gets more",
          "[TestRebuild][OwnerGuard][Reprime]")
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
        for (const V24PrimePurge &purge : warm) {
            REQUIRE(purge.new_filament >= 0);
            REQUIRE(purge.new_filament < int(run->config.filament_map.values.size()));
            REQUIRE(purge.new_nozzle >= 0);
            REQUIRE(purge.new_nozzle < 2);
            ++by_nozzle[size_t(purge.new_nozzle)];
            // The arriving nozzle follows the project's map.
            CHECK(purge.new_nozzle == run->config.filament_map.values[size_t(purge.new_filament)] - 1);
            const double floor_mm3 = reprime_floor_mm3(*run, purge);
            const double vendor_mm3 = run->config.filament_prime_volume.get_at(size_t(purge.new_filament));
            const double row = v24_prime_row_allowance_mm3(run->config, purge.new_nozzle,
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
    for (const V24PrimePurge &purge : slow_coarse) {
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
    for (const V24PrimePurge &purge : warm_returns(fast, -1))
        some_short_wait_below_vendor |=
            purge.volume_mm3 + 1. < fast.config.filament_prime_volume.get_at(size_t(purge.new_filament));
    CHECK(some_short_wait_below_vendor);
}
