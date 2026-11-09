#pragma once

// Shared fixtures for mixed-nozzle tests: config and coupon builders, G-code export helpers, and
// tower audits. Definitions are in mixed_nozzle_fixtures.cpp.

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleSelector.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace mixed_nozzle_fixtures {

using Slic3r::BoundingBoxf;
using Slic3r::DynamicPrintConfig;
using Slic3r::EnforcerBlockerType;
using Slic3r::Model;
using Slic3r::ModelVolume;
using Slic3r::Print;
using Slic3r::PrintConfig;
using Slic3r::TriangleMesh;
using Slic3r::Vec2d;
using Slic3r::Vec3d;
using Slic3r::MixedNozzleSlicingMode;
using Slic3r::WipeTowerWallType;

// ---- Configs ----------------------------------------------------------------------------------

// Two filaments on a 0.2/0.4 pair, everything that perturbs a coupon slice switched off.
// `enabled` selects Body Split, otherwise mode Off.
DynamicPrintConfig coupon_config(bool enabled);
// coupon_config(true) with every per-filament vector widened to `count` filaments.
DynamicPrintConfig coupon_config_with_filaments(size_t count);
// Feature Split tower config on a fine_d/coarse_d pair (walls fine, sparse infill coarse).
DynamicPrintConfig feature_split_tower_config(double fine_d, double coarse_d, bool swapped);
// feature_split_tower_config switched to Body Split, with the given cadences, pad and wall type.
DynamicPrintConfig body_split_pad_config(double fine_d, double coarse_d,
                                             double fine_cadence, double coarse_cadence,
                                             bool swapped, bool auto_pad, double width_cap,
                                             WipeTowerWallType wall_type);
// Tower exit retraction, wipe and z-hop settings: a 0.4 mm retraction, a 2 mm
// wipe and a 0.4 mm z-hop on both tools, so a fine tower exit pulls 0.4 + 0.3 relief = 0.7 mm.
void apply_tower_exit_retraction(DynamicPrintConfig &config);
// coupon_config(true) on a 0.2/0.6 pair that admits a ratio-3 (0.30/0.10) coarse cadence.
DynamicPrintConfig fine_skin_ratio3_config();
// Feature Split on eight H2D-like slots, 0.2/0.6 with slot 7 (sparse infill) on the 0.6, 10000 s tool changes.
DynamicPrintConfig feature_economics_baseline_config();
// feature_economics_baseline_config() with free tool changes and loads.
DynamicPrintConfig free_switch_economics_config();
// The tower coupon config widened to three filaments: 1 on the 0.2 nozzle, 2 and 3 on the 0.4 nozzle.
DynamicPrintConfig paint_cadence_tower_config();

// coupon_config() and the configs built on it size only a few per-filament vectors. Print::apply
// pads every other one with zeros for filament 2 and up, so that filament exports with a flow ratio,
// volumetric speed and nozzle temperature of 0: its roads carry E0 and extrude nothing. This copies
// filament 1's value into every missing slot of each per-filament option.
void fill_per_filament_values(DynamicPrintConfig &config);

// ---- Models -----------------------------------------------------------------------------------

// A box with an oppositely wound internal shell: an internal pocket.
TriangleMesh box_with_internal_cavity(const Vec3d &outer, const Vec3d &cavity_min, const Vec3d &cavity);
// A 10x20x4 fine body on filament 1 (0.10) touching the coarse body on filament 2 (0.20) at x = 10.
void init_coupon(Model &model, Print &print, const DynamicPrintConfig &config,
                 TriangleMesh coarse_mesh = Slic3r::make_cube(20., 20., 4.));
// One plain cube, no per-volume settings (Feature Split fixtures).
void init_feature_flow_fixture(Model &model, Print &print, const DynamicPrintConfig &config,
                               double height = 1.2, double footprint = 20.);
// Stamps each body's native cadence by the nozzle its filament resolves to, then re-applies.
void declare_native_cadences(Model &model, Print &print, const DynamicPrintConfig &config,
                             double fine_cadence = 0.10, double coarse_cadence = 0.20);
// The standard prism pair with each body's logical filament named explicitly.
void init_mapped_coupon(Model &model, Print &print, const DynamicPrintConfig &config,
                        int fine_filament_1based, int coarse_filament_1based, double height = 4.);
// init_coupon on a BBL printer, so the Type 1 tower is what gets built.
void init_tower_coupon(Model &model, Print &print, const DynamicPrintConfig &config,
                       TriangleMesh coarse_mesh = Slic3r::make_cube(20., 20., 4.));
// Body Split tower coupon: fine body on filament 1, coarse body on filament 2, native cadences.
void init_body_split_pad_coupon(Model &model, Print &print, const DynamicPrintConfig &config,
                                    bool swapped, double height, double fine_cadence, double coarse_cadence);
// Fine body at x 0..w and a coarse body 20 mm on that opts into fine skins.
void init_native_skin_coupon(Model &model, Print &print, const DynamicPrintConfig &config,
                             TriangleMesh fine_mesh, TriangleMesh coarse_mesh,
                             double fine_cadence, double coarse_cadence);
void paint_cadence_facets(ModelVolume &volume, const std::vector<int> &facets, EnforcerBlockerType state);
// Fine body (filament 1, 0.10) and coarse body (0.20) 2 mm apart, optional paint, optional second
// coarse body on its own filament.
void init_paint_cadence_coupon(Model &model, Print &print, const DynamicPrintConfig &config,
                               int coarse_extruder,
                               std::optional<EnforcerBlockerType> fine_paint,
                               std::optional<EnforcerBlockerType> coarse_paint,
                               const std::vector<int> &facets,
                               int second_coarse_extruder = 0,
                               bool bbl_printer = false);

// ---- Tower G-code audits ----------------------------------------------------------------------

// How the fine nozzle leaves the tower after each "; FINE_TOWER_RETURN_RELIEF".
struct TowerExitAudit
{
    size_t returns { 0 };
    size_t lifted_before_pull { 0 };
    size_t moves_off_rows { 0 };
    size_t returns_without_wipe { 0 };
    size_t unbalanced { 0 };            // total pull differs from `expected_pull`
    size_t not_lifted_next { 0 };
    size_t crossings { 0 };             // tower-to-part travel crossing the part
};
TowerExitAudit audit_tower_exit(const std::string &gcode, double expected_pull);

// Feedrates of the tower's sparse-grid roads.
struct TowerGridSpeedAudit
{
    size_t grid_roads { 0 };
    size_t faster_roads { 0 };          // roads faster than the grid's first road
};
TowerGridSpeedAudit audit_tower_grid_speed(const std::string &gcode);

// Volume per mm of the coarse tool's tower wall arcs and sparse-grid roads, against a solid road
// for its own nozzle. Needs arc fitting on (the wall is told apart by its arcs).
struct TowerFlowAudit
{
    size_t coarse_grid_roads { 0 };
    size_t coarse_wall_arcs { 0 };
    size_t thin_grid { 0 };
    size_t thin_wall { 0 };
    double thinnest_ratio { 1e9 };
};
TowerFlowAudit audit_coarse_tower_flow(const std::string &gcode, const PrintConfig &resolved);

// Tower roads outside the planned tower footprint (plus half a road).
struct TowerFootprintAudit
{
    size_t tower_roads { 0 };
    size_t outside { 0 };
};
TowerFootprintAudit audit_tower_footprint(const std::string &gcode, const Print &print);

// ---- Prism and overhang fixtures ---------------------------------------------------------------

// A watertight prism: the counter-clockwise X/Z outline extruded across y = 0..depth, one closed part.
TriangleMesh extruded_xz_outline(const std::vector<Vec2d> &outline, double depth);

// A base base_width wide that widens to shelf_width at base_height: the shelf beyond base_width is a real
// overhang that needs support.
TriangleMesh overhang_shelf_mesh(double base_width, double shelf_width, double base_height, double total_height);

// Fine body on filament 1 and coarse body on filament 2, 20 mm on, at their own layer heights.
void init_native_coupon(Model &model, Print &print, const DynamicPrintConfig &config,
                        TriangleMesh fine_mesh, TriangleMesh coarse_mesh,
                        double fine_cadence = 0.10, double coarse_cadence = 0.20);

// One overhang_shelf_mesh(12, 20, 2, 4) part.
void init_feature_overhang_fixture(Model &model, Print &print, const DynamicPrintConfig &config);

// The purge each nozzle change lays for the arriving filament.
struct ArrivingPurge
{
    int    new_filament { -1 };
    int    new_nozzle   { -1 };
    bool   warm_return  { false };
    double volume_mm3   { 0. };
    size_t positive_moves { 0 };
    // Type1 bridge rows explicitly use 0.2 mm even on a thinner owning layer.
    double max_emitted_height { 0. };
    // The part layer this switch belongs to, from the last "; Z_HEIGHT:" before its marker.
    double print_z { 0. };
};

std::vector<ArrivingPurge> scan_arriving_purges(const std::string &gcode,
                                                     const std::vector<double> &filament_diameters);

double prime_row_allowance_mm3(const PrintConfig &config, int physical_nozzle, double layer_height);

// Every warm return (a nozzle arriving back at the filament it left) checked against its bounds: at least the
// hand-off prime for a tower `tower_width` wide (negative: the config's width), and with `upper_bound` at most the
// larger of that and the vendor prime plus one purge row. Returns how many there were and how many laid clearly
// less than the vendor prime.
struct WarmReturns
{
    size_t count { 0 };
    size_t below_vendor { 0 };
};
WarmReturns check_warm_returns(const std::string &gcode, const PrintConfig &resolved, double tower_width = -1.,
                               bool upper_bound = true);

// Per one-based G-code line (GCodeProcessor's gcode_id; index 0 is unused): the number, counted from 1, of the
// bracket the line sits in, from a line starting with one of `starts` through the next line starting with one of
// `ends`, or 0 outside every bracket.
std::vector<size_t> bracketed_lines(const std::string &gcode, const std::vector<std::string> &starts,
                                    const std::vector<std::string> &ends);

// Support on a 0.2/0.6 pair with a tower: the 0.6 lays the support body, the 0.2 the interface.
DynamicPrintConfig coarse_base_support_config(MixedNozzleSlicingMode mode);

} // namespace mixed_nozzle_fixtures
