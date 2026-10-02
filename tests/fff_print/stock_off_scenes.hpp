// Scenes shared by the stock OrcaSlicer golden generator and the mode Off comparison.
// Only upstream keys and upstream test helpers are used, so the same code compiles
// and slices the same way on stock OrcaSlicer and on this tree.
#pragma once

#include "test_helpers.hpp"

#include <string>
#include <vector>

namespace StockOffScenes {

inline std::vector<std::string> names() { return {"cube", "dual_nozzle", "support"}; }

inline std::string slice(const std::string &name)
{
    using namespace Slic3r;
    using namespace Slic3r::Test;
    if (name == "cube")
        return Slic3r::Test::slice({make_cube(10., 10., 3.)}, DynamicPrintConfig::full_print_config());
    if (name == "dual_nozzle")
        return slice_with_object_overrides(
            {make_cube(8., 8., 2.), make_cube(8., 8., 2.)},
            multifilament_config(2, {
                {"nozzle_diameter",                "0.4,0.4"},
                {"printer_extruder_id",            "1,2"},
                {"printer_extruder_variant",       "Direct Drive Standard,Direct Drive Standard"},
                {"extruder_printable_height",      "0,0"},
                {"single_extruder_multi_material", 0},
                {"enable_prime_tower",             1},
                {"prime_tower_width",              20},
                {"wipe_tower_x",                   "60"},
                {"wipe_tower_y",                   "60"},
            }),
            {{{"extruder", 1}}, {{"extruder", 2}}});
    if (name == "support")
        return Slic3r::Test::slice({TestMesh::overhang}, {{"enable_support", 1}});
    return {};
}

} // namespace StockOffScenes
