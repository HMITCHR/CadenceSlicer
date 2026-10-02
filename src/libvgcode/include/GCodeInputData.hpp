///|/ Copyright (c) Prusa Research 2023 Enrico Turri @enricoturri1966
///|/
///|/ libvgcode is released under the terms of the AGPLv3 or higher
///|/
#ifndef VGCODE_GCODEINPUTDATA_HPP
#define VGCODE_GCODEINPUTDATA_HPP

#include "PathVertex.hpp"

#include <cstddef>

namespace libvgcode {

struct GCodeInputData
{
    //
    // Whether or not the gcode was generated with spiral vase mode enabled.
    // Required to properly detect fictitious layer changes when spiral vase mode is enabled.
    //
    bool spiral_vase_mode{ false };
    //
    // List of path vertices (gcode moves)
    // See: PathVertex
    //
    std::vector<PathVertex> vertices;
    //
    // Palette for extruders colors
    //
    Palette tools_colors;
    //
    // Palette for color print colors
    //
    Palette color_print_colors;
    //
    // ORCA Mixed-Nozzle Preview: physical-tool palette, independent of filament colors and
    // indexed only by a valid zero-based physical_tool_id. 255 / index >= size is neutral gray.
    //
    Palette physical_tool_colors;
    //
    // ORCA Mixed-Nozzle Preview: indices into the final converted vertices marking proven
    // different-physical-tool transitions. Bounds/ToolChange checked, sorted and deduplicated
    // on ingestion.
    //
    std::vector<std::size_t> physical_transition_vertex_indices;
};

} // namespace libvgcode

#endif // VGCODE_BITSET_HPP
