#pragma once

#include "Widgets/StateColor.hpp"

// App chrome only: printer, material, preview and status colors keep their semantics.
namespace CadencePalette {
inline constexpr const char *Action = "#F9C314";
inline constexpr const char *Hover = "#FFD24D";
inline constexpr const char *Pressed = "#DEAA00";
inline constexpr const char *OnAction = "#202020";
inline constexpr const char *AccentLight = "#856400";
inline constexpr const char *Selected = "#FFF4CF";
inline constexpr const char *SelectedDark = "#403615";

inline wxColour accent() { return StateColor::darkModeColorFor(AccentLight); }
inline bool is_action(const wxColour &color) {
    return color == wxColour(Action) || color == wxColour(Hover) || color == wxColour(Pressed);
}
}
