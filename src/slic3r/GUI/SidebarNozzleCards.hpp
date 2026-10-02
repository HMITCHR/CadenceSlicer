#ifndef slic3r_GUI_SidebarNozzleCards_hpp_
#define slic3r_GUI_SidebarNozzleCards_hpp_

// Pure decisions behind the Printer sidebar's Left and Right Nozzle cards. No wx here, so
// slic3rutils can test them.

#include <algorithm>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace Slic3r::GUI {

// Mirrors the isDual argument Sidebar::update_presets() passes to layout_printer(): the Left and
// Right Nozzle cards are laid out only for a Bambu vendor printer whose preset lists exactly two
// extruder variants. Every other printer keeps the single card or the plain diameter box, and the
// cards' re-show step must leave it alone.
inline bool sidebar_shows_dual_nozzle_cards(bool is_bbl_vendor, std::size_t extruder_variant_count)
{
    return is_bbl_vendor && extruder_variant_count == 2;
}

// Sizes of the per-extruder vectors Sidebar::update_presets() indexes, without a bounds check,
// when it fills a nozzle card.
struct SidebarNozzleCardInputs
{
    std::size_t extruder_variants{0};   // printer extruder_variant_list
    std::size_t nozzle_diameters{0};    // effective pair (project pair or printer nozzle_diameter)
    std::size_t extruder_types{0};      // printer extruder_type
    std::size_t nozzle_volume_types{0}; // project nozzle_volume_type
};

// Names every input too short to fill the card for extruder_index, comma separated, or returns ""
// when all of them cover it. Used only for the nozzle card log line.
inline std::string sidebar_nozzle_card_short_inputs(const SidebarNozzleCardInputs &inputs, std::size_t extruder_index)
{
    std::string out;
    auto check = [&](std::size_t size, const char *name) {
        if (extruder_index < size)
            return;
        if (!out.empty())
            out += ',';
        out += name;
    };
    check(inputs.extruder_variants, "extruder_variant_list");
    check(inputs.nozzle_diameters, "nozzle_diameter");
    check(inputs.extruder_types, "extruder_type");
    check(inputs.nozzle_volume_types, "nozzle_volume_type");
    return out;
}

// The entry of a per-extruder vector a nozzle card reads: its own when the vector covers
// extruder_index, else the first, as ConfigOptionVector::get_at falls back. Empty when the vector
// has no entries, and the card then skips that row.
inline std::optional<std::size_t> sidebar_nozzle_card_entry(std::size_t size, std::size_t extruder_index)
{
    if (size == 0)
        return std::nullopt;
    return extruder_index < size ? extruder_index : 0;
}

// An enum value read from a config vector is a label index only when it is inside the label list.
inline bool sidebar_enum_label_valid(int value, std::size_t label_count)
{
    return value >= 0 && std::size_t(value) < label_count;
}

// The Flow combo for one nozzle card: the nozzle volume types to list, in enum order, and which
// row to select. offered is what the printer and the High Flow check offer; current is the
// project's nozzle_volume_type for that extruder, or -1 when it has none.
struct SidebarFlowChoices
{
    std::vector<int> types;
    int selection{-1};
};

// A rebuild never changes nozzle_volume_type (only a user pick in the combo does), so it must not
// show another type either: a held type the High Flow check does not offer right now (stale
// diameters, a 0.2 or 0.4 printer variant) is listed anyway and selected. With nothing held the
// last offered entry is selected, as upstream.
inline SidebarFlowChoices sidebar_flow_choices(const std::vector<int> &offered, int current)
{
    SidebarFlowChoices choices;
    choices.types = offered;
    if (current >= 0 && std::find(choices.types.begin(), choices.types.end(), current) == choices.types.end())
        choices.types.insert(std::upper_bound(choices.types.begin(), choices.types.end(), current), current);
    const auto held = std::find(choices.types.begin(), choices.types.end(), current);
    if (current >= 0 && held != choices.types.end())
        choices.selection = int(held - choices.types.begin());
    else
        choices.selection = choices.types.empty() ? -1 : int(choices.types.size()) - 1;
    return choices;
}

} // namespace Slic3r::GUI

#endif // slic3r_GUI_SidebarNozzleCards_hpp_
