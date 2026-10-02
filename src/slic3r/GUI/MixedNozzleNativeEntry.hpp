#ifndef slic3r_GUI_MixedNozzleNativeEntry_hpp_
#define slic3r_GUI_MixedNozzleNativeEntry_hpp_

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "MixedNozzleSetupController.hpp"

namespace Slic3r::GUI {

inline constexpr const char *MIXED_NOZZLE_UNSUPPORTED_TOPOLOGY =
    "This printer profile does not have two nozzles that Feature Split and Body Split can use together. Select a compatible profile or choose Off.";

// Two or more tools. A toolchanger runs the modes on two of its tools; the engine refuses a third
// tool that would print.
inline bool mixed_nozzle_configured_topology_supported(std::size_t nozzle_count)
{
    return nozzle_count >= 2;
}

// A non-Bambu printer is offered setup when its profile has two or more tools with a nozzle each.
// Some profiles list two extruders on one shared nozzle (single_extruder_multi_material), leaving
// no second nozzle. The flag cannot decide this in the engine, since Bambu's two-nozzle profiles
// set it too, so it is only read here for a named non-Bambu vendor profile.
inline bool mixed_nozzle_other_printer_offered(std::size_t nozzle_count, bool shares_one_nozzle)
{
    return mixed_nozzle_configured_topology_supported(nozzle_count) && !shares_one_nozzle;
}

inline const char *mixed_nozzle_mode_name(MixedNozzleSlicingMode mode)
{
    switch (mode) {
    case MixedNozzleSlicingMode::FeatureSplit: return "Feature Split";
    case MixedNozzleSlicingMode::BodySplit: return "Body Split";
    default: return "Off";
    }
}

// KeepPair writes the project nozzle pair and stops there, leaving the mode as it was, so grouping,
// presets and setup can follow in any order.
enum class DissimilarNozzleDecision { LeftOnly, RightOnly, KeepPair, Setup, Cancel };

struct DissimilarNozzleSelection {
    std::string left;
    std::string right;
    std::string printer_preset;
    bool operator==(const DissimilarNozzleSelection &rhs) const
    { return left == rhs.left && right == rhs.right && printer_preset == rhs.printer_preset; }
};

struct DissimilarNozzleOutcome {
    DissimilarNozzleSelection selection;
    DissimilarNozzleDecision decision;
    bool return_before_preset_lookup {false};
    std::optional<std::string> single_diameter;
    // Write the captured pair to the project. Without open_setup, return there.
    bool keep_pair {false};
    // True for the answers that open the wizard. With keep_pair it opens after the pair is kept.
    bool open_setup {false};
};

// Exact text of the sidebar Diameter combos; Sidebar::priv::switch_diameter() delegates here so a
// route decision compares like with like.
inline std::string mixed_nozzle_diameter_text(double diameter)
{
    std::ostringstream stream; // 2 decimals so 0.25 / 0.15 survive; trailing zeros trimmed.
    stream << std::fixed << std::setprecision(2) << diameter;
    std::string text = stream.str();
    if (text.find('.') != std::string::npos) {
        text.erase(text.find_last_not_of('0') + 1);
        if (text.back() == '.')
            text += '0'; // "1." -> "1.0"
    }
    return text;
}

// Every word of the nozzle-change prompt, built here so it can be checked without a GUI.
struct DissimilarNozzlePromptText {
    std::string title;
    std::string body;
    std::string use_left;
    std::string use_right;
    std::string keep_both;
    std::string set_up_now;
    std::string cancel;
    // Empty unless the printer profile cannot take two different nozzles.
    std::string unsupported;
};

// `previous_pair` is the pair the project owned before, empty when none. When it differs, the
// prompt says so first, so "Keep both nozzles" plainly means the nozzles just picked.
inline DissimilarNozzlePromptText dissimilar_nozzle_prompt_text(const DissimilarNozzleSelection &selection,
                                                                bool setup_supported,
                                                                const std::vector<double> &previous_pair = {})
{
    DissimilarNozzlePromptText text;
    text.title = "Two different nozzles";
    if (previous_pair.size() == 2) {
        const std::string before_left = mixed_nozzle_diameter_text(previous_pair[0]);
        const std::string before_right = mixed_nozzle_diameter_text(previous_pair[1]);
        if (before_left != selection.left || before_right != selection.right)
            text.body = "This project had a " + before_left + " mm nozzle on the left and a " + before_right +
                        " mm nozzle on the right. ";
    }
    // Say what the pair can do, not what one split mode does.
    text.body += "You now have a " + selection.left + " mm nozzle on the left and a " + selection.right +
                 " mm nozzle on the right. Cadence Slicer can use both in one print, each at its own "
                 "layer height.";
    text.use_left = "Use the left " + selection.left + " only";
    text.use_right = "Use the right " + selection.right + " only";
    text.keep_both = "Keep both nozzles";
    text.set_up_now = "Set up mixed-nozzle slicing now";
    text.cancel = "Cancel";
    if (!setup_supported)
        text.unsupported = "This printer cannot use two different nozzles together. Pick one nozzle, "
                           "or choose another printer profile.";
    return text;
}

inline bool dissimilar_nozzle_cta_eligible(bool single, const std::string &left, const std::string &right,
                                           bool configured_dual_topology)
{
    return !single && configured_dual_topology && !left.empty() && !right.empty() && left != right;
}

struct MixedNozzleProcessRow {
    std::string name;
    double coarse_height;
    double fine_height {0.};
    int priority {0};
};
inline std::string mixed_nozzle_default_process_row(std::vector<MixedNozzleProcessRow> rows,
                                                   double minimum_base_height = 0.)
{
    rows.erase(std::remove_if(rows.begin(), rows.end(), [minimum_base_height](const auto &row) {
        return row.name.empty() || !std::isfinite(row.coarse_height) || row.coarse_height <= 0. ||
               !std::isfinite(row.fine_height) || row.fine_height + 1e-9 < minimum_base_height;
    }), rows.end());
    if (rows.empty())
        return {};
    const int priority = std::max_element(rows.begin(), rows.end(), [](const auto &a, const auto &b) {
        return a.priority < b.priority;
    })->priority;
    if (priority > 0)
        rows.erase(std::remove_if(rows.begin(), rows.end(), [priority](const auto &row) {
            return row.priority != priority;
        }), rows.end());
    std::sort(rows.begin(), rows.end(), [](const auto &a, const auto &b) {
        if (a.coarse_height != b.coarse_height)
            return a.coarse_height < b.coarse_height;
        if (a.fine_height != b.fine_height)
            return a.fine_height > b.fine_height;
        return a.name < b.name;
    });
    // Alternate fine heights at one coarse height do not skew the existing median.
    rows.erase(std::unique(rows.begin(), rows.end(), [](const auto &a, const auto &b) {
        return std::abs(a.coarse_height - b.coarse_height) < 1e-9;
    }), rows.end());
    return rows[rows.size() / 2].name;
}

// Said when no printer profile gives the limits for a requested nozzle pair. The nozzles are left
// as they were, and the text says how to add the missing size.
inline std::string mixed_nozzle_missing_profile_text(const std::string &printer_model, const std::vector<double> &pair)
{
    const std::string printer = printer_model.empty() ? std::string("this printer") : "the " + printer_model;
    std::string sizes;
    if (pair.size() == 2)
        sizes = " for a " + mixed_nozzle_diameter_text(pair[0]) + " mm left and " +
                mixed_nozzle_diameter_text(pair[1]) + " mm right nozzle";
    return "There is no printer profile" + sizes + " on " + printer + ", so the nozzles were not changed. "
           "Add the missing nozzle size in Help > Setup Wizard (select the printer and tick every nozzle "
           "size), then sync or pick the nozzles again.";
}

// Printer sync reads nozzle sizes as floats (0.2f is 0.2000000030). Snap each to the 0.01 mm the
// Nozzle boxes show, so it compares equal to the profiles' values.
inline double mixed_nozzle_reported_diameter(double reported)
{
    return std::round(reported * 100.) / 100.;
}

// The pair the project owns outright: two distinct usable nozzles in the project config, the shape
// mixed_nozzle_effective_nozzle_diameters() admits as an override. Read from the project alone so
// the answer does not move with the printer variant.
inline std::vector<double> mixed_nozzle_project_owned_pair(const DynamicPrintConfig &project)
{
    const auto *nozzles = project.option<ConfigOptionFloats>("nozzle_diameter");
    if (nozzles == nullptr || !mixed_nozzle_project_nozzle_pair_valid(nozzles->values) ||
        nozzles->values[0] == nozzles->values[1])
        return {};
    return nozzles->values;
}

// Shared by the Printer panel refresh, background sync and post-load preset adoption. The printer
// preset is deliberately not an input: the pair belongs to the project, so a variant that disagrees
// with it leaves it standing.
inline bool mixed_nozzle_sync_preserves_project(const DynamicPrintConfig &project,
                                                const DynamicPrintConfig & /*printer*/)
{
    return !mixed_nozzle_project_owned_pair(project).empty();
}

// A slot may hold its material's profile for the nozzle it prints on, such as "PLA Basic @BBL H2D
// 0.8 nozzle" under a 0.2 printer preset, in a project that owns a pair. `sibling_report` is
// PresetBundle::mixed_nozzle_filament_sibling(...).report for the slot's nozzle: "base:" fits,
// while "sibling:" or "unresolved:" is a real mismatch.
inline bool mixed_nozzle_slot_preset_fits_nozzle(bool project_owns_pair, const std::string &sibling_report)
{
    return project_owns_pair && sibling_report.rfind("base:", 0) == 0;
}

enum class SidebarDiameterRoute { NoOp, PresetSwitch, DissimilarPrompt };

// Which action a sidebar Diameter combo change takes. `effective` is the pair the combos were
// seeded from, `left`/`right` the requested text (equal on a single-tool sidebar).
inline SidebarDiameterRoute resolve_sidebar_diameter_route(
    const std::vector<double> &effective, const std::string &left, const std::string &right,
    bool /*project_owns_pair*/, bool /*topology_supported*/)
{
    // Whoever owns the pair, a dissimilar request is confirmed in the prompt. The caller still
    // reads ownership to word the prompt and to drop the project override on a single-diameter
    // answer.
    const bool dissimilar_request = left != right;

    // Already installed: nothing to switch and nothing to write.
    if (effective.size() >= 2) {
        if (mixed_nozzle_diameter_text(effective[0]) == left &&
            mixed_nozzle_diameter_text(effective[1]) == right)
            return SidebarDiameterRoute::NoOp;
    } else if (!effective.empty() && !dissimilar_request &&
               mixed_nozzle_diameter_text(effective[0]) == left) {
        return SidebarDiameterRoute::NoOp;
    }

    // A project-owned pair changing to another pair is confirmed in the same prompt, never written
    // silently. Collapsing to one diameter is a printer-preset switch, and the caller must drop the
    // project override because Tab::select_preset() only clears it on an actual preset change.
    return dissimilar_request ? SidebarDiameterRoute::DissimilarPrompt : SidebarDiameterRoute::PresetSwitch;
}

inline DissimilarNozzleOutcome resolve_dissimilar_nozzle_decision(const DissimilarNozzleSelection &selection,
                                                                   DissimilarNozzleDecision decision,
                                                                   bool set_up_now = false)
{
    DissimilarNozzleOutcome outcome {selection, decision};
    switch (decision) {
    case DissimilarNozzleDecision::LeftOnly: outcome.single_diameter = selection.left; break;
    case DissimilarNozzleDecision::RightOnly: outcome.single_diameter = selection.right; break;
    case DissimilarNozzleDecision::KeepPair:
        // Write the pair; there is no single diameter to look a printer preset up by. With "Set up
        // now" ticked, setup opens on the kept pair afterwards, so Keep and setup are two Undo
        // steps.
        outcome.keep_pair = true;
        outcome.open_setup = set_up_now;
        outcome.return_before_preset_lookup = true;
        break;
    case DissimilarNozzleDecision::Setup:
        outcome.open_setup = true;
        outcome.return_before_preset_lookup = true;
        break;
    case DissimilarNozzleDecision::Cancel: outcome.return_before_preset_lookup = true; break;
    }
    return outcome;
}

struct MixedNozzleSearchDescriptor {
    std::string option_key;
    std::string label;
    std::string aliases;

    bool matches(std::string query) const
    {
        auto lower = [](std::string value) {
            std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return char(std::tolower(c)); });
            return value;
        };
        const std::string haystack = lower(label + " " + aliases);
        return haystack.find(lower(std::move(query))) != std::string::npos;
    }
};

inline const MixedNozzleSearchDescriptor &mixed_nozzle_search_descriptor()
{
    static const MixedNozzleSearchDescriptor descriptor {
        "mixed_nozzle_slicing_mode", "Mixed-Nozzle Slicing",
        "Body Split Fine Shell Coarse Core"};
    return descriptor;
}

} // namespace Slic3r::GUI
#endif
