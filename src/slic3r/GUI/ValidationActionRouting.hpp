#pragma once

#include <algorithm>
#include <array>
#include <string>
#include <vector>

namespace Slic3r::GUI {

enum class ValidationAction {
    StockSidebarJump,
    FilamentMapDialog,
    PlateSettingsFeature,
    SharedMixedNozzleSetup,
};

enum class ValidationFocus {
    None,
    FineAndStatus,
    CoarseAndStatus,
};

struct ValidationActionRoute {
    ValidationAction action {ValidationAction::StockSidebarJump};
    ValidationFocus focus {ValidationFocus::None};
    int source {0};
    bool auto_apply {false};
};

inline ValidationActionRoute mixed_nozzle_validation_route(const std::string& option_key, bool preview_source)
{
    if (option_key == "filament_map_mode" || option_key == "filament_map")
        return {ValidationAction::FilamentMapDialog, ValidationFocus::None, preview_source ? 1 : 0, false};
    static constexpr std::array<const char*, 5> fine_keys {
        "outer_wall_filament_id", "inner_wall_filament_id", "top_surface_filament_id",
        "bottom_surface_filament_id", "internal_solid_filament_id"};
    for (const char* key : fine_keys)
        if (option_key == key)
            return {ValidationAction::PlateSettingsFeature, ValidationFocus::FineAndStatus, 0, false};
    if (option_key == "sparse_infill_filament_id")
        return {ValidationAction::PlateSettingsFeature, ValidationFocus::CoarseAndStatus, 0, false};
    if (option_key == "mixed_nozzle_slicing_mode")
        return {ValidationAction::SharedMixedNozzleSetup, ValidationFocus::None, 0, false};
    return {};
}

inline const char* validation_feature_focus_event(ValidationFocus focus)
{
    return focus == ValidationFocus::FineAndStatus ? "feature_fine" :
           focus == ValidationFocus::CoarseAndStatus ? "feature_coarse" : "";
}

// Plate Settings opens its Feature rows only for a validation action that names one. Every other
// open, including the plain one, starts with Edit here off.
inline bool plate_settings_event_opens_feature_rows(const std::string& event_string)
{
    return event_string == "feature_fine" || event_string == "feature_coarse";
}

// An engine message split for display. The engine tags each refusal and warning with a code in
// square brackets or parentheses that tests, logs and bug reports match on. A person reads the
// sentence alone, and the codes are kept apart for a Details line.
struct EngineMessageParts {
    std::string text;
    std::vector<std::string> codes;
};

inline EngineMessageParts split_engine_message(const std::string& message)
{
    EngineMessageParts parts;
    std::string text;
    for (std::size_t i = 0; i < message.size();) {
        const char open = message[i];
        if (open == '[' || open == '(') {
            const char close = open == '[' ? ']' : ')';
            std::size_t end = i + 1;
            while (end < message.size() && ((message[end] >= 'A' && message[end] <= 'Z') ||
                                            (message[end] >= '0' && message[end] <= '9') || message[end] == '-'))
                ++end;
            const std::string code = message.substr(i + 1, end - i - 1);
            if (end < message.size() && message[end] == close && code.size() > 4 &&
                (code.rfind("SRL-", 0) == 0 || code.rfind("MNS-", 0) == 0)) {
                if (std::find(parts.codes.begin(), parts.codes.end(), code) == parts.codes.end())
                    parts.codes.push_back(code);
                i = end + 1;
                continue;
            }
        }
        text += message[i++];
    }
    if (parts.codes.empty()) {
        parts.text = message;
        return parts;
    }
    // Close the gaps a code leaves: no doubled spaces, no space before punctuation or a line end,
    // and no space at the start of a line.
    for (const char c : text) {
        if (c == ' ' && (parts.text.empty() || parts.text.back() == ' ' || parts.text.back() == '\n'))
            continue;
        if ((c == '.' || c == ',' || c == ';' || c == ':' || c == '\n') && !parts.text.empty() && parts.text.back() == ' ')
            parts.text.pop_back();
        parts.text += c;
    }
    while (!parts.text.empty() && parts.text.back() == ' ')
        parts.text.pop_back();
    return parts;
}

// The sentence, then "Details: <code>" on a line of its own. A message with no engine code is
// returned unchanged.
inline std::string engine_message_for_display(const std::string& message)
{
    const EngineMessageParts parts = split_engine_message(message);
    if (parts.codes.empty())
        return message;
    std::string shown = parts.text + "\nDetails: ";
    for (std::size_t i = 0; i < parts.codes.size(); ++i)
        shown += (i == 0 ? "" : ", ") + parts.codes[i];
    return shown;
}

} // namespace Slic3r::GUI
