#include "MixedNozzleDecisionReport.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace Slic3r { namespace GUI {

namespace {

// Not std::ostringstream: the wording is asserted verbatim and must not depend on the GUI's locale.
std::string format_fixed(const char *format, double value)
{
    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), format, value);
    return buffer;
}

std::string format_z(double z) { return format_fixed("%.2f", z); }

// A height a user reads at a glance: whole millimetres from 10 mm up, one decimal below.
std::string format_plain_height(double z)
{
    return z >= 10. ? format_fixed("%.0f", z) : format_fixed("%.1f", z);
}

// A nozzle as the Printer panel names it: 0.2, 0.25, 0.8 (no trailing zeros).
std::string format_nozzle(double diameter)
{
    std::string text = format_fixed("%.2f", diameter);
    while (!text.empty() && text.back() == '0')
        text.pop_back();
    if (!text.empty() && text.back() == '.')
        text += '0';
    return text;
}

// A minute or more reads as minutes, anything shorter as seconds.
std::string format_duration(double seconds)
{
    const double magnitude = std::abs(seconds);
    if (magnitude >= 60.)
        return "~" + format_fixed("%.0f", seconds / 60.) + " min";
    return "~" + format_fixed("%.0f", seconds) + " s";
}

std::string format_seconds(double seconds) { return "~" + format_fixed("%.0f", seconds) + " s"; }

// The number is the selector's per-band model plus its hand-off sums, not what a printer measures,
// and whole-print effects are outside it. So the line says what it leaves out and names the
// selector as its source.
std::string format_saving_estimate(double seconds)
{
    std::string amount = format_duration(seconds);
    // "~15 min" reads as "about 15 min" in a sentence.
    if (!amount.empty() && amount.front() == '~')
        amount.erase(amount.begin());
    return "about " + amount + " less than all-fine (measured prints differ; the whole-print "
           "effects are not in this number)";
}

std::string format_signed_seconds(double seconds) { return format_fixed("%+.0f", seconds) + " s"; }

bool is_coarse_outcome(const AdvisorFeatureBand &band)
{
    return band.outcome == AdvisorFeatureOutcome::Committed;
}

// Mean of the values that exist. Absent when nothing was priced, so zeros never read as a free
// switch.
std::optional<double> mean_of(const std::vector<double> &values)
{
    if (values.empty())
        return std::nullopt;
    double total = 0.;
    for (double value : values)
        total += value;
    return total / double(values.size());
}

std::string mixed_nozzle_decision_reason_word(MixedNozzleDecisionReason reason)
{
    switch (reason) {
    case MixedNozzleDecisionReason::SavesTime:   return "Saves time";
    case MixedNozzleDecisionReason::CostsMore:   return "Costs more";
    case MixedNozzleDecisionReason::NotEligible: return "Not eligible";
    case MixedNozzleDecisionReason::Fallback:    return "Fallback";
    case MixedNozzleDecisionReason::Unavailable: break;
    }
    return "Unavailable";
}

MixedNozzleDecisionReason mixed_nozzle_decision_reason(const AdvisorFeatureBand &band)
{
    // Print.cpp logs these strings verbatim; they are matched here, never rewritten at the source.
    if (band.reason == "coarse_faster_with_native_tower")
        return MixedNozzleDecisionReason::SavesTime;
    // The whole-print check sends a band fine because coarse costs more overall or through the
    // tower height it forces.
    if (band.reason == "fine_faster_with_native_tower" || band.reason == "whole_print_slower_than_fine" ||
        band.reason == "tower_height_costs_more_than_band_saves")
        return MixedNozzleDecisionReason::CostsMore;
    // Each of these means the comparison could not be priced, so the band kept the geometry stage's
    // decision.
    if (band.reason.find("unavailable") != std::string::npos)
        return MixedNozzleDecisionReason::Fallback;
    if (!band.reason.empty())
        return MixedNozzleDecisionReason::NotEligible;
    return MixedNozzleDecisionReason::Unavailable;
}

} // namespace

std::vector<MixedNozzleDecisionBandRow> build_decision_band_rows(const AdvisorSliceObservations &observations)
{
    std::vector<MixedNozzleDecisionBandRow> rows;
    rows.reserve(observations.feature_bands.size());
    for (std::size_t index = 0; index < observations.feature_bands.size(); ++index) {
        const AdvisorFeatureBand &band = observations.feature_bands[index];
        MixedNozzleDecisionBandRow row;
        // Older evidence has no band index; fall back to the position rather than "band 0".
        row.band_index = band.band_index != 0 ? band.band_index : index + 1;
        row.region_id = band.region_id;
        row.z_low = band.z_low;
        row.z_high = band.z_high;
        row.coarse = is_coarse_outcome(band);
        row.decision = row.coarse ? "Coarse" : "Fine";
        row.fine_seconds = band.fine_model_seconds;
        row.coarse_seconds = band.coarse_model_seconds;
        row.switch_seconds = band.incremental_switch_seconds;
        row.tower_seconds = band.incremental_tower_seconds;
        row.handoff_seconds = band.incremental_handoff_seconds;
        row.tower_volume_mm3 = band.incremental_tower_volume_mm3;
        if (band.fine_model_seconds && band.coarse_model_seconds && band.incremental_handoff_seconds)
            row.net_seconds = *band.fine_model_seconds - *band.coarse_model_seconds -
                              *band.incremental_handoff_seconds;
        row.reason = mixed_nozzle_decision_reason(band);
        row.reason_word = mixed_nozzle_decision_reason_word(row.reason);
        row.raw_reason = band.reason;
        rows.push_back(std::move(row));
    }
    return rows;
}

std::vector<MixedNozzleDecisionRangeRow> build_decision_range_rows(const std::vector<MixedNozzleDecisionBandRow> &rows)
{
    std::vector<MixedNozzleDecisionRangeRow> ranges;
    std::vector<double> net_values;

    const auto flush = [&](const MixedNozzleDecisionBandRow &last) {
        if (ranges.empty())
            return;
        MixedNozzleDecisionRangeRow &range = ranges.back();
        range.z_high = last.z_high;
        range.last_band_index = last.band_index;
        if (!net_values.empty()) {
            range.min_net_seconds = *std::min_element(net_values.begin(), net_values.end());
            range.max_net_seconds = *std::max_element(net_values.begin(), net_values.end());
            range.typical_net_seconds = mean_of(net_values);
        }
        net_values.clear();
    };

    for (std::size_t index = 0; index < rows.size(); ++index) {
        const MixedNozzleDecisionBandRow &row = rows[index];
        // Only an adjacent band extends a range, so separate stretches stay separate. Regions are
        // separate stacks of bands and never merge.
        const bool extends = !ranges.empty() && index > 0 && rows[index - 1].region_id == row.region_id &&
                             rows[index - 1].decision == row.decision && rows[index - 1].reason == row.reason;
        if (!extends) {
            if (index > 0)
                flush(rows[index - 1]);
            MixedNozzleDecisionRangeRow range;
            range.z_low = row.z_low;
            range.z_high = row.z_high;
            range.decision = row.decision;
            range.reason = row.reason;
            range.reason_word = row.reason_word;
            range.first_band_index = row.band_index;
            range.last_band_index = row.band_index;
            range.band_count = 0;
            ranges.push_back(std::move(range));
        }
        ++ranges.back().band_count;
        if (row.net_seconds)
            net_values.push_back(*row.net_seconds);
    }
    if (!rows.empty())
        flush(rows.back());
    return ranges;
}

MixedNozzleDecisionSummary build_decision_summary(const MixedNozzleDecisionInput &input)
{
    MixedNozzleDecisionSummary summary;
    if (input.observations == nullptr)
        return summary;

    summary.available = true;
    summary.physical_tool_changes = input.physical_tool_changes;
    summary.tower_volume_mm3 = input.tower_volume_mm3;
    summary.tower_mass_g = input.tower_mass_g;
    summary.coarse_nozzle_diameter = input.coarse_nozzle_diameter;

    const std::vector<MixedNozzleDecisionBandRow> rows = build_decision_band_rows(*input.observations);
    summary.band_count = rows.size();

    std::vector<double> savings;
    std::vector<double> handoffs;
    std::vector<double> switches;
    std::vector<double> towers;
    for (const MixedNozzleDecisionBandRow &row : rows) {
        if (!row.coarse)
            continue;
        ++summary.coarse_band_count;
        if (!summary.coarse_z_low || row.z_low < *summary.coarse_z_low)
            summary.coarse_z_low = row.z_low;
        if (!summary.coarse_z_high || row.z_high > *summary.coarse_z_high)
            summary.coarse_z_high = row.z_high;
        // Only priced bands contribute; an unpriced coarse band is a missing comparison, not a zero
        // saving.
        if (row.net_seconds)
            savings.push_back(*row.net_seconds);
        if (row.handoff_seconds)
            handoffs.push_back(*row.handoff_seconds);
        if (row.switch_seconds)
            switches.push_back(*row.switch_seconds);
        if (row.tower_seconds)
            towers.push_back(*row.tower_seconds);
    }
    if (!savings.empty()) {
        double total = 0.;
        for (double saving : savings)
            total += saving;
        summary.estimated_saving_seconds = total;
    }
    summary.typical_cost_seconds = mean_of(handoffs);
    summary.typical_switch_seconds = mean_of(switches);
    summary.typical_tower_seconds = mean_of(towers);

    // Why the model stopped using the coarse nozzle: the lowest non-coarse band that starts at or
    // above the last coarse band's top.
    if (summary.coarse_z_high) {
        const MixedNozzleDecisionBandRow *above = nullptr;
        for (const MixedNozzleDecisionBandRow &row : rows) {
            if (row.coarse || row.z_low + EPSILON < *summary.coarse_z_high)
                continue;
            if (above == nullptr || row.z_low < above->z_low)
                above = &row;
        }
        if (above != nullptr)
            summary.above_reason = above->reason;
    }
    return summary;
}

std::vector<std::string> format_decision_summary(const MixedNozzleDecisionSummary &summary)
{
    std::vector<std::string> lines;
    if (!summary.available) {
        lines.push_back("Detailed nozzle decisions are unavailable for this slice. Reslice this plate to generate them.");
        return lines;
    }

    if (summary.coarse_band_count == 0 || !summary.coarse_z_low || !summary.coarse_z_high) {
        lines.push_back("The coarse nozzle was not used: all " + std::to_string(summary.band_count) +
                        " bands stayed fine.");
    } else {
        lines.push_back("Coarse nozzle used from Z " + format_z(*summary.coarse_z_low) + " to Z " +
                        format_z(*summary.coarse_z_high) + " mm (" + std::to_string(summary.coarse_band_count) +
                        " of " + std::to_string(summary.band_count) + " bands).");
        std::string why;
        switch (summary.above_reason) {
        case MixedNozzleDecisionReason::CostsMore:   why = "switching cost more than it saved."; break;
        case MixedNozzleDecisionReason::NotEligible: why = "the bands were too small to pay for a switch."; break;
        case MixedNozzleDecisionReason::Fallback:    why = "the coarse comparison was unavailable."; break;
        default: break;
        }
        if (!why.empty())
            lines.push_back("Above Z " + format_z(*summary.coarse_z_high) + " mm, " + why);
    }

    if (summary.estimated_saving_seconds)
        lines.push_back("Selector estimate: " + format_saving_estimate(*summary.estimated_saving_seconds) + ".");

    std::string materials;
    if (summary.physical_tool_changes)
        materials = std::to_string(*summary.physical_tool_changes) + " nozzle changes.";
    if (summary.tower_volume_mm3) {
        if (!materials.empty())
            materials += " ";
        materials += "Prime tower material: " + format_fixed("%.1f", *summary.tower_volume_mm3 / 1000.) + " cm3";
        if (summary.tower_mass_g)
            materials += " (~" + format_fixed("%.1f", *summary.tower_mass_g) + " g)";
        materials += ".";
    }
    if (!materials.empty())
        lines.push_back(materials);

    if (summary.typical_cost_seconds) {
        std::string cost = "Switch + tower cost per band: " + format_seconds(*summary.typical_cost_seconds);
        if (summary.typical_switch_seconds && summary.typical_tower_seconds)
            cost += " (" + format_seconds(*summary.typical_switch_seconds) + " nozzle changes, " +
                    format_seconds(*summary.typical_tower_seconds) + " tower purge)";
        cost += ".";
        lines.push_back(cost);
    }
    return lines;
}

std::string format_decision_headline(const MixedNozzleDecisionSummary &summary)
{
    if (!summary.available)
        return "Detailed nozzle decisions are unavailable for this slice.";
    // One plain line. Z, band counts and the estimate stay in the report Explain opens.
    const std::string nozzle = summary.coarse_nozzle_diameter && *summary.coarse_nozzle_diameter > 0.
        ? "The " + format_nozzle(*summary.coarse_nozzle_diameter) + " mm nozzle"
        : std::string("The coarse nozzle");
    if (summary.coarse_band_count == 0 || !summary.coarse_z_low || !summary.coarse_z_high)
        return nozzle + " printed no coarse layers. Every layer printed fine.";
    return nozzle + " printed coarse layers up to " + format_plain_height(*summary.coarse_z_high) + " mm.";
}

std::string mixed_nozzle_decision_legend()
{
    return "Fine time / coarse time = printing this band's infill with each nozzle at its material's speed limit. "
           "Switch + tower = the extra nozzle changes and tower purge needed to use the coarse nozzle here. "
           "Estimates are nominal; the slice total also includes motion and cooling.";
}

std::vector<MixedNozzleBandNote> build_decision_band_notes(const std::vector<MixedNozzleDecisionBandRow> &rows)
{
    std::vector<MixedNozzleBandNote> notes;
    notes.reserve(rows.size());
    for (const MixedNozzleDecisionBandRow &row : rows) {
        MixedNozzleBandNote note;
        note.z_low = row.z_low;
        note.z_high = row.z_high;
        note.text = row.decision + " nozzle - " + row.reason_word;
        // net_seconds is always "what using the coarse nozzle for this band nets", whichever
        // nozzle actually ran, so the sign reads the same way on both kinds of row.
        if (row.net_seconds)
            note.text += " (using coarse here: " + format_signed_seconds(*row.net_seconds) + ")";
        notes.push_back(std::move(note));
    }
    return notes;
}

const MixedNozzleBandNote *find_decision_band_note(const std::vector<MixedNozzleBandNote> &notes, double print_z)
{
    for (const MixedNozzleBandNote &note : notes)
        if (print_z > note.z_low - EPSILON && print_z <= note.z_high + EPSILON)
            return &note;
    return nullptr;
}

}} // namespace Slic3r::GUI
