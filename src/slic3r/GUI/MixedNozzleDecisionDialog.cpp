#include "MixedNozzleDecisionDialog.hpp"

#include "GUI_App.hpp"
#include "I18N.hpp"
#include "PartPlate.hpp"
#include "Plater.hpp"

#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"

#include <algorithm>
#include <wx/button.h>
#include <wx/dataview.h>
#include <wx/dialog.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/stattext.h>

namespace Slic3r { namespace GUI {

namespace {

wxString to_wx(const std::string &text) { return wxString::FromUTF8(text.c_str()); }

wxString format_number(const char *format, double value)
{
    return wxString::Format(format, value);
}

wxString z_range_label(double z_low, double z_high)
{
    return wxString::Format("%.2f - %.2f", z_low, z_high);
}

wxString optional_seconds(const std::optional<double> &seconds)
{
    return seconds ? format_number("%.1f", *seconds) : wxString("-");
}

// The tower's real material from the finished slice, never a sum of per-band trial deltas, which
// compare candidate slices rather than describe anything deposited.
void read_tower_material(const GCodeProcessorResult &slice, const PrintConfig &config,
                         MixedNozzleDecisionInput &input)
{
    double volume_mm3 = 0.;
    double mass_g = 0.;
    bool   have_density = true;
    for (const auto &[filament, volume] : slice.print_statistics.wipe_tower_volumes_per_extruder) {
        volume_mm3 += volume;
        if (filament < config.filament_density.size()) {
            // filament_density is g/cm3 and the volume is mm3.
            mass_g += volume * config.filament_density.get_at(filament) / 1000.;
        } else {
            have_density = false;
        }
    }
    if (volume_mm3 > 0.) {
        input.tower_volume_mm3 = volume_mm3;
        if (have_density && mass_g > 0.)
            input.tower_mass_g = mass_g;
    }
    if (slice.print_statistics.total_physical_tool_changes > 0)
        input.physical_tool_changes = std::size_t(slice.print_statistics.total_physical_tool_changes);
}

} // namespace

MixedNozzleDecisionData collect_mixed_nozzle_decision_data(Plater *plater, bool require_valid_slice_result)
{
    MixedNozzleDecisionData data;
    if (plater == nullptr)
        return data;
    auto &plates = plater->get_partplate_list();
    PartPlate *plate = plates.get_plate(plates.get_curr_plate_index());
    if (plate == nullptr)
        return data;
    // An advisor snapshot can outlive some invalidations, so its existence does not mean it
    // describes the G-code on screen.
    if (require_valid_slice_result && !plate->is_slice_result_valid())
        return data;

    PrintBase *base = nullptr;
    GCodeResult *result = nullptr;
    plate->get_print(&base, &result, nullptr);
    const auto *print = dynamic_cast<const Print *>(base);
    if (print == nullptr)
        return data;
    data.feature_split = is_mixed_nozzle_feature_split(print->config());

    // One plate, one report: bands from every object are renumbered into one sequence and each
    // object's regions get their own range, so two objects' bands never collapse into one row.
    AdvisorSliceObservations merged;
    merged.mode = AdvisorMode::FeatureSplit;
    std::size_t region_offset = 0;
    for (const PrintObject *object : print->objects()) {
        const AdvisorSliceObservationsPtr observations = object->mixed_nozzle_advisor_observations();
        if (!observations)
            continue;
        std::size_t max_region = 0;
        for (const AdvisorFeatureBand &band : observations->feature_bands) {
            AdvisorFeatureBand copy = band;
            copy.region_id = band.region_id + region_offset;
            copy.band_index = merged.feature_bands.size() + 1;
            max_region = std::max(max_region, band.region_id);
            merged.feature_bands.push_back(std::move(copy));
        }
        region_offset += max_region + 1;
    }
    if (merged.feature_bands.empty())
        return data;

    MixedNozzleDecisionInput input;
    input.observations = &merged;
    if (const GCodeProcessorResult *slice = plate->get_slice_result())
        read_tower_material(*slice, print->config(), input);
    // The headline names the coarse nozzle, always the larger of the two.
    const std::vector<double> &nozzles = print->config().nozzle_diameter.values;
    if (nozzles.size() == 2 && nozzles[0] != nozzles[1])
        input.coarse_nozzle_diameter = std::max(nozzles[0], nozzles[1]);

    data.summary = build_decision_summary(input);
    data.bands = build_decision_band_rows(merged);
    data.ranges = build_decision_range_rows(data.bands);
    data.available = data.summary.available;
    return data;
}

std::vector<MixedNozzleBandNote> collect_mixed_nozzle_band_notes(Plater *plater)
{
    const MixedNozzleDecisionData data = collect_mixed_nozzle_decision_data(plater);
    if (!data.available)
        return {};
    return build_decision_band_notes(data.bands);
}

void show_mixed_nozzle_decision_dialog(wxWindow *parent)
{
    const MixedNozzleDecisionData data = collect_mixed_nozzle_decision_data(wxGetApp().plater());
    if (!data.available) {
        // A current slice that is not Feature Split has no layer bands to explain.
        PartPlate *plate = wxGetApp().plater()->get_partplate_list().get_curr_plate();
        const bool sliced = plate != nullptr && plate->is_slice_result_valid();
        const wxString message = !sliced ? _L("Slice the current plate first. Results from an older slice are not used.")
            : !data.feature_split ? _L("Only Feature Split picks the fine or coarse nozzle for each layer band. In Body Split each part prints on the nozzle its material is on.")
            : _L("This slice has no layer bands to explain.");
        wxMessageBox(message,
                     _L("Why fine or coarse?"), wxOK | wxICON_INFORMATION, parent);
        return;
    }

    wxDialog dialog(parent, wxID_ANY, _L("Why each layer band used the fine or coarse nozzle"),
                    wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER);
    const int margin = dialog.FromDIP(12);
    const int gap = dialog.FromDIP(4);
    auto *sizer = new wxBoxSizer(wxVERTICAL);

    // 1. The summary, one wxStaticText per line, so no line is lost to rewrapping after
    // SetLabel/Layout.
    for (const std::string &line : format_decision_summary(data.summary))
        sizer->Add(new wxStaticText(&dialog, wxID_ANY, to_wx(line)), 0, wxLEFT | wxRIGHT | wxTOP, margin);

    // 2. Grouped ranges: consecutive bands with the same answer, one row each.
    sizer->Add(new wxStaticText(&dialog, wxID_ANY, _L("Layer ranges")), 0, wxLEFT | wxRIGHT | wxTOP, margin);
    auto *ranges = new wxDataViewListCtrl(&dialog, wxID_ANY, wxDefaultPosition, dialog.FromDIP(wxSize(760, 220)));
    ranges->AppendTextColumn(_L("Z range (mm)"), wxDATAVIEW_CELL_INERT, dialog.FromDIP(130));
    ranges->AppendTextColumn(_L("Nozzle"), wxDATAVIEW_CELL_INERT, dialog.FromDIP(80));
    ranges->AppendTextColumn(_L("Why"), wxDATAVIEW_CELL_INERT, dialog.FromDIP(120));
    ranges->AppendTextColumn(_L("Bands"), wxDATAVIEW_CELL_INERT, dialog.FromDIP(70));
    ranges->AppendTextColumn(_L("Min net (s)"), wxDATAVIEW_CELL_INERT, dialog.FromDIP(100));
    ranges->AppendTextColumn(_L("Typical net (s)"), wxDATAVIEW_CELL_INERT, dialog.FromDIP(110));
    ranges->AppendTextColumn(_L("Max net (s)"), wxDATAVIEW_CELL_INERT, dialog.FromDIP(100));
    for (const MixedNozzleDecisionRangeRow &range : data.ranges) {
        wxVector<wxVariant> row;
        row.push_back(wxVariant(z_range_label(range.z_low, range.z_high)));
        row.push_back(wxVariant(to_wx(range.decision)));
        row.push_back(wxVariant(to_wx(range.reason_word)));
        row.push_back(wxVariant(wxString::Format("%d", int(range.band_count))));
        row.push_back(wxVariant(optional_seconds(range.min_net_seconds)));
        row.push_back(wxVariant(optional_seconds(range.typical_net_seconds)));
        row.push_back(wxVariant(optional_seconds(range.max_net_seconds)));
        ranges->AppendItem(row);
    }
    sizer->Add(ranges, 1, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, margin);

    // 3. The per-band table, filtered to the selected range. It starts collapsed and is filled on
    // demand, since a print can have hundreds of bands.
    auto *bands_label = new wxStaticText(&dialog, wxID_ANY, _L("Select a range above to see its bands."));
    sizer->Add(bands_label, 0, wxLEFT | wxRIGHT | wxTOP, margin);
    auto *bands = new wxDataViewListCtrl(&dialog, wxID_ANY, wxDefaultPosition, dialog.FromDIP(wxSize(760, 200)));
    bands->AppendTextColumn(_L("Z range (mm)"), wxDATAVIEW_CELL_INERT, dialog.FromDIP(130));
    bands->AppendTextColumn(_L("Nozzle"), wxDATAVIEW_CELL_INERT, dialog.FromDIP(80));
    bands->AppendTextColumn(_L("Fine (s)"), wxDATAVIEW_CELL_INERT, dialog.FromDIP(90));
    bands->AppendTextColumn(_L("Coarse (s)"), wxDATAVIEW_CELL_INERT, dialog.FromDIP(90));
    bands->AppendTextColumn(_L("Switch + tower (s)"), wxDATAVIEW_CELL_INERT, dialog.FromDIP(140));
    bands->AppendTextColumn(_L("Net (s)"), wxDATAVIEW_CELL_INERT, dialog.FromDIP(90));
    bands->AppendTextColumn(_L("Why"), wxDATAVIEW_CELL_INERT, dialog.FromDIP(120));
    sizer->Add(bands, 1, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, margin);

    const std::vector<MixedNozzleDecisionBandRow> &all_bands = data.bands;
    const auto fill_bands = [bands, &all_bands](std::size_t first_index, std::size_t last_index) {
        bands->DeleteAllItems();
        for (const MixedNozzleDecisionBandRow &band : all_bands) {
            if (band.band_index < first_index || band.band_index > last_index)
                continue;
            wxVector<wxVariant> row;
            row.push_back(wxVariant(z_range_label(band.z_low, band.z_high)));
            row.push_back(wxVariant(to_wx(band.decision)));
            row.push_back(wxVariant(optional_seconds(band.fine_seconds)));
            row.push_back(wxVariant(optional_seconds(band.coarse_seconds)));
            row.push_back(wxVariant(optional_seconds(band.handoff_seconds)));
            row.push_back(wxVariant(optional_seconds(band.net_seconds)));
            row.push_back(wxVariant(to_wx(band.reason_word)));
            bands->AppendItem(row);
        }
    };

    ranges->Bind(wxEVT_DATAVIEW_SELECTION_CHANGED, [ranges, bands_label, &data, fill_bands](wxDataViewEvent &) {
        const int selected = ranges->GetSelectedRow();
        if (selected < 0 || std::size_t(selected) >= data.ranges.size())
            return;
        const MixedNozzleDecisionRangeRow &range = data.ranges[std::size_t(selected)];
        bands_label->SetLabel(_L("Bands") + wxString::Format(" %d - %d", int(range.first_band_index), int(range.last_band_index)));
        fill_bands(range.first_band_index, range.last_band_index);
    });

    auto *show_all = new wxButton(&dialog, wxID_ANY,
                                  _L("Show all bands") + wxString::Format(" (%d)", int(data.bands.size())));
    show_all->Bind(wxEVT_BUTTON, [bands_label, &data, fill_bands](wxCommandEvent &) {
        bands_label->SetLabel(_L("All bands") + wxString::Format(" (%d)", int(data.bands.size())));
        fill_bands(1, data.bands.empty() ? 0 : data.bands.back().band_index);
    });
    sizer->Add(show_all, 0, wxLEFT | wxRIGHT | wxTOP, margin);

    // 4. The legend, in plain language.
    auto *legend = new wxStaticText(&dialog, wxID_ANY, to_wx(mixed_nozzle_decision_legend()));
    legend->Wrap(dialog.FromDIP(760));
    sizer->Add(legend, 0, wxLEFT | wxRIGHT | wxTOP, margin);

    sizer->Add(dialog.CreateButtonSizer(wxOK), 0, wxALIGN_RIGHT | wxALL, margin);
    sizer->AddSpacer(gap);
    dialog.SetSizerAndFit(sizer);
    dialog.ShowModal();
}

}} // namespace Slic3r::GUI
