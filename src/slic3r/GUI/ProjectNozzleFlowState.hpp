#ifndef slic3r_GUI_ProjectNozzleFlowState_hpp_
#define slic3r_GUI_ProjectNozzleFlowState_hpp_

#include "libslic3r/PrintConfig.hpp"
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace Slic3r::GUI {

// Native preset refresh mutations run through this boundary so explicit project publication
// can retain its reviewed flow vector while ordinary printer changes still initialize defaults.
template<class Update>
void with_project_nozzle_flows_preserved(DynamicPrintConfig &project, bool preserve, Update update)
{
    const ConfigOption *flow = preserve ? project.option("nozzle_volume_type") : nullptr;
    std::unique_ptr<ConfigOption> reviewed(flow ? flow->clone() : nullptr);
    try {
        update();
    } catch (...) {
        if (reviewed) project.set_key_value("nozzle_volume_type", reviewed.release());
        throw;
    }
    if (reviewed) project.set_key_value("nozzle_volume_type", reviewed.release());
}

// Flow types describe the installed hotends. A project that owns a dissimilar nozzle pair owns
// their flow types too, so a printer refresh that leaves the pair standing (a project load, or a
// printer variant switch made on the project's behalf) must keep the project's flow vector
// instead of loading the per-printer remembered or default one. An explicit printer change in the
// Printer tab clears the pair first (Tab::select_preset), so it still initializes the new
// printer's flows as upstream does.
inline bool project_owns_nozzle_flows(const DynamicPrintConfig &project, std::size_t printer_extruder_count)
{
    if (printer_extruder_count != 2)
        return false;
    const auto *pair = project.option<ConfigOptionFloats>("nozzle_diameter");
    if (pair == nullptr || pair->values.size() != 2)
        return false;
    const double first = pair->values[0];
    const double second = pair->values[1];
    if (!std::isfinite(first) || first <= 0. || !std::isfinite(second) || second <= 0. || first == second)
        return false;
    const auto *flows = project.option<ConfigOptionEnumsGeneric>("nozzle_volume_type");
    return flows != nullptr && flows->values.size() == 2;
}

// Adopting the printer's nozzles says yes to the flow types it reports, except a Standard reading
// over one the project has: the printer reads Standard for a hotend it has not identified
// (DevNozzle's default), so that change is still confirmed.
inline bool adopted_flows_need_confirmation(const std::vector<int> &project,
                                            const std::vector<NozzleVolumeType> &printer)
{
    for (std::size_t slot = 0; slot < project.size() && slot < printer.size(); ++slot)
        if (printer[slot] == nvtStandard && project[slot] != int(nvtStandard))
            return true;
    return false;
}

// What adopting the printer's flow readings does to the project: the slots it would write, and
// whether the owner is asked first. Without the adopt every change is asked. The sidebar's nozzle
// cards were drawn before the write, so they are redrawn whenever a slot is written.
struct AdoptedFlowPlan
{
    std::vector<std::pair<std::size_t, NozzleVolumeType>> writes;
    bool ask = false;
    bool writes_apply(bool confirmed) const { return !writes.empty() && (!ask || confirmed); }
    bool redraw_cards(bool confirmed) const { return writes_apply(confirmed); }
};

inline AdoptedFlowPlan plan_adopted_flows(const std::vector<int> &project, const std::vector<NozzleVolumeType> &printer,
                                          bool adopted)
{
    AdoptedFlowPlan plan;
    for (std::size_t slot = 0; slot < project.size() && slot < printer.size(); ++slot)
        if (project[slot] != int(printer[slot]))
            plan.writes.emplace_back(slot, printer[slot]);
    plan.ask = !plan.writes.empty() && (!adopted || adopted_flows_need_confirmation(project, printer));
    return plan;
}

// A printer sync that is about to open setup offers the filament sync first, so setup opens with
// the printer's filaments. Stock makes that offer after the nozzle sync, which here would be after
// setup. A nozzle changed by hand involves no printer and opens setup at once. A printer without an
// AMS keeps the stock order, and one sync asks once.
inline bool sync_filaments_before_setup(bool printer_sync, bool printer_has_ams, bool already_offered)
{
    return printer_sync && printer_has_ams && !already_offered;
}

// The flow types a printer sync writes, in config order (left, right). The device reports in its
// own order; extruder_map[d] is the config slot of device extruder d (the H2D reports its right
// nozzle first). A reported flow is used except on a 0.2 mm nozzle, which prints Standard only; a
// selected type from the nozzle stats wins. Each rule reads the same device extruder.
inline std::vector<NozzleVolumeType> device_sync_nozzle_volume_types(
    const std::vector<int> &extruder_map, const std::vector<double> &device_diameters,
    const std::vector<std::optional<NozzleVolumeType>> &device_flows,
    const std::vector<std::optional<NozzleVolumeType>> &device_selected)
{
    std::vector<NozzleVolumeType> types(extruder_map.size(), nvtStandard);
    for (std::size_t device = 0; device < extruder_map.size(); ++device) {
        const int slot = extruder_map[device];
        if (slot < 0 || std::size_t(slot) >= types.size())
            continue;
        NozzleVolumeType type = nvtStandard;
        if (device < device_flows.size() && device_flows[device] && device < device_diameters.size() &&
            std::abs(device_diameters[device] - 0.2) > 1e-6)
            type = *device_flows[device];
        if (device < device_selected.size() && device_selected[device])
            type = *device_selected[device];
        types[std::size_t(slot)] = type;
    }
    return types;
}

} // namespace Slic3r::GUI
#endif
