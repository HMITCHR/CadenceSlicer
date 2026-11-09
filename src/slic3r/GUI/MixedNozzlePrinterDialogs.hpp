#pragma once

// Cadence rules for the stock printer dialogs (Synchronize AMS Filament Information, the filament
// sync that follows it, and Send print job) in Feature Split and Body Split. There every material
// prints on the nozzle its filament map names, so it can only come from the AMS that feeds that
// nozzle. Mode Off keeps the stock behavior, where the grouping is recomputed after a sync.

#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/ProjectTask.hpp"
#include "slic3r/GUI/DeviceManager.hpp"
#include "slic3r/GUI/DeviceCore/DevFilaSystem.h"
#include "slic3r/GUI/DeviceCore/DevMapping.h"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace Slic3r::GUI {

// True when each material has a fixed nozzle: Feature Split and Body Split.
inline bool mixed_nozzle_fixed_nozzle_mode(MixedNozzleSlicingMode mode)
{
    return mode == MixedNozzleSlicingMode::FeatureSplit || mode == MixedNozzleSlicingMode::BodySplit;
}

// The Synchronize dialog lists only the materials the plate prints with in Feature Split and Body
// Split. Stock lists every project material (mode Off keeps that). `used` is 1-based.
inline bool mixed_nozzle_sync_lists_filament(MixedNozzleSlicingMode mode, const std::vector<int> &used,
                                             std::size_t filament_index)
{
    if (!mixed_nozzle_fixed_nozzle_mode(mode))
        return true;
    return std::find(used.begin(), used.end(), int(filament_index) + 1) != used.end();
}

// Send print job: the stock "grouping is not optimal ... Regroup and slice" hint compares against
// the automatic grouping, which Feature Split and Body Split do not use. Shown in mode Off only.
inline bool mixed_nozzle_send_shows_grouping_hint(MixedNozzleSlicingMode mode)
{
    return !mixed_nozzle_fixed_nozzle_mode(mode);
}

// Color and type matching per nozzle, as the Send dialog does it: a material on the left nozzle
// (filament map 1) is matched only against trays of the AMS that feeds the left nozzle, one on the
// right (map 2) only against the right AMS. A nozzle without an AMS falls back to its external
// spool when `allow_ext`. A material on neither nozzle stays unmapped. Returns 0 when at least one
// side mapped, as DevMappingUtil::ams_filament_mapping does for one side.
inline int mixed_nozzle_ams_mapping_by_nozzle(const MachineObject *obj, const std::vector<FilamentInfo> &filaments,
                                              const std::vector<int> &filament_map,
                                              std::vector<FilamentInfo> &result, bool allow_ext,
                                              bool nozzle_has_ams_then_ignore_ext)
{
    result.clear();
    if (obj == nullptr)
        return -1;
    std::vector<FilamentInfo> left, right, unplaced;
    for (const FilamentInfo &filament : filaments) {
        const int nozzle = filament.id >= 0 && std::size_t(filament.id) < filament_map.size()
            ? filament_map[std::size_t(filament.id)] : 0;
        if (nozzle == 1)
            left.push_back(filament);
        else if (nozzle == 2)
            right.push_back(filament);
        else
            unplaced.push_back(filament);
    }
    bool has_left_ams = false, has_right_ams = false;
    for (const auto &ams : obj->GetFilaSystem()->GetAmsList()) {
        const int extruder = ams.second->GetExtruderId();
        if (extruder == MAIN_EXTRUDER_ID)
            (obj->is_main_extruder_on_left() ? has_left_ams : has_right_ams) = true;
        else if (extruder == DEPUTY_EXTRUDER_ID)
            (obj->is_main_extruder_on_left() ? has_right_ams : has_left_ams) = true;
    }
    // Four values: use_left_ams, use_right_ams, use_left_ext, use_right_ext.
    std::vector<FilamentInfo> left_result, right_result;
    const int left_code = DevMappingUtil::ams_filament_mapping(obj, left, left_result,
        {true, false, allow_ext && !has_left_ams, false}, {}, nozzle_has_ams_then_ignore_ext);
    const int right_code = DevMappingUtil::ams_filament_mapping(obj, right, right_result,
        {false, true, false, allow_ext && !has_right_ams}, {}, nozzle_has_ams_then_ignore_ext);
    for (FilamentInfo filament : unplaced) {
        filament.tray_id = -1;
        filament.distance = 99999;
        filament.ams_id.clear();
        filament.slot_id.clear();
        right_result.push_back(filament);
    }
    result.reserve(left_result.size() + right_result.size());
    result.insert(result.end(), left_result.begin(), left_result.end());
    result.insert(result.end(), right_result.begin(), right_result.end());
    std::stable_sort(result.begin(), result.end(),
                     [](const FilamentInfo &a, const FilamentInfo &b) { return a.id < b.id; });
    return left_code == 0 || right_code == 0 ? 0 : -1;
}

// The nozzle each synced material now prints on, for the filament map: the one its tray's AMS or
// external spool feeds. `source_keys` is PresetBundle::sync_ams_list's per-material tray key (the
// filament_ams_list key, -1 for a project material the sync kept). Keys at or above 0x10000 are the
// main extruder's trays, the rest the deputy's (Sidebar::build_filament_ams_list).
// `physical_extruder_map[d]` is the config slot of device extruder d. Returns a 1-based nozzle per
// material, 0 where it stays as it was.
inline std::vector<int> mixed_nozzle_synced_filament_nozzles(const std::vector<int> &source_keys,
                                                             const std::vector<int> &physical_extruder_map)
{
    std::vector<int> nozzles(source_keys.size(), 0);
    if (physical_extruder_map.size() != 2)
        return nozzles;
    for (std::size_t i = 0; i < source_keys.size(); ++i) {
        if (source_keys[i] < 0)
            continue;
        const std::size_t device = source_keys[i] >= 0x10000 ? MAIN_EXTRUDER_ID : DEPUTY_EXTRUDER_ID;
        const int slot = physical_extruder_map[device];
        if (slot == 0 || slot == 1)
            nozzles[i] = slot + 1;
    }
    return nozzles;
}

// Writes `nozzles` (from mixed_nozzle_synced_filament_nozzles) over a filament map, leaving the
// zero entries alone. True when anything changed.
inline bool mixed_nozzle_apply_synced_nozzles(std::vector<int> &filament_map, const std::vector<int> &nozzles)
{
    bool changed = false;
    for (std::size_t i = 0; i < nozzles.size() && i < filament_map.size(); ++i)
        if (nozzles[i] != 0 && filament_map[i] != nozzles[i]) {
            filament_map[i] = nozzles[i];
            changed = true;
        }
    return changed;
}

} // namespace Slic3r::GUI
