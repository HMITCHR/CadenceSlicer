#pragma once

#include "PrintConfig.hpp"

#include <array>
#include <optional>
#include <string>

namespace Slic3r {

enum class MixedNozzleProcessColumnOrder { CanonicalRoles, PhysicalTools };

// A process preset may keep its two nozzle columns in canonical role order
// (role 1 = metadata[0], role 2 = metadata[1]) while a project installs those
// roles in either physical slot.  This mapping is deliberately independent of
// the process values themselves so every consumer resolves the same role.
struct MixedNozzleProcessRoleMapping {
    std::array<size_t, 2> physical_to_role;
    std::array<size_t, 2> role_to_physical;
};

struct MixedNozzleProcessRoleResolution {
    std::optional<MixedNozzleProcessRoleMapping> mapping;
    // Short diagnostic for the legacy fallback path.  It is intentionally
    // returned rather than inferred from a preset filename.
    std::string diagnostic;

    explicit operator bool() const { return mapping.has_value(); }
};

// Validate and resolve canonical process roles against the effective physical
// nozzle pair.  `project` supplies the project-owned nozzle override when it
// is valid; `printer` supplies the physical variant inventory and extruder
// types.  Invalid or absent metadata returns a diagnostic and no mapping.
// Applied configs identify columns by physical tool; preset configs use canonical roles.
MixedNozzleProcessRoleResolution resolve_mixed_nozzle_process_roles(
    const DynamicPrintConfig &process, const DynamicPrintConfig &project,
    const DynamicPrintConfig &printer,
    MixedNozzleProcessColumnOrder column_order = MixedNozzleProcessColumnOrder::CanonicalRoles);

// Shared GUI lookup.  The returned value is a zero-based canonical process
// role, or nullopt when the process must use its legacy physical-column map.
std::optional<size_t> mixed_nozzle_process_role_for_physical_tool(
    const DynamicPrintConfig &process, const DynamicPrintConfig &project,
    const DynamicPrintConfig &printer, size_t physical_extruder);

// Rewrite only the copied process `print_extruder_id` vector into physical
// one-based IDs.  Process variants, metadata, and all value vectors remain
// untouched.  Returns false when the supplied copy has no valid ID vector.
bool mixed_nozzle_remap_process_extruder_ids(
    DynamicPrintConfig &process, const MixedNozzleProcessRoleMapping &mapping);

} // namespace Slic3r
