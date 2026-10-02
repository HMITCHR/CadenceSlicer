#pragma once

#include <string>

namespace Slic3r {

// Successful print completion retains an already selected LAN device with recent
// received push data. is_connected is a freshness heuristic; the pending flag alone
// does not establish liveness. Explicit same-device reconnect behavior elsewhere
// remains unchanged.
template <class Manager>
void select_machine_after_print(Manager& manager, const std::string& completed_id)
{
    auto* current = manager.get_selected_machine();
    const bool reuse = !completed_id.empty()
        && current != nullptr
        && current->get_dev_id() == completed_id
        && current->is_lan_mode_printer()
        && current->is_connected()
        && current->m_push_count > 0
        && !current->get_lan_mode_connection_state();

    if (!reuse)
        manager.set_selected_machine(completed_id);
}

} // namespace Slic3r
