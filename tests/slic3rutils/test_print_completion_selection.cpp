#include <catch2/catch_all.hpp>

#include "slic3r/GUI/DeviceCore/PrintCompletionSelection.hpp"

#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace {

struct FakeMachine {
    std::string id;
    bool lan{true};
    bool connected{true};
    bool pending{false};
    int m_push_count{1};

    std::string get_dev_id() const { return id; }
    bool is_lan_mode_printer() const { return lan; }
    bool is_connected() const { return connected; }
    bool get_lan_mode_connection_state() const { return pending; }
};

struct FakeManager {
    FakeMachine* selected{nullptr};
    std::vector<std::string> selections;
    int disconnects{0};
    int connects{0};
    std::vector<std::function<void()>> callbacks;

    FakeMachine* get_selected_machine() { return selected; }
    void set_selected_machine(const std::string& id)
    {
        selections.push_back(id);
        ++disconnects;
        ++connects;
        callbacks.emplace_back([this] { ++connects; });
    }
    void drain_one_callback()
    {
        REQUIRE(!callbacks.empty());
        auto callback = std::move(callbacks.front());
        callbacks.erase(callbacks.begin());
        callback();
    }
};

struct SelectionSnapshot {
    std::vector<std::string> selections;
    int disconnects{0};
    size_t callbacks{0};
};

TEST_CASE("Print completion retains a live LAN selection with received pushes", "[PrintCompletionSelection]")
{
    FakeMachine machine{"lan-a"};
    FakeManager manager{&machine};
    Slic3r::select_machine_after_print(manager, "lan-a");
    CHECK(manager.selections.empty());
    CHECK(manager.disconnects == 0);
    CHECK(manager.connects == 0);
    CHECK(manager.callbacks.empty());
    CHECK(manager.selected == &machine);
    CHECK(machine.m_push_count == 1);
}

TEST_CASE("Print completion falls back to legacy selection when reuse is unsafe", "[PrintCompletionSelection]")
{
    auto exercise = [](FakeMachine& machine, std::string completed) {
        FakeManager manager{&machine};
        Slic3r::select_machine_after_print(manager, completed);
        return SelectionSnapshot{manager.selections, manager.disconnects, manager.callbacks.size()};
    };

    SECTION("different device") {
        FakeMachine machine{"lan-a"};
        auto manager = exercise(machine, "lan-b");
        CHECK(manager.selections == std::vector<std::string>{"lan-b"});
        CHECK(manager.disconnects == 1);
        CHECK(manager.callbacks == 1);
    }
    SECTION("no selected object") {
        FakeManager manager;
        Slic3r::select_machine_after_print(manager, "lan-a");
        CHECK(manager.selections == std::vector<std::string>{"lan-a"});
    }
    SECTION("stale connection") {
        FakeMachine machine{"lan-a"}; machine.connected = false;
        auto manager = exercise(machine, "lan-a");
        CHECK(manager.selections.size() == 1);
    }
    SECTION("fresh reset has no pushes") {
        FakeMachine machine{"lan-a"}; machine.m_push_count = 0;
        auto manager = exercise(machine, "lan-a");
        CHECK(manager.selections.size() == 1);
    }
    SECTION("pending connection") {
        FakeMachine machine{"lan-a"}; machine.pending = true;
        auto manager = exercise(machine, "lan-a");
        CHECK(manager.selections.size() == 1);
    }
    SECTION("cloud device") {
        FakeMachine machine{"lan-a"}; machine.lan = false;
        auto manager = exercise(machine, "lan-a");
        CHECK(manager.selections.size() == 1);
    }
    SECTION("empty completion id preserves deselection") {
        FakeMachine machine{"lan-a"};
        auto manager = exercise(machine, "");
        CHECK(manager.selections == std::vector<std::string>{""});
    }
}

TEST_CASE("Print completion evaluates the selected device at execution time", "[PrintCompletionSelection]")
{
    FakeMachine a{"lan-a"};
    FakeMachine b{"lan-b"};
    FakeManager manager{&a};
    manager.selected = &b;
    Slic3r::select_machine_after_print(manager, "lan-a");
    CHECK(manager.selections == std::vector<std::string>{"lan-a"});
}

TEST_CASE("Deferred print completion observes selection and freshness at dispatch", "[PrintCompletionSelection]")
{
    FakeMachine a{"lan-a"};
    FakeMachine b{"lan-b"};
    FakeManager changed_manager{&a};
    changed_manager.callbacks.emplace_back([&] {
        changed_manager.selected = &b;
        Slic3r::select_machine_after_print(changed_manager, "lan-a");
    });
    changed_manager.drain_one_callback();
    CHECK(changed_manager.selections == std::vector<std::string>{"lan-a"});

    FakeMachine stale{"lan-a"};
    stale.connected = false;
    FakeManager freshness_manager{&stale};
    freshness_manager.callbacks.emplace_back([&] {
        stale.connected = true;
        Slic3r::select_machine_after_print(freshness_manager, "lan-a");
    });
    freshness_manager.drain_one_callback();
    CHECK(freshness_manager.selections.empty());
}

TEST_CASE("Print completion preserves deferred callback effects", "[PrintCompletionSelection]")
{
    FakeMachine machine{"lan-a"};
    FakeManager manager{&machine};
    manager.callbacks.emplace_back([&] { machine.m_push_count = 2; });
    const auto callbacks_before = manager.callbacks.size();
    Slic3r::select_machine_after_print(manager, "lan-a");
    CHECK(manager.callbacks.size() == callbacks_before);
    manager.drain_one_callback();
    CHECK(machine.m_push_count == 2);

    machine.connected = false;
    Slic3r::select_machine_after_print(manager, "lan-a");
    CHECK(manager.selections.size() == 1);
    CHECK(manager.callbacks.size() == 1);
    CHECK(manager.connects == 1);
    manager.drain_one_callback();
    CHECK(manager.connects == 2);
}

} // namespace
