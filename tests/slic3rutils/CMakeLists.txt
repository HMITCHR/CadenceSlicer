get_filename_component(_TEST_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
add_executable(${_TEST_NAME}_tests
    ${_TEST_NAME}_tests_main.cpp
    test_dev_mapping.cpp
    test_network_versions.cpp
    test_action_source.cpp
    test_plugin_host_api.cpp
    test_plugin_capability_config.cpp
    test_plugin_config.cpp
    test_plugin_capabilities_in_use.cpp
    test_plugin_install.cpp
    test_plugin_lifecycle.cpp
    test_slicing_pipeline_bindings.cpp
    test_slicing_pipeline_config.cpp
    test_plugin_sort.cpp
    test_plugin_cloud_metadata.cpp
    test_plugin_audit.cpp
    test_print_completion_selection.cpp
    test_webview_identity.cpp
    test_windows_identity.cpp
    test_mixed_nozzle_plate.cpp
    test_mixed_nozzle_editors.cpp
    test_mixed_nozzle_ranking.cpp
    test_mixed_nozzle_preview.cpp
    test_mixed_nozzle_wizard_math.cpp
    test_mixed_nozzle_workflows.cpp
    test_mixed_nozzle_bug_guards.cpp
    test_mixed_nozzle_plater.cpp
    test_mixed_nozzle_apply_bed_type.cpp
    test_mixed_nozzle_apply_stale_paint.cpp
    test_mixed_nozzle_apply_support_filaments.cpp
    test_mixed_nozzle_apply_support_settings.cpp
    test_mixed_nozzle_printer_sync.cpp
    test_mixed_nozzle_high_flow.cpp
    test_mixed_nozzle_device_sync.cpp
    ../fff_print/mixed_nozzle_fixtures.cpp
    wizard_apply_support.hpp
    ../fff_print/test_helpers.cpp
    ../fff_print/mixed_nozzle_facts.cpp
    )

if (MSVC)
    target_link_libraries(${_TEST_NAME}_tests Setupapi.lib)
endif ()

target_link_libraries(${_TEST_NAME}_tests test_common libslic3r_gui libslic3r pybind11::embed Catch2::Catch2WithMain)
set_property(TARGET ${_TEST_NAME}_tests PROPERTY FOLDER "tests")

orcaslicer_copy_test_dlls()

if (WIN32)
    add_custom_command(TARGET ${_TEST_NAME}_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${_TEST_NAME}_tests>/python"
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_PREFIX_PATH}/libpython" "$<TARGET_FILE_DIR:${_TEST_NAME}_tests>/python"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_PREFIX_PATH}/libpython/python${_bundled_python_abi}.dll"
            "${CMAKE_PREFIX_PATH}/libpython/vcruntime140.dll"
            "${CMAKE_PREFIX_PATH}/libpython/vcruntime140_1.dll"
            "$<TARGET_FILE_DIR:${_TEST_NAME}_tests>"
        COMMENT "Copying Python runtime for slic3rutils plugin host API tests"
        VERBATIM
    )
elseif (APPLE)
    target_link_options(${_TEST_NAME}_tests PRIVATE
        "LINKER:-rpath,@executable_path/python/lib")

    add_custom_command(TARGET ${_TEST_NAME}_tests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E rm -rf
            "$<TARGET_FILE_DIR:${_TEST_NAME}_tests>/python"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_PREFIX_PATH}/libpython"
            "$<TARGET_FILE_DIR:${_TEST_NAME}_tests>/python"
        COMMENT "Copying Python runtime for macOS plugin host API tests"
        VERBATIM
    )
endif()

orcaslicer_discover_tests(${_TEST_NAME}_tests)
