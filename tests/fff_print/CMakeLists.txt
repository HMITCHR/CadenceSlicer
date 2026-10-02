get_filename_component(_TEST_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
add_executable(${_TEST_NAME}_tests 
	${_TEST_NAME}_tests.cpp
	test_helpers.cpp
	test_helpers.hpp
	test_cooling.cpp
	test_mixed_nozzle_admission.cpp
	test_mixed_nozzle_off.cpp
	test_mixed_nozzle_printers.cpp
	test_mixed_nozzle_feature_support.cpp
	test_mixed_nozzle_feature.cpp
	test_mixed_nozzle_cooling.cpp
	test_mixed_nozzle_tower.cpp
	test_mixed_nozzle_tower_first_layer.cpp
	test_mixed_nozzle_raft_and_narrow_infill.cpp
	test_mixed_nozzle_reprime.cpp
	test_mixed_nozzle_body_options.cpp
	test_mixed_nozzle_body.cpp
	test_mixed_nozzle_geometry.cpp
	test_mixed_nozzle_bug_guards.cpp
	test_mixed_nozzle_support.cpp
	test_mixed_nozzle_start_line.cpp
	test_extrusion_entity.cpp
	test_fill.cpp
	test_flow.cpp
	test_gcode_timing.cpp
	test_gcodewriter.cpp
	test_model.cpp
	test_multifilament.cpp
	test_perimeters.cpp
	test_print.cpp
	test_printobject.cpp
	test_skirt_brim.cpp
	test_slicing_pipeline_hook.cpp
	mixed_nozzle_facts.hpp
	mixed_nozzle_facts.cpp
	mixed_nozzle_fixtures.hpp
	mixed_nozzle_fixtures.cpp
	test_support_material.cpp
	test_trianglemesh.cpp
	test_wipe_tower.cpp
	)
target_link_libraries(${_TEST_NAME}_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET ${_TEST_NAME}_tests PROPERTY FOLDER "tests")

orcaslicer_copy_test_dlls()

orcaslicer_discover_tests(${_TEST_NAME}_tests)
