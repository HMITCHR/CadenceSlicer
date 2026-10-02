get_filename_component(_TEST_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
add_executable(${_TEST_NAME}_tests 
	${_TEST_NAME}_tests.cpp
	test_helpers.cpp
	test_helpers.hpp
	test_cooling.cpp
	test_rebuild_admission.cpp
	test_rebuild_off.cpp
	test_rebuild_printers.cpp
	test_rebuild_feature_support.cpp
	test_rebuild_feature.cpp
	test_rebuild_cooling.cpp
	test_rebuild_tower.cpp
	test_rebuild_rc5_tower.cpp
	test_rebuild_rc5_misc.cpp
	test_rebuild_reprime.cpp
	test_rebuild_body_options.cpp
	test_rebuild_body.cpp
	test_rebuild_geometry.cpp
	test_rebuild_bug_guards.cpp
	test_rebuild_b38_support.cpp
	test_rebuild_b39_startline.cpp
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
	srl_fixtures.hpp
	srl_fixtures.cpp
	test_support_material.cpp
	test_trianglemesh.cpp
	test_wipe_tower.cpp
	)
target_link_libraries(${_TEST_NAME}_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET ${_TEST_NAME}_tests PROPERTY FOLDER "tests")

orcaslicer_copy_test_dlls()

orcaslicer_discover_tests(${_TEST_NAME}_tests)
