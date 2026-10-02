# Runs before GUI compilation. configure_file only changes the header when the
# actual source identity changes, so unrelated translation units do not rebuild.
set(ORCA_GUI_BUILD_COMMIT "${ORCA_BUILD_ID_FALLBACK}")
if(NOT ORCA_GUI_BUILD_COMMIT)
    set(ORCA_GUI_BUILD_COMMIT "unknown")
endif()
set(ORCA_GUI_BUILD_DIRTY "")
if(GIT_EXECUTABLE AND EXISTS "${ORCA_SOURCE_DIR}/.git")
    execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse --short=9 HEAD
        WORKING_DIRECTORY "${ORCA_SOURCE_DIR}" RESULT_VARIABLE result
        OUTPUT_VARIABLE actual_head OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "Cannot identify the GUI source commit")
    endif()
    set(ORCA_GUI_BUILD_COMMIT "${actual_head}")
    execute_process(COMMAND "${GIT_EXECUTABLE}" status --porcelain --untracked-files=no
        WORKING_DIRECTORY "${ORCA_SOURCE_DIR}" RESULT_VARIABLE result
        OUTPUT_VARIABLE dirty OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "Cannot determine whether GUI sources are modified")
    endif()
    if(dirty)
        set(ORCA_GUI_BUILD_DIRTY "-dirty")
    endif()
endif()
set(ORCA_GUI_BUILD_ID "${ORCA_GUI_BUILD_COMMIT}${ORCA_GUI_BUILD_DIRTY}")
configure_file("${CMAKE_CURRENT_LIST_DIR}/GuiBuildId.hpp.in" "${ORCA_BUILD_ID_HEADER}" @ONLY)
