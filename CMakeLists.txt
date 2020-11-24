cmake_minimum_required (VERSION 3.4.3)

project(njones-vst)

include(../windows.cmake)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

option(SMTG_ADD_VST3_PLUGINS_SAMPLES "Add VST3 Plug-ins Samples to the solution" OFF)
option(SMTG_ADD_VSTGUI "Add VSTGUI Support" ON)


set(SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/vst3sdk")

list(APPEND CMAKE_MODULE_PATH "${SDK_ROOT}/cmake/modules")
include(SMTG_VST3_SDK)

smtg_setup_platform_toolset()

add_subdirectory(eris)
add_subdirectory(a2m)
add_subdirectory(vst3sdk)
