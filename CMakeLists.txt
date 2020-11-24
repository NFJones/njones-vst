cmake_minimum_required (VERSION 3.4.3)

project(a2m-vst)

option(SMTG_ADD_VST3_PLUGINS_SAMPLES "Add VST3 Plug-ins Samples to the solution" OFF)
option(SMTG_ADD_VSTGUI "Add VSTGUI Support" ON)


set(SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/vst3sdk")

list(APPEND CMAKE_MODULE_PATH "${SDK_ROOT}/cmake/modules")
include(SMTG_VST3_SDK)

smtg_setup_platform_toolset()

add_subdirectory(src)
add_subdirectory(a2m)
add_subdirectory(vst3sdk)
