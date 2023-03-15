
include_directories("../njones-audio/src/")
include_directories("${SDK_ROOT}")

set(CMAKE_CXX_STANDARD 20)

add_compile_definitions(FMT_HEADER_ONLY=1)

if(SMTG_ADD_VSTGUI)
    set(eris_sources
        ${SDK_ROOT}/public.sdk/source/vst/vstsinglecomponenteffect.cpp
        ${SDK_ROOT}/public.sdk/source/vst/vstsinglecomponenteffect.h
        ${SDK_ROOT}/vstgui4/vstgui/vstgui_win32.cpp
        ${SDK_ROOT}/vstgui4/vstgui/vstgui_uidescription_win32.cpp
        source/erisparamids.h
        source/eris.cpp
        source/eris.h
        source/version.h
        source/erisuimessagecontroller.h
        source/pitch_set.h
        source/pitch_set.cpp
        resource/eris.uidesc
    )

    project(eris VERSION 1.2.0)
    set(target eris)
    smtg_add_vst3plugin(${target} ${eris_sources})
    target_link_libraries(${target} PRIVATE sdk vstgui_support a2m-static njonesaudiolib-static fftw3)
    add_dependencies(${target} a2m-static njonesaudiolib-static)

    smtg_add_vst3_resource(${target} "resource/eris.uidesc")
    smtg_add_vst3_resource(${target} "resource/background.png")

    if(SMTG_MAC)
        smtg_set_bundle(${target} INFOPLIST "${CMAKE_CURRENT_LIST_DIR}/mac/Info.plist" PREPROCESS)
    elseif(SMTG_WIN)
        target_sources(${target} PRIVATE resource/eris.rc)
    endif()
endif(SMTG_ADD_VSTGUI)
