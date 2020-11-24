
include_directories("../a2m/src/")
include_directories("${SDK_ROOT}")

add_compile_definitions(FMT_HEADER_ONLY=1 VSTGUI_LIVE_EDITING=1)

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
    )

    set(target eris)
    smtg_add_vst3plugin(${target} ${eris_sources})
    # set_target_properties(${target} PROPERTIES ${SDK_IDE_PLUGIN_EXAMPLES_FOLDER})
    target_link_libraries(${target} PRIVATE sdk vstgui_support a2m-static fftw3)
    add_dependencies(${target} a2m-static)

    smtg_add_vst3_resource(${target} "resource/eris.uidesc")
    smtg_add_vst3_resource(${target} "resource/background.png")
    smtg_add_vst3_resource(${target} "resource/slider_background.png")
    smtg_add_vst3_resource(${target} "resource/slider_handle.png")
    smtg_add_vst3_resource(${target} "resource/slider_handle_2.0x.png")
    smtg_add_vst3_resource(${target} "resource/vu_on.png")
    smtg_add_vst3_resource(${target} "resource/vu_off.png")

    if(SMTG_MAC)
        smtg_set_bundle(${target} INFOPLIST "${CMAKE_CURRENT_LIST_DIR}/mac/Info.plist" PREPROCESS)
    elseif(SMTG_WIN)
        target_sources(${target} PRIVATE resource/eris.rc)
    endif()
endif(SMTG_ADD_VSTGUI)