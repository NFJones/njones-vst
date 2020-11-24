
include_directories("../a2m/src/")
include_directories("${SDK_ROOT}")

if(SMTG_ADD_VSTGUI)
    set(eris_sources
        ${SDK_ROOT}/public.sdk/source/vst/vstsinglecomponenteffect.cpp
        ${SDK_ROOT}/public.sdk/source/vst/vstsinglecomponenteffect.h
        src/erisparamids.h
        src/eris.cpp
        src/eris.h
        src/version.h
        src/erisuimessagecontroller.h
    )

    set(target eris)
    smtg_add_vst3plugin(${target} ${eris_sources})
    # set_target_properties(${target} PROPERTIES ${SDK_IDE_PLUGIN_EXAMPLES_FOLDER})
    target_link_libraries(${target} PRIVATE sdk vstgui_support a2m-static fftw3)

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