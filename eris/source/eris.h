#pragma once

#include "public.sdk/source/vst/vstsinglecomponenteffect.h"

#include "pluginterfaces/vst/ivstcontextmenu.h"
#include "pluginterfaces/vst/ivstplugview.h"
#include "public.sdk/source/vst/vstguieditor.h"

#include "vstgui/plugin-bindings/vst3editor.h"

#include <a2m/converter.h>
#include <map>
#include <mutex>

#include "block_processor.h"
#include "pitch_set.h"

namespace Steinberg {
namespace Vst {

template <typename T>
class ErisUIMessageController;

struct NoteState {
    NoteState(const bool is_active = false, int32 count = 0);
    bool is_active;
    int32 count;
};

class Eris : public SingleComponentEffect, public VSTGUI::VST3EditorDelegate, public IMidiMapping {
   public:
    //------------------------------------------------------------------------
    using UIMessageController = ErisUIMessageController<Eris>;
    using UTF8StringPtr = VSTGUI::UTF8StringPtr;
    using IUIDescription = VSTGUI::IUIDescription;
    using IController = VSTGUI::IController;
    using VST3Editor = VSTGUI::VST3Editor;

    Eris();

    static FUnknown* createInstance(void* /*context*/) { return (IAudioProcessor*)new Eris; }

    tresult PLUGIN_API initialize(FUnknown* context) SMTG_OVERRIDE;
    tresult PLUGIN_API terminate() SMTG_OVERRIDE;
    tresult PLUGIN_API setActive(TBool state) SMTG_OVERRIDE;
    tresult PLUGIN_API process(ProcessData& data) SMTG_OVERRIDE;
    tresult PLUGIN_API canProcessSampleSize(int32 symbolicSampleSize) SMTG_OVERRIDE;
    tresult PLUGIN_API setState(IBStream* state) SMTG_OVERRIDE;
    tresult PLUGIN_API getState(IBStream* state) SMTG_OVERRIDE;
    tresult PLUGIN_API setupProcessing(ProcessSetup& newSetup) SMTG_OVERRIDE;
    tresult PLUGIN_API setBusArrangements(SpeakerArrangement* inputs,
                                          int32 numIns,
                                          SpeakerArrangement* outputs,
                                          int32 numOuts) SMTG_OVERRIDE;

    IPlugView* PLUGIN_API createView(const char* name) SMTG_OVERRIDE;
    tresult PLUGIN_API setEditorState(IBStream* state) SMTG_OVERRIDE;
    tresult PLUGIN_API getEditorState(IBStream* state) SMTG_OVERRIDE;
    tresult PLUGIN_API setParamNormalized(ParamID tag, ParamValue value) SMTG_OVERRIDE;
    tresult PLUGIN_API getParamStringByValue(ParamID tag, ParamValue valueNormalized, String128 string) SMTG_OVERRIDE;
    tresult PLUGIN_API getParamValueByString(ParamID tag, TChar* string, ParamValue& valueNormalized) SMTG_OVERRIDE;

    tresult PLUGIN_API getMidiControllerAssignment(int32 busIndex,
                                                   int16 channel,
                                                   CtrlNumber midiControllerNumber,
                                                   ParamID& tag) SMTG_OVERRIDE;

    IController* createSubController(UTF8StringPtr name,
                                     const IUIDescription* description,
                                     VST3Editor* editor) SMTG_OVERRIDE;

    OBJ_METHODS(Eris, SingleComponentEffect)
    tresult PLUGIN_API queryInterface(const TUID iid, void** obj) SMTG_OVERRIDE;
    REFCOUNT_METHODS(SingleComponentEffect)

    void addUIMessageController(UIMessageController* controller);
    void removeUIMessageController(UIMessageController* controller);
    void setDefaultMessageText(String128 text);
    TChar* getDefaultMessageText();

   private:
    int32 time_window_param;
    int32 note_count;
    int32 sync;
    int32 beat_numerator;
    int32 beat_denominator;
    int32 combine_notes;
    int32 key;
    int32 pitch_set_index;
    int32 threshold;
    int32 octave;
    int32 note_min;
    int32 note_max;
    int32 max_length;
    int32 ceiling;

    std::vector<unsigned int> pitch_set;
    int32 time_window;
    int32 block_size;
    float tempo;
    int32 sample_rate;

    a2m::Converter converter;
    std::vector<std::array<NoteState, 128>> note_state;
    njones::audio::BlockProcessor<Sample32> buffer_32;
    njones::audio::BlockProcessor<Sample64> buffer_64;

    int32 currentProcessMode;
    using UIMessageControllerList = std::vector<UIMessageController*>;
    UIMessageControllerList uiMessageControllers;
    String128 defaultMessageText;

    int time_window_to_block_size();
    void process_parameters(ProcessData& data);
    void process_inputs(ProcessData& data);
    void set_time_window(const int32 time_window);
    void set_beat();
    void clear_buffers();
    void terminate_notes(const int channel, int offset, ProcessData& data, const std::vector<a2m::Note>& new_notes);
    void initiate_notes(const int channel, const std::vector<a2m::Note>& notes, int offset, ProcessData& data);
    void convert(ProcessData& data);
};
}  // namespace Vst
}  // namespace Steinberg
