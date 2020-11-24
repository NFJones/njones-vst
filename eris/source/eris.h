//------------------------------------------------------------------------
// Project     : VST SDK
//
// Category    : Examples
// Filename    : public.sdk/samples/vst/eris/source/eris.h
// Created by  : Steinberg, 04/2005
// Description : Eris Example for VST SDK 3.0
//
//-----------------------------------------------------------------------------
// LICENSE
// (c) 2020, Steinberg Media Technologies GmbH, All Rights Reserved
//-----------------------------------------------------------------------------
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//   * Neither the name of the Steinberg Media Technologies nor the names of its
//     contributors may be used to endorse or promote products derived from this
//     software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
//-----------------------------------------------------------------------------

#pragma once

// must always come first
#include "public.sdk/source/vst/vstsinglecomponenteffect.h"
//------------------------------------------------------------------------

#include "pluginterfaces/vst/ivstcontextmenu.h"
#include "pluginterfaces/vst/ivstplugview.h"
#include "public.sdk/source/vst/vstguieditor.h"

#include "vstgui/plugin-bindings/vst3editor.h"

#include <a2m/converter.h>
#include <map>
#include <mutex>

namespace Steinberg {
namespace Vst {

template <typename T>
class ErisUIMessageController;

//------------------------------------------------------------------------
// Eris as combined processor and controller
//------------------------------------------------------------------------
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

    //---from IComponent-----------------------
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

    //---from IEditController-------
    IPlugView* PLUGIN_API createView(const char* name) SMTG_OVERRIDE;
    tresult PLUGIN_API setEditorState(IBStream* state) SMTG_OVERRIDE;
    tresult PLUGIN_API getEditorState(IBStream* state) SMTG_OVERRIDE;
    tresult PLUGIN_API setParamNormalized(ParamID tag, ParamValue value) SMTG_OVERRIDE;
    tresult PLUGIN_API getParamStringByValue(ParamID tag, ParamValue valueNormalized, String128 string) SMTG_OVERRIDE;
    tresult PLUGIN_API getParamValueByString(ParamID tag, TChar* string, ParamValue& valueNormalized) SMTG_OVERRIDE;

    //---from IMidiMapping-----------------
    tresult PLUGIN_API getMidiControllerAssignment(int32 busIndex,
                                                   int16 channel,
                                                   CtrlNumber midiControllerNumber,
                                                   ParamID& tag) SMTG_OVERRIDE;

    //---from VST3EditorDelegate-----------
    IController* createSubController(UTF8StringPtr name,
                                     const IUIDescription* description,
                                     VST3Editor* editor) SMTG_OVERRIDE;

    //---Interface---------
    OBJ_METHODS(Eris, SingleComponentEffect)
    tresult PLUGIN_API queryInterface(const TUID iid, void** obj) SMTG_OVERRIDE;
    REFCOUNT_METHODS(SingleComponentEffect)

    //---Internal functions-------
    void addUIMessageController(UIMessageController* controller);
    void removeUIMessageController(UIMessageController* controller);
    void setDefaultMessageText(String128 text);
    TChar* getDefaultMessageText();

    //------------------------------------------------------------------------
    template <typename SampleType>
    SampleType processAudio(SampleType** in, SampleType** out, int32 numChannels, int32 sampleFrames, float gain) {
        SampleType vuPPM = 0;

        // in real plug-in it would be better to do dezippering to avoid jump (click) in gain value
        for (int32 i = 0; i < numChannels; i++) {
            int32 samples = sampleFrames;
            auto* ptrIn = (SampleType*)in[i];
            auto* ptrOut = (SampleType*)out[i];
            SampleType tmp;
            while (--samples >= 0) {
                // apply gain
                tmp = (*ptrIn++) * gain;
                (*ptrOut++) = tmp;

                // check only positive values
                if (tmp > vuPPM) {
                    vuPPM = tmp;
                }
            }
        }
        return vuPPM;
    }

    //------------------------------------------------------------------------
   private:
    // our model values
    std::map<size_t, std::map<unsigned int, bool>> note_state;
    std::vector<std::vector<double>> sample_buffer;
    int remaining_samples;
    int32 time_window;
    int32 block_size;
    int32 note_count;
    int spill_samples;
    std::recursive_mutex lock;

    int32 currentProcessMode;

    a2m::Converter converter;

    using UIMessageControllerList = std::vector<UIMessageController*>;
    UIMessageControllerList uiMessageControllers;

    String128 defaultMessageText;

    int time_window_to_block_size();
    void set_time_window(const int32 time_window);
    void clear_buffers();
    void terminate_notes(int offset);
    std::vector<std::vector<a2m::Note>> convert(int channels);
    void terminate_notes(int offset, ProcessData& data);
    void add_notes(const std::vector<std::vector<a2m::Note>>& notes, int offset, ProcessData& data);

    template<class T>
    int buffer_samples(const int channel, T* samples, const int nsamples) {
        while (sample_buffer.size() < channel + 1)
            sample_buffer.push_back(std::vector<double>(block_size));

        int index = spill_samples;
        while (index < nsamples) {
            sample_buffer[channel][index] = *(samples + index);
            index++;
        }

        if (nsamples < block_size)
            spill_samples = index;
        else
            spill_samples = 0;

        return index;
    }
};

//------------------------------------------------------------------------
}  // namespace Vst
}  // namespace Steinberg
