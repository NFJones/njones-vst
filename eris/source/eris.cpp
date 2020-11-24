#include "eris.h"
#include "erisparamids.h"
#include "erisuimessagecontroller.h"
#include "version.h"  // for versioning

#include "public.sdk/source/main/pluginfactory.h"
#include "public.sdk/source/vst/vstaudioprocessoralgo.h"

#include "base/source/fstreamer.h"
#include "pluginterfaces/base/ibstream.h"
#include "pluginterfaces/base/ustring.h"  // for UString128
#include "pluginterfaces/vst/ivstevents.h"
#include "pluginterfaces/vst/ivstmidicontrollers.h"
#include "pluginterfaces/vst/ivstparameterchanges.h"
#include "pluginterfaces/vst/vstpresetkeys.h"  // for use of IStreamAttributes

#include "public.sdk/source/main/dllmain.cpp"
#include "vstgui/plugin-bindings/vst3editor.h"

#include <fmt/format.h>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>

// this allows to enable the communication example between eris and its controller
#define ERIS_TEST 1

using namespace VSTGUI;

static void log(const std::string& msg) {
    std::ofstream outfile("C:/Users/neilf/Downloads/eris.log", std::ios_base::app);
    outfile << msg << std::endl;
}

namespace Steinberg {
namespace Vst {

class TimeWindowParameter : public Parameter {
   public:
    TimeWindowParameter(int32 flags, int32 id);

    void toString(ParamValue normValue, String128 string) const SMTG_OVERRIDE;
    bool fromString(const TChar* string, ParamValue& normValue) const SMTG_OVERRIDE;
};

TimeWindowParameter::TimeWindowParameter(int32 flags, int32 id) {
    Steinberg::UString(info.title, USTRINGSIZE(info.title)).assign(USTRING("Time Window"));
    Steinberg::UString(info.units, USTRINGSIZE(info.units)).assign(USTRING("ms"));

    info.flags = flags;
    info.id = id;
    info.stepCount = 0;
    info.defaultNormalizedValue = 5 / 5000.0;
    info.unitId = kRootUnitId;

    setNormalized(info.defaultNormalizedValue);
}

void TimeWindowParameter::toString(ParamValue normValue, String128 string) const {
    char text[32];
    sprintf(text, "%d", static_cast<int>(normValue * 5000));

    Steinberg::UString(string, 128).fromAscii(text);
}

bool TimeWindowParameter::fromString(const TChar* string, ParamValue& normValue) const {
    Steinberg::UString wrapper((TChar*)string, -1);  // don't know buffer size here!
    int64 tmp = 0;
    if (wrapper.scanInt(tmp)) {
        normValue = tmp / 5000.0;
        return true;
    }
    return false;
}

class NoteCountParameter : public Parameter {
   public:
    NoteCountParameter(int32 flags, int32 id);

    void toString(ParamValue normValue, String128 string) const SMTG_OVERRIDE;
    bool fromString(const TChar* string, ParamValue& normValue) const SMTG_OVERRIDE;
};

NoteCountParameter::NoteCountParameter(int32 flags, int32 id) {
    Steinberg::UString(info.title, USTRINGSIZE(info.title)).assign(USTRING("Note Count"));
    Steinberg::UString(info.units, USTRINGSIZE(info.units)).assign(USTRING("notes"));

    info.flags = flags;
    info.id = id;
    info.stepCount = 0;
    info.defaultNormalizedValue = 0.0;
    info.unitId = kRootUnitId;

    setNormalized(info.defaultNormalizedValue);
}

void NoteCountParameter::toString(ParamValue normValue, String128 string) const {
    char text[32];
    sprintf(text, "%d", static_cast<int>(normValue * 127.0));

    Steinberg::UString(string, 128).fromAscii(text);
}

bool NoteCountParameter::fromString(const TChar* string, ParamValue& normValue) const {
    Steinberg::UString wrapper((TChar*)string, -1);  // don't know buffer size here!
    int64 tmp = 0;
    if (wrapper.scanInt(tmp)) {
        normValue = tmp / 127.0;
        return true;
    }
    return false;
}

Eris::Eris()
    : time_window(5),
      block_size(0),
      note_count(0),
      currentProcessMode(-1),
      converter(processSetup.sampleRate, 0),
      spill_samples(0) {
    clear_buffers();
    set_time_window(5);
}

tresult PLUGIN_API Eris::initialize(FUnknown* context) {
    tresult result = SingleComponentEffect::initialize(context);
    if (result != kResultOk)
        return result;

    addAudioInput(STR16("Stereo In"), SpeakerArr::kStereo);
    addAudioOutput(STR16("Stereo Out"), SpeakerArr::kStereo);
    addEventOutput(STR16("Event Out"), 1);

    auto* time_window_param = new TimeWindowParameter(ParameterInfo::kCanAutomate, kTimeWindowId);
    parameters.addParameter(time_window_param);

    auto* note_count_param = new NoteCountParameter(ParameterInfo::kCanAutomate, kNoteCountId);
    parameters.addParameter(note_count_param);

    return result;
}

tresult PLUGIN_API Eris::terminate() {
    return SingleComponentEffect::terminate();
}

tresult PLUGIN_API Eris::setActive(TBool state) {
#if ERIS_TEST
    if (state)
        fprintf(stderr, "[Eris] Activated \n");
    else
        fprintf(stderr, "[Eris] Deactivated \n");
#endif
    return kResultOk;
}

void Eris::terminate_notes(int offset, ProcessData& data) {
    for (auto& channel : note_state)
        for (auto& state : channel.second) {
            if (state.second) {
                NoteOffEvent note_off{static_cast<int16>(channel.first), static_cast<int16>(state.first), 0.0, -1, 0.0};
                Event event{0, offset, 0, Event::kIsLive, Event::kNoteOffEvent};
                event.noteOff = note_off;
                data.outputEvents->addEvent(event);
                state.second = false;
            }
        }
}

void Eris::add_notes(const std::vector<std::vector<a2m::Note>>& notes, int offset, ProcessData& data) {
    for (size_t channel = 0; channel < notes.size(); ++channel)
        for (auto& note : notes[channel]) {
            NoteOnEvent note_on{channel, note.pitch, 0.0, note.velocity / 127.0, 0, -1};
            Event event{0, offset, 0, Event::kIsLive, Event::kNoteOnEvent};
            event.noteOn = note_on;
            data.outputEvents->addEvent(event);
            note_state[channel][note.pitch] = true;
        }
}

tresult PLUGIN_API Eris::process(ProcessData& data) {
    try {
        IParameterChanges* paramChanges = data.inputParameterChanges;
        if (paramChanges) {
            int32 numParamsChanged = paramChanges->getParameterCount();
            for (int32 i = 0; i < numParamsChanged; i++) {
                IParamValueQueue* paramQueue = paramChanges->getParameterData(i);
                if (paramQueue) {
                    ParamValue value;
                    int32 sampleOffset;
                    int32 numPoints = paramQueue->getPointCount();
                    switch (paramQueue->getParameterId()) {
                        case kTimeWindowId:
                            if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue)
                                set_time_window(5000 * value);
                            block_size = time_window_to_block_size();
                            break;
                        case kNoteCountId:
                            if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue)
                                note_count = 127 * value;
                            break;
                    }
                }
            }
        }

        if (data.numInputs == 0 || data.numOutputs == 0)
            return kResultOk;

        int32 numChannels = data.inputs[0].numChannels;

        uint32 sampleFramesSize = getSampleFramesSizeInBytes(processSetup, data.numSamples);
        void** in = getChannelBuffersPointer(processSetup, data.inputs[0]);
        void** out = getChannelBuffersPointer(processSetup, data.outputs[0]);

        if (data.inputs[0].silenceFlags != 0) {
            data.outputs[0].silenceFlags = data.inputs[0].silenceFlags;

            for (int32 i = 0; i < numChannels; i++)
                if (in[i] != out[i])
                    memset(out[i], 0, sampleFramesSize);

            return kResultOk;
        }

        data.outputs[0].silenceFlags = 0;

        for (size_t channel = 0; channel < numChannels; ++channel)
            if (note_state.find(channel) == note_state.end())
                note_state[channel] = std::map<unsigned int, bool>();

        int offset = 0;
        int event_offset = -spill_samples;
        int remaining = data.numSamples + spill_samples;
        while (remaining > 0) {
            int count = (remaining >= block_size) ? block_size : remaining;
            for (size_t i = 0; i < numChannels; ++i) {
                if (data.symbolicSampleSize == kSample32)
                    remaining -= buffer_samples<Sample32>(i, ((Sample32**)in)[i] + offset, count);
                else
                    remaining -= buffer_samples<Sample64>(i, ((Sample64**)in)[i] + offset, count);
            }

            if (count == block_size) {
                terminate_notes(event_offset, data);
                add_notes(convert(numChannels), event_offset, data);
                event_offset += count;
            }

            offset += count;
        }
    } catch (const std::exception& e) {
        log(fmt::format("Error: {}", e.what()));
    }

    return kResultOk;
}

int Eris::time_window_to_block_size() {
    return static_cast<int>((processSetup.sampleRate / 1000.0) * time_window);
}

void Eris::clear_buffers() {
    sample_buffer = std::vector<std::vector<double>>();
    spill_samples = 0;
}

void Eris::set_time_window(const int32 time_window) {
    this->time_window = time_window >= 5 ? time_window : 5;
    int old_block_size = block_size;
    block_size = time_window_to_block_size();

    if (old_block_size != block_size) {
        clear_buffers();
    }
}

std::vector<std::vector<a2m::Note>> Eris::convert(int channels) {
    converter.set_samplerate(processSetup.sampleRate);
    converter.set_block_size(block_size);
    converter.set_note_count(note_count);

    std::vector<std::vector<a2m::Note>> notes;

    try {
        for (size_t channel = 0; channel < channels; ++channel) {
            notes.push_back(std::vector<a2m::Note>());
            for (auto& note : converter.convert(sample_buffer[channel].data()))
                notes[channel].push_back(note);
        }
    } catch (const std::runtime_error& e) {
    }

    return notes;
}

tresult PLUGIN_API Eris::setState(IBStream* state) {
    IBStreamer streamer(state, kLittleEndian);
    int32 saved_time_window = 5;
    if (streamer.readInt32(saved_time_window) == false)
        return kResultFalse;

    int32 saved_note_count = 0;
    if (streamer.readInt32(saved_note_count) == false)
        return kResultFalse;

    set_time_window(saved_time_window);
    note_count = saved_note_count;

    setParamNormalized(kTimeWindowId, saved_time_window);
    setParamNormalized(kNoteCountId, note_count);

    return kResultOk;
}

tresult PLUGIN_API Eris::getState(IBStream* state) {
    IBStreamer streamer(state, kLittleEndian);

    streamer.writeInt32(time_window);
    streamer.writeInt32(note_count);

    return kResultOk;
}

tresult PLUGIN_API Eris::setupProcessing(ProcessSetup& newSetup) {
    currentProcessMode = newSetup.processMode;

    return SingleComponentEffect::setupProcessing(newSetup);
}

tresult PLUGIN_API Eris::setBusArrangements(SpeakerArrangement* inputs,
                                            int32 numIns,
                                            SpeakerArrangement* outputs,
                                            int32 numOuts) {
    if (numIns == 1 && numOuts == 1) {
        // the host wants Mono => Mono (or 1 channel -> 1 channel)
        if (SpeakerArr::getChannelCount(inputs[0]) == 1 && SpeakerArr::getChannelCount(outputs[0]) == 1) {
            auto* bus = FCast<AudioBus>(audioInputs.at(0));
            if (bus) {
                // check if we are Mono => Mono, if not we need to recreate the busses
                if (bus->getArrangement() != inputs[0]) {
                    bus->setArrangement(inputs[0]);
                    bus->setName(STR16("Mono In"));
                    if (auto* busOut = FCast<AudioBus>(audioOutputs.at(0))) {
                        busOut->setArrangement(inputs[0]);
                        busOut->setName(STR16("Mono Out"));
                    }
                }
                return kResultOk;
            }
        }
        // the host wants something else than Mono => Mono, in this case we are always Stereo => Stereo
        else {
            auto* bus = FCast<AudioBus>(audioInputs.at(0));
            if (bus) {
                tresult result = kResultFalse;

                // the host wants 2->2 (could be LsRs -> LsRs)
                if (SpeakerArr::getChannelCount(inputs[0]) == 2 && SpeakerArr::getChannelCount(outputs[0]) == 2) {
                    bus->setArrangement(inputs[0]);
                    bus->setName(STR16("Stereo In"));
                    if (auto* busOut = FCast<AudioBus>(audioOutputs.at(0))) {
                        busOut->setArrangement(outputs[0]);
                        busOut->setName(STR16("Stereo Out"));
                    }
                    result = kResultTrue;
                }
                // the host want something different than 1->1 or 2->2 : in this case we want stereo
                else if (bus->getArrangement() != SpeakerArr::kStereo) {
                    bus->setArrangement(SpeakerArr::kStereo);
                    bus->setName(STR16("Stereo In"));
                    if (auto* busOut = FCast<AudioBus>(audioOutputs.at(0))) {
                        busOut->setArrangement(SpeakerArr::kStereo);
                        busOut->setName(STR16("Stereo Out"));
                    }

                    result = kResultFalse;
                }

                return result;
            }
        }
    }
    return kResultFalse;
}

tresult PLUGIN_API Eris::canProcessSampleSize(int32 symbolicSampleSize) {
    if (symbolicSampleSize == kSample32)
        return kResultTrue;

    // we support double processing
    if (symbolicSampleSize == kSample64)
        return kResultTrue;

    return kResultFalse;
}

IPlugView* PLUGIN_API Eris::createView(const char* name) {
    // someone wants my editor
    if (name && FIDStringsEqual(name, ViewType::kEditor)) {
        auto* view = new VST3Editor(this, "view", "eris.uidesc");
        return view;
    }
    return nullptr;
}

tresult PLUGIN_API Eris::getMidiControllerAssignment(int32 busIndex,
                                                     int16 /*midiChannel*/,
                                                     CtrlNumber midiControllerNumber,
                                                     ParamID& tag) {
    // we support for the Gain parameter all MIDI Channel but only first bus (there is only one!)
    if (busIndex == 0 && midiControllerNumber == kCtrlVolume) {
        return kResultTrue;
    }
    return kResultFalse;
}

IController* Eris::createSubController(UTF8StringPtr name,
                                       const IUIDescription* /*description*/,
                                       VST3Editor* /*editor*/) {
    if (UTF8StringView(name) == "MessageController") {
        auto* controller = new UIMessageController(this);
        addUIMessageController(controller);
        return controller;
    }
    return nullptr;
}

tresult PLUGIN_API Eris::setEditorState(IBStream* state) {
    tresult result = kResultFalse;

    int8 byteOrder;
    if ((result = state->read(&byteOrder, sizeof(int8))) != kResultTrue)
        return result;
    if ((result = state->read(defaultMessageText, 128 * sizeof(TChar))) != kResultTrue)
        return result;

    // if the byteorder doesn't match, byte swap the text array ...
    if (byteOrder != BYTEORDER) {
        for (int32 i = 0; i < 128; i++)
            SWAP_16(defaultMessageText[i])
    }

    for (auto& uiMessageController : uiMessageControllers)
        uiMessageController->setMessageText(defaultMessageText);

    return result;
}

tresult PLUGIN_API Eris::getEditorState(IBStream* state) {
    // here we can save UI settings for example

    IBStreamer streamer(state, kLittleEndian);

    // as we save a Unicode string, we must know the byteorder when setState is called
    int8 byteOrder = BYTEORDER;
    if (streamer.writeInt8(byteOrder) == false)
        return kResultFalse;

    if (streamer.writeRaw(defaultMessageText, 128 * sizeof(TChar)) == false)
        return kResultFalse;

    return kResultTrue;
}

tresult PLUGIN_API Eris::setParamNormalized(ParamID tag, ParamValue value) {
    // called from host to update our parameters state
    tresult result = SingleComponentEffect::setParamNormalized(tag, value);
    return result;
}

tresult PLUGIN_API Eris::getParamStringByValue(ParamID tag, ParamValue valueNormalized, String128 string) {
    return SingleComponentEffect::getParamStringByValue(tag, valueNormalized, string);
}

tresult PLUGIN_API Eris::getParamValueByString(ParamID tag, TChar* string, ParamValue& valueNormalized) {
    return SingleComponentEffect::getParamValueByString(tag, string, valueNormalized);
}

void Eris::addUIMessageController(UIMessageController* controller) {
    uiMessageControllers.push_back(controller);
}

void Eris::removeUIMessageController(UIMessageController* controller) {
    UIMessageControllerList::const_iterator it =
        std::find(uiMessageControllers.begin(), uiMessageControllers.end(), controller);
    if (it != uiMessageControllers.end())
        uiMessageControllers.erase(it);
}

void Eris::setDefaultMessageText(String128 text) {
    UString str(defaultMessageText, 128);
    str.assign(text, -1);
}

TChar* Eris::getDefaultMessageText() {
    return defaultMessageText;
}

tresult PLUGIN_API Eris::queryInterface(const TUID iid, void** obj) {
    DEF_INTERFACE(IMidiMapping)
    return SingleComponentEffect::queryInterface(iid, obj);
}

enum {
    // UI size
    kEditorWidth = 350,
    kEditorHeight = 120
};

}  // namespace Vst
}  // namespace Steinberg

bool InitModule() {
    return true;
}

bool DeinitModule() {
    return true;
}

BEGIN_FACTORY_DEF("Neil Jones", "https://github.com/NFJones", "mailto:neil.jones.music@gmail.com")

DEF_CLASS2(INLINE_UID(0xB9F9ADE1, 0xCD9C4B6D, 0xA57E61E3, 0x123535FD),
           PClassInfo::kManyInstances,            // cardinality
           kVstAudioEffectClass,                  // the component category (do not changed this)
           "Eris VST3",                           // here the plug-in name (to be changed)
           0,                                     // single component effects cannot be distributed so this is zero
           "Fx",                                  // Subcategory for this plug-in (to be changed)
           FULL_VERSION_STR,                      // Plug-in version (to be changed)
           kVstVersionString,                     // the VST 3 SDK version (do not changed this, use always this define)
           Steinberg::Vst::Eris::createInstance)  // function pointer called when this component should be instantiated

END_FACTORY
