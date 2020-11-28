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
    info.defaultNormalizedValue = 5 / 1000.0;
    info.unitId = kRootUnitId;

    setNormalized(info.defaultNormalizedValue);
}

void TimeWindowParameter::toString(ParamValue normValue, String128 string) const {
    char text[32];
    int val = static_cast<int>(normValue * 1000);
    val = (val > 5) ? val : 5;
    sprintf(text, "%d", val);

    Steinberg::UString(string, 128).fromAscii(text);
}

bool TimeWindowParameter::fromString(const TChar* string, ParamValue& normValue) const {
    Steinberg::UString wrapper((TChar*)string, -1);  // don't know buffer size here!
    int64 tmp = 0;
    if (wrapper.scanInt(tmp)) {
        normValue = tmp / 1000.0;
        return true;
    }
    return false;
}

Eris::Eris()
    : time_window(5),
      time_window_param(5),
      block_size(0),
      note_count(0),
      sync(false),
      beat_numerator(1),
      beat_denominator(4),
      combine_notes(false),
      currentProcessMode(-1),
      converter(processSetup.sampleRate, 0) {
    clear_buffers();
    set_time_window(time_window_param);
    for (int32 channel = 0; channel < 2; ++channel)
        if (note_state.find(channel) == note_state.end())
            note_state[channel] = std::map<unsigned int, bool>();
}

tresult PLUGIN_API Eris::initialize(FUnknown* context) {
    tresult result = SingleComponentEffect::initialize(context);
    if (result != kResultOk)
        return result;

    addAudioInput(STR16("Stereo In"), SpeakerArr::kStereo);
    addAudioOutput(STR16("Stereo Out"), SpeakerArr::kStereo);
    addEventOutput(STR16("Event Out"), 2);

    auto* time_window_param = new RangeParameter(STR16("Time Window"), kTimeWindowId, nullptr, 5.0, 1000.0, 5.0, 995);
    parameters.addParameter(time_window_param);

    auto* note_count_param = new RangeParameter(STR16("Note Count"), kNoteCountId, nullptr, 0.0, 127.0, 0.0, 127);
    parameters.addParameter(note_count_param);

    auto* sync_param = new RangeParameter(STR16("Sync"), kSyncId, nullptr, 0.0, 1.0, 0.0, 1);
    parameters.addParameter(sync_param);

    auto* beat_numerator_param =
        new RangeParameter(STR16("Beat Numerator"), kBeatNumeratorId, nullptr, 1.0, 16.0, 1.0, 15);
    parameters.addParameter(beat_numerator_param);

    auto* beat_denominator_param =
        new RangeParameter(STR16("Beat Denominator"), kBeatDenominatorId, nullptr, 1.0, 16.0, 1.0, 15);
    parameters.addParameter(beat_denominator_param);

    auto* combine_notes_param = new RangeParameter(STR16("Combine Notes"), kCombineNotesId, nullptr, 0.0, 1.0, 0.0, 1);
    parameters.addParameter(combine_notes_param);

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

void Eris::terminate_notes(const int channel, int offset, ProcessData& data, const std::vector<a2m::Note>& new_notes) {
    for (auto& state : note_state.at(channel)) {
        if (state.second) {
            if ((!combine_notes) || (combine_notes && (find(new_notes.begin(), new_notes.end(),
                                                            a2m::Note(state.first, 0)) == new_notes.end()))) {
                NoteOffEvent note_off{static_cast<int16>(channel), static_cast<int16>(state.first), 0.0f, -1, 0.0f};
                Event event{0, offset, 0.0, Event::kIsLive, Event::kNoteOffEvent};
                event.noteOff = note_off;
                data.outputEvents->addEvent(event);
                note_state.at(channel)[state.first] = false;
            }
        }
    }
}

void Eris::initiate_notes(const int channel, const std::vector<a2m::Note>& notes, int offset, ProcessData& data) {
    for (auto& note : notes) {
        NoteOnEvent note_on{static_cast<int16>(channel),
                            static_cast<int16>(note.pitch),
                            0.0f,
                            static_cast<float>(note.velocity / 127.0),
                            0,
                            -1};
        if ((!combine_notes) || (combine_notes && !note_state.at(channel)[note.pitch])) {
            Event event{0, offset, 0.0, Event::kIsLive, Event::kNoteOnEvent};
            event.noteOn = note_on;
            data.outputEvents->addEvent(event);
            note_state.at(channel)[note.pitch] = true;
        }
    }
}

void Eris::convert(ProcessData& data) {
    void** in = getChannelBuffersPointer(processSetup, data.inputs[0]);
    converter.set_samplerate(processSetup.sampleRate);
    converter.set_block_size(block_size);
    converter.set_note_count(note_count);
    std::function<void(const int, double*, const int)> processor = [&](const int channel, double* samples,
                                                                       const int event_offset) -> void {
        auto notes = converter.convert(samples);
        terminate_notes(channel, event_offset, data, notes);
        initiate_notes(channel, notes, event_offset, data);
    };

    if (data.symbolicSampleSize == kSample32) {
        buffer_32.resize(2, block_size);
        buffer_32.set_processor(processor);
        buffer_32.add((Sample32**)in, data.numSamples);
    } else {
        buffer_64.resize(2, block_size);
        buffer_64.set_processor(processor);
        buffer_64.add((Sample64**)in, data.numSamples);
    }
}

tresult PLUGIN_API Eris::process(ProcessData& data) {
    process_parameters(data);
    if (data.numInputs == 0 || data.numOutputs == 0)
        return kResultOk;
    if (data.inputs[0].silenceFlags == 0) {
        data.outputs[0].silenceFlags = 1;
        convert(data);
    }

    return kResultOk;
}

void Eris::process_parameters(ProcessData& data) {
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
                    case kNoteCountId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue)
                            note_count = 128 * value;
                        break;
                    case kSyncId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            sync = static_cast<int>(value) == 1;
                            if (sync)
                                set_beat();
                            else
                                set_time_window(time_window_param);
                        }
                        break;
                    case kTimeWindowId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            time_window_param = 1000 * value;
                            if (!sync) {
                                set_time_window(time_window_param);
                            }
                        }
                        break;
                    case kBeatNumeratorId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            beat_numerator = 16 * value;
                            set_beat();
                        }
                        break;
                    case kBeatDenominatorId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            beat_denominator = 16 * value;
                            set_beat();
                        }
                        break;
                    case kCombineNotesId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            combine_notes = static_cast<int>(value) == 1;
                        }
                        break;
                }
            }
        }
    }

    if (data.processContext->tempo != tempo) {
        tempo = data.processContext->tempo;
        set_beat();
    }
}

int Eris::time_window_to_block_size() {
    return static_cast<int>((processSetup.sampleRate / 1000.0) * time_window);
}

void Eris::clear_buffers() {
    buffer_32.clear();
    buffer_64.clear();
}

void Eris::set_time_window(const int32 time_window) {
    this->time_window = time_window >= 5 ? time_window : 5;
    int old_block_size = block_size;
    block_size = time_window_to_block_size();

    if (old_block_size != block_size) {
        clear_buffers();
    }
}

void Eris::set_beat() {
    if (sync) {
        beat_numerator = beat_numerator < 1 ? 1 : beat_numerator;
        beat_denominator = beat_denominator < 1 ? 1 : beat_denominator;
        double ratio = static_cast<double>(beat_numerator) / static_cast<double>(beat_denominator);
        double second_per_beat = 60.0 / tempo;
        double beat_per_ms = second_per_beat * 1000;
        set_time_window(ratio * beat_per_ms);
    }
}

tresult PLUGIN_API Eris::setState(IBStream* state) {
    IBStreamer streamer(state, kLittleEndian);
    int32 saved_time_window = 5;
    if (streamer.readInt32(saved_time_window) == false)
        return kResultFalse;

    int32 saved_note_count = 0;
    if (streamer.readInt32(saved_note_count) == false)
        return kResultFalse;

    bool saved_sync = 0;
    if (streamer.readBool(saved_sync) == false)
        return kResultFalse;

    int32 saved_beat_numerator = 0;
    if (streamer.readInt32(saved_beat_numerator) == false)
        return kResultFalse;

    int32 saved_beat_denominator = 0;
    if (streamer.readInt32(saved_beat_denominator) == false)
        return kResultFalse;

    bool saved_combine_notes = 0;
    if (streamer.readBool(saved_combine_notes) == false)
        return kResultFalse;

    time_window_param = saved_time_window;
    note_count = saved_note_count;
    sync = saved_sync;
    beat_numerator = saved_beat_numerator;
    beat_denominator = saved_beat_denominator;
    combine_notes = saved_combine_notes;

    setParamNormalized(kTimeWindowId, time_window / 1000.0);
    setParamNormalized(kNoteCountId, note_count / 127.0);
    setParamNormalized(kSyncId, sync);
    setParamNormalized(kBeatNumeratorId, beat_numerator / 16.0);
    setParamNormalized(kBeatDenominatorId, beat_denominator / 16.0);
    setParamNormalized(kCombineNotesId, combine_notes);

    if (sync) {
        set_beat();
    } else {
        set_time_window(time_window_param);
    }

    return kResultOk;
}

tresult PLUGIN_API Eris::getState(IBStream* state) {
    IBStreamer streamer(state, kLittleEndian);

    streamer.writeInt32(time_window_param);
    streamer.writeInt32(note_count);
    streamer.writeBool(sync);
    streamer.writeInt32(beat_numerator);
    streamer.writeInt32(beat_denominator);
    streamer.writeBool(combine_notes);

    return kResultOk;
}

tresult PLUGIN_API Eris::setupProcessing(ProcessSetup& newSetup) {
    currentProcessMode = newSetup.processMode;

    return SingleComponentEffect::setupProcessing(newSetup);
}

tresult PLUGIN_API Eris::setBusArrangements(SpeakerArrangement* inputs, int32 numIns, SpeakerArrangement*, int32) {
    if (numIns == 1) {
        // the host wants Mono => Mono (or 1 channel -> 1 channel)
        if (SpeakerArr::getChannelCount(inputs[0]) == 1) {
            auto* bus = FCast<AudioBus>(audioInputs.at(0));
            if (bus) {
                // check if we are Mono => Mono, if not we need to recreate the busses
                if (bus->getArrangement() != inputs[0]) {
                    bus->setArrangement(inputs[0]);
                    bus->setName(STR16("Mono In"));
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
                if (SpeakerArr::getChannelCount(inputs[0]) == 2) {
                    bus->setArrangement(inputs[0]);
                    bus->setName(STR16("Stereo In"));
                    result = kResultTrue;
                }
                // the host want something different than 1->1 or 2->2 : in this case we want stereo
                else if (bus->getArrangement() != SpeakerArr::kStereo) {
                    bus->setArrangement(SpeakerArr::kStereo);
                    bus->setName(STR16("Stereo In"));

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
