#include "eris.h"
#include "erisparamids.h"
#include "erisuimessagecontroller.h"
#include "version.h"

#include "base/source/fstreamer.h"
#include "pluginterfaces/base/ibstream.h"
#include "pluginterfaces/base/ustring.h"
#include "pluginterfaces/vst/ivstevents.h"
#include "pluginterfaces/vst/ivstmidicontrollers.h"
#include "pluginterfaces/vst/ivstparameterchanges.h"
#include "pluginterfaces/vst/vstpresetkeys.h"
#include "public.sdk/source/main/dllmain.cpp"
#include "public.sdk/source/main/pluginfactory.h"
#include "public.sdk/source/vst/vstaudioprocessoralgo.h"
#include "vstgui/plugin-bindings/vst3editor.h"

#include <fmt/format.h>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>

#define ERIS_TEST 1

using namespace VSTGUI;

static void log(const std::string& msg) {
    std::ofstream outfile("C:/Users/neilf/Downloads/eris.log", std::ios_base::app);
    outfile << msg << std::endl;
}

namespace Steinberg {
namespace Vst {

Eris::Eris()
    : time_window(5),
      time_window_param(5),
      threshold(0),
      block_size(0),
      note_count(0),
      sync(false),
      beat_numerator(1),
      beat_denominator(4),
      combine_notes(false),
      currentProcessMode(-1),
      converter(processSetup.sampleRate, 0),
      transpose(0),
      pitch_set_index(0),
      sample_rate(0),
      note_min(0),
      note_max(127),
      max_length(0) {
    clear_buffers();
    set_time_window(time_window_param);
    for (int32 channel = 0; channel < 2; ++channel)
        if (note_state.find(channel) == note_state.end())
            note_state[channel] = std::map<unsigned int, std::pair<bool, uint32>>();
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
        converter.set_block_size(block_size);
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

void Eris::terminate_notes(const int channel, int offset, ProcessData& data, const std::vector<a2m::Note>& new_notes) {
    for (auto& state : note_state.at(channel)) {
        if (state.second.first) {
            if ((!combine_notes) || (combine_notes && ((find(new_notes.begin(), new_notes.end(),
                                                             a2m::Note(state.first, 0)) == new_notes.end()) ||
                                                       (state.second.second > max_length && max_length > 0)))) {
                NoteOffEvent note_off{static_cast<int16>(channel), static_cast<int16>(state.first), 0.0f, -1, 0.0f};
                Event event{0, offset, 0.0, Event::kIsLive, Event::kNoteOffEvent};
                event.noteOff = note_off;
                data.outputEvents->addEvent(event);
                note_state.at(channel)[state.first].first = false;
                note_state.at(channel)[state.first].second = 0;
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
        if ((!combine_notes) || (combine_notes && !note_state.at(channel)[note.pitch].first)) {
            Event event{0, offset, 0.0, Event::kIsLive, Event::kNoteOnEvent};
            event.noteOn = note_on;
            data.outputEvents->addEvent(event);
            note_state.at(channel)[note.pitch].first = true;
            note_state.at(channel)[note.pitch].second = 1;
        } else if (combine_notes && note_state.at(channel)[note.pitch].first)
            note_state.at(channel)[note.pitch].second += 1;
    }
}

void Eris::convert(ProcessData& data) {
    void** in = getChannelBuffersPointer(processSetup, data.inputs[0]);
    if (sample_rate != processSetup.sampleRate) {
        converter.set_samplerate(processSetup.sampleRate);
        if (sync)
            set_beat();
        else
            set_time_window(time_window_param);
        sample_rate = processSetup.sampleRate;
    }
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

tresult PLUGIN_API Eris::initialize(FUnknown* context) {
    tresult result = SingleComponentEffect::initialize(context);
    if (result != kResultOk)
        return result;

    addAudioInput(STR16("Stereo In"), SpeakerArr::kStereo);
    addAudioOutput(STR16("Stereo Out"), SpeakerArr::kStereo);
    addEventOutput(STR16("Event Out"), 2);

    auto* time_window_param = new RangeParameter(STR16("Time Window"), kTimeWindowId, nullptr, 5.0, 1000.0, 5.0, 995);
    parameters.addParameter(time_window_param);

    auto* threshold_param = new RangeParameter(STR16("Threshold"), kThresholdId, nullptr, 0.0, 127.0, 0.0, 127);
    parameters.addParameter(threshold_param);

    auto* ceiling_param = new RangeParameter(STR16("Normalize"), kCeilingId, nullptr, 1.0, 127.0, 127.0, 126);
    parameters.addParameter(ceiling_param);

    auto* note_count_param = new RangeParameter(STR16("Note Count"), kNoteCountId, nullptr, 0.0, 127.0, 0.0, 127);
    parameters.addParameter(note_count_param);

    auto* note_min_param = new RangeParameter(STR16("Note Min"), kNoteMinId, nullptr, 0.0, 127.0, 0.0, 127);
    parameters.addParameter(note_min_param);

    auto* note_max_param = new RangeParameter(STR16("Note Max"), kNoteMaxId, nullptr, 0.0, 127.0, 127.0, 127);
    parameters.addParameter(note_max_param);

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

    auto* max_length_param = new RangeParameter(STR16("Max Length"), kMaxLengthId, nullptr, 0.0, 32, 0.0, 32);
    parameters.addParameter(max_length_param);

    auto* transpose_param = new RangeParameter(STR16("Transpose"), kTransposeId, nullptr, -127.0, 127.0, 0.0, 255);
    parameters.addParameter(transpose_param);

    auto* pitch_set_param = new StringListParameter(STR16("Scale"), kPitchSetId);
    for (const auto& set : njones::audio::PITCH_SET) {
        String128 str;
        str8ToStr16(str, set.first.c_str(), static_cast<int32>(set.first.length()));
        pitch_set_param->appendString(str);
    }
    parameters.addParameter(pitch_set_param);

    return result;
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
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            note_count = 128 * value;
                            converter.set_note_count(note_count);
                        }
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
                    case kThresholdId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            threshold = 127 * value;
                            converter.set_activation_level(threshold / 127.0);
                        }
                        break;
                    case kCeilingId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            ceiling = 127 * value;
                            converter.set_ceiling(ceiling / 127.0);
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
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue)
                            combine_notes = static_cast<int>(value) == 1;
                        break;
                    case kTransposeId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            transpose = 255 * value - 127;
                            converter.set_transpose(transpose);
                        }
                        break;
                    case kPitchSetId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            pitch_set_index = value * (njones::audio::PITCH_SET.size() - 1);
                            pitch_set = njones::audio::PITCH_SET.at(pitch_set_index).second;
                            converter.set_pitch_set(pitch_set);
                        }
                        break;
                    case kNoteMinId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            note_min = 127 * value;
                            converter.set_pitch_range(std::array<unsigned int, 2>{note_min, note_max});
                        }
                    case kNoteMaxId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            note_max = 127 * value;
                            converter.set_pitch_range(std::array<unsigned int, 2>{note_min, note_max});
                        }
                        break;
                    case kMaxLengthId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            max_length = 32 * value;
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
    int32 saved_threshold = 0;
    if (streamer.readInt32(saved_threshold) == false)
        return kResultFalse;
    int32 saved_transpose = 0;
    if (streamer.readInt32(saved_transpose) == false)
        return kResultFalse;
    int32 saved_ceiling = 0;
    if (streamer.readInt32(saved_ceiling) == false)
        return kResultFalse;
    int32 saved_pitch_set_index = 0;
    if (streamer.readInt32(saved_pitch_set_index) == false)
        return kResultFalse;
    uint32 saved_note_min = 0;
    if (streamer.readInt32u(saved_note_min) == false)
        return kResultFalse;
    uint32 saved_note_max = 0;
    if (streamer.readInt32u(saved_note_max) == false)
        return kResultFalse;
    uint32 saved_max_length = 0;
    if (streamer.readInt32u(saved_max_length) == false)
        return kResultFalse;

    time_window_param = saved_time_window;
    threshold = saved_threshold;
    converter.set_activation_level(threshold / 127.0);
    note_count = saved_note_count;
    converter.set_note_count(note_count);
    sync = saved_sync;
    beat_numerator = saved_beat_numerator;
    beat_denominator = saved_beat_denominator;
    combine_notes = saved_combine_notes;
    transpose = saved_transpose;
    converter.set_transpose(transpose);
    ceiling = saved_ceiling;
    converter.set_ceiling(ceiling / 127.0);
    pitch_set_index = saved_pitch_set_index;
    pitch_set = njones::audio::PITCH_SET.at(pitch_set_index).second;
    converter.set_pitch_set(pitch_set);
    note_min = saved_note_min;
    note_max = saved_note_max;
    converter.set_pitch_range(std::array<unsigned int, 2>{note_min, note_max});
    max_length = saved_max_length;

    setParamNormalized(kTimeWindowId, time_window / 1000.0);
    setParamNormalized(kNoteCountId, note_count / 127.0);
    setParamNormalized(kSyncId, sync);
    setParamNormalized(kBeatNumeratorId, beat_numerator / 16.0);
    setParamNormalized(kBeatDenominatorId, beat_denominator / 16.0);
    setParamNormalized(kCombineNotesId, combine_notes);
    setParamNormalized(kThresholdId, threshold / 127.0);
    setParamNormalized(kTransposeId, (transpose + 127) / 255.0);
    setParamNormalized(kCeilingId, ceiling / 127.0);
    setParamNormalized(kPitchSetId, static_cast<double>(pitch_set_index) / njones::audio::PITCH_SET.size());
    setParamNormalized(kNoteMinId, note_min / 127.0);
    setParamNormalized(kNoteMaxId, note_max / 127.0);
    setParamNormalized(kMaxLengthId, max_length / 32.0);

    if (sync)
        set_beat();
    else
        set_time_window(time_window_param);

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
    streamer.writeInt32(threshold);
    streamer.writeInt32(transpose);
    streamer.writeInt32(ceiling);
    streamer.writeInt32(pitch_set_index);
    streamer.writeInt32u(note_min);
    streamer.writeInt32u(note_max);
    streamer.writeInt32u(max_length);

    return kResultOk;
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
