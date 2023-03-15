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

using namespace VSTGUI;

namespace Steinberg {
namespace Vst {

Eris::Eris()
    :
#if ERIS_TEST
      logger("127.0.0.1", 4567),
#endif
      time_window(500.0),
      time_window_param(500),
      threshold(0),
      block_size(0),
      real_block_size(0.0),
      note_count(5),
      sync(1),
      beat_numerator(2),
      beat_denominator(5),
      combine_notes(1),
      currentProcessMode(-1),
      converter(processSetup.sampleRate, 0),
      pitch_set_index(1),
      pitch_set(njones::audio::PITCH_SET.at(1).second),
      sample_rate(0),
      note_min(0),
      note_max(127),
      max_length(3),
      key(0),
      octave(0),
      ceiling(30),
      transpose(0),
      tempo(0) {
    clear_buffers();
    set_beat();
    for (int32 channel = 0; channel < 2; ++channel)
        note_state.push_back(std::array<NoteState, 128>());
#if ERIS_TEST
    converter.set_logger([&](const std::string& msg) { log(msg); });
#endif
}

void Eris::log(const std::string& msg) {
#if ERIS_TEST
    logger.log(fmt::format("{}\r\n", msg));
#endif
}

double Eris::time_window_to_block_size() {
    return (processSetup.sampleRate / 1000.0) * time_window;
}

void Eris::clear_buffers() {
    buffer_32.clear();
    buffer_64.clear();
}

void Eris::set_time_window(const double time_window) {
    this->time_window = time_window >= 5.0 ? time_window : 5.0;
    this->time_window = time_window >= 10000.0 ? 10000.0 : time_window;
    const int old_block_size = block_size;
    real_block_size = time_window_to_block_size();
    block_size = static_cast<int>(real_block_size);

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

void Eris::set_rotation() {
    std::vector<unsigned int> rotated;
    std::copy(pitch_set.begin(), pitch_set.end(), std::back_inserter(rotated));
    for (auto& pitch : rotated)
        pitch = (pitch + key) % 12;
    converter.set_pitch_set(rotated);
}

NoteState::NoteState(const bool is_active, const int32 count) : is_active(is_active), count(count) {}

void Eris::terminate_notes(const int channel,
                           int offset,
                           ProcessData& data,
                           const std::vector<njones::audio::a2m::Note>& new_notes) {
    for (unsigned int pitch = 0; pitch < note_state[channel].size(); ++pitch) {
        auto& state = note_state[channel][pitch];
        if (state.is_active) {
            if ((!combine_notes) ||
                (combine_notes && ((find(new_notes.begin(), new_notes.end(),
                                         njones::audio::a2m::Note(pitch, pitch, 0)) == new_notes.end()) ||
                                   (state.count > max_length && max_length > 0)))) {
                NoteOffEvent note_off{static_cast<int16>(channel), static_cast<int16>(pitch), 0.0f, -1, 0.0f};
                Event event{0, offset, 0.0, Event::kIsLive, Event::kNoteOffEvent};
                event.noteOff = note_off;
                data.outputEvents->addEvent(event);
                state.is_active = false;
                state.count = 0;
            }
        }
    }
}

void Eris::initiate_notes(const int channel,
                          const std::vector<njones::audio::a2m::Note>& notes,
                          int offset,
                          ProcessData& data) {
    for (auto& note : notes) {
        NoteOnEvent note_on{static_cast<int16>(channel),
                            static_cast<int16>(note.pitch),
                            0.0f,
                            static_cast<float>(note.velocity / 127.0),
                            0,
                            -1};
        auto& state = note_state[channel][note.pitch];
        if ((!combine_notes) || (combine_notes && !state.is_active)) {
            Event event{0, offset, 0.0, Event::kIsLive, Event::kNoteOnEvent};
            event.noteOn = note_on;
            data.outputEvents->addEvent(event);
            state.is_active = true;
            state.count = 1;
        } else if (combine_notes && state.is_active)
            state.count += 1;
    }
}

void Eris::convert(ProcessData& data) {
    void** in = getChannelBuffersPointer(processSetup, data.inputs[0]);
    if (sample_rate != data.processContext->sampleRate) {
        converter.set_samplerate(data.processContext->sampleRate);
        sample_rate = data.processContext->sampleRate;
    }

    auto quantize = [&](int event_offset) -> int {
        if (sync) {
            int block_number = event_offset / real_block_size;
            int quantized_block_number = (block_number / beat_numerator) * beat_numerator;
            int quantized_offset = quantized_block_number * real_block_size;
            return quantized_offset;
        } else {
            return event_offset;
        }
    };

    auto processor = [&](const int channel, double* samples, const int event_offset) -> void {
        auto notes = converter.convert(samples);
        int offset = quantize(event_offset);

        terminate_notes(channel, offset, data, notes);
        initiate_notes(channel, notes, offset, data);
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
    try {
        process_inputs(data);
        process_parameters(data);
        if (data.numInputs == 0 || data.numOutputs == 0)
            return kResultOk;
        if (data.inputs && data.outputs && data.inputs[0].silenceFlags == 0) {
            data.outputs[0].silenceFlags = 1;
            convert(data);
        }
        return kResultOk;
    } catch (const std::exception&) {
        return kInternalError;
    }
}

void Eris::set_parameter(ProcessData& data, const int param_id, const double value) {
    int32 queue_index = 0;
    int32 param_index = 0;
    IParameterChanges* outParamChanges = data.outputParameterChanges;
    if (outParamChanges) {
        IParamValueQueue* paramQueue = outParamChanges->addParameterData(param_id, queue_index);
        if (paramQueue)
            paramQueue->addPoint(0, value, param_index);
    }
    setParamNormalized(param_id, value);
}

void Eris::process_inputs(ProcessData& data) {
    const int input_count = data.inputEvents->getEventCount();

    if (input_count == 0)
        return;

    std::vector<std::pair<std::pair<int, int>, std::function<void(const int)>>> input_processors = {
        {{0, 20},
         [&](const int pitch) {
             // Octave
             octave = pitch - 10;
             converter.set_transpose(octave * 12 + transpose);
             set_parameter(data, kOctaveId, (octave + 10) / 20.0);
         }},
        {{24, 24 + njones::audio::KEY.size() - 1},
         [&](const int pitch) {
             // Key
             key = pitch - 24;
             set_rotation();
             set_parameter(data, kKeyId, static_cast<double>(key) / (njones::audio::KEY.size() - 1));
         }},
        {{24 + njones::audio::KEY.size(), 24 + njones::audio::KEY.size() + njones::audio::PITCH_SET.size() - 1},
         [&](const int pitch) {
             // Scale
             pitch_set_index = static_cast<int32>(static_cast<unsigned int>(pitch) - (24 + njones::audio::KEY.size()));
             pitch_set = njones::audio::PITCH_SET.at(pitch_set_index).second;
             set_rotation();
             set_parameter(data, kPitchSetId,
                           static_cast<double>(pitch_set_index) / (njones::audio::PITCH_SET.size() - 1));
         }},
        {{69, 101},
         [&](const int pitch) {
             // Max Combine Length
             max_length = pitch - 69;
             set_parameter(data, kMaxLengthId, max_length / 32.0);
         }},
        {{102, 118},
         [&](const int pitch) {
             // Beat Numerator
             beat_numerator = pitch - 101;
             set_parameter(data, kBeatNumeratorId, beat_numerator / 16.0);
         }},
        {{119, 125},
         [&](const int pitch) {
             // Beat Denominator
             beat_denominator = pitch - 118;
             set_parameter(data, kBeatDenominatorId, beat_denominator / 16.0);
         }},
        {{126, 126},
         [&](const int pitch) {
             // Beat Sync On/Off
             sync = !sync;
             set_parameter(data, kSyncId, sync);
         }},
        {{127, 127},
         [&](const int pitch) {
             // Combine Notes On/Off
             combine_notes = !combine_notes;
             set_parameter(data, kCombineNotesId, combine_notes);
         }},
    };

    for (int input = 0; input < input_count; ++input) {
        Event event;
        data.inputEvents->getEvent(input, event);

        if (event.type == Event::kNoteOnEvent && event.noteOn.channel == 0) {
            NoteOnEvent note_on = event.noteOn;
            for (const auto& processor : input_processors)
                if (note_on.pitch >= processor.first.first && note_on.pitch <= processor.first.second)
                    processor.second(note_on.pitch);
        }
    }
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
                            beat_numerator = 15 * value + 1;
                            set_beat();
                        }
                        break;
                    case kBeatDenominatorId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            beat_denominator = 15 * value + 1;
                            set_beat();
                        }
                        break;
                    case kCombineNotesId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue)
                            combine_notes = static_cast<int>(value) == 1;
                        break;
                    case kKeyId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            key = (value * njones::audio::KEY.size()) - 1;
                            set_rotation();
                        }
                        break;
                    case kOctaveId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            octave = value * 20 - 10;
                            converter.set_transpose(octave * 12 + transpose);
                        }
                        break;
                    case kTransposeId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            transpose = value * (njones::audio::KEY.size() - 1);
                            converter.set_transpose(octave * 12 + transpose);
                        }
                        break;
                    case kPitchSetId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            pitch_set_index = value * (njones::audio::PITCH_SET.size() - 1);
                            pitch_set = njones::audio::PITCH_SET.at(pitch_set_index).second;
                            set_rotation();
                        }
                        break;
                    case kNoteMinId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            note_min = 127 * value;
                            converter.set_pitch_range(std::array<unsigned int, 2>{static_cast<unsigned int>(note_min),
                                                                                  static_cast<unsigned int>(note_max)});
                        }
                        break;
                    case kNoteMaxId:
                        if (paramQueue->getPoint(numPoints - 1, sampleOffset, value) == kResultTrue) {
                            note_max = 127 * value;
                            converter.set_pitch_range(std::array<unsigned int, 2>{static_cast<unsigned int>(note_min),
                                                                                  static_cast<unsigned int>(note_max)});
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

tresult PLUGIN_API Eris::initialize(FUnknown* context) {
    tresult result = SingleComponentEffect::initialize(context);
    if (result != kResultOk)
        return result;

    addAudioInput(STR16("Stereo In"), SpeakerArr::kStereo);
    addAudioOutput(STR16("Stereo Out"), SpeakerArr::kStereo);
    addEventInput(STR16("Event In"), 1);
    addEventOutput(STR16("Event Out"), 2);

    auto add_range_param = [&](const char* name, const int tag, const double min, const double max,
                               const double default_setting, const int steps) {
        String name_str = name;
        auto* new_param = new RangeParameter(name_str, tag, nullptr, min, max, default_setting, steps);
        parameters.addParameter(new_param);
    };

    auto add_list_param = [&](const char* name, const int tag, const auto& arr) {
        String name_str = name;
        auto* new_param = new StringListParameter(name_str, tag);
        for (const auto& name : arr) {
            String128 str;
            str8ToStr16(str, name.first.c_str(), static_cast<int32>(name.first.length()));
            new_param->appendString(str);
        }
        parameters.addParameter(new_param);
    };

    add_range_param("Window", kTimeWindowId, 5.0, 1000.0, 500.0, 995);
    add_range_param("Threshold", kThresholdId, 0.0, 127.0, 0.0, 127);
    add_range_param("Normalize", kCeilingId, 1.0, 127.0, 30.0, 126);
    add_range_param("Count", kNoteCountId, 0.0, 127.0, 5.0, 127);
    add_range_param("Min", kNoteMinId, 0.0, 127.0, 0.0, 127);
    add_range_param("Max", kNoteMaxId, 0.0, 127.0, 127.0, 127);
    add_range_param("Beat Sync", kSyncId, 0.0, 1.0, 1.0, 1);
    add_range_param("Multiplier", kBeatNumeratorId, 1.0, 16.0, 2.0, 15);
    add_range_param("Divider", kBeatDenominatorId, 1.0, 16.0, 5.0, 15);
    add_range_param("Combine", kCombineNotesId, 0.0, 1.0, 1.0, 1);
    add_range_param("Length", kMaxLengthId, 0.0, 32, 3.0, 32);
    add_list_param("Key", kKeyId, njones::audio::KEY);
    add_range_param("Transpose", kTransposeId, 0.0, 11.0, 0.0, 11);
    add_range_param("Octave", kOctaveId, -10.0, 10.0, 0.0, 20);
    add_list_param("Scale", kPitchSetId, njones::audio::PITCH_SET);

    return result;
}

tresult PLUGIN_API Eris::setState(IBStream* state) {
    IBStreamer streamer(state, kLittleEndian);
    auto read_param = [&](
                          auto& var, std::function<void(void)> post_process = []() {}) {
        auto saved_var = var;
        if (streamer.readInt32(saved_var) == false)
            throw std::runtime_error("Failed to read parameter.");
        var = saved_var;
        post_process();
    };
    auto int32_reader = [&](int32& var) -> bool { return streamer.readInt32(var); };

    try {
        read_param(time_window_param, [&]() { setParamNormalized(kTimeWindowId, time_window_param / 1000.0); });
        read_param(note_count, [&]() {
            converter.set_note_count(note_count);
            setParamNormalized(kNoteCountId, note_count / 127.0);
        });
        read_param(sync, [&]() {
            if (sync)
                set_beat();
            else
                set_time_window(static_cast<double>(time_window_param));
            setParamNormalized(kSyncId, sync);
        });
        read_param(beat_numerator, [&]() { setParamNormalized(kBeatNumeratorId, (beat_numerator - 1) / 15.0); });
        read_param(beat_denominator, [&]() { setParamNormalized(kBeatDenominatorId, (beat_denominator - 1) / 15.0); });
        read_param(combine_notes, [&]() { setParamNormalized(kCombineNotesId, combine_notes); });
        read_param(threshold, [&]() {
            converter.set_activation_level(threshold / 127.0);
            setParamNormalized(kThresholdId, threshold / 127.0);
        });
        read_param(key, [&]() {
            set_rotation();
            setParamNormalized(kKeyId, static_cast<double>(key) / njones::audio::KEY.size());
        });
        read_param(octave, [&]() {
            converter.set_transpose(octave * 12 + transpose);
            setParamNormalized(kOctaveId, (octave + 10) / 20.0);
        });
        read_param(ceiling, [&]() {
            converter.set_ceiling(ceiling / 127.0);
            setParamNormalized(kCeilingId, ceiling / 127.0);
        });
        read_param(pitch_set_index, [&]() {
            pitch_set = njones::audio::PITCH_SET.at(pitch_set_index).second;
            set_rotation();
            setParamNormalized(kPitchSetId, static_cast<double>(pitch_set_index) / njones::audio::PITCH_SET.size());
        });
        read_param(note_min, [&]() {
            converter.set_pitch_range(
                std::array<unsigned int, 2>{static_cast<unsigned int>(note_min), static_cast<unsigned int>(note_max)});
            setParamNormalized(kNoteMinId, note_min / 127.0);
        });
        read_param(note_max, [&]() {
            converter.set_pitch_range(
                std::array<unsigned int, 2>{static_cast<unsigned int>(note_min), static_cast<unsigned int>(note_max)});
            setParamNormalized(kNoteMaxId, note_max / 127.0);
        });
        read_param(max_length, [&]() { setParamNormalized(kMaxLengthId, max_length / 32.0); });
        read_param(transpose, [&]() {
            converter.set_transpose(octave * 12 + transpose);
            setParamNormalized(kTransposeId, transpose / 11.0);
        });
    } catch (const std::runtime_error&) {
        return kResultFalse;
    }

    return kResultOk;
}

tresult PLUGIN_API Eris::getState(IBStream* state) {
    IBStreamer streamer(state, kLittleEndian);

    streamer.writeInt32(time_window_param);
    streamer.writeInt32(note_count);
    streamer.writeInt32(sync);
    streamer.writeInt32(beat_numerator);
    streamer.writeInt32(beat_denominator);
    streamer.writeInt32(combine_notes);
    streamer.writeInt32(threshold);
    streamer.writeInt32(key);
    streamer.writeInt32(octave);
    streamer.writeInt32(ceiling);
    streamer.writeInt32(pitch_set_index);
    streamer.writeInt32(note_min);
    streamer.writeInt32(note_max);
    streamer.writeInt32(max_length);
    streamer.writeInt32(transpose);

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
    kEditorWidth = 535,
    kEditorHeight = 320
};

}  // namespace Vst
}  // namespace Steinberg

BEGIN_FACTORY_DEF("Neil F. Jones", "https://github.com/NFJones", "mailto:neil.jones.music@gmail.com")

DEF_CLASS2(INLINE_UID(0x2D7B2972, 0x3F4547FA, 0xA996468E, 0xE2668886),
           PClassInfo::kManyInstances,              // cardinality
           kVstAudioEffectClass,                    // the component category (do not changed this)
           "Eris",                                  // here the plug-in name (to be changed)
           0,                                       // single component effects cannot be distributed so this is zero
           Steinberg::Vst::PlugType::kFxGenerator,  // Subcategory for this plug-in (to be changed)
           FULL_VERSION_STR,                        // Plug-in version (to be changed)
           kVstVersionString,                     // the VST 3 SDK version (do not changed this, use always this define)
           Steinberg::Vst::Eris::createInstance)  // function pointer called when this component should be instantiated

END_FACTORY
