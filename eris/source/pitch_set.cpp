#include "pitch_set.h"

const std::vector<std::pair<std::string, std::vector<unsigned int>>> njones::audio::PITCH_SET = {
    {"Chromatic", {}},
    {"Major", {0, 2, 4, 5, 7, 9, 11}},
    {"Natural Minor", {0, 2, 3, 5, 7, 8, 10}},
    {"Pentatonic Major", {0, 2, 5, 7, 9}},
    {"Pentatonic Minor", {0, 2, 5, 7, 10}},
    {"Wholetone", {0, 2, 4, 6, 8, 10}},
    {"Dorian", {0, 2, 3, 5, 7, 9, 10}},
    {"Harmonic Minor", {0, 2, 3, 5, 7, 8, 11}},
    {"Locrian", {0, 1, 3, 5, 6, 8, 10}},
    {"Lydian", {0, 2, 4, 6, 7, 9, 11}},
    {"Melodic Minor", {0, 2, 3, 5, 7, 8, 11}},
    {"Mixolydian", {0, 2, 4, 5, 7, 9, 10}},
    {"Phrygian", {0, 1, 3, 5, 7, 8, 10}},
    {"Octaves", {0}},
    {"Fifths", {0, 7}},
    {"Fourths", {0, 5}},
};

extern const std::vector<std::pair<std::string, int>> njones::audio::KEY = {
    {"C", 0},  {"C#", 1}, {"D", 2},  {"Eb", 3}, {"E", 4},   {"F", 5},
    {"F#", 6}, {"G", 7},  {"G#", 8}, {"A", 9},  {"Bb", 10}, {"B", 11},
};