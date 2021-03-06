#include "synth.hpp"

const int fnum[12] = {172,181,192,204,216,229,242,257,272,288,305,323};

const unsigned char programs[128] = {
  0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0xB, 0xB, //Piano
  0xC, 0xC, 0xC, 0xC, 0xC, 0xC, 0xC, 0xC, // Chromatic Percussion
  0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, 0x8, // Organ
  0x2, 0x2, 0x2, 0x2, 0x2, 0xF, 0xF, 0x2, // Guitar
  0xE, 0xD, 0xD, 0xE, 0xD, 0xD, 0xD, 0xD, // Bass
  0x1, 0x1, 0x1, 0x1, 0x1, 0xB, 0xB, 0x2, // Strings
  0x1, 0x1, 0xA, 0xA, 0xA, 0xA, 0xA, 0x9, // Ensemble
  0x7, 0x7, 0x9, 0x7, 0x9, 0x7, 0xA, 0xA, // Brass
  0x7, 0x7, 0x7, 0x7, 0x6, 0x9, 0x5, 0x5, // Reed
  0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, // Pipe
  0xA, 0xA, 0xA, 0xA, 0xA, 0xA, 0xA, 0xA, // Synth Lead
  0xA, 0xA, 0xA, 0xA, 0xA, 0xA, 0xA, 0xA, // Synth Pad
  0xA, 0xA, 0xA, 0xA, 0xA, 0xA, 0xA, 0xA, // Synth Effect
  0x2, 0x2, 0x2, 0x2, 0x2, 0x5, 0x1, 0x5, // Ethnic
  0xC, 0xC, 0xC, 0xC, 0x2, 0x2, 0x2, 0xA, // Percussive
  0xA, 0x4, 0xF, 0x4, 0xC, 0xF, 0x2, 0xF // Sound effects
};

const unsigned char parcussion[ 128 ] = {
  0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5,
  0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5,
  0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5,
  0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5,
  0x5, 0x5, 0x5, 0x0, 0x0, 0x5, 0x1, 0x5,
  0x1, 0x2, 0x5, 0x2, 0x4, 0x2, 0x4, 0x2,
  0x2, 0x3, 0x2, 0x3, 0x3, 0x5, 0x1, 0x3,
  0x5, 0x3, 0x5, 0x3, 0x0, 0x0, 0x2, 0x2,
  0x2, 0x5, 0x5, 0x5, 0x5, 0x5, 0x3, 0x5,
  0x5, 0x4, 0x4, 0x2, 0x2, 0x2, 0x5, 0x5,
  0x4, 0x4, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5,
  0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5,
  0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5,
  0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5,
  0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5,
  0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5, 0x5
};

