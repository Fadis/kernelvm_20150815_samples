#ifndef LPC1114_GLOBALS_HPP
#define LPC1114_GLOBALS_HPP

#include "synth.hpp"
#include "receive_buffer.hpp"

extern receive_buffer rbuf;
extern command_state< YM2413 > synth;

#endif
