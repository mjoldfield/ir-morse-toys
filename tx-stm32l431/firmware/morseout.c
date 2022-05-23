/*
  This file was produced by the morse-tool
  from morse.cfg

  DO NOT EDIT IT MANUALLY
*/

#include <stdint.h>


#include "morseout.h"

static const uint8_t morse_time_data[];
static const uint8_t * const morse_messages[];

uint32_t morse_get_next_time(uint32_t n_msg, uint32_t * const msg_i)
{
  if (n_msg >= n_morse_messages)
    return 0;

  uint8_t tau = morse_messages[n_msg][*msg_i];
  *msg_i = (tau == 0) ? 0 : *msg_i + 1;

  return tau;
}

// number of messages
const uint32_t n_morse_messages = 2;

// main array of times, 296 entries
static const uint8_t morse_time_data[] = { 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 7, 1, 1, 1, 1, 1, 1, 1, 3, 1, 3, 1, 1, 3, 1, 1, 1, 1, 3, 1, 1, 3, 1, 1, 1, 1, 3, 3, 1, 3, 1, 3, 7, 1, 1, 3, 1, 3, 3, 3, 1, 3, 1, 3, 3, 1, 1, 3, 1, 1, 3, 1, 1, 3, 1, 1, 1, 1, 3, 3, 1, 1, 1, 1, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 3, 7, 3, 3, 1, 1, 1, 1, 1, 1, 1, 3, 1, 7, 3, 1, 3, 1, 1, 1, 3, 3, 1, 1, 1, 1, 3, 3, 1, 1, 1, 3, 3, 1, 1, 1, 3, 1, 1, 3, 3, 1, 1, 1, 3, 7, 3, 1, 1, 1, 1, 1, 1, 3, 1, 1, 3, 1, 1, 3, 3, 1, 3, 1, 3, 3, 1, 1, 3, 1, 3, 3, 3, 1, 1, 7, 1, 1, 1, 1, 3, 1, 1, 3, 3, 1, 3, 1, 3, 3, 3, 1, 1, 1, 1, 1, 3, 7, 1, 1, 3, 1, 3, 1, 3, 3, 1, 1, 1, 1, 3, 3, 3, 1, 3, 3, 1, 1, 3, 1, 3, 1, 1, 3, 1, 1, 1, 1, 1, 7, 3, 1, 3, 1, 3, 3, 1, 1, 1, 1, 1, 1, 3, 3, 1, 3, 1, 1, 3, 1, 1, 7, 3, 3, 1, 1, 1, 1, 1, 1, 1, 3, 1, 7, 1, 1, 3, 1, 1, 1, 1, 3, 1, 1, 3, 3, 3, 1, 3, 1, 1, 1, 1, 3, 3, 1, 1, 1, 3, 1, 3, 7, 3, 1, 1, 1, 1, 3, 3, 1, 3, 1, 3, 3, 3, 1, 3, 1, 1, 0 };

// pointers into the array above corresponding to messages
static const uint8_t * const morse_messages[] =
  { morse_time_data +    0
  // - Hello World
  // -....- / .... . .-.. .-.. --- / .-- --- .-. .-.. -..
  // 133 dits : 68 (51.1%) marks, 65 (48.9%) spaces
  
  , morse_time_data +   76
  // - The quick brown fox jumps over the lazy dog
  // -....- / - .... . / --.- ..- .. -.-. -.- / -... .-. --- .-- -. / ..-. --- -..- / .--- ..- -- .--. ... / --- ...- . .-. / - .... . / .-.. .- --.. -.-- / -.. --- --.
  // 429 dits : 214 (49.9%) marks, 215 (50.1%) spaces
  
  };
