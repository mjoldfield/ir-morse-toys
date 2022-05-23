/*
  This file was produced by the morse-tool
  from morse.cfg

  DO NOT EDIT IT MANUALLY
*/

#include <stdint.h>


#ifndef _MORSEOUT_H_
#define _MORSEOUT_H_

// the number of messages we have defined
extern const uint32_t n_morse_messages;

// Return the next time (in dits) for message n_msg.
//
//   - silly values of n_msg return 0.
//
//   - changing n_msg between calls will break things, so
//     don't do it.
//
//   - *msg_i should be set to 0 before the first call, then
//     left alone. It isn't safe to try arbitrary values! This
//     call will increment the value, then reset it, so you
//     can just loop forever over it.
//
//   - *msg_i will be set to 0 at the end of the message.
//
//  BUG: it would be better to make all values of msg_n and msg_i safe.
//
extern uint32_t morse_get_next_time(uint32_t n_msg, uint32_t * const msg_i);

#endif
