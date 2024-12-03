#ifndef BOARD_H
#define BOARD_H

#include <avr/io.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define RESET  12
#define SLEEP  11
#define STEP   10
#define DIR     9
#define PROBE   8

#define RESET_MASK   0x10
#define SLEEP_MASK   0x08
#define STEP_MASK    0x04
#define DIR_MASK     0x02
#define PROBE_MASK   0x01

#define POT_PIN   A4
#define BUTTON_IN A2
#define BUTTON_1 1014
#define BUTTON_2 1004
#define BTN_TOL 4

#endif
