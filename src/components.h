/*
components.h
A library that handles the parameters of mahjong scoring,
into three different structs : ENV, PIN, VAL

Copyright (c) 2023, Chito Kim
All rights reserved.
*/
#include "ADS1X15.h"
#ifndef _COMPONENTS_H
#define _COMPONENTS_H
typedef struct ENV
{
  // handles overall environment: number of slots, type of electrical components, etc
  int NSLOT;        // number of slots; 5k 1k .1k or 5k 1k .1k .1k
  int NUMPIN;       // total number of slots; 12 or 16
  int pull_type;    // PULLUP, PULLDOWN, or INPUT_PULLUP
  int mes_type;     // RES, or CAP
  uint16_t ADC_MAX; // MAX adc value; for avr-arduinos, 1024
} ENV;
typedef struct PIN
{
  // handles gpio pins and measurement parameters
  ADS1X15 *ext_adc[4];
  int extADCMode;
  int charge_pin[16];  // pins determine to charge/discharge capacitors
  int analog_pin[16];  // analog pins measuring voltages
  int button_mode[4];  // button pin controlling the display modes(INPUT_PULLUP)
  int button_honba[4]; // button pin controlling the honbas(INPUT_PULLUP)
  int button_seat;
  float RLC_per_unit[4]; // resistor/capacitor value per stick
#if REF_CORRECTION_DIMENTION == 1
  float R_REF[4]; // reference resistor used to divide voltages (1 dimention)
#elif REF_CORRECTION_DIMENTION == 2
  float R_REF[4][4]; // reference resistor used to divide voltages (2 dimention)
#endif
  float R_PAR[4]; // resistance parallel to the capacitor; only for specific types of models
                  // such as GOLD-stick CENTURY TENPAL
#if REF_CORRECTION_DIMENTION == 1
  float weight[4]; // only for resistors. add weight to the ratio calculated (1 dimention)
#elif REF_CORRECTION_DIMENTION == 2
  float weight[4][4]; // only for resistors. add weight to the ratio calculated (2 dimention)
#endif
} PIN;
typedef struct VAL
{
  // handles actual values(mainly score)
  /**
   * @brief A 2D array storing information related to each stick slot.
   *
   * The first dimension [16] represents the number of each stick slot.
   * The second dimension [3] contains the following information:
   * - [0]: The currently used number.
   * - [1]: A candidate value for the next number to be used.
   * - [2]: A count of how many times the value at index 1 has appeared consecutively.
   *        When this count exceeds debounce_count, the value at index 0 is updated to the value at index 1.
   */
  int prev_num[16][3];
  int score[4];
  int error[4]; // checks if the sticks are in the right places; if error 1, if not 0
  int mode[4];  // display mode of the scores : NORMAL, DIFF(score difference), PM(+/-)
  bool prev_button_honba[4];
  int honba;
  int bust_offset; // dealing with the 10k busting sticks; subtracts the offset from players' scores
  unsigned long lastTime;
  unsigned int debounce_count;
  unsigned int totalScore;
  int emptySeat;
  bool prev_button_seat;
} VAL;
#endif
