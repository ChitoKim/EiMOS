/*
  components.h
  A library that handles the parameters of mahjong scoring,
  into three different structs : ENV, PIN, VAL

 *  Copyright (c) 2023, Chito Kim
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "ADS1X15.h"
#ifndef _COMPONENTS_H
#define _COMPONENTS_H
typedef struct ENV
{
  //handles overall environment: number of slots, type of electrical components, etc
  int NSLOT;    //number of slots; 5k 1k .1k or 5k 1k .1k .1k
  int NUMPIN;   //total number of slots; 12 or 16
  int pull_type; // PULLUP, PULLDOWN, or INPUT_PULLUP
  int mes_type;  // RES, or CAP
  uint16_t ADC_MAX;   // MAX adc value; for avr-arduinos, 1024
} ENV;
typedef struct PIN
{
  //handles gpio pins and measurement parameters
  ADS1X15 *ext_adc[4];
  int extADCMode;
  int charge_pin[16];    // pins determine to charge/discharge capacitors
  int analog_pin[16];    // analog pins measuring voltages
  int button_mode[4];    // button pin controlling the display modes(INPUT_PULLUP)
  int button_honba;      // button pin controlling the honbas(INPUT_PULLUP)
  float RLC_per_unit[4]; // resistor/capacitor value per stick
  float R_REF[4]; //reference resistor used to divide voltages
  float R_PAR[4]; //resistance parallel to the capacitor; only for specific types of models
                  //such as GOLD-stick CENTURY TENPAL
  float weight[4]; //only for resistors. add weight to the ratio calculated 
} PIN;
typedef struct VAL
{
  //handles actual values(mainly score)
  int score[4];
  int error[4]; //checks if the sticks are in the right places; if error 1, if not 0
  int mode[4];  //display mode of the scores : NORMAL, DIFF(score difference), PM(+/-)
  int honba;
  int bust_offset; //dealing with the 10k busting sticks; subtracts the offset from players' scores
  unsigned long lastTime;
} VAL;
#endif