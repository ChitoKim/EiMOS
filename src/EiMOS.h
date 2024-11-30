/*
Special Thanks to Mahjong

EiMOS Library - EiMOS.h
Formerly known as mahjongAsst
Renamed 6 June, 2024

A Library for legacy scoring system of Japanese mahjong tables.
Legacy mahjong scorers implement special score sticks containing electrical elements such as R, L, or C.
This library measures parallel resistances/capacitances of stack-piled score sticks,
and convert the values into actual scores of 4 mahjong players.

Copyright (c) 2023, Chito Kim
All rights reserved.

Codes for capacitance measurement is based on an example code of Jonathan Nethercott.
https://wordpress.codewrite.co.uk/pic/2014/01/25/capacitance-meter-mk-ii/
*/

#ifndef _MAHJONGASST_H
#define _MAHJONGASST_H

#if defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_MBED_RP2040) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_STM32)
#define ADC_RESOLUTION_MUTABLE
#endif

#define REF_CORRECTION_DIMENTION 2

#include "ADS1X15.h"
#include "MUX.h"
#include "components.h"

#define DEFAULT_ADC_MAX 1024
#define DEFAULT_NSLOT 4
#define DEFAULT_NUMPIN 16
#define DEFAULT_HONBA 0
#define MAXSTICK 50
#define MAXSTICK_10000P 16
#define MAXSTICK_1000P 16
#define MAXSTICK_500P 4
#define MAXSTICK_100P 20
#define MAXSTICK_100P_3SLOT 40

#define RES 0
#define CAP 1
#define PIN_NONE -1

#ifndef PULLUP
#define PULLUP 5
#endif

#ifndef PULLDOWN
#define PULLDOWN -1
#endif
// INPUT_PULLUP

#define MAXTIME sizeof(unsigned long)

#define NORMAL 1
#define DIFF 0
#define PM 2
#define SUM 1000

class EiMOS
{
 private:
  MUX *mux_p;
  ENV *env_p;
  PIN *pin_p;
  VAL *val_p;

 public:
  EiMOS(MUX *mux, ENV *env, PIN *pin, VAL *val);
  EiMOS(int charge[], int analog[], float v_unit[], float ref[]);
  EiMOS(int charge[], int analog, float v_unit[], float ref[]);
  EiMOS(int analog, float v_unit[], float ref[]);
  EiMOS(int analog[], float v_unit[], float ref[]);
  EiMOS(int charge[], ADS1X15 *ext_adc[], float v_unit[], float ref[]);
  EiMOS(ADS1X15 *ext_adc[], float v_unit[], float ref[]);
  EiMOS(ADS1X15 *ext_adc[], float v_unit[], float ref[][4]);
  MUX *getMUX();
  ENV *getENV();
  PIN *getPIN();
  VAL *getVAL();
  void setMUX4051(int a[], int b[]);
  void setMUX4067(int a[]);
  void initMUX();
  void initExtADC();
  void slotSelect(int slot_num);
  void setNSlot(int a);
  void setPullType(int a);
  void setMesType(int a);
  void setModeButton(int a[]);
  void setHonbaButton(int a[]);
  void setDebounceCount(unsigned int count);
  void setSeatButton(int btn);
  void setTotalScore(unsigned int score);
  void setADCResolution(int a);
  void setExtADC(int gain, int bit, float vcc, int mode = 0);
  void setWeight(float a[]);
  void setWeight(float weight[][4]);
  void setOffset(int a);

  void getScore(int scr[]);
  int *getScore();
  int getTotalScore();
  void getError(int err[]);
  int *getError();
  void getMode(int mode[]);
  int *getMode();
  int getHonba();

  int getEmptySeat();

  void incrementEmptySeat();

  int boolRead(int pin);
  uint16_t adcRead(int pin);
  uint16_t extADCRead(int slot_num);
  void pullAnalog(int apin);

  void mesLoop(float RLC[]);
  void numLoop(float RLC[], int num[]);
  void scoreLoop(int num[]);
  void buttonLoop();
  void loop();
  void loop(int period_ms);
  void loop(float RLC[], int num[]);

  void begin();
  void prepMes(int slot_num);
  float mesRLC(int slot_num);
  int RLCToNum(float RLC, int slot_num);

  // resistance specific////
  ////
  float adcToRes(uint16_t adc, float r);
  // capacitance specific///
  ////
  void setParRes(float f[]);
  int hasParRes(float f);
  float adcToCap(unsigned long t, uint16_t adc, float r, float c_unit, float r_par);
  void discharge(int cpin, int apin);
  void charge(int cpin);
  //
};
void _HONBA();
#endif
