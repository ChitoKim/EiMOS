#include <mahjongAsst.h>

MUX mux(nullptr, 0, nullptr, 0, 0); 
ENV env = {DEFAULT_NSLOT, DEFAULT_NUMPIN, PULLDOWN, RES, DEFAULT_ADC_MAX};
PIN pin = 
{{PIN_NONE, PIN_NONE, PIN_NONE, PIN_NONE,
  PIN_NONE, PIN_NONE, PIN_NONE, PIN_NONE,
  PIN_NONE, PIN_NONE, PIN_NONE, PIN_NONE,
  PIN_NONE, PIN_NONE, PIN_NONE, PIN_NONE},
  {PIN_NONE, PIN_NONE, PIN_NONE, PIN_NONE,
  PIN_NONE, PIN_NONE, PIN_NONE, PIN_NONE,
  PIN_NONE, PIN_NONE, PIN_NONE, PIN_NONE,
  PIN_NONE, PIN_NONE, PIN_NONE, PIN_NONE}, 
  {PIN_NONE, PIN_NONE, PIN_NONE, PIN_NONE},
  PIN_NONE,
  {PIN_NONE, PIN_NONE, PIN_NONE, PIN_NONE},
  {PIN_NONE, PIN_NONE, PIN_NONE, PIN_NONE},
  {PIN_NONE, PIN_NONE, PIN_NONE, PIN_NONE}
};
mahjongAsst Asst(&mux, &env, &pin);

const int address_pin[] =
{
};
// const int enable_pin[] =
// {
// };
const int button_mode[] =
{
};
const int button_honba = ;
void
setup()
{
  Asst.begin();
  Asst.setNSlot(4);          //set number of slots; 5k 1k 100 (NSLOT = 3), or 5k 1k 100 100 (NSLOT = 4)
  Asst.setADCResolution(12); //set ADC Resolution; e.g. if arduino uno, set it to 10

  Asst.setMesType(RES);     //choose measure type; resistance(RES) or CAP(capacitance)
  Asst.setPullType(PULLDOWN); //choose whether to pull up or down the reference resistors
                              //one of these: PULLUP, PULLDOWN, INPUT_PULLUP
  // Asst.setMUX4051(address_pin, enable_pin); //set two 74hc4051 ics to multiplex 16 analog inputs
                                               //3 address pins shared, and 2 enable pins for each mux 
  Asst.setMUX4067(address_pin);
  Asst.setModeButton(button_mode);
  Asst.setHonbaButton(button_honba);
}
void
loop()
{
  int score[4] = {0};
  int error[4] = {0};
  Asst.loop(500); // loop measurements every 500ms
  Asst.getScore(score); 
  Asst.getError(error);
  // now you have scores and errors, so display it way you like
}