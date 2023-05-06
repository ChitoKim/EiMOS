#include <mahjongAsst.h>
int address_pin[] = 
{
  16, 17, 5, 18
};
int analog_pin[] = 
{
  32, 32, 32, 32,
  32, 32, 32, 32,
  32, 32, 32, 32,
  32, 32, 32, 32
};
int button_mode[] =
{
  36, 39, 34, 35
};
int button_honba = 32;
float RES_AMOS_MONSTER[] =
{
  // in kiloohms
  20.0f, 100.0f, 1000.0f ,1000.0f
};
float R_REF[] =
{
  // in kiloohms
  2.0f, 22.0f, 22.0f, 22.0f 
};
mahjongAsst Asst(analog_pin, RES_AMOS_MONSTER, R_REF);
float weight[] = {0.3f, 0.6f, 0.3f, 0.3f};

void
setup()
{
  Asst.begin();
  Asst.setNSlot(4);          //set number of slots; 5k 1k 100 (NSLOT = 3), or 5k 1k 100 100 (NSLOT = 4)
  Asst.setADCResolution(12); //set ADC Resolution; e.g. if arduino uno, set it to 10

  Asst.setMesType(RES);     //choose measure type; resistance(RES) or CAP(capacitance)
  Asst.setPullType(PULLDOWN); //choose whether to pull up or down the reference resistors
                              //one of these: PULLUP, PULLDOWN, INPUT_PULLUP
  // Asst.setOffset(200);
  Asst.setWeight(weight);
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
