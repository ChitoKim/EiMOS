#include <mahjongAsst.h>

int charge_pin[] =
{
  //please note: avoid using RX, TX pin as charge pins 
  7, 6, 5,
  4, 3, 2,
  14, 15, 16,
  17, 18, 19
};
int analog_pin[] = 
{
  A0, A1, A2,
  A4, A5, A6,
  A8, A9, A10,
  A12, A13, A14
};
int button_mode[] =
{
  A3, A7, A11, A15
};
int button_honba = 19;
float CAP_CENTURY_GOLD[] =
{
  //in kiloohms
  5.0f, 10.0f, 1.0f, 1.0f
};
float CAP_CENTURY_SILVER[] =
{
  //in nanoFarads
  225.0f, 10.0f, 1.0f, 1.0f
};
float R_REF[] =
{
  //internal pullup resistor; in kiloohms
  34.8f, 34.8f, 34.8f, 34.8f 
};
float CENTURY_GOLD_R_PAR[] =
{
  1000.0f, PIN_NONE, PIN_NONE, PIN_NONE
};

mahjongAsst Asst(charge_pin, analog_pin, CAP_CENTURY_GOLD, R_REF); //CENTURY TENPAL, GOLD sticks
//mahjongAsst Asst(charge_pin, analog_pin, CAP_CENTURY_SILVER, R_REF); //CENTURY TENPAL, SILVER sticks

void
setup()
{
  Asst.setNSlot(3);
  Asst.setPullType(INPUT_PULLUP);
  Asst.setADCResolution(10);
  Asst.setMesType(CAP);
  Asst.setParRes(CENTURY_GOLD_R_PAR); // if silver sticks, comment this
  Asst.setModeButton(button_mode);
  Asst.setHonbaButton(button_honba);
  // Asst.setOffset(200);
  Asst.begin();
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
