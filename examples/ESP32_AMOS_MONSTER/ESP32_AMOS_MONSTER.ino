#include <mahjongAsst.h>
int button_mode[] =
{
  36, 39, 34, 35
  // 36, 39, 34, 35 are input only pins without internal pullup.
  // so, add external pullup on these
  // if you don't want to, use other pins
};
int button_honba = 19;
float RES_AMOS_MONSTER[] =
{
  //in kiloohms
  20.0f, 100.0f, 1000.0f ,1000.0f
};
float R_REF[] =
{
  //in kiloohms
  10.0f, 10.0f, 10.0f, 10.0f
};

ADS1115 ADC0(0x48, &Wire);
ADS1115 ADC1(0x49, &Wire);
ADS1115 ADC2(0x4A, &Wire);
ADS1115 ADC3(0x4B, &Wire);

ADS1X15 *adc[] = {&ADC0, &ADC1, &ADC2, &ADC3};
mahjongAsst Asst(adc, RES_AMOS_MONSTER, R_REF);

float weight[] = {0.3f, 0.3f, 0.3f, 0.3f};
enum I2CPIN {
  _SDA0 = 19,
  _SCL0 = 21,
  _SDA1 = 22,
  _SCL1 = 23
};


void
setup()
{
  Asst.setNSlot(4);          //set number of slots; 5k 1k 100 (NSLOT = 3), or 5k 1k 100 100 (NSLOT = 4)

  Asst.setMesType(RES);     //choose measure type; resistance(RES) or CAP(capacitance)
  Asst.setPullType(PULLUP); //choose whether to pull up or down the reference resistors
                              //one of these: PULLUP, PULLDOWN, INPUT_PULLUP
                              
  // Asst.setOffset(200);    // uncomment to enable busting sticks
  Asst.setWeight(weight);
  // Asst.setModeButton(button_mode); // uncomment to enable mode buttons
  // Asst.setHonbaButton(button_honba); // uncomment to enable a honba button

  Wire.begin(_SDA0, _SCL0);
  Wire1.begin(_SDA1, _SCL1); //tweak pins_arduino.h for second I2C

  Asst.initExtADC();
  Asst.setExtADC(/*setGain*/ 1, /*ADC Resolution*/16, /*VCC*/3.3f);
  Asst.begin();
}
void
loop()
{
  int score[4] = {0};
  int error[4] = {0};
  Asst.loop(500);
  Asst.getScore(score);
  Asst.getError(error);
  // Now you have scores and errors.
  // Display them anyway you like
}