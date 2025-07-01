#ifndef EIMOS_H
#define EIMOS_H

// EiMOS Library - EiMOS.h
// Formerly known as mahjongAsst
// Renamed 6 June, 2024

class EiMOS
{
  private:
  EiMOS_MEAS *meas;
  EiMOS_DP *dp;
  public:
  EiMOS(EiMOS_MEAS *meas, EiMOS_DP *dp) : meas(meas), dp(dp) { }
  EiMOS(EiMOS_MEAS meas, EiMOS_DP dp) : meas(&meas), dp(&dp) { }
  EiMOS_MEAS *getMeas()
  {
    return this->meas;
  }
  EiMOS_DP *getDP()
  {
    return this->dp;
  }
  void measure()
  {
    meas->measure();
  }
  void show()
  {
    dp->show();
  }
  void loop()
  {
    measure();
    show();
  }
};

//Two interface classes; EiMOS_MEAS for measurements and EIMOS_DP for displays
class EiMOS_MEAS
{
  public:
  virtual void measure();
};
class EiMOS_DP
{
  public:
  virtual void show();
};
#endif