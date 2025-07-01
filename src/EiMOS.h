#ifndef EIMOS_H
#define EIMOS_H

// EiMOS Library - EiMOS.h
// Formerly known as mahjongAsst
// Renamed 6 June, 2024

//Two interface classes; EiMOS_MEAS for measurements and EIMOS_DP for displays
class EiMOS_MEAS
{
  public:
  EiMOS_MEAS() {}
  virtual void measure();
};
class EiMOS_DP
{
  public:
  EiMOS_DP() {}
  virtual void show();
};
#endif