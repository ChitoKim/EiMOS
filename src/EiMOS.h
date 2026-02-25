#ifndef EIMOS_H
#define EIMOS_H

// EiMOS Library - EiMOS.h
// Formerly known as mahjongAsst
// Renamed 6 June, 2024

// Two interface classes; EiMOS_MEAS for measurements and EIMOS_DP for displays

#define NORMAL 1
#define DIFF 0
#define PM 2
#define SUM 1000
typedef struct Results
{
  int score[4];
  int error[4]; // checks if the sticks are in the right places; if error 1, if not 0
  int mode[4];  // display mode of the scores : NORMAL, DIFF(score difference), PM(+/-)
  int honba;
  unsigned int totalScore;
  int emptySeat;
} Results;
class EiMOS_MEAS
{
 public:
  EiMOS_MEAS()
  {
  }
  virtual void measure();
};
class EiMOS_DP
{
 public:
  EiMOS_DP()
  {
  }
  virtual void show(Results *results);
  virtual void show(const char *str);
};
#endif