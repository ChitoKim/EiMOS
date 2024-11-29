/*
MUX.h
A library that controlls the switching of an MUX.

Copyright (c) 2023, Chito Kim
All rights reserved.
*/

#ifndef _MUX_H
#define _MUX_H
class MUX
{
 private:
  int *address;
  int address_size;
  int *enable;
  int enable_size;

 public:
  MUX(int addr[], int addr_size, int enb[], int enb_size);
  void setMUX4051(int a[], int b[]);
  void setMUX4067(int a[]);
  void initMUX();
  void slotSelect(int slot_num);
  void enbSelect(int a);
  void writeAddress(int n, int state);
  void writeEnable(int n, int state);
};
#endif
