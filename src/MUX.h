/*  
  MUX.h
  A library that controlls the switching of an MUX.

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

#ifndef _MUX_H
#define _MUX_H
#endif
class MUX
{
private:
  int *address;
  int address_size;
  int *enable;
  int enable_size;
  int unscramble_4051;
public:
  MUX(int addr[], int addr_size, int enb[], int enb_size, int unscramble = 0);
  void setMUX4051(int a[], int b[], int unscramble = 0);
  void setMUX4067(int a[]);
  void initMUX();
  int  unScramble4051(int slot_num);
  void slotSelect(int slot_num);
  void enbSelect(int a);
  void writeAddress(int n, int state);
  void writeEnable(int n, int state);
};