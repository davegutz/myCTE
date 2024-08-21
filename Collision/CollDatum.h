// MIT License
//
// Copyright (C) 2024 - Dave Gutz
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifndef _COLLDATUM_H
#define _COLLDATUM_H

#include <SafeString.h>
#include "constants.h"
#include "Sensors.h"
#include "TimeLib.h"

#ifdef USE_ARDUINO
  #include <Arduino.h> //needed for Serial.println
  #include <Arduino_LSM6DS3.h>
#else
  #error "Only Arduino nano 33 iot has built in IMU"
  #include "application.h"  // Particle
#endif

extern time_t time_initial;


void time_long_2_str(const unsigned long long time_ms, SafeString &tempStr);


// Datum struct
struct Datum_st
{
  unsigned long long t_raw_ms = 1ULL;
  int16_t T_rot_raw_int = 0;
  int16_t a_raw_int = 0;
  int16_t b_raw_int = 0;
  int16_t c_raw_int = 0;
  int16_t T_acc_raw_int = 0;
  int16_t x_raw_int = 0;
  int16_t y_raw_int = 0;
  int16_t z_raw_int = 0;

  void get() {};
  void nominal();
  void print();
  void from(Datum_st input);
  void put_nominal();
  void from(Sensors *Sen);
};



// Index to current data
struct Register_st
{
public:
  uint16_t i = 0;
  uint16_t n = 0;
  unsigned long long t_ms = 0ULL;
  boolean locked = false;

  boolean is_empty() { if (t_ms) return(false); else return(true); };
  void print()
  {
    Serial.print("Reg: t_ms:"); Serial.print(t_ms);
    Serial.print(" i:"); Serial.print(i);
    Serial.print(" n: "); Serial.println(n);
  }
  void put_nominal() { i = 0; n = 0; t_ms = 0ULL; locked = false; };
};


// Ram manager
class Data_st
{
public:
  Data_st() : iR_(0), nR_(0), iP_(0), nP_(0), iRr_(0), nRr_(0) {};
  Data_st(uint16_t ram_datums, uint16_t pre_datums, uint16_t reg_registers) :
   iR_(ram_datums-1), nR_(ram_datums),
   iP_(pre_datums-1), nP_(pre_datums),
   iRr_(reg_registers-1), nRr_(reg_registers)
  {
    int j;
    Precursor = new Datum_st*[nP_];
    for (j=0; j<nP_; j++) Precursor[j] = new Datum_st();
    Ram = new Datum_st*[nR_];
    for (j=0; j<nR_; j++) Ram[j] = new Datum_st();
    Reg = new Register_st*[nRr_];
    for (j=0; j<nRr_; j++) Reg[j] = new Register_st();
  };
  ~Data_st();
  void get();
  void move_precursor();
  void print_latest_ram();
  void print_all_registers();
  void print_latest_register();
  void print_ram();
  void put_precursor(Sensors *Sen);
  // void from(Datum_st input);
  void put_ram(Sensors *Sen);
  void put_ram(Datum_st *point);
  void register_lock();
  void register_unlock();
  void reset(const boolean reset);

protected:
  Datum_st **Precursor; // Precursor storage
  Datum_st **Ram;       // Ram storage
  Register_st **Reg;    // Register for Ram
  uint16_t iR_, iP_, iRr_;
  uint16_t nR_, nP_, nRr_;
};


#endif
