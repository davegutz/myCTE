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


void time_long_2_str(const unsigned long long time, SafeString &tempStr);


// Datum struct
struct Datum_st
{
  unsigned long long t_raw = 1ULL;
  int16_t T_rot_raw_int = 0;
  int16_t a_raw_int = 0;
  int16_t b_raw_int = 0;
  int16_t c_raw_int = 0;
  int16_t T_acc_raw_int = 0;
  int16_t x_raw_int = 0;
  int16_t y_raw_int = 0;
  int16_t z_raw_int = 0;

  void put(const time_t event, Sensors *Sen);
  void copy_to_datum_ram_from(Datum_st input);
  void get() {};
  void nominal();
  void print();
  void put(Datum_st source);
  void put_nominal();
  void put_sparse(Sensors *Sen);
};

class Data_st
{
public:
  Data_st() : i_(0), n_(0) {};
  Data_st(uint16_t size) : i_(size-1), n_(size)
  {
    data = new Datum_st*[n_];
    for (int j=0; j<n_; j++) data[j] = new Datum_st();
  };
  ~Data_st();
  void copy_to_data_ram_from(Datum_st input);
  void get();
  void print_all();
  void put_datum(Sensors *Sen);
  void reset(const boolean reset);

protected:
  Datum_st **data;
  uint8_t i_;
  uint8_t n_;
};

#endif
