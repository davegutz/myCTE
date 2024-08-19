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


void time_long_2_str(const time_t time, SafeString &tempStr);


// SRAM retention summary
struct Datum_st
{
  time_t stamp = 0;  // ID of event
  time_t t_filt = 1UL; // Timestamp seconds since start of epoch
  // Gyroscope in radians/second
  int16_t a_filt = 0;
  int16_t b_filt = 0;
  int16_t c_filt = 0;
  int16_t o_filt = 0;
  // Acceleration in g's  
  int16_t x_filt = 0;
  int16_t y_filt = 0;
  int16_t z_filt = 0;
  int16_t g_filt = 0;

  void put(const time_t event, Sensors *Sen);
  void copy_to_datum_ram_from(Datum_st input);
  void get() {};
  void nominal();
  void print();
  void put(Datum_st source);
  void put_nominal();
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
  void get();
  void put(const boolean reset, const time_t id, Sensors *Sen);
  void put_nominal();

protected:
  Datum_st **data;
  uint16_t i_;
  uint16_t n_;
};

#endif
