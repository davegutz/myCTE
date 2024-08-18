// MIT License
//
// Copyright (C) 2023 - Dave Gutz
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
  unsigned long dummy = 0;  // padding to absorb Wire.write corruption

  void assign(const time_t now, Sensors *Sen);
  void copy_to_datum_ram_from(Datum_st input);
  void get() {};
  void nominal();
  void print();
  void put(Datum_st source);
  void put_nominal();
};

class Datum_ram : public Datum_st
{
public:
  Datum_ram();
  ~Datum_ram();

  #ifdef USE_EEPROM
    void get_t_flt()        { unsigned long value;  rP_->get(t_flt_eeram_.a16, value);        t_flt = value; };
    void get_Tb_hdwe()      { int16_t value;        rP_->get(Tb_hdwe_eeram_.a16, value);      Tb_hdwe = value; };
    void get_vb_hdwe()      { int16_t value;        rP_->get(vb_hdwe_eeram_.a16, value);      vb_hdwe = value; };
    void get_ib_amp_hdwe()  { int16_t value;        rP_->get(ib_amp_hdwe_eeram_.a16, value);  ib_amp_hdwe = value; };
    void get_ib_noa_hdwe()  { int16_t value;        rP_->get(ib_noa_hdwe_eeram_.a16, value);  ib_noa_hdwe = value; };
    void get_Tb()           { int16_t value;        rP_->get(Tb_eeram_.a16, value);           Tb = value; };
    void get_vb()           { int16_t value;        rP_->get(vb_eeram_.a16, value);           vb = value; };
    void get_ib()           { int16_t value;        rP_->get(ib_eeram_.a16, value);           ib = value; };
    void get_soc()          { int16_t value;        rP_->get(soc_eeram_.a16, value);          soc = value; };
    void get_soc_min()      { int16_t value;        rP_->get(soc_min_eeram_.a16, value);      soc_min = value; };
    void get_soc_ekf()      { int16_t value;        rP_->get(soc_ekf_eeram_.a16, value);      soc_ekf = value; };
    void get_voc()          { int16_t value;        rP_->get(voc_eeram_.a16, value);          voc = value; };
    void get_voc_stat()     { int16_t value;        rP_->get(voc_stat_eeram_.a16, value);     voc_stat = value; };
    void get_e_wrap_filt()  { int16_t value;        rP_->get(e_wrap_filt_eeram_.a16, value);  e_wrap_filt = value; };
    void get_fltw()         { uint16_t value;       rP_->get(fltw_eeram_.a16, value);         fltw = value; };
    void get_falw()         { uint16_t value;       rP_->get(falw_eeram_.a16, value);         falw = value; };
    void instantiate(SerialRAM *ram, uint16_t *next);
  #endif

  void get();
  void put(const Datum_st input);
  void put_nominal();

  #ifdef USE_EEPROM
   void put_t_flt()        { rP_->put(t_flt_eeram_.a16, t_flt); };
    void put_Tb_hdwe()      { rP_->put(Tb_hdwe_eeram_.a16, Tb_hdwe); };
    void put_vb_hdwe()      { rP_->put(vb_hdwe_eeram_.a16, vb_hdwe); };
    void put_ib_amp_hdwe()  { rP_->put(ib_amp_hdwe_eeram_.a16, ib_amp_hdwe); };
    void put_ib_noa_hdwe()  { rP_->put(ib_noa_hdwe_eeram_.a16, ib_noa_hdwe); };
    void put_Tb()           { rP_->put(Tb_eeram_.a16, Tb); };
    void put_vb()           { rP_->put(vb_eeram_.a16, vb); };
    void put_ib()           { rP_->put(ib_eeram_.a16, ib); };
    void put_soc()          { rP_->put(soc_eeram_.a16, soc); };
    void put_soc_min()      { rP_->put(soc_min_eeram_.a16, soc_min); };
    void put_soc_ekf()      { rP_->put(soc_ekf_eeram_.a16, soc_ekf); };
    void put_voc()          { rP_->put(voc_eeram_.a16, voc); };
    void put_voc_stat()     { rP_->put(voc_stat_eeram_.a16, voc_stat); };
    void put_e_wrap_filt()  { rP_->put(e_wrap_filt_eeram_.a16, e_wrap_filt); };
    void put_fltw()         { rP_->put(fltw_eeram_.a16, fltw); };
    void put_falw()         { rP_->put(falw_eeram_.a16, falw); };
    void put_t_flt(const unsigned long value)     { rP_->put(t_flt_eeram_.a16, value);        t_flt = value; };
    void put_Tb_hdwe(const int16_t value)         { rP_->put(Tb_hdwe_eeram_.a16, value);      Tb_hdwe = value; };
    void put_vb_hdwe(const int16_t value)         { rP_->put(vb_hdwe_eeram_.a16, value);      vb_hdwe = value; };
    void put_ib_amp_hdwe(const int16_t value)     { rP_->put(ib_amp_hdwe_eeram_.a16, value);  ib_amp_hdwe = value; };
    void put_ib_noa_hdwe(const int16_t value)     { rP_->put(ib_noa_hdwe_eeram_.a16, value);  ib_noa_hdwe = value; };
    void put_Tb(const int16_t value)              { rP_->put(Tb_eeram_.a16, value);           Tb = value; };
    void put_vb(const int16_t value)              { rP_->put(vb_eeram_.a16, value);           vb = value; };
    void put_ib(const int16_t value)              { rP_->put(ib_eeram_.a16, value);           ib = value; };
    void put_soc(const int16_t value)             { rP_->put(soc_eeram_.a16, value);          soc = value; };
    void put_soc_min(const int16_t value)         { rP_->put(soc_min_eeram_.a16, value);      soc_min = value; };
    void put_soc_ekf(const int16_t value)         { rP_->put(soc_ekf_eeram_.a16, value);      soc_ekf = value; };
    void put_voc(const int16_t value)             { rP_->put(voc_eeram_.a16, value);          voc = value; };
    void put_voc_stat(const int16_t value)        { rP_->put(voc_stat_eeram_.a16, value);     voc_stat = value; };
    void put_e_wrap_filt(const int16_t value)     { rP_->put(e_wrap_filt_eeram_.a16, value);  e_wrap_filt = value; };
    void put_fltw(const uint16_t value)           { rP_->put(fltw_eeram_.a16, value);         fltw = value; };
    void put_falw(const uint16_t value)           { rP_->put(falw_eeram_.a16, value);         falw = value; };
  #else
    void put_t_filt(const time_t value)            { t_filt = value; };
    void put_a_filt(const int16_t value)           { a_filt = value; };
    void put_b_filt(const int16_t value)           { b_filt = value; };
    void put_c_filt(const int16_t value)           { c_filt = value; };
    void put_o_filt(const int16_t value)           { o_filt = value; };
    void put_x_filt(const int16_t value)           { x_filt = value; };
    void put_y_filt(const int16_t value)           { y_filt = value; };
    void put_z_filt(const int16_t value)           { z_filt = value; };
    void put_g_filt(const int16_t value)           { g_filt = value; };
  #endif

protected:
  // SerialRAM *rP_;
  #ifdef USE_EEPROM
    address16b t_flt_eeram_;
    address16b Tb_hdwe_eeram_;
    address16b vb_hdwe_eeram_;
    address16b ib_amp_hdwe_eeram_;
    address16b ib_noa_hdwe_eeram_;
    address16b Tb_eeram_;
    address16b vb_eeram_;
    address16b ib_eeram_;
    address16b soc_eeram_;
    address16b soc_min_eeram_;
    address16b soc_ekf_eeram_;
    address16b voc_eeram_;
    address16b voc_stat_eeram_;
    address16b e_wrap_filt_eeram_;
    address16b fltw_eeram_;
    address16b falw_eeram_;
  #endif
};


#endif
