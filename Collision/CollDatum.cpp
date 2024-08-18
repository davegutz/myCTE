//
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

#include "CollDatum.h"

// extern SavedPars sp;       // Various parameters to be static at system level and saved through power cycle
// extern VolatilePars ap; // Various adjustment parameters shared at system level

// struct Datum_st.  This file needed to avoid circular reference to sp in header files
void Datum_st::assign(const time_t now, Sensors *Sen)
{
  t_filt = Sen->t_filt;
  a_filt = Sen->a_filt;
  b_filt = Sen->b_filt;
  c_filt = Sen->c_filt;
  o_filt = Sen->o_filt;
  x_filt = Sen->x_filt;
  y_filt = Sen->y_filt;
  z_filt = Sen->z_filt;
  g_filt = Sen->g_filt;
}

// Copy function
void Datum_st::copy_to_datum_ram_from(Datum_st input)
{
  t_filt = input.t_filt;
  a_filt = input.a_filt;
  b_filt = input.b_filt;
  c_filt = input.c_filt;
  o_filt = input.o_filt;
  x_filt = input.x_filt;
  y_filt = input.y_filt;
  z_filt = input.z_filt;
  g_filt = input.g_filt;
}

// Nominal values
void Datum_st::nominal()
{
  t_filt = time_t (0);
  a_filt = int16_t(0);
  b_filt = int16_t(0);
  c_filt = int16_t(0);
  o_filt = int16_t(0);
  x_filt = int16_t(0);
  y_filt = int16_t(0);
  z_filt = int16_t(0);
  g_filt = int16_t(0);
}

// Print functions
void Datum_st::print()
{
  cSF(buffer, 32, "");
  buffer = "---";
  if ( this->t_filt > 1L )
  {
    time_long_2_str(this->t_filt, buffer);
    Serial.print(t_filt);
    Serial.print(" "); Serial.print(buffer);
    Serial.print(" t_filt "); Serial.print(t_filt);
    Serial.print(" a_filt "); Serial.print(a_filt);
    Serial.print(" b_filt "); Serial.print(b_filt);
    Serial.print(" o_filt "); Serial.print(o_filt);
    Serial.print(" c_filt "); Serial.print(c_filt);
    Serial.print(" x_filt "); Serial.print(x_filt);
    Serial.print(" y_filt "); Serial.print(y_filt);
    Serial.print(" z_filt "); Serial.print(z_filt);
    Serial.print(" g_filt "); Serial.println(g_filt);
     }
}

// Regular put
void Datum_st::put(Datum_st source)
{
  copy_to_datum_ram_from(source);
}

// nominalize
void Datum_st::put_nominal()
{
  Datum_st source;
  source.nominal();
  copy_to_datum_ram_from(source);
}

// Class fault ram to interface Datum_st to ram
Datum_ram::Datum_ram()
{
  Datum_st();
}
Datum_ram::~Datum_ram(){}

// Load all
#ifdef USE_EEPROM

  void Datum_ram::get()
  {
    get_t_flt();
    get_Tb_hdwe();
    get_vb_hdwe();
    get_ib_amp_hdwe();
    get_ib_noa_hdwe();
    get_Tb();
    get_vb();
    get_ib();
    get_soc();
    get_soc_ekf();
    get_voc();
    get_voc_stat();
    get_e_wrap_filt();
    get_e_wrap_m_filt();
    get_e_wrap_n_filt();
    get_fltw();
    get_falw();
  }

  // Initialize each structure
  void Datum_ram::instantiate(SerialRAM *ram, uint16_t *next)
  {
    t_flt_eeram_.a16 = *next; *next += sizeof(t_flt);
    Tb_hdwe_eeram_.a16 = *next; *next += sizeof(Tb_hdwe);
    vb_hdwe_eeram_.a16 = *next; *next += sizeof(vb_hdwe);
    ib_amp_hdwe_eeram_.a16 = *next; *next += sizeof(ib_amp_hdwe);
    ib_noa_hdwe_eeram_.a16 = *next; *next += sizeof(ib_noa_hdwe);
    Tb_eeram_.a16 = *next; *next += sizeof(Tb);
    vb_eeram_.a16 = *next; *next += sizeof(vb);
    ib_eeram_.a16 = *next; *next += sizeof(ib);
    soc_eeram_.a16 = *next; *next += sizeof(soc);
    soc_min_eeram_.a16 = *next; *next += sizeof(soc_min);
    soc_ekf_eeram_.a16 = *next; *next += sizeof(soc_ekf);
    voc_eeram_.a16 = *next; *next += sizeof(voc);
    voc_stat_eeram_.a16 = *next; *next += sizeof(voc_stat);
    e_wrap_filt_eeram_.a16 = *next; *next += sizeof(e_wrap_filt);
    e_wrap_m_filt_eeram_.a16 = *next; *next += sizeof(e_wrap_m_filt);
    e_wrap_n_filt_eeram_.a16 = *next; *next += sizeof(e_wrap_n_filt);
    fltw_eeram_.a16 = *next; *next += sizeof(fltw);
    falw_eeram_.a16 = *next; *next += sizeof(falw);
    rP_ = ram;
    nominal();
  }
#endif

// Save all
void Datum_ram::put(const Datum_st value)
{
  copy_to_datum_ram_from(value);
  #ifdef USE_EEPROM

    put_t_flt();
    put_Tb_hdwe();
    put_vb_hdwe();
    put_ib_amp_hdwe();
    put_ib_noa_hdwe();
    put_Tb();
    put_vb();
    put_ib();
    put_soc();
    put_soc_ekf();
    put_voc();
    put_voc_stat();
    put_e_wrap_filt();
    put_e_wrap_m_filt();
    put_e_wrap_n_filt();
    put_fltw();
    put_falw();
  #endif
}


// nominalize
void Datum_ram::put_nominal()
{
  Datum_st source;
  source.nominal();
  put(source);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

// For summary prints
void time_long_2_str(const time_t time, SafeString &return_str)
{
    // Serial.printf("Time.year:  time_t %d ul %d as-is %d\n", 
    //   Time.year((time_t) 1703267248), Time.year((unsigned long )1703267248), Time.year(time));
    char tempStr[32];
    #ifndef USE_ARDUINO
      uint32_t year_ = Time.year(time);
      uint8_t month_ = Time.month(time);
      uint8_t day_ = Time.day(time);
      uint8_t hours_ = Time.hour(time);
      uint8_t minutes_   = Time.minute(time);
      uint8_t seconds_   = Time.second(time);
    #else
      uint32_t year_ = year(time);
      uint8_t month_ = month(time);
      uint8_t day_ = day(time);
      uint8_t hours_ = hour(time);
      uint8_t minutes_   = minute(time);
      uint8_t seconds_   = second(time);
    #endif
    sprintf(tempStr, "%4u-%02u-%02uT%02u:%02u:%02u", int(year_), month_, day_, hours_, minutes_, seconds_);
    return_str = tempStr;
}

