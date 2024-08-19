//
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

#include "CollDatum.h"

// extern SavedPars sp;       // Various parameters to be static at system level and saved through power cycle
// extern VolatilePars ap; // Various adjustment parameters shared at system level

// struct Datum_st data points
// Copy function
void Datum_st::copy_to_datum_ram_from(Datum_st input)
{
  stamp = input.stamp;
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
  stamp = time_t(0);
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
    Serial.print(" ID "); Serial.print(stamp);
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
void Datum_st::put(const time_t event, Sensors *Sen)
{
  stamp = event;
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


// struct Data_st data log
void Data_st::put(const boolean reset, const time_t id, Sensors *Sen)
{
  if ( reset )
  {
    i_ = 0;
    for ( int j=0; j<n_; j++ ) data[j]->put_nominal();
  }
  if ( ++i_ > (n_-1) ) i_ = 0; // circular buffer
  data[i_]->put(id, Sen);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////

// For summary prints
void time_long_2_str(const time_t time, SafeString &return_str)
{
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

