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
  t_raw = input.t_raw;
  T_rot_raw_int = input.T_rot_raw_int;
  a_raw_int = input.a_raw_int;
  b_raw_int = input.b_raw_int;
  c_raw_int = input.c_raw_int;
  T_acc_raw_int = input.T_acc_raw_int;
  x_raw_int = input.x_raw_int;
  y_raw_int = input.y_raw_int;
  z_raw_int = input.z_raw_int;
}

// Nominal values
void Datum_st::nominal()
{
  t_raw = time_t (1UL);
  T_rot_raw_int = int16_t(0);
  a_raw_int = int16_t(0);
  b_raw_int = int16_t(0);
  c_raw_int = int16_t(0);
  T_acc_raw_int = int16_t(0);
  x_raw_int = int16_t(0);
  y_raw_int = int16_t(0);
  z_raw_int = int16_t(0);
}

// Print functions
void Datum_st::print()
{
  cSF(buffer, 32, "");
  buffer = "---";
  if ( this->t_raw > 1L )
  {
    time_long_2_str(this->t_raw, buffer);
    Serial.print(t_raw);
    Serial.print(" "); Serial.print(buffer);
    Serial.print(" t_raw "); Serial.print(t_raw);
    Serial.print(" T_rot_raw "); Serial.print(float(T_rot_raw_int) / T_SCL);
    Serial.print(" a_raw "); Serial.print(float(a_raw_int) / O_SCL);
    Serial.print(" b_raw "); Serial.print(float(b_raw_int) / O_SCL);
    Serial.print(" c_raw "); Serial.print(float(c_raw_int) / O_SCL);
    Serial.print(" T_acc_raw "); Serial.print(float(T_acc_raw_int) / T_SCL);
    Serial.print(" x_raw "); Serial.print(float(x_raw_int) / G_SCL);
    Serial.print(" y_raw "); Serial.print(float(y_raw_int) / G_SCL);
    Serial.print(" z_raw "); Serial.println(float(z_raw_int) / G_SCL);
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

// Load data
void Datum_st::put_sparse(Sensors *Sen)
{
  t_raw = Sen->t_filt;
  T_rot_raw_int = int16_t(Sen->T_rot_raw() * T_SCL);
  a_raw_int = int16_t(Sen->a_raw * O_SCL);
  b_raw_int = int16_t(Sen->b_raw * O_SCL);
  c_raw_int = int16_t(Sen->c_raw * O_SCL);
  T_acc_raw_int = int16_t(Sen->T_acc_raw() * T_SCL);
  x_raw_int = int16_t(Sen->a_raw * G_SCL);
  y_raw_int = int16_t(Sen->b_raw * G_SCL);
  z_raw_int = int16_t(Sen->c_raw * G_SCL);
}


// struct Data_st data log
void Data_st::print_all()
{
  for (int j = 0; j < n_; j++)
  {
    data[j]->print();
  }
}

void Data_st::put_datum(Sensors *Sen)
{
  if ( ++i_ > (n_-1) ) i_ = 0; // circular buffer
  data[i_]->put_sparse(Sen);
}

void Data_st::reset(const boolean reset)
{
  if ( reset )
  {
    i_ = 0;
    for ( int j=0; j<n_; j++ ) data[j]->put_nominal();
  }
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

