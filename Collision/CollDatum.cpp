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

////////////////////////////////////////////////////////////////
// struct Datum_st data points

// Copy function
void Datum_st::from(Datum_st input)
{
  t_raw_ms = input.t_raw_ms;
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
  t_raw_ms = time_t (1ULL);
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
void Datum_st::print(const uint16_t i)
{
  cSF(prn_buff, INPUT_BYTES, "");
  prn_buff = "---";
  time_long_2_str(t_raw_ms, prn_buff);
  Serial.print(i);
  Serial.print(" "); Serial.print(t_raw_ms);
  Serial.print(" "); Serial.print(prn_buff);
  Serial.print(" T_rot_raw "); Serial.print(float(T_rot_raw_int) / T_SCL);
  Serial.print(" a_raw "); Serial.print(float(a_raw_int) / O_SCL);
  Serial.print(" b_raw "); Serial.print(float(b_raw_int) / O_SCL);
  Serial.print(" c_raw "); Serial.print(float(c_raw_int) / O_SCL);
  Serial.print(" T_acc_raw "); Serial.print(float(T_acc_raw_int) / T_SCL);
  Serial.print(" x_raw "); Serial.print(float(x_raw_int) / G_SCL);
  Serial.print(" y_raw "); Serial.print(float(y_raw_int) / G_SCL);
  Serial.print(" z_raw "); Serial.println(float(z_raw_int) / G_SCL);
}

// nominalize
void Datum_st::put_nominal()
{
  Datum_st source;
  source.nominal();
  from(source);
}

// Load data
void Datum_st::from(Sensors *Sen)
{
  t_raw_ms = Sen->t_raw_ms;
  T_rot_raw_int = int16_t(Sen->T_rot_raw() * T_SCL);
  a_raw_int = int16_t(Sen->a_raw * O_SCL);
  b_raw_int = int16_t(Sen->b_raw * O_SCL);
  c_raw_int = int16_t(Sen->c_raw * O_SCL);
  T_acc_raw_int = int16_t(Sen->T_acc_raw() * T_SCL);
  x_raw_int = int16_t(Sen->a_raw * G_SCL);
  y_raw_int = int16_t(Sen->b_raw * G_SCL);
  z_raw_int = int16_t(Sen->c_raw * G_SCL);
}


//////////////////////////////////////////////////////////
// struct Data_st data log

// Delete the over-ridden registers except the one we're currently filling
void Data_st::adjust_register_excepting(Register_st *CurrentReg)
{
  for ( int j=0; j<nRg_; j++ )
  {
    if ( CurrentReg == Reg[j] ) continue;
    if ( (iR_ < (Reg[j]->i + Reg[j]->n-1)) && (iStart_ > Reg[j]->i)  && (iR_ > Reg[j]->i) ) 
    {
Serial.print("j="); Serial.print(j); Serial.print(" iStart_="); Serial.print(iStart_); Serial.print(" iR_="); Serial.print(iR_); Serial.print(" Reg[j]->i="); Serial.print(Reg[j]->i); Serial.print(" Reg[j]->n="); Serial.print(Reg[j]->n); Serial.print(" Reg[j]->t_ms="); Serial.print(Reg[j]->t_ms);
      Serial.println(" nominal");
      Reg[j]->put_nominal();
    }
  }
  // sort_registers();
}

// Transfer precursor data to storage
void Data_st::move_precursor()
{
  uint16_t count = 0;
  uint16_t j = iP_;
  while ( count++ < nP_-1 )  // Last precursor is first of next result, so -1
  {
    if ( ++j > (nP_-1) ) j = 0;  // circular buffer
    if ( Precursor[j]->t_raw_ms == 1ULL ) continue;
    put_ram(Precursor[j]);
  }
}

void Data_st::print_all_registers()
{
  for ( int i=0; i<nRg_; i++ )
    Reg[i]->print(nR_);
}


void Data_st::print_latest_register()
{
  Reg[iRg_]->print(nR_);
}

void Data_st::print_latest_ram()
{
  int begin = max(min( Reg[iRg_]->i, nR_-1), 0);
  int end = max(min(begin + Reg[iRg_]->n, nR_-1), 0);
  for ( int i=begin; i<end; i++ )
  {
    Ram[i]->print(i);
  }
}

void Data_st::print_ram()
{
  for (int j = 0; j < nR_; j++)
  {
    Ram[j]->print(j);
  }
}

// Precursor storage
void Data_st:: put_precursor(Sensors *Sen)
{
  if ( ++iP_ > (nP_-1) ) iP_ = 0;  // circular buffer
  Precursor[iP_]->from(Sen);
}

void Data_st::put_ram(Sensors *Sen)
{
  if ( ++iR_ > (nR_-1) ) iR_ = 0;  // circular buffer
  Ram[iR_]->from(Sen);
  adjust_register_excepting(CurrentRegPtr_);
}

void Data_st::put_ram(Datum_st *point)
{
  if ( ++iR_ > (nR_-1) ) iR_ = 0;  // circular buffer
  Ram[iR_]->from(*point);
  adjust_register_excepting(CurrentRegPtr_);
}

// Enter information about last data set into register
void Data_st::register_lock()
{
  iRg_++;
  if ( iRg_ > (nRg_-1) ) iRg_ = 0;  // circular buffer
  iStart_ = iR_;
  if ( iStart_ > (nR_-1) ) iStart_ = 0;  // circular buffer
  Serial.print(" lock: iRg_="); Serial.print(iRg_); Serial.print(" iStart_="); Serial.println(iStart_);
  Reg[iRg_]->locked = true;
  Reg[iRg_]->i = iStart_;
  CurrentRegPtr_ = Reg[iRg_];
}
void Data_st::register_unlock()
{
  Reg[iRg_]->t_ms = Ram[iStart_]->t_raw_ms;
  boolean reg_wrapped = false;
  if ( Reg[iRg_]->i < iR_ )
  {
    Reg[iRg_]->n = iR_ - Reg[iRg_]->i;
  }
  else
  {
    Reg[iRg_]->n = nR_ - (Reg[iRg_]->i - iR_);
    reg_wrapped = true;
  }
  if ( reg_wrapped )
  {
    uint16_t j = 0;
    while ( ( Reg[j]->i < iR_ ) && ( j < nRg_ ) )
    {
      Reg[j]->put_nominal();
      j++;
    }
  }
  Reg[iRg_]->locked = false;
Serial.print("unlock: iRg_="); Serial.print(iRg_); Serial.print(" iR_="); Serial.print(iR_); Serial.print(" Reg[iRg_]->i="); Serial.print(Reg[iRg_]->i); Serial.print(" n="); Serial.println(Reg[iRg_]->n);

}

// Reset
void Data_st::reset(const boolean reset)
{
  if ( reset )
  {
    iP_ = 0;
    for ( int j=0; j<nP_; j++ ) Precursor[j]->put_nominal();
    iR_ = 0;
    for ( int j=0; j<nR_; j++ ) Ram[j]->put_nominal();
  }
}

// Sort registers
void Data_st::sort_registers()
{
  Serial.println("TODO: sort registers");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

// For summary prints
void time_long_2_str(const unsigned long long _time_ms, SafeString &return_str)
{
    int thou_ = _time_ms % 1000;
    time_t time = _time_ms / 1000;
    char tempStr[36];
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
    sprintf(tempStr, "%4u-%02u-%02uT%02u:%02u:%02u.%03d", int(year_), month_, day_, hours_, minutes_, seconds_, thou_);
    return_str = tempStr;
}

