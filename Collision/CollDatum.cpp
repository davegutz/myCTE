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
void Datum_st::assign(const unsigned long now, Sensors *Sen)
{
  t_raw = now;
  a_raw = Sen->a_raw;
  b_raw = Sen->b_raw;
  c_raw = Sen->c_raw;
  x_raw = Sen->x_raw;
  y_raw = Sen->y_raw;
  z_raw = Sen->z_raw;
}

// Copy function
void Datum_st::copy_to_datum_ram_from(Datum_st input)
{
  t_raw = input.t_raw;
  a_raw = input.a_raw;
  b_raw = input.b_raw;
  c_raw = input.c_raw;
  x_raw = input.x_raw;
  y_raw = input.y_raw;
  z_raw = input.z_raw;
}

// Nominal values
void Datum_st::nominal()
{
  t_raw = 1UL;
  a_raw = int16_t(0);
  b_raw = int16_t(0);
  c_raw = int16_t(0);
  x_raw = int16_t(0);
  y_raw = int16_t(0);
  z_raw = int16_t(0);
}

// Print functions
void Datum_st::pretty_print(SafeString &code)
{
  char buffer[32];
  strcpy(buffer, "---");
  if ( this->t_raw > 1UL )
  {
    Serial.print("code "); Serial.println(code);
    time_long_2_str((time_t)this->t_raw, buffer);
    Serial.print("buffer "); Serial.println(buffer);
    Serial.print("t_raw "); Serial.println(t_raw);
    Serial.print("a_raw "); Serial.println(a_raw);
    Serial.print("b_raw "); Serial.println(b_raw);
    Serial.print("c_raw "); Serial.println(c_raw);
    Serial.print("x_raw "); Serial.println(x_raw);
    Serial.print("y_raw "); Serial.println(y_raw);
    Serial.print("z_raw "); Serial.println(z_raw);
  }
}


void Datum_st::print_datum(SafeString &code)
{
  char buffer[32];
  strcpy(buffer, "---");
  if ( this->t_raw > 1UL )
  {
    time_long_2_str(this->t_raw, buffer);
    // Serial.printf("%s, %s, %ld, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %7.4f, %7.4f, %7.4f, %7.3f, %7.3f, %7.3f, %7.3f, %7.3f, %ld, %ld,\n",
    //   code.c_str(), buffer, this->t_flt,
    //   float(this->Tb_hdwe)/600.,
    //   float(this->vb_hdwe)/sp.vb_hist_slr(),
    //   float(this->ib_amp_hdwe)/sp.ib_hist_slr(),
    //   float(this->ib_noa_hdwe)/sp.ib_hist_slr(),
    //   float(this->Tb)/600.,
    //   float(this->vb)/sp.vb_hist_slr(),
    //   float(this->ib)/sp.ib_hist_slr(),
    //   float(this->soc)/16000.,
    //   float(this->soc_min)/16000.,
    //   float(this->soc_ekf)/16000.,
    //   float(this->voc)/sp.vb_hist_slr(),
    //   float(this->voc_stat)/sp.vb_hist_slr(),
    //   float(this->e_wrap_filt)/sp.vb_hist_slr(),
    //   float(this->e_wrap_m_filt)/sp.vb_hist_slr(),
    //   float(this->e_wrap_n_filt)/sp.vb_hist_slr(),
    //   this->fltw,
    //   this->falw);
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
