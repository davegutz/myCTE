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


#include "constants.h"
#include "Sensors.h"

// Filter noise
void Sensors::filter(const boolean reset)
{

    if ( reset || acc_available )
    {
        x_filt = X_Filt->calculate(x_raw, reset, TAU_FILT, T_acc);
        y_filt = Y_Filt->calculate(y_raw, reset, TAU_FILT, T_acc);
        z_filt = Z_Filt->calculate(z_raw, reset, TAU_FILT, T_acc);
        g_filt = G_Filt->calculate(g_raw, reset, TAU_FILT, T_acc);
    }

    if ( reset || rot_available )
    {
        a_filt = A_Filt->calculate(a_raw, reset, TAU_FILT, T_rot);
        b_filt = B_Filt->calculate(b_raw, reset, TAU_FILT, T_rot);
        c_filt = C_Filt->calculate(c_raw, reset, TAU_FILT, T_rot);
        o_filt = O_Filt->calculate(o_raw, reset, TAU_FILT, T_rot);
    }

}

// Publish header
void Sensors::publish_all_header()
{
  Serial.println("T_rot*100\ta_filt\tb_filt\tc_filt\to_filt\t\tT_acc*100\tx_filt\ty_filt\tz_filt\tg_filt");
}

// Print publish
void Sensors::publish_all()
{
  Serial.print(T_rot*100.);
  Serial.print('\t');
  Serial.print(a_filt);
  Serial.print('\t');
  Serial.print(b_filt);
  Serial.print('\t');
  Serial.print(c_filt);
  Serial.print('\t');
  Serial.print(o_filt);
  Serial.print('\t');
  Serial.print('\t');
  Serial.print(T_acc*100.);
  Serial.print('\t');
  Serial.print(x_filt);
  Serial.print('\t');
  Serial.print(y_filt);
  Serial.print('\t');
  Serial.print(z_filt);
  Serial.print('\t');
  Serial.println(g_filt);
}

void Sensors::publish_total_header()
{
  Serial.println("T_rot*100\to_filt\t\tT_acc*100\tg_filt");
}

// Print publish
void Sensors::publish_total()
{
  Serial.print(T_rot*100.);
  Serial.print('\t');
  Serial.print(o_filt);
  Serial.print('\t');
  Serial.print('\t');
  Serial.print(T_acc*100.);
  Serial.print('\t');
  Serial.println(g_filt);
}

// Sample the IMU
void Sensors::sample(const boolean reset, const unsigned long long time_now)
{
    // Reset
    if ( reset )
    {
        time_rot_last = time_now;
        time_acc_last = time_now;
    }

    // Accelerometer
    if ( !reset && IMU.accelerationAvailable() )
    {
        IMU.readAcceleration(x_raw, y_raw, z_raw);
        time_acc_last = time_now;
        acc_available = true;
        g_raw = sqrt(x_raw*x_raw + y_raw*y_raw + z_raw*z_raw);
    }
    else acc_available = false;
    T_acc = max( double(time_now - time_acc_last) / 1000., NOM_DT );

    // Gyroscope
    if ( !reset && IMU.gyroscopeAvailable() )
    {
        IMU.readGyroscope(a_raw, b_raw, c_raw);
        a_raw *= deg_to_rps;
        b_raw *= deg_to_rps;
        c_raw *= deg_to_rps;
        time_rot_last = time_now;
        rot_available = true;
        o_raw = sqrt(a_raw*a_raw + b_raw*b_raw + c_raw*c_raw);
    }
    else rot_available = false;
    T_rot = max( double(time_now - time_rot_last) / 1000., NOM_DT );

    // Serial.print("sample: "); Serial.print(time_now); Serial.print(" "); Serial.print(time_acc_last); Serial.print(" "); Serial.println(T_acc);
    // Serial.print("acc_available: "); Serial.println(acc_available);
}
