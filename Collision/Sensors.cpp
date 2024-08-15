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

// Sample the IMU
void Sensors::sample(const boolean reset, const unsigned long long time_now)
{

    if ( reset )
    {
        time_rot_last = time_now;
        time_acc_last = time_now;
    }

    if ( !reset && IMU.accelerationAvailable() )
    {
        IMU.readAcceleration(x_raw, y_raw, z_raw);
        T_acc = double(time_now - time_rot_last) / 1000.;
        time_acc_last = time_now;
        acc_available = true;
    }
    else acc_available = false;

    if ( !reset && IMU.gyroscopeAvailable() )
    {
        IMU.readGyroscope(a_raw, b_raw, c_raw);
        T_rot = double(time_now - time_rot_last) / 1000.;
        time_rot_last = time_now;
        rot_available = true;
    }
    else rot_available = false;

}

// Filter noise
void Sensors::filter(const boolean reset)
{

    if ( reset || acc_available )
    {
        x_filt = X_Filt->calculate(x_raw, reset, TAU_FILT, T_acc);
        y_filt = Y_Filt->calculate(y_raw, reset, TAU_FILT, T_acc);
        z_filt = Z_Filt->calculate(z_raw, reset, TAU_FILT, T_acc);
    }

    if ( reset || rot_available )
    {
        a_filt = A_Filt->calculate(a_raw, reset, TAU_FILT, T_rot);
        b_filt = B_Filt->calculate(b_raw, reset, TAU_FILT, T_rot);
        c_filt = C_Filt->calculate(c_raw, reset, TAU_FILT, T_rot);
    }

}
