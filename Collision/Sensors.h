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

#ifndef _MY_SENSORS_H
#define _MY_SENSORS_H


#ifdef USE_ARDUINO
  #include <Arduino.h> //needed for Serial.println
  #include <Arduino_LSM6DS3.h>
#else
  #error "Only Arduino nano 33 iot has built in IMU"
  #include "application.h"  // Particle
#endif

#include "myFilters.h"



// Sensors (like a big struct with public access)
class Sensors
{
public:
    Sensors(): T(0), a_raw(0), b_raw(0), c_raw(0), x_raw(0), y_raw(0), z_raw(0),
                a_filt(0), b_filt(0), c_filt(0), x_filt(0), y_filt(0), z_filt(0),
                time_acc_last(0ULL), time_rot_last(0ULL)
    {
    };
    Sensors(const unsigned long long time_now, const double NOM_DT ): T(0), a_raw(0), b_raw(0), c_raw(0), x_raw(0), y_raw(0), z_raw(0),
                a_filt(0), b_filt(0), c_filt(0), x_filt(0), y_filt(0), z_filt(0),
                time_acc_last(time_now), time_rot_last(time_now)
    {
        LagExp *A_Filt = new LagExp(READ_DELAY, TAU_FILT, -G_MAX, G_MAX);  // Update time and time constant changed on the fly
        LagExp *B_Filt = new LagExp(READ_DELAY, TAU_FILT, -G_MAX, G_MAX);  // Update time and time constant changed on the fly
        LagExp *C_Filt = new LagExp(READ_DELAY, TAU_FILT, -G_MAX, G_MAX);  // Update time and time constant changed on the fly
        LagExp *X_Filt = new LagExp(READ_DELAY, TAU_FILT, -W_MAX, W_MAX);  // Update time and time constant changed on the fly
        LagExp *Y_Filt = new LagExp(READ_DELAY, TAU_FILT, -W_MAX, W_MAX);  // Update time and time constant changed on the fly
        LagExp *Z_Filt = new LagExp(READ_DELAY, TAU_FILT, -W_MAX, W_MAX);  // Update time and time constant changed on the fly
    };
    unsigned long long millis;
    ~Sensors(){};

    void filter(const boolean reset);
    void sample(const boolean reset, const unsigned long long time_now);
    double T;
    float a_raw;  // Gyroscope in degrees/second
    float b_raw;  // Gyroscope in degrees/second
    float c_raw;  // Gyroscope in degrees/second
    float x_raw;  // Acceleration in g's
    float y_raw;  // Acceleration in g's
    float z_raw;  // Acceleration in g's
    float a_filt;
    float b_filt;
    float c_filt;
    float x_filt;
    float y_filt;
    float z_filt;
protected:
    LagExp *A_Filt;     // Noise filter
    LagExp *B_Filt;     // Noise filter
    LagExp *C_Filt;     // Noise filter
    LagExp *X_Filt;     // Noise filter
    LagExp *Y_Filt;     // Noise filter
    LagExp *Z_Filt;     // Noise filter
    unsigned long long time_acc_last;
    unsigned long long time_rot_last;
    double T_acc;
    double T_rot;
    boolean acc_available;
    boolean rot_available;
};

#endif
