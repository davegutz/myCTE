/*  Collision constants

15-Aug-2024 	DA Gutz 	Created from SOC_Particle code.

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

*/

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#undef USE_EEPROM
#undef USE_ARDUINO

// Setup
#include "local_config.h"
#include <SafeString.h>

const double NOM_DT = 0.01;

// Constants; anything numeric (adjustable)
// #define NOM_DT                 0.01     // Nominal time step, placeholder overwritten with measured time, sec (0.01)
#define TALK_DELAY            313UL     // Talk wait, ms (313UL = 0.313 sec)
#define READ_DELAY             10UL     // Sensor read wait, ms (10UL = 0.01 sec) Dr
#define CONTROL_DELAY         100UL     // Control read wait, ms (100UL = 0.1 sec)
#define PLOT_DELAY            100UL     // Plot wait, ms (100UL = 0.1 sec)
#define TAU_FILT               0.05     // Tau filter, sec (0.05)
#define G_MAX                  100.     // Max G value, g's (20.) 
#define W_MAX                  100.     // Max rotational value, deg/s (20.)
#define INPUT_BYTES             200     // Serial input buffer sizes
#define SERIAL_BAUD          115200     // Serial baud rate

#endif
