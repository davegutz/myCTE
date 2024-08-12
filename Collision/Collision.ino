/*
  Arduino LSM6DS3 - Simple Accelerometer

  This example reads the acceleration values from the LSM6DS3
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Uno WiFi Rev 2 or Arduino Nano 33 IoT

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#define USE_ARDUINO

#ifdef USE_ARDUINO
  #include <Arduino.h> //needed for Serial.println
  #include <Arduino_LSM6DS3.h>
#else
  #error "Only Arduino nano 33 iot has built in IMU"
  #include "application.h"  // Particle
#endif

// Dependent includes.   Easier to sp.debug code if remove unused include files
#include "Sync.h"
#include "myFilters.h"

#define TALK_DELAY            313UL     // Talk wait, ms (313UL = 0.313 sec)
#define READ_DELAY             10UL     // Sensor read wait, ms (10UL = 0.01 sec) Dr
#define CONTROL_DELAY         100UL     // Control read wait, ms (100UL = 0.1 sec)
#define PLOT_DELAY            100UL     // Plot wait, ms (100UL = 0.1 sec)
#define TAU_FILT               0.05     // Tau filter, sec (0.05)
#define G_MAX                   20.     // Max G value, g's (20.) 

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  delay(5);  // Time to start serial monitor not Arduino IDE

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");

  Serial.println("T\tA\tB\tC\tX\tY\tZ");


}

void loop()
{
  // Synchronization
  static unsigned long long now = (unsigned long long) millis();
  now = (unsigned long long) millis();
  boolean chitchat = false;
  static Sync *Talk = new Sync(TALK_DELAY);
  boolean read = false;
  static Sync *ReadSensors = new Sync(READ_DELAY);
  boolean updating_plots;
  static Sync *Plotting = new Sync(PLOT_DELAY);
  boolean control;
  static Sync *ControlSync = new Sync(CONTROL_DELAY);
  unsigned long long elapsed = 0;
  static boolean reset = true;
  static unsigned long long start = millis();
  // LagExp *A_Filt = new LagExp(READ_DELAY, TAU_FILT, -G_MAX, G_MAX);  // Update time and time constant changed on the fly
  static float a = 0;
  static float b = 0;
  static float c = 0;
  static float x = 0;
  static float y = 0;
  static float z = 0;
  static float T = 0;
  static float a_filt = 0;
  boolean gyro_ready = false;
  boolean accel_ready = false;


  ///////////////////////////////////////////////////////////// Top of loop////////////////////////////////////////

  // Synchronize
  read = ReadSensors->update(millis(), reset);
  chitchat = Talk->update(millis(), reset);
  elapsed = ReadSensors->now() - start;
  T = ReadSensors->updateTime();
  control = ControlSync->update(millis(), reset);
  updating_plots = Plotting->update(millis(), reset);



  // Read sensors
  if ( read )
  {
    T = elapsed/1000.;
    if (IMU.accelerationAvailable())
    {
      IMU.readAcceleration(x, y, z);
      accel_ready = true;
    }
    if (IMU.gyroscopeAvailable())
    {
      IMU.readGyroscope(a, b, c);
      gyro_ready = true;
    }
    // a_filt = A_Filt->calculate(a, reset, TAU_FILT, T);
  }

  if (updating_plots)
  {
    Serial.print(T);
    Serial.print('\t');
    Serial.print(a_filt);
    Serial.print('\t');
    Serial.print(b);
    Serial.print('\t');
    Serial.print(c);
    Serial.print('\t');
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }

  gyro_ready = false;
  accel_ready = false;

  // Initialize complete once sensors and models started and summary written
  if ( read ) reset = false;

}  // loop
