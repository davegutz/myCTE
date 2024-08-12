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

#include <Arduino_LSM6DS3.h>

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
  
  Serial.println(""A\tB\tC\yX\tY\tZ");


}

void loop()
{

  float a = 0;
  float b = 0;
  float c = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  boolean gyro_ready = false;
  boolean accel_ready = false;

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

  if (accel_ready || gyro_ready)
  {
    Serial.print(a);
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

}
