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
#include <SafeString.h>
#include "FlashStorage.h"

// Constants; anything numeric (adjustable)
#define TALK_DELAY            313UL     // Talk wait, ms (313UL = 0.313 sec)
#define READ_DELAY             10UL     // Sensor read wait, ms (10UL = 0.01 sec) Dr
#define CONTROL_DELAY         100UL     // Control read wait, ms (100UL = 0.1 sec)
#define PLOT_DELAY            100UL     // Plot wait, ms (100UL = 0.1 sec)
#define TAU_FILT               0.05     // Tau filter, sec (0.05)
#define G_MAX                  100.     // Max G value, g's (20.) 
#define W_MAX                  100.     // Max rotational value, deg/s (20.)
#define INPUT_BYTES             200     // Serial input buffer sizes
#define SERIAL_BAUD          115200     // Serial baud rate

// Global
cSF(serial_str, INPUT_BYTES, "");
cSF(input_str, INPUT_BYTES, "");
boolean string_cpt = false;
boolean plotting = false;
boolean monitoring = false;

// Create a structure that is big enough to contain a name
// and a surname. The "valid" variable is set to "true" once
// the structure is filled with actual data for the first time.
typedef struct {
  boolean valid;
  char name[100];
  char surname[100];
} Person;

// Reserve a portion of flash memory to store a "Person" and
// call it "my_flash_store".
FlashStorage(my_flash_store, Person);

// Note: the area of flash memory reserved lost every time
// the sketch is uploaded on the board.

// Setup
void setup() {

  // Serial
  Serial.begin(SERIAL_BAUD);
  while (!Serial);

  if ( !IMU.begin() )
  {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Create a "Person" variable and call it "owner"
  Person owner;

  // Read the content of "my_flash_store" into the "owner" variable
  owner = my_flash_store.read();

  // If this is the first run the "valid" value should be "false"...
  if (owner.valid == false) {

    // ...in this case we ask for user data.
    SERIAL_PORT_MONITOR.setTimeout(30000);
    SERIAL_PORT_MONITOR.println("Insert your name:");
    String name = SERIAL_PORT_MONITOR.readStringUntil('\n');
    SERIAL_PORT_MONITOR.println("Insert your surname:");
    String surname = SERIAL_PORT_MONITOR.readStringUntil('\n');

    // Fill the "owner" structure with the data entered by the user...
    name.toCharArray(owner.name, 100);
    surname.toCharArray(owner.surname, 100);
    // set "valid" to true, so the next time we know that we
    // have valid data inside
    owner.valid = true;

    // ...and finally save everything into "my_flash_store"
    my_flash_store.write(owner);

    // Print a confirmation of the data inserted.
    SERIAL_PORT_MONITOR.println();
    SERIAL_PORT_MONITOR.print("Your name: ");
    SERIAL_PORT_MONITOR.println(owner.name);
    SERIAL_PORT_MONITOR.print("and your surname: ");
    SERIAL_PORT_MONITOR.println(owner.surname);
    SERIAL_PORT_MONITOR.println("have been saved. Thank you!");

  }
  else
  {

    // Say hello to the returning user!
    SERIAL_PORT_MONITOR.println();
    SERIAL_PORT_MONITOR.print("Hi ");
    SERIAL_PORT_MONITOR.print(owner.name);
    SERIAL_PORT_MONITOR.print(" ");
    SERIAL_PORT_MONITOR.print(owner.surname);
    SERIAL_PORT_MONITOR.println(", nice to see you again :-)");

  }

  // Time to start serial monitor not Arduino IDE
  delay(5);

  // Say 'Hello'
  say_hello();

}


// Loop
void loop()
{
  // Synchronization
  static unsigned long long now = (unsigned long long) millis();
  now = (unsigned long long) millis();
  boolean chitchat = false;
  static Sync *Talk = new Sync(TALK_DELAY);
  boolean read = false;
  static Sync *ReadSensors = new Sync(READ_DELAY);
  boolean publishing;
  static Sync *Plotting = new Sync(PLOT_DELAY);
  boolean control;
  static Sync *ControlSync = new Sync(CONTROL_DELAY);
  unsigned long long elapsed = 0;
  static boolean reset = true;
  static unsigned long long start = millis();
  static LagExp *A_Filt = new LagExp(READ_DELAY, TAU_FILT, -G_MAX, G_MAX);  // Update time and time constant changed on the fly
  static LagExp *B_Filt = new LagExp(READ_DELAY, TAU_FILT, -G_MAX, G_MAX);  // Update time and time constant changed on the fly
  static LagExp *C_Filt = new LagExp(READ_DELAY, TAU_FILT, -G_MAX, G_MAX);  // Update time and time constant changed on the fly
  static LagExp *X_Filt = new LagExp(READ_DELAY, TAU_FILT, -W_MAX, W_MAX);  // Update time and time constant changed on the fly
  static LagExp *Y_Filt = new LagExp(READ_DELAY, TAU_FILT, -W_MAX, W_MAX);  // Update time and time constant changed on the fly
  static LagExp *Z_Filt = new LagExp(READ_DELAY, TAU_FILT, -W_MAX, W_MAX);  // Update time and time constant changed on the fly
  static float a = 0;
  static float b = 0;
  static float c = 0;
  static float x = 0;
  static float y = 0;
  static float z = 0;
  static float T = 0;
  static float a_filt = 0;
  static float b_filt = 0;
  static float c_filt = 0;
  static float x_filt = 0;
  static float y_filt = 0;
  static float z_filt = 0;
  boolean gyro_ready = false;
  boolean accel_ready = false;
  static boolean monitoring_past = monitoring;


  ///////////////////////////////////////////////////////////// Top of loop////////////////////////////////////////

  // Synchronize
  read = ReadSensors->update(millis(), reset);
  chitchat = Talk->update(millis(), reset);
  elapsed = ReadSensors->now() - start;
  T = ReadSensors->updateTime();
  control = ControlSync->update(millis(), reset);
  publishing = Plotting->update(millis(), reset) && ( plotting || monitoring );

  // Read sensors
  if ( read )
  {
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
    a_filt = A_Filt->calculate(a, reset, TAU_FILT, T);
    b_filt = B_Filt->calculate(b, reset, TAU_FILT, T);
    c_filt = C_Filt->calculate(c, reset, TAU_FILT, T);
    x_filt = X_Filt->calculate(x, reset, TAU_FILT, T);
    y_filt = Y_Filt->calculate(y, reset, TAU_FILT, T);
    z_filt = Z_Filt->calculate(z, reset, TAU_FILT, T);
  }

  if ( publishing )
  {
    if ( plotting || ( monitoring && ( monitoring != monitoring_past ) ) ) print_header();
    monitoring_past = monitoring;
    if ( monitoring || plotting )
    {
      publish_print(T, a_filt, b_filt, c_filt, x_filt, y_filt, z_filt);
    }
  }

  gyro_ready = false;
  accel_ready = false;

  // Initialize complete once sensors and models started and summary written
  if ( read ) reset = false;

  if ( chitchat )
  {
    read_serial();  // returns one command at a time
    if ( input_str.length() )
    {
      switch ( input_str.charAt(0) )
      {
        case ( 'P' ):
          plotting = !plotting;
          monitoring = false;
          break;
        case ( 'M' ):
          monitoring = !monitoring;
          plotting = false;
          break;
        default:
          Serial.print(input_str.charAt(0)); Serial.println(" unknown");
      }
    }
    input_str = "";
  }


}  // loop


// Read serial for chitchat
void read_serial()
{
  boolean serial_ready = false;
  serial_str = "";

  // Each pass try to complete input from avaiable
  while ( !serial_ready && Serial.available() )
  {
    char in_char = (char)Serial.read();  // get the new byte

    // Intake
    // if the incoming character to finish, add a ';' and set flags so the main loop can do something about it:
    if ( is_finished(in_char) )
    {
        if ( serial_str.length() ) serial_str.concat(';');
        serial_ready = true;
        break;
    }

    else if ( in_char == '\r' )
    {
        Serial.println("\n");  // scroll user terminal
    }

    else if ( in_char == '\b' && serial_str.length() )
    {
        Serial.print("\b \b");  // scroll user terminal
        serial_str.remove(serial_str.length() -1 );  // backspace
    }

    else
    {
        serial_str += in_char;  // process new valid character
    }

  }

  // Pass info to serial_str
  if ( serial_ready )
  {
    input_str += serial_str.c_str();
    finish_request(input_str);
    serial_ready = false;
    serial_str = "";
  }
}

// Cleanup string for final processing by chitchat
void finish_request(SafeString &str)
{
  // Remove whitespace
  str.trim();
  str.replace("\n", "");
  str.replace("\0", "");
  str.replace("", "");
  str.replace(",", "");
  str.replace(" ", "");
  str.replace("=", "");
  str.replace(";", "");
}

// Test for string completion character
boolean is_finished(const char in_char)
{
    return  in_char == '\n' ||
            in_char == '\0' ||
            in_char == ';'  ||
            in_char == ',';    
}

// Print publish
void publish_print(const float T, const float a_filt, const float b_filt, const float c_filt, const float x_filt, const float y_filt, const float z_filt)
{
  Serial.print(T);
  Serial.print('\t');
  Serial.print(a_filt);
  Serial.print('\t');
  Serial.print(b_filt);
  Serial.print('\t');
  Serial.print(c_filt);
  Serial.print('\t');
  Serial.print(x_filt);
  Serial.print('\t');
  Serial.print(y_filt);
  Serial.print('\t');
  Serial.println(z_filt);
}

// Publish header
void print_header()
{
  Serial.println("T\ta_filt\tb_filt\tc_filt\tx_filt\ty_filt\tz_filt");
}

// Say hello
void say_hello()
{
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
}
