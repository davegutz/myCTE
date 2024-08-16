/*
  Uses LSM6DS3 in Arduino Nano 33 IoT
  This unit is reported to have BLE but I have not confirmed.
  This unit requires special EEPROM handling using FlashStorage library.

  The circuit:
  - Arduino Uno WiFi Rev 2 or Arduino Nano 33 IoT
  - USB for monitor and power

  created 10 Jul 2024
  by Dave Gutz

  This example code is in the public domain.

  Requirements:
  1.  >500 Hz sampling of 6 dof IMU to support 25 Hz high fidelity analysis
  2.  Capture and store and limited number of collisions for post download and analysis
  3.  5 collisions for now
  4.  Button cell battery
  5.  UT managed by EEPROM.  OK to synchronize on restart before use.
  6.  Store 'worst' 2 collisions in EEPROM.

  Notes:
  1.  IMU in Nano captures 6 dof IMU at 100 Hz.   Too slow but good for 20 Hz analysis
  2.  A collision is approximately 3 seconds = 300 samples of 7 variables (6 dof + integer time).
    Use experience to scale to 16 bit integers for storage of several collisions possible.  Need experiment soon.
    Each collision would need 7 * 16 * 300 bits ~ 32k bits
  3. Don't know how big EEPROM is.  Want to save 'worst' collision in EEPROM.  Always preserve 'worst'
  in both EEPROM and RAM.
  4. Arduino compiler with this barebones program says 27400 bytes left in RAM = 220 k bits enough for 4 collisions.
  5. Arduino specs say EEPROM storage is 520 KB SRAM.
    AMD21G18A Processor
    256 kB Flash 32 kB Flash (256 is future possible EEPROM.  32 is now.)
    Arduino reports:
    EEPROM size: 201
    EEPROM PAGE_SIZE: 64
    EEPROM PAGES: 4096
    EEPROM MAX_FLASH: 262144 bits = 32 kB  = 262144 / 7 / 16 = 2340 samples ~8 collisions.  Confirms info about 2nd Flash being available for EEPROM.
    EEPROM ROW_SIZE: 256

*/

#include <SafeString.h>
#include "FlashStorage.h"
#include "constants.h"

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
#include "Sensors.h"

// Global
cSF(unit, INPUT_BYTES);
cSF(serial_str, INPUT_BYTES, "");
cSF(input_str, INPUT_BYTES, "");
boolean string_cpt = false;
boolean plotting_all = false;
boolean plotting_total = false;
boolean monitoring = false;

#ifdef USE_EEPROM
  // Reserve a portion of flash memory to store a "Person" and
  // call it "my_flash_store".
  // Create a structure that is big enough to contain a name
  // and a surname. The "valid" variable is set to "true" once
  // the structure is filled with actual data for the first time.
  typedef struct {
    boolean valid;
    char name[100];
    char surname[100];
  } Person;

  FlashStorage(my_flash_store, Person);
  // Create a "Person" variable and call it "owner"
  Person owner;
  // Note: the area of flash memory reserved lost every time the sketch is uploaded on the board.
#endif

// Setup
void setup() {

  unit = version.c_str(); unit  += "_"; unit += HDWE_UNIT.c_str();

  // Serial
  Serial.begin(SERIAL_BAUD);
  while (!Serial);

  if ( !IMU.begin() )
  {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  #ifdef USE_EEPROM

    // Read the content of "my_flash_store" into the "owner" variable
    owner = my_flash_store.read();


    Serial.print("EEPROM size: "); Serial.println(my_flash_store.size());
    Serial.print("EEPROM PAGE_SIZE: "); Serial.println(my_flash_store.page_size());
    Serial.print("EEPROM PAGES: "); Serial.println(my_flash_store.pages());
    Serial.print("EEPROM MAX_FLASH: "); Serial.println(my_flash_store.max_flash());
    Serial.print("EEPROM ROW_SIZE: "); Serial.println(my_flash_store.row_size());

    // If this is the first run the "valid" value should be "false"...
    if (owner.valid == false)
    {
      populate_owner();
    }
    else
    {
      homecoming();
    }
  #endif

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
  boolean gyro_ready = false;
  boolean accel_ready = false;
  static boolean monitoring_past = monitoring;
  static Sensors *Sen = new Sensors(millis(), double(NOM_DT));


  ///////////////////////////////////////////////////////////// Top of loop////////////////////////////////////////

  // Synchronize
  read = ReadSensors->update(millis(), reset);
  chitchat = Talk->update(millis(), reset);
  elapsed = ReadSensors->now() - start;
  control = ControlSync->update(millis(), reset);
  publishing = Plotting->update(millis(), reset);

  // Read sensors
  if ( read )
  {
    // Serial.println("Reading sensors");
    Sen->sample(reset, millis());
    // Serial.println("Filtering sensors");
    Sen->filter(reset);
  }

  if ( publishing )
  {
    if ( plotting_all || ( monitoring && ( monitoring != monitoring_past ) ) ) Sen->publish_all_header();
    else if ( plotting_total || ( monitoring && ( monitoring != monitoring_past ) ) ) Sen->publish_total_header();
    monitoring_past = monitoring;
    if ( monitoring || plotting_all )
    {
      Sen->publish_all();
    }
    else if ( plotting_total ) Sen->publish_total();
  }

  gyro_ready = false;
  accel_ready = false;

  // Initialize complete once sensors and models started and summary written
  if ( read ) reset = false;

  if ( chitchat )
  {
    // Serial.println("chitchat");
    read_serial();  // returns one command at a time
    if ( input_str.length() )
    {
      switch ( input_str.charAt(0) )
      {
        case ( 'P' ):
          switch ( input_str.charAt(1) )
          {
            case ( 'a' ):
              plotting_all = !plotting_all;
              plotting_total = false;
              monitoring = false;
              break;
            case ( 't' ):
              plotting_total = !plotting_total;
              plotting_all = false;
              monitoring = false;
              break;
            default:
              Serial.print(input_str.charAt(1)); Serial.println(" unknown");
              break;
          }
        break;
        case ( 'M' ):
          monitoring = !monitoring;
          plotting_all = false;
          plotting_total = false;
          break;
        default:
          Serial.print(input_str.charAt(0)); Serial.println(" unknown");
          break;
      }
    }
    input_str = "";
    
  }
    // Serial.println("end");


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

#ifdef USE_EEPROM
  // Populate owner structure first time
  void populate_owner()
  {
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

  // Prodigal son returns!
  void homecoming()
  {
    // Say hello to the returning user!
    SERIAL_PORT_MONITOR.println();
    SERIAL_PORT_MONITOR.print("Hi ");
    SERIAL_PORT_MONITOR.print(owner.name);
    SERIAL_PORT_MONITOR.print(" ");
    SERIAL_PORT_MONITOR.print(owner.surname);
    SERIAL_PORT_MONITOR.println(", nice to see you again :-)");
  }
#endif
