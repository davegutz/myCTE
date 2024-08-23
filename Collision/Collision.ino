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
#include "CollDatum.h"
#include "TimeLib.h"

// Global
cSF(unit, INPUT_BYTES);
cSF(serial_str, INPUT_BYTES, "");
cSF(input_str, INPUT_BYTES, "");
cSF(prn_buff, INPUT_BYTES, "");
boolean string_cpt = false;
boolean plotting_all = false;
uint8_t plot_num = 0;
boolean monitoring = false;
time_t time_initial = ARBITRARY_TIME;
unsigned long long millis_flip = millis(); // Timekeeping
unsigned long long last_sync = millis();   // Timekeeping

extern int debug;
int debug = 0;


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

  // Wire.begin();
  // Wire.setClock(400000UL);

  unit = version.c_str(); unit  += "_"; unit += HDWE_UNIT.c_str();
  setTime(time_initial);

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
  static unsigned long long now_ms = (unsigned long long) millis();
  boolean chitchat = false;
  static Sync *Talk = new Sync(TALK_DELAY);
  boolean read = false;
  static Sync *ReadSensors = new Sync(READ_DELAY);
  boolean publishing;
  static Sync *Plotting = new Sync(PLOT_DELAY);
  boolean control;
  static Sync *ControlSync = new Sync(CONTROL_DELAY);
  boolean log;
  static Sync *LogSync = new Sync(LOG_DELAY);
  unsigned long long elapsed = 0;
  static boolean reset = true;
  static unsigned long long time_start = millis();
  boolean gyro_ready = false;
  boolean accel_ready = false;
  static boolean monitoring_past = monitoring;
  static time_t new_event = 0;
  static Sensors *Sen = new Sensors(millis(), double(NOM_DT));
  static Data_st *L = new Data_st(NDATUM, NHOLD, NDATA);  // Event log
  static boolean logging = false;
  static boolean logging_past = false;
  static uint16_t log_size = 0;
  boolean plotting = false;


  ///////////////////////////////////////////////////////////// Top of loop////////////////////////////////////////

  // Synchronize
  now_ms = (unsigned long long) millis();
  if ( now_ms - last_sync > ONE_DAY_MILLIS || reset )  sync_time(&last_sync, &millis_flip); 
  read = ReadSensors->update(millis(), reset);
  chitchat = Talk->update(millis(), reset);
  elapsed = ReadSensors->now() - time_start;
  control = ControlSync->update(millis(), reset);
  log = LogSync->update(millis(), reset);
  publishing = Plotting->update(millis(), reset);
  plotting = plotting_all;

  if ( reset )
  {
    Serial.print("size of ram NDATUM="); Serial.println(NDATUM);
    Serial.print("num precursors NHOLD="); Serial.println(NHOLD);
    Serial.print("num reg entries NDATA="); Serial.println(NDATA);
    Serial.print("iR="); Serial.println(L->iR());
    Serial.print("iRg="); Serial.println(L->iRg());
  }

  // Read sensors
  if ( read )
  {

    // Serial.println("Reading sensors");
    Sen->sample(reset, millis(), time_start, now());
    // Serial.println("Filtering sensors");
    Sen->filter(reset);
    Sen->quiet_decisions(reset);
    L->put_precursor(Sen);

  }  // end read

  // Log
  if ( log )
  {
    // Logic
    if ( Sen->both_not_quiet() && !logging )
    {
      logging = true;
      new_event = Sen->t_ms;
      log_size++;
    }
    else
    {
      if ( Sen->both_are_quiet() && logging ) logging = false;
       log_size = 0;
    }

    // Log data
    if ( logging )
    {
      if ( !logging_past )
      {
        Serial.println(""); Serial.println("Logging");
        L->register_lock();  // after move_precursor so has values on first save
        L->move_precursor();
      }
      L->put_ram(Sen);
    }
    else
    {
      if ( logging_past )
      {
        L->register_unlock();
        Serial.println("Logging stopped");
        if ( !plotting )
        {
          // Serial.println("All ram");
          // L->print_ram();
          Serial.println("Latest ram");
          L->print_latest_ram();
          Serial.println("Registers");
          L->print_all_registers();
          Serial.println("Latest register");
          L->print_latest_register();
        }
      }
      if ( !Sen->o_is_quiet_sure() ) Serial.print(".");
      if ( !Sen->g_is_quiet_sure() ) Serial.print(",");
    }

    logging_past = logging;
  }

  // Publish
  if ( publishing )
  {
    if ( monitoring && ( monitoring != monitoring_past ) ) Sen->print_all_header();
    if ( monitoring ) Sen->print_all();
    else if ( plotting_all )
    {
      switch ( plot_num )
      {
      case 0:
        Sen->plot_all_sum();
        break;
      case 1:
        Sen->plot_all_acc();
        break;
      case 2:
        Sen->plot_all_rot();
        break;
      case 3:
        Sen->plot_all();
        break;
      case 4:
        Sen->plot_quiet();
        break;
      case 5:
        Sen->plot_quiet_raw();
        break;
      case 6:
        Sen->plot_total();
        break;
      default:
        Serial.println("plot number unknown enter plot number e.g. pa0 (sum), pa1 (acc), pa2 (rot), or pa3 (all)");
        break;
      }
    }

    monitoring_past = monitoring;
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
      // Now we know the letters
      Serial.print("input_str: "); Serial.print(input_str);
      char letter_0 = '\0';
      char letter_1 = '\0';
      letter_0 = input_str.charAt(0);
      letter_1 = input_str.charAt(1);
      int i_value = -1;  // default value is not something used, so it stops plots
      input_str.substring(input_str, 2).toInt(i_value);
      Serial.print(" letter_0: "); Serial.print(letter_0); Serial.print(" letter_1: "); Serial.print(letter_1); Serial.print(" i_value: "); Serial.println(i_value);
      switch ( letter_0 )
      {
        case ( 'p' ):
          switch ( letter_1 )
          {
            case ( 'p' ):  // pa - plot all filtered
              switch ( i_value )
              {
                case 0 ... 6:
                  plot_num = i_value;
                  plotting_all = true;
                  monitoring = false;
                  break;
                default:
                  Serial.println("plot number unknown enter plot number e.g. pa0 (sum), pa1 (acc), pa2 (rot), or pa3 (all)");
                  plotting_all = false;
                  break;
              }
              break;
            case ( 'h' ):  // ph - print history
              Serial.println("History:");
              L->print_ram();
              break;
            case ( 'r' ):  // pr - print registers
              Serial.println("Registers:");
              L->print_all_registers();
              break;
            default:
              Serial.print(input_str.charAt(1)); Serial.print(" for "); Serial.print(input_str); Serial.println(" unknown");
              break;
          }
          break;
        case ( 'm' ):  // m  - print all
          plotting_all = false;
          monitoring = !monitoring;
          break;
        case ( 'h' ):  // h  - help
          plotting_all = false;
          monitoring = false;
          Serial.println("h - this help");
          Serial.println("HELP");
          Serial.println("ppX - plot all version X");
          Serial.println("\t X=blank - stop plotting");
          Serial.println("\t X=0 - summary (g_raw, g_filt, g_quiet, q_is_quiet_sure, o_raw, o_filt, o_quiet, o_is_quiet_sure)");
          Serial.println("\t X=1 - g sensors (T_acc, x_filt, y_filt, z_filt, g_filt, g_is_quiet, g_is_quiet_sure)");
          Serial.println("\t X=2 - rotational sensors (T_rot, a_filt, b_filt, c_filt, o_filt, o_is_quiet, o_is_quiet_sure)");
          Serial.println("\t X=3 - all sensors (x_filt, y_filt, z_filt, g_filt, a_filt, b_filt, c_filt, o_filt)");
          Serial.println("\t X=4 - quiet results ( T_rot, o_filt, o_quiet, o_is_quiet_sure, T_acc, g_filt, g_quiet, g_is_quiet_sure)");
          Serial.println("\t X=5 - quiet filtering metrics (o_quiet, g_quiet)");
          Serial.println("\t X=6 - total (T_rot, o_filt, T_acc, g_filt)");
          Serial.println("ph - print history");
          Serial.println("pr - print registers");
          Serial.println("m  - print all");
          break;
        case ( 'U' ):
          switch ( letter_1 )
          {
            case ( 'T' ):
              input_str.substring(input_str, 2).toInt(i_value);
              time_initial = time_t ( i_value );
              setTime(time_initial);
              prn_buff = "---";
              time_long_2_str(time_initial*1000, prn_buff);
              Serial.println("Time set to: "); Serial.print(time_initial); Serial.print(" = "); Serial.println(prn_buff);
              break;
            default:
              Serial.print(input_str.charAt(1)); Serial.println(" unknown");
              break;
          }
          break;
        case ( 'v' ):
          switch ( letter_1 )
          {
            case ( 'v' ):
              debug = i_value;
            break;
            default:
              Serial.print(letter_1); Serial.println(" unknown");
              break;
          }
          break;
        default:
          Serial.print(letter_0); Serial.println(" unknown");
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
  Serial.println("Set time using command 'UTxxxxxxx' where 'xxxxxx' is integer from https://www.epochconverter.com/");
  Serial.println("Check time using command 'vv9;vv0;");
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

// Time synchro so printed decimal times align with hms rolls
void sync_time(unsigned long long *last_sync, unsigned long long *millis_flip)
{
  *last_sync = millis();

  // Refresh millis() at turn of Time.now
  int count = 0;
  long time_begin = now();  // Seconds since start of epoch
  while ( now()==time_begin && ++count<1100 )  // Time.now() truncates to seconds
  {
    delay(1);
    *millis_flip = millis()%1000;
  }
}
