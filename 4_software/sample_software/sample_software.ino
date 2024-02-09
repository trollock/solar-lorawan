/* LoRa GPS Receiver (needs a better name)
rev 0.1 J R Shipley 26/12/2020 
The state machine contains 4 basic parts - 1) sleep mode (slp_mode), 2) gps mode (gps_mode), 3) transmit mode (tx_mode) and 
receive mode (rx_mode).
We will use a set of different basic counters to determine how long it should attempt to obtain a gps fix 
In the future, it would be ideal component to implement a simplified version of a PID controller to determine the 
trajectory of the battery charge state, to make basic predictions on future charge states, and the amount of time
that should be dedicated to getting a fix.

Hardware changes 
rev 0.1 - changed the ZOE-M8Q RAM backup to use a dedicated I/O on the CMWX module, so we can turn off 
battery RAM backup if the ephemeris has expired.  This would reduce the power consumption from 18 to 2.2 uA when it
has been greater than 4 hours since the last ephemeris update.

rev 0.2 - scrapped the UBlox related code, switch to use Quectel L70 to save on power and $$$, active searching current is 14.1 mA on the demo module, sleep mode
battery backup RAM is only 12.9 uA. 

rev 0.3 - added the CAT24 EEPROM to write all data that isn't immediately able to be sent, then checks later if gateway is within range and sends historical data  

rev 1.0 - added the LG77L GPS module, switched GPS REST pin from 3 to 2, don't measure lithium poly under load with LC709203!
*/ 
#include <TinyGPS++.h>
#include "STM32L0.h"
#include "LoRaWAN.h"
#include <Wire.h>
#include <extEEPROM.h>    //https://github.com/PaoloP74/extEEPROM
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include "SparkFun_BMP581_Arduino_Library.h"

TinyGPSPlus gps;

SFE_MAX1704X lipo(MAX1704X_MAX17048); //create a gas gauge object

BMP581 pressureSensor; // Create a pressure sensor object

//!!! This is different on the demo board, it is the DEFAULT and not the SECONDARY address
//uint8_t i2cAddress = BMP581_I2C_ADDRESS_DEFAULT; // 0x47
uint8_t i2cAddress = BMP581_I2C_ADDRESS_SECONDARY; // 0x46

//LoRaWAN provisioning data
//---------------------------------------------------------------------------------------------------------
const char *appEui = "0000000000000000";

//test board data
const char *appKey = "8836AAE8CBDF34101AC94EAFA8F94575"; 
const char *devEui = "70B3D57ED0063AB7"; 

//defines for the serial - Quectel interface - GPS related globals
//---------------------------------------------------------------------------------------------------------
#define GPSTX 1 //pin number for GPS TX output - data from Arduino to GPS
#define GPSRX 0 //pin number for GPS RX input - from GPS to Arduino 
static const uint32_t GPSBaud = 9600;

// board definitions for the power and reset for the GPS module
#define GPS_PIN A2  // pin 24
#define GPS_RST 2 // pin 39

//some basic state machine related vars
enum State_enum {SLEEP_MODE, STARTUP_MODE, GPS_MODE, TX_RX_MODE, HIST_MODE}; 
uint8_t state = STARTUP_MODE; 

//time keeping related, including sleep timers and GPS timeouts
unsigned long startTime = 0;
unsigned int slp_time = 60000 * 240;
unsigned int hist_time = 8000;
unsigned int gps_timeout = 60000 * 3; //5 minutes = 300,000 milliseconds, demo 30 seconds

uint8_t payload[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint8_t   Status = 0;           // 1 bit
uint8_t   Month = 12;           // 4 bits
uint8_t   Day = 31;             // 5 bits 
uint8_t   Year = 99;            // 7 bits
uint8_t   Hour = 23;            // 5 bits
uint8_t   Minute = 59;          // 6 bits
uint8_t   Second = 59;          // 6 bits but reduced to value between 1 and 12 (5 second accuracy) i.e. 4 bits
int32_t   Latitude = 46833722;  // 3 bytes 
int32_t   Longitude = 8183062;  // 3 bytes
uint8_t   Hdop = 124;           // 1 byte
uint16_t  Volts = 173;          // 2 bytes
uint32_t  Pressure = 0;         // 24 bits
uint16_t  Temp = 402;           // 8 bits

uint8_t Temp_tx = 255;

int32_t   prev_Longitude = 0;
uint8_t   Siv = 0;
uint8_t   Siv_thresh = 4;

bool gps_fix = false;
bool has_data = false;
bool in_range = false;
bool has_join = false;
bool sleep_flag = false;

uint8_t rem_data = 0;

//voltage measurement 
//---------------------------------------------------------------------------------------------------------
#define VOLTS_PIN A3  // pin 24

uint16_t sensorValue = 0;
float voltage = 0;
uint16_t volts = 0;
uint16_t old_volts = 0;

//some globals for the CAT24 EEPROM module
//---------------------------------------------------------------------------------------------------------
//One 24LC256 EEPROMs on the bus
const uint32_t totalKBytes = 32;         //for read and write test functions
extEEPROM eep(kbits_512, 1, 128, 0x50);         //device size, number of devices, page size, i2c address

// variables stored on the EEPROM
uint8_t  has_data_address = 0;
uint16_t last_save_address = 1;
uint16_t last_sent_address = 3;
uint8_t init_start_address = 5;

//start eeprom storage on 6
// variables not stored on EEPROM
uint8_t save_offset = 6;
uint16_t curr_sent = 0;
uint16_t sent_counter = 5;
uint16_t max_counter = 0;
uint16_t max_send = 5;
uint16_t last_save = 0;
uint16_t last_save_idx = 0;
uint16_t last_sent = 0;
uint16_t last_sent_idx = 0;

//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------

void setup(){ 
         uint8_t eepStatus = eep.begin(eep.twiClock400kHz);   //go fast!
          if (eepStatus) 
          {
            while (1);
          }

          //for (int i = 1; i < 1000; i++){
          //  eep.write(i, 1);
          //}

         //initialize the EEPROM for after first startup post programming
          uint8_t init_start = eep.read(init_start_address);
          if (init_start != 0) 
          {
            writeIntEEPROM(last_save_address, 0);      
            writeIntEEPROM(last_sent_address, 0);
            //write a zero because the eeprom starts with FEFE or FFFF, a zero means it's not the first time it started
            eep.write(init_start_address, 0);
            init_lora();
          }
          STM32L0.stop(5000);
}

void loop() {
  switch (state) {
       case STARTUP_MODE:
          Wire.begin();
          eep.begin(eep.twiClock400kHz);
          lipo.begin();
          init_pins();
          batt_check();
          sleep_flag = false;
          duty_cycle();
          
         if (sleep_flag == true)
         {
          state = SLEEP_MODE;
         }
         else 
         {
          if (!LoRaWAN.joined())
          {
          init_lora(); 
          } 
          init_gps();
          startTime = millis();
          state = GPS_MODE;
         }
          break; 
    
       case GPS_MODE:
          if (millis() - startTime <= gps_timeout && gps_fix == false)
          {
            smartDelay(1000);
            
            //time related variables from the GPS module fix
            Status  = 0;
            Month   = gps.date.month();
            Day     = gps.date.day();
            Year    = (uint16_t) gps.date.year() - 2000;
            Hour    = gps.time.hour();
            Minute  = gps.time.minute();
            Second  = gps.time.second();

            //Spatial variables of the GPS module fix
            Latitude = (float) gps.location.lat() * 10000000;
            Longitude = (float) gps.location.lng() * 10000000;
            Hdop = (float) (gps.hdop.hdop() * 10);
            Siv = gps.satellites.value();

              if (Siv >= Siv_thresh && Longitude != prev_Longitude)
              {
                gps_fix = true;
                digitalWrite(GPS_PIN, LOW);
                prev_Longitude = Longitude;
                state=TX_RX_MODE;
              }  
              else 
              {
              gps_fix = false;
              }    
          } 
          else 
          {
            digitalWrite(GPS_PIN, LOW);
            state = TX_RX_MODE;
          } 
          break;
    
      case TX_RX_MODE:
          delay(50);
          stored_data(); //this provides has_data, last_sent, and last_save updates 
          pressure_check();
          if (LoRaWAN.joined() && !LoRaWAN.busy())
          {
            create_Packet(payload);
            LoRaWAN.beginPacket();
            LoRaWAN.write(payload, sizeof(payload));  
            LoRaWAN.endPacket(true); 
              while (LoRaWAN.busy())
              {
              STM32L0.sleep(1000);
              }   
              if (LoRaWAN.confirmed())
              {
                in_range = true;
              }
              else  //goes here after first handshake fail
              {   
                in_range = false;
                save_Packet(payload);
              }
          }
          if (has_data && in_range)
          {
            state = HIST_MODE;  
          }
          else 
          {
            state = SLEEP_MODE;
          }
          break;  

       case HIST_MODE:
         if (LoRaWAN.joined() && !LoRaWAN.busy() && has_data)
            {
              for (int i = 0; i < sent_counter; i++)
              {
                Wire.begin();
                eep.begin(eep.twiClock400kHz);
                read_Packet(payload);
                  LoRaWAN.beginPacket();
                  LoRaWAN.write(payload, sizeof(payload));  
                  LoRaWAN.endPacket(false);
                STM32L0.stop(hist_time);
              }
            }
          state = SLEEP_MODE;
          break;   

      case SLEEP_MODE:
          init_sleep();
          STM32L0.stop(slp_time);
          state = STARTUP_MODE;
          break; 
  }
}

void init_pins (void) {  
      Serial1.begin(GPSBaud);
      //pinMode(A0, INPUT);
      pinMode(GPS_PIN, OUTPUT);
        digitalWrite(GPS_PIN, LOW);
      pinMode(GPS_RST, OUTPUT);
        digitalWrite(GPS_RST, LOW);
}

void init_lora (void) {
    LoRaWAN.begin(EU868);
    LoRaWAN.setDutyCycle(false);
    LoRaWAN.setADR(false);
    LoRaWAN.setDataRate(0);
    LoRaWAN.setTxPower(17);
    LoRaWAN.setAntennaGain(0.5);
    LoRaWAN.setReceiveDelay(4000);
    LoRaWAN.setJoinRetries(2);
    LoRaWAN.setRetries(2);
    LoRaWAN.joinOTAA(appEui, appKey, devEui);
}

void init_gps (void) {
    digitalWrite(GPS_PIN, HIGH);
    digitalWrite(GPS_RST, HIGH);
      delay(200);
    digitalWrite(GPS_RST, LOW);
      delay(200);
    digitalWrite(GPS_RST, HIGH);
    gps_fix = false; 
    in_range = false;
}

void init_sleep (void) {
          digitalWrite(GPS_PIN, LOW);
          delay(25);
          digitalWrite(GPS_RST, LOW); 
          delay(25);
          Wire.end();
          digitalWrite(SCL, HIGH);
          delay(25);
          digitalWrite(SDA, HIGH);
          delay(25);
          Serial1.end();
          delay(25);
}

uint8_t create_Packet(uint8_t payload [21])
{
  payload[0]  = Status << 7 | Month << 3 | Day >> 2; 
  payload[1]  = Day << 6 | Year >> 1; 
  payload[2]  = Year << 7 | Hour << 2 | Minute >> 4;
  payload[3]  = Minute << 4 | static_cast<int>(floor(Second / 5));
  payload[4]  = Latitude / 100 >> 16; 
  payload[5]  = Latitude / 100 >> 8; 
  payload[6]  = Latitude / 100;
  payload[7]  = Longitude / 100 >> 16;
  payload[8]  = Longitude / 100 >> 8;
  payload[9]  = Longitude / 100;
  payload[10] = Hdop;
  payload[11] = Volts >> 8;
  payload[12] = Volts;
  payload[13] = Pressure >> 16;
  payload[14] = Pressure >> 8;
  payload[15] = Pressure;
  payload[16] = Temp < 510 && Temp > 0 ? Temp / 2 : 255;
  payload[17] = last_save >> 8;
  payload[18] = last_save;
  payload[19] = last_sent >> 8;
  payload[20] = last_sent;
}

void save_Packet(uint8_t payload[17]) {
  last_save = readIntEEPROM(last_save_address);
  last_save_idx = (last_save * 17) + save_offset;

  last_save = last_save + 1; // Increment last_save

  for (int i = 0; i < 17; i++) {
    eep.write(last_save_idx + i, payload[i]);
  }

  writeIntEEPROM(last_save_address, last_save);
  eep.write(has_data_address, 0);
  has_data = true;
}

void read_Packet(uint8_t payload[21]) {
  last_sent = readIntEEPROM(last_sent_address);
  last_sent_idx = (last_sent * 17) + save_offset;

  last_sent = last_sent + 1; // Increment last_sent

  for (int i = 0; i < 17; i++) {
    payload[i] = eep.read(last_sent_idx + i);
  }

  payload[17] = last_save >> 8;
  payload[18] = last_save;
  payload[19] = last_sent >> 8;
  payload[20] = last_sent;

  payload[0] |= (1 << 7); // Set the 7th bit (bit 7) of the first element

  writeIntEEPROM(last_sent_address, last_sent);
}

bool stored_data() {
  last_save = readIntEEPROM(last_save_address);
  last_sent = readIntEEPROM(last_sent_address);
  has_data = eep.read(has_data_address);

  if (last_save > last_sent) {
    rem_data = last_save - last_sent;
    sent_counter = (rem_data < max_send && rem_data != 0) ? rem_data : max_send;
  } else {
    rem_data = sent_counter = last_sent = last_save = 0;
  }

  has_data = (last_save > 0 && last_sent < last_save);
  
  if (!has_data) {
    last_save = last_sent = 0;
  }

  eep.write(has_data_address, has_data);
  writeIntEEPROM(last_save_address, last_save);
  writeIntEEPROM(last_sent_address, last_sent);
}

// basic function to check the battery and filter out erroneous measurements using a voltage change threshold
const uint16_t VOLTS_THRESHOLD = 30000;
const uint16_t VOLTS_UPPER = 44000;
const uint16_t MULTIPLIER = 10000;

void batt_check() {
  old_volts = volts;
  lipo.wake();
  delay(100);
  volts = lipo.getVoltage() * MULTIPLIER;
  delay(25);
  if (volts > VOLTS_UPPER | (old_volts - volts) > VOLTS_THRESHOLD) {
    volts = old_volts;
  }

  Volts = volts;
  // Serial.println(volts);
  lipo.enableHibernate();
}

void duty_cycle (){
    if (volts > 41500) 
    {
      gps_timeout = 60000 * 6;
      slp_time = 60000 * 30;
      Siv_thresh = 5;
      max_counter = 0;
      sleep_flag = false;
    }
    else if (volts > 39500) 
    {
      gps_timeout = 60000 * 4;
      slp_time = 60000 * 180;
      Siv_thresh = 4;
      max_counter = 0;
      sleep_flag = false;
    }
    else if (volts > 36800 && max_counter == 0) 
    {
      gps_timeout = 60000 * 3;
      slp_time = 60000 * 180;
      Siv_thresh = 3;
      max_counter = max_counter + 1;
      sleep_flag = false;
    }
    else if (volts > 36800 && max_counter != 0) 
    {
      gps_timeout = 60000 * 3;
      slp_time = 60000 * 180;
      max_counter = 0;
      Siv_thresh = 3;
      sleep_flag = true;
    }
    else
    {
      gps_timeout = 60000 * 3;
      slp_time = 60000 * 180;
      Siv_thresh = 3;
      sleep_flag = true;
    }
}

void pressure_check ()
{
    pressureSensor.beginI2C(i2cAddress);
    pressureSensor.setMode(BMP5_POWERMODE_FORCED);

    bmp5_osr_odr_press_config osrMultipliers =
    {
        .osr_t = BMP5_OVERSAMPLING_8X,
        .osr_p = BMP5_OVERSAMPLING_16X
    };
    
    pressureSensor.setOSRMultipliers(&osrMultipliers);
    
    delay(100);
    // Get measurements from the sensor
    bmp5_sensor_data data = {0};
    int8_t err = pressureSensor.getSensorData(&data);
    
    // Check whether data was acquired successfully
    if(err == BMP5_OK)
    {
        // Acquisistion succeeded, print temperature and pressure
    //    Serial.print("Temperature (C): ");
        Temp = round(data.temperature * 10);
    //    Serial.print(data.temperature); Serial.print(" : "); Serial.println(Temp);
    //    Serial.print("Pressure (Pa): ");
        Pressure = round(data.pressure * 10);
    //    Serial.print(data.pressure);Serial.print(" : "); Serial.println(Pressure);
    }
    else
    {
        // Acquisition failed, most likely a communication error (code -2)
    //    Serial.print("Error getting data from sensor! Error code: ");
    //    Serial.println(err);
    }
    delay(50);
    pressureSensor.setMode(BMP5_POWERMODE_STANDBY);
}

static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do {
    while (Serial1.available() > 0)
      gps.encode(Serial1.read());
  }
  while (millis() - start < ms);
}

void writeIntEEPROM(int address, int number) {
  eep.write(address, number >> 8);
  eep.write(address + 1, number & 0xFF);
}

int readIntEEPROM(int address) {
  return (eep.read(address) << 8) + eep.read(address + 1);
}
