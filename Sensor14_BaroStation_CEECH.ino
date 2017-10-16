 /**
 *     PROJECT: MySensors / Small battery sensor low power 8 mhz
 *     PROGRAMMER: Jumping
 *     DATE: october 10, 2016/ last update: october 16, 2017
 *     FILE: Sensor14_BaroStation_CEECH.ino.ino
 *     LICENSE: Public domain
 *    
 *     Hardware: ATMega328p board w/ NRF24l01
 *        and MySensors 2.0
 *        CEECH BOARD
 *            
 *    Special:
 *        program with Arduino Pro 3.3V 8Mhz!!!
 *        
 *    Summary:
 *        low power (battery)
 *        BMP 280 + HTU21D
 *        voltage meter for battery and solar
 *    
 *    Remarks:
 *
*******************************
 *
 * REVISION HISTORY
 * Version 1.0 - idefix
 * 
 * DESCRIPTION
 * Arduino BH1750FVI Light sensor
 * communicate using I2C Protocol
 * this library enable 2 slave device addresses
 * Main address  0x23
 * secondary address 0x5C
 * connect the sensor as follows :
 *
 *   VCC  >>> 5V
 *   Gnd  >>> Gnd
 *   ADDR >>> NC or GND  
 *   SCL  >>> A5
 *   SDA  >>> A4
 * http://www.mysensors.org/build/light
 * 
 * USING CEECH BOARD
 */

// Enable debug prints to serial monitor
#define MY_DEBUG 
// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// change NRF24 pin
#define MY_RF24_CE_PIN 7
#define MY_RF24_CS_PIN 8
// define NRF24 channel
#define MY_RF24_CHANNEL  76
//
// NODE ID
#define MY_NODE_ID 14
#define NODE_TXT "WEATHER_14"               // Text to add to sensor name
//
#include <MySensors.h>  
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_HTU21DF.h>
//-------------------------------------------------------------------
#define LTC4079_CHRG_PIN            A7      //analog input A7 on ATmega 328 is /CHRG signal from LTC4079
#define batteryVoltage_PIN          A0      //analog input A0 on ATmega328 is battery voltage
//-------------------------------------------------------------------
float R1 = 47000.0;
float R2 = 10000.0;
// Reference values for ADC and Battery measurements
const float VccMin          = 1.0*2.8 ;             // Minimum expected Vcc level, in Volts.
const float VccMax          = 1.0*4.3 ;             // Maximum expected Vcc level, in Volts. 
//const float VccMin        = 2.0*0.6 ;             // Minimum expected Vcc level, in Volts. for 2xAA Alkaline.
//const float VccMax        = 2.0*1.5 ;             // Maximum expected Vcc level, in Volts.for 2xAA Alkaline.
const float VccCorrection   = 3.32/3.3 ;            // Measured Vcc by multimeter divided by reported Vcc
float VccReference = 4.3 ;                          // voltage reference for measurement, definitive init in setup
//-------------------------------------------------------------------
#define TEMP_CHILD_ID           1
#define PRESS_CHILD_ID          2
#define HUM_CHILD_ID            3
#define BATT_CHILD_ID           7
//-------------------------------------------------------------------
#define P0 1013.25 // sea level pressure for my place
#define MY_ALTITUDE 340 // Via Michelangelo Cappuccini
//-------------------------------------------------------------------
const char *weather[] = { "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown" };
enum FORECAST
{
  STABLE = 0,           // "Stable Weather Pattern"
  SUNNY = 1,            // "Slowly rising Good Weather", "Clear/Sunny "
  CLOUDY = 2,           // "Slowly falling L-Pressure ", "Cloudy/Rain "
  UNSTABLE = 3,         // "Quickly rising H-Press",     "Not Stable"
  THUNDERSTORM = 4,     // "Quickly falling L-Press",    "Thunderstorm"
  UNKNOWN = 5           // "Unknown (More Time needed)
};
//-------------------------------------------------------------------
  const float Threshold = 0.1 ;                    // send only if change > treshold
  const int heartbeat = 12 ;                      // heartbeat every hour (x times SLEEP_TIME)
  unsigned long lastHeartbeat = 0 ;
  const unsigned long SLEEP_TIME = 300000UL;      // Sleep time between reads (in milliseconds)
//-------------------------------------------------------------------
  float lastPressure = -1;
  float lastTemperature = -1;
  float lastHumidity = -1 ;
  String lastForecast;
  float lastBattVoltage = -1;
  int lastBattPct = 0;
  // flags to indicate if transmission is needed, heartbeat and/or changes > treshold
  boolean txTemperature = true ;  
  boolean txHumidity = true ;
  boolean txPressure = true ;
  boolean txForecast = true ;
  boolean txBattVoltage = true ;
//-------------------------------------------------------------------
  const int LAST_SAMPLES_COUNT = 5;
  float lastPressureSamples[LAST_SAMPLES_COUNT];
  #define CONVERSION_FACTOR (1.0/10.0) // this CONVERSION_FACTOR is used to convert from Pa to kPa in forecast algorithm get kPa/h be dividing hPa by 10
  int minuteCount = 0;
  bool firstRound = true;                                   // average value is used in forecast algorithm.
  float pressureAvg;                                        // average after 2 hours is used as reference value for the next iteration.
  float pressureAvg2;
  float dP_dt;
//-------------------------------------------------------------------
Adafruit_BMP280 bmp = Adafruit_BMP280();                    // Digital Pressure Sensor
Adafruit_HTU21DF SHT21; 
//-------------------------------------------------------------------
MyMessage tempMsg(TEMP_CHILD_ID, V_TEMP);                   // BMP 280
MyMessage pressureMsg(PRESS_CHILD_ID, V_PRESSURE);          // BMP 280
MyMessage forecastMsg(PRESS_CHILD_ID, V_FORECAST);          // BMP 280
MyMessage humidityMsg(HUM_CHILD_ID, V_HUM);                 // HTU21D
//--------------------------------------------------------------------------------
MyMessage batteryVoltageMsg(BATT_CHILD_ID, V_VOLTAGE);      // Battery voltage (V)
MyMessage batteryCurrentMsg(BATT_CHILD_ID, V_CURRENT);      // Battery current (A)
//--------------------------------------------------------------------------------
void setup()  
{ 
  Serial.begin(9600);
  Wire.begin();                                 // START I2C  
  if (!bmp.begin(0x76))                         // BMP SENSOR
  {
    Serial.println("BMP init failed!");
    while (1);
  }
  else Serial.println("BMP init success!");
  SHT21.begin();                                // initialize temp/hum 
}

void presentation()  {
    // Send the sketch version information to the gateway and Controller
    sendSketchInfo("JMP "NODE_TXT, "1.0");
    wait(200);
    // Register all sensors to gateway (they will be created as child devices)
    present(BATT_CHILD_ID, S_POWER, "Batt " NODE_TXT);        // Battery parameters
    wait(200);
    present(TEMP_CHILD_ID, S_TEMP, "Temp " NODE_TXT);         // Temp sensor
    wait(200);
    present(HUM_CHILD_ID, S_HUM, "Hum" NODE_TXT);
    wait(200); 
    present(PRESS_CHILD_ID, S_BARO, "Press " NODE_TXT);       // pressure sensor
    wait(200);
}
//-------------------------------------------------------------------
void loop(){
  Serial.println();
  readTempHum();
  readBaro();
  readVoltages();
  sendSensors();
  //-------------------------
  sleep(SLEEP_TIME);
}
//-------------------------------------------------------------------
void readTempHum(void){
    // SHT2x sensor
    // Temperature and Humidity
    float hum = SHT21.readHumidity();  
    float temp = SHT21.readTemperature();
     
    if ( abs(hum  - lastHumidity) >= Threshold ){   // update only if threshold exceeded
        lastHumidity = hum ;
        txHumidity = true ;
        } 
    if ( abs(temp - lastTemperature) >= Threshold ){ 
        lastTemperature = temp ;
        txTemperature = true ;
        } 
    #ifdef MY_DEBUG
     Serial.print("T: ");
     Serial.println(temp);
     Serial.print("H: ");
     Serial.println(hum);
    #endif
}
//-------------------------------------------------------------------
void readBaro() // used to read sensor data and send it to controler
{
  double T, P, P_raw, A;
  T = bmp.readTemperature();
  P_raw = bmp.readPressure()/100;
  //float pressure = (float)bmp.readSealevelPressure(MY_ALTITUDE)/100;
  P = P_raw/pow((1.0 - ( MY_ALTITUDE/44330.0 )), 5.255);
  A = bmp.readAltitude(P0);
  int forecast = sample(P);
  
  if ( abs(P  - lastPressure) >= Threshold ){   // update only if threshold exceeded
        lastPressure = P ;
        txPressure = true ;
        lastForecast = weather[forecast];
        txForecast = true;
  }  
  #ifdef MY_DEBUG
      Serial.print(F("BMP280: T= "));
      Serial.print(T);
      Serial.print(F(" P_raw = "));
      Serial.print(P_raw);
      Serial.print(F(" P = "));
      Serial.print(P);
      Serial.print(F(" A = "));
      Serial.print(A);
      Serial.print(F(" previsione --> "));
      Serial.println(weather[forecast]);
  #endif 
}
//-------------------------------------------------------------------
void readVoltages(void)
{ 
   float voltage = readVcc();                                                         // actual VOLTAGE VCC
   #ifdef MY_DEBUG
     Serial.print("Vcc ");
     Serial.print(voltage);
     Serial.println("V ");
   #endif

   //------------------
   float batteryVoltage = ((float)analogRead(batteryVoltage_PIN) * voltage/1024)* 2;  // VOLTAGE battery
   if (abs(batteryVoltage  - lastBattVoltage) >= Threshold) {
      lastBattVoltage = batteryVoltage;
      txBattVoltage= true;
   } 
   #ifdef MY_DEBUG
     Serial.print("Battery voltage ");
     Serial.print(batteryVoltage);
     Serial.println("V ");
   #endif
   //------------------
   int charge = (int)analogRead(LTC4079_CHRG_PIN);
   #ifdef MY_DEBUG
     Serial.print("CHRG ");
     Serial.println(charge);
   #endif
}
//-------------------------------------------------------------------
void sendSensors(void)
{
    lastHeartbeat++ ;                                       // update Heartbeatcount every call
    if ( lastHeartbeat > heartbeat) {                       // if heartbeat update all sensors & battery status
        txTemperature = txHumidity = txBattVoltage = txPressure = txForecast = true ;
        int batteryPcnt = 100.0*(lastBattVoltage - VccMin)/(VccMax - VccMin); // Convert voltage to percentage
        //int batteryPcnt = static_cast<int>(((lastVoltage-VccMin)/(VccMax - VccMin))*100.);
        sendBatteryLevel(batteryPcnt, true);
        lastHeartbeat = 0;
        wait(200);
        }
    if (txTemperature){
        send(tempMsg.set(lastTemperature, 2));                  // Send in deg C
        txTemperature = false ;
        wait(200);
        }
    if (txHumidity){
        send(humidityMsg.set(lastHumidity, 2));                 // Send in %RH
        txHumidity = false;
        wait(200);
        }
    if (txPressure){
        send(pressureMsg.set(lastPressure, 2));                 // Send in milliBar
        txPressure = false ;
        wait(200);
        }
    if (txForecast){
        send(forecastMsg.set(lastForecast));                    // Send string
        txForecast = false ;
        wait(200);
        }
    if (txBattVoltage){
        send(batteryVoltageMsg.set(lastBattVoltage, 2));        // Send battery V
        txBattVoltage = false;
        wait(200);
        }
}
//-------------------------------------------------------------------
float readVcc() 
{
  signed long resultVcc;
  float resultVccFloat;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(10);                           // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                 // Convert
  while (bit_is_set(ADCSRA,ADSC));
  resultVcc = ADCL;
  resultVcc |= ADCH<<8;
  resultVcc = 1126400L / resultVcc;    // Back-calculate AVcc in mV
  resultVccFloat = (float) resultVcc / 1000.0; // Convert to Float
  return resultVccFloat;
}
//----------------------------------------------------------------/
float getLastPressureSamplesAverage()
{
  float lastPressureSamplesAverage = 0;
  for (int i = 0; i < LAST_SAMPLES_COUNT; i++)
  {
    lastPressureSamplesAverage += lastPressureSamples[i];
  }
  lastPressureSamplesAverage /= LAST_SAMPLES_COUNT;

  return lastPressureSamplesAverage;
}

int sample(float pressure)
{
  // Calculate the average of the last n minutes.
  int index = minuteCount % LAST_SAMPLES_COUNT;
  lastPressureSamples[index] = pressure;

  minuteCount++;
  if (minuteCount > 185)
  {
    minuteCount = 6;
  }

  if (minuteCount == 5)
  {
    pressureAvg = getLastPressureSamplesAverage();
  }
  else if (minuteCount == 35)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change * 2; // note this is for t = 0.5hour
    }
    else
    {
      dP_dt = change / 1.5; // divide by 1.5 as this is the difference in time from 0 value.
    }
  }
  else if (minuteCount == 65)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) //first time initial 3 hour
    {
      dP_dt = change; //note this is for t = 1 hour
    }
    else
    {
      dP_dt = change / 2; //divide by 2 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 95)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 1.5; // note this is for t = 1.5 hour
    }
    else
    {
      dP_dt = change / 2.5; // divide by 2.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 125)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    pressureAvg2 = lastPressureAvg; // store for later use.
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2; // note this is for t = 2 hour
    }
    else
    {
      dP_dt = change / 3; // divide by 3 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 155)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 2.5; // note this is for t = 2.5 hour
    }
    else
    {
      dP_dt = change / 3.5; // divide by 3.5 as this is the difference in time from 0 value
    }
  }
  else if (minuteCount == 185)
  {
    float lastPressureAvg = getLastPressureSamplesAverage();
    float change = (lastPressureAvg - pressureAvg) * CONVERSION_FACTOR;
    if (firstRound) // first time initial 3 hour
    {
      dP_dt = change / 3; // note this is for t = 3 hour
    }
    else
    {
      dP_dt = change / 4; // divide by 4 as this is the difference in time from 0 value
    }
    pressureAvg = pressureAvg2; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
    firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
  }

  int forecast = UNKNOWN;
  if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
  {
    forecast = UNKNOWN;
  }
  else if (dP_dt < (-0.25))
  {
    forecast = THUNDERSTORM;
  }
  else if (dP_dt > 0.25)
  {
    forecast = UNSTABLE;
  }
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
  {
    forecast = CLOUDY;
  }
  else if ((dP_dt > 0.05) && (dP_dt < 0.25))
  {
    forecast = SUNNY;
  }
  else if ((dP_dt > (-0.05)) && (dP_dt < 0.05))
  {
    forecast = STABLE;
  }
  else
  {
    forecast = UNKNOWN;
  }
  #ifdef MY_DEBUG
    Serial.print(F("Forecast at minute "));
    Serial.print(minuteCount);
    Serial.print(F(" dP/dt = "));
    Serial.print(dP_dt);
    Serial.print(F("kPa/h --> "));
    Serial.println(weather[forecast]);
  #endif
  
  return forecast;
}
/* Ceech board specifics for reference:
voltmeters on both battery and solar cell connections
They are connected to analog inputs A0 and A2 on the ATmega328p. The voltage dividers resistors are equal, so the measured voltage is double the shown voltage.

NRF24l01+ socket
with CE and CSN pins connected to digital pins 7 and 8 ( you use RF24 radio(7, 8); in Arduino code). There is a 4.7uF capacitor connected across Vin and GND of the port
*/
