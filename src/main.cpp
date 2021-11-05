#include "Adafruit_BMP3XX.h"
#include "Adafruit_VEML7700.h"
#include "ClosedCube_HDC1080.h"
#include "TTN_CayenneLPP.h"
#include "secrets.h"
#include "ttn.h"
#include "utils.h"
#include <Arduino.h>
#include <SimpleTimer.h>
#include <TTN_esp32.h>
#include <WiFi.h>

#include <kwBatteryMonitor.h>

// #define DEBUG
#define SAMPLE_DURATIONS_SECS        60
#define WAKEUP_TIME_SECS             6
#define TEMPERATURE_OVERREAD_BMP388  -0.5
#define TEMPERATURE_OVERREAD_HDC1080 0.9
#define BMP388_CHANNEL               1
#define VEML7700_CHANNEL             2
#define HDC1080_CHANNEL              3
#define BATTERY_CHANNEL              4
#define VIRTUAL_CHANNEL              5
#define DATA_PIN                     13
#define CLOCK_PIN                    12
#define PIN_SDA                      4
#define PIN_SCL                      15

Adafruit_BMP3XX    bmp388;
Adafruit_VEML7700  veml7700 = Adafruit_VEML7700();
ClosedCube_HDC1080 hdc1080;
TTN_esp32          ttn;
TTN_CayenneLPP     lpp;
SimpleTimer        timer;

uint16_t sleepDurationSeconds = 0;
bool     deepSleepMode = true;
bool     hasBMP388 = false;
bool     hasVEML7700 = false;
bool     hasHDC1080 = false;

void publishEvent()
{
  float temperature_bmp388 = 0;
  float pressure_bmp388 = 0;
  float temperature_hdc1080 = 0;
  float humidity_hdc1080 = 0;
  float lux_veml7700 = 0;
  float temperature_virtual = 0;
  float dewpoint_virtual = 0;
  float voltage_battery = 0;

  lpp.reset();

  // Prime the pressure sensor after deep sleep or else it under-reads
  if ( deepSleepMode )
  {
    for ( int i = 0; i < 2; i++ )
    {
      bmp388.performReading();
      delay( 100 );
    }
  }

  // BMP388_CHANNEL

  if ( hasBMP388 && bmp388.performReading() )
  {
    temperature_bmp388 = bmp388.temperature - TEMPERATURE_OVERREAD_BMP388;
    pressure_bmp388 = bmp388.pressure / 100.0;

    lpp.addTemperature( BMP388_CHANNEL, temperature_bmp388 );
    lpp.addBarometricPressure( BMP388_CHANNEL, pressure_bmp388 );
  }

  // VEML7700_CHANNEL

  if ( hasVEML7700 )
  {
    lux_veml7700 = veml7700.readLuxNormalized();
    lpp.addLuminosity( VEML7700_CHANNEL, lux_veml7700 );
  }

  // HDC1080_CHANNEL

  if ( hasHDC1080 )
  {
    temperature_hdc1080 =
        hdc1080.readTemperature() - TEMPERATURE_OVERREAD_HDC1080;
    humidity_hdc1080 = hdc1080.readHumidity();

    lpp.addTemperature( HDC1080_CHANNEL, temperature_hdc1080 );
    lpp.addRelativeHumidity( HDC1080_CHANNEL, humidity_hdc1080 );
  }

  // BATTERY_CHANNEL

  voltage_battery = Sample() / 1000.0;

  lpp.addAnalogOutput( BATTERY_CHANNEL, voltage_battery );

  // VIRTUAL_CHANNEL

  if ( hasHDC1080 and hasBMP388 )
  {
    temperature_virtual = ( temperature_hdc1080 + temperature_bmp388 ) / 2;
  } else if ( hasHDC1080 )
  {
    temperature_virtual = temperature_hdc1080;
  } else if ( hasBMP388 )
  {
    temperature_virtual = temperature_bmp388;
  } else
  {
    temperature_virtual = -99.0;
  }
  dewpoint_virtual = dewpoint( temperature_virtual, humidity_hdc1080 );

  lpp.addTemperature( VIRTUAL_CHANNEL, dewpoint_virtual );

  ttn.sendBytes( lpp.getBuffer(), lpp.getSize(), 1, true );

#ifdef DEBUG
  Serial.printf(
      "T(HDC1080): %.1f, T(BMP388): %.1f, T(VIRTUAL): %.1f, H(HDC1080): %.1f, "
      "DP( VIRTUAL ): %.1f, LUX: %.1f, VBatt: %.1f\n",
      temperature_hdc1080, temperature_bmp388, temperature_virtual,
      humidity_hdc1080, dewpoint_virtual, lux_veml7700, voltage_battery );
#endif

  if ( deepSleepMode )
  {
    delay( 1000 );  // Let the bytes transmit
    ESP.deepSleep( (sleepDurationSeconds)*1000000L );
  }
}

void setup()
{
#ifdef DEBUG
  Serial.begin( 115200 );
#endif

  Wire.begin( PIN_SDA, PIN_SCL );
  Wire.setClock( 400000L );

#ifdef DEBUG
  char deviceID[16] = { 0 };
  getMacAddress( deviceID );
  Serial.println( "Environment monitor" );
  Serial.printf( "Device ID: %s\n", deviceID );
#endif

  // BMP388 - Pressure / Temperature

  if ( bmp388.begin_I2C() )
  {
    hasBMP388 = true;
    bmp388.setTemperatureOversampling( BMP3_OVERSAMPLING_8X );
    bmp388.setPressureOversampling( BMP3_OVERSAMPLING_4X );
    bmp388.setIIRFilterCoeff( BMP3_IIR_FILTER_COEFF_3 );
    bmp388.setOutputDataRate( BMP3_ODR_50_HZ );
  } else
  {
#ifdef DEBUG
    Serial.println( "Couldn't find BMP388" );
#endif
  }

  // VEML - Lux

  if ( veml7700.begin() )
  {
    hasVEML7700 = true;
    veml7700.setGain( VEML7700_GAIN_1_8 );
    veml7700.setIntegrationTime( VEML7700_IT_25MS );
    veml7700.powerSaveEnable( true );
    veml7700.setPowerSaveMode( VEML7700_POWERSAVE_MODE4 );
    veml7700.setLowThreshold( 10000 );
    veml7700.setHighThreshold( 20000 );
    veml7700.interruptEnable( true );
  } else
  {
#ifdef DEBUG
    Serial.println( "Couldn't find VEML7700" );
#endif
  }

  // HDC1080 - Temperature / Humidity

  hdc1080.begin( 0x40 );
  if ( ( hdc1080.readDeviceId() == 0x1050 ) )
  {
    hasHDC1080 = true;
    hdc1080.heatUp( 2 );
  } else
  {
#ifdef DEBUG
    Serial.println( "Couldn't find HDC1080" );
#endif
  }

  // Battery

  initialiseBatteryMonitor();

  sleepDurationSeconds = ( SAMPLE_DURATIONS_SECS >= WAKEUP_TIME_SECS )
                             ? ( SAMPLE_DURATIONS_SECS - WAKEUP_TIME_SECS )
                             : 0;

  // Connect to the network

  ttn.begin();
  ttn.onMessage( message );  // Declare callback function for handling downlink
  ttn.join( devEui, appEui, appKey, -1, 1000 );
  while ( !ttn.isJoined() ) { ; }

#ifdef DEBUG
  ttn.showStatus();
#endif

  // Go

  timer.setInterval( SAMPLE_DURATIONS_SECS * 1000L, publishEvent );
}

void loop() { timer.run(); }