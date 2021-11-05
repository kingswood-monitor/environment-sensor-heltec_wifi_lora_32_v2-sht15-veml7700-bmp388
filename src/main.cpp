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

#define SAMPLE_DURATIONS_SECS        1
#define WAKEUP_TIME_SECONDS          4
#define TEMPERATURE_OVERREAD_BMP388  0
#define TEMPERATURE_OVERREAD_HDC1080 0
#define BMP388_CHANNEL               1
#define VEML7700_CHANNEL             2
#define HDC1080_CHANNEL              3
#define BATTERY_CHANNEL              4
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
char     deviceID[16] = { 0 };
bool     deepSleepMode = false;
bool     hasBMP388 = false;
bool     hasVEML7700 = false;
bool     hasHDC1080 = false;

void publishEvent()
{
  lpp.reset();

  if ( hasBMP388 && bmp388.performReading() )
  {
    float temperature = bmp388.temperature - TEMPERATURE_OVERREAD_BMP388;
    float pressure = bmp388.pressure / 100.0;

    lpp.addTemperature( BMP388_CHANNEL, temperature );
    lpp.addBarometricPressure( BMP388_CHANNEL, pressure );

    Serial.printf( "BMP388   - T: %.1f degC, P: %.1f, \n", temperature,
                   pressure );
  }

  if ( hasVEML7700 )
  {
    float lux = veml7700.readLuxNormalized();
    lpp.addLuminosity( VEML7700_CHANNEL, lux );

    Serial.printf( "VEML7700 - Lux: %.1f \n", lux );
  }

  if ( hasHDC1080 )
  {
    float temperature =
        hdc1080.readTemperature() - TEMPERATURE_OVERREAD_HDC1080;
    float humidity = hdc1080.readHumidity();

    lpp.addTemperature( HDC1080_CHANNEL, temperature );
    lpp.addRelativeHumidity( HDC1080_CHANNEL, humidity );

    Serial.printf( "HDC1080  - T: %.1f degC, H: %.1f %%, \n", temperature,
                   humidity );
  }

  // Battery

  float voltage = Sample() / 1000.0;

  lpp.addAnalogOutput( BATTERY_CHANNEL, voltage );
  Serial.printf( "BATTERY  - V: %.3f mV\n", voltage );

  ttn.sendBytes( lpp.getBuffer(), lpp.getSize() );

  if ( deepSleepMode ) { ESP.deepSleep( (sleepDurationSeconds)*1000000L ); }
}

void setup()
{
  Serial.begin( 115200 );

  Wire.begin( PIN_SDA, PIN_SCL );
  Wire.setClock( 400000L );

  getMacAddress( deviceID );
  Serial.println( "Environment monitor" );
  Serial.printf( "Device ID: %s\n", deviceID );

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
    Serial.println( "Couldn't find BMP388" );
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
    Serial.println( "Couldn't find VEML7700" );
  }

  // HDC1080 - Temperature / Humidity

  hdc1080.begin( 0x40 );
  if ( ( hdc1080.readDeviceId() == 0x1050 ) )
  {
    hasHDC1080 = true;
  } else
  {
    Serial.println( "Couldn't find HDC1080" );
  }

  // Battery

  initialiseBatteryMonitor();


  sleepDurationSeconds = ( SAMPLE_DURATIONS_SECS >= WAKEUP_TIME_SECONDS )
                             ? ( SAMPLE_DURATIONS_SECS - WAKEUP_TIME_SECONDS )
                             : 0;

  ttn.begin();
  ttn.onMessage( message );  // Declare callback function for handling downlink
  ttn.join( devEui, appEui, appKey, -1, 1000 );
  while ( !ttn.isJoined() ) { ; }

  timer.setInterval( SAMPLE_DURATIONS_SECS * 1000L, publishEvent );

  // delay( 1000 );
  // Serial.end();
}

void loop() { timer.run(); }