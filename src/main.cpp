#include "TTN_CayenneLPP.h"
#include <Arduino.h>
#include <TTN_esp32.h>
#include <WiFi.h>

// HELTEC WIFI LORA 32 V2 pinout
#define UNUSED_PIN 0xFF
#define SS         18
#define RST_LoRa   14
#define DIO0       26
#define DIO1       35
#define DIO2       34

TTN_esp32      ttn;
TTN_CayenneLPP lpp;

// OTTA
const char* appEui = "0000000000000000";  // Change to TTN Application EUI
const char* devEui = "000010521C5C9438";  // Change to TTN Device EUI
const char* appKey =
    "317002F9C5B5C94305E7CD47A560D592";  // Chaneg to TTN Application Key

int  SAMPLE_DRATION_S = 10;
char deviceID[16] = { 0 };

// Populate the device ID
void getMacAddress()
{
  String theAddress = WiFi.macAddress();
  theAddress.replace( ":", "" );
  strcpy( deviceID, theAddress.c_str() );
}

// Callback function when a TTN message is received
void message( const uint8_t* payload, size_t size, int rssi )
{
  Serial.println( "-- MESSAGE" );
  Serial.print( "Received " + String( size ) + " bytes RSSI=" + String( rssi ) +
                "db" );
  for ( int i = 0; i < size; i++ )
  {
    Serial.print( " " + String( payload[i] ) );
    // Serial.write(payload[i]);
  }

  // COMMAND: CHANGE SAMPLE DURATION
  // Two bytes: (i) 01 (ii) XX specifying sample duration in seconds.
  if ( size == 2 )
  {
    int command = payload[0];
    if ( command == 1 )
    {
      int duration_s = payload[1];
      if ( duration_s > 0 )
      {
        SAMPLE_DRATION_S = duration_s;
        Serial.printf( "\nChanged sample duration to %d ms", duration_s );
      }
    }
  }
  Serial.println();
}

void setup()
{
  Serial.begin( 115200 );
  getMacAddress();
  Serial.println( deviceID );

  // TheThingsNetwork
  ttn.begin();
  ttn.onMessage( message );  // Declare callback function for handling downlink
  Serial.print( "Joining" );
  // ttn.join();                // Activate the device via OTAA (default)
  ttn.join( devEui, appEui, appKey, -1, 10000 );
  while ( !ttn.isJoined() ) { Serial.print( "." ); }
  ttn.showStatus();
}

void loop()
{
  // put your main code here, to run repeatedly:
}