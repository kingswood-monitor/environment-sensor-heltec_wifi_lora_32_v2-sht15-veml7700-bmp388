#include "TTN_CayenneLPP.h"
#include "secrets.h"
#include "ttn.h"
#include "utils.h"
#include <Arduino.h>
#include <TTN_esp32.h>
#include <WiFi.h>

TTN_esp32      ttn;
TTN_CayenneLPP lpp;

int  SAMPLE_DURATION_S = 10;
char deviceID[16] = { 0 };

void setup()
{
  Serial.begin( 115200 );
  getMacAddress( deviceID );
  Serial.println( deviceID );

  // TheThingsNetwork
  Serial.print( "Joining" );

  ttn.begin();
  ttn.onMessage( message );  // Declare callback function for handling downlink
  ttn.join( devEui, appEui, appKey, -1, 10000 );
  while ( !ttn.isJoined() ) { ; }
  ttn.showStatus();
}

void loop()
{
  // put your main code here, to run repeatedly:
}