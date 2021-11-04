#include <ttn.h>

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
        // SAMPLE_DURATION_S = duration_s;
        Serial.printf( "\nChanged sample duration to %d ms", duration_s );
      }
    }
  }
  Serial.println();
}