#include "utils.h"

// Populate the device ID
void getMacAddress( char* deviceID )
{
  String theAddress = WiFi.macAddress();
  theAddress.replace( ":", "" );
  strcpy( deviceID, theAddress.c_str() );
}

// See:
// https://calcunation.com/calculator/dew-point.php#:~:text=What%20is%20the%20Magnus%20Dew%20Point%20Formula%3F%20To,is%3A%20Approximate%20Dew%20Point%20%3D%2047.90%20Degrees%20Celsius.
float dewpoint( float temp_C, uint16_t relativeHumidity )
{
  float logHumidity = log( relativeHumidity / 100.0 );
  float adjustedTemperature = ( 17.62 * temp_C ) / ( 243.12 + temp_C );
  return ( 243.12 * ( logHumidity + adjustedTemperature ) ) /
         ( 17.62 - ( logHumidity + adjustedTemperature ) );
}