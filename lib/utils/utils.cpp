#include "utils.h"

// Populate the device ID
void getMacAddress( char* deviceID )
{
  String theAddress = WiFi.macAddress();
  theAddress.replace( ":", "" );
  strcpy( deviceID, theAddress.c_str() );
}