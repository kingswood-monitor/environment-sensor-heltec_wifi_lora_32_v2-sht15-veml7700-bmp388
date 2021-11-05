#ifndef H_UTILS
#define H_UTILS

#include <Arduino.h>
#include <WiFi.h>

void getMacAddress( char* deviceID );
float dewpoint( float temp_C, uint16_t relativeHumidity );

#endif