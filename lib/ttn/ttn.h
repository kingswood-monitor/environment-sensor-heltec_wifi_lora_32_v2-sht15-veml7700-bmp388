#ifndef H_TTN
#define H_TTN

#include <Arduino.h>

// Callback function when a TTN message is received
void message( const uint8_t* payload, size_t size, int rssi );

#endif