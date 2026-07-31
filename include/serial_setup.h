#pragma once
#include <Arduino.h>

#ifndef APP_NAME
#define APP_NAME unknown_app     // safety net: still compiles standalone
#endif

#define STRINGIFY_(x) #x
#define STRINGIFY(x) STRINGIFY_(x)
#define APP_NAME_STR STRINGIFY(APP_NAME)

// Brings up the USB VCP and prints a build banner.
// Blocks up to `wait_ms` for a host terminal to attach, then continues.
void serialSetup(const char* app_name, uint32_t wait_ms = 2000);

// Blink the onboard LED forever with a fault code. Never returns.
void fatal(const char* msg, uint8_t blinks = 3);