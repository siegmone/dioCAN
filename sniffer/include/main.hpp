#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/twai.h"

// DEBUG INFO OPTION
// decomment to display debug informations
// #define DEBUG

#ifdef DEBUG
#define debug(x)         Serial.print(x)
#define debugln(x)       Serial.println(x)
#define debugf(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#else
#define debug(x)
#define debugln(x)
#define debugf(fmt, ...)
#endif  // DEBUG

// pin definitions
#define GPIO_ERROR_LED GPIO_NUM_22
#define GPIO_CAN_TX    GPIO_NUM_18
#define GPIO_CAN_RX    GPIO_NUM_19
