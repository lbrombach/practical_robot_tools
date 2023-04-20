// PinConfig.h

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H


#ifdef USING_ESP32_W_DISPLAY

// I2c oled display pins and address
// Define proper RST_PIN if required. (16 for esp32 dev board with oled display)
#define RST_PIN 16
#define SDA_PIN 4
#define SCL_PIN 15
#define I2C_ADDRESS                                                            \
  0x3C // onboard oled. maybe 0X3C+SA0 - 0x3C or 0x3D for other oleds


// ESP32 pin configurations

#define LED_PIN 25

// Output pin definitions
#define HVRBRD_BTN_PIN 12
#define E_STOP_PIN 13
#define CHARGE_CONNECT_PIN 14
#define CHARGE_PROBE_ISOLATE_PIN 26
#define SPARE_PIN_2 27
#define SPARE_PIN_3 32
#define SPARE_PIN_4 33
#define SPARE_PIN_5 23

// Input pin definitions
#define INPUT_PIN_1 17
#define INPUT_PIN_2 2
#define INPUT_PIN_3 3
#define INPUT_PIN_4 5
#define INPUT_PIN_5 6
#define INPUT_PIN_6 7
#define INPUT_PIN_7 8
#define INPUT_PIN_8 9

#elif USING_ARDUINO_NANO
// Arduino Nano pin configurations

#define LED_PIN 13

// Output pin definitions
#define HVRBRD_BTN_PIN A5
#define E_STOP_PIN A4
#define CHARGE_CONNECT_PIN A3
#define CHARGE_PROBE_ISOLATE_PIN A2
#define SPARE_PIN_2 A1
#define SPARE_PIN_3 A0
#define SPARE_PIN_4 A6
#define SPARE_PIN_5 A7

// Input pin definitions
#define INPUT_PIN_1 2
#define INPUT_PIN_2 3
#define INPUT_PIN_3 4
#define INPUT_PIN_4 5
#define INPUT_PIN_5 6
#define INPUT_PIN_6 7
#define INPUT_PIN_7 8
#define INPUT_PIN_8 9

#endif

#endif // PIN_CONFIG_H
