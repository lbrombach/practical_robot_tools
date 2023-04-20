#include "SerialMessage.h"

#define USING_ESP32_W_DISPLAY // or USING_ARDUINO_NANO see PinConfig.h for all options
#include "PinConfig.h"

#include "SerialMessageTypes.h"
#include "SerialPackager.h"
#include "Pin.h"

#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <vector>

#define DELAY                                                                  \
  2500 // Change this to adjust the delay that allows time to read oled messages


std::vector<Pin> outputPins; // Create a vector of output Pin objects
std::vector<Pin> inputPins; // Create a vector of input Pin objects

#ifdef USING_ESP32_W_DISPLAY
SSD1306AsciiWire oled;
#endif

SerialPackager serialPackager(Serial); // Create a SerialPackager object
SerialMessage<uint8_t> outputPinsMsg;  // Create a SerialMessage object
SerialMessage<uint8_t> inputPinsMsg;   // Create a SerialMessage object


void print(const String &msg, int line = 0) {
#ifdef USING_ESP32_W_DISPLAY
  if (line == 0)
    oled.clear();
  oled.setCursor(0, line);
  oled.print(msg.c_str());
#endif
}

String vec2String(const std::vector<uint8_t>& data) {
  String result = "";
  for (uint8_t byte : data) {
    result += (int)byte;
    result += " ";
  }
  return result;
}

void blink(int numBlinks) {
  int delayTime = 100; // Change this to adjust the blink interval
  for (int i = 0; i < numBlinks; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayTime);
    digitalWrite(LED_PIN, LOW);
    delay(delayTime);
  }
  delay(delayTime);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  outputPins.push_back(Pin(HVRBRD_BTN_PIN, OUTPUT));
  outputPins.push_back(Pin(E_STOP_PIN, OUTPUT, LOW));
  outputPins.push_back(Pin(CHARGE_CONNECT_PIN, OUTPUT));
  outputPins.push_back(Pin(CHARGE_PROBE_ISOLATE_PIN, OUTPUT));


  inputPins.push_back(Pin(INPUT_PIN_1, INPUT_PULLUP));
  inputPins.push_back(Pin(INPUT_PIN_2, INPUT_PULLUP));
  /*
    inputPins.push_back(Pin(INPUT_PIN_3, INPUT_PULLUP));
    inputPins.push_back(Pin(INPUT_PIN_4, INPUT_PULLUP));
    inputPins.push_back(Pin(INPUT_PIN_5, INPUT_PULLUP));
    inputPins.push_back(Pin(INPUT_PIN_6, INPUT_PULLUP));
    inputPins.push_back(Pin(INPUT_PIN_7, INPUT_PULLUP));
    inputPins.push_back(Pin(INPUT_PIN_8, INPUT_PULLUP));
  */

  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(1000);

  blink(2);
  delay(1000);

#ifdef USING_ESP32_W_DISPLAY
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x32, I2C_ADDRESS, RST_PIN);
  oled.setFont(System5x7);
  print("Starting with:", 0);
  String str = "outputPins: " + String(outputPins.size());
  print(str, 1);
  str = "inputPins: " + String(inputPins.size());
  print(str, 2);
#endif

  blink(outputPins.size() + inputPins.size());
  delay(DELAY * 2);
}

void loop() {
  // check for messages from the serial port
  std::vector<uint8_t> rawMsg = serialPackager.getMessage(true);
  SerialMessage<uint8_t> msg;


  if (rawMsg.empty()) { // no message
    blink(1);
    print("No Msg", 0);
    String outputs = "";
    for (int i = 0; i < outputPins.size(); i++) {
      outputs += String(outputPins[i].getPinState());
      outputs += " ";
    }
    print("outputs:", 1);
    print(outputs, 2);

  } else if (rawMsg[0] == 0xff) { //invalid msg
    print(vec2String(rawMsg));
    delay(DELAY * 4);
  } else { // valid message

    blink(3);

    // unpack the message into a SerialMessage object
    msg = msg.unpack(rawMsg);

    if (strcmp(msg.topic.c_str(), "outputPins") == 0) {
      outputPinsMsg = msg;
      // set the output pins
      for (int i = 0; i < msg.data.size() && i < outputPins.size(); i++) {
        outputPins[i].setPinState(msg.data[i]);
      }
      // send the output pins state
      std::vector<uint8_t> replyMsg(msg.pack());

      if (serialPackager.sendMessage(replyMsg)) {
        print("reply sent");
      } else {
        print("reply not sent");
      }
    } else if (strcmp(msg.topic.c_str(), "inputPins") == 0) { // send the input pins state
      msg.data.clear();
      for (int i = 0; i < inputPins.size(); i++) {
        msg.data.push_back(inputPins[i].getPinState());
      }
      std::vector<uint8_t> replyMsg(msg.pack());
      if (serialPackager.sendMessage(replyMsg)) {
        print("reply sent");
      } else {
        print("reply not sent");
      }
    } else {
      print("Unknown", 0);
      print(msg.topic.c_str(), 2);
      delay(DELAY * 2);
      // msg.topic = "Unknown";
    }

    String topic = msg.topic.c_str();
    topic += " " + String(msg.data.size());
    String dat1 = "";
    String dat2 = "";

    for (int i = 0; i < msg.data.size(); i++) {
      if (i < 4)
        dat1 += String(msg.data[i]) + " ";
      else
        dat2 += String(msg.data[i]) + " ";
    }
    print(topic, 1);
    print(dat1, 2);
    print(dat2, 3);
    delay(DELAY);
  }
}
