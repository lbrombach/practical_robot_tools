#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

//declare constants
const long CONNECTION_TIMEOUT_SECONDS = 5;
const int LIDAR_RELAY_PIN = 2;
const int POWER_BTN_RELAY_PIN = 5;
const int LOWER_FANS_RELAY_PIN = 6;
const int UPPER_FANS_RELAY_PIN = 7;
const int SAFETY_LIGHT_RELAY_PIN = 9;

//declare ROS NodeHandle
ros::NodeHandle nh;

//declare some global boolean flags
bool hbConnected = false;
bool activelyBlinking = false;

//timestamps for last message recieved for each
unsigned long lastHbRequiredMsg = 0;
unsigned long lastLightMsg = 0;
unsigned long lastBlinkMsg = 0;
unsigned long lastLidarMsg = 0;

/// helper to get the number of seconds that have elapsed since the timeStamp passed in.
double getSecondsSince(unsigned long timeStamp)
{
  return (millis() - timeStamp) / 1000;
}

/// A class to manage relay functions.
class Relay
{
private:
  // The GPIO pin number the relayy will be connected to
  int pinNumber;

  // how long to hold a relay "on" in response to a call to the pulse() function
  long pulseDuration;

  // the time that must elapse before a relay will pulse again
  long minMillisBetPulses;

  // the timestamp of the last time the relay has been pulsed
  long lastPulse;

public:
  /// constructor.
  Relay(int pin, long pulse = 350, long minMillis = 3000)
  {
    pinNumber = pin;
    pulseDuration = pulse;
    minMillisBetPulses = minMillis;
    lastPulse = 0;
    pinMode(pinNumber, OUTPUT);
    digitalWrite(pinNumber, HIGH);
  };

  /// Energizes the relay.
  void on() { digitalWrite(pinNumber, LOW); };

  /// De-energizes the relay.
  void off() { digitalWrite(pinNumber, HIGH); };

  /// Toggles the state of the relay (energized vs de-energized).
  void toggle() { digitalWrite(pinNumber, !digitalRead(pinNumber)); };

  /// Toggles the state of the relay, pauses for pulseDuration milliseconds, then toggles the relay back.
  void pulse()
  {
    if (millis() - lastPulse >= minMillisBetPulses)
    {
      lastPulse = millis();
      toggle();
      delay(pulseDuration);
      toggle();
    }
  };
};
// Declare relay objects
Relay lidarRelay(LIDAR_RELAY_PIN);
Relay powerBtnRelay(POWER_BTN_RELAY_PIN);
Relay safetyLightRelay(SAFETY_LIGHT_RELAY_PIN, 800, 800);
Relay upperFansRelay(UPPER_FANS_RELAY_PIN);
Relay lowerFansRelay(LOWER_FANS_RELAY_PIN);

/// callback for the hoverboard/connected topic.
void hbConnectedCb(const std_msgs::Bool &msg)
{
  // Update the global hbConnected boolean flag
  hbConnected = msg.data;
}

/// callback for the hoverboard_required.
/// Turns pulses the powerbutton relay to turn the hoverboard control board on or off as needed
void hbRequiredCb(const std_msgs::Bool &hbRequired)
{
  //update last message timestamp
  lastHbRequiredMsg = millis();

  //pulse the control board power button if the requirement does node equal the connected status
  if (hbRequired.data != hbConnected)
  {
    powerBtnRelay.pulse();
  }
}

/// is light (steady light) required callback.
void lightRequiredCb(const std_msgs::Bool &lightRequired)
{
  //update last message timestamp
  lastLightMsg = millis();

  // check if the light is already blinking before commanding to on. Also check that it is needed before turning on
  if (!activelyBlinking && lightRequired.data == true)
    safetyLightRelay.on();
  else
    safetyLightRelay.off();
}

/// is light blinking required callback.
void blinkRequiredCb(const std_msgs::Bool &activelyBlinking)
{
  //update last message timestamp
  lastBlinkMsg = millis();

  // perform a single blink if blinking is required
  if (activelyBlinking.data)
    safetyLightRelay.pulse();
}

/// is the lidar required callback.
void lidarRequiredCb(const std_msgs::Bool &lidarRequired)
{
  //update last message timestamp
  lastLidarMsg = millis();

  // turn on the relay to power the lidar if the lidar is required. Otherwise turn off.
  if (lidarRequired.data == true)
    lidarRelay.on();
  else
    lidarRelay.off();
}

// define subscribers
ros::Subscriber<std_msgs::Bool> connectionSub("hoverboard/connected", &hbConnectedCb);
ros::Subscriber<std_msgs::Bool> hbRequiredSub("hoverboard_required", &hbRequiredCb);
ros::Subscriber<std_msgs::Bool> lightRequiredSub("light_required", &lightRequiredCb);
ros::Subscriber<std_msgs::Bool> blinkRequiredSub("light_blink_required", &blinkRequiredCb);
ros::Subscriber<std_msgs::Bool> lidarRequiredSub("lidar_required", &lidarRequiredCb);

/// The standard Arduino setup() function that runs once upon startup
void setup()
{
  // initialize with ROS and subscribe to topics
  nh.initNode();
  nh.subscribe(connectionSub);
  nh.subscribe(hbRequiredSub);
  nh.subscribe(lightRequiredSub);
  nh.subscribe(blinkRequiredSub);
  nh.subscribe(lidarRequiredSub);

  //blink the safety light as an indicator that the Arduino is starting and running
  safetyLightRelay.on();
  delay(1000);
  safetyLightRelay.off();
  delay(1000);

  //turn the cooling fan relays on 
  lowerFansRelay.on();
  upperFansRelay.on();
}

/// The standard Arduino loop that runs forever
void loop()
{
  nh.spinOnce();

  if (getSecondsSince(lastHbRequiredMsg) > CONNECTION_TIMEOUT_SECONDS)
    powerBtnRelay.off();
  if (getSecondsSince(lastBlinkMsg) > CONNECTION_TIMEOUT_SECONDS && getSecondsSince(lastLightMsg) > CONNECTION_TIMEOUT_SECONDS)
    safetyLightRelay.off();
  if (getSecondsSince(lastLidarMsg) > CONNECTION_TIMEOUT_SECONDS)
    lidarRelay.off();
}
