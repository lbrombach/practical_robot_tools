#ifndef PIN_DEVICE_H
#define PIN_DEVICE_H

#include <chrono>
#include <string>

class PinDevice {
public:
  enum class OutputDeviceID {
    HOVERBOARD_PWR_BTN,
    E_STOP_RELAY,
    CHARGE_CONNECT_RELAY,
    CHARGE_PROBE_ISOLATE_RELAY,
    SPARE_1,
    SPARE_2,
    SPARE_3,
    SPARE_4
  };

  enum class InputDeviceID {
    E_STOP,
    CHARGE_PROBE,
    SPARE_1,
    SPARE_2,
    SPARE_3,
    SPARE_4,
    SPARE_5,
    SPARE_6
  };

  enum class DeviceControlType {
    ACTIVE_LOW,
    ACTIVE_HIGH,
    PULSE_ACTIVE_LOW,
    PULSE_ACTIVE_HIGH
  };

  PinDevice(std::string name, OutputDeviceID id, DeviceControlType control_type,
            std::chrono::milliseconds pulse_duration =
                std::chrono::milliseconds(200));

  bool getDeviceNeeded() const;
  bool getDeviceState() const;
  bool getPinState();
  std::string getName() const;
  DeviceControlType getControlType() const;

  bool setDeviceNeeded(bool needed);
  void setDeviceState(bool state);
  void setPulseMinTime(std::chrono::milliseconds pulse_min_time =
                           std::chrono::milliseconds(5000));
  // returns

private:
  void updateDeviceState();

  OutputDeviceID id_;
  DeviceControlType control_type_;
  std::chrono::milliseconds pulse_duration_;

  std::string name_;
  bool device_needed_; // True if the device is needed
  bool device_state_;  // True if the device is on according to feedback

  std::chrono::time_point<std::chrono::steady_clock> pulse_start_time_;
  std::chrono::time_point<std::chrono::steady_clock> last_pulse_time_;
  // the minimum number of milliseconds between pulses
  std::chrono::milliseconds pulse_min_time_;
};

#endif // PIN_DEVICE_H
