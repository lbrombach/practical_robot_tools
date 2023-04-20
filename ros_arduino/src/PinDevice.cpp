#include "ros_arduino/PinDevice.h"
#include <iostream>

PinDevice::PinDevice(std::string name, OutputDeviceID id,
                     DeviceControlType control_type,
                     std::chrono::milliseconds pulse_duration)
    : name_(name), id_(id), control_type_(control_type),
      pulse_duration_(pulse_duration) {
  device_needed_ = false;
  device_state_ = false;
  pulse_start_time_ = std::chrono::steady_clock::time_point();
  last_pulse_time_ = std::chrono::steady_clock::time_point();
  pulse_min_time_ = std::chrono::milliseconds(5000);
}

bool PinDevice::getDeviceNeeded() const { return device_needed_; }
bool PinDevice::getDeviceState() const { return device_state_; }

// this returns whether or not the pins needs to be on or off for a given cycle
// in the case of a pulse type device, this will return the active value for
// the device for pulse_duration_ milliseconds, then return the inactive value
bool PinDevice::getPinState() {
  bool requiredState;
  if (control_type_ == DeviceControlType::ACTIVE_HIGH) {
    requiredState = device_needed_;
  } else if (control_type_ == DeviceControlType::ACTIVE_LOW) {
    requiredState = !device_needed_;
  } else { // control_type_  is PULSE_ACTIVE_HIGH or PULSE_ACTIVE_LOW
    // if the device is needed and the device is not on AND the pulse has not
    // started, then the pin needs to be in the active state

  //  std::cout << "in pulse" << std::endl;
    auto now = std::chrono::steady_clock::now();
    // seconds since the last pulse started
    std::chrono::duration<double> seconds_since_pulse_start =
        now - pulse_start_time_;
    static bool pulseActive = false;

    // if (control_type_ == DeviceControlType::PULSE_ACTIVE_LOW)
       std::cout << "needed/state/pulseactive : " << (int)device_needed_ << " "
                 << (int)device_state_ << " " << (int)pulseActive << std::endl;
    // // print the last pulse time and the current time - last pulse time
    // std::cout << "last pulse time: "
    //           << last_pulse_time_.time_since_epoch().count() << std::endl;
    // std::cout << "now: " << now.time_since_epoch().count() << std::endl;
    // std::cout << "now - last pulse time: " << (now - last_pulse_time_).count()
    //           << std::endl;

    if (device_needed_ != device_state_ && !pulseActive &&
        now - last_pulse_time_ > pulse_min_time_) {
      pulse_start_time_ = now;
      last_pulse_time_ = now;
      pulseActive = true;
  //    std::cout << "pulse start" << std::endl;
    } else if (pulseActive && seconds_since_pulse_start >= pulse_duration_) {
      pulseActive = false;      
  //    std::cout << "pulse end" << std::endl;
    }

    if (pulseActive) {
      requiredState = (control_type_ == DeviceControlType::PULSE_ACTIVE_HIGH)
                          ? true
                          : false;
  //    std::cout << "pulse ACTIVE" << std::endl;
    } else {
      requiredState = (control_type_ == DeviceControlType::PULSE_ACTIVE_HIGH)
                          ? false
                          : true;
  //    std::cout << "pulse inactive" << std::endl;
    }
  }

  return requiredState;
}

std::string PinDevice::getName() const { return name_; }

bool PinDevice::setDeviceNeeded(bool needed) {
  device_needed_ = needed;
  return (device_state_ == device_needed_);
}

void PinDevice::setDeviceState(bool state) { device_state_ = state; }

PinDevice::DeviceControlType PinDevice::getControlType() const { return control_type_; }

// void PinDevice::updateDeviceState() {
//   if (control_type_ == DeviceControlType::PULSE) {
//     auto now = std::chrono::steady_clock::now();
//     if (device_needed_ && !device_state_) {
//       device_state_ = true;
//       pin_state_ = true;
//       pulse_start_time_ = now;
//     } else if (device_state_ &&
//                std::chrono::duration_cast<std::chrono::milliseconds>(
//                    now - pulse_start_time_) >= pulse_duration_) {
//       device_state_ = false;
//       pin_state_ = false;
//     }
//   } else {
//     device_state_ = device_needed_;
//     pin_state_ = (control_type_ == DeviceControlType::ACTIVE_HIGH)
//                      ? device_state_
//                      : !device_state_;
//   }
// }
