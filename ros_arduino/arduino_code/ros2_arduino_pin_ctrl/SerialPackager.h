#ifndef SERIAL_PACKAGER_H
#define SERIAL_PACKAGER_H

// A class for gettting and sending messages over serial
// The messages are formatted as follows:
// 3 start bytes, 2 msg length bytes, n msg bytes, 2 checksum bytes
// Big endian (high byte first)
// "packet" is used to refer to the entire message, including start sequence, msg length, and checksum bytes
// "message" is used to refer to the message data, stripped of start sequence, msg length, and checksum bytes

#include <stdint.h>
#include <cstddef>
#include <vector>
//include Arduino Serial library
#include <Arduino.h>


class SerialPackager
{
  public:

    // Start sequence bytes
    const uint8_t START_SIGNAL[3] = {0xAA, 0xAA, 0xAA};
    static const int WAIT_LIMIT = 1000;

    // constructor
    SerialPackager(HardwareSerial &serial);

    ~SerialPackager();

    bool isSerialConnected();

    // get a message from the serial port
    // returns the packet as a vector of bytes, stripped of start sequence, msg length, and checksum bytes
    // returns an empty vector if no message was received
    // timeout is the number of milliseconds to wait for a message
    // if timeout is -1, the function will wait indefinitely
    // returns first element -1 if message invalid
    // if returnInvalid is true, the function will return the first element as -1 plus the rest of the message (useful for debugging)
    std:: vector<uint8_t> getMessage(bool returnInvalid = false);

    // send a message over the serial port
    // returns true if the message was sent
    bool sendMessage(std::vector<uint8_t> &message);

  private:
    // the serial port object to be passed in by consructor
    HardwareSerial &serial;

    // calculate the checksum of a message
    // returns the checksum to be appended to the end of the message
    uint16_t calculateChecksum(const std::vector<uint8_t> &message);

    // validate the checksum of a packet
    // returns true if the checksum is valid
    bool validateChecksum(const std::vector<uint8_t> &packet);

    // calculate the checksum of a message and append it to the end of the message
    void appendChecksum(std::vector<uint8_t> &packet);
};

#endif /* SERIAL_PACKAGER_H */
