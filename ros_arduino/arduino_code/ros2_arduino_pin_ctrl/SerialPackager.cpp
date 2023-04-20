#include "SerialPackager.h"

//const uint8_t SerialPackager::START_SIGNAL[3] = {0xAA, 0xAA, 0xAA};


SerialPackager::SerialPackager(HardwareSerial &serial) : serial(serial) {

}

SerialPackager::~SerialPackager() {}

bool SerialPackager::isSerialConnected() {
  return serial;
}


// get a packet from the serial port
// returns the packet as a vector of bytes
// returns an empty vector if no packet was received
// if returnInvalid is true, the function will return the first element as -1
// plus the rest of the packet (useful for debugging) (default false)
std :: vector<uint8_t> SerialPackager::getMessage(bool returnInvalid) {


  bool gotStart = false;
  // Discard bytes until the start sequence is detected
  while (serial.available() > 3 && !gotStart) {
    uint8_t readVal = serial.read();
    if (readVal == START_SIGNAL[0]); // Wait for start signal
    {
      if (serial.read() == START_SIGNAL[1] &&
          serial.read() == START_SIGNAL[2]) {
        gotStart = true; // Start signal detected
      }
    }
  }
  if (!gotStart)
    return std::vector<uint8_t>();

  // Wait for enough bytes to be available before reading packet length
  while (serial.available() < 2)
    delay(1);

  // Read packet length, which includes the start sequence bytes (3), packet
  // length bytes (2), and checksum bytes (2)
  uint16_t packetLength = serial.read() << 8;
  packetLength |= serial.read();


  // Wait for enough bytes to be available before reading packet
  // (packet length - 5 because we already read 5 bytes, leaving only the
  // message and checksum bytes)
  while (serial.available() < packetLength - 5)
    delay(1);

  // Read packet
  std::vector<uint8_t> packet;
  for (int i = 0; i < packetLength - 5; i++) {
    packet.push_back(serial.read());
  }

  // Validate checksum and return packet
  if (validateChecksum(packet)) {

    //////////////////////////////////////
    std::vector<uint8_t> temp;
    uint16_t _type = 2;
    String topic = "valid";
    temp.push_back((_type >> 8) & 0xff); // high byte first
    temp.push_back(_type & 0xff);        // low byte second



    uint16_t topic_length = (uint16_t)topic.length();
    temp.push_back((topic_length >> 8) & 0xff); // high byte first
    temp.push_back(topic_length & 0xff);        // low byte second

    temp.insert(temp.end(), topic.begin(), topic.end());
    temp.push_back((uint8_t)1);
    temp.push_back((uint8_t)0);
    temp.push_back((uint8_t)8);

    //return temp; ////////////////////////////////////
    /////////////////////////////////////////////////

    // remove checksum bytes from packet
    packet.pop_back();
    packet.pop_back();
    return packet;
  } else {
    if (returnInvalid) {
      packet.insert(packet.begin(), 0xff);
      return packet;
    } else {
      //////////////////////////////////////
      std::vector<uint8_t> temp;
      uint16_t _type = 2;
      temp.push_back((_type >> 8) & 0xff); // high byte first
      temp.push_back(_type & 0xff);        // low byte second


      uint8_t chkHB = packet[packet.size() - 2];
      uint8_t chkLB = packet[packet.size() - 1];
      uint16_t chk1 = chkHB << 8;
      chk1 |= chkLB;

      String topic = String(packet.size());
      topic += " ";

      topic += String(calculateChecksum(packet));  /////add calc check
      // case 2:  1554 - 1225 = 329      170*3=510   4 = 100   201 = 11001001   = 1225
      // case 3:  1426 - 1046 = 380                  4 = 100    22 = 00010110   = 1046


      uint16_t topic_length = (uint16_t)topic.length();
      temp.push_back((topic_length >> 8) & 0xff); // high byte first
      temp.push_back(topic_length & 0xff);        // low byte second

      temp.insert(temp.end(), topic.begin(), topic.end());
      temp.push_back((uint8_t)1);
      temp.push_back((uint8_t)0);
      temp.push_back((uint8_t)8);

      return temp; ////////////////////////////////////
      /////////////////////////////////////////////////

      return std::vector<uint8_t>();
    }
  }
}

// send a packet over the serial port
// returns true if the packet was sent
bool SerialPackager::sendMessage(std::vector<uint8_t> &message) {
  // create a packet from the message
  std::vector<uint8_t> packet = message;

  // add start sequence and packet length to the packet
  uint8_t pktLenHB = ((packet.size() + 7) >> 8) & 0xFF;
  uint8_t pktLenLB = (packet.size() + 7) & 0xFF;
  packet.insert(packet.begin(), pktLenLB);
  packet.insert(packet.begin(), pktLenHB);
  // insert start sequence all at once
  packet.insert(packet.begin(), START_SIGNAL,
                START_SIGNAL + sizeof(START_SIGNAL));

  // calculate the checksum and append it to the packet
  appendChecksum(packet);
  message = packet;/////////////////////////////////////////////////////////TEMP LINE /////////////////////////
  // send the packet
  for (int i = 0; i < packet.size(); i++) {
    serial.write(packet[i]);
  }

  return true;
}

// calculate the checksum of a packet
// returns the checksum
uint16_t SerialPackager::calculateChecksum(const std::vector<uint8_t> &message) {
  uint16_t checksum = 0;
  for (int i = 0; i < message.size() - 2; i++) {
    checksum += message[i];
  }
  return checksum;
}

// validate the checksum of a packet
// returns true if the checksum is valid  
bool SerialPackager::validateChecksum(const std::vector<uint8_t> &packet) {
  uint16_t checksum = calculateChecksum(packet);
  // - 2 so we don't add the checksum values 
  uint16_t packetChecksum = packet[packet.size() - 2] << 8;
  packetChecksum |= packet[packet.size() - 1];
  return checksum == packetChecksum;
}

// append the checksum to a packet
void SerialPackager::appendChecksum(std::vector<uint8_t> &packet) {
  // calculate checksum starting at the first byte of the message, not the start
  // sequence or packet length. Must push back checksum bytes before calling calculateChecksum() because of how it maths
  packet.push_back(0);
  packet.push_back(0);
  uint16_t checksum =
    calculateChecksum(std::vector<uint8_t>(packet.begin() + 5, packet.end()));
  uint8_t checksumHB = (checksum >> 8) & 0xFF;
  uint8_t checksumLB = checksum & 0xFF;
  packet[packet.size() - 2] = checksumHB;
  packet[packet.size() - 1] = checksumLB;

}
