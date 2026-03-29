#ifndef EXVECTRLINK_SERIALTELECOMPACKETS_HPP
#define EXVECTRLINK_SERIALTELECOMPACKETS_HPP

#include <concepts>
#include <cstdint>
#include <cstring>

#include "ExVectrCore/CanSerialize.hpp"

namespace VCTR::ExVectrLink::packets {

enum SerialPacketType : uint8_t {
  Ack, // Ack for received packet.

  PacketData,   // Packet data. Max 255 bytes.
  ChannelState, // If channel is blocked and max packet size

  SetModulationPreset, // Set the modulation preset of the radio link.
  SetTxPower,          // Set the Tx power of the radio link. 0-20 dBm.
  SetLinkChannel,      // Set the channel of the radio link. 0-9. Stops FHSS.
  StartFHSS,           // Start FHSS. Requires 4 byte key.

  DeviceTemperature, // Send device temperature.
  FhssSyncState,     // FHSS sync status
  Error,             // Error occured.
  Print,             // Print message to serial console.

  Heartbeat,   // Heartbeat packet.
  LinkInfo,    // Received Packet with RSSI, SNR, and loss rate.
  SetBaudRate, // Sets the baud rate of the serial communication.
  UpdateMode,  // Places ExVectrLink into Update mode
  InitLink, // Send to startup the link and also set the Media access key (MAK).
};

template <typename T>
concept IsSerialPacket = VCTR::Core::CanSerialize<T> && requires(const T a) {
  { a.getPacketType() } -> std::same_as<SerialPacketType>;
};

} // namespace VCTR::ExVectrLink::packets

namespace VCTR::ExVectrLink::packets {

class SerialPacket_Ack {
public:
  bool success;

  SerialPacketType getPacketType() const { return SerialPacketType::Ack; }
  uint8_t numBytes() const { return 1; }
  void serialize(uint8_t *buffer) const { buffer[0] = success ? 1 : 0; }
  bool deserialize(const uint8_t *buffer) {
    success = buffer[0] == 1;
    return true;
  }
};

class SerialPacket_ChannelState {
public:
  uint8_t maxBytes;
  bool blocked;

  SerialPacketType getPacketType() const {
    return SerialPacketType::ChannelState;
  }
  uint8_t numBytes() const { return 2; }
  void serialize(uint8_t *buffer) const {
    buffer[0] = maxBytes;
    buffer[1] = blocked ? 1 : 0;
  }
  bool deserialize(const uint8_t *buffer) {
    maxBytes = buffer[0];
    blocked = buffer[1] == 1;
    return true;
  }
};

class SerialPacket_SetModulationPreset {
public:
  uint8_t presetIndex;

  SerialPacketType getPacketType() const {
    return SerialPacketType::SetModulationPreset;
  }
  uint8_t numBytes() const { return 1; }
  void serialize(uint8_t *buffer) const { buffer[0] = presetIndex; }
  bool deserialize(const uint8_t *buffer) {
    presetIndex = buffer[0];
    return true;
  }
};

class SerialPacket_SetTxPower {
public:
  uint8_t txPower; // Tx power in dBm.

  SerialPacketType getPacketType() const {
    return SerialPacketType::SetTxPower;
  }
  uint8_t numBytes() const { return 1; }
  void serialize(uint8_t *buffer) const { buffer[0] = txPower; }
  bool deserialize(const uint8_t *buffer) {
    txPower = buffer[0];
    return true;
  }
};

class SerialPacket_SetLinkChannel {
public:
  uint8_t channelIndex; // 0-9

  SerialPacketType getPacketType() const {
    return SerialPacketType::SetLinkChannel;
  }
  uint8_t numBytes() const { return 1; }
  void serialize(uint8_t *buffer) const { buffer[0] = channelIndex; }
  bool deserialize(const uint8_t *buffer) {
    channelIndex = buffer[0];
    return true;
  }
};

class SerialPacket_SetEnableFhss {
public:
  bool enable;
  uint32_t seqKey; // Sequence key for FHSS.

  SerialPacketType getPacketType() const { return SerialPacketType::StartFHSS; }
  uint8_t numBytes() const { return 5; }
  void serialize(uint8_t *buffer) const {
    buffer[0] = enable ? 1 : 0;
    buffer[1] = seqKey & 0xFF;
    buffer[2] = (seqKey >> 8) & 0xFF;
    buffer[3] = (seqKey >> 16) & 0xFF;
    buffer[4] = (seqKey >> 24) & 0xFF;
  }
  bool deserialize(const uint8_t *buffer) {
    enable = buffer[0] == 1;
    seqKey =
        buffer[1] | (buffer[2] << 8) | (buffer[3] << 16) | (buffer[4] << 24);
    return true;
  }
};

class SerialPacket_DeviceTemperature {
public:
  int8_t temperatureC; // Temperature in Celsius.

  SerialPacketType getPacketType() const {
    return SerialPacketType::DeviceTemperature;
  }
  uint8_t numBytes() const { return 1; }
  void serialize(uint8_t *buffer) const { buffer[0] = temperatureC; }
  bool deserialize(const uint8_t *buffer) {
    temperatureC = buffer[0];
    return true;
  }
};

class SerialPacket_FhssSyncState {
public:
  bool synced;

  SerialPacketType getPacketType() const {
    return SerialPacketType::FhssSyncState;
  }
  uint8_t numBytes() const { return 1; }
  void serialize(uint8_t *buffer) const { buffer[0] = synced; }
  bool deserialize(const uint8_t *buffer) {
    synced = buffer[0];
    return true;
  }
};

class SerialPacket_SetBaudRate {
public:
  uint32_t baudRate;

  SerialPacketType getPacketType() const {
    return SerialPacketType::SetBaudRate;
  }
  uint8_t numBytes() const { return 4; }
  void serialize(uint8_t *buffer) const {
    buffer[0] = baudRate & 0xFF;
    buffer[1] = (baudRate >> 8) & 0xFF;
    buffer[2] = (baudRate >> 16) & 0xFF;
    buffer[3] = (baudRate >> 24) & 0xFF;
  }
  bool deserialize(const uint8_t *buffer) {
    baudRate =
        buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
    return true;
  }
};

class SerialPacket_UpdateMode {
public:
  // If true, then all other nodes will be placed into update mode,
  // Otherwise, only this node.
  bool forward;
  // How long to stay in update mode.
  int64_t duration;

  SerialPacketType getPacketType() const {
    return SerialPacketType::UpdateMode;
  }
  uint8_t numBytes() const { return 9; }
  void serialize(uint8_t *buffer) const {
    buffer[0] = forward ? 1 : 0;
    buffer[1] = duration & 0xFF;
    buffer[2] = (duration >> 8) & 0xFF;
    buffer[3] = (duration >> 16) & 0xFF;
    buffer[4] = (duration >> 24) & 0xFF;
    buffer[5] = (duration >> 32) & 0xFF;
    buffer[6] = (duration >> 40) & 0xFF;
    buffer[7] = (duration >> 48) & 0xFF;
    buffer[8] = (duration >> 56) & 0xFF;
  }
  bool deserialize(const uint8_t *buffer) {
    forward = buffer[0] == 1;
    duration = static_cast<int64_t>(buffer[1]) |
               (static_cast<int64_t>(buffer[2]) << 8) |
               (static_cast<int64_t>(buffer[3]) << 16) |
               (static_cast<int64_t>(buffer[4]) << 24) |
               (static_cast<int64_t>(buffer[5]) << 32) |
               (static_cast<int64_t>(buffer[6]) << 40) |
               (static_cast<int64_t>(buffer[7]) << 48) |
               (static_cast<int64_t>(buffer[8]) << 56);
    return true;
  }
};

class SerialPacket_InitLink {
public:
  uint8_t mak; // Media Access Key for FHSS.

  SerialPacketType getPacketType() const { return SerialPacketType::InitLink; }
  uint8_t numBytes() const { return 1; }
  void serialize(uint8_t *buffer) const { buffer[0] = mak; }
  bool deserialize(const uint8_t *buffer) {
    mak = buffer[0];
    return true;
  }
};

class SerialPacket_Print {
public:
  static constexpr size_t maxMessageLength = 100; // Max length of message.
  char message[maxMessageLength]; // Null-terminated string to print.
  uint8_t length = 0;

  SerialPacketType getPacketType() const { return SerialPacketType::Print; }
  uint8_t numBytes() const { return length + 1; }
  void serialize(uint8_t *buffer) const {
    buffer[0] = length;
    std::memcpy(buffer + 1, message, length);
  }
  bool deserialize(const uint8_t *buffer) {
    length = buffer[0];
    if (length > SerialPacket_Print::maxMessageLength) {
      length = SerialPacket_Print::maxMessageLength;
    }
    if (length > 0) {
      std::memcpy(message, buffer + 1, length);
      message[length - 1] = '\0'; // Ensure null termination.
    } else {
      message[0] = '\0';
    }
    return true;
  }
};

class SerialPacket_Heartbeat {
public:
  // If we are receiving packets from the serial port.
  // Allows other end to see if we also seeing their connection.
  bool isConnected;

  SerialPacketType getPacketType() const { return SerialPacketType::Heartbeat; }
  uint8_t numBytes() const { return 1; }
  void serialize(uint8_t *buffer) const { buffer[0] = isConnected ? 1 : 0; }
  bool deserialize(const uint8_t *buffer) {
    isConnected = buffer[0] == 1;
    return true;
  }
};

class SerialPacket_LinkInfo {
public:
  int8_t rssi;
  int8_t snr;
  uint8_t antenna; // Current antenna in use.
  int8_t txPower;

  uint8_t lossRate; // percentage of packets lost. 0 good, 100 all.

  bool dualLinkMode;

  SerialPacketType getPacketType() const { return SerialPacketType::LinkInfo; }
  uint8_t numBytes() const { return 6; }
  void serialize(uint8_t *buffer) const {
    buffer[0] = rssi;
    buffer[1] = snr;
    buffer[2] = antenna;
    buffer[3] = lossRate;
    buffer[4] = dualLinkMode ? 1 : 0;
    buffer[5] = txPower;
  }
  bool deserialize(const uint8_t *buffer) {
    rssi = buffer[0];
    snr = buffer[1];
    antenna = buffer[2];
    lossRate = buffer[3];
    dualLinkMode = buffer[4] == 1;
    txPower = buffer[5];
    return true;
  }
};

} // namespace VCTR::ExVectrLink::packets

#endif // EXVECTRLINK_SERIALTELECOMPACKETS_HPP