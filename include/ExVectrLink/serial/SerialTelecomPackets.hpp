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
  SetLinkChannel,      // Set the channel of the radio link. 0-9. Stops FHSS.
  StartFHSS,           // Start FHSS. Requires 4 byte key.
  SetPowerParams,      // Set tx power parameters.

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

class SerialPacket_SetPowerParams {
public:
  uint8_t txPower; // Tx power in dBm, set to 0 for max power.
  uint8_t
      maxDynPower; // Max power the dynamic power system will use. 0 for max.
  uint8_t minDynPower;     // Min power the dynamic power system will use.
  bool enableDynamicPower; // If disabled, then will use txPower, if enabled,
                           // then txPower is max power.

  SerialPacketType getPacketType() const {
    return SerialPacketType::SetPowerParams;
  }
  uint8_t numBytes() const { return 3; }
  void serialize(uint8_t *buffer) const {
    buffer[0] = (enableDynamicPower ? 0x80 : 0x00) | (txPower & 0x7F);
    buffer[1] = maxDynPower;
    buffer[2] = minDynPower;
  }
  bool deserialize(const uint8_t *buffer) {
    enableDynamicPower = (buffer[0] & 0x80) != 0;
    txPower = buffer[0] & 0x7F;
    maxDynPower = buffer[1];
    minDynPower = buffer[2];
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
  /// Per-side link statistics (13 bytes serialised).
  struct SideStats {
    int8_t rssi = 0;             ///< dBm (negative)
    int8_t snr = 0;              ///< dB
    int8_t txPower = 0;          ///< dBm
    bool dynamicPower = false;   ///< if dyn power enabled
    uint8_t antenna = 0;         ///< Active antenna index
    uint8_t linkQuality = 0;     ///< 0-100 % receive slots carrying payload
    uint8_t lossRate = 0;        ///< 0-100 % (100 = all lost)
    uint8_t packetQuality = 0;   ///< 0-100 % simply packets received / arrived
    uint16_t desyncCount = 0;    ///< FHSS desync count
    uint16_t packetRate = 0;     ///< Accepted RC packets per second
    uint16_t crsfTxFailures = 0; ///< CRSF frames that failed to queue per s
  };

  SideStats local;  ///< Stats as seen / reported by this node.
  SideStats remote; ///< Stats received from the far end via OTA LinkTelemetry.
  bool remoteValid = false; ///< True once at least one OTA packet received.

  SerialPacketType getPacketType() const { return SerialPacketType::LinkInfo; }
  // 13 bytes × 2 sides + 1 valid flag = 27 bytes.
  uint8_t numBytes() const { return 27; }

  void serialize(uint8_t *buffer) const {
    buffer[0] = (uint8_t)local.rssi;
    buffer[1] = (uint8_t)local.snr;
    buffer[2] = (local.dynamicPower ? 0x80 : 0x00) | (local.txPower & 0x7F);
    buffer[3] = local.antenna;
    buffer[4] = local.linkQuality;
    buffer[5] = local.lossRate;
    buffer[6] = local.packetQuality;
    buffer[7] = local.desyncCount & 0xFF;
    buffer[8] = (local.desyncCount >> 8) & 0xFF;
    buffer[9] = local.packetRate & 0xFF;
    buffer[10] = (local.packetRate >> 8) & 0xFF;
    buffer[11] = local.crsfTxFailures & 0xFF;
    buffer[12] = (local.crsfTxFailures >> 8) & 0xFF;
    buffer[13] = (uint8_t)remote.rssi;
    buffer[14] = (uint8_t)remote.snr;
    buffer[15] = (remote.dynamicPower ? 0x80 : 0x00) | (remote.txPower & 0x7F);
    buffer[16] = remote.antenna;
    buffer[17] = remote.linkQuality;
    buffer[18] = remote.lossRate;
    buffer[19] = remote.packetQuality;
    buffer[20] = (uint8_t)(remote.desyncCount & 0xFF);
    buffer[21] = (uint8_t)((remote.desyncCount >> 8) & 0xFF);
    buffer[22] = (uint8_t)(remote.packetRate & 0xFF);
    buffer[23] = (uint8_t)((remote.packetRate >> 8) & 0xFF);
    buffer[24] = (uint8_t)(remote.crsfTxFailures & 0xFF);
    buffer[25] = (uint8_t)((remote.crsfTxFailures >> 8) & 0xFF);
    buffer[26] = remoteValid ? 1 : 0;
  }
  bool deserialize(const uint8_t *buffer) {
    local.rssi = (int8_t)buffer[0];
    local.snr = (int8_t)buffer[1];
    local.txPower = (int8_t)(buffer[2] & 0x7F);
    local.dynamicPower = (buffer[2] & 0x80) != 0;
    local.antenna = buffer[3];
    local.linkQuality = buffer[4];
    local.lossRate = buffer[5];
    local.packetQuality = buffer[6];
    local.desyncCount = (uint16_t)buffer[7] | ((uint16_t)buffer[8] << 8);
    local.packetRate = (uint16_t)buffer[9] | ((uint16_t)buffer[10] << 8);
    local.crsfTxFailures = (uint16_t)buffer[11] | ((uint16_t)buffer[12] << 8);
    remote.rssi = (int8_t)buffer[13];
    remote.snr = (int8_t)buffer[14];
    remote.txPower = (int8_t)(buffer[15] & 0x7F);
    remote.dynamicPower = (buffer[15] & 0x80) != 0;
    remote.antenna = buffer[16];
    remote.linkQuality = buffer[17];
    remote.lossRate = buffer[18];
    remote.packetQuality = buffer[19];
    remote.desyncCount = (uint16_t)buffer[20] | ((uint16_t)buffer[21] << 8);
    remote.packetRate = (uint16_t)buffer[22] | ((uint16_t)buffer[23] << 8);
    remote.crsfTxFailures = (uint16_t)buffer[24] | ((uint16_t)buffer[25] << 8);
    remoteValid = buffer[26] == 1;
    return true;
  }
};

} // namespace VCTR::ExVectrLink::packets

#endif // EXVECTRLINK_SERIALTELECOMPACKETS_HPP