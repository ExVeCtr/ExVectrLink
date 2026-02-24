#ifndef EXVECTRLINK_SERIALTELECOMPACKETS_HPP
#define EXVECTRLINK_SERIALTELECOMPACKETS_HPP

#include <cstdint>

namespace VCTR::SerialTelecoms::packets {

enum SerialPacketType : uint8_t {
  DataSend,        // Send the packet buffer contents.
  LinkInfo,        // Received Packet with RSSI, SNR and length info.
  PacketDataClear, // Clears packet data buffer.
  PacketData,      // Packet data. Max 255 bytes.
  UpdateMode,      // Places ExVectrLink into Update mode
  SetBaudRate,     // Sets the baud rate of the serial communication.
  Heartbeat,       // Heartbeat timeout (1s) will reset baud to 115200.
  ChannelBlocked,  // Cannot send data.
  ChannelFree,     // Can send data.
  Error,
};
};

namespace VCTR::SerialTelecoms::packets {
class SerialPacket_LinkInfo {
public:
  uint8_t rssi;
  uint8_t snr;
  uint8_t antenna;

  uint8_t linkQuality;

  bool dualLinkMode;
  uint8_t linkSpeed;

  uint8_t numBytes() { return 6; }
  void serialize(uint8_t *buffer) {
    buffer[0] = rssi;
    buffer[1] = snr;
    buffer[2] = antenna;
    buffer[3] = linkQuality;
    buffer[4] = dualLinkMode ? 1 : 0;
    buffer[5] = linkSpeed;
  }
  static SerialPacket_LinkInfo deserialize(const uint8_t *buffer) {
    SerialPacket_LinkInfo info;
    info.rssi = buffer[0];
    info.snr = buffer[1];
    info.antenna = buffer[2];
    info.linkQuality = buffer[3];
    info.dualLinkMode = buffer[4] == 1;
    info.linkSpeed = buffer[5];
    return info;
  }
};

} // namespace VCTR::SerialTelecoms::packets

#endif // EXVECTRLINK_SERIALTELECOMPACKETS_HPP