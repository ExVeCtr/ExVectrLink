#ifndef EXVECTRLINK_SERIALTELECOMS_HPP
#define EXVECTRLINK_SERIALTELECOMS_HPP

#include <cstdint>
#include <functional>

#include "ExVectrHAL/digital_io.hpp"

#include "ExVectrCore/data_buffer.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrLink/SerialTelecomPackets.hpp"

namespace VCTR::SerialTelecoms {

/// @brief  Current ExVectrLink version.
/// Will be incremented if incompatible changes have been made.
constexpr uint8_t ExVectrLinkVersion = 3;

class ExVectrLinkSerialTelecoms : public Core::Task_Periodic {
private:
  static constexpr uint32_t standardBaudrate = 115200;

  enum class SerialReadState {
    WaitingForStartByteA,
    WaitingForStartByteB,
    WaitingForPacketType,
    WaitingForPacketLength,
    ReadingPacketData,
    WaitingForEndByte
  };

  struct SerialPacketCommand {
    // Command type.
    VCTR::SerialTelecoms::packets::SerialPacketType packetType;
    // Packet data. Max 255 bytes.
    Core::ListArray<uint8_t> packetData;
  };

  struct SerialPacketHandler {
    // Command type to call handler on.
    VCTR::SerialTelecoms::packets::SerialPacketType packetType;
    std::function<void(const Core::ListArray<uint8_t> &data)> processFunction;
  };

public:
  ExVectrLinkSerialTelecoms(HAL::DigitalIO &serialPort);

  void taskInit() override;
  void taskCheck() override;
  void taskThread() override;

  void addSerialPacketHandler(
      const VCTR::SerialTelecoms::packets::SerialPacketType &type,
      std::function<void(const Core::ListArray<uint8_t> &data)> handler);

  template <SerializablePacket T>
  void addSerialPacketHandler(std::function<void(const T &packet)> handler) {
    addSerialPacketHandler(T().getPacketType(),
                           [handler](const Core::ListArray<uint8_t> &data) {
                             handler(T::deserialize(data.getPtr()));
                           });
  }

  void
  sendSerialPacket(const VCTR::SerialTelecoms::packets::SerialPacketType &type,
                   const void *data, size_t numBytes);
  void
  sendSerialPacket(const VCTR::SerialTelecoms::packets::SerialPacketType &type,
                   const Core::ListArray<uint8_t> &data = {});

  template <SerializablePacket T> void sendSerialPacket(const T &packet) {
    Core::ListArray<uint8_t> data(packet.numBytes());
    packet.serialize(data.getPtr());
    sendSerialPacket(packet.getPacketType(), data);
  }

  bool isConnected() const;

  /**
   * @brief Will block and send everything currently waiting in the buffer.
   */
  void forcePacketSendNow(int64_t timeout = 500 * Core::MILLISECONDS);

private:
  void decodeSerialByte(uint8_t incomingByte);

  void setPortBaudRate(uint32_t baudrate);

  HAL::DigitalIO &serialPort;

  SerialReadState serialReadState = SerialReadState::WaitingForStartByteA;

  Core::ListArray<uint8_t> recievePacketData;
  VCTR::SerialTelecoms::packets::SerialPacketType currentPacketType;
  uint8_t packetLength;

  Core::ListBuffer<uint8_t, 1024> sendDataBuffer;

  int64_t lastSerialByteTime = 0;
  int64_t lastValidPacketTime = 0;
  int64_t lastPacketSendTime = 0;
  int64_t lastLoopTime = 0;

  Core::ListArray<SerialPacketHandler> serialPacketHandlers;

  uint32_t baudrate = standardBaudrate;

  bool isSerialConnected = false;
};

} // namespace VCTR::SerialTelecoms

#endif // EXVECTRLINK_SERIALTELECOMS_HPP