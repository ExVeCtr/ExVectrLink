#ifndef EXVECTRLINK_SERIALTELECOMS_HPP
#define EXVECTRLINK_SERIALTELECOMS_HPP

#include <cstdint>
#include <functional>

#include "ExVectrHAL/digital_io.hpp"

#include "ExVectrCore/data_buffer.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

namespace VCTR::SerialTelecoms {

/// @brief  Current ExVectrLink version.
/// Will be incremented if incompatible changes have been made.
constexpr uint8_t ExVectrLinkVersion = 1;

enum SerialPacketType : uint8_t {
  DataSend,          // Send the packet buffer contents.
  LinkInfo,          // Received Packet with RSSI, SNR and length info.
  PacketDataClear,   // Clears packet data buffer.
  PacketData,        // Packet data. Max 255 bytes.
  UpdateMode,        // Places ExVectrLink into Update mode
  SetBaudRate,       // Sets the baud rate of the serial communication.
  Heartbeat,         // Heartbeat timeout (1s) will reset baud to 115200.
  ChannelBlocked,    // Cannot send data.
  ChannelFree,       // Can send data.
  DeviceTemperature, // Send device temperature.
  Error,
};

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
    SerialPacketType packetType;
    // Packet data. Max 255 bytes.
    Core::ListArray<uint8_t> packetData;
  };

  struct SerialPacketHandler {
    // Command type to call handler on.
    SerialPacketType packetType;
    std::function<void(const Core::ListArray<uint8_t> &data)> processFunction;
  };

public:
  ExVectrLinkSerialTelecoms(HAL::DigitalIO &serialPort);

  void taskInit() override;
  void taskCheck() override;
  void taskThread() override;

  void addSerialPacketHandler(
      const SerialPacketType &type,
      std::function<void(const Core::ListArray<uint8_t> &data)> handler);

  void sendSerialPacket(const SerialPacketType &type, const void *data,
                        size_t numBytes);
  void sendSerialPacket(const SerialPacketType &type,
                        const Core::ListArray<uint8_t> &data = {});

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
  SerialPacketType currentPacketType;
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

#endif // ExVectrLink_SerialTelecoms_HPP