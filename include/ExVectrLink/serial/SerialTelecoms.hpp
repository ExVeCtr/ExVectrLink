#ifndef EXVECTRLINK_SERIALTELECOMS_HPP
#define EXVECTRLINK_SERIALTELECOMS_HPP

#include <cstdint>
#include <functional>

#include "ExVectrHAL/digital_io.hpp"

#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrLink/ExVectrLinkI.hpp"
#include "ExVectrLink/serial/SerialTelecomPackets.hpp"

namespace VCTR::ExVectrLink /* ExVectrLinkSerialTelecoms */ {

/// @brief  Current ExVectrLink version.
/// Will be incremented if incompatible changes have been made.
constexpr uint8_t ExVectrLinkVersion = 4;

class SerialTelecoms : public Core::Task_Periodic {
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
    VCTR::ExVectrLink::packets::SerialPacketType packetType;
    // Packet data. Max 255 bytes.
    Core::ListArray<uint8_t> packetData;
  };

  struct SerialPacketHandler {
    // Command type to call handler on.
    VCTR::ExVectrLink::packets::SerialPacketType packetType;
    std::function<void(const Core::ListArray<uint8_t> &data)> processFunction;
  };

public:
  SerialTelecoms(HAL::DigitalIO &serialPort);

  void taskInit() override;
  void taskCheck() override;
  void taskThread() override;

  void addSerialPacketHandler(
      const VCTR::ExVectrLink::packets::SerialPacketType &type,
      std::function<void(const Core::ListArray<uint8_t> &data)> handler);

  template <VCTR::ExVectrLink::packets::SerializablePacket T>
  void addSerialPacketHandler(std::function<void(const T &packet)> handler) {
    addSerialPacketHandler(T().getPacketType(),
                           [handler](const Core::ListArray<uint8_t> &data) {
                             handler(T::deserialize(data.getPtr()));
                           });
  }

  void
  sendSerialPacket(const VCTR::ExVectrLink::packets::SerialPacketType &type,
                   const void *data, size_t numBytes);
  void
  sendSerialPacket(const VCTR::ExVectrLink::packets::SerialPacketType &type,
                   const Core::ListArray<uint8_t> &data = {});

  template <VCTR::ExVectrLink::packets::SerializablePacket T>
  void sendSerialPacket(const T &packet) {
    Core::ListArray<uint8_t> data(packet.numBytes());
    packet.serialize(data.getPtr());
    sendSerialPacket(packet.getPacketType(), data);
  }

  /// @brief If we can here the other side.
  bool isConnected() const;
  /// @brief If the other side can hear us.
  bool isOtherEndConnected() const;

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
  VCTR::ExVectrLink::packets::SerialPacketType currentPacketType;
  uint8_t packetLength;

  Core::ListBuffer<uint8_t, 1024> sendDataBuffer;

  int64_t lastSerialByteTime = 0;
  int64_t lastValidPacketTime = 0;
  int64_t lastPacketSendTime = 0;
  int64_t lastHeartbeatTime = 0;
  int64_t lastLoopTime = 0;

  Core::ListArray<SerialPacketHandler> serialPacketHandlers;

  uint32_t baudrate = standardBaudrate;

  bool isSerialConnected = false;
  bool isOtherEndSerialConnected = false;
};

} // namespace VCTR::ExVectrLink

namespace VCTR::ExVectrLink /* SerialTelecomsDatalink */ {

class SerialTelecomsDatalink : public VCTR::ExVectrLink::ExVectrLinkI {
public:
  SerialTelecomsDatalink(SerialTelecoms &telecoms);

  // --------------- DatalinkI implementation ---------------

  bool transmitDataframe(const VCTR::network::DataPacket &dataframe) override;

  /**
   * @brief Get the maximum packet size that can be transmitted by the datalink.
   * @note packets over this size will be dropped and not transmitted.
   * @return size_t The maximum packet size in bytes.
   */
  size_t getMaxPacketSize() const override;

  /**
   * @returns true if the datalink is currently blocked and cannot send
   * dataframes.
   */
  bool isChannelBlocked() const override;

  // --------------- ExVectrLinkI implementation ---------------

  void setTxPower(uint8_t txPower) override;

  void setModulationPreset(
      VCTR::ExVectrLink::datalink::ModulationPresets preset) override;

  void setEnableFhss(bool enable, uint32_t seqKey = 0) override;

  // Channel index 0-9. Stops FHSS if enabled.
  void setLinkChannel(uint8_t channelIndex) override;

  void setMediaAccessKey(uint8_t mak) override;

  const VCTR::ExVectrLink::datalink::LinkInfo &getLinkInfo() const override;

private:
  void addHandlers();

  SerialTelecoms &telecoms;

  VCTR::ExVectrLink::datalink::LinkInfo linkinfo;
};

} // namespace VCTR::ExVectrLink

#endif // EXVECTRLINK_SERIALTELECOMS_HPP