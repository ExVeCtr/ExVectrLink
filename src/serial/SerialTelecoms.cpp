#include <functional>

#include "ExVectrHAL/digital_io.hpp"

#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/print.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/time_definitions.hpp"

#include "ExVectrLink/serial/SerialTelecomPackets.hpp"

#include "ExVectrLink/serial/SerialTelecoms.hpp"

namespace VCTR::ExVectrLink {

using namespace VCTR::ExVectrLink::packets;

enum SerialByteType : uint8_t {
  StartByteA = 0x7E,
  StartByteB = 0x7C,
  EndByte = 0x7F,
};

SerialTelecoms::SerialTelecoms(HAL::DigitalIO &serialPort)
    : Core::Task_Periodic("Serial Communication", Core::MILLISECONDS * 100),
      serialPort(serialPort) {
  Core::getSystemScheduler().addTask(*this);
}

void SerialTelecoms::taskInit() {
  serialReadState = SerialReadState::WaitingForStartByteA;
  recievePacketData.clear();

  addSerialPacketHandler(SerialPacketType::SetBaudRate,
                         [this](const Core::ListArray<uint8_t> &data) {
                           if (data.size() == 4) {
                             uint32_t baudRate = *((uint32_t *)data.getPtr());
                             if (baudRate != baudrate) {
                               sendSerialPacket(SerialPacketType::SetBaudRate,
                                                data);
                               forcePacketSendNow();
                               setPortBaudRate(baudRate);
                             }
                           }
                         });

  addSerialPacketHandler<SerialPacket_Heartbeat>(
      [this](const SerialPacket_Heartbeat &packet) {
        isOtherEndSerialConnected = packet.isConnected;
      });
}

void SerialTelecoms::taskCheck() {
  if (serialPort.readable() > 0 || sendDataBuffer.size() > 0) {
    setDeadline(Core::NOW());
  }
}

void SerialTelecoms::taskThread() {
  int64_t loopStart = Core::NOW();

  while (serialPort.readable() > 0 &&
         Core::NOW() - loopStart < 1 * Core::MILLISECONDS) {
    lastSerialByteTime = loopStart;
    uint8_t incomingByte;
    serialPort.readByte(incomingByte);
    decodeSerialByte(incomingByte);
  }

  while (sendDataBuffer.size() > 0 &&
         Core::NOW() - loopStart < 1 * Core::MILLISECONDS) {
    uint8_t byteToSend;
    sendDataBuffer.takeFront(byteToSend);
    serialPort.writeByte(byteToSend);
  }

  if (lastSerialByteTime != lastLoopTime && baudrate != standardBaudrate &&
      loopStart - lastSerialByteTime > 1000 * Core::MILLISECONDS) {
    LOG_MSG("Serial communication timeout. Resetting serial state and baud "
            "rate. \n");
    serialReadState = SerialReadState::WaitingForStartByteA;
    recievePacketData.clear();
    setPortBaudRate(standardBaudrate);
  }

  if (isSerialConnected &&
      loopStart - lastValidPacketTime > 5000 * Core::MILLISECONDS) {
    LOG_MSG("Connection timeout. No valid packets received for 5s. Marking as "
            "disconnected. \n");
    isSerialConnected = false;
  }

  if (loopStart - lastHeartbeatTime > 500 * Core::MILLISECONDS) {
    lastHeartbeatTime = loopStart;
    sendSerialPacket<SerialPacket_Heartbeat>({isSerialConnected});
  }

  lastLoopTime = loopStart;
}

void SerialTelecoms::addSerialPacketHandler(
    const SerialPacketType &type,
    std::function<void(const Core::ListArray<uint8_t> &data)> handler) {
  serialPacketHandlers.append({type, handler});
}

void SerialTelecoms::sendSerialPacket(const SerialPacketType &type,
                                      const void *data, size_t numBytes) {
  if (numBytes > 255) {
    LOG_MSG("Packet data was over 255 bytes. Not sending packet. \n");
    return;
  }
  if (sendDataBuffer.size() + numBytes + 5 > 1024) {
    LOG_MSG(
        "Send buffer overflow. Not sending packet. Consider increasing buffer "
        "size or sending less data.\n");
    return;
  }

  sendDataBuffer.placeBack(static_cast<uint8_t>(SerialByteType::StartByteA));
  sendDataBuffer.placeBack(
      static_cast<uint8_t>(SerialByteType::StartByteB + ExVectrLinkVersion));
  sendDataBuffer.placeBack(static_cast<uint8_t>(type));
  sendDataBuffer.placeBack(numBytes);
  for (size_t i = 0; i < numBytes; i++) {
    sendDataBuffer.placeBack(((uint8_t *)data)[i]);
  }
  sendDataBuffer.placeBack(static_cast<uint8_t>(SerialByteType::EndByte));
  lastPacketSendTime = Core::NOW();
}

void SerialTelecoms::sendSerialPacket(const SerialPacketType &type,
                                      const Core::ListArray<uint8_t> &data) {
  sendSerialPacket(type, data.getPtr(), data.size());
}

bool SerialTelecoms::isConnected() const { return isSerialConnected; }

bool SerialTelecoms::isOtherEndConnected() const {
  return isOtherEndSerialConnected;
}

void SerialTelecoms::forcePacketSendNow(int64_t timeout) {
  auto start = Core::NOW();
  while (sendDataBuffer.size() > 0 && Core::NOW() - start < timeout) {
    uint8_t byteToSend;
    sendDataBuffer.takeFront(byteToSend);
    serialPort.writeByte(byteToSend);
  }
}

void SerialTelecoms::decodeSerialByte(uint8_t incomingByte) {
  switch (serialReadState) {
  case SerialReadState::WaitingForStartByteA:
    if (incomingByte == SerialByteType::StartByteA) {
      serialReadState = SerialReadState::WaitingForStartByteB;
    }
    break;

  case SerialReadState::WaitingForStartByteB:
    if (incomingByte ==
        (uint8_t)SerialByteType::StartByteB + ExVectrLinkVersion) {
      serialReadState = SerialReadState::WaitingForPacketType;
    } else {
      serialReadState = SerialReadState::WaitingForStartByteA;
    }
    break;

  case SerialReadState::WaitingForPacketType:
    currentPacketType = static_cast<SerialPacketType>(incomingByte);
    serialReadState = SerialReadState::WaitingForPacketLength;
    break;

  case SerialReadState::WaitingForPacketLength:
    packetLength = incomingByte;
    serialReadState = SerialReadState::ReadingPacketData;
    recievePacketData.clear();
    break;

  case SerialReadState::ReadingPacketData:
    if (incomingByte == SerialByteType::EndByte) {
      serialReadState = SerialReadState::WaitingForStartByteA;
      lastValidPacketTime = Core::NOW();
      isSerialConnected = true;
      // Process the packet
      for (size_t i = 0; i < serialPacketHandlers.size(); i++) {
        if (serialPacketHandlers[i].packetType == currentPacketType) {
          serialPacketHandlers[i].processFunction(recievePacketData);
        }
      }
    } else if (recievePacketData.size() < 255) {
      recievePacketData.append(incomingByte);
    } else {
      serialReadState = SerialReadState::WaitingForStartByteA;
    }
    break;

  default:
    serialReadState = SerialReadState::WaitingForStartByteA;
    break;
  }
}

void SerialTelecoms::setPortBaudRate(uint32_t baudrate) {
  this->baudrate = baudrate;
  serialPort.setInputParam(HAL::IO_PARAM_t::SPEED, baudrate);
  serialPort.setOutputParam(HAL::IO_PARAM_t::SPEED, baudrate);
  LOG_MSG("Setting port baudrate to: %d\n", baudrate);
}

} // namespace VCTR::ExVectrLink

namespace VCTR::ExVectrLink /* SerialTelecomsDatalink */ {

SerialTelecomsDatalink::SerialTelecomsDatalink(SerialTelecoms &telecoms)
    : telecoms(telecoms) {}

bool SerialTelecomsDatalink::transmitDataframe(
    const VCTR::network::DataPacket &dataframe) {
  if (dataframe.payload.size() > getMaxPacketSize()) {
    LOG_MSG("Dataframe size exceeds maximum packet size. Not transmitting.\n");
    return false;
  }
  telecoms.sendSerialPacket(SerialPacketType::PacketData, dataframe.payload);
  return true;
}

/**
 * @brief Get the maximum packet size that can be transmitted by the datalink.
 * @note packets over this size will be dropped and not transmitted.
 * @return size_t The maximum packet size in bytes.
 */
size_t SerialTelecomsDatalink::getMaxPacketSize() const { return 250; }

/**
 * @returns true if the datalink is currently blocked and cannot send
 * dataframes.
 */
bool SerialTelecomsDatalink::isChannelBlocked() const { return false; }

void SerialTelecomsDatalink::initialize() {
  if (!initialized_) {
    addHandlers();
    initialized_ = true;
  }
}

void SerialTelecomsDatalink::addHandlers() {
  telecoms.addSerialPacketHandler(SerialPacketType::PacketData,
                                  [this](const Core::ListArray<uint8_t> &data) {
                                    network::DataPacket packet(data);
                                    receiveHandlers_.callHandlers(packet);
                                  });
  telecoms.addSerialPacketHandler<SerialPacket_LinkInfo>(
      [this](const SerialPacket_LinkInfo &packet) {
        linkinfo = {packet.rssi, packet.snr, packet.antenna, packet.lossRate,
                    packet.dualLinkMode};
      });
}

void SerialTelecomsDatalink::setTxPower(uint8_t txPower) {
  telecoms.sendSerialPacket<SerialPacket_SetTxPower>(
      SerialPacket_SetTxPower{txPower});
}

void SerialTelecomsDatalink::setModulationPreset(
    VCTR::ExVectrLink::datalink::ModulationPresets preset) {
  uint8_t presetByte = static_cast<uint8_t>(preset);
  telecoms.sendSerialPacket<SerialPacket_SetModulationPreset>(
      SerialPacket_SetModulationPreset{presetByte});
}

void SerialTelecomsDatalink::setEnableFhss(bool enable, uint32_t seqKey) {
  telecoms.sendSerialPacket<SerialPacket_SetEnableFhss>(
      SerialPacket_SetEnableFhss{enable, seqKey});
}

// Channel index 0-9. Stops FHSS if enabled.
void SerialTelecomsDatalink::setLinkChannel(uint8_t channelIndex) {
  telecoms.sendSerialPacket<SerialPacket_SetLinkChannel>(
      SerialPacket_SetLinkChannel{channelIndex});
}

void SerialTelecomsDatalink::setMediaAccessKey(uint8_t mak) {
  telecoms.sendSerialPacket<SerialPacket_InitLink>(SerialPacket_InitLink{mak});
}

const VCTR::ExVectrLink::datalink::LinkInfo &
SerialTelecomsDatalink::getLinkInfo() const {
  return linkinfo;
}

} // namespace VCTR::ExVectrLink