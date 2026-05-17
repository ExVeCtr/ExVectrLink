#include <cstring>
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
  if (disabled) {
    return;
  }
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
  if (disabled) {
    return;
  }
  if (serialPort.readable() > 0 || sendDataBuffer.size() > 0) {
    setDeadline(Core::NowNs());
  }
}

void SerialTelecoms::taskThread() {
  if (disabled) {
    return;
  }
  int64_t loopStart = Core::NowNs();

  const auto bufferSize = 10;
  auto readData = [this, &loopStart]() {
    // Bulk read incoming serial data
    uint8_t readBuffer[bufferSize];
    size_t available;
    while ((available = serialPort.readable()) > 0 &&
           Core::NowNs() - loopStart < 1 * Core::MILLISECONDS) {
      lastSerialByteTime = loopStart;
      size_t toRead =
          available < sizeof(readBuffer) ? available : sizeof(readBuffer);
      size_t bytesRead = serialPort.readData(readBuffer, toRead);
      for (size_t i = 0; i < bytesRead; i++) {
        decodeSerialByte(readBuffer[i]);
      }
    }
  };

  // Bulk write outgoing serial data
  auto writeData = [this, &loopStart]() {
    uint8_t writeBuffer[bufferSize];
    while (sendDataBuffer.size() > 0 &&
           Core::NowNs() - loopStart < 1 * Core::MILLISECONDS &&
           serialPort.writable() > 0) {
      size_t toSend = sendDataBuffer.size();
      size_t writable = serialPort.writable();
      size_t chunkSize =
          toSend < sizeof(writeBuffer) ? toSend : sizeof(writeBuffer);
      if (chunkSize > writable) {
        chunkSize = writable;
      }
      for (size_t i = 0; i < chunkSize; i++) {
        writeBuffer[i] = sendDataBuffer[i];
      }
      size_t bytesWritten = serialPort.writeData(writeBuffer, chunkSize);
      if (bytesWritten == 0)
        break;
      sendDataBuffer.removeFront(bytesWritten);
    }
  };

  if (readWriteSwitch) {
    readData();
    writeData();
  } else {
    writeData();
    readData();
  }
  readWriteSwitch = !readWriteSwitch;

  if (lastSerialByteTime != lastLoopTime && baudrate != standardBaudrate &&
      loopStart - lastSerialByteTime > 1000 * Core::MILLISECONDS) {
    LOG_MSG("Serial communication timeout. Resetting serial state and baud "
            "rate. \n");
    serialReadState = SerialReadState::WaitingForStartByteA;
    recievePacketData.clear();
    setPortBaudRate(standardBaudrate);
  }

  if (isSerialConnected &&
      loopStart - lastValidPacketTime > 500 * Core::MILLISECONDS) {
    LOG_MSG(
        "Connection timeout. No valid packets received for 500ms. Marking as "
        "disconnected. \n");
    isSerialConnected = false;
  }

  if (loopStart - lastHeartbeatTime > 100 * Core::MILLISECONDS) {
    lastHeartbeatTime = loopStart;
    sendSerialPacket<SerialPacket_Heartbeat>({isSerialConnected});
  }

  lastLoopTime = loopStart;
}

void SerialTelecoms::setDisabled() { disabled = true; }

void SerialTelecoms::addSerialPacketHandler(
    const SerialPacketType &type,
    std::function<void(const Core::ListArray<uint8_t> &data)> handler) {
  if (disabled) {
    return;
  }
  serialPacketHandlers.append({type, handler});
}

void SerialTelecoms::sendSerialPacket(const SerialPacketType &type,
                                      const void *data, size_t numBytes) {
  if (disabled) {
    return;
  }
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
  lastPacketSendTime = Core::NowNs();
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
  if (disabled) {
    return;
  }
  auto start = Core::NowNs();
  uint8_t writeBuffer[256];
  while (sendDataBuffer.size() > 0 && Core::NowNs() - start < timeout) {
    size_t toSend = sendDataBuffer.size();
    size_t chunkSize =
        toSend < sizeof(writeBuffer) ? toSend : sizeof(writeBuffer);
    for (size_t i = 0; i < chunkSize; i++) {
      writeBuffer[i] = sendDataBuffer[i];
    }
    size_t bytesWritten = serialPort.writeData(writeBuffer, chunkSize);
    if (bytesWritten == 0)
      break;
    sendDataBuffer.removeFront(bytesWritten);
  }
}

void SerialTelecoms::decodeSerialByte(uint8_t incomingByte) {
  if (disabled) {
    return;
  }
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
      lastValidPacketTime = Core::NowNs();
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
  if (disabled) {
    return;
  }
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
                                    receiveHandlers_.callHandlers(data);
                                  });
  telecoms.addSerialPacketHandler<SerialPacket_LinkInfo>(
      [this](const SerialPacket_LinkInfo &packet) {
        linkinfo.rssi = packet.local.rssi;
        linkinfo.snr = packet.local.snr;
        linkinfo.antenna = packet.local.antenna;
        linkinfo.lossRate = packet.local.lossRate;
        linkinfo.linkQuality = packet.local.linkQuality;
        linkinfo.txPower = packet.local.txPower;
        linkinfo.remoteRssi = packet.remote.rssi;
        linkinfo.remoteSnr = packet.remote.snr;
        linkinfo.remoteAntenna = packet.remote.antenna;
        linkinfo.remoteLossRate = packet.remote.lossRate;
        linkinfo.remoteLinkQuality = packet.remote.linkQuality;
        linkinfo.remoteTxPower = packet.remote.txPower;
        linkinfo.remoteValid = packet.remoteValid;
        linkinfo.remoteDeviceTime = packet.remoteDeviceTime;
        linkinfo.remoteDesyncCount = packet.remoteDesyncCount;
        linkinfo.dualLinkMode = false;
      });

  telecoms
      .addSerialPacketHandler<ExVectrLink::packets::SerialPacket_FhssSyncState>(
          [this](
              const ExVectrLink::packets::SerialPacket_FhssSyncState &packet) {
            isConnected_ = packet.synced;
          });
}

void SerialTelecomsDatalink::setTxPower(uint8_t txPower, bool dynamicPower) {
  SerialPacket_SetPowerParams packet{};
  packet.txPower = txPower;
  packet.maxDynPower = 0;
  packet.minDynPower = 0;
  packet.enableDynamicPower = dynamicPower;
  telecoms.sendSerialPacket<SerialPacket_SetPowerParams>(packet);
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

bool SerialTelecomsDatalink::isConnected() { return isConnected_; }

const VCTR::ExVectrLink::datalink::LinkInfo &
SerialTelecomsDatalink::getLinkInfo() const {
  return linkinfo;
}

} // namespace VCTR::ExVectrLink