#include <functional>

#include "ExVectrHAL/digital_io.hpp"

#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/print.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/time_definitions.hpp"

#include "ExVectrLink/SerialTelecoms.hpp"

namespace VCTR::SerialTelecoms {

enum SerialByteType : uint8_t {
  StartByteA = 0x7E,
  StartByteB = 0x7C,
  EndByte = 0x7F,
};

ExVectrLinkSerialTelecoms::ExVectrLinkSerialTelecoms(HAL::DigitalIO &serialPort)
    : Core::Task_Periodic("Serial Communication", Core::MILLISECONDS * 100),
      serialPort(serialPort) {
  Core::getSystemScheduler().addTask(*this);
}

void ExVectrLinkSerialTelecoms::taskInit() {
  serialReadState = SerialReadState::WaitingForStartByteA;
  recievePacketData.clear();

  addSerialPacketHandler(
      SerialPacketType::SetBaudRate,
      [this](uint8_t radioNum, const Core::ListArray<uint8_t> &data) {
        if (data.size() == 4) {
          uint32_t baudRate = *((uint32_t *)data.getPtr());
          if (baudRate != baudrate) {
            sendSerialPacket(SerialPacketType::SetBaudRate, 0, data);
            forcePacketSendNow();
            setPortBaudRate(baudRate);
          }
        }
      });
}

void ExVectrLinkSerialTelecoms::taskCheck() {
  if (serialPort.readable() > 0 || sendDataBuffer.size() > 0) {
    setDeadline(Core::NOW());
  }
}

void ExVectrLinkSerialTelecoms::taskThread() {
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

  if (loopStart - lastPacketSendTime > 500 * Core::MILLISECONDS) {
    lastPacketSendTime = loopStart;
    sendSerialPacket(SerialPacketType::Heartbeat, 0);
  }

  lastLoopTime = loopStart;
}

void ExVectrLinkSerialTelecoms::addSerialPacketHandler(
    const SerialPacketType &type,
    std::function<void(uint8_t radioNum, const Core::ListArray<uint8_t> &data)>
        handler) {
  serialPacketHandlers.append({type, handler});
}

void ExVectrLinkSerialTelecoms::sendSerialPacket(const SerialPacketType &type,
                                                 uint8_t radioNum,
                                                 const void *data,
                                                 size_t numBytes) {
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
  sendDataBuffer.placeBack(radioNum);
  sendDataBuffer.placeBack(numBytes);
  for (size_t i = 0; i < numBytes; i++) {
    sendDataBuffer.placeBack(((uint8_t *)data)[i]);
  }
  sendDataBuffer.placeBack(static_cast<uint8_t>(SerialByteType::EndByte));
  lastPacketSendTime = Core::NOW();
}

void ExVectrLinkSerialTelecoms::sendSerialPacket(
    const SerialPacketType &type, uint8_t radioNum,
    const Core::ListArray<uint8_t> &data) {
  sendSerialPacket(type, radioNum, data.getPtr(), data.size());
}

bool ExVectrLinkSerialTelecoms::isConnected() const {
  return isSerialConnected;
}

void ExVectrLinkSerialTelecoms::forcePacketSendNow(int64_t timeout) {
  auto start = Core::NOW();
  while (sendDataBuffer.size() > 0 && Core::NOW() - start < timeout) {
    uint8_t byteToSend;
    sendDataBuffer.takeFront(byteToSend);
    serialPort.writeByte(byteToSend);
  }
}

void ExVectrLinkSerialTelecoms::decodeSerialByte(uint8_t incomingByte) {
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
    serialReadState = SerialReadState::WaitingForRadioNum;
    break;

  case SerialReadState::WaitingForRadioNum:
    radioNum = incomingByte;
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
          serialPacketHandlers[i].processFunction(radioNum, recievePacketData);
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

void ExVectrLinkSerialTelecoms::setPortBaudRate(uint32_t baudrate) {
  this->baudrate = baudrate;
  serialPort.setInputParam(HAL::IO_PARAM_t::SPEED, baudrate);
  serialPort.setOutputParam(HAL::IO_PARAM_t::SPEED, baudrate);
  LOG_MSG("Setting port baudrate to: %d\n", baudrate);
}

} // namespace VCTR::SerialTelecoms