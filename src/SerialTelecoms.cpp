#include <functional>

#include "ExVectrHAL/digital_io.hpp"

#include "ExVectrCore/print.hpp"
#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/time_definitions.hpp"

#include "ExVectrLink/SerialTelecoms.hpp"

#include "ExVectrCore/list_array.hpp"

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
          uint32_t baudRate =
              (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
          serialPort.setInputParam(HAL::IO_PARAM_t::SPEED, baudRate);
          serialPort.setOutputParam(HAL::IO_PARAM_t::SPEED, baudRate);
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

  if (lastSerialByteTime != lastLoopTime &&
      loopStart - lastSerialByteTime > 1000 * Core::MILLISECONDS) {
    // If it's been more than 1s. We should reset everything.
    // We also check last loop time to avoid scheduling hicks causing this to
    // trigger.
    LOG_MSG("Serial communication timeout. Resetting serial state and baud "
            "rate. \n");
    serialReadState = SerialReadState::WaitingForStartByteA;
    recievePacketData.clear();
    serialPort.setInputParam(HAL::IO_PARAM_t::SPEED, 115200);
    serialPort.setOutputParam(HAL::IO_PARAM_t::SPEED, 115200);
  }

  lastLoopTime = loopStart;
}

void ExVectrLinkSerialTelecoms::addSerialPacketHandler(
    const SerialPacketType &type,
    std::function<void(uint8_t radioNum, const Core::ListArray<uint8_t> &data)>
        handler) {
  serialPacketHandlers.append({type, handler});
}

void ExVectrLinkSerialTelecoms::sendSerialPacket(
    const SerialPacketType &type, uint8_t radioNum,
    const Core::ListArray<uint8_t> &data) {

  if (data.size() > 255) {
    LOG_MSG("Packet data was over 255 bytes. Not sending packet. \n");
    return;
  }
  if (sendDataBuffer.size() + data.size() + 5 > 1024) {
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
  sendDataBuffer.placeBack(data.size());
  for (size_t i = 0; i < data.size(); i++) {
    sendDataBuffer.placeBack(data[i]);
  }
  sendDataBuffer.placeBack(static_cast<uint8_t>(SerialByteType::EndByte));
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

} // namespace VCTR::SerialTelecoms