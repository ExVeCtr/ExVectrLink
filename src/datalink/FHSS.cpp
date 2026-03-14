#include "ExVectrCore/CanSerialize.hpp"
#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/RadioI.hpp"
#include "ExVectrNetwork/datalink/sx1280/Sx1280.hpp"

#include "ExVectrLink/datalink/FHSS.hpp"

namespace VCTR::ExVectrLink::datalink {

enum class FHSSPacketType : uint8_t {
  Data,
  LinkInfo,
};

struct LinkInfoPacket {
  uint8_t rssi;
  uint8_t snr;
  uint8_t lq;
  uint8_t channel;
  uint8_t uid;

  void serialize(uint8_t *buffer) const {
    buffer[0] = rssi;
    buffer[1] = snr;
    buffer[2] = lq;
    buffer[3] = channel;
    buffer[4] = uid;
  }

  void deserialize(const uint8_t *buffer) {
    rssi = buffer[0];
    snr = buffer[1];
    lq = buffer[2];
    channel = buffer[3];
    uid = buffer[4];
  }

  size_t numBytes() { return 5; }
};

FHSS::FHSS(VCTR::network::datalink::RadioI &radioI)
    : Core::Task_Periodic("FHSS", 10 * Core::MILLISECONDS), radioLink(radioI) {

  Core::getSystemScheduler().addTask(*this);
}

void FHSS::setFhssKey(uint8_t newKey) {
  key = newKey;
  generateSequence();
}

uint8_t FHSS::getFhssKey() const { return key; }

void FHSS::enableFhss(bool enable) { fhssEnabled = enable; }

FHSSState FHSS::getFhssState() const { return fhssState; }

void FHSS::addChannelBlockedChangeHandler(
    VCTR::Core::HandlerGroup<bool, uint8_t>::HandlerFunction handler) {
  channelBlockedChangeHandlers.addHandler(handler);
}

bool FHSS::transmitDataframe(const VCTR::network::DataPacket &dataframe) {
  if (waitingForSendFinish) {
    return false;
  }
  packetToSend = dataframe;
  packetToSend.payload.append((uint8_t)FHSSPacketType::Data);
  channelBlockedChangeHandlers.callHandlers(true, 0);
  waitingForSendFinish = true;
  return true;
}

size_t FHSS::getMaxPacketSize() const { return radioLink.getMaxPacketSize(); }

bool FHSS::isChannelBlocked() const {
  return radioLink.isChannelBlocked() || waitingForSendFinish;
}

void FHSS::taskCheck() {
  if (waitingForSendFinish) {
    setDeadline(Core::NOW());
  }
}

void FHSS::taskInit() {
  radioLink.addReceiveHandler([this](const network::DataPacket &packet) {
    if (packet.payload.size() < 1) {
      return;
    }
    lastPacketReceivedTime = Core::NOW();
    auto packetType = (FHSSPacketType)packet.payload[packet.payload.size() - 1];
    if (packetType == FHSSPacketType::Data) {
      auto dataPacket = packet;
      dataPacket.payload.pop();
      receiveHandlers_.callHandlers(dataPacket);
    }
  });
}

void FHSS::taskThread() {
  if (!radioLink.isChannelBlocked()) {
    if (packetToSend.payload.size() > 0) {
      radioLink.transmitDataframe(packetToSend);
      packetToSend.payload.clear();
    } else if (Core::NOW() - lastFHSSPacketSentTime >
               500 * Core::MILLISECONDS) {
      sendFHSSPacket();
      lastFHSSPacketSentTime = Core::NOW();
    } else if (waitingForSendFinish) {
      waitingForSendFinish = false;
      channelBlockedChangeHandlers.callHandlers(false, 10);
    }
  }

  if (Core::NOW() - lastPacketReceivedTime > 200 * Core::MILLISECONDS) {
    fhssState = FHSSState::Searching;
  } else {
    fhssState = FHSSState::Synced;
  }
}

void FHSS::generateSequence() {} // For now nothing

void FHSS::sendFHSSPacket() {
  LinkInfoPacket linkInfo;
  linkInfo.rssi = 0;
  linkInfo.snr = 0;
  linkInfo.lq = 0;
  linkInfo.channel = radioLink.getCurrentChannel();
  linkInfo.uid = 0;
  network::DataPacket packet;
  packet.payload.setSize(linkInfo.numBytes() + 1);
  linkInfo.serialize(packet.payload.getPtr());
  packet.payload[packet.payload.size() - 1] = (uint8_t)FHSSPacketType::LinkInfo;
  radioLink.transmitDataframe(packet);
}

} // namespace VCTR::ExVectrLink::datalink