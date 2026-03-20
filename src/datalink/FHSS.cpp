#include "ExVectrCore/CanSerialize.hpp"
#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/RadioI.hpp"
#include "ExVectrNetwork/datalink/sx1280/Sx1280_2.hpp"

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
    : Core::Task_Periodic("FHSS", 1 * Core::MILLISECONDS), radioLink(radioI) {

  Core::getSystemScheduler().addTask(*this);
}

void FHSS::setFhssKey(uint8_t newKey) {
  key = newKey;
  generateSequence();
}

uint8_t FHSS::getFhssKey() const { return key; }

FHSSState FHSS::getFhssState() const { return fhssState; }

void FHSS::setReceiveStartTime(int64_t time) { packetReceiveStartTime = time; }

void FHSS::setNumReceiveChannels(size_t num) { numReceiveChannels = num; }

void FHSS::setHoppingInterval(int64_t interval) { hoppingInterval = interval; }

void FHSS::setHoppingSyncInterval(int64_t offset) { hoppingOffset = offset; }

void FHSS::updateReceiveStartTime(int64_t time) {
  lastPacketReceiveStartTime = time;
}

void FHSS::setIsRxSide(bool isRxSide) { this->isRxSide = isRxSide; }

bool FHSS::transmitDataframe(const VCTR::network::DataPacket &dataframe) {
  if (packetToSend.payload.size() > 0) {
    return false;
  }
  packetToSend = dataframe;
  packetToSend.payload.append((uint8_t)FHSSPacketType::Data);
  packetToSend.payload.append((uint8_t)key);
  return true;
}

size_t FHSS::getMaxPacketSize() const { return radioLink.getMaxPacketSize(); }

bool FHSS::isChannelBlocked() const {
  return radioLink.isChannelBlocked() || packetToSend.payload.size() > 0;
}

void FHSS::taskCheck() {
  if (packetToSend.payload.size() > 0) {
    setDeadline(Core::NOW());
  }
}

void FHSS::taskInit() {

  radioLink.addReceiveHandler([this](const network::DataPacket &packet) {
    if (packet.payload.size() < 2) {
      return;
    }

    channelReady = false;

    auto packetKey = packet.payload[packet.payload.size() - 1];
    if (packetKey != key) {
      return;
    }
    auto packetType = (FHSSPacketType)packet.payload[packet.payload.size() - 2];

    // Store the preamble-detect timestamp so updateTiming() can align the
    // local hop schedule to the transmitter's slot boundaries.
    lastPacketReceiveStartTime = packet.timestamp;
    updateTiming();

    lastPacketReceivedTime = Core::NOW();
    if (packetType == FHSSPacketType::Data) {
      auto dataPacket = packet;
      dataPacket.payload.popDiscard(2);
      receiveHandlers_.callHandlers(dataPacket);
    }
  });

  // Anchor the hop timer to the current time so the very first call to
  // updateHopping() doesn't see a huge elapsed time and fast-hops.
  lastHoppingTime = Core::NOW();
  lastPacketReceivedTime = Core::NOW();
  generateSequence();
}

void FHSS::taskThread() {

  updateChannel();

  if (channelReady && !radioLink.isChannelBlocked()) {
    channelReady = false;

    if (packetToSend.payload.size() > 0) {

      // LOG_MSG("Transmitting FHSS packet. Payload size: %d\n",
      //  packetToSend.payload.size());
      radioLink.transmitDataframe(packetToSend);
      packetToSend.payload.clear();
    } else {
      // sendFHSSPacket();
    }
  }
}

void FHSS::generateSequence() {

  channelSequence.clear();
  for (uint8_t i = 0; i < radioLink.getNumChannels(); i++) {
    channelSequence.append({i, i % 2 == 0});
  }
  currentSeqIndex = 0;
}

void FHSS::sendFHSSPacket() {
  LinkInfoPacket linkInfo;
  linkInfo.rssi = 0;
  linkInfo.snr = 0;
  linkInfo.lq = 0;
  linkInfo.channel = radioLink.getCurrentChannel();
  linkInfo.uid = 0;
  network::DataPacket packet;
  packet.payload.setSize(linkInfo.numBytes() + 2);
  linkInfo.serialize(packet.payload.getPtr());
  packet.payload[packet.payload.size() - 2] = (uint8_t)FHSSPacketType::LinkInfo;
  packet.payload[packet.payload.size() - 1] = (uint8_t)key;
  radioLink.transmitDataframe(packet);
}

void FHSS::updateChannel() {
  if (fhssState == FHSSState::Synced) {
    updateHopping();
  } else {
    if (isRxSide) {
      updateSearch();
    } else {
      updateHopping();
    }
  }
}

void FHSS::updateTiming() {
  // Align our local hop schedule to when the received packet's preamble was
  // detected.  That moment is the start of the transmitter's TX slot, so
  // using it as lastHoppingTime keeps our slot boundaries in sync.
  lastHoppingTime = lastPacketReceiveStartTime;
  fhssState = FHSSState::Synced;
}

void FHSS::updateSearch() {
  int64_t time = Core::NOW();

  // TX side: keep channelReady true so we can transmit on every tick and
  // be discovered by the RX side regardless of which channel it is scanning.
  if (!isRxSide && !channelReady) {
    channelReady = true;
  }

  // RX side: scan through channels slowly, one channel per full sweep period.
  if (time - lastPacketReceivedTime >
      hoppingInterval * radioLink.getNumChannels()) {
    lastPacketReceivedTime = time;

    if (currentSeqIndex == 0) {
      currentSeqIndex = (uint8_t)(radioLink.getNumChannels() - 1);
    } else {
      currentSeqIndex--;
    }

    radioLink.setChannel(channelSequence[currentSeqIndex].channel);
  }
}

void FHSS::updateHopping() {

  int64_t time = Core::NOW();

  if (isRxSide && time - lastPacketReceivedTime > 500 * Core::MILLISECONDS) {
    fhssState = FHSSState::Searching;
    hoppingOffset = 0;
    return;
  }

  const int64_t slotElapsed = time - lastHoppingTime;

  if (slotElapsed >= hoppingInterval) {
    // ---- Slot boundary ----
    // Advance lastHoppingTime by exactly one interval so we never drift.
    lastHoppingTime += hoppingInterval;
    currentSeqIndex = (currentSeqIndex + 1) % channelSequence.size();
    // Only call setChannel if we haven't already pre-switched to this channel.
    if (radioLink.getCurrentChannel() !=
        channelSequence[currentSeqIndex].channel) {
      radioLink.setChannel(channelSequence[currentSeqIndex].channel);
    }
    // Open the TX/RX window for the side whose turn it is this slot.
    channelReady =
        channelSequence[currentSeqIndex].isReceiveChannel == isRxSide;

  } else if (slotElapsed >= (int64_t)(hoppingInterval * 0.8f)) {
    // ---- Pre-switch (80 % into the slot) ----
    // Tune to the NEXT channel early so the radio has time to settle before
    // the slot boundary where TX/RX actually happens.
    const uint8_t nextIndex = (currentSeqIndex + 1) % channelSequence.size();
    if (radioLink.getCurrentChannel() != channelSequence[nextIndex].channel) {
      radioLink.setChannel(channelSequence[nextIndex].channel);
    }
  }
}

} // namespace VCTR::ExVectrLink::datalink