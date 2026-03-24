#include <Arduino.h>

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

FHSS::FHSS(VCTR::network::datalink::RadioI &radioI)
    : Core::Task_Periodic("FHSS", 100 * Core::MILLISECONDS), radioLink(radioI) {

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
  packetToSend.payload.append((uint8_t)txPacketCount);
  packetToSend.payload.append((uint8_t)linkQuality);
  packetToSend.payload.append((uint8_t)key);
  return true;
}

size_t FHSS::getMaxPacketSize() const { return radioLink.getMaxPacketSize(); }

bool FHSS::isChannelBlocked() const { return packetToSend.payload.size() > 0; }

bool FHSS::shouldHop() const {
  if (fhssState == FHSSState::Synced) {
    int64_t targetTime =
        isNextChannelRecv ? (hoppingInterval * 1) : hoppingInterval;
    return (Core::NOW() - lastHoppingTime + hoppingOffset) >= targetTime;
  } else {
    return (Core::NOW() - lastHoppingTime) > 10 * hoppingInterval;
  }
}

void FHSS::taskCheck() {
  if (shouldHop()) {
    setDeadline(Core::NOW());
  }
}

void FHSS::taskInit() {

  radioLink.addReceiveHandler([this](const network::DataPacket &packet) {
    if (packet.payload.size() < 4) {
      return;
    }

    auto packetKey = packet.payload[packet.payload.size() - 1];
    if (packetKey != key) {
      return;
    }

    auto recvLinkQuality = packet.payload[packet.payload.size() - 2];
    auto txPacketCountRecv = packet.payload[packet.payload.size() - 3];
    auto packetType = (FHSSPacketType)packet.payload[packet.payload.size() - 4];

    if (isRxSide) {
      txPacketCount = txPacketCountRecv;
    }

    otherEndLinkQuality = recvLinkQuality;

    updateTiming(packet.timestamp);

    lastPacketReceivedTime = Core::NOW();
    if (packetType == FHSSPacketType::Data) {
      auto dataPacket = packet;
      dataPacket.payload.popDiscard(4);
      receiveHandlers_.callHandlers(dataPacket);
    }
  });

  lastHoppingTime = Core::NOW();
  lastPacketReceivedTime = Core::NOW();
  generateSequence();
}

void FHSS::taskThread() {

  updateChannel();

  if (channelTxReady && !radioLink.isChannelBlocked()) {

    if (packetToSend.payload.size() > 0) {
      radioLink.transmitDataframe(packetToSend);
      packetToSend.payload.clear();
    } else {
      packetToSend.payload.append((uint8_t)FHSSPacketType::LinkInfo);
      packetToSend.payload.append((uint8_t)txPacketCount);
      packetToSend.payload.append((uint8_t)linkQuality);
      packetToSend.payload.append((uint8_t)key);
      radioLink.transmitDataframe(packetToSend);
      packetToSend.payload.clear();
    }
  }

  // Falsify here so we dont end up sending data near end of hop slot
  channelTxReady = false;
}

void FHSS::generateSequence() {

  channelSequence.clear();
  for (uint8_t i = 0; i < radioLink.getNumChannels(); i++) {
    channelSequence.append({i, i % 2 == 0});
  }
  currentSeqIndex = 0;
}

void FHSS::updateChannel() {

  // Tx is always synced.
  if (!isRxSide && fhssState != FHSSState::Synced) {
    fhssState = FHSSState::Synced;
  }

  if (isRxSide && fhssState == FHSSState::Synced &&
      Core::NOW() - lastPacketReceivedTime > 100 * hoppingInterval) {
    fhssState = FHSSState::Searching;
    hoppingOffset = 0;
    lastHoppingTime = Core::NOW();
    hoppingErrors.clear();
  }

  if (fhssState == FHSSState::Synced) {
    if (shouldHop()) {
      hopChannel();
    }
  } else if (shouldHop()) {
    hopChannel();
    lastHoppingTime = Core::NOW();
  }

  if (receiveTimestamps.size() > 1 &&
      (newRecv || Core::NOW() - lastUpdateTime > 1 * Core::SECONDS)) {
    lastUpdateTime = Core::NOW();
    newRecv = false;
    int64_t timeSpan = receiveTimestamps[receiveTimestamps.size() - 1] -
                       receiveTimestamps[0] + hoppingInterval;
    size_t expectedPackets = timeSpan / hoppingInterval;
    expectedPackets = expectedPackets - expectedPackets / rxPacketRatio;
    float quality = (float)receiveTimestamps.size() / expectedPackets * 255;
    if (quality > 255) {
      linkQuality = 255;
    } else {
      linkQuality = (uint8_t)quality;
    }
  } else if (receiveTimestamps.size() > 0 &&
             Core::NOW() - receiveTimestamps[receiveTimestamps.size() - 1] >
                 1 * Core::SECONDS) {
    linkQuality = 0;
    otherEndLinkQuality = 0;
  }

  if (receiveTimestamps.size() > 0) {
    while (receiveTimestamps.size() > 0 &&
           Core::NOW() - receiveTimestamps[0] > 1 * Core::SECONDS) {
      receiveTimestamps.removeFront();
    }
  }
}

void FHSS::updateTiming(int64_t revcStartTimestamp) {

  receiveTimestamps.placeBack(revcStartTimestamp, true);
  newRecv = true;
  channelReceived = true;

  if (fhssState == FHSSState::Searching) {
    fhssState = FHSSState::Synced;
    hoppingOffset = 0;
    hopOffsetConfidence = 1;
    lastHoppingTime = revcStartTimestamp;
    hoppingErrors.clear();
  } else if (isRxSide) {
    int64_t expectedReceiveTime = lastHoppingTime + hoppingOffset;
    int64_t offsetError = revcStartTimestamp - expectedReceiveTime;

    hoppingErrors.placeBack(offsetError, true);

    int64_t offsetCorrection = 0;
    if (hoppingErrors.size() > 3) {
      auto variance = hoppingErrors.getStandardDeviation();
      VCTR::Core::ListBuffer<int64_t, 50> filteredErrors;
      for (size_t i = 0; i < hoppingErrors.size(); i++) {
        if (abs(hoppingErrors[i]) < variance * 2) {
          filteredErrors.placeBack(hoppingErrors[i]);
        }
      }
      auto averageError = filteredErrors.getAverage();
      offsetCorrection = offsetError / 100;
    } else {
      offsetCorrection = offsetError / 2;
    }
    hoppingOffset += offsetCorrection;

    // if (abs(offsetError) < hoppingInterval / hopOffsetConfidence * 2) {
    //   const int maxConfidence = 50;
    //   hopOffsetConfidence++;
    //   if (hopOffsetConfidence > maxConfidence) {
    //     hopOffsetConfidence = maxConfidence;
    //   }
    //   hoppingOffset += offsetError / hopOffsetConfidence;
    // }
    // lastHoppingTime = revcStartTimestamp;
  }

  // Premature hop if we need to receive on next channel.
  // This way we are ready as soon as possible.
  // if (isNextChannelRecv && fhssState == FHSSState::Synced &&
  //     Core::NOW() - lastHoppingTime > hoppingInterval * 0.5) {
  //   hopChannel();
  // }
}

void FHSS::hopChannel() {

  lastHoppingTime += hoppingInterval;

  if (!channelReceived && isReceiveChannel && hopOffsetConfidence > 1) {
    hopOffsetConfidence--;
  }

  currentSeqIndex = (currentSeqIndex + 1) % channelSequence.size();
  radioLink.setChannel(channelSequence[currentSeqIndex].channel);

  channelTxReady = true;
  isNextChannelRecv = false;
  if (txPacketCount == 0) {
    channelTxReady = false;
  }

  isReceiveChannel = !channelTxReady;

  txPacketCount++;
  if (txPacketCount > rxPacketRatio) {
    txPacketCount = 0;
  }

  if (txPacketCount == 0) {
    isNextChannelRecv = true;
  }

  if (isRxSide) {
    channelTxReady = !channelTxReady;
    isNextChannelRecv = !isNextChannelRecv;
  }
}

// int64_t FHSS::getDio1Timestamp() {
//   if (getDio1TimestampFunc != nullptr) {
//     return getDio1TimestampFunc();
//   }
//   return 0;
// }

void FHSS::updateHopping() {}

} // namespace VCTR::ExVectrLink::datalink