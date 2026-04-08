#include <Arduino.h>

#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/RadioI.hpp"
#include "ExVectrNetwork/datalink/sx1280/Sx1280_2.hpp"

#include "ExVectrLink/datalink/Sx1280Diversity.hpp"

namespace VCTR::ExVectrLink::datalink {

Sx1280Diversity::Sx1280Diversity(
    std::initializer_list<VCTR::network::datalink::Datalink_SX1280_V2 *>
        links) {
  if (links.size() == 0) {
    return;
  }
  for (auto link : links) {
    addDiversityLink(*link);
  }
}

void Sx1280Diversity::addDiversityLink(
    VCTR::network::datalink::Datalink_SX1280_V2 &link) {
  auto linkIndex = diversityLinks.size();
  diversityLinks.append({&link, {0, 0}});
  // link.addTransmitFinishedHandler([this]() {
  //   transmitting = 0;
  //   startReceiveOnAllLinks();
  // });
  link.addReceiveHandler(
      [this, linkIndex](const VCTR::network::DataPacket &dataframe) {
        auto time = Core::NOW();

        auto &linkInfo = diversityLinks[linkIndex];
        linkInfo.lastPacketInfo.rssi = linkInfo.link->lastPacketRSSI();
        linkInfo.lastPacketInfo.snr = linkInfo.link->lastPacketSNR();

        if (Core::NOW() - lastPacketReceivedTime >= 3 * Core::MILLISECONDS) {
          lastPacketReceivedTime = Core::NOW();
          receiveHandlers_.callHandlers(dataframe);
        }

        // lastReceivedPacketId = dataId;
        determineBestLink();
      });
}

const VCTR::network::datalink::Datalink_SX1280_V2 *
Sx1280Diversity::getDiversityLink(size_t index) const {
  if (index >= diversityLinks.size())
    return nullptr;
  return diversityLinks[index].link;
}

size_t Sx1280Diversity::getCurrentBestLinkIndex() const {
  return currentBestLinkIndex;
}

bool Sx1280Diversity::setDesignatedTxLink(
    const VCTR::network::datalink::Datalink_SX1280_V2 &link) {

  for (size_t i = 0; i < diversityLinks.size(); i++) {
    if (diversityLinks[i].link == &link) {
      designatedTxLink = i;
      return true;
    }
  }
  return false;
}

size_t Sx1280Diversity::getTxLinkIndex() const {
  return (designatedTxLink != (size_t)-1) ? designatedTxLink
                                          : currentBestLinkIndex;
}

bool Sx1280Diversity::transmitDataframe(
    const VCTR::network::DataPacket &dataframe) {
  if (diversityLinks.size() == 0) {
    return false;
  }

  auto txLinkIndex = getTxLinkIndex();

  auto &txLink = diversityLinks[txLinkIndex];
  stopReceiveOnAllLinks(txLinkIndex);
  bool result = txLink.link->transmitDataframe(dataframe);
  if (result) {
    transmitting = Core::NOW();
  }
  return result;
}

/**
 * @brief Get the maximum packet size that can be transmitted by the datalink.
 * @note packets over this size will be dropped and not transmitted.
 * @return size_t The maximum packet size in bytes.
 */
size_t Sx1280Diversity::getMaxPacketSize() const {
  size_t maxPacketSize = 0;
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    maxPacketSize =
        std::max(maxPacketSize, diversityLinks[i].link->getMaxPacketSize());
  }
  return maxPacketSize;
}

/**
 * @returns true if the datalink is currently blocked and cannot send
 * dataframes.
 */
bool Sx1280Diversity::isChannelBlocked() const {
  if (diversityLinks.size() == 0) {
    return true;
  }
  // Check the link that will actually be used for TX.
  return diversityLinks[getTxLinkIndex()].link->isChannelBlocked();
}

size_t Sx1280Diversity::getNumChannels() const {
  if (diversityLinks.size() == 0) {
    return 0;
  }
  return diversityLinks[0].link->getNumChannels();
}
size_t Sx1280Diversity::getCurrentChannel() const {
  if (diversityLinks.size() == 0) {
    return 0;
  }
  return diversityLinks[0].link->getCurrentChannel();
}
void Sx1280Diversity::setChannel(size_t channel) {
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    diversityLinks[i].link->setChannel(channel);
  }
}

int16_t Sx1280Diversity::lastPacketSNR() const {
  if (diversityLinks.size() == 0) {
    return 0;
  }
  return diversityLinks[currentBestLinkIndex].lastPacketInfo.snr;
}

void Sx1280Diversity::startReceiveOnAllLinks() {
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    diversityLinks[i].link->setStartReceive(true);
  }
}
void Sx1280Diversity::stopReceiveOnAllLinks(size_t exceptIndex) {
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    if (i == exceptIndex) {
      continue;
    }
    diversityLinks[i].link->setStartReceive(false);
  }
}

void Sx1280Diversity::setStartReceive(bool rxEnabled) {
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    diversityLinks[i].link->setStartReceive(rxEnabled);
  }
}

void Sx1280Diversity::setEnableTxRx(bool enable) {
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    diversityLinks[i].link->setEnableTxRx(enable);
  }
}

void Sx1280Diversity::setEnableAutoRx(bool enableAutoRx) {
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    diversityLinks[i].link->setEnableAutoRx(enableAutoRx);
  }
}

void Sx1280Diversity::determineBestLink() {
  // Require a new link to be at least kHysteresisDb better than the current
  // best to prevent rapid flapping when both radios have similar signal.
  static constexpr int16_t kHysteresisDb = 3;

  int16_t currentSnr = diversityLinks[currentBestLinkIndex].lastPacketInfo.snr;
  size_t bestLinkIndex = currentBestLinkIndex;

  for (size_t i = 0; i < diversityLinks.size(); i++) {
    if (i == currentBestLinkIndex)
      continue;
    int16_t snr = diversityLinks[i].lastPacketInfo.snr;
    if (snr > currentSnr + kHysteresisDb) {
      bestLinkIndex = i;
    }
  }

  currentBestLinkIndex = bestLinkIndex;
}

} // namespace VCTR::ExVectrLink::datalink