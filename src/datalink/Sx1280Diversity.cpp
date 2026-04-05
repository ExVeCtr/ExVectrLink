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
  link.addTransmitFinishedHandler([this]() {
    transmitting = 0;
    startReceiveOnAllLinks();
  });
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

bool Sx1280Diversity::transmitDataframe(
    const VCTR::network::DataPacket &dataframe) {
  if (diversityLinks.size() == 0) {
    return false;
  }

  auto &bestLink = diversityLinks[currentBestLinkIndex];
  stopReceiveOnAllLinks(currentBestLinkIndex);
  bool result = bestLink.link->transmitDataframe(dataframe);
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
  // Check if any diversity link reports blocked.
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    if (diversityLinks[i].link->isChannelBlocked()) {
      return true;
    }
  }
  return false;
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
    diversityLinks[i].link->setEnableTxRx(true);
  }
}
void Sx1280Diversity::stopReceiveOnAllLinks(size_t exceptIndex) {
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    if (i == exceptIndex) {
      continue;
    }
    diversityLinks[i].link->setEnableTxRx(false);
  }
}

void Sx1280Diversity::determineBestLink() {
  size_t bestLinkIndex = currentBestLinkIndex;
  size_t bestSnr = -200;

  for (size_t i = 0; i < diversityLinks.size(); i++) {
    const auto &linkInfo = diversityLinks[i];
    const auto &lnkSnr = linkInfo.lastPacketInfo.snr;

    if (lnkSnr > bestSnr) {
      bestSnr = lnkSnr;
      bestLinkIndex = i;
    }
  }

  currentBestLinkIndex = bestLinkIndex;
}

} // namespace VCTR::ExVectrLink::datalink