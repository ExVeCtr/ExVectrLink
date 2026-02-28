#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/RadioI.hpp"
#include "ExVectrNetwork/datalink/sx1280/Sx1280.hpp"

#include "ExVectrLink/datalink/Sx1280Diversity.hpp"

namespace VCTR::ExVectrLink::datalink {

Sx1280Diversity::Sx1280Diversity(
    std::initializer_list<VCTR::network::datalink::Datalink_SX1280 *> links) {
  for (auto link : links) {
    addDiversityLink(*link);
  }
}

void Sx1280Diversity::addDiversityLink(
    VCTR::network::datalink::Datalink_SX1280 &link) {
  diversityLinks.append({&link, {0, 0}});
  link.addTransmitFinishedHandler([this]() {
    transmitting = false;
    startReceiveOnAllLinks();
  });
  link.addReceiveHandler([this, &link, linkIndex = diversityLinks.size() - 1](
                             const VCTR::network::DataPacket &dataframe) {
    bool othersReceiving = false;
    for (size_t i = 0; i < diversityLinks.size(); i++) {
      if (diversityLinks[i].link->isChannelBlocked()) {
        othersReceiving = true;
        break;
      }
    }
    if (!receiving) {
      receiveHandlers_.callHandlers(dataframe);
      currentBestLinkLq =
          calcLinkQuality({link.lastPacketRSSI(), link.lastPacketSNR()});
      currentBestLinkIndex = linkIndex;
    } else if (!othersReceiving) {
      receiving = false;
    } else {
      uint8_t linkLq =
          calcLinkQuality({link.lastPacketRSSI(), link.lastPacketSNR()});
      if (linkLq > currentBestLinkLq) {
        currentBestLinkLq = linkLq;
        currentBestLinkIndex = linkIndex;
      }
    }
  });
}

const VCTR::network::datalink::Datalink_SX1280 *
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
  stopReceiveOnAllLinks();
  bestLink.link->enableTxRx(true);
  transmitting = true;
  return bestLink.link->transmitDataframe(dataframe);
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
  bool blocked = transmitting;
  for (size_t i = 0; i < diversityLinks.size() && !blocked; i++) {
    blocked = blocked || diversityLinks[i].link->isChannelBlocked();
  }
  return blocked;
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

void Sx1280Diversity::startReceiveOnAllLinks() {
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    diversityLinks[i].link->enableTxRx(true);
  }
}
void Sx1280Diversity::stopReceiveOnAllLinks() {
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    diversityLinks[i].link->enableTxRx(false);
  }
}

uint8_t
Sx1280Diversity::calcLinkQuality(const Sx1280PacketRfInfo &linkInfo) const {
  return linkInfo.snr;
}

} // namespace VCTR::ExVectrLink::datalink