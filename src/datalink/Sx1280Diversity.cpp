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
  for (auto link : links) {
    addDiversityLink(*link);
  }
}

void Sx1280Diversity::addDiversityLink(
    VCTR::network::datalink::Datalink_SX1280_V2 &link) {
  auto linkIndex = diversityLinks.size();
  diversityLinks.append({&link, {0, 0}});
  link.addTransmitFinishedHandler([this]() {
    transmitting = false;
    startReceiveOnAllLinks();
  });
  link.addReceiveHandler(
      [this, linkIndex](const VCTR::network::DataPacket &dataframe) {
        auto time = Core::NOW();

        auto data = dataframe;
        auto dataId = data.payload[data.payload.size() - 1];
        data.payload.pop();

        auto &linkInfo = diversityLinks[linkIndex];
        linkInfo.lastPacketInfo.rssi = linkInfo.link->lastPacketRSSI();
        linkInfo.lastPacketInfo.snr = linkInfo.link->lastPacketSNR();
        linkInfo.lastPacketInfo.receivedIds.placeBack(dataId, true);

        // LOG_MSG("Received packet %d (last %d) on link %d with RSSI %d and SNR
        // "
        //         "%d start at %.3fms, size: %d\n",
        //         dataId, lastReceivedPacketId, linkIndex,
        //         linkInfo.lastPacketInfo.rssi, linkInfo.lastPacketInfo.snr,
        //         (double)dataframe.timestamp / Core::MILLISECONDS,
        //         data.payload.size());

        // char packetContent[100];
        // size_t contentSize = std::min((size_t)100, data.payload.size());
        // for (size_t i = 0; i < contentSize; i++) {
        //   packetContent[i] = static_cast<char>(data.payload[i]) + '0';
        // }
        // packetContent[contentSize] = '\0';
        // LOG_MSG("Packet content: %s\n", packetContent);

        uint8_t idDiff = uint8_t(dataId - lastReceivedPacketId);
        if (idDiff > 0 && idDiff < 128) {
          receiveHandlers_.callHandlers(data);
        }

        lastReceivedPacketId = dataId;
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
  if (diversityLinks.size() == 0 || transmitting) {
    return false;
  }

  auto data = dataframe;
  lastReceivedPacketId++;
  data.payload.append(lastReceivedPacketId);

  auto &bestLink = diversityLinks[currentBestLinkIndex];
  stopReceiveOnAllLinks();
  bestLink.link->setEnableTxRx(true);
  transmitting = true;
  return bestLink.link->transmitDataframe(data);
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
  if (transmitting) {
    return true;
  }
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

void Sx1280Diversity::startReceiveOnAllLinks() {
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    diversityLinks[i].link->setEnableTxRx(true);
  }
}
void Sx1280Diversity::stopReceiveOnAllLinks() {
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    diversityLinks[i].link->setEnableTxRx(false);
  }
}

void Sx1280Diversity::determineBestLink() {
  size_t bestLinkIndex = currentBestLinkIndex;
  size_t bestLostPackets = (size_t)-1;

  for (size_t i = 0; i < diversityLinks.size(); i++) {
    const auto &linkInfo = diversityLinks[i];
    const auto &ids = linkInfo.lastPacketInfo.receivedIds;

    if (ids.size() == 0) {
      // No packets received on this link yet; skip
      continue;
    }

    if (ids.size() == 1) {
      // Only one sample; cannot estimate loss, assume none
      if (bestLostPackets > 0) {
        bestLostPackets = 0;
        bestLinkIndex = i;
      }
      continue;
    }

    uint8_t firstId = ids[0];
    uint8_t lastId = ids[ids.size() - 1];
    size_t expectedPackets =
        static_cast<size_t>(static_cast<uint8_t>(lastId - firstId)) + 1u;
    size_t receivedPackets = ids.size();
    size_t lostPackets = (expectedPackets > receivedPackets)
                             ? (expectedPackets - receivedPackets)
                             : 0u;

    if (lostPackets < bestLostPackets) {
      bestLostPackets = lostPackets;
      bestLinkIndex = i;
    }
  }

  currentBestLinkIndex = bestLinkIndex;
}

} // namespace VCTR::ExVectrLink::datalink