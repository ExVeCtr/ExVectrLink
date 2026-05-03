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
    std::initializer_list<VCTR::network::datalink::Datalink_SX1280_V2 *> links)
    : VCTR::Core::Scheduler::Task("Sx1280Diversity") {
  Core::getSystemScheduler().addTask(*this);
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
  diversityLinks.append({&link, {0, 0, 0}});
  // link.addTransmitFinishedHandler([this, linkIndex]() {
  //   // transmitting = 0;
  //   // startReceiveOnAllLinks();
  // });
  link.addReceiveHandler(
      [this, linkIndex](const VCTR::network::DataPacket &dataframe) {
        // if (transmitting != 0) {
        //   int64_t txDelta = dataframe.timestamp - transmitting;

        //   if (txDelta <= 5 * Core::MILLISECONDS &&
        //       txDelta >= -1 * Core::MILLISECONDS) {
        //     return;
        //   }
        // }

        // receiveHandlers_.callHandlers(dataframe);
        // return;

        auto &linkInfo = diversityLinks[linkIndex];
        linkInfo.lastPacketInfo.rssi = linkInfo.lastPacketInfo.rssi * 0.5 +
                                       linkInfo.link->lastPacketRSSI() * 0.5;
        linkInfo.lastPacketInfo.snr = linkInfo.lastPacketInfo.snr * 0.5 +
                                      linkInfo.link->lastPacketSNR() * 0.5;
        linkInfo.lastPacketInfo.receivedTime = dataframe.timestamp;

        determineBestLink();

        receiveHandlers_.callHandlers(dataframe);

        /**
         * Ok so we've received a packet. We now check if all links have
         * received a packet. If not, then we trigger our task to run in a bit
         * to give other links time to process their rx. After that time we
         * simply choose the best packet and forward it. If all links have
         * received somthing, then we immediatly forward the best packet and
         * reset.
         */

        // if (dataframe.timestamp - transmitting > 3 * Core::MILLISECONDS) {
        //   receiving = true;

        //   auto &linkInfo = diversityLinks[linkIndex];
        //   linkInfo.lastPacketInfo.rssi = linkInfo.link->lastPacketRSSI();
        //   linkInfo.lastPacketInfo.snr = linkInfo.link->lastPacketSNR();
        //   linkInfo.lastPacketInfo.receivedTime = dataframe.timestamp;
        //   // linkInfo.lastPacketInfo.packet = dataframe;

        //   determineBestLink();

        //   receiveHandlers_.callHandlers(dataframe);
        //   // processReceivedPackets();
        // }

        // auto linkMissingRecv = false;
        // for (size_t i = 0; i < diversityLinks.size(); i++) {
        //   auto &linkInfo = diversityLinks[i].lastPacketInfo;
        //   if (linkInfo.receivedTime == 0) {
        //     linkMissingRecv = true;
        //     break;
        //   }
        // }
        // if (linkMissingRecv) {
        //   auto taskRun = Core::NOW() + 1 * Core::MILLISECONDS;
        //   setDeadline(taskRun);
        //   setRelease(taskRun);
        // } else {
        //   receiving = false;
        //   // processReceivedPackets();
        // }
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

  transmitting = dataframe.timestamp == 0 ? Core::NOW() : dataframe.timestamp;

  auto txLinkIndex = getTxLinkIndex();
  auto &txLink = diversityLinks[txLinkIndex];
  //  stopReceiveOnAllLinks(txLinkIndex);
  return txLink.link->transmitDataframe(dataframe);
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

int16_t Sx1280Diversity::lastPacketRSSI() const {
  if (diversityLinks.size() == 0) {
    return 0;
  }
  return diversityLinks[currentBestLinkIndex].lastPacketInfo.rssi;
}

void Sx1280Diversity::startReceiveOnAllLinks() {
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    diversityLinks[i].link->setEnableTxRx(true);
    // diversityLinks[i].link->setStartReceive(true);
  }
}
void Sx1280Diversity::stopReceiveOnAllLinks(size_t exceptIndex) {
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    if (i == exceptIndex) {
      continue;
    }
    // diversityLinks[i].link->setStartReceive(false);
    diversityLinks[i].link->setEnableTxRx(false);
  }
}

void Sx1280Diversity::setStartReceive(bool rxEnabled) {
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    diversityLinks[i].link->setStartReceive(rxEnabled);
    // diversityLinks[i].link->setEnableTxRx(rxEnabled);
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

void Sx1280Diversity::taskInit() {}

void Sx1280Diversity::taskThread() {
  if (receiving) {
    receiving = false;
    processReceivedPackets();
  }
  setRelease(Core::END_OF_TIME);
}

void Sx1280Diversity::determineBestLink() {
  // Require a new link to be at least kHysteresisDb better than the current
  // best to prevent rapid flapping when both radios have similar signal.

  if (Core::NOW() - lastbestLinkUpdateTime < 5 * Core::MILLISECONDS) {
    return;
  }
  lastbestLinkUpdateTime = Core::NOW();
  static constexpr int16_t kHysteresisDb = 0;

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

void Sx1280Diversity::processReceivedPackets() {

  auto bestLinkIndex = currentBestLinkIndex;
  int64_t earliestReceivedTime = Core::END_OF_TIME;
  for (size_t i = 0; i < diversityLinks.size(); i++) {
    auto &linkInfo = diversityLinks[i].lastPacketInfo;
    if (linkInfo.packet.timestamp < earliestReceivedTime) {
      earliestReceivedTime = linkInfo.packet.timestamp;
    }
    linkInfo.receivedTime = 0; // reset for next round
  }

  auto bestPacket = diversityLinks[bestLinkIndex].lastPacketInfo.packet;
  // bestPacket.timestamp = earliestReceivedTime;

  receiveHandlers_.callHandlers(bestPacket);
}

} // namespace VCTR::ExVectrLink::datalink