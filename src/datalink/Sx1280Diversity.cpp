#include "ExVectrLink/datalink/Sx1280Diversity.hpp"

namespace VCTR::ExVectrLink::datalink {

Sx1280Diversity::Sx1280Diversity(
    std::initializer_list<VCTR::network::datalink::Sx1280_DirectI *> links) {
  for (auto link : links) {
    if (link == nullptr) {
      continue;
    }
    addDiversityLink(*link);
  }
}

void Sx1280Diversity::addDiversityLink(
    VCTR::network::datalink::Sx1280_DirectI &link) {
  if (diversityLinkCount >= kMaxDiversityLinks) {
    return;
  }

  auto &linkInfo = diversityLinks[diversityLinkCount++];
  linkInfo.link = &link;
  linkInfo.lastSeenRxPacketCount = link.getRxPacketCount();
  linkInfo.lastPacketRssi = link.getPacketRSSI();
  linkInfo.lastPacketSnr = link.getPacketSNR();

  refreshBestLink();
}

const VCTR::network::datalink::Sx1280_DirectI *
Sx1280Diversity::getDiversityLink(size_t index) const {
  if (index >= diversityLinkCount) {
    return nullptr;
  }
  return diversityLinks[index].link;
}

size_t Sx1280Diversity::getCurrentBestLinkIndex() const {
  return currentBestLinkIndex;
}

size_t Sx1280Diversity::getCurrentTxLinkIndex() const {
  return getTxLinkIndex();
}

bool Sx1280Diversity::setDesignatedTxLink(
    const VCTR::network::datalink::Sx1280_DirectI &link) {

  for (size_t i = 0; i < diversityLinkCount; i++) {
    if (diversityLinks[i].link == &link) {
      designatedTxLink = i;
      return true;
    }
  }
  return false;
}

size_t Sx1280Diversity::getTxLinkIndex() const {
  if (diversityLinkCount == 0) {
    return 0;
  }

  if (designatedTxLink < diversityLinkCount) {
    return designatedTxLink;
  }

  return (currentBestLinkIndex < diversityLinkCount) ? currentBestLinkIndex : 0;
}

bool Sx1280Diversity::configureRadio() {
  if (diversityLinkCount == 0) {
    return false;
  }

  bool configured = true;
  for (size_t i = 0; i < diversityLinkCount; i++) {
    auto &linkInfo = diversityLinks[i];
    configured = linkInfo.link->configureRadio() && configured;
    linkInfo.lastSeenRxPacketCount = linkInfo.link->getRxPacketCount();
    linkInfo.lastPacketRssi = linkInfo.link->getPacketRSSI();
    linkInfo.lastPacketSnr = linkInfo.link->getPacketSNR();
  }

  refreshBestLink();
  pendingTxLinkIndex = getTxLinkIndex();
  rxPacketLatched = false;

  return configured;
}

void Sx1280Diversity::startRx(int64_t timeout) {
  rxPacketLatched = false;

  for (size_t i = 0; i < diversityLinkCount; i++) {
    auto &linkInfo = diversityLinks[i];
    linkInfo.lastSeenRxPacketCount = linkInfo.link->getRxPacketCount();
    linkInfo.link->startRx(timeout);
  }
}

int16_t Sx1280Diversity::getPacketRSSI() const {
  return lastDeliveredPacketRssi;
}

int16_t Sx1280Diversity::getPacketSNR() const { return lastDeliveredPacketSnr; }

VCTR::network::DataPacket Sx1280Diversity::getRxPacket() const {
  return lastRxPacket;
}

uint32_t Sx1280Diversity::getRxPacketCount() const { return rxPacketCount; }

bool Sx1280Diversity::setupTxPacket(const VCTR::network::DataPacket &packet) {
  if (diversityLinkCount == 0) {
    return false;
  }

  pendingTxLinkIndex = getTxLinkIndex();
  return diversityLinks[pendingTxLinkIndex].link->setupTxPacket(packet);
}

void Sx1280Diversity::startTx() {
  if (diversityLinkCount == 0) {
    return;
  }

  diversityLinks[pendingTxLinkIndex].link->startTx();
}

size_t Sx1280Diversity::getNumChannels() const {
  if (diversityLinkCount == 0) {
    return 0;
  }
  return diversityLinks[0].link->getNumChannels();
}

size_t Sx1280Diversity::getCurrentChannel() const {
  if (diversityLinkCount == 0) {
    return 0;
  }
  return diversityLinks[0].link->getCurrentChannel();
}

void Sx1280Diversity::setChannel(size_t channel) {
  for (size_t i = 0; i < diversityLinkCount; i++) {
    diversityLinks[i].link->setChannel(channel);
  }
}

void Sx1280Diversity::setFrequency(uint32_t newFreqHz) {
  for (size_t i = 0; i < diversityLinkCount; i++) {
    diversityLinks[i].link->setFrequency(newFreqHz);
  }
}

void Sx1280Diversity::setSpreadingFactor(
    VCTR::network::datalink::SX1280_SF sf) {
  for (size_t i = 0; i < diversityLinkCount; i++) {
    diversityLinks[i].link->setSpreadingFactor(sf);
  }
}

void Sx1280Diversity::setBandwidth(VCTR::network::datalink::SX1280_BW bw) {
  for (size_t i = 0; i < diversityLinkCount; i++) {
    diversityLinks[i].link->setBandwidth(bw);
  }
}

void Sx1280Diversity::setCodingRate(VCTR::network::datalink::SX1280_CR cr) {
  for (size_t i = 0; i < diversityLinkCount; i++) {
    diversityLinks[i].link->setCodingRate(cr);
  }
}

void Sx1280Diversity::setTxPower(int8_t power) {
  for (size_t i = 0; i < diversityLinkCount; i++) {
    diversityLinks[i].link->setTxPower(power);
  }
}

uint8_t Sx1280Diversity::getTxPower() const {
  if (diversityLinkCount == 0) {
    return 0;
  }

  return diversityLinks[getTxLinkIndex()].link->getTxPower();
}

void Sx1280Diversity::setTxMaxPower(int8_t maxTxPower) {
  for (size_t i = 0; i < diversityLinkCount; i++) {
    diversityLinks[i].link->setTxMaxPower(maxTxPower);
  }
}

void Sx1280Diversity::setPacketMode(
    VCTR::network::datalink::SX1280_PacketMode mode) {
  for (size_t i = 0; i < diversityLinkCount; i++) {
    diversityLinks[i].link->setPacketMode(mode);
  }
}

void Sx1280Diversity::setFixedPacketLength(uint8_t length) {
  for (size_t i = 0; i < diversityLinkCount; i++) {
    diversityLinks[i].link->setFixedPacketLength(length);
  }
}

void Sx1280Diversity::setPAdbm(uint8_t paDbm) {
  for (size_t i = 0; i < diversityLinkCount; i++) {
    diversityLinks[i].link->setPAdbm(paDbm);
  }
}

void Sx1280Diversity::push(bool keepOscRunning) {
  for (size_t i = 0; i < diversityLinkCount; i++) {
    diversityLinks[i].link->push(keepOscRunning);
  }
}

void Sx1280Diversity::pull() {
  if (diversityLinkCount == 0) {
    return;
  }

  size_t candidateIndexes[kMaxDiversityLinks] = {};
  VCTR::network::DataPacket candidatePackets[kMaxDiversityLinks];
  size_t candidateCount = 0;

  for (size_t i = 0; i < diversityLinkCount; i++) {
    auto &linkInfo = diversityLinks[i];
    linkInfo.link->pull();

    const uint32_t newRxCount = linkInfo.link->getRxPacketCount();
    if (newRxCount == linkInfo.lastSeenRxPacketCount) {
      continue;
    }

    linkInfo.lastSeenRxPacketCount = newRxCount;
    linkInfo.lastPacketRssi = linkInfo.link->getPacketRSSI();
    linkInfo.lastPacketSnr = linkInfo.link->getPacketSNR();

    if (candidateCount < kMaxDiversityLinks) {
      candidateIndexes[candidateCount] = i;
      candidatePackets[candidateCount] = linkInfo.link->getRxPacket();
      candidateCount++;
    }
  }

  refreshBestLink();

  if (rxPacketLatched || candidateCount == 0) {
    return;
  }

  size_t selectedCandidate = 0;
  for (size_t i = 1; i < candidateCount; i++) {
    if (candidatePackets[i].timestamp <
        candidatePackets[selectedCandidate].timestamp) {
      selectedCandidate = i;
    }
  }

  const size_t selectedLinkIndex = candidateIndexes[selectedCandidate];
  lastRxPacket = candidatePackets[selectedCandidate];
  lastDeliveredPacketRssi = diversityLinks[selectedLinkIndex].lastPacketRssi;
  lastDeliveredPacketSnr = diversityLinks[selectedLinkIndex].lastPacketSnr;
  rxPacketLatched = true;
  rxPacketCount++;
}

void Sx1280Diversity::refreshBestLink() {
  if (diversityLinkCount == 0) {
    currentBestLinkIndex = 0;
    return;
  }

  size_t bestLinkIndex =
      (currentBestLinkIndex < diversityLinkCount) ? currentBestLinkIndex : 0;
  int16_t bestSnr = diversityLinks[bestLinkIndex].lastPacketSnr;

  for (size_t i = 0; i < diversityLinkCount; i++) {
    if (diversityLinks[i].lastPacketSnr > bestSnr) {
      bestSnr = diversityLinks[i].lastPacketSnr;
      bestLinkIndex = i;
    }
  }

  currentBestLinkIndex = bestLinkIndex;
}

} // namespace VCTR::ExVectrLink::datalink