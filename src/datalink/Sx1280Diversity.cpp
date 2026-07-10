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
  activeTxLinkIndex = kNoLink;
  rxPacketLatched = false;
  txInProgress = false;
  snrHistory.clear();

  return configured;
}

void Sx1280Diversity::startRx(int64_t timeout) {
  // If the previous slot was our own TX, the passive radio(s) sat in RX on
  // the same channel and likely received our own transmission at point-blank
  // range. The snapshot in setupTxPacket() cannot cover this: it is taken
  // BEFORE the TX, while the echo increments the passive radio's RX count
  // only when its pull() runs AFTER the TX. Drain those echoes here --
  // process the pending IRQ and resync the seen-count -- without letting
  // their near-field RSSI/SNR into lastPacketRssi/lastPacketSnr, where they
  // would poison refreshBestLink()'s TX-antenna choice and the buffered
  // RSSI/SNR reported below.
  if (txInProgress) {
    for (size_t i = 0; i < diversityLinkCount; i++) {
      if (i == activeTxLinkIndex) {
        continue;
      }
      auto &linkInfo = diversityLinks[i];
      linkInfo.link->pull();
      linkInfo.lastSeenRxPacketCount = linkInfo.link->getRxPacketCount();
    }
  }

  rxPacketLatched = false;
  txInProgress = false;
  activeTxLinkIndex = kNoLink;

  // Keep a buffer value so RSSI and SNR aren't invalid when waiting for the
  // packet. Set the reported values to the best radio's last packet values.
  if (diversityLinkCount > 0) {
    size_t bestIdx = 0;
    int16_t bestSnr = std::numeric_limits<int16_t>::min();
    for (size_t i = 0; i < diversityLinkCount; ++i) {
      if (diversityLinks[i].lastPacketSnr > bestSnr) {
        bestSnr = diversityLinks[i].lastPacketSnr;
        bestIdx = i;
      }
    }
    lastDeliveredPacketRssi = diversityLinks[bestIdx].lastPacketRssi;
    if (snrHistory.size() > 0) {
      lastDeliveredPacketSnr = snrHistory.getMedian();
    } else {
      lastDeliveredPacketSnr = diversityLinks[bestIdx].lastPacketSnr;
    }
  }

  // Reset internal cycle buffer
  currentCycleHasPacket = false;
  currentCycleRssi = 0;
  currentCycleSnr = std::numeric_limits<int16_t>::min();

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
  if (!diversityLinks[pendingTxLinkIndex].link->setupTxPacket(packet)) {
    return false;
  }

  // Do ALL non-active-radio bookkeeping for this TX slot here, well before
  // the caller's timing-critical busy-wait -> startTx() sequence, so
  // startTx() itself has nothing left to do but trigger the active radio.
  // This used to run inside startTx() (called right as the busy-wait
  // releases at the precise slot boundary): an extra loop iteration, a
  // redundant push(true) (push(true) already gets called on every link,
  // including this one, by the caller's push() moments later -- see
  // FHSS::transmitDataPacket()), and a virtual getRxPacketCount() call, all
  // sitting between "busy-wait exits" and "the actual RADIO_SET_TX SPI
  // command reaches the active radio". None of that belongs in the
  // timing-critical path; the non-diversity (single-link) path never paid
  // this cost since the loop body never had a second link to iterate over.
  activeTxLinkIndex = pendingTxLinkIndex;
  txInProgress = true;
  for (size_t i = 0; i < diversityLinkCount; i++) {
    if (i == activeTxLinkIndex) {
      continue;
    }
    // Snapshot each passive radio's RX count so anything already pending is
    // not misattributed later. Packets a passive radio receives DURING the
    // TX slot (self-interference echoes) only increment its RX count on the
    // pull() after TX completes, so they are drained separately in
    // startRx()'s txInProgress branch.
    diversityLinks[i].lastSeenRxPacketCount =
        diversityLinks[i].link->getRxPacketCount();
  }

  return true;
}

void Sx1280Diversity::startTx() {
  if (activeTxLinkIndex >= diversityLinkCount) {
    return;
  }
  // Nothing but the actual trigger belongs here -- see setupTxPacket().
  diversityLinks[activeTxLinkIndex].link->startTx();
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

void Sx1280Diversity::setAutoFS(bool enable) {
  for (size_t i = 0; i < diversityLinkCount; i++) {
    diversityLinks[i].link->setAutoFS(enable);
  }
}

void Sx1280Diversity::setIdle() {
  for (size_t i = 0; i < diversityLinkCount; i++) {
    diversityLinks[i].link->setIdle();
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

  if (txInProgress) {
    if (activeTxLinkIndex < diversityLinkCount) {
      diversityLinks[activeTxLinkIndex].link->pull();
    }
    return;
  }

  size_t winningLinkIndex = kNoLink;
  // Only the timestamp is needed for the tie-break comparison below --
  // getRxPacket() only carries valid RSSI/SNR/timestamp at this point, not
  // the payload (see Sx1280_DirectI::fetchRxPayload()'s doc comment): the
  // actual FIFO read is deferred until fetchRxPayload() is called on
  // whichever radio wins, below, so a losing radio's FIFO is never read.
  int64_t winningTimestamp = 0;
  int16_t winningSnr = std::numeric_limits<int16_t>::min();

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

    if (!currentCycleHasPacket) {
      currentCycleHasPacket = true;
      currentCycleRssi = linkInfo.lastPacketRssi;
      currentCycleSnr = linkInfo.lastPacketSnr;
      lastDeliveredPacketRssi = currentCycleRssi;
      lastDeliveredPacketSnr = currentCycleSnr;
    } else {
      if (linkInfo.lastPacketSnr > currentCycleSnr) {
        currentCycleRssi = linkInfo.lastPacketRssi;
        currentCycleSnr = linkInfo.lastPacketSnr;
        lastDeliveredPacketRssi = currentCycleRssi;
        lastDeliveredPacketSnr = currentCycleSnr;
      }
    }

    const int64_t candidateTimestamp = linkInfo.link->getRxPacket().timestamp;
    if (winningLinkIndex == kNoLink || linkInfo.lastPacketSnr > winningSnr ||
        (linkInfo.lastPacketSnr == winningSnr &&
         candidateTimestamp < winningTimestamp)) {
      winningLinkIndex = i;
      winningTimestamp = candidateTimestamp;
      winningSnr = linkInfo.lastPacketSnr;
    }
  }

  refreshBestLink();

  if (rxPacketLatched || winningLinkIndex == kNoLink) {
    return;
  }

  // Defer the actual FIFO payload read to fetchRxPayload() -- the losing
  // radio(s)' FIFO is simply never read.
  pendingPayloadWinnerIndex = winningLinkIndex;
  lastDeliveredPacketRssi = diversityLinks[winningLinkIndex].lastPacketRssi;
  snrHistory.placeBack(diversityLinks[winningLinkIndex].lastPacketSnr, true);
  lastDeliveredPacketSnr = snrHistory.getMedian();
  rxPacketLatched = true;
  rxPacketCount++;
}

bool Sx1280Diversity::isReceivingPacket() const {
  if (txInProgress) {
    return false;
  }
  for (size_t i = 0; i < diversityLinkCount; i++) {
    if (diversityLinks[i].link->isReceivingPacket()) {
      return true;
    }
  }
  return false;
}

void Sx1280Diversity::fetchRxPayload() {
  if (pendingPayloadWinnerIndex == kNoLink ||
      pendingPayloadWinnerIndex >= diversityLinkCount) {
    return;
  }
  auto *winner = diversityLinks[pendingPayloadWinnerIndex].link;
  winner->fetchRxPayload();
  lastRxPacket = winner->getRxPacket();
  pendingPayloadWinnerIndex = kNoLink;
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