#include <cstring>

#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/time_definitions.hpp"

#include "ExVectrLink/datalink/PacketTypes.hpp"

#include "ExVectrLink/LinkManager.hpp"

namespace VCTR::ExVectrLink {

namespace {

constexpr uint8_t MinTxPowerDBm = 12;
constexpr uint8_t MaxTxPowerDBm = 33;

uint8_t clampTxPower(uint8_t txPowerDBm) {
  if (txPowerDBm < MinTxPowerDBm) {
    return MinTxPowerDBm;
  }
  if (txPowerDBm > MaxTxPowerDBm) {
    return MaxTxPowerDBm;
  }
  return txPowerDBm;
}

uint32_t makeFhssSequenceKey(uint8_t mak) {
  uint32_t seed = 0xA5F01234u;
  seed ^= static_cast<uint32_t>(mak) << 24;
  seed ^= static_cast<uint32_t>(Core::NOW());
  seed ^= static_cast<uint32_t>(Core::NOW() >> 16);
  return seed;
}

} // namespace

LinkManager::LinkManager(VCTR::ExVectrLink::ExVectrLinkI &link, uint8_t mak)
    : Core::Task_Periodic("LinkManager", 100 * Core::MILLISECONDS), link(link),
      mak(mak) {
  setMak(mak);
  setMaxTxPower(maxTxPowerDBm);

  fhssSequenceKey = makeFhssSequenceKey(mak);

  Core::getSystemScheduler().addTask(*this);
}

void LinkManager::taskInit() {
  failsafe = true;
  bindingInProgress = false;

  link.addReceiveHandler([this](const VCTR::network::DataPacket &packet) {
    receivePacket(packet);
  });
}

void LinkManager::taskCheck() {}

void LinkManager::taskThread() {
  updateFailsafeState();
  updateDynamicPowerManagement();
  updateBindingState();
  updateLinkQualityMetrics();
  updateDynamicPowerManagement();
}

void LinkManager::enableFhss(bool enable) {
  fhssEnabled = enable;
  link.setEnableFhss(enable, fhssSequenceKey);
}

void LinkManager::setMinTxPower(uint8_t minDBm) { minTxPowerDBm = minDBm; }

void LinkManager::setMaxTxPower(uint8_t maxDBm) { maxTxPowerDBm = maxDBm; }

uint8_t LinkManager::getCurrentTxPower() const {
  return powerLevels[currentTxPowerDBm];
}

bool LinkManager::isFailsafe() const { return failsafe; }

uint8_t LinkManager::getLinkQuality() const { return linkQuality; }

int8_t LinkManager::getLinkRSSI() const { return linkRssi; }

int8_t LinkManager::getLinkSNR() const { return linkSnr; }

uint8_t LinkManager::getLinkAntenna() const {
  return link.getLinkInfo().antenna;
}

void LinkManager::startBinding() {
  bindingInProgress = true;

  link.setEnableFhss(false);
  link.setLinkChannel(0);
}

uint32_t LinkManager::getFhssSequenceKey() const { return fhssSequenceKey; }

void LinkManager::setMak(uint8_t mak) {
  this->mak = mak;
  link.setMediaAccessKey(mak);
}

bool LinkManager::transmitDataframe(
    const VCTR::network::DataPacket &dataframe) {

  VCTR::network::DataPacket dataPacket;
  dataPacket.payload = dataframe.payload;
  dataPacket.payload.append(VCTR::ExVectrLink::datalink::PacketTypes::Data);
  return link.transmitDataframe(dataPacket);
}

size_t LinkManager::getMaxPacketSize() const { return link.getMaxPacketSize(); }

bool LinkManager::isChannelBlocked() const { return link.isChannelBlocked(); }

void LinkManager::receivePacket(const VCTR::network::DataPacket &packet) {
  int64_t receiveTime = Core::NOW();

  auto packetType = packet.payload[packet.payload.size() - 1];
  if (packetType == VCTR::ExVectrLink::datalink::PacketTypes::Data) {
    VCTR::network::DataPacket dataPacket;
    dataPacket.payload.setSize(packet.payload.size() - 1);
    std::memcpy(dataPacket.payload.getPtr(), packet.payload.getPtr(),
                packet.payload.size() - 1);
    receiveHandlers_.callHandlers(dataPacket);
  }
  lastPacketTime = receiveTime;
}

void LinkManager::updateFailsafeState() {
  constexpr int64_t failsafeTimeout = 1000 * Core::MILLISECONDS;
  // failsafe = (Core::NOW() - lastPacketTime) > failsafeTimeout;
  // if (failsafe) {
  //   currentTxPowerDBm = 0;
  //   for (size_t i = 0; i < numPowerLevels - 1 &&
  //                      powerLevels[currentTxPowerDBm] < maxTxPowerDBm;
  //        i++) {
  //     currentTxPowerDBm++;
  //   }
  //   link.setTxPower(powerLevels[currentTxPowerDBm]);
  // }
  failsafe = !link.isConnected();
  if (failsafe) {
    currentTxPowerDBm = 0;
    for (size_t i = 0; i < numPowerLevels - 1 &&
                       powerLevels[currentTxPowerDBm] < maxTxPowerDBm;
         i++) {
      currentTxPowerDBm++;
    }
    link.setTxPower(powerLevels[currentTxPowerDBm]);
  }
}

void LinkManager::updateDynamicPowerManagement() {
  if (Core::NOW() - lastPowerChangeTime > 0) {
    lastPowerChangeTime = Core::NOW() + 10 * Core::MILLISECONDS;

    if (lastLinkQuality - linkQuality > 20 &&
        currentTxPowerDBm < numPowerLevels - 1 &&
        powerLevels[currentTxPowerDBm + 1] <= maxTxPowerDBm) {
      currentTxPowerDBm++;
      lastPowerChangeTime = Core::NOW() + 100 * Core::MILLISECONDS;
    } else if ((linkQuality <= 70 || linkSnr <= 0) &&
               currentTxPowerDBm < numPowerLevels - 1 &&
               powerLevels[currentTxPowerDBm + 1] <= maxTxPowerDBm) {
      currentTxPowerDBm++;
      lastPowerChangeTime = Core::NOW() + 100 * Core::MILLISECONDS;
    } else if (linkQuality >= 95 && currentTxPowerDBm > 0 && linkSnr >= 5 &&
               powerLevels[currentTxPowerDBm - 1] > minTxPowerDBm) {
      currentTxPowerDBm--;
      lastPowerChangeTime = Core::NOW() + 500 * Core::MILLISECONDS;
    }

    if (currentTxPowerDBm >= numPowerLevels) {
      currentTxPowerDBm = numPowerLevels - 1;
    }
    link.setTxPower(powerLevels[currentTxPowerDBm]);

    lastLinkQuality = linkQuality;
  }
}

void LinkManager::updateBindingState() {}

void LinkManager::updateLinkQualityMetrics() {
  const auto &linkInfo = link.getLinkInfo();
  linkQuality = 100 - linkInfo.lossRate;
  linkSnr = linkInfo.snr;
  linkRssi = linkInfo.rssi;
}

} // namespace VCTR::ExVectrLink
