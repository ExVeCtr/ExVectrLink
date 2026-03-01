#include <cstring>

#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/time_definitions.hpp"

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
}

void LinkManager::enableFhss(bool enable) {
  fhssEnabled = enable;
  link.setEnableFhss(enable, fhssSequenceKey);
}

void LinkManager::setMaxTxPower(uint8_t maxDBm) {
  maxTxPowerDBm = clampTxPower(maxDBm);

  if (currentTxPowerDBm > maxTxPowerDBm) {
    currentTxPowerDBm = maxTxPowerDBm;
    link.setTxPower(currentTxPowerDBm);
  }
}

uint8_t LinkManager::getCurrentTxPower() const { return currentTxPowerDBm; }

bool LinkManager::isFailsafe() const { return failsafe; }

uint8_t LinkManager::getLinkQuality() const { return linkQuality; }

uint8_t LinkManager::getLinkRSSI() const { return link.getLinkInfo().rssi; }

uint8_t LinkManager::getLinkSNR() const { return link.getLinkInfo().snr; }

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

void LinkManager::receivePacket(const VCTR::network::DataPacket &packet) {
  lastPacketTime = Core::NOW();
  updateLinkQualityMetrics();
  if (Core::NOW() - lastPowerChangeTime > 100 * Core::MILLISECONDS) {
    lastPowerChangeTime = Core::NOW();

    bool updatePower = false;
    if (linkQuality < 90) {
      currentTxPowerDBm++;
      updatePower = true;
    } else if (linkQuality > 99) {
      currentTxPowerDBm--;
      updatePower = true;
    }
    if (updatePower) {
      currentTxPowerDBm = clampTxPower(currentTxPowerDBm);
      link.setTxPower(currentTxPowerDBm);
    }
  }
}

void LinkManager::updateFailsafeState() {
  constexpr int64_t failsafeTimeout = 500 * Core::MILLISECONDS;
  failsafe = (Core::NOW() - lastPacketTime) > failsafeTimeout;
}

void LinkManager::updateDynamicPowerManagement() {}

void LinkManager::updateBindingState() {}

void LinkManager::updateLinkQualityMetrics() {
  linkQuality = 100 - link.getLinkInfo().lossRate;
}

} // namespace VCTR::ExVectrLink
