#include "ExVectrCore/task_types.hpp"
#include "ExVectrCore/time_definitions.hpp"

#include "ExVectrLink/LinkManager.hpp"

namespace VCTR::ExVectrLink {

namespace {

constexpr uint8_t MinTxPowerDBm = 2;
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
  // Lightweight deterministic key seeding. Replace with stronger randomness
  // or pairing state as required.
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
  lastHealthyLinkTime = Core::NOW();

  Core::getSystemScheduler().addTask(*this);
}

void LinkManager::taskInit() {
  failsafe = false;
  bindingInProgress = false;
  lastHealthyLinkTime = Core::NOW();
}

void LinkManager::taskCheck() {
  // Keep this task light. Run immediately if in a transient link state.
  if (bindingInProgress || failsafe || getLinkQuality() < 80) {
    setDeadline(Core::NOW());
  }
}

void LinkManager::taskThread() {
  const auto &info = link.getLinkInfo();

  if (info.lossRate < 100) {
    lastHealthyLinkTime = Core::NOW();
  }

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

bool LinkManager::isFailsafe() const { return failsafe; }

uint8_t LinkManager::getLinkQuality() const {
  const auto &info = link.getLinkInfo();
  if (info.lossRate >= 100) {
    return 0;
  }
  return static_cast<uint8_t>(100 - info.lossRate);
}

uint8_t LinkManager::getLinkRSSI() const {
  const auto rssi = link.getLinkInfo().rssi;
  return rssi < 0 ? 0 : static_cast<uint8_t>(rssi);
}

uint8_t LinkManager::getLinkSNR() const {
  const auto snr = link.getLinkInfo().snr;
  return snr < 0 ? 0 : static_cast<uint8_t>(snr);
}

uint8_t LinkManager::getLinkAntenna() const {
  return link.getLinkInfo().antenna;
}

void LinkManager::startBinding() {
  bindingInProgress = true;

  // Generate a fresh sequence key and re-apply FHSS state to aid pairing.
  fhssSequenceKey = makeFhssSequenceKey(mak);
  link.setMediaAccessKey(mak);

  if (fhssEnabled) {
    link.setEnableFhss(false, 0);
    link.setEnableFhss(true, fhssSequenceKey);
  }
}

uint32_t LinkManager::getFhssSequenceKey() const { return fhssSequenceKey; }

void LinkManager::setMak(uint8_t mak) {
  this->mak = mak;
  link.setMediaAccessKey(mak);
}

void LinkManager::updateFailsafeState() {
  constexpr int64_t failsafeTimeout = 750 * Core::MILLISECONDS;
  failsafe = (Core::NOW() - lastHealthyLinkTime) > failsafeTimeout;
}

void LinkManager::updateDynamicPowerManagement() {
  // TODO: implement adaptive TX power logic using RSSI/SNR/loss metrics.
  // Current placeholder keeps TX power pinned to the configured cap.
  if (currentTxPowerDBm != maxTxPowerDBm) {
    currentTxPowerDBm = maxTxPowerDBm;
    link.setTxPower(currentTxPowerDBm);
  }
}

void LinkManager::updateBindingState() {
  // TODO: implement full bind state machine (timeouts, retries, confirmation).
  // Placeholder exits binding once link quality is reasonable.
  if (bindingInProgress && getLinkQuality() >= 60) {
    bindingInProgress = false;
  }
}

} // namespace VCTR::ExVectrLink
