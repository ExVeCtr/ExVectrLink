#include <stddef.h>

#include "ExVectrCore/time_definitions.hpp"

#include "ExVectrLink/datalink/DynamicPower.hpp"

namespace VCTR::ExVectrLink::datalink {

void DynamicPower::setMaxPower(uint8_t maxPowerDBm) {
  uint8_t snappedMax = kPowerLevels[0];
  for (size_t i = 0; i < kNumPowerLevels; i++) {
    if (kPowerLevels[i] <= maxPowerDBm) {
      snappedMax = kPowerLevels[i];
      continue;
    }
    break;
  }

  this->maxPowerDBm = snappedMax;
  if (this->maxPowerDBm < minPowerDBm) {
    minPowerDBm = this->maxPowerDBm;
  }

  setPower(currentPowerDBm);
}

void DynamicPower::setMinPower(uint8_t minPowerDBm) {
  uint8_t snappedMin = kPowerLevels[kNumPowerLevels - 1];
  for (size_t i = 0; i < kNumPowerLevels; i++) {
    if (kPowerLevels[i] >= minPowerDBm) {
      snappedMin = kPowerLevels[i];
      break;
    }
  }

  this->minPowerDBm = snappedMin;
  if (this->minPowerDBm > maxPowerDBm) {
    maxPowerDBm = this->minPowerDBm;
  }

  setPower(currentPowerDBm);
}

void DynamicPower::setEnableDynamicPower(bool enable) {
  dynamicPowerEnabled = enable;
}

bool DynamicPower::isDynamicPowerEnabled() const { return dynamicPowerEnabled; }

int8_t DynamicPower::getPower() const { return currentPowerDBm; }

void DynamicPower::setIncParameters(int8_t minRssi, int8_t minSnr,
                                    uint8_t minLq) {
  this->minRssi = minRssi;
  this->minSnr = minSnr;
  this->minLq = minLq;
}

void DynamicPower::setDecParameters(int8_t maxRssi, int8_t maxSnr,
                                    uint8_t maxLq) {
  this->maxRssi = maxRssi;
  this->maxSnr = maxSnr;
  this->maxLq = maxLq;
}

void DynamicPower::incPower() {
  for (size_t i = 0; i < kNumPowerLevels; i++) {
    const uint8_t level = kPowerLevels[i];
    if (level <= currentPowerDBm) {
      continue;
    }

    if (level >= minPowerDBm && level <= maxPowerDBm) {
      setPower(level);
    }
    return;
  }
}

void DynamicPower::decPower() {
  for (size_t i = kNumPowerLevels; i > 0; i--) {
    const uint8_t level = kPowerLevels[i - 1];
    if (level >= currentPowerDBm) {
      continue;
    }

    if (level >= minPowerDBm && level <= maxPowerDBm) {
      lastDecTime = VCTR::Core::NOW();
      setPower(level);
    }
    return;
  }
}

void DynamicPower::setPower(uint8_t powerDBm) {
  uint8_t boundedMin = minPowerDBm;
  uint8_t boundedMax = maxPowerDBm;
  if (boundedMin > boundedMax) {
    const uint8_t tmp = boundedMin;
    boundedMin = boundedMax;
    boundedMax = tmp;
  }

  uint8_t selectedPower = boundedMin;
  for (size_t i = 0; i < kNumPowerLevels; i++) {
    const uint8_t level = kPowerLevels[i];
    if (level < boundedMin || level > boundedMax) {
      continue;
    }

    if (level <= powerDBm) {
      selectedPower = level;
      continue;
    }
    break;
  }

  currentPowerDBm = selectedPower;
}

void DynamicPower::update(bool receivedPacket, int8_t rssi, int8_t snr,
                          uint8_t lq) {
  // Filter real link-quality values from telemetry (0-100).
  const uint8_t instantLq = receivedPacket ? (lq > 100 ? 100 : lq) : 0;
  static constexpr uint8_t lqWindow = 8;
  currentLq = static_cast<uint8_t>(
      ((uint16_t)currentLq * (lqWindow - 1) + instantLq) / lqWindow);

  if (!dynamicPowerEnabled) {
    return;
  }

  const bool shouldIncrease =
      (rssi < minRssi) || (snr < minSnr) || (currentLq < minLq);
  const bool shouldDecrease =
      (rssi > maxRssi) && (snr > maxSnr) && (currentLq > maxLq);

  if (shouldIncrease) {
    incPower();
  } else if (shouldDecrease &&
             (VCTR::Core::NOW() - lastDecTime) > 1 * VCTR::Core::SECONDS) {
    decPower();
  }
}

} // namespace VCTR::ExVectrLink::datalink