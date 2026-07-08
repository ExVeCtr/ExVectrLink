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

void DynamicPower::constrainPowerToLimits() {
  if (!dynamicPowerEnabled) {
    // Disabled: run fixed at the configured dynamic-max.
    setPower(maxDynPowerDBm);
    return;
  }
  // Enabled: snap the current level back inside the dynamic [min, max] range
  // straight away, so changing a limit to sit outside the current power takes
  // effect immediately instead of only on the next algorithm step.
  if (currentPowerDBm < minDynPowerDBm) {
    setPower(minDynPowerDBm);
  } else if (currentPowerDBm > maxDynPowerDBm) {
    setPower(maxDynPowerDBm);
  }
}

void DynamicPower::setDynMaxPower(uint8_t maxPowerDBm) {
  this->maxDynPowerDBm = maxPowerDBm;
  constrainPowerToLimits();
}

void DynamicPower::setDynMinPower(uint8_t minPowerDBm) {
  this->minDynPowerDBm = minPowerDBm;
  constrainPowerToLimits();
}

uint8_t DynamicPower::getMaxPower() const { return maxPowerDBm; }

uint8_t DynamicPower::getMinPower() const { return minPowerDBm; }

uint8_t DynamicPower::getDynMaxPower() const { return maxDynPowerDBm; }

uint8_t DynamicPower::getDynMinPower() const { return minDynPowerDBm; }

void DynamicPower::setEnableDynamicPower(bool enable) {
  dynamicPowerEnabled = enable;
  // When the automatic algorithm is turned off this jumps straight to the max
  // (the configured dynamic-max ceiling) and holds there; when turned on it
  // snaps the current level inside the dynamic [min, max] range.
  constrainPowerToLimits();
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

void DynamicPower::incPower(bool force) {
  for (size_t i = 0; i < kNumPowerLevels; i++) {
    const uint8_t level = kPowerLevels[i];
    if (level <= currentPowerDBm) {
      continue;
    }

    if (level >= minPowerDBm && level <= maxPowerDBm &&
        (force || level <= maxDynPowerDBm)) {
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
      lastDecTime = VCTR::Core::NowNs();
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

  if (!dynamicPowerEnabled) {
    return;
  }

  if (!receivedPacket && missedPacketCount < 10) {
    missedPacketCount++;
  } else {
    missedPacketCount = 0;
  }

  bool missedPacketInc = false;
  if (!receivedPacket && missedPacketCount >= 2) {
    missedPacketInc = true;
  }

  const bool shouldIncrease = (rssi < minRssi) || (snr < minSnr) ||
                              (lq < minLq) || missedPacketInc ||
                              currentPowerDBm < minDynPowerDBm;
  const bool shouldDecrease =
      (rssi > maxRssi) && (snr > maxSnr) && (lq > maxLq) && receivedPacket;

  if (shouldIncrease && currentPowerDBm < maxDynPowerDBm) {
    incPower();
  } else if (shouldDecrease && currentPowerDBm > minDynPowerDBm &&
             (VCTR::Core::NowNs() - lastDecTime) > 1 * VCTR::Core::SECONDS) {
    decPower();
  }
}

} // namespace VCTR::ExVectrLink::datalink