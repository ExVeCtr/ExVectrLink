#include <Arduino.h>

#include "ExVectrCore/CanSerialize.hpp"
#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/RadioI.hpp"

#include "ExVectrLink/datalink/FHSS.hpp"

namespace VCTR::ExVectrLink::datalink {

// =============================================================================
// Construction
// =============================================================================

FHSS::FHSS(VCTR::network::datalink::Sx1280_DirectI &radio)
    : Core::Task_Periodic("FHSS", 100 * Core::MILLISECONDS), radioLink(radio) {
  setPriority(5000);
  Core::getSystemScheduler().addTask(*this);
}

// =============================================================================
// Configuration setters / getters
// =============================================================================

void FHSS::setFhssKey(uint8_t newKey) {
  key = newKey;
  generateChannelSequence(key);
}

uint8_t FHSS::getFhssKey() const { return key; }
void FHSS::setIsRxSide(bool isRxSide) { this->isRxSide = isRxSide; }

void FHSS::setSlotInterval(int64_t interval) { slotInterval = interval; }
int64_t FHSS::getSlotInterval() const { return slotInterval; }

void FHSS::setSlotsPerHop(uint8_t slots) {
  if (slots > 16)
    slots = 16;
  slotsPerHop = slots;
}
uint8_t FHSS::getSlotsPerHop() const { return slotsPerHop; }

void FHSS::setRxSlotIndex(uint8_t index) {
  if (index > 4) {
    index = 4; // Cap at 4 to fit in 2 bits of trailer.
  } else if (index == 0) {
    index = 1; // Cannot be 0
  }
  numTxPacketsToRx = index;
}
uint8_t FHSS::getRxSlotIndex() const { return numTxPacketsToRx; }

void FHSS::resetDesyncCounter() { desyncCounter = 0; }
uint32_t FHSS::getDesyncCounter() const { return desyncCounter; }

// =============================================================================
// Status getters
// =============================================================================

FHSSState FHSS::getFhssState() const { return fhssState; }

float FHSS::getLinkQuality() const {
  return (float)(isRxSide ? linkQuality : otherEndLinkQuality);
}

int64_t FHSS::getTimingOffset() const { return slotTimingOffset; }

int64_t FHSS::getTrueSlotInterval() const { return trueSlotInterval; }

uint8_t FHSS::getSlotCounter() const { return slotCounter; }

void FHSS::addSlotHandler(HandlerFunction handler) {
  slotHandlers.addHandler(handler);
}

// =============================================================================
// DatalinkI interface
// =============================================================================

bool FHSS::transmitDataframe(const VCTR::network::DataPacket &dataframe) {

  if (dataframe.payload.size() > getMaxPacketSize()) {
    // Packet too large, drop it.
    return false;
  }

  // uint8_t trailerByte1 =
  //     (0b10000000) | (uint8_t)(uint8_t)(linkQuality * 127.0f) & 0x7F;
  // uint8_t trailerByte2 = ((slotCounter & 0x0F) << 4) | (key & 0x0F);

  packetToSend = dataframe;
  return true;
}

size_t FHSS::getMaxPacketSize() const { return 9; }

bool FHSS::isChannelBlocked() const { return packetToSend.payload.size() > 0; }

// =============================================================================
// Sequence generation
// =============================================================================

void FHSS::generateChannelSequence(uint8_t key) {
  channelSequence.clear();

  size_t numChannels = radioLink.getNumChannels();
  if (numChannels == 0)
    return;

  // Simple Fisher-Yates-style deterministic shuffle seeded by key.
  for (uint8_t i = 0; i < numChannels; i++) {
    channelSequence.append(i);
  }

  // Deterministic shuffle using the key as seed.
  uint32_t seed = key * 2654435761u; // Knuth multiplicative hash
  for (size_t i = numChannels - 1; i > 0; i--) {
    seed ^= (seed << 13);
    seed ^= (seed >> 17);
    seed ^= (seed << 5);
    size_t j = seed % (i + 1);
    // Swap
    uint8_t tmp = channelSequence[i];
    channelSequence[i] = channelSequence[j];
    channelSequence[j] = tmp;
  }

  currentChannelIdx = 0;
}

// =============================================================================
// Slot/hop logic helpers
// =============================================================================

void FHSS::syncTimer(int64_t receiveStartTime) {

  // Phase error relative to the current slot grid.
  // The packet timestamp (receiveStartTime) marks the start of reception
  // which approximates the TX-side slot boundary.  Since the receivePacket
  // callback fires later (after the full packet is received), currentSlotStart
  // may already have been advanced past the slot the packet belongs to.
  // The modulo wrapping makes the phase error independent of how many
  // slots have elapsed since then.

  receiveStartTime -= 300 * Core::MICROSECONDS;

  int64_t referenceSlotStart = currentSlotStart;
  int64_t estimatedSlotStart = receiveStartTime - slotOffsetTime;

  int64_t slotStartError = estimatedSlotStart - referenceSlotStart;

  // Wrap to [-slotInterval/2, +slotInterval/2).  The modulo handles the
  // Searching case where the grids may be completely unaligned.
  if (slotInterval > 0) {
    slotStartError = slotStartError % slotInterval;
    if (slotStartError > slotInterval / 2)
      slotStartError -= slotInterval;
    if (slotStartError < -slotInterval / 2)
      slotStartError += slotInterval;
  }

  if (isRxSide) {
    if (fhssState == FHSSState::Searching) {
      // First packet received while searching.  Compute the phase error
      // against the raw (unshifted) grid so that a stale slotOffsetTime
      // left over from a previous sync session doesn't corrupt the
      // initial estimate.
      int64_t rawError = receiveStartTime - currentSlotStart;
      if (slotInterval > 0) {
        rawError = rawError % slotInterval;
        if (rawError > slotInterval / 2)
          rawError -= slotInterval;
        if (rawError < -slotInterval / 2)
          rawError += slotInterval;
      }
      slotTimingOffset = rawError;
      slotOffsetTime = rawError;
      intervalCorrection = 0;
      fhssState = FHSSState::Syncing;

    } else if (fhssState == FHSSState::Syncing) {
      slotTimingOffset = slotStartError;
      slotOffsetTime += slotStartError * 0.1;

      auto lqThres = 1.0f - 1.0f / (float)slotsPerHop;
      if (slotTimingOffset > slotInterval / 10) {
        syncedStartTime = Core::NowNs();
      } else if (Core::NowNs() - syncedStartTime > 500 * Core::MILLISECONDS &&
                 linkQuality > lqThres && linkQuality > 0.5f) {
        fhssState = FHSSState::Synced;
      }

    } else {
      slotTimingOffset = slotTimingOffset * 0.95 + slotStartError * 0.05;
      slotOffsetTime += slotStartError * 0.05;

      // Slowly integrate the filtered phase error to correct for clock
      // frequency offset.  Uses slotOffsetTime (slow average) so noise
      // doesn't feed directly into the integrator.  Leaky decay prevents
      // windup if conditions change.
      constexpr float kIntervalGain = 0.00002f;
      constexpr float kIntervalMaxPct = 0.01f; // ±1 % of slot interval
      intervalCorrection += (float)slotStartError * kIntervalGain;
      float maxCorr = (float)slotInterval * kIntervalMaxPct;
      if (intervalCorrection > maxCorr)
        intervalCorrection = maxCorr;
      if (intervalCorrection < -maxCorr)
        intervalCorrection = -maxCorr;
    }
  }
  lastPacketRcvTime = receiveStartTime;
}

void FHSS::hopChannel(bool reverse) {
  if (channelSequence.size() == 0)
    return;
  lastChannelHopTime = Core::NowNs();
  if (reverse) {
    if (currentChannelIdx == 0) {
      currentChannelIdx = channelSequence.size();
    }
    currentChannelIdx--;
  } else {
    currentChannelIdx = (currentChannelIdx + 1) % channelSequence.size();
  }
  radioLink.setChannel(channelSequence[currentChannelIdx]);
}

void FHSS::transmitDataPacket(network::DataPacket &packetData,
                              int64_t txTargetTime) {

  auto slotCounterBuf = slotCounter % slotsPerHop;
  auto roleReverseBuf = (roleReverseCounter) % numTxPacketsToRx;
  uint8_t byte1 = ((uint8_t)((roleReverseBuf) & 0x3F) << 2) |
                  (uint8_t)(slotCounterBuf & 0x03);

  packetData.payload.append(byte1);

  auto crc = key + OTA_VERSION + (isRxSide ? 0x40 : 0x00);
  for (size_t i = 0; i < packetData.payload.size(); i++) {
    crc ^= packetData.payload[i];
  }

  packetData.payload.append((uint8_t)crc);

  if (radioLink.setupTxPacket(packetData)) {
    // Load the TX FIFO early (SPI-heavy) so only startTx() remains at the
    // slot boundary.
    radioLink.push();
    // Busy-wait until the exact slot start time for precise TX alignment.
    while (Core::NowNs() < txTargetTime) {
    }
    radioLink.startTx();
  }
  packetData.payload.clear();
}

void FHSS::receivePacket(const network::DataPacket &packet) {
  if (packet.payload.size() < 3) {
    return;
  }

  int64_t now = Core::NowNs();

  size_t payloadEnd = packet.payload.size();

  // Read trailer (appended at the end).
  auto packetCrc = packet.payload[payloadEnd - 1];
  auto byte1 = packet.payload[payloadEnd - 2];

  auto crc = key + OTA_VERSION + (!isRxSide ? 0x40 : 0x00);
  for (size_t i = 0; i < payloadEnd - 1; i++) {
    crc ^= packet.payload[i];
  }
  if (crc != packetCrc) {
    return;
  }

  // int64_t duplicateWindow =
  //     std::max<int64_t>(1 * Core::MILLISECONDS, getAdjustedSlotInterval() /
  //     2);
  // bool duplicatePacket = lastReceivedPacketTime != 0 &&
  //                        now - lastReceivedPacketTime <= duplicateWindow &&
  //                        lastReceivedCounterByte == byte1 &&
  //                        lastReceivedPacketCrc == packetCrc;
  // if (duplicatePacket || thisSlotIsTx) {
  //   return;
  // }

  lastReceivedCounterByte = byte1;
  lastReceivedPacketCrc = packetCrc;
  lastReceivedPacketTime = now;

  uint8_t txSlotCounter = byte1 & 0x03;
  uint8_t txRoleReverseCounter = (byte1 >> 2) & 0x3F;

  if (isRxSide && fhssState != FHSSState::Synced) {
    roleReverseCounter = txRoleReverseCounter;
    slotCounter = txSlotCounter;
  }
  if (isRxSide) {
    if (roleReverseCounter != txRoleReverseCounter) {
      falseCounterCount++;
      if (falseCounterCount > 10) {
        fhssState = FHSSState::Syncing;
        roleReverseCounter = txRoleReverseCounter;
        slotCounter = txSlotCounter;
      }
    }
  }
  syncTimer(packet.timestamp);

  if (!isRxSide || roleReverseCounter == txRoleReverseCounter) {
    receivedPacket = true;
    falseCounterCount = 0;
    if (packet.payload.size() > 2) {
      auto dataPacket = packet;
      dataPacket.payload.popDiscard(2);

      // Serial.printf("Received %d byte packet with payload: ",
      //               dataPacket.payload.size());
      // for (size_t i = 0; i < dataPacket.payload.size(); i++) {
      //   Serial.print(dataPacket.payload[i], HEX);
      //   Serial.print(" ");
      // }
      // Serial.println();

      receiveHandlers_.callHandlers(dataPacket);
    }
  }
}

// =============================================================================
// Task interface
// =============================================================================

void FHSS::taskInit() {
  slotCounter = 0;

  generateChannelSequence(key);

  // Set initial channel.
  if (channelSequence.size() > 0) {
    radioLink.setChannel(channelSequence[0]);
  }

  radioLink.configureRadio();
  lastSeenRxPacketCount = radioLink.getRxPacketCount();

  currentSlotStart = Core::NowNs();

  // Start listening.
  radioLink.push();
  radioLink.startRx(0xFFFF);
}

void FHSS::taskCheck() {}

void FHSS::taskThread() {

  // Poll the radio for any completed RX operations.
  radioLink.pull();
  const uint32_t newRxCount = radioLink.getRxPacketCount();
  if (newRxCount != lastSeenRxPacketCount) {
    lastSeenRxPacketCount = newRxCount;
    receivePacket(radioLink.getRxPacket());
  }

  threadStart = Core::NowNs() - slotOffsetTime;

  timingControl();

  // --- Searching mode (RX side only) ---
  // In searching mode, slowly hop through channels trying to find a signal.
  if (isRxSide && fhssState == FHSSState::Synced &&
      Core::NowNs() - lastPacketRcvTime > 3 * Core::SECONDS) {
    fhssState = FHSSState::Searching;
    desyncCounter++;
    intervalCorrection = 0;
    slotOffsetTime = 0;
    slotTimingOffset = 0;
    receiveSuccesses.clear();
  } else if (isRxSide && fhssState == FHSSState::Syncing &&
             Core::NowNs() - lastPacketRcvTime > 0.5 * Core::SECONDS) {
    fhssState = FHSSState::Searching;
    desyncCounter++;
    intervalCorrection = 0;
    slotOffsetTime = 0;
    slotTimingOffset = 0;
    receiveSuccesses.clear();
  }
  // else if (linkQuality < 0.1f && fhssState == FHSSState::Synced) {
  //   fhssState = FHSSState::Syncing;
  // }

  // --- Sync Info for tx side ---
  if (!isRxSide) {
    fhssState = (Core::NowNs() - lastPacketRcvTime > 1 * Core::SECONDS)
                    ? FHSSState::Searching
                    : FHSSState::Synced;
  }

  // Update link quality stats.
  updateLinkQuality();
}

void FHSS::updateLinkQuality() {

  if (receiveSuccesses.size() < 2) {
    linkQuality = 0;
    otherEndLinkQuality = 0;
    return;
  }

  size_t successCount = 0;
  for (size_t i = 0; i < receiveSuccesses.size(); i++) {
    if (receiveSuccesses[i]) {
      successCount++;
    }
  }

  linkQuality = (float)successCount / (float)receiveSuccesses.size();

  if (!receiveSuccesses(-1) && !receiveSuccesses(-2)) {
    // linkQuality = 0;
    // otherEndLinkQuality = 0;
  }

  if (Core::NowNs() - lastPacketRcvTime > 1 * Core::SECONDS) {
    otherEndLinkQuality = 0;
  }
}

void FHSS::timingControl() {
  // -----------------------------------------------------------------
  // Single-phase slot processing.
  //
  // Each invocation handles one complete slot:
  //   1. Record link quality from the previous slot.
  //   2. Advance slot / hop counters.
  //   3. Hop channel if needed.
  //   4. TX or enter RX immediately.
  //   5. Schedule next wakeup at the next slot boundary.
  //
  // The task wakes up slightly before the slot boundary (via the
  // scheduler release/deadline) so that TX/RX begins right at the
  // slot start.
  // -----------------------------------------------------------------

  slotHandlers.callHandlers(FHSSSlotEvent::SlotEnd, !thisSlotIsTx,
                            receivedPacket, fhssState);

  // --- Update interval correction for clock drift ---
  trueSlotInterval = slotInterval + (int64_t)intervalCorrection +
                     (receivedPacket ? slotTimingOffset * 0.0001 : 0);

  // --- Advance slot timing ---
  lastSlotStart = currentSlotStart;
  currentSlotStart += getAdjustedSlotInterval();

  // --- Catch-up if we fell behind ---
  int64_t interval = getAdjustedSlotInterval();
  if (currentSlotStart + interval < threadStart) {
    int64_t missed = (threadStart - currentSlotStart) / interval;
    currentSlotStart += missed * interval;

    // Compute channel hops for the skipped slots.
    if (channelSequence.size() > 0 && slotsPerHop > 0) {
      size_t totalAdvance = (size_t)missed + 1;
      size_t totalHops = (slotCounter + totalAdvance) / slotsPerHop;
      size_t finalSlot = (slotCounter + totalAdvance) % slotsPerHop;
      // The hop for finalSlot==0 is handled below.
      if (finalSlot == 0 && totalHops > 0) {
        totalHops--;
      }
      currentChannelIdx =
          (currentChannelIdx + totalHops) % channelSequence.size();
    }

    slotCounter = (slotCounter + missed) % slotsPerHop;
    roleReverseCounter = (roleReverseCounter + missed) % numTxPacketsToRx;
  }

  // --- Record link quality for the previous slot ---
  if (lastSlotWasReceive) {
    receiveSuccesses.placeBack(receivedPacket, true);
  }

  // --- Advance counters for this new slot ---
  slotCounter = (slotCounter + 1) % slotsPerHop;
  roleReverseCounter = (roleReverseCounter + 1) % numTxPacketsToRx;

  // Determine the role for this slot:
  //   RX side:  TX when roleReverseCounter == 0, RX otherwise.
  //   TX side:  TX when roleReverseCounter != 0, RX otherwise.
  thisSlotIsTx =
      isRxSide ? (roleReverseCounter == 0) : (roleReverseCounter != 0);

  lastSlotWasReceive = !thisSlotIsTx;
  receivedPacket = false;

  // --- Channel hop (must happen BEFORE any radio operation) ---
  if (isRxSide && fhssState == FHSSState::Searching) {
    if (threadStart - lastSearchHopTime >= slotInterval * 2.2) {
      lastSearchHopTime = threadStart;
      hopChannel(true);
    }
  } else if (slotCounter == 0) {
    hopChannel();
  }

  // --- TX or RX ---
  allowedToTx = !isRxSide || fhssState == FHSSState::Synced;
  blocked = false; // Sx1280_Direct is always ready; we control timing directly.

  if (thisSlotIsTx && allowedToTx && !blocked) {
    int64_t nextSlotStart = currentSlotStart + slotOffsetTime;
    packetToSend.timestamp = nextSlotStart;
    transmitDataPacket(packetToSend, nextSlotStart);
  } else {
    radioLink.push();
    radioLink.startRx(0xFFFF);
  }

  // --- Schedule next wakeup at the next slot boundary ---
  // Wake up slightly before the boundary so the task is ready to
  // act right when the slot starts.
  int64_t nextSlotStart =
      currentSlotStart + getAdjustedSlotInterval() + slotOffsetTime;
  setDeadline(nextSlotStart - 2 * Core::MILLISECONDS);
  setRelease(nextSlotStart - 2 * Core::MILLISECONDS);

  schedulingPhase = true;
}

int64_t FHSS::getAdjustedSlotInterval() const { return trueSlotInterval; }

} // namespace VCTR::ExVectrLink::datalink
