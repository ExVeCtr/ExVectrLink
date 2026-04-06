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

FHSS::FHSS(VCTR::network::datalink::RadioI &radioI)
    : Core::Task_Periodic("FHSS", 100 * Core::MILLISECONDS), radioLink(radioI) {
  setPriority(500);
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
  if (index > 16) {
    index = 16; // Cap at 16 to fit in 4 bits of trailer.
  } else if (index == 0) {
    index = 1; // Cannot be 0
  }
  numTxPacketsToRx = index;
}
uint8_t FHSS::getRxSlotIndex() const { return numTxPacketsToRx; }

// =============================================================================
// Status getters
// =============================================================================

FHSSState FHSS::getFhssState() const { return fhssState; }

float FHSS::getLinkQuality() const {
  return (float)(isRxSide ? linkQuality : otherEndLinkQuality);
}

float FHSS::getSnr() const {
  return isRxSide ? (float)radioLink.lastPacketSNR() : otherEndSnr;
}

int64_t FHSS::getTimingOffset() const { return slotTimingOffset; }

uint8_t FHSS::getSlotCounter() const { return slotCounter; }

// =============================================================================
// Hop Guard
// =============================================================================

void FHSS::addHopGuardedTask(Core::Scheduler::Task &task) {
  hopGuardedTasks.append(&task);
}

// =============================================================================
// DatalinkI interface
// =============================================================================

bool FHSS::transmitDataframe(const VCTR::network::DataPacket &dataframe) {
  if (packetToSend.payload.size() > 0) {
    return false; // Already have a pending packet.
  }

  // uint8_t trailerByte1 =
  //     (0b10000000) | (uint8_t)(uint8_t)(linkQuality * 127.0f) & 0x7F;
  // uint8_t trailerByte2 = ((slotCounter & 0x0F) << 4) | (key & 0x0F);

  packetToSend = dataframe;
  return true;
}

size_t FHSS::getMaxPacketSize() const { return 8; }

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
  // The TX side schedules transmission 100us into the slot, so subtract that
  // to estimate the actual slot boundary on the TX side.
  int64_t estimatedSlotStart = receiveStartTime;

  // Compute phase error: where this packet landed within our slot grid.
  int64_t slotStartError =
      (estimatedSlotStart - currentSlotStart) % slotInterval;

  // Wrap to [-slotInterval/2, +slotInterval/2) so the filter converges
  // correctly regardless of which side of the boundary the error falls on.
  if (slotStartError > slotInterval / 2)
    slotStartError -= slotInterval;
  if (slotStartError < -slotInterval / 2)
    slotStartError += slotInterval;

  if (isRxSide) {
    if (fhssState == FHSSState::Searching) {
      slotTimingOffset = slotStartError;
      slotOffsetTime = slotStartError;
      fhssState = FHSSState::Syncing;

    } else if (fhssState == FHSSState::Syncing) {
      slotTimingOffset = slotStartError;
      slotOffsetTime = slotOffsetTime * 0.75 + slotStartError * 0.25;

      if (linkQuality > 0.9f) {
        fhssState = FHSSState::Synced;
      }

    } else {
      slotTimingOffset = slotTimingOffset * 0.9 + slotStartError * 0.1;
    }
  }
  lastPacketRcvTime = receiveStartTime;
}

void FHSS::hopChannel() {
  if (channelSequence.size() == 0)
    return;
  lastChannelHopTime = Core::NOW();
  currentChannelIdx = (currentChannelIdx + 1) % channelSequence.size();
  radioLink.setChannel(channelSequence[currentChannelIdx]);
}

void FHSS::transmitPacket(network::DataPacket &packet) {
  // Link quality: 4 bits, exponential encoding (cube) for finer resolution
  // near 100%.  encoded = round(15 * lq^3), decoded = (encoded/15)^(1/3).
  float lqClamped =
      linkQuality < 0.0f ? 0.0f : (linkQuality > 1.0f ? 1.0f : linkQuality);
  int lqEnc = (int)(15.0f * lqClamped * lqClamped * lqClamped + 0.5f);
  if (lqEnc > 15)
    lqEnc = 15;

  // Byte 1: SNR (upper nibble, signed 4-bit: -8..+7 dB) | slotCounter (lower
  // nibble)
  int16_t snrRaw = radioLink.lastPacketSNR();
  if (snrRaw > 7)
    snrRaw = 7;
  if (snrRaw < -8)
    snrRaw = -8;
  uint8_t byte1 =
      ((uint8_t)(snrRaw & 0x0F) << 4) | (uint8_t)(slotCounter & 0x0F);

  // Byte 2: roleReverseCounter (upper nibble) | linkQuality (lower nibble)
  auto roleReverseBuf = (roleReverseCounter + 1) % numTxPacketsToRx;
  uint8_t byte2 =
      ((uint8_t)(roleReverseBuf & 0x0F) << 4) | (uint8_t)(lqEnc & 0x0F);

  packet.payload.append(byte1);
  packet.payload.append(byte2);

  auto crc = key;
  for (size_t i = 0; i < packet.payload.size(); i++) {
    crc ^= packet.payload[i];
  }

  packet.payload.append((uint8_t)crc);
  radioLink.transmitDataframe(packet);
  packet.payload.clear();
}

void FHSS::receivePacket(const network::DataPacket &packet) {
  // return;
  if (packet.payload.size() < 3) {
    return;
  }

  size_t payloadEnd = packet.payload.size();

  // Read trailer (appended at the end).
  auto packetCrc = packet.payload[payloadEnd - 1];
  auto byte2 = packet.payload[payloadEnd - 2];
  auto byte1 = packet.payload[payloadEnd - 3];

  auto crc = key;
  for (size_t i = 0; i < payloadEnd - 1; i++) {
    crc ^= packet.payload[i];
  }
  if (crc != packetCrc) {
    return;
  }

  // Byte 1: SNR (upper nibble, signed 4-bit: -8..+7 dB) | slotCounter (lower
  // nibble)
  int8_t txSnr = (int8_t)((byte1 >> 4) | ((byte1 & 0x80) ? 0xF0 : 0x00));
  uint8_t txSlotCounter = byte1 & 0x0F;

  // Byte 2: roleReverseCounter (upper nibble) | linkQuality (lower nibble)
  uint8_t txRoleReverseCounter = (byte2 >> 4) & 0x0F;
  uint8_t lqNibble = byte2 & 0x0F;
  otherEndLinkQuality = cbrtf((float)lqNibble / 15.0f);
  otherEndSnr = (float)txSnr;
  receivedPacket = true;

  // Sync counters from the TX side — ONLY during initial acquisition.
  // When already Synced, let counters free-run via their own progression.
  //
  // Rationale: receivePacket() is called asynchronously from the SX1280
  // driver (higher priority) between FHSS ticks. If we overwrite
  // slotCounter here while Synced, the catch-up logic in timingControl()
  // later adds 'missed' slots to the already-synced counter, double-
  // counting the advancement and computing the wrong number of channel
  // hops → instant desync.
  //
  // Once synced, both sides increment their counters identically (by 1
  // per slot), kept in phase by the timing filter (syncTimer). The
  // counters stay aligned without runtime sync.
  if (isRxSide && fhssState != FHSSState::Synced) {
    roleReverseCounter = txRoleReverseCounter;
    slotCounter = (txSlotCounter + 1) % slotsPerHop;
  }
  syncTimer(packet.timestamp);

  // Forward data packets to application handlers.
  if (packet.payload.size() > 3) {
    auto dataPacket = packet;
    dataPacket.payload.popDiscard(3);
    receiveHandlers_.callHandlers(dataPacket);
  }
}
// =============================================================================
// Task interface
// =============================================================================

void FHSS::taskInit() {
  // Register receive handler on the underlying radio link.
  radioLink.addReceiveHandler(
      [this](const network::DataPacket &packet) { receivePacket(packet); });

  // Initialise timing.
  slotCounter = 0;

  generateChannelSequence(key);

  // Set initial channel.
  if (channelSequence.size() > 0) {
    radioLink.setChannel(channelSequence[0]);
  }

  currentSlotStart = Core::NOW();
}

void FHSS::taskCheck() {

  // if (slotFinalQuart &&
  //     Core::NOW() - currentSlotStart >= getAdjustedSlotInterval()) {
  //   setDeadline(Core::NOW());
  // } else if (slotFirstQuart &&
  //            Core::NOW() - currentSlotStart >= slotGuardMargin) {
  //   setDeadline(Core::NOW());
  // } else if (Core::NOW() - currentSlotStart >= getAdjustedSlotInterval()) {
  //   setDeadline(Core::NOW());
  // }
}

void FHSS::taskThread() {
  threadStart = Core::NOW() - slotOffsetTime;

  timingControl();

  // --- Searching mode (RX side only) ---
  // In searching mode, slowly hop through channels trying to find a signal.
  if (isRxSide && fhssState == FHSSState::Synced &&
      Core::NOW() - lastPacketRcvTime > 5 * Core::SECONDS) {
    fhssState = FHSSState::Searching;
    receiveSuccesses.clear();
  } else if (isRxSide && fhssState == FHSSState::Syncing &&
             Core::NOW() - lastPacketRcvTime > 0.5 * Core::SECONDS) {
    fhssState = FHSSState::Searching;
    receiveSuccesses.clear();
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

  if (Core::NOW() - lastPacketRcvTime > 1 * Core::SECONDS) {
    otherEndLinkQuality = 0;
  }
}

void FHSS::timingControl() {
  // setDeadline(Core::NOW());

  switch (slotPhase) {
  case SlotPhase::Start: {
    slotPhase = SlotPhase::Idle;

    currentSlotStart += getAdjustedSlotInterval();
    trueSlotInterval =
        slotInterval + (receivedPacket ? slotTimingOffset * 0.005 : 0);

    // If we've fallen behind real time (e.g. due to higher-priority tasks
    // delaying us), skip forward instead of rapidly replaying every missed
    // slot.  This prevents a tight catch-up loop that floods the radio with
    // back-to-back scheduling attempts.
    int64_t now = Core::NOW();
    int64_t interval = getAdjustedSlotInterval();
    if (currentSlotStart + interval < now) {
      int64_t missed = (now - currentSlotStart) / interval;
      currentSlotStart += missed * interval;

      // Correctly compute channel hops during the full slot advancement.
      // Total slot increments = missed (catch-up) + 1 (normal +1 below).
      // The number of times slotCounter crosses 0 from position S after
      // K increments is floor((S + K) / slotsPerHop).
      // The Idle phase will call hopChannel() if the final slotCounter
      // lands on 0, so subtract that hop here to avoid double-counting.
      if (channelSequence.size() > 0 && slotsPerHop > 0) {
        size_t totalAdvance = (size_t)missed + 1;
        size_t totalHops = (slotCounter + totalAdvance) / slotsPerHop;
        size_t finalSlot = (slotCounter + totalAdvance) % slotsPerHop;
        if (finalSlot == 0 && totalHops > 0) {
          totalHops--; // Idle phase will handle this hop
        }
        currentChannelIdx =
            (currentChannelIdx + totalHops) % channelSequence.size();
      }

      slotCounter = (slotCounter + missed) % slotsPerHop;
      roleReverseCounter = (roleReverseCounter + missed) % numTxPacketsToRx;
    }

    auto nextRun = currentSlotStart + slotGuardMargin;
    setDeadline(nextRun);
    setRelease(nextRun);

    if (lastSlotWasReceive) {
      receiveSuccesses.placeBack(receivedPacket, true);
    }

    slotCounter = (slotCounter + 1) % slotsPerHop;
    roleReverseCounter = (roleReverseCounter + 1) % numTxPacketsToRx;

    lastSlotWasReceive =
        isRxSide ? roleReverseCounter != 0 : roleReverseCounter == 0;

    receivedPacket = false;

    break;
  }

  case SlotPhase::Idle: {
    slotPhase = SlotPhase::Scheduling;
    auto nextRun =
        currentSlotStart + getAdjustedSlotInterval() - slotGuardMargin;
    setDeadline(nextRun);
    setRelease(nextRun);

    // Hop in Idle phase of slot 0 (first slot of new hop cycle).
    // Runs at +2ms into the slot (slotGuardMargin).
    //
    // TX side: radio is in Transmitting state (TX fired at t=0), so the
    //   freq change is naturally deferred until TX completes (~+3ms).
    //   The radio then applies the freq change and enters RX on the new
    //   channel.
    //
    // RX side: a packet may be arriving (preamble ~2ms, full packet ~3.5ms).
    //   The radio driver's receiveFlagTrig() includes preambleDetected,
    //   so isActivelyReceiving() returns true and the freq change is
    //   deferred until the packet is fully processed. After rxDone, the
    //   radio applies the freq change and enters RX on the new channel.
    //
    // Both sides: the next TX is prepared in Scheduling (+10ms), AFTER
    //   the freq change is applied, so it goes on the new channel.
    if (isRxSide && fhssState == FHSSState::Searching) {
      if (threadStart - lastSearchHopTime >=
          slotInterval * radioLink.getNumChannels()) {
        lastSearchHopTime = threadStart;
        hopChannel();
      }
    } else {
      if (slotCounter == 0) {
        hopChannel();
      }
    }

    setGuardedTasks(false);

    break;
  }

  case SlotPhase::Scheduling: {
    slotPhase = SlotPhase::Start;
    auto nextRun = currentSlotStart + getAdjustedSlotInterval();
    setDeadline(nextRun);
    setRelease(nextRun);

    bool nextSlotTx = isRxSide ? roleReverseCounter + 1 == numTxPacketsToRx
                               : roleReverseCounter + 1 != numTxPacketsToRx;
    bool allowedToTx = !isRxSide || fhssState == FHSSState::Synced;
    bool blocked = radioLink.isChannelBlocked();
    bool hasDataToSend = packetToSend.payload.size() > 0 || isRxSide;
    if (nextSlotTx && allowedToTx && !blocked && hasDataToSend) {
      lastTxPrint = threadStart;
      int64_t txTime = currentSlotStart + getAdjustedSlotInterval();
      packetToSend.timestamp = txTime;
      // Serial.printf(
      //     "Scheduling packet for transmission at %.4f (dT: %.3f ms)\n",
      //     (double)txTime / Core::SECONDS,
      //     (double)(txTime - Core::NOW()) / Core::MILLISECONDS);
      transmitPacket(packetToSend);
    }

    setGuardedTasks(true);

    break;
  }
  }
}

int64_t FHSS::getAdjustedSlotInterval() const { return trueSlotInterval; }

void FHSS::setGuardedTasks(bool paused) {
  for (size_t i = 0; i < hopGuardedTasks.size(); i++) {
    hopGuardedTasks[i]->setPaused(paused);
  }
}

} // namespace VCTR::ExVectrLink::datalink
