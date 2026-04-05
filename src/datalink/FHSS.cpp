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
  // uint32_t seed = key * 2654435761u; // Knuth multiplicative hash
  // for (size_t i = numChannels - 1; i > 0; i--) {
  //   seed ^= (seed << 13);
  //   seed ^= (seed >> 17);
  //   seed ^= (seed << 5);
  //   size_t j = seed % (i + 1);
  //   // Swap
  //   uint8_t tmp = channelSequence[i];
  //   channelSequence[i] = channelSequence[j];
  //   channelSequence[j] = tmp;
  // }

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
      slotOffsetTime = slotTimingOffset;
    } else {
      slotTimingOffset = slotTimingOffset * 0.75 + slotStartError * 0.25;
      // slotOffsetTime = slotTimingOffset;
    }
  }
  fhssState = FHSSState::Synced;
  lastPacketRcvTime = receiveStartTime;
}

void FHSS::hopChannel() {
  if (channelSequence.size() == 0)
    return;
  lastChannelHopTime = Core::NOW();
  currentChannelIdx = (currentChannelIdx + 1) % channelSequence.size();
  // radioLink.setChannel(channelSequence[currentChannelIdx]);
}

void FHSS::transmitPacket(network::DataPacket &packet) {
  // Pack link quality (upper 4 bits) and SNR (lower 4 bits) into one byte.
  // LQ:  0-15 maps to 85-100%   (encode: clamp((lq - 0.85) / 0.15 * 15))
  // SNR: 0-15 maps to -20..+15 dB (encode: clamp((snr + 20) * 15 / 35))
  int lqEnc = (int)((linkQuality - 0.85f) / 0.15f * 15.0f + 0.5f);
  if (lqEnc < 0)
    lqEnc = 0;
  if (lqEnc > 15)
    lqEnc = 15;

  int snrRaw = radioLink.lastPacketSNR();
  int snrEnc = (snrRaw + 20) * 15 / 35;
  if (snrEnc < 0)
    snrEnc = 0;
  if (snrEnc > 15)
    snrEnc = 15;

  uint8_t lqSnrByte = ((uint8_t)lqEnc << 4) | (uint8_t)snrEnc;

  auto slotCountBuf = (slotCounter + 1) % slotsPerHop;
  auto roleReverseBuf = (roleReverseCounter + 1) % numTxPacketsToRx;

  // Pack slotCounter (upper nibble) and roleReverseCounter (lower nibble)
  // into one byte.  Both fit in 4 bits (max 16 each).
  uint8_t slotRoleByte =
      ((uint8_t)(slotCountBuf & 0x0F) << 4) | (uint8_t)(roleReverseBuf & 0x0F);

  packet.payload.append(slotRoleByte);
  packet.payload.append(lqSnrByte);

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
  auto packetKey = packet.payload[payloadEnd - 1];
  auto lqSnrByte = packet.payload[payloadEnd - 2];
  auto slotRoleByte = packet.payload[payloadEnd - 3];

  auto crc = key;
  for (size_t i = 0; i < payloadEnd - 1; i++) {
    crc ^= packet.payload[i];
  }
  // Check CRC and Key simultaneously:
  if (crc != packetKey) {
    return;
  }

  // Unpack slotCounter (upper nibble) and roleReverseCounter (lower nibble).
  uint8_t txSlotCounter = (slotRoleByte >> 4) & 0x0F;
  uint8_t txRoleReverseCounter = slotRoleByte & 0x0F;

  // Decode packed LQ (upper 4 bits) and SNR (lower 4 bits).
  // LQ:  0-15 → 85-100%     (decode: 0.85 + nibble * 0.15 / 15)
  // SNR: 0-15 → -20..+15 dB (decode: nibble * 35 / 15 - 20)
  uint8_t lqNibble = (lqSnrByte >> 4) & 0x0F;
  uint8_t snrNibble = lqSnrByte & 0x0F;
  otherEndLinkQuality = 0.85f + (float)lqNibble * 0.01f;
  otherEndSnr = (float)snrNibble * 35.0f / 15.0f - 20.0f;
  receivedPacket = true;

  // Serial.printf("Received packet with LQ %.2f and SNR %.1f dB\n",
  //               otherEndLinkQuality, otherEndSnr);

  // Resync role-reversal counter and slot counter to match the TX side.
  if (isRxSide) {
    roleReverseCounter = txRoleReverseCounter;
    // slotCounter = txSlotCounter;
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
  threadStart = Core::NOW();

  timingControl();

  // --- Searching mode (RX side only) ---
  // In searching mode, slowly hop through channels trying to find a signal.
  if (isRxSide && fhssState == FHSSState::Synced &&
      Core::NOW() - lastPacketRcvTime > 3 * Core::SECONDS) {
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

    // if (Core::NOW() > currentSlotStart + getAdjustedSlotInterval()*1.1) {
    //   Serial.printf("Warning: FHSS slot overrun! Now: %.3f, currentSlotStart:
    //   %.3f, interval: %.3f\n",
    //                 (double)Core::NOW() / Core::SECONDS,
    //                 (double)currentSlotStart / Core::SECONDS,
    //                 (double)getAdjustedSlotInterval() / Core::SECONDS);
    // }

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

    if (!isRxSide || fhssState == FHSSState::Synced) {
      if (slotCounter == 0) {
        hopChannel();
      }
    } else if (fhssState == FHSSState::Searching) {
      if (threadStart - lastSearchHopTime >=
          slotInterval * radioLink.getNumChannels()) {
        lastSearchHopTime = threadStart;
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
    if (nextSlotTx && allowedToTx && !blocked) {
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
