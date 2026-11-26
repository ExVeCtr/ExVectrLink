#include <Arduino.h>

#include "ExVectrCore/CanSerialize.hpp"
#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/RadioI.hpp"

#include "ExVectrLink/datalink/FHSS.hpp"
#include "ExVectrLink/datalink/FhssMath.hpp"

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

void FHSS::setSyncLatencyCompensation(int64_t latencyNs) {
  syncLatencyCompensation = latencyNs;
}
int64_t FHSS::getSyncLatencyCompensation() const {
  return syncLatencyCompensation;
}

void FHSS::setSlotsPerHop(uint8_t slots) {
  // The trailer carries the slot counter in 2 bits (known issue: the old cap
  // of 16 broke counter sync above 4). Nothing in the FPV firmware calls this.
  if (slots > fhss::kMaxSlotsPerHop)
    slots = fhss::kMaxSlotsPerHop;
  if (slots == 0)
    slots = 1;
  slotsPerHop = slots;
}
uint8_t FHSS::getSlotsPerHop() const { return slotsPerHop; }

void FHSS::setRxSlotIndex(uint8_t index) {
  // The role counter travels in 6 bits of the trailer (the old cap of 4 was
  // below the default of 15). A period of 1 would leave the TX side without
  // TX slots. Nothing in the FPV firmware calls this.
  if (index > fhss::kMaxRolePeriod) {
    index = fhss::kMaxRolePeriod;
  } else if (index < 2) {
    index = 2;
  }
  numTxPacketsToRx = index;
}

void FHSS::setLinkVersion(uint8_t version) { linkVersion = version; }

void FHSS::setFixedFrameLength(uint8_t length) {
  // At least the trailer plus one payload byte, at most what the radio holds.
  if (length != 0 && length < TRAILER_SIZE + 1)
    length = TRAILER_SIZE + 1;
  fixedFrameLength = length;
}

void FHSS::setWakeupLead(int64_t leadNs) {
  if (leadNs < 0)
    leadNs = 0;
  wakeupLead = leadNs;
}

void FHSS::applyProfile(const AirProfile &profile) {
  setSlotInterval(profile.slotIntervalNs);
  trueSlotInterval = slotInterval;
  setSlotsPerHop(profile.slotsPerHop);
  setRxSlotIndex(profile.rolePeriod);
  setWakeupLead(profile.wakeupLeadNs);
  wakeupLeadRx = profile.wakeupLeadRxNs;
  rxInFlightAtWakeup = profile.fixedFrameLength != 0 && netSlotTiming(profile).wakeupSlackNs < 0;
  setFixedFrameLength(profile.fixedFrameLength);
  setLinkVersion(profile.linkVersion);
  setFhssKey(profile.hopKey);
}
uint8_t FHSS::getRxSlotIndex() const { return numTxPacketsToRx; }

void FHSS::resetDesyncCounter() { desyncCounter = 0; }

void FHSS::triggerCounterDesyncTest() {
  if (channelSequence.size() > 0) {
    // Roughly half way around the hop sequence -- as far from the current
    // channel as the sequence allows.
    currentChannelIdx = (currentChannelIdx + channelSequence.size() / 2 + 1) %
                        channelSequence.size();
    radioLink.setChannel(channelSequence[currentChannelIdx]);
  }
  if (numTxPacketsToRx > 0) {
    roleReverseCounter =
        (roleReverseCounter + numTxPacketsToRx / 2 + 1) % numTxPacketsToRx;
  }
  if (slotsPerHop > 0) {
    slotCounter = (slotCounter + slotsPerHop / 2 + 1) % slotsPerHop;
  }
}

void FHSS::triggerFullResyncTest() {
  triggerCounterDesyncTest();
  if (!isRxSide) {
    return;
  }
  // Mirrors taskThread()'s own desync-timeout reset (see the Synced/Syncing
  // -> Searching transitions there) so this test exercises the exact same
  // reacquisition path, just triggered on demand instead of after the
  // 0.5-3 s wait for lastPacketRcvTime to go stale.
  fhssState = FHSSState::Searching;
  desyncCounter++;
  intervalCorrection = 0;
  slotOffsetTime = 0;
  slotTimingOffset = 0;
  receiveSuccesses.clear();
}
uint32_t FHSS::getDesyncCounter() const { return desyncCounter; }

uint32_t FHSS::getMissedSlotCounter() const { return missedSlotCounter; }

// =============================================================================
// Status getters
// =============================================================================

FHSSState FHSS::getFhssState() const { return fhssState; }

float FHSS::getLinkQuality() const { return linkQuality; }

int64_t FHSS::getTimingOffset() const { return slotTimingOffset; }

int64_t FHSS::getIntervalCorrection() const {
  return (int64_t)intervalCorrection;
}

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

size_t FHSS::getMaxPacketSize() const {
  return fixedFrameLength ? fixedFrameLength - TRAILER_SIZE : 9;
}

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

  // Compensate the constant hardware latency between the true TX-side slot
  // boundary and the driver-reported packet start time. Per-target value,
  // set from HardwareConfig -- see setSyncLatencyCompensation().
  // For tuning: Higher improves TxLQ, lower improves RxLQ.
  receiveStartTime -= syncLatencyCompensation;

  int64_t referenceSlotStart = currentSlotStart;
  int64_t estimatedSlotStart = receiveStartTime - slotOffsetTime;

  int64_t slotStartError = estimatedSlotStart - referenceSlotStart;

  // Wrap to [-slotInterval/2, +slotInterval/2).  The modulo handles the
  // Searching case where the grids may be completely unaligned.
  slotStartError = fhss::wrapPhaseError(slotStartError, slotInterval);

  if (isRxSide) {
    if (fhssState == FHSSState::Searching) {
      // First packet received while searching.  Compute the phase error
      // against the raw (unshifted) grid so that a stale slotOffsetTime
      // left over from a previous sync session doesn't corrupt the
      // initial estimate.
      int64_t rawError =
          fhss::wrapPhaseError(receiveStartTime - currentSlotStart, slotInterval);
      slotTimingOffset = rawError;
      slotOffsetTime = rawError;
      intervalCorrection = 0;
      fhssState = FHSSState::Syncing;

    } else if (fhssState == FHSSState::Syncing) {
      slotTimingOffset = slotStartError;
      fhss::syncingStep(slotOffsetTime, slotStartError);

      if (slotTimingOffset > slotInterval / 10) {
        syncedStartTime = Core::NowNs();
      } else if (Core::NowNs() - syncedStartTime > 500 * Core::MILLISECONDS &&
                 fhss::syncLinkQualityOk(linkQuality, slotsPerHop)) {
        fhssState = FHSSState::Synced;
      }

    } else {
      // Phase filter plus a slow integrator of the phase error that corrects
      // the clock frequency offset (clamped to +-1 % of the slot interval).
      fhss::syncedStep(slotTimingOffset, slotOffsetTime, intervalCorrection,
                       slotStartError, slotInterval);
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

  auto packetCopy = packetData;

  // Fixed frame length (network mode): zero-pad the payload so every frame
  // has the same length on air. Not used by FPV (fixedFrameLength == 0).
  if (fixedFrameLength) {
    const size_t payloadLen = fixedFrameLength - TRAILER_SIZE;
    while (packetCopy.payload.size() < payloadLen) {
      packetCopy.payload.append(0);
    }
  }

  uint8_t byte1 = fhss::trailerCounterByte(roleReverseCounter, numTxPacketsToRx,
                                           slotCounter, slotsPerHop);

  packetCopy.payload.append(byte1);

  uint8_t crc = fhss::trailerChecksum(
      fhss::trailerSeed(key, linkVersion, isRxSide), packetCopy.payload.getPtr(),
      packetCopy.payload.size());

  packetCopy.payload.append(crc);

  if (radioLink.setupTxPacket(packetCopy)) {
    // Load the TX FIFO early (SPI-heavy) so only startTx() remains at the
    // slot boundary.
    radioLink.push(true);
    // Busy-wait until the exact slot start time for precise TX alignment.
    // auto margin = txTargetTime - Core::NowNs();
    // LOG_MSG("TX slot margin: %.3f us", margin / Core::MICROSECONDS);
    // The first clock read of the wait doubles as the "prep finished" stamp
    // for SlotDiag (same number of reads as a plain while (NowNs() < t) loop).
    int64_t t = Core::NowNs();
    const int64_t prep = t - wakeNs_;
    diag_.prepSumNs += prep;
    if (prep > diag_.prepMaxNs)
      diag_.prepMaxNs = prep;
    if (t > txTargetTime)
      diag_.txLateStart++;
    while (t < txTargetTime) {
      t = Core::NowNs();
    }
    if (txDoneCount_) {
      if (txAwaitDone_)
        diag_.airLate++; // previous frame's TX done never came (the radio was re-armed over it)
      txCountAtStart_ = txDoneCount_();
      txStartNs_ = t;
      txAwaitDone_ = true;
    }
    radioLink.startTx();
    diag_.txStarted++;
    if (t - txTargetTime > diag_.startLateMaxNs)
      diag_.startLateMaxNs = t - txTargetTime;
  } else {
    diag_.txSkipSetup++;
  }
  packetData.payload.clear();
}

void FHSS::receivePacket(const network::DataPacket &packet) {
  if (packet.payload.size() < 2) {
    return;
  }

  int64_t now = Core::NowNs();

  size_t payloadEnd = packet.payload.size();

  // Read trailer (appended at the end).
  auto packetCrc = packet.payload[payloadEnd - 1];
  auto byte1 = packet.payload[payloadEnd - 2];

  // The sender is the other side: its side bit is the opposite of ours.
  uint8_t crc = fhss::trailerChecksum(fhss::trailerSeed(key, linkVersion, !isRxSide),
                                      packet.payload.getPtr(), payloadEnd - 1);
  if (crc != packetCrc) {
    diag_.rxTrailerBad++;
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

  uint8_t txSlotCounter = fhss::trailerSlotCounter(byte1);
  uint8_t txRoleReverseCounter = fhss::trailerRoleCounter(byte1);

  if (isRxSide && fhssState != FHSSState::Synced) {
    roleReverseCounter = txRoleReverseCounter;
    slotCounter = txSlotCounter;
  }
  if (isRxSide) {
    if (roleReverseCounter != txRoleReverseCounter) {
      // This packet passed both the LoRa CRC16 and the trailer checksum, so
      // its counter is almost certainly right and OURS is off -- typically a
      // missed-slot catch-up that advanced the local counters wrongly. Every
      // mismatched packet is rejected below despite being perfectly good, so
      // waiting 10 of them (the old rule) burned ~10 slots of LQ per desync
      // and could meanwhile place our TX slot on top of the other side's
      // transmissions. Two consecutive packets agreeing on the same shift is
      // conclusive (random corruption reproducing the same delta twice is
      // implausible): resnap immediately and keep Synced -- the timing sync
      // is untouched, only the counters were shifted. Fall back to the old
      // threshold with a state drop for inconsistent garbage.
      const uint8_t delta =
          (uint8_t)((txRoleReverseCounter + numTxPacketsToRx -
                     (roleReverseCounter % numTxPacketsToRx)) %
                    numTxPacketsToRx);
      falseCounterCount++;
      if (falseCounterCount >= 2 && delta == lastCounterDelta) {
        roleReverseCounter = txRoleReverseCounter;
        slotCounter = txSlotCounter;
        counterResyncCount++;
        falseCounterCount = 0;
      } else if (falseCounterCount > 10) {
        fhssState = FHSSState::Syncing;
        roleReverseCounter = txRoleReverseCounter;
        slotCounter = txSlotCounter;
        counterResyncCount++;
        falseCounterCount = 0;
      }
      lastCounterDelta = delta;
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
  } else {
    diag_.rxCounterMismatch++;
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

  radioLink.setAutoFS(true);
  radioLink.configureRadio();
  lastSeenRxPacketCount = radioLink.getRxPacketCount();

  currentSlotStart = Core::NowNs();

  // Start listening.
  radioLink.push();
  radioLink.startRx(0xFFFF);
}

void FHSS::taskCheck() {}

void FHSS::taskThread() {

  // SlotDiag: when did the scheduler really start us, against what timingControl() asked for.
  wakeNs_ = Core::NowNs();
  diag_.wakeups++;
  if (intendedWake_ != 0) {
    const int64_t wakeLate = wakeNs_ - intendedWake_;
    if (wakeLate > diag_.wakeLateMaxNs)
      diag_.wakeLateMaxNs = wakeLate;
    if (wakeLate > 1 * Core::MILLISECONDS)
      diag_.wakeLate1ms++;
  }

  // Poll the radio for any completed RX operations.
  radioLink.pull();
  uint32_t newRxCount = radioLink.getRxPacketCount();

  // SlotDiag air=: pull() above is the only place TX done is read (once per slot), so the frame's done is observed
  // here at the earliest. wakeNs_ is reused as the observation time (pull() itself is a fraction of a millisecond).
  if (txAwaitDone_ && txDoneCount_() != txCountAtStart_) {
    txAwaitDone_ = false;
    const int64_t air = wakeNs_ - txStartNs_;
    diag_.airCount++;
    diag_.airSumNs += air;
    if (diag_.airMinNs == 0 || air < diag_.airMinNs)
      diag_.airMinNs = air;
    if (air > diag_.airMaxNs)
      diag_.airMaxNs = air;
    if (air > getAdjustedSlotInterval())
      diag_.airLate++;
  }

  // A packet can still be in the air at this wakeup: RX_DONE lands only
  // ~1 ms before the slot boundary with the current airtime/slot budget, so
  // a transmission that left the other side slightly late (its scheduler
  // woke past the busy-wait lead) has not completed yet. Without this, the
  // slot processing below would re-arm RX / hop channel, aborting the
  // reception and losing the packet outright even though it was only
  // marginally late. Grant in-flight receptions a bounded grace window:
  // keep polling until the packet completes (or errors out), capped so a
  // noise-triggered false preamble detect can never stall the schedule.
  //
  // The cap depends on the upcoming slot's role:
  //  - RX next: up to ~0.5 ms PAST the boundary. Arming RX slightly late is
  //    safe -- the preamble is 12 symbols (~1.9 ms) and detection only
  //    needs part of it.
  //  - TX next: up to ~0.5 ms BEFORE the boundary, leaving room for the TX
  //    prep (hop, FIFO load, push) plus the busy-wait so the transmission
  //    still leaves exactly on the boundary the other side syncs to. A
  //    blanket skip here instead loses the packet before every own-TX slot
  //    outright once tuning settles packets past the wakeup: exactly 1 of
  //    numTxPacketsToRx-1 receive slots per cycle, a hard LQ ceiling.
  // The TX side's state is "Searching" until its first packet arrives, so a profile whose frame is on air at the
  // wakeup (rxInFlightAtWakeup, v2-trial) must grant the window to the TX side regardless, or it never locks.
  if (newRxCount == lastSeenRxPacketCount &&
      (fhssState != FHSSState::Searching || (rxInFlightAtWakeup && !isRxSide)) && radioLink.isReceivingPacket()) {
    uint8_t nextRoleReverseCounter =
        (roleReverseCounter + 1) % numTxPacketsToRx;
    bool nextSlotIsTx = fhss::slotIsTx(isRxSide, nextRoleReverseCounter);
    const int64_t nextSlotBoundary =
        currentSlotStart + getAdjustedSlotInterval() + slotOffsetTime;
    const int64_t graceDeadline =
        nextSlotBoundary + (nextSlotIsTx ? -kGraceTxCutoffNs : 500 * Core::MICROSECONDS);
    while (Core::NowNs() < graceDeadline) {
      radioLink.pull();
      newRxCount = radioLink.getRxPacketCount();
      if (newRxCount != lastSeenRxPacketCount ||
          !radioLink.isReceivingPacket()) {
        break;
      }
    }
    if (newRxCount != lastSeenRxPacketCount) {
      graceRescueCount++;
    } else {
      graceExpireCount++;
    }
  }

  if (newRxCount != lastSeenRxPacketCount) {
    diag_.rxRadioPkts += newRxCount - lastSeenRxPacketCount;
    lastSeenRxPacketCount = newRxCount;
    // Only now fetch the actual FIFO payload bytes -- pull() alone only reads
    // cheap RSSI/SNR/length status. In diversity this means the losing
    // radio's FIFO is never read at all; see Sx1280_DirectI::fetchRxPayload().
    radioLink.fetchRxPayload();
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
    return;
  }

  size_t successCount = 0;
  for (size_t i = 0; i < receiveSuccesses.size(); i++) {
    if (receiveSuccesses[i]) {
      successCount++;
    }
  }

  linkQuality = (float)successCount / (float)receiveSuccesses.size();
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
  trueSlotInterval =
      slotInterval + (int64_t)intervalCorrection +
      (receivedPacket ? slotTimingOffset * 0.0001 : 0) +
      (fhssState == FHSSState::Searching && isRxSide ? slotInterval * 0.9 : 0);

  // --- Advance slot timing ---
  lastSlotStart = currentSlotStart;
  currentSlotStart += getAdjustedSlotInterval();

  // --- Catch-up if we fell behind ---
  int64_t interval = getAdjustedSlotInterval();
  if (currentSlotStart + interval < threadStart) {
    int64_t missed = (threadStart - currentSlotStart) / interval;
    currentSlotStart += missed * interval;

    // Count the slots we had to fast-forward past: the task woke too late to
    // service them at their boundary. This is a pure scheduling-latency
    // metric, independent of RF packet loss.
    missedSlotCounter += (uint32_t)missed;
    diag_.missEvents++;

    // Record missed rx slots as false in link quality buffer
    int64_t missedToRecord = missed > 100 ? 100 : missed;
    for (int64_t i = missed - missedToRecord + 1; i <= missed; i++) {
      uint8_t missedRoleReverseCounter =
          (roleReverseCounter + i) % numTxPacketsToRx;
      bool missedSlotIsTx = fhss::slotIsTx(isRxSide, missedRoleReverseCounter);
      if (!missedSlotIsTx) {
        receiveSuccesses.placeBack(false, true);
      } else {
        diag_.txMissed++;
      }
    }

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
    diag_.rxSlots++;
    if (receivedPacket)
      diag_.rxValid++;
    else
      diag_.rxNone++;
  }

  // --- Advance counters for this new slot ---
  slotCounter = (slotCounter + 1) % slotsPerHop;
  roleReverseCounter = (roleReverseCounter + 1) % numTxPacketsToRx;

  // Determine the role for this slot:
  //   RX side:  TX when roleReverseCounter == 0, RX otherwise.
  //   TX side:  TX when roleReverseCounter != 0, RX otherwise.
  thisSlotIsTx = fhss::slotIsTx(isRxSide, roleReverseCounter);

  lastSlotWasReceive = !thisSlotIsTx;
  receivedPacket = false;

  bool hoppedChannel = false;
  // --- Channel hop (must happen BEFORE any radio operation) ---
  if (isRxSide && fhssState == FHSSState::Searching) {
    if (threadStart - lastSearchHopTime >= slotInterval) {
      lastSearchHopTime = threadStart;
      hopChannel(true);
      hoppedChannel = true;
    }
  } else if (slotCounter == 0) {
    hopChannel();
    hoppedChannel = true;
  }

  // --- TX or RX ---
  allowedToTx = !isRxSide || fhssState == FHSSState::Synced;
  if (thisSlotIsTx) {
    diag_.txSlots++;
    if (!allowedToTx)
      diag_.txSkipNotSynced++;
  }

  if (thisSlotIsTx && allowedToTx) {
    int64_t nextSlotStart = currentSlotStart + slotOffsetTime;
    if (txProvider != nullptr) {
      // Pull mode (network mode): the payload is built now, inside the
      // wakeup lead, so it is as fresh as possible. Never used by FPV.
      uint8_t buf[255];
      size_t maxLen = getMaxPacketSize();
      if (maxLen > sizeof(buf))
        maxLen = sizeof(buf);
      size_t n = txProvider->fillTxPayload(buf, maxLen, nextSlotStart);
      if (n > maxLen)
        n = maxLen;
      packetToSend.payload.setSize(n);
      for (size_t i = 0; i < n; i++)
        packetToSend.payload[i] = buf[i];
    }
    packetToSend.timestamp = nextSlotStart;
    transmitDataPacket(packetToSend, nextSlotStart);
  } else {
    // if (hoppedChannel) {
    //   radioLink.push();
    // }
    radioLink.push();
    radioLink.startRx(0xFFFF);
  }

  // --- Schedule next wakeup at the next slot boundary ---
  // Wake up slightly before the boundary so the task is ready to
  // act right when the slot starts.
  int64_t nextSlotStart =
      currentSlotStart + getAdjustedSlotInterval() + slotOffsetTime;
  int64_t wakeupLeadTime = wakeupLead;
  if (wakeupLeadRx > 0 &&
      !fhss::slotIsTx(isRxSide, (roleReverseCounter + 1) % numTxPacketsToRx)) {
    // The next slot is a receive slot: the wakeup only hops and arms RX, so it can wait until the own TX has ended.
    wakeupLeadTime = wakeupLeadRx;
  }

  intendedWake_ = nextSlotStart - wakeupLeadTime;
  setDeadline(nextSlotStart - wakeupLeadTime);
  setRelease(nextSlotStart - wakeupLeadTime);

  schedulingPhase = true;
}

int64_t FHSS::getAdjustedSlotInterval() const { return trueSlotInterval; }

} // namespace VCTR::ExVectrLink::datalink
