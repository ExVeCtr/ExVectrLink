#ifndef EXVECTRLINK_FHSS_HPP
#define EXVECTRLINK_FHSS_HPP

#include "ExVectrCore/IntervalTimer.hpp"
#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/RadioI.hpp"
#include "ExVectrNetwork/datalink/sx1280/Sx1280_Direct.hpp"

namespace VCTR::ExVectrLink::datalink {

enum class FHSSState : uint8_t {
  Searching, ///< Not synced, scanning channels to find a signal.
  Syncing,   ///< Detected a signal and is adjusting timing to sync.
  Synced,    ///< Synced with the other side, normal operation.
};

enum class FHSSSlotEvent : uint8_t {
  SlotStart,
  SlotEnd,
};

/**
 * @brief FHSS (Frequency Hopping Spread Spectrum) with slot-based timing.
 *
 * ## Timing Architecture
 *
 * The timing is organized in two hierarchical levels:
 *
 * - **Slots** -- The base timing unit. Each slot is one TX or RX operation.
 *   Duration is configured via setSlotInterval().
 *
 * - **Hops** -- A frequency channel hop occurs every slotsPerHop slots.
 *   The hop happens at slot index 0 (start of each hop cycle).
 *
 * ## Role Reversal
 *
 * Within each hop cycle, one slot (at rxSlotIndex) is a role-reversal slot:
 * - The TX side switches to RX to listen for the other end.
 * - The RX side switches to TX to send data back.
 *
 * All other slots: TX side transmits, RX side receives.
 *
 * ## Early RX Offset
 *
 * When transitioning to a receive slot, the receiving side switches
 * rxEarlyOffset nanoseconds early to avoid missing the preamble
 * due to timing drift between the two sides. If a hop is also due,
 * the channel switch happens early too.
 */
class FHSS : public VCTR::network::datalink::DatalinkI,
             public VCTR::Core::Task_Periodic {
private:
  using HandlerFunction = std::function<void(
      FHSSSlotEvent, bool isRxSlot, bool receivedPacket, FHSSState fhssState)>;

public:
  FHSS(VCTR::network::datalink::Sx1280_DirectI &radio);

  // ===================== Configuration =====================

  /// @brief Set the FHSS key used for hopping sequence generation and packet
  /// validation.
  void setFhssKey(uint8_t key);
  uint8_t getFhssKey() const;

  /// @brief Set whether this node is the RX (receiver/secondary) side.
  /// The TX side is always considered synced. The RX side syncs to the TX side.
  void setIsRxSide(bool isRxSide);

  /// @brief Set the duration of a single slot in nanoseconds.
  void setSlotInterval(int64_t interval);
  int64_t getSlotInterval() const;

  /// @brief Set the sync latency compensation in nanoseconds (RX side only).
  ///
  /// Constant latency between the true TX-side slot boundary and the
  /// driver-reported packet start time that syncTimer() subtracts before
  /// computing the phase error. It absorbs everything the datasheet
  /// time-on-air formula cannot capture: the far end's setTx() SPI command +
  /// FS->TX startup ramp, the local SX1280's RX_DONE assertion delay after
  /// the last symbol, and -- the reason this is per-target -- the local
  /// slot-servicing time (SPI traffic per slot, scheduler wake latency),
  /// which shifts where the RX grid must sit for both link directions to
  /// have margin. Configure from the target's HardwareConfig; calibrate per
  /// hardware target and per modulation settings.
  void setSyncLatencyCompensation(int64_t latencyNs);
  int64_t getSyncLatencyCompensation() const;

  /// @brief Set how many slots make up one hop cycle. Hop occurs at slot 0.
  void setSlotsPerHop(uint8_t slots);
  uint8_t getSlotsPerHop() const;

  /// @brief Set the number of tx packets to transmit before a rx packet is
  /// sent. Max 16.
  void setRxSlotIndex(uint8_t index);
  uint8_t getRxSlotIndex() const;

  void resetDesyncCounter();
  uint32_t getDesyncCounter() const;

  /// @brief Test hook: jumps to a far-away channel and shifts the local
  /// slot/role counters, without touching fhssState or the timing filters.
  /// Simulates the other side's counters having silently diverged (e.g. a
  /// missed-slot catch-up misfire) while everything else about the link
  /// stays healthy -- exercises the resync-on-agreement path in
  /// receivePacket() rather than a full reacquisition.
  void triggerCounterDesyncTest();

  /// @brief Test hook: same channel/counter perturbation as
  /// triggerCounterDesyncTest(), but additionally forces fhssState straight
  /// to Searching (RX side only) -- the same reset timingControl()'s own
  /// desync-timeout path performs, just without waiting the 0.5-3 s for that
  /// timeout to fire, so reacquisition can be tested back-to-back.
  void triggerFullResyncTest();

  /// @brief Number of slots skipped because the FHSS task was scheduled too
  /// late to service them at their boundary (i.e. the catch-up path in
  /// timingControl() had to fast-forward the slot grid) -- distinct from
  /// packets lost to a bad link. Cumulative since boot.
  uint32_t getMissedSlotCounter() const;

  /// @brief Times the local slot/role counters were resnapped to the counter
  /// received in a packet trailer (RX side only). Each event means the local
  /// counters had drifted from the other side -- typically a missed-slot
  /// catch-up misfire -- and good packets were being rejected until the
  /// resnap. Cumulative since boot.
  uint32_t getCounterResyncCount() const { return counterResyncCount; }

  /// @brief Packets that were only received because the in-flight grace
  /// window at the slot wakeup waited for their late RX_DONE. Cumulative.
  uint32_t getGraceRescueCount() const { return graceRescueCount; }

  /// @brief Times the grace window expired without the reception completing
  /// (very late packet lost anyway, or a noise-triggered false preamble
  /// detect). Cumulative since boot.
  uint32_t getGraceExpireCount() const { return graceExpireCount; }

  // ===================== Status =====================

  FHSSState getFhssState() const;

  /// @brief Returns link quality as 0.0 (no packets) to 1.0 (a valid
  /// packet in every receive slot). Counts any valid frame, with or
  /// without payload data -- an empty keep-alive frame is just as much
  /// proof of a healthy link as a full one.
  float getLinkQuality() const;

  /// @brief Returns the current timing offset correction in nanoseconds.
  int64_t getTimingOffset() const;

  /// @brief Returns the slot-interval clock-drift correction currently applied
  /// (nanoseconds). This is the slow integrator compensating for the TX/RX
  /// crystal frequency mismatch, distinct from getTimingOffset() (the per-slot
  /// phase error). RX side only; ~0 on the TX side.
  int64_t getIntervalCorrection() const;

  /// @brief Returns the actual slot interval including corrections (ns).
  int64_t getTrueSlotInterval() const;

  /// @brief Returns the current slot counter within the hop cycle.
  uint8_t getSlotCounter() const;

  // ===================== Events =====================

  void addSlotHandler(HandlerFunction handler);

  // ===================== DatalinkI Interface =====================

  bool transmitDataframe(const VCTR::network::DataPacket &dataframe) override;
  size_t getMaxPacketSize() const override;
  bool isChannelBlocked() const override;

  // ===================== Task Interface =====================

  void taskCheck() override;
  void taskInit() override;
  void taskThread() override;

private:
  static constexpr uint8_t OTA_VERSION = 2;

  /// @brief Number of trailer bytes appended to each outgoing frame.
  /// Layout: 1 bit data flag | 7 bits Quality | 4 bit slotCounter | 4 bit key
  static constexpr size_t TRAILER_SIZE = 2;

  // ---- Internal methods ----
  void generateChannelSequence(uint8_t key);

  void syncTimer(int64_t receiveStartTime);
  void hopChannel(bool reverse = false);

  void transmitDataPacket(network::DataPacket &packetData,
                          int64_t txTargetTime);
  void receivePacket(const network::DataPacket &packet);

  void updateLinkQuality();

  void timingControl();

  int64_t getAdjustedSlotInterval() const;

  // ======================= Configuration =======================

  int64_t slotInterval = 10 * Core::MILLISECONDS;
  int64_t trueSlotInterval = slotInterval;
  size_t slotsPerHop = 4;

  /// See setSyncLatencyCompensation(). Per-target; set from HardwareConfig.
  int64_t syncLatencyCompensation = 900 * Core::MICROSECONDS;

  bool isRxSide = false;
  // After this amount of tx Packets, send an rx Packet. Max 16
  uint8_t numTxPacketsToRx = 15;

  // ===================== State =====================

  VCTR::network::datalink::Sx1280_DirectI &radioLink;

  // ---- Channel sequence ----
  VCTR::Core::ListArray<uint8_t> channelSequence;
  size_t currentChannelIdx = 0;
  uint8_t key = 0;
  int64_t lastChannelHopTime = 0;

  // ----- Timing -----
  int64_t threadStart = 0;
  bool schedulingPhase = false;

  /// Correction applied to the slot interval to compensate for clock
  /// frequency mismatch between TX and RX (RX side only, in nanoseconds).
  float intervalCorrection = 0;

  // ---- Slot state ----
  int64_t currentSlotStart = 0;
  int64_t lastSlotStart = 0;
  int64_t slotTimingOffset = 0;
  int64_t slotOffsetTime = 0;
  size_t slotCounter = 0;        // Counts from 0 to slotsPerHop
  size_t roleReverseCounter = 0; // Counts from 0 to numTxPacketsToRx
  bool lastSlotWasReceive = false;
  bool receivedPacket = false;
  bool txSlotTrig = false;
  int64_t lastPacketRcvTime = 0;
  bool thisSlotIsTx = false;
  bool allowedToTx = false;
  bool blocked = false;

  Core::HandlerGroup<FHSSSlotEvent, bool, bool, FHSSState> slotHandlers;

  // ---- Sync state ----
  FHSSState fhssState = FHSSState::Searching;
  int64_t lastSearchHopTime = 0;
  int64_t syncedStartTime = 0;
  size_t falseCounterCount = 0;
  uint8_t lastCounterDelta = 0;
  uint8_t lastReceivedCounterByte = 0;
  uint8_t lastReceivedPacketCrc = 0;
  int64_t lastReceivedPacketTime = 0;

  uint32_t desyncCounter = 0;
  uint32_t missedSlotCounter = 0;
  uint32_t counterResyncCount = 0;
  uint32_t graceRescueCount = 0;
  uint32_t graceExpireCount = 0;
  int64_t lastTxPrint = 0;

  // ---- Poll-based RX tracking ----
  uint32_t lastSeenRxPacketCount = 0;

  // ---- Packet data ----
  VCTR::network::DataPacket packetToSend;

  // ---- Link quality tracking ----
  VCTR::Core::ListBuffer<bool, 100> receiveSuccesses;
  float linkQuality = 0;
};

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_FHSS_HPP