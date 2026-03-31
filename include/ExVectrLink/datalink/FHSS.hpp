#ifndef EXVECTRLINK_FHSS_HPP
#define EXVECTRLINK_FHSS_HPP

#include "ExVectrCore/IntervalTimer.hpp"
#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/RadioI.hpp"

namespace VCTR::ExVectrLink::datalink {

enum class FHSSState : uint8_t {
  Searching, ///< Not synced, scanning channels to find a signal.
  Synced,    ///< Synced with the other side, normal operation.
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
 *
 * ## Example (slotsPerHop=4, rxSlotIndex=3):
 * @code
 *   Slot:  |  0 [HOP] |    1     |    2     |  3 [REV] |  0 [HOP] | ...
 *   TX:    |    TX    |    TX    |    TX    |    RX    |    TX    | ...
 *   RX:    |   RX*    |   RX*   |   RX*   |    TX    |   RX*    | ...
 *                                                       * = early offset
 * applied
 * @endcode
 */
class FHSS : public VCTR::network::datalink::DatalinkI,
             public VCTR::Core::Task_Periodic {

public:
  FHSS(VCTR::network::datalink::RadioI &radioI);

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

  /// @brief Set how many slots make up one hop cycle. Hop occurs at slot 0.
  void setSlotsPerHop(uint8_t slots);
  uint8_t getSlotsPerHop() const;

  /// @brief Set the number of tx packets to transmit before a rx packet is
  /// sent. Max 16.
  void setRxSlotIndex(uint8_t index);
  uint8_t getRxSlotIndex() const;

  // ===================== Status =====================

  FHSSState getFhssState() const;

  /// @brief Returns link quality as 0.0 (no link) to 1.0 (perfect).
  float getLinkQuality() const;

  /// @brief Returns the current timing offset correction in nanoseconds.
  int64_t getTimingOffset() const;

  /// @brief Returns the current slot counter within the hop cycle.
  uint8_t getSlotCounter() const;

  // ===================== Hop Guard =====================

  /// @brief Register a scheduler task to be paused during the hop guard
  /// window. The task will be automatically paused ~margin before and after
  /// each frequency hop to protect timing-critical radio operations.
  void addHopGuardedTask(Core::Scheduler::Task &task);

  /// @brief Remove a previously registered hop-guarded task.
  void removeHopGuardedTask(Core::Scheduler::Task &task);

  /// @brief Set the hop guard margin in nanoseconds. Guarded tasks are paused
  /// this long before and after each hop. Default is 2 ms.
  void setHopGuardMargin(int64_t marginNs);

  /// @brief Returns true if currently within the hop guard window.
  bool isInHopGuardWindow() const;

  // ===================== DatalinkI Interface =====================

  bool transmitDataframe(const VCTR::network::DataPacket &dataframe) override;
  size_t getMaxPacketSize() const override;
  bool isChannelBlocked() const override;

  // ===================== Task Interface =====================

  void taskCheck() override;
  void taskInit() override;
  void taskThread() override;

private:
  /// @brief Internal packet types appended to each frame as a trailer.
  enum class PacketType : uint8_t {
    Data,     ///< User data packet.
    LinkInfo, ///< Keep-alive / link quality info packet.
  };

  /// @brief Number of trailer bytes appended to each outgoing frame.
  /// Layout: 1 bit data flag | 7 bits Quality | 4 bit slotCounter | 4 bit key
  // static constexpr size_t TRAILER_SIZE = 2;

  // ---- Internal methods ----
  void generateChannelSequence(uint8_t key);

  void syncTimer(int64_t receiveStartTime);
  void hopChannel();

  void transmitPacket(network::DataPacket &packet);
  void receivePacket(const network::DataPacket &packet);

  void updateLinkQuality();

  void timerEvent(bool isSlotStart);

  // ======================= Configuration =======================

  int64_t slotInterval = 20 * Core::MILLISECONDS;
  size_t slotsPerHop = 1;

  bool isRxSide = false;
  // After this amount of tx Packets, send an rx Packet. Max 16
  uint8_t numTxPacketsToRx = 11;

  // ===================== State =====================

  VCTR::network::datalink::RadioI &radioLink;

  // ---- Channel sequence ----
  VCTR::Core::ListArray<uint8_t> channelSequence;
  size_t currentChannelIdx = 0;
  uint8_t key = 0;
  int64_t lastChannelHopTime = 0;

  // ---- Slot state ----
  int64_t currentSlotStart = 0;
  size_t slotCounter = 0;        // Counts from 0 to slotsPerHop
  size_t roleReverseCounter = 0; // Counts from 0 to numTxPacketsToRx
  bool lastSlotWasReceive =
      false; // Whether the previous slot was an RX slot (for LQ recording)
  bool isTransmitSlot = true; // Whether the current slot is a transmit slot
  bool receivedPacket = false;
  int64_t lastPacketRcvTime = 0;

  Core::IntervalTimer slotTimer;

  // ---- Sync state ----
  FHSSState fhssState = FHSSState::Searching;
  int64_t slotRxError = 0;
  Core::ListBuffer<int64_t, 50> syncErrorHistory;

  // ===================== Hop Guard =====================

  void updateHopGuard(int64_t now);

  int64_t hopGuardMargin_ = 2 * Core::MILLISECONDS;
  bool hopGuardActive_ = false;
  Core::ListArray<Core::Scheduler::Task *> hopGuardedTasks_;

public:
  int64_t timingOffset = 0;

  // ---- Packet data ----
  VCTR::network::DataPacket packetToSend;

  // ---- Link quality tracking ----
  VCTR::Core::ListBuffer<bool, 50> receiveSuccesses;
  float linkQuality = 0;
  float otherEndLinkQuality = 0;
};

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_FHSS_HPP