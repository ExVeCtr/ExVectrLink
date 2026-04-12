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
  Syncing,   ///< Detected a signal and is adjusting timing to sync.
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
  float getSnr() const;

  /// @brief Returns the current timing offset correction in nanoseconds.
  int64_t getTimingOffset() const;

  /// @brief Returns the actual slot interval including corrections (ns).
  int64_t getTrueSlotInterval() const;

  /// @brief Returns the current slot counter within the hop cycle.
  uint8_t getSlotCounter() const;

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
  void hopChannel(bool reverse = false);

  void transmitPacket(network::DataPacket &packet);
  void receivePacket(const network::DataPacket &packet);

  void updateLinkQuality();

  void timingControl();

  int64_t getAdjustedSlotInterval() const;

  // ======================= Configuration =======================

  int64_t slotInterval = 10 * Core::MILLISECONDS;
  int64_t trueSlotInterval = slotInterval;
  size_t slotsPerHop = 4;

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

  // ----- Timing -----
  int64_t threadStart = 0;

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

  // ---- Sync state ----
  FHSSState fhssState = FHSSState::Searching;
  int64_t lastSearchHopTime = 0;
  int64_t syncedStartTime = 0;

  int64_t lastTxPrint = 0;

  // ---- Packet data ----
  VCTR::network::DataPacket packetToSend;

  // ---- Link quality tracking ----
  VCTR::Core::ListBuffer<bool, 100> receiveSuccesses;
  float linkQuality = 0;
  float otherEndLinkQuality = 0;
  float otherEndSnr = 0;
};

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_FHSS_HPP