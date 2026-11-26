#ifndef EXVECTRLINK_SLOTDIAG_HPP
#define EXVECTRLINK_SLOTDIAG_HPP

#include <stdint.h>

namespace VCTR::ExVectrLink::datalink {

/// @brief Slot-path diagnostic counters of the FHSS task (docs/air-link-optimisation.md, "AIRDIAG").
///
/// Plain integers, incremented inside the slot path with nothing but additions and comparisons (no logging, no
/// float, no heap, no SPI). Cumulative since boot, except the four `*Max*` fields which the reader resets once per
/// reporting window with FHSS::resetSlotDiagWindow(). Read and reset from the same cooperative loop that runs the
/// FHSS task, so no locking is needed. Pure data, so host code can include it without Arduino.
struct SlotDiag {
  // ---- own TX slots
  uint32_t txSlots = 0;         ///< slots whose role is "this end transmits" (scheduled)
  uint32_t txStarted = 0;       ///< startTx() issued (setupTxPacket accepted the frame)
  uint32_t txSkipNotSynced = 0; ///< own TX slot passed without TX because the RX side is not Synced yet
  uint32_t txSkipSetup = 0;     ///< setupTxPacket() refused the frame (empty / wrong length / no radio)
  uint32_t txLateStart = 0;     ///< TX prep ended AFTER the slot boundary (TX left late, not skipped)
  uint32_t txMissed = 0;        ///< own TX slots inside a catch-up fast-forward (never serviced; last 100 slots of each)
  // ---- RX slots
  uint32_t rxSlots = 0;           ///< completed slots whose role was "this end receives"
  uint32_t rxValid = 0;           ///< of those, with a valid packet (trailer ok, counters agree)
  uint32_t rxNone = 0;            ///< of those, without one (nothing arrived / timeout / rejected)
  uint32_t rxRadioPkts = 0;       ///< packets the radio completed (CRC ok at the radio), before the trailer check
  uint32_t rxTrailerBad = 0;      ///< radio-valid packets whose FHSS trailer checksum failed (other link / key)
  uint32_t rxCounterMismatch = 0; ///< trailer ok but role counter disagreed with ours (rejected)
  // ---- scheduling
  uint32_t wakeups = 0;      ///< slot task runs
  uint32_t wakeLate1ms = 0;  ///< wakeups later than 1 ms after the intended time
  uint32_t missEvents = 0;   ///< catch-up fast-forwards (the task woke more than one slot late)
  // ---- timing sums and maxima (ns)
  int64_t prepSumNs = 0;       ///< sum over started TX of (wakeup -> TX prep finished, before the boundary busy-wait)
  int64_t prepMaxNs = 0;       ///< window max of the same
  int64_t wakeLateMaxNs = 0;   ///< window max of (actual wakeup - intended wakeup)
  int64_t startLateMaxNs = 0;  ///< window max of (actual startTx time - slot boundary), >0 = TX left late
  // ---- own TX frame, startTx() -> TX done first OBSERVED (docs/air-link-optimisation.md, "air="). TX done is only
  // seen by pull() at the next slot wakeup, so this is an upper bound with slot resolution, not the airtime itself.
  uint32_t airCount = 0;       ///< frames whose TX done was observed (cumulative)
  uint32_t airLate = 0;        ///< frames whose TX done came after one slot interval, or never (cumulative)
  int64_t airSumNs = 0;        ///< sum of (observation time - startTx time) over airCount frames
  int64_t airMinNs = 0;        ///< window min of the same (0 = no sample in the window)
  int64_t airMaxNs = 0;        ///< window max of the same
};

} // namespace VCTR::ExVectrLink::datalink

#endif
