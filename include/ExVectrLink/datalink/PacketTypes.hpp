#ifndef EXVECTRLINK_PACKETTYPES_HPP
#define EXVECTRLINK_PACKETTYPES_HPP

#include <cstdint>

#include "ExVectrNetwork/datalink/sx1280/Sx1280_2.hpp"

namespace VCTR::ExVectrLink::datalink {

enum PacketTypes : uint8_t {
  Data = 0, ///< Payload is raw data from the external serial connection.
  Heartbeat = 1,
  LinkTelemetry =
      2, ///< 4-byte compact link telemetry (antenna, LQ, txPower, RSSI, SNR).
  RxState = 3,    ///< Current rx time, desync count etc.
  UploadMode = 4, ///< OTA upload mode (for firmware updates).
  SlotStats = 5,  ///< FHSS missed-slot (scheduling-latency) counter.

  // --- Test/debug triggers (Lua menu commands, relayed to the far end) ---
  TestBlock = 6, ///< Blocks the receiving side's FHSS task for a fixed
                 ///< duration -- exercises the in-flight-reception grace
                 ///< window / catch-up path under a real scheduling stall.
  TestForceResync =
      7, ///< Forces the receiving side to desync (shifted channel/counters)
         ///< and drop straight to FHSSState::Searching, bypassing the
         ///< multi-second timeout, for repeatable reacquisition testing.
  SetSyncOffset =
      8, ///< Sets the receiving side's FHSS sync latency compensation (see
         ///< FHSS::setSyncLatencyCompensation()). 2-byte little-endian
         ///< payload, microseconds, 0-2000.
  ResetRcGapStat =
      9, ///< Resets the receiving side's max-gap-between-CRSF-RC-frames
         ///< statistic (see getCrsfMaxRcFrameGapUs(); the gap is reported
         ///< back inside the SlotStats packet). No payload.
};

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_PACKETTYPES_HPP