#ifndef EXVECTRLINK_AIRPROFILE_HPP
#define EXVECTRLINK_AIRPROFILE_HPP
// FHSS runtime parameter sets ("air profiles") and the TX payload hook. Pure: no Arduino, no ExVectrCore.
//
// kFpvProfile holds today's FPV values. They are also the FHSS member defaults, and the FPV firmware never applies a
// profile, so FPV is unchanged. kNetworkProfileV1 is the ExVectrNetwork-mode air link (workspace
// docs/network-design.md, Revision 1, "Air profile v1"). The radio modulation itself is set by the application on
// the Sx1280_Direct objects (FHSS does not own it); `lora` here documents it and feeds the slot budget check.
#include <stddef.h>
#include <stdint.h>

#include "ExVectrLink/datalink/FhssMath.hpp"

namespace VCTR::ExVectrLink::datalink {

struct AirProfile {
  int64_t slotIntervalNs;   ///< one TX or RX operation
  uint8_t slotsPerHop;      ///< 1..4 (2-bit trailer field)
  uint8_t rolePeriod;       ///< FHSS numTxPacketsToRx: 1 slot of each period goes RX side -> TX side. 2..64
  int64_t wakeupLeadNs;     ///< the FHSS task wakes this long before each slot boundary (TX prep happens in it)
  uint8_t fixedFrameLength; ///< 0 = variable payload up to 9 bytes (FPV); else every frame is padded to this length
                            ///< on air (payload + 2-byte trailer)
  uint8_t linkVersion;      ///< seeds the trailer checksum (FPV: FHSS::OTA_VERSION = 2)
  uint8_t hopKey;           ///< hop sequence seed and checksum seed
  fhss::LoRaParams lora;    ///< modulation the application configures on the radios
  uint8_t onAirBytes;       ///< bytes on air per packet (Limited mode adds the 1-byte length prefix)
};

/// Today's FPV link: 10 ms slots, 4 slots per hop, 1 RX-side slot in 15, SF7/BW800/LI 4/8, 12 bytes on air.
constexpr AirProfile kFpvProfile{
    10 * fhss::kNsPerMs, 4, 15, 1 * fhss::kNsPerMs, 0, 2, 0,
    fhss::LoRaParams{7, 812500, 7, 12, false, true}, 12};

/// Network mode v1: LoRa SF5, BW1600, CR 4/5, 12-symbol preamble, CRC on, implicit header, fixed 255 bytes on air
/// (10.56 ms), 20 ms slots, 1:1 roles (25 packets/s each way), 2 slots per hop (one uplink and one downlink slot per
/// channel: a 40 ms superframe). Own link version and hop key so FPV and network units never accept each other's
/// frames (they cannot decode each other anyway: different modulation and length). The wakeup lead is 2.5 ms
/// because each own TX slot builds a 253-byte air packet (aggregator + CRC-32) and loads the FIFO before the
/// boundary; the slot budget leaves 9.4 ms of guard.
constexpr AirProfile kNetworkProfileV1{
    20 * fhss::kNsPerMs, 2, 2, 2500000LL, 255, 0x11, 0x37,
    fhss::LoRaParams{5, 1625000, 1, 12, false, true}, 255};

/// Pull hook for the payload of an own TX slot. Called by FHSS from its slot task, inside the wakeup lead, right
/// before the frame is loaded into the radio. Fill at most `maxLen` bytes and return the count (0 = nothing; FHSS
/// then sends a keep-alive: the trailer only, or zero padding plus trailer with a fixed frame length).
class FhssTxProviderI {
public:
  virtual ~FhssTxProviderI() = default;
  virtual size_t fillTxPayload(uint8_t *buf, size_t maxLen, int64_t slotStartNs) = 0;
};

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_AIRPROFILE_HPP
