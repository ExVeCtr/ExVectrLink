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
  int64_t wakeupLeadRxNs = 0; ///< wakeup lead before a slot in which this end RECEIVES; 0 = same as wakeupLeadNs. Only
                              ///< needed when the frame is still on air at wakeupLeadNs (v2-trial): the wakeup hops the
                              ///< channel and arms RX, which must not happen while the own TX is still running.
};

/// Today's FPV link: 10 ms slots, 4 slots per hop, 1 RX-side slot in 15, SF7/BW800/LI 4/8, 12 bytes on air.
constexpr AirProfile kFpvProfile{
    10 * fhss::kNsPerMs, 4, 15, 1 * fhss::kNsPerMs, 0, 2, 0,
    fhss::LoRaParams{7, 812500, 7, 12, false, true}, 12};

/// Wakeup lead of the network profiles (before every slot, TX prep happens in it). 2.5 ms by default; a BUILD-time
/// experiment can override it with -DEXVECTRLINK_NET_WAKEUP_LEAD_NS=<ns> (docs/air-link-optimisation.md section 10).
/// Not used by kFpvProfile, so FPV is unaffected.
#ifdef EXVECTRLINK_NET_WAKEUP_LEAD_NS
constexpr int64_t kNetworkWakeupLeadNs = EXVECTRLINK_NET_WAKEUP_LEAD_NS;
#else
constexpr int64_t kNetworkWakeupLeadNs = 2500000LL;
#endif

/// Network mode v1: LoRa SF5, BW1600, CR 4/5, 12-symbol preamble, CRC on, implicit header, fixed 255 bytes on air
/// (10.56 ms), 20 ms slots, 1:1 roles (25 packets/s each way), 2 slots per hop (one uplink and one downlink slot per
/// channel: a 40 ms superframe). Own link version and hop key so FPV and network units never accept each other's
/// frames (they cannot decode each other anyway: different modulation and length). The wakeup lead is 2.5 ms
/// because each own TX slot builds a 253-byte air packet (aggregator + CRC-32) and loads the FIFO before the
/// boundary; the slot budget leaves 9.4 ms of guard.
constexpr AirProfile kNetworkProfileV1{
    20 * fhss::kNsPerMs, 2, 2, kNetworkWakeupLeadNs, 255, 0x11, 0x37,
    fhss::LoRaParams{5, 1625000, 1, 12, false, true}, 255};

/// Network mode "v2-trial" (owner decision 2026-11-26, experiment; design: ExVectrLink docs/air-link-optimisation.md
/// section 8): v1 with LoRa SF6 instead of SF5, everything else identical (BW1600, CR 4/5, 20 ms slots, 2.5 ms wakeup
/// lead, hop key, link version). Frame length reduced from 255 B to 220 B on air (owner decision 2026-11-26) so the
/// frame fits the strict slot budget (16.5 ms, netSlotTiming().fitsStrict). Estimated airtime about 15.5 ms at 220 B
/// (SF6, BW1600, CR 4/5); not measured yet. Single 2.5 ms wakeup lead for TX and RX slots (wakeupLeadRxNs = 0): the
/// 220 B frame ends 2.09 ms before that wakeup. (A 1.0 ms RX lead was tried for the 255 B frame; on the bench it left
/// the TX module too little time to hop and arm RX before the SF6 preamble: downlink 43-54 %.) Selected at BUILD time
/// only (-DEXVECTRLINK_NET_PROFILE_V2_TRIAL).
constexpr AirProfile kNetworkProfileV2Trial{
    20 * fhss::kNsPerMs, 2, 2, kNetworkWakeupLeadNs, 220, 0x11, 0x37,
    fhss::LoRaParams{6, 1625000, 1, 12, false, true}, 220, 0};

/// The profile the network-mode firmwares run. v1 unless the build defines EXVECTRLINK_NET_PROFILE_V2_TRIAL.
#if defined(EXVECTRLINK_NET_PROFILE_V2_TRIAL)
static constexpr const AirProfile &kNetworkProfileActive = kNetworkProfileV2Trial;
static constexpr const char *kNetworkProfileActiveName = "v2-trial";
#else
static constexpr const AirProfile &kNetworkProfileActive = kNetworkProfileV1;
static constexpr const char *kNetworkProfileActiveName = "v1";
#endif

/// Slot timing of a 1:1 network profile, derived from the profile (all in ns, signed: negative = late).
/// Timeline of one slot, t = 0 at the sender's slot boundary: the frame ends at `toaNs`; the receiver's own TX slot
/// starts at `slotIntervalNs` (1:1 roles: the next slot of the receiving end is always its TX slot) and its FHSS task
/// wakes at `slotIntervalNs - wakeupLeadNs`. If the frame is still in flight at that wakeup the grace window in
/// FHSS::taskThread() waits for it, but never past `slotIntervalNs - kGraceTxCutoffNs`.
struct NetSlotTiming {
  int64_t toaNs;               ///< time on air of the fixed frame
  int64_t wakeupSlackNs;       ///< (slot - TX lead) - frame end. Negative: frame still in flight at the wakeup before the
                               ///< own TX slot (grace path)
  int64_t graceMarginNs;       ///< (slot - kGraceTxCutoffNs) - frame end: room before the hard grace cutoff
  int64_t txPrepLeftNs;        ///< slot boundary - frame end: what is left for RX fetch, TX prep and busy-wait
  int64_t strictMarginNs;      ///< (slot - TX lead - kSlotMarginNs) - frame end (the 16.5 ms design budget)
  int64_t graceBudgetMarginNs; ///< (slot - kGraceTxCutoffNs - kSlotMarginNs) - frame end
  int64_t ownTxDoneSlackNs;    ///< (slot - RX lead) - frame end: the own TX has ended when the wakeup before the next
                               ///< (RX) slot hops the channel and arms RX. Negative: that wakeup truncates the own TX.
  bool fitsStrict;             ///< frame ends before the wakeup, with margin: no grace rescue needed (v1)
  bool fitsWithGrace;          ///< needs the grace rescue but is safe: grace budget and own-TX-done both >= margin
};

/// Margin kept for sync error, scheduler jitter and RX_DONE handling (design doc section 1).
constexpr int64_t kSlotMarginNs = 1 * fhss::kNsPerMs;
/// FHSS::taskThread() grace cutoff before the boundary when the next slot is TX (same value as in FHSS.cpp).
constexpr int64_t kGraceTxCutoffNs = 500000LL;

/// Effective wakeup lead before a slot in which this end receives.
inline int64_t wakeupLeadForRx(const AirProfile &p) { return p.wakeupLeadRxNs > 0 ? p.wakeupLeadRxNs : p.wakeupLeadNs; }

inline NetSlotTiming netSlotTiming(const AirProfile &p) {
  NetSlotTiming t{};
  t.toaNs = (int64_t)(fhss::loraTimeOnAirUs(p.lora, p.onAirBytes) * 1000.0f);
  t.wakeupSlackNs = (p.slotIntervalNs - p.wakeupLeadNs) - t.toaNs;
  t.graceMarginNs = (p.slotIntervalNs - kGraceTxCutoffNs) - t.toaNs;
  t.txPrepLeftNs = p.slotIntervalNs - t.toaNs;
  t.strictMarginNs = (p.slotIntervalNs - p.wakeupLeadNs - kSlotMarginNs) - t.toaNs;
  t.graceBudgetMarginNs = (p.slotIntervalNs - kGraceTxCutoffNs - kSlotMarginNs) - t.toaNs;
  t.ownTxDoneSlackNs = (p.slotIntervalNs - wakeupLeadForRx(p)) - t.toaNs;
  t.fitsStrict = t.strictMarginNs >= 0 && t.ownTxDoneSlackNs >= kSlotMarginNs;
  t.fitsWithGrace = t.graceBudgetMarginNs >= 0 && t.ownTxDoneSlackNs >= kSlotMarginNs;
  return t;
}

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
