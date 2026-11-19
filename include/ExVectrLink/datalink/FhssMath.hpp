#ifndef EXVECTRLINK_FHSSMATH_HPP
#define EXVECTRLINK_FHSSMATH_HPP
// Pure FHSS schedule, trailer, sync-filter and time-on-air maths. No Arduino, no ExVectrCore, no heap, so it is
// host-testable (ExVectrLink parent: pio test -e native). FHSS.cpp uses these functions; every expression keeps the
// exact types and order of the original inline code so the FPV behaviour is bit-for-bit unchanged (checked by the
// golden-trace test in the parent's test/test_fhss).
#include <math.h>
#include <stddef.h>
#include <stdint.h>

namespace VCTR::ExVectrLink::datalink::fhss {

constexpr int64_t kNsPerMs = 1000000LL;

// ------------------------------------------------------------------ slot roles

/// Role of a slot. The role counter counts 0 .. rolePeriod-1 (FHSS numTxPacketsToRx). The TX side (sync master)
/// transmits whenever it is not 0, the RX side (follower) only when it is 0. rolePeriod 2 gives 1:1.
constexpr bool slotIsTx(bool isRxSide, size_t roleCounter) {
  return isRxSide ? (roleCounter == 0) : (roleCounter != 0);
}

/// Receive slots per role period for one side: TX side 1, RX side rolePeriod - 1.
constexpr size_t rxSlotsPerPeriod(bool isRxSide, size_t rolePeriod) {
  return isRxSide ? (rolePeriod > 0 ? rolePeriod - 1 : 0) : 1;
}

/// Packets per second one side expects to receive when every packet arrives.
constexpr uint32_t expectedRxPacketsPerSecond(bool isRxSide, size_t rolePeriod, int64_t slotIntervalNs) {
  return (rolePeriod == 0 || slotIntervalNs <= 0)
             ? 0
             : (uint32_t)((1000000000LL * (int64_t)rxSlotsPerPeriod(isRxSide, rolePeriod)) /
                          (slotIntervalNs * (int64_t)rolePeriod));
}

// ------------------------------------------------------------------ trailer (2 bytes after the payload)

/// Trailer byte 1: bits 7..2 role counter (6 bit), bits 1..0 slot counter (2 bit). Both are reduced modulo their
/// period first, exactly as FHSS always did.
constexpr uint8_t trailerCounterByte(size_t roleCounter, size_t rolePeriod, size_t slotCounter, size_t slotsPerHop) {
  return (uint8_t)(((uint8_t)((roleCounter % rolePeriod) & 0x3F) << 2) |
                   (uint8_t)((slotCounter % slotsPerHop) & 0x03));
}
constexpr uint8_t trailerSlotCounter(uint8_t byte1) { return byte1 & 0x03; }
constexpr uint8_t trailerRoleCounter(uint8_t byte1) { return (byte1 >> 2) & 0x3F; }

/// Largest role period and slots per hop the trailer can carry.
constexpr size_t kMaxRolePeriod = 64;
constexpr size_t kMaxSlotsPerHop = 4;

/// Checksum seed: key + link version + 0x40 if the SENDER is the RX side. Kept in 8 bits (the original int sum is
/// below 256 for every FPV value, so FPV is unchanged; larger network values would otherwise never verify).
constexpr uint8_t trailerSeed(uint8_t key, uint8_t linkVersion, bool senderIsRxSide) {
  return (uint8_t)(key + linkVersion + (senderIsRxSide ? 0x40 : 0x00));
}

/// XOR checksum over `n` bytes, starting at the seed.
inline uint8_t trailerChecksum(uint8_t seed, const uint8_t *data, size_t n) {
  uint8_t crc = seed;
  for (size_t i = 0; i < n; i++) crc ^= data[i];
  return crc;
}

// ------------------------------------------------------------------ sync maths (RX side)

/// Wraps a phase error into [-interval/2, +interval/2].
inline int64_t wrapPhaseError(int64_t error, int64_t interval) {
  if (interval > 0) {
    error = error % interval;
    if (error > interval / 2) error -= interval;
    if (error < -interval / 2) error += interval;
  }
  return error;
}

/// Syncing state: the grid offset moves 0.2 of the error per packet.
inline void syncingStep(int64_t &slotOffsetTime, int64_t slotStartError) { slotOffsetTime += slotStartError * 0.2; }

/// Synced state: phase filter 0.98/0.02, offset correction 0.01, leaky-free drift integrator (gain 1e-5) clamped to
/// +-1 % of the slot interval.
inline void syncedStep(int64_t &slotTimingOffset, int64_t &slotOffsetTime, float &intervalCorrection,
                       int64_t slotStartError, int64_t slotInterval) {
  slotTimingOffset = slotTimingOffset * 0.98 + slotStartError * 0.02;
  slotOffsetTime += slotStartError * 0.01;
  constexpr float kIntervalGain = 0.00001f;
  constexpr float kIntervalMaxPct = 0.01f;
  intervalCorrection += (float)slotStartError * kIntervalGain;
  float maxCorr = (float)slotInterval * kIntervalMaxPct;
  if (intervalCorrection > maxCorr) intervalCorrection = maxCorr;
  if (intervalCorrection < -maxCorr) intervalCorrection = -maxCorr;
}

/// Syncing -> Synced: link quality above max(1 - 1/slotsPerHop, 0.5) (both strict).
inline bool syncLinkQualityOk(float linkQuality, size_t slotsPerHop) {
  auto lqThres = 1.0f - 1.0f / (float)slotsPerHop;
  return linkQuality > lqThres && linkQuality > 0.5f;
}

// ------------------------------------------------------------------ LoRa time on air (SX1280 datasheet 7.4.4)

struct LoRaParams {
  uint8_t sf = 7;          ///< 5..12
  uint32_t bwHz = 812500;  ///< real bandwidth: 203125, 406250, 812500, 1625000
  uint8_t cr = 7;          ///< SX1280 register value: 1..4 = 4/5..4/8, 5 = LI 4/5, 6 = LI 4/6, 7 = LI 4/8
  uint16_t preamble = 12;  ///< symbols
  bool explicitHeader = false;
  bool crcOn = true;
};

/// Symbol count, identical to SX128XLT::calcLoRaSymbolCount() (the driver subtracts the same time on air from the
/// RX_DONE timestamp for sync, so the two must agree).
inline float loraSymbolCount(const LoRaParams &p, uint8_t payloadBytes) {
  const int32_t PL = (int32_t)payloadBytes;
  const int32_t SF = (int32_t)p.sf;
  const int32_t CRC = p.crcOn ? 16 : 0;
  const float pre = (float)p.preamble;
  if (p.cr <= 4) {
    const int32_t hdr = p.explicitHeader ? 20 : 0;
    int32_t num;
    int32_t den;
    float overhead;
    if (SF < 7) {
      num = 8 * PL + CRC - 4 * SF + hdr;
      den = 4 * SF;
      overhead = 6.25f;
    } else if (SF <= 10) {
      num = 8 * PL + CRC - 4 * SF + 8 + hdr;
      den = 4 * SF;
      overhead = 4.25f;
    } else {
      num = 8 * PL + CRC - 4 * SF + 8 + hdr;
      den = 4 * (SF - 2);
      overhead = 4.25f;
    }
    const float pay = (num > 0) ? ceilf((float)num / (float)den) * (float)(p.cr + 4) : 0.0f;
    return pre + overhead + 8.0f + pay;
  }
  // Long interleaving
  const int32_t crLI = (p.cr == 5) ? 5 : (p.cr == 6) ? 6 : (p.cr == 7) ? 8 : 6;
  // Explicit header with long interleaving is not used by any ExVectrLink profile: approximated as implicit with
  // 20 extra header bits (an upper bound, good enough for slot budgeting; FHSS sync never uses this function).
  const int32_t bitPayload = 8 * PL + CRC + (p.explicitHeader ? 20 : 0);
  if (SF < 7) {
    const float nb = ceilf((float)bitPayload / (float)(4 * SF) * (float)crLI);
    if (nb < 8.0f) return pre + 6.25f + nb;
    return pre + 6.25f + 8.0f + ceilf((float)bitPayload / (float)(4 * SF) * (float)crLI - 8.0f);
  }
  if (SF <= 10) {
    const float nb = ceilf((float)bitPayload / (float)(4 * (SF - 2)) * (float)crLI);
    if (nb < 8.0f) return pre + 4.25f + nb;
    return pre + 4.25f + 8.0f + ceilf(((float)bitPayload * (float)crLI + 64.0f) / (float)(4 * SF) - 8.0f);
  }
  const float nb = ceilf((float)bitPayload / (float)(4 * (SF - 2)) * (float)crLI);
  if (nb < 8.0f) return pre + 4.25f + nb;
  return pre + 4.25f + 8.0f + ceilf((float)bitPayload / (float)(4 * (SF - 2)) * (float)crLI - 8.0f);
}

/// Time on air in microseconds for `payloadBytes` bytes on air.
inline float loraTimeOnAirUs(const LoRaParams &p, uint8_t payloadBytes) {
  if (p.bwHz == 0) return 0.0f;
  const float symbolUs = (float)(1UL << p.sf) / (float)p.bwHz * 1e6f;
  return symbolUs * loraSymbolCount(p, payloadBytes);
}

/// True if a packet of `onAirBytes` fits a slot with `guardNs` left for turnaround, scheduling jitter and sync error.
inline bool slotFits(const LoRaParams &p, uint8_t onAirBytes, int64_t slotIntervalNs, int64_t guardNs) {
  return (int64_t)(loraTimeOnAirUs(p, onAirBytes) * 1000.0f) + guardNs <= slotIntervalNs;
}

} // namespace VCTR::ExVectrLink::datalink::fhss

#endif // EXVECTRLINK_FHSSMATH_HPP
