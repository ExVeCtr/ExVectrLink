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

// ---------------------------------------------------------------------------
// Type byte encoding
//
// The type value travels in the low 7 bits of the trailing type byte, with
// bit 7 carrying odd parity over the whole byte. 127 types is plenty, and the
// spare bit buys detection of every single-bit error in the one byte that
// decides what a frame *is* -- without it a single flipped bit silently turns
// one type into another (Data = 0 and UploadMode = 4 are one bit apart).
//
// Odd rather than even parity, so that an all-zero byte -- what an erased or
// stuck-low buffer reads as -- is never a valid type.
//
// Everything that puts a type byte on the wire must go through
// encodePacketType(), and everything that reads one must check
// packetTypeIsValid() before decodePacketType().
// ---------------------------------------------------------------------------

/// Mask of the bits carrying the type itself. Every PacketTypes value must
/// fit inside this.
constexpr uint8_t kPacketTypeMask = 0x7F;
/// Mask of the parity bit.
constexpr uint8_t kPacketTypeParityMask = 0x80;

static_assert(PacketTypes::ResetRcGapStat <= kPacketTypeMask,
              "PacketTypes values must fit in 7 bits -- see kPacketTypeMask.");

/// @returns true if `value` has an odd number of set bits.
constexpr bool hasOddParity(uint8_t value) {
  value ^= static_cast<uint8_t>(value >> 4);
  value ^= static_cast<uint8_t>(value >> 2);
  value ^= static_cast<uint8_t>(value >> 1);
  return (value & 0x01) != 0;
}

/// @brief Encodes `type` into the byte that goes on the wire: the type in the
/// low 7 bits, plus whichever parity bit makes the byte odd-parity.
constexpr uint8_t encodePacketType(PacketTypes type) {
  const uint8_t bits = static_cast<uint8_t>(type) & kPacketTypeMask;
  return hasOddParity(bits) ? bits
                            : static_cast<uint8_t>(bits | kPacketTypeParityMask);
}

/// @returns true if `encoded`'s parity bit agrees with the type it carries,
/// i.e. no single bit of the type byte has been corrupted. Always check this
/// before acting on decodePacketType().
constexpr bool packetTypeIsValid(uint8_t encoded) { return hasOddParity(encoded); }

/// @returns the type value carried by an encoded type byte. Only meaningful
/// once packetTypeIsValid() has confirmed it.
constexpr uint8_t decodePacketType(uint8_t encoded) {
  return encoded & kPacketTypeMask;
}

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_PACKETTYPES_HPP