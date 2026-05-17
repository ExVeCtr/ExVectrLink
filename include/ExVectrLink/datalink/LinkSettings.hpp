#ifndef EXVECTRLINK_LINKSETTINGS_HPP
#define EXVECTRLINK_LINKSETTINGS_HPP

#include <cstdint>

#include "ExVectrNetwork/datalink/sx1280/Sx1280_2.hpp"

namespace VCTR::ExVectrLink::datalink {

enum ModulationPresets : uint8_t {
  Hyperspeed, // Shortest range. 250Hz. Used mainly for dual end throughput.
  Fast,       // Short range, high speed. 250Hz
  Medium,     // Medium range, balanced speed. 150Hz
  LongRange,  // Long range. 50Hz
  MAX,
};

struct ModulationParams {
  VCTR::network::datalink::SX1280_SF spreadingFactor;
  VCTR::network::datalink::SX1280_BW bandwidth;
  VCTR::network::datalink::SX1280_CR codingRate;
  int64_t hopInterval; // Time the radio stays on each channel
};

extern const ModulationParams modulationPresets[ModulationPresets::MAX];

struct LinkInfo {
  // --- Local side (as seen by this node) ---
  int8_t rssi = 0;
  int8_t snr = 0;
  uint8_t antenna = 0;     // Current antenna in use.
  uint8_t lossRate = 0;    // percentage of last 100 packets that were lost.
  uint8_t linkQuality = 0; // percentage of last 100 packets received.
  int8_t txPower = 0;      // TX power in dBm.

  // --- Remote side (received via OTA LinkTelemetry) ---
  int8_t remoteRssi = 0;
  int8_t remoteSnr = 0;
  uint8_t remoteAntenna = 0;
  uint8_t remoteLossRate = 0;
  uint8_t remoteLinkQuality = 0;
  int8_t remoteTxPower = 0;
  bool remoteValid = false;
  int32_t remoteDeviceTime = 0;
  uint16_t remoteDesyncCount = 0;

  bool dualLinkMode = false;
};

/**
 * @brief Get the maximum payload size that can be transmitted while ensuring
 * that the time on air does not exceed the given limit.
 * @param params The modulation parameters to use for the transmission.
 * @param transmitTimeLimit The maximum allowed time on air for the transmission
 * in milliseconds.
 * @return The maximum payload size in bytes that can be transmitted within the
 * given time limit
 */
// uint8_t getPayloadSizeLimitForInterval(const ModulationPresets preset,
//                                        int64_t transmitTimeLimit);

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_LINKSETTINGS_HPP