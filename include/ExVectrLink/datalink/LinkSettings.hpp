#ifndef EXVECTRLINK_LINKSETTINGS_HPP
#define EXVECTRLINK_LINKSETTINGS_HPP

#include <cstdint>

#include "ExVectrNetwork/datalink/sx1280/Sx1280.hpp"

namespace VCTR::ExVectrLink::datalink {

enum class ModulationPresets : uint8_t {
  Hyperspeed, // Shortest range. Optimized for highest throughput.
  Fast,       // Short range, high speed.
  Medium,     // Medium range, balanced speed.
  LongRange,  // Long range, low speed.
  MAX,
};

struct ModulationParams {
  VCTR::network::datalink::SX1280_SF spreadingFactor;
  VCTR::network::datalink::SX1280_BW bandwidth;
  VCTR::network::datalink::SX1280_CR codingRate;
};

const ModulationParams modulationPresets[] = {
    {VCTR::network::datalink::SX1280_SF::SF_5,
     VCTR::network::datalink::SX1280_BW::BW_1600KHz,
     VCTR::network::datalink::SX1280_CR::LI_4_5}, // Hyperspeed
    {VCTR::network::datalink::SX1280_SF::SF_6,
     VCTR::network::datalink::SX1280_BW::BW_800KHz,
     VCTR::network::datalink::SX1280_CR::LI_4_8}, // Fast
    {VCTR::network::datalink::SX1280_SF::SF_7,
     VCTR::network::datalink::SX1280_BW::BW_800KHz,
     VCTR::network::datalink::SX1280_CR::LI_4_8}, // Medium
    {VCTR::network::datalink::SX1280_SF::SF_8,
     VCTR::network::datalink::SX1280_BW::BW_800KHz,
     VCTR::network::datalink::SX1280_CR::LI_4_8}, // LongRange
};

struct LinkInfo {
  int8_t rssi;
  int8_t snr;
  uint8_t antenna; // Current antenna in use.

  uint8_t lossRate; // percentage of packets lost. 0 good, 100 all.

  bool dualLinkMode;
};

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_LINKSETTINGS_HPP