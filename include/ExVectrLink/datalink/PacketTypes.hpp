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
};

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_PACKETTYPES_HPP