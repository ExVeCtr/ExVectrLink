#ifndef EXVECTRLINK_PACKETTYPES_HPP
#define EXVECTRLINK_PACKETTYPES_HPP

#include <cstdint>

#include "ExVectrNetwork/datalink/sx1280/Sx1280_2.hpp"

namespace VCTR::ExVectrLink::datalink {

enum PacketTypes : uint8_t {
  Data,
  Heartbeat,
};

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_PACKETTYPES_HPP