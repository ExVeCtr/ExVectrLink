#ifndef EXVECTRLINK_FHSS_HPP
#define EXVECTRLINK_FHSS_HPP

#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/DatalinkI.hpp"
#include "ExVectrNetwork/physical/HasChannels.hpp"

namespace VCTR::ExVectrLink::datalink {

/**
 * The FHSS class takes care of switching channels for a given datalink. It only
 * works with a single datalink but multiple connected the same physical layer
 * with auto synchronise and track.
 */

class FHSS {
public:
  FHSS();

  void receiveDataframe(const VCTR::network::DataPacket &dataframe);

private:
};

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_FHSS_HPP