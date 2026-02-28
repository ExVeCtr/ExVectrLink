#ifndef EXVECTRLINK_EXVECTRLINKI_HPP
#define EXVECTRLINK_EXVECTRLINKI_HPP

#include <cstdint>

#include "ExVectrNetwork/datalink/DatalinkI.hpp"

#include "ExVectrLink/datalink/LinkSettings.hpp"

namespace VCTR::ExVectrLink /* ExVectrLinkI */ {

class ExVectrLinkI : public VCTR::network::datalink::DatalinkI {
public:
  // Power in dBm.
  virtual void setTxPower(uint8_t txPower) = 0;

  virtual void setModulationPreset(
      VCTR::ExVectrLink::datalink::ModulationPresets preset) = 0;

  virtual void setEnableFhss(bool enable, uint32_t seqKey = 0) = 0;

  // Channel index 0-9. Stops FHSS if enabled.
  virtual void setLinkChannel(uint8_t channelIndex) = 0;

  virtual void setMediaAccessKey(uint8_t mak) = 0;

  virtual const VCTR::ExVectrLink::datalink::LinkInfo &getLinkInfo() const = 0;
};

} // namespace VCTR::ExVectrLink

#endif // EXVECTRLINK_EXVECTRLINKI_HPP