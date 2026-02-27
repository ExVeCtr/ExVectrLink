#ifndef EXVECTRLINK_FHSS_HPP
#define EXVECTRLINK_FHSS_HPP

#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/RadioI.hpp"

namespace VCTR::ExVectrLink::datalink {

/**
 * The FHSS class takes care of switching channels for a given datalink.
 * This does not limit the max packet size. Switching is only done once a packet
 * is fully received or transmitted.
 */
class FHSS : public VCTR::network::datalink::DatalinkI,
             public VCTR::Core::Task_Periodic {
public:
  FHSS(VCTR::network::datalink::RadioI &radioI);

  void setFhssKey(uint8_t key);
  uint8_t getFhssKey() const;

  void enableFhss(bool enable);

  //--- DatalinkI interface implementation ---

  bool transmitDataframe(const VCTR::network::DataPacket &dataframe) override;

  /**
   * @brief Get the maximum packet size that can be transmitted by the datalink.
   * @note packets over this size will be dropped and not transmitted.
   * @return size_t The maximum packet size in bytes.
   */
  size_t getMaxPacketSize() const override;

  /**
   * @returns true if the datalink is currently blocked and cannot send
   * dataframes.
   */
  bool isChannelBlocked() const override;

  //--- Task_Periodic interface implementation ---

  void taskCheck() override;
  void taskInit() override;
  void taskThread() override;

private:
  void generateSequence();

  VCTR::network::datalink::RadioI &radioLink;

  VCTR::Core::ListArray<uint8_t> channelSequence;
  uint8_t key = 0;

  VCTR::network::DataPacket packetToSend;
  bool waitingForSendFinish = false;

  bool fhssEnabled = false;
};

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_FHSS_HPP