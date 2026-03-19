#ifndef EXVECTRLINK_FHSS_HPP
#define EXVECTRLINK_FHSS_HPP

#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/RadioI.hpp"

namespace VCTR::ExVectrLink::datalink {

enum class FHSSState : uint8_t {
  Searching,
  Synced,
};

/**
 * The FHSS class takes care of switching channels for a given datalink.
 * This does not limit the max packet size. Switching is only done once a packet
 * is fully received or transmitted.
 */
class FHSS : public VCTR::network::datalink::DatalinkI,
             public VCTR::Core::Task_Periodic {

  struct ChannelSetting {
    uint8_t channel;
    bool isReceiveChannel;
  };

public:
  FHSS(VCTR::network::datalink::RadioI &radioI);

  void setFhssKey(uint8_t key);
  uint8_t getFhssKey() const;

  FHSSState getFhssState() const;

  void setReceiveStartTime(int64_t time);

  void setNumReceiveChannels(size_t num);
  void setHoppingInterval(int64_t interval);
  void setHoppingSyncInterval(int64_t offset);

  void updateReceiveStartTime(int64_t time);

  void setIsRxSide(bool isRxSide);

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
  void sendFHSSPacket();
  void updateChannel();

  void updateTiming();

  void updateSearch();
  void updateHopping();

  VCTR::network::datalink::RadioI &radioLink;

  VCTR::Core::ListArray<ChannelSetting> channelSequence;
  uint8_t currentSeqIndex = 0;
  uint8_t key = 0;
  uint8_t numReceiveChannels = 0;
  int64_t hoppingInterval = 20 * Core::MILLISECONDS;

  VCTR::network::DataPacket packetToSend;

  int64_t lastPacketReceiveStartTime = 0;

  int64_t lastFHSSPacketSentTime = 0;
  int64_t lastPacketReceivedTime = 0;
  int64_t packetReceiveStartTime = 0;
  int64_t lastHoppingTime = 0;
  int64_t hoppingOffset = 0;

  bool channelReady = false;
  bool isRxSide = true;

  FHSSState fhssState = FHSSState::Searching;
};

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_FHSS_HPP