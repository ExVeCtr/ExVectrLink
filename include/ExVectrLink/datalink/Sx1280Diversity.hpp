#ifndef EXVECTRLINK_SX1280DIVERSITY_HPP
#define EXVECTRLINK_SX1280DIVERSITY_HPP

#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/RadioI.hpp"
#include "ExVectrNetwork/datalink/sx1280/Sx1280_2.hpp"

namespace VCTR::ExVectrLink::datalink {

class Sx1280Diversity : public VCTR::network::datalink::RadioI,
                        public VCTR::Core::Task_Periodic {
private:
  struct Sx1280PacketRfInfo {
    int16_t rssi;
    int16_t snr;
    int64_t receivedTime = 0;
    VCTR::network::DataPacket packet;
  };
  struct Sx1280LinkInfo {
    VCTR::network::datalink::Datalink_SX1280_V2 *link;
    Sx1280PacketRfInfo lastPacketInfo;
  };

public:
  Sx1280Diversity() = default;
  Sx1280Diversity(
      std::initializer_list<VCTR::network::datalink::Datalink_SX1280_V2 *>
          links);

  void addDiversityLink(VCTR::network::datalink::Datalink_SX1280_V2 &link);

  const VCTR::network::datalink::Datalink_SX1280_V2 *
  getDiversityLink(size_t index) const;

  size_t getCurrentBestLinkIndex() const;

  /**
   * @brief Use only the given link for tx. Rx will uise any.
   * @return true if the lgiven link is in the diversity and has been set.
   */
  bool
  setDesignatedTxLink(const VCTR::network::datalink::Datalink_SX1280_V2 &link);

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

  size_t getNumChannels() const override;
  size_t getCurrentChannel() const override;
  void setChannel(size_t channel) override;

  int16_t lastPacketSNR() const override;

  void setStartReceive(bool rxEnabled) override;
  void setEnableTxRx(bool enable) override;
  void setEnableAutoRx(bool enableAutoRx) override;

  void taskInit() override;
  void taskThread() override;

private:
  void startReceiveOnAllLinks();
  void stopReceiveOnAllLinks(size_t exceptIndex = -1);
  void determineBestLink();

  void processReceivedPackets();

  size_t getTxLinkIndex() const;

  Core::ListArray<Sx1280LinkInfo> diversityLinks;

  int64_t lastPacketReceivedTime = 0;

  size_t designatedTxLink = -1;

  size_t currentBestLinkIndex = 0;
  uint8_t currentBestLinkLq = 0;

  int64_t transmitting = 0;
  bool receiving = false;
};

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_SX1280DIVERSITY_HPP