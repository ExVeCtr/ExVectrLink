#ifndef EXVECTRLINK_SX1280DIVERSITY_HPP
#define EXVECTRLINK_SX1280DIVERSITY_HPP

#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/RadioI.hpp"
#include "ExVectrNetwork/datalink/sx1280/Sx1280.hpp"

namespace VCTR::ExVectrLink::datalink {

class Sx1280Diversity : public VCTR::network::datalink::RadioI {
private:
  struct Sx1280PacketRfInfo {
    int16_t rssi;
    int16_t snr;
  };
  struct Sx1280LinkInfo {
    VCTR::network::datalink::Datalink_SX1280 *link;
    Sx1280PacketRfInfo lastPacketInfo;
  };
  struct ReceivedDataframeInfo {
    VCTR::network::DataPacket dataframe;
    size_t linkIndex;
    Sx1280PacketRfInfo rfInfo;
  };

public:
  Sx1280Diversity() = default;
  Sx1280Diversity(
      std::initializer_list<VCTR::network::datalink::Datalink_SX1280 *> links);

  void addDiversityLink(VCTR::network::datalink::Datalink_SX1280 &link);

  const VCTR::network::datalink::Datalink_SX1280 *
  getDiversityLink(size_t index) const;

  size_t getCurrentBestLinkIndex() const;

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

private:
  void startReceiveOnAllLinks();
  void stopReceiveOnAllLinks();

  void handleReceivedDataframes();

  uint8_t calcLinkQuality(const Sx1280PacketRfInfo &linkInfo) const;

  Core::ListArray<Sx1280LinkInfo> diversityLinks;

  size_t currentBestLinkIndex = 0;
  uint8_t currentBestLinkLq = 0;

  bool transmitting = false;
  bool receiving = false;
};

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_SX1280DIVERSITY_HPP