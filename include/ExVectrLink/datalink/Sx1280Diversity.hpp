#ifndef EXVECTRLINK_SX1280DIVERSITY_HPP
#define EXVECTRLINK_SX1280DIVERSITY_HPP

#include "ExVectrCore/list_buffer.hpp"
#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/sx1280/Sx1280_Direct.hpp"

#include <initializer_list>
#include <limits>

namespace VCTR::ExVectrLink::datalink {

class Sx1280Diversity : public VCTR::network::datalink::Sx1280_DirectI {
private:
  struct Sx1280LinkInfo {
    VCTR::network::datalink::Sx1280_DirectI *link = nullptr;
    uint32_t lastSeenRxPacketCount = 0;
    int16_t lastPacketRssi = 0;
    int16_t lastPacketSnr = std::numeric_limits<int16_t>::min();
  };

  static constexpr size_t kMaxDiversityLinks = 2;
  static constexpr size_t kNoLink = static_cast<size_t>(-1);
  static constexpr size_t kSnrMedianWindow = 10;

public:
  Sx1280Diversity() = default;
  Sx1280Diversity(
      std::initializer_list<VCTR::network::datalink::Sx1280_DirectI *> links);

  void addDiversityLink(VCTR::network::datalink::Sx1280_DirectI &link);

  const VCTR::network::datalink::Sx1280_DirectI *
  getDiversityLink(size_t index) const;

  size_t getCurrentBestLinkIndex() const;
  size_t getCurrentTxLinkIndex() const;

  /**
   * @brief Use only the given link for tx. Rx will uise any.
   * @return true if the lgiven link is in the diversity and has been set.
   */
  bool setDesignatedTxLink(const VCTR::network::datalink::Sx1280_DirectI &link);

  bool configureRadio() override;

  void startRx(int64_t timeout) override;
  int16_t getPacketRSSI() const override;
  int16_t getPacketSNR() const override;
  VCTR::network::DataPacket getRxPacket() const override;
  uint32_t getRxPacketCount() const override;

  bool setupTxPacket(const VCTR::network::DataPacket &packet) override;
  void startTx() override;

  size_t getNumChannels() const override;
  size_t getCurrentChannel() const override;
  void setChannel(size_t channel) override;

  void setFrequency(uint32_t newFreqHz) override;
  void setSpreadingFactor(VCTR::network::datalink::SX1280_SF sf) override;
  void setBandwidth(VCTR::network::datalink::SX1280_BW bw) override;
  void setCodingRate(VCTR::network::datalink::SX1280_CR cr) override;
  void setTxPower(int8_t power) override;
  uint8_t getTxPower() const override;
  void setTxMaxPower(int8_t maxTxPower) override;
  void setPacketMode(VCTR::network::datalink::SX1280_PacketMode mode) override;
  void setFixedPacketLength(uint8_t length) override;
  void setPAdbm(uint8_t paDbm) override;
  void setAutoFS(bool enable) override;

  void setIdle() override;

  void push(bool keepOscRunning = false) override;
  void pull() override;

private:
  void refreshBestLink();

  size_t getTxLinkIndex() const;

  Sx1280LinkInfo diversityLinks[kMaxDiversityLinks] = {};
  size_t diversityLinkCount = 0;
  size_t designatedTxLink = kNoLink;
  size_t currentBestLinkIndex = 0;
  size_t pendingTxLinkIndex = 0;
  size_t activeTxLinkIndex = kNoLink;

  int16_t lastDeliveredPacketRssi = 0;
  int16_t lastDeliveredPacketSnr = 0;
  VCTR::Core::ListBuffer<int16_t, kSnrMedianWindow> snrHistory;
  bool currentCycleHasPacket = false;
  int16_t currentCycleRssi = 0;
  int16_t currentCycleSnr = std::numeric_limits<int16_t>::min();
  VCTR::network::DataPacket lastRxPacket;
  uint32_t rxPacketCount = 0;
  bool rxPacketLatched = false;
  bool txInProgress = false;
};

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_SX1280DIVERSITY_HPP