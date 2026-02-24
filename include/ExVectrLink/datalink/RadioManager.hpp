#ifndef EXVECTRLINK_RADIOMANAGER_HPP
#define EXVECTRLINK_RADIOMANAGER_HPP

#include "ExVectrCore/list_array.hpp"

#include "ExVectrNetwork/network/NetworkNode.hpp"

#include "ExVectrNetwork/datalink/sx1280/Sx1280.hpp"

namespace VCTR::ExVectrLink::datalink {

enum class LinkType : uint8_t {
  // One radio is used for tx and rx.
  Single,
  // Best pair are used for tx and rx.
  Diversity,
  // Both radios are used on tx and rx for max throughput.
  DualLink,
};

class Radiomanager {
public:
  Radiomanager(uint16_t nodeAddress);

  void addRadioDriver(network::datalink::Datalink_SX1280 &radioDriver);

  network::network::NetworkNode &getNetworkNode();

  /**
   * @brief Enables or disables FHSS.
   * @param enabled
   * @param key The key is used to determine the hopping sequence if FHSS is
   * enabled. Otherwise the key is the permanent channel to use.
   *
   */
  void setFHSSEnabled(bool enabled, uint16_t key);

  /**
   * @brief The ratio is the number of packets for one end to send before the
   * other sends one packet.
   * @note this setting is irrelevant for dual link.
   * @param numPackets The ratio to set.
   * @param forUs if true, then we send the given number packets before the
   * other sends one. If false, then the other sends the given number of packets
   * before we send one.
   *
   */
  void setPacketRatio(uint8_t numPackets, bool forUs = true);

  /**
   * @brief Enables or disables the use of both radios for tx and rx.
   * @details In dual link mode, two seperate links are setup for maximum
   * throughput.
   * @note this will fallback to diversity if link quality is insufficient for
   * dual link.
   */
  void enableDualLink(bool enable);

  /**
   * @brief Get the current link type.
   * @details The link type is determined by the hardware and current
   * conditions. E.g. for rx and tx with only a single radio, then the link type
   * will be Single. for rx and or tx with dual radios, then the link will be
   * Diversity
   */
  LinkType getCurrentLinkType() const;

private:
  network::network::NetworkNode networkNode;

  Core::ListArray<network::datalink::Datalink_SX1280 *> radioDrivers;

  LinkType linkTypeCurrent = LinkType::Single;

  bool dualLinkEnabled = false;
};

} // namespace VCTR::ExVectrLink::datalink

#endif // EXVECTRLINK_RADIOMANAGER_HPP