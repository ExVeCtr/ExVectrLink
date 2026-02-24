#include "ExVectrCore/list_array.hpp"

#include "ExVectrNetwork/network/NetworkNode.hpp"

#include "ExVectrNetwork/datalink/sx1280/Sx1280.hpp"

#include "ExVectrLink/datalink/RadioManager.hpp"

namespace VCTR::ExVectrLink::datalink {

Radiomanager::Radiomanager(uint16_t nodeAddress) : networkNode(nodeAddress) {}

void Radiomanager::addRadioDriver(
    network::datalink::Datalink_SX1280 &radioDriver) {
  radioDrivers.append(&radioDriver);
}

network::network::NetworkNode &Radiomanager::getNetworkNode() {
  return networkNode;
}
/**
 * @brief Enables or disables FHSS.
 * @param enabled
 * @param key The key is used to determine the hopping sequence if FHSS is
 * enabled. Otherwise the key is the permanent channel to use.
 *
 */
void Radiomanager::setFHSSEnabled(bool enabled, uint16_t key) {}

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
void Radiomanager::setPacketRatio(uint8_t numPackets, bool forUs) {}

/**
 * @brief Enables or disables the use of both radios for tx and rx.
 * @details In dual link mode, two seperate links are setup for maximum
 * throughput.
 * @note this will fallback to diversity if link quality is insufficient for
 * dual link.
 */
void Radiomanager::enableDualLink(bool enable) { dualLinkEnabled = enable; }

/**
 * @brief Get the current link type.
 * @details The link type is determined by the hardware and current
 * conditions. E.g. for rx and tx with only a single radio, then the link type
 * will be Single. for rx and or tx with dual radios, then the link will be
 * Diversity
 */
LinkType Radiomanager::getCurrentLinkType() const { return linkTypeCurrent; }

} // namespace VCTR::ExVectrLink::datalink