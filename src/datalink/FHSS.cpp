#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/RadioI.hpp"
#include "ExVectrNetwork/datalink/sx1280/Sx1280.hpp"

#include "ExVectrLink/datalink/FHSS.hpp"

namespace VCTR::ExVectrLink::datalink {

FHSS::FHSS(VCTR::network::datalink::RadioI &radioI)
    : Core::Task_Periodic("FHSS", 200 * Core::MILLISECONDS), radioLink(radioI) {

  Core::getSystemScheduler().addTask(*this);
}

void FHSS::setFhssKey(uint8_t newKey) {
  key = newKey;
  generateSequence();
}

uint8_t FHSS::getFhssKey() const { return key; }

void FHSS::enableFhss(bool enable) { fhssEnabled = enable; }

bool FHSS::transmitDataframe(const VCTR::network::DataPacket &dataframe) {
  if (waitingForSendFinish) {
    return false;
  }
  packetToSend = dataframe;
  waitingForSendFinish = true;
  return true;
}

size_t FHSS::getMaxPacketSize() const { return radioLink.getMaxPacketSize(); }

bool FHSS::isChannelBlocked() const { return radioLink.isChannelBlocked(); }

void FHSS::taskCheck() {
  if (waitingForSendFinish) {
    setDeadline(Core::NOW());
  }
}

void FHSS::taskInit() {
  radioLink.addReceiveHandler([this](const network::DataPacket &packet) {
    receiveHandlers_.callHandlers(packet);
  });
}

void FHSS::taskThread() {
  if (!radioLink.isChannelBlocked() && waitingForSendFinish) {
    radioLink.transmitDataframe(packetToSend);
    waitingForSendFinish = false;
  }
}

void FHSS::generateSequence() {} // For now nothing

} // namespace VCTR::ExVectrLink::datalink