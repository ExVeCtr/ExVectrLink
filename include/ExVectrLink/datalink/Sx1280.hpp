#ifndef EXVECTRNETWORK_DATALINKSX1280_H_
#define EXVECTRNETWORK_DATALINKSX1280_H_

#include "ExVectrCore/list_buffer.hpp"

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrCore/list.hpp"

#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/interfaces/DatalinkInterface.hpp"

#include "sx12xxAL/src/SX128XLT.h"

#include <memory>

namespace VCTR::Net {

/**
 * @brief A class implementing a datalink layer for the SX1280 LoRa transceiver.
 */
class Datalink_SX1280 : public Net::Datalink_Interface,
                        public Core::Task_Periodic {
private:
  ///@brief Maximum length a data frame can be.
  static constexpr size_t dataLinkMaxFrameLength = 200;

  SX128XLT lora_;

  int NSS_PIN_;
  int NRESET_PIN_;
  int RFBUSY_PIN_;
  int DIO1_PIN_;
  int TX_EN_PIN_;
  int RX_EN_PIN_;

  int16_t receivedDataRSSI_;
  int16_t receivedDataSNR_;

  Core::ListBuffer<uint8_t, dataLinkMaxFrameLength * 5> transmitBuffer_;

  enum class RadioState {
    Idle,
    ChannelBusy,
    Transmitting,
    Receiving,
    ActivityDetection
  };
  RadioState radioState_ = RadioState::Idle;

  int64_t transmitStart_ = 0;
  int64_t transmitEnd_ = 0;

  int64_t receiveStart_ = 0;
  int64_t receiveEnd_ = 0;

  int64_t activityDetectionStart_ = 0;
  int64_t activityDetectionEnd_ = 0;

  int64_t channelBusyStart_ = 0;
  int64_t channelBusyEnd_ = 0;

  int64_t idleStart_ = 0;

  bool enableTransmit_ = false;

public:
  Datalink_SX1280(int nssPin, int nresetPin, int rfbusyPin, int dio1Pin,
                  int txEnPin, int rxEnPin);

  void taskInit() override;

  void taskThread() override;

  void taskCheck() override;

  int16_t lastPacketRSSI() const { return receivedDataRSSI_; }
  int16_t lastPacketSNR() const { return receivedDataSNR_; }

  size_t getBufferFreeSpace() const override;

  bool isChannelBlocked() const override;

private:
  void beginReceive(int timeout = 0);

  bool transmitAwaitingData();

  void receiveAwaitingData();

  bool transmitDataframe(const Dataframe &dataframe) override;
};

} // namespace VCTR::Net

#endif