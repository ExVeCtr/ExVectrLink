#include "ExVectrCore/print.hpp"

#include "ExVectrCore/list_buffer.hpp"

#include "ExVectrCore/topic.hpp"
#include "ExVectrCore/topic_subscribers.hpp"

#include "ExVectrCore/list.hpp"

#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/interfaces/DatalinkInterface.hpp"

#include "sx12xxAL/src/SX128XLT.h"

#include "Sx1280.hpp"

// #define SX1280_DEBUG

namespace VCTR {

namespace Net {

Datalink_SX1280::Datalink_SX1280(int nssPin, int nresetPin, int rfbusyPin,
                                 int dio1Pin, int txEnPin, int rxEnPin)
    : Task_Periodic("Datalink_SX1280", 100 * Core::MILLISECONDS) {

  NSS_PIN_ = nssPin;
  NRESET_PIN_ = nresetPin;
  RFBUSY_PIN_ = rfbusyPin;
  DIO1_PIN_ = dio1Pin;
  TX_EN_PIN_ = txEnPin;
  RX_EN_PIN_ = rxEnPin;

  Core::getSystemScheduler().addTask(*this);
}

void Datalink_SX1280::taskInit() {

  pinMode(NSS_PIN_, OUTPUT);
  pinMode(NRESET_PIN_, OUTPUT);
  pinMode(RFBUSY_PIN_, INPUT);
  pinMode(DIO1_PIN_, INPUT);
  pinMode(TX_EN_PIN_, OUTPUT);
  pinMode(RX_EN_PIN_, OUTPUT);

  digitalWrite(NSS_PIN_, HIGH);
  digitalWrite(NRESET_PIN_, HIGH);
  digitalWrite(TX_EN_PIN_, LOW);
  digitalWrite(RX_EN_PIN_, LOW);

  if (!lora_.begin(NSS_PIN_, NRESET_PIN_, RFBUSY_PIN_, DIO1_PIN_, RX_EN_PIN_,
                   TX_EN_PIN_, DEVICE_SX1280)) {
    LOG_MSG("Failed to initialize SX1280!\n");
    setInitialised(false);
    setPaused(true);
    return;
  }

  lora_.setupLoRa(2445000000, 0, LORA_SF5, LORA_BW_1600, LORA_CR_4_6);
  lora_.setDioIrqParams(IRQ_RADIO_ALL, IRQ_RADIO_ALL, 0, 0);
  lora_.setHighSensitivity();

  lora_.receiveSXBuffer(0, 0, NO_WAIT);

  setPaused(true); // Pause the thread until dio interrupt

  radioState_ = RadioState::Idle;
}

void Datalink_SX1280::taskCheck() {

  if (digitalRead(DIO1_PIN_) == HIGH ||
      (transmitBuffer_.size() > 0 &&
       radioState_ ==
           RadioState::Idle)) { // If the dio pin is high or we have data to
                                // send, then we should run the task
    setPaused(false);
    setRelease(0);
  }
}

void Datalink_SX1280::taskThread() {

  auto threadTime = Core::NOW();

  if (digitalRead(DIO1_PIN_) == HIGH) {

#ifdef SX1280_DEBUG
    // Serial.println("DIO1 Pin is high!");
#endif

    uint16_t irqStatus = lora_.readIrqStatus();
    // lora_.clearIrqStatus(IRQ_RADIO_ALL);

    if ((irqStatus &
         IRQ_PREAMBLE_DETECTED)) { // We have detected a preamble. This means we
                                   // are receiving data.

      radioState_ = RadioState::ChannelBusy;
      channelBusyStart_ = threadTime;

      VRBS_MSG("Channel busy! Detected preamble\n");
      lora_.clearIrqStatus(IRQ_PREAMBLE_DETECTED);
    }

    if ((irqStatus & IRQ_RX_DONE) && (irqStatus & IRQ_HEADER_VALID) &&
        !(irqStatus & IRQ_CRC_ERROR)) {

      VRBS_MSG("Received data! Decoding...\n");

      receiveAwaitingData();
      radioState_ = RadioState::Idle;
      idleStart_ = threadTime;
      receiveEnd_ = threadTime;

      lora_.clearIrqStatus(IRQ_RX_DONE + IRQ_HEADER_VALID +
                           IRQ_PREAMBLE_DETECTED); // Clear the irq status. We
                                                   // are done receiving data.
    }

    if (irqStatus & (IRQ_HEADER_ERROR)) {

      radioState_ = RadioState::Idle;
      idleStart_ = threadTime;
      receiveEnd_ = threadTime;

      LOG_MSG("Interrupt says header error!\n");

      lora_.clearIrqStatus(IRQ_HEADER_ERROR); // Clear the irq status. We are
                                              // done receiving data.
    }

    if (irqStatus & (IRQ_CRC_ERROR)) {

      radioState_ = RadioState::Idle;
      idleStart_ = threadTime;
      receiveEnd_ = threadTime;

      LOG_MSG("Interrupt says crc error!\n");

      lora_.clearIrqStatus(
          IRQ_CRC_ERROR); // Clear the irq status. We are done receiving data.
    }

    if (irqStatus & (IRQ_RX_TIMEOUT)) {

      radioState_ = RadioState::Idle;
      idleStart_ = threadTime;
      receiveEnd_ = threadTime;

      LOG_MSG("Interrupt says rx timed out!\n");

      lora_.clearIrqStatus(
          IRQ_RX_TIMEOUT); // Clear the irq status. We are done receiving data.
    }

    if (irqStatus & (IRQ_TX_DONE)) {

      radioState_ = RadioState::Idle;
      idleStart_ = threadTime;
      transmitEnd_ = threadTime;

      VRBS_MSG("Interrupt says tx done!\n");

      lora_.clearIrqStatus(
          IRQ_TX_DONE); // Clear the irq status. We are done receiving data.
    }

    if (irqStatus & (IRQ_TX_TIMEOUT)) {

      radioState_ = RadioState::Idle;
      idleStart_ = threadTime;
      transmitEnd_ = threadTime;

      VRBS_MSG("Interrupt says tx timeout!\n");

      lora_.clearIrqStatus(
          IRQ_TX_TIMEOUT); // Clear the irq status. We are done receiving data.
    }

    // lora_.clearIrqStatus(IRQ_RADIO_ALL);

    if (irqStatus & (IRQ_CAD_ACTIVITY_DETECTED)) {

      radioState_ = RadioState::Receiving;
      // radioState_ = RadioState::ActivityDetection;
      channelBusyStart_ = threadTime;

      beginReceive(1000);
      // lora_.receiveSXBuffer(0, 0, NO_WAIT); //Set to receive mode. Use 0xFFFF
      // to go into continuous receive mode.

      // lora_.startCAD(LORA_CAD_08_SYMBOL);

      VRBS_MSG("Channel busy! Channel activity detected! Checking again\n");

      lora_.clearIrqStatus(
          IRQ_CAD_ACTIVITY_DETECTED); // Clear the irq status. We are done
                                      // receiving data.
    }

    if (irqStatus & (IRQ_CAD_DONE)) {

      lora_.clearIrqStatus(
          IRQ_CAD_DONE); // Clear the irq status. We are done receiving data.

      if (radioState_ == RadioState::ChannelBusy) {
        channelBusyEnd_ = threadTime;
        VRBS_MSG("Channel is free again\n");
      }
      channelBusyEnd_ = threadTime;
      radioState_ = RadioState::Idle;
      idleStart_ = threadTime;

      VRBS_MSG("Channel is free!\n");

      if (transmitBuffer_.size() > 0) {

        if (transmitAwaitingData()) { // If we have data to send, then send it.
          radioState_ = RadioState::Transmitting;
          transmitStart_ = threadTime;
          VRBS_MSG("Sending data\n");
        }
      }
    }
  }

  /*if (radioState_ == RadioState::ChannelBusy) { // The channel is busy. We
  should immediatly start receiving as we want this data.

      radioState_ = RadioState::Receiving;
      beginReceive();
      receiveStart_ = threadTime;

      VRBS_MSG("Channel is busy. Beginning receive to collect the data\n");

  }*/

  if (transmitBuffer_.size() > 0 &&
      (radioState_ == RadioState::Idle ||
       radioState_ == RadioState::Receiving) &&
      threadTime - transmitEnd_ >
          1 * Core::MILLISECONDS) { // If we have data to send and are currently
                                    // not doing anything. Lets check if we can
                                    // send.

    radioState_ = RadioState::ActivityDetection;
    lora_.startCAD(LORA_CAD_08_SYMBOL);

    VRBS_MSG("Channel busy check before transmit!\n");
  }

  if (radioState_ == RadioState::Idle) { // Looks like we went into idle state.
                                         // We should start receiving data.

    beginReceive(); // Set to receive mode. Use 0xFFFF to go into continuous
                    // receive mode.
    receiveStart_ = threadTime;

    VRBS_MSG(
        "Radio is idle. Beginning receive to collect possible transmissions\n");
  }

  if (radioState_ == RadioState::Receiving &&
      threadTime - receiveStart_ >
          500 * Core::MILLISECONDS) { // Timout case for receiving data. We
                                      // should start receiving data again.

    beginReceive();
    receiveStart_ = threadTime;

    VRBS_MSG("Radio RX timeout issue. Putting back into receive\n");
  }

  if (radioState_ == RadioState::Transmitting &&
      threadTime - transmitStart_ >
          500 * Core::MILLISECONDS) { // Timout case for transmitting data. Go
                                      // into idle as something went wrong

    radioState_ = RadioState::Idle;

    VRBS_MSG("Radio TX timeout issue. Putting back into idle\n");
  }

  if ((radioState_ == RadioState::ChannelBusy ||
       radioState_ == RadioState::ActivityDetection) &&
      threadTime - channelBusyStart_ >
          500 * Core::MILLISECONDS) { // Timout case for channel busy. Go into
                                      // idle as something went wrong

    radioState_ = RadioState::Idle;

    VRBS_MSG("Radio Channel busy timeout issue. Putting back into idle\n");
  }
}

void Datalink_SX1280::beginReceive(int timeout) {

  radioState_ = RadioState::Receiving; // Set the radio to receiving mode.

  lora_.setMode(MODE_STDBY_RC);     // Set the radio to standby mode. This will
                                    // allow us to receive data.
  lora_.setBufferBaseAddress(0, 0); // order is TX RX
  lora_.setDioIrqParams(IRQ_RADIO_ALL,
                        (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT + IRQ_HEADER_ERROR +
                         IRQ_PREAMBLE_DETECTED),
                        0, 0); // set for IRQ on RX done or timeout
  lora_.setRx(timeout);        // set to receive mode
}

bool Datalink_SX1280::transmitAwaitingData() {

  if (transmitBuffer_.size() == 0)
    return false; // Nothing to send. Return.
  // if (radioState_ != RadioState::Idle) return false; //Radio is not ready to
  // send data. Return.

  const uint8_t packetLimit = 250;
  uint8_t buffer[packetLimit];
  uint8_t bufferSize = 0;
  uint8_t crc = 0;

  VRBS_MSG("There is data to send. Moving segments into buffer...\n");

  // We keep placing each packet frame into the buffer untill the next one would
  // be too much.
  while (transmitBuffer_.size() > 0 &&
         bufferSize + transmitBuffer_[0] + 2 <= packetLimit &&
         transmitBuffer_[0] !=
             0) { // The next addition of bytes would be the frame size plus
                  // also the frame size number. This number is needed by
                  // receiver to decode. An addistion byte for the crc.

    auto frameLen = transmitBuffer_[0];
    VRBS_MSG("Segment is %d bytes long\n", frameLen);

    // transmitBuffer_.removeFront(); //Dont remove this. This is needed by the
    for (int i = 0; i < frameLen + 1; i++) {
      buffer[i + bufferSize] = transmitBuffer_[i];
      crc += buffer[i + bufferSize];
    }
    transmitBuffer_.removeFront(frameLen + 1);
    bufferSize += frameLen + 1;
  }
  buffer[bufferSize] = crc;
  bufferSize += 1;

  /*transmitBuffer_.takeFront(bufferSize); //Get the first byte. This is the
  length of the frame.

  if (bufferSize == 0) {
      LOG_MSG("Datalink: No data to send. Failure.\n");
      transmitBuffer_.clear(); //Clear the buffer. Something is wrong.
      return false; //No data to send. Failure.
  }

  if (bufferSize > packetLimit) {
      LOG_MSG("Datalink: Buffer overflow. Failure.\n");
      transmitBuffer_.clear(); //Clear the buffer. Something is wrong.
      return false; //Buffer overflow case. Failure.
  }


  for (size_t i = 0; i < bufferSize; i++) { //We have to move all elements ahead
  the one to be removed, one place down. buffer[i] = transmitBuffer_[i];
      //crc += buffer[i];
  }
  //bufferSize = transmitBuffer_.size();
  transmitBuffer_.removeFront(bufferSize); //Remove the data bytes from the
  buffer.*/

  VRBS_MSG("Finished making buffer. Sending data! Buffer length: %d\n",
           bufferSize);

  if (bufferSize > 0) {
    lora_.transmit(buffer, bufferSize, 0, 12, NO_WAIT);
    return true;
  } else {
    LOG_MSG(
        "Datalink: No data to send. Failure. Why does the buffer contain data, "
        "but the data is marked with 0 length?... Buffer will be cleared\n");
    transmitBuffer_.clear();
  }

  return false; // No data to send. Failure.
}

void Datalink_SX1280::receiveAwaitingData() {

  VRBS_MSG("Interrupt says data received\n");

  size_t packetL = lora_.readRXPacketL();

  if (packetL > 0) { // make sure packet is okay

    receivedDataRSSI_ = lora_.readPacketRSSI();
    receivedDataSNR_ = lora_.readPacketSNR();

    // uint8_t buffer[packetL];
    Core::ListArray<uint8_t> bufferArray;
    bufferArray.setSize(packetL);
    // uint8_t crc = 0;

    lora_.startReadSXBuffer(0);
    lora_.readBuffer(bufferArray.getPtr(), packetL);
    lora_.endReadSXBuffer();

    // receiveTopic_.publish(bufferArray);

    uint8_t crc = 0;
    for (int i = 0; i < packetL - 1; i++)
      crc += bufferArray[i];
    uint8_t crcRcv = bufferArray[packetL - 1];

    if (crc == crcRcv) { // Only decode if crc is correct.

      // Core::ListBuffer<uint8_t, dataLinkMaxFrameLength> receivedData;
      Dataframe receivedData;
      for (size_t i = 0; i < packetL - 1;) {

        auto segLen = bufferArray[i];
        i++;                               // Read the size
        receivedData.data.clear();         // Make sure its empty
        for (int j = 0; j < segLen; j++) { // Place data into list.
          receivedData.data.placeBack(bufferArray[j + i]);
        }
        VRBS_MSG("Data segment Received is %d bytes long.\n", segLen);
        receiveHandlers_.callHandlers(receivedData);

        i += segLen;
      }
    } else {
      LOG_MSG("Received data is corrupt and cant be decoded reliably! Data len "
              "%d, crcRcv %d, crc calc %d\n",
              packetL, crcRcv, crc);
    }
  } else {
    LOG_MSG("Data was 0 bytes long! CRITICAL ERROR\n");
  }
}

bool Datalink_SX1280::transmitDataframe(const Dataframe &dataframe) {

  // VRBS_MSG("Received %d bytes from topic to send. Pointer %d \n",
  // item.size(), this);

  auto len = dataframe.data.size();
  if (len > dataLinkMaxFrameLength) {
    LOG_MSG("Datalink: Max frame length exceeded. Failure.\n");
    return false; // Max frame length exceeded. Failure.
  }
  if (len > transmitBuffer_.sizeMax() - transmitBuffer_.size() - 1) {
    LOG_MSG("Datalink: Buffer overflow. Failure.\n");
    return false; // Buffer overflow case. Failure.
  }

#ifdef SX1280_DEBUG
  Serial.println("Data received to send is " + String(len) + " bytes long");
#endif

  transmitBuffer_.placeBack(len);
  for (size_t i = 0; i < len; i++)
    transmitBuffer_.placeBack(dataframe.data[i]);
  return true;
}

} // namespace Net

} // namespace VCTR