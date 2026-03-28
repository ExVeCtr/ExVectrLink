#include <Arduino.h>

#include "ExVectrCore/CanSerialize.hpp"
#include "ExVectrCore/handler.hpp"
#include "ExVectrCore/list_array.hpp"
#include "ExVectrCore/task_types.hpp"

#include "ExVectrNetwork/DataPacket.hpp"
#include "ExVectrNetwork/datalink/RadioI.hpp"

#include "ExVectrLink/datalink/FHSS.hpp"

namespace VCTR::ExVectrLink::datalink {

// =============================================================================
// Construction
// =============================================================================

FHSS::FHSS(VCTR::network::datalink::RadioI &radioI)
    : Core::Task_Periodic("FHSS", 1 * Core::MILLISECONDS), radioLink(radioI) {
  Core::getSystemScheduler().addTask(*this);
}

// =============================================================================
// Configuration setters / getters
// =============================================================================

void FHSS::setFhssKey(uint8_t newKey) {
  key = newKey;
  generateChannelSequence(key);
}

uint8_t FHSS::getFhssKey() const { return key; }
void FHSS::setIsRxSide(bool isRxSide) { this->isRxSide = isRxSide; }

void FHSS::setSlotInterval(int64_t interval) {
  slotInterval = interval;
  slotTimer.setPeriod(slotInterval);
}
int64_t FHSS::getSlotInterval() const { return slotInterval; }

void FHSS::setSlotsPerHop(uint8_t slots) { slotsPerHop = slots; }
uint8_t FHSS::getSlotsPerHop() const { return slotsPerHop; }

void FHSS::setRxSlotIndex(uint8_t index) {
  if (index > 16) {
    index = 16; // Cap at 16 to fit in 4 bits of trailer.
  } else if (index == 0) {
    index = 1; // Cannot be 0
  }
  numTxPacketsToRx = index;
}
uint8_t FHSS::getRxSlotIndex() const { return numTxPacketsToRx; }

void FHSS::setRxEarlyOffset(int64_t offset) { rxEarlyOffset = offset; }
int64_t FHSS::getRxEarlyOffset() const { return rxEarlyOffset; }

// =============================================================================
// Status getters
// =============================================================================

FHSSState FHSS::getFhssState() const { return fhssState; }

float FHSS::getLinkQuality() const {
  return (float)(isRxSide ? linkQuality : otherEndLinkQuality) * 255.0f;
}

int64_t FHSS::getTimingOffset() const { return timingOffset; }

uint8_t FHSS::getSlotCounter() const { return slotCounter; }

// =============================================================================
// DatalinkI interface
// =============================================================================

bool FHSS::transmitDataframe(const VCTR::network::DataPacket &dataframe) {
  if (packetToSend.payload.size() > 0) {
    return false; // Already have a pending packet.
  }

  // uint8_t trailerByte1 =
  //     (0b10000000) | (uint8_t)(uint8_t)(linkQuality * 127.0f) & 0x7F;
  // uint8_t trailerByte2 = ((slotCounter & 0x0F) << 4) | (key & 0x0F);

  packetToSend = dataframe;
  // Append trailer: [PacketType][slotCounter][linkQuality][key]
  packetToSend.payload.append((uint8_t)PacketType::Data);
  packetToSend.payload.append(roleReverseCounter);
  packetToSend.payload.append(linkQuality);
  packetToSend.payload.append(key);
  return true;
}

size_t FHSS::getMaxPacketSize() const { return 8; }

bool FHSS::isChannelBlocked() const { return packetToSend.payload.size() > 0; }

// =============================================================================
// Sequence generation
// =============================================================================

void FHSS::generateChannelSequence(uint8_t key) {
  channelSequence.clear();

  size_t numChannels = radioLink.getNumChannels();
  if (numChannels == 0)
    return;

  // Simple Fisher-Yates-style deterministic shuffle seeded by key.
  for (uint8_t i = 0; i < numChannels; i++) {
    channelSequence.append(i);
  }

  // Deterministic shuffle using the key as seed.
  uint32_t seed = key * 2654435761u; // Knuth multiplicative hash
  for (size_t i = numChannels - 1; i > 0; i--) {
    seed ^= (seed << 13);
    seed ^= (seed >> 17);
    seed ^= (seed << 5);
    size_t j = seed % (i + 1);
    // Swap
    uint8_t tmp = channelSequence[i];
    channelSequence[i] = channelSequence[j];
    channelSequence[j] = tmp;
  }

  currentChannelIdx = 0;
}

// =============================================================================
// Slot/hop logic helpers
// =============================================================================

void FHSS::syncTimer(int64_t receiveStartTime) {
  slotTimer.start(receiveStartTime);
}

void FHSS::hopChannel() {
  if (channelSequence.size() == 0)
    return;
  currentChannelIdx = (currentChannelIdx + 1) % channelSequence.size();
  radioLink.setChannel(channelSequence[currentChannelIdx]);
}

void FHSS::transmitPacket(const uint8_t *data, size_t length) {
  network::DataPacket packet;
  for (size_t i = 0; i < length && data != nullptr; i++) {
    packet.payload.append(data[i]);
  }
  packet.payload.append((uint8_t)PacketType::Data);
  packet.payload.append(roleReverseCounter);
  packet.payload.append(linkQuality);
  packet.payload.append(key);
  radioLink.transmitDataframe(packet);
}

void FHSS::receivePacket(const network::DataPacket &packet) {
  // return;
  if (packet.payload.size() < 4)
    return;

  size_t payloadEnd = packet.payload.size();

  // Read trailer (appended at the end).
  auto packetKey = packet.payload[payloadEnd - 1];
  auto recvLinkQuality = packet.payload[payloadEnd - 2];
  auto txRoleReverseCounter = packet.payload[payloadEnd - 3];
  auto packetType = (PacketType)packet.payload[payloadEnd - 4];

  // Validate key.
  if (packetKey != key)
    return;

  // Store the other side's reported link quality.
  otherEndLinkQuality = recvLinkQuality;

  // Forward data packets to application handlers.
  if (packetType == PacketType::Data && packet.payload.size() > 4) {
    auto dataPacket = packet;
    dataPacket.payload.popDiscard(4);
    receiveHandlers_.callHandlers(dataPacket);
  }
}
// =============================================================================
// Task interface
// =============================================================================

void FHSS::taskInit() {
  // Register receive handler on the underlying radio link.
  radioLink.addReceiveHandler(
      [this](const network::DataPacket &packet) { receivePacket(packet); });

  // Initialise timing.
  slotCounter = 0;

  generateChannelSequence(key);

  // Set initial channel.
  if (channelSequence.size() > 0) {
    radioLink.setChannel(channelSequence[0]);
  }

  slotTimer.start();
  slotTimer.setPeriod(slotInterval);
  slotTimer.setDutyCycleRatio(0.5);
  slotTimer.setCallback([this](Core::EdgeType edge) {
    timerEvent(edge == Core::EdgeType::Rising);
  });
}

void FHSS::taskCheck() {
  int64_t now = Core::NOW();

  // if (slotTimer.needUpdate()) {
  //   setDeadline(now);
  // }
}

void FHSS::taskThread() {
  int64_t now = Core::NOW();

  // TX side is always synced.
  if (!isRxSide && fhssState != FHSSState::Synced) {
    fhssState = FHSSState::Synced;
  }

  // if (packetToSend.payload.size() > 0) {
  //   transmitPacket(packetToSend.payload.getPtr(),
  //   packetToSend.payload.size()); packetToSend.payload.clear();
  // }

  // slotTimer.update(now);

  // --- Searching mode (RX side only) ---
  // In searching mode, slowly hop through channels trying to find a signal.
  // if (fhssState == FHSSState::Searching) {
  // }

  // --- Synced mode ---
  // Advance slots as needed. May advance multiple if we fell behind.
  // if (fhssState == FHSSState::Synced) {
  //   slotTimer.update(now);
  // }

  // Update link quality stats.
  updateLinkQuality();
}

void FHSS::updateLinkQuality() {

  if (receiveSuccesses.size() == 0) {
    linkQuality = 0;
    return;
  }

  size_t successCount = 0;
  for (size_t i = 0; i < receiveSuccesses.size(); i++) {
    if (receiveSuccesses[i]) {
      successCount++;
    }
  }

  linkQuality = (float)successCount / (float)receiveSuccesses.size();
}

void FHSS::timerEvent(bool isSlotStart) {
  if (isSlotStart) {

    if (packetToSend.payload.size() > 0) {
      transmitPacket(packetToSend.payload.getPtr(),
                     packetToSend.payload.size());
      packetToSend.payload.clear();
    } else {
      transmitPacket();
    }

    // slotCounter = (slotCounter + 1) % slotsPerHop;
    // roleReverseCounter = (roleReverseCounter + 1) % numTxPacketsToRx;

    // if (isTransmitSlot) {
    //   if (packetToSend.payload.size() > 0) {
    //     transmitPacket(packetToSend.payload.getPtr(),
    //                    packetToSend.payload.size());
    //     packetToSend.payload.clear();
    //   } else {
    //     transmitPacket();
    //   }
    // }

    // isTransmitSlot = !isRxSide;

  } else {

    // if (roleReverseCounter == 0) {
    //   isTransmitSlot = isRxSide && fhssState == FHSSState::Synced;
    // }

    // if (slotCounter == 0) {
    //   // hopChannel();
    // }
  }
}

} // namespace VCTR::ExVectrLink::datalink
