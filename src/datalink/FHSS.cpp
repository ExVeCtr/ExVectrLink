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
    : Core::Task_Periodic("FHSS", 100 * Core::MILLISECONDS), radioLink(radioI) {
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

// =============================================================================
// Status getters
// =============================================================================

FHSSState FHSS::getFhssState() const { return fhssState; }

float FHSS::getLinkQuality() const {
  return (float)(isRxSide ? linkQuality : otherEndLinkQuality);
}

int64_t FHSS::getTimingOffset() const { return slotTimer.getTimingOffset(); }

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
  if (isRxSide) {
    if (fhssState == FHSSState::Searching) {
      slotTimer.start(receiveStartTime);
    } else {
      slotTimer.sync(receiveStartTime);
    }
  }
  fhssState = FHSSState::Synced;
  lastPacketRcvTime = receiveStartTime;
}

void FHSS::hopChannel() {
  if (channelSequence.size() == 0)
    return;
  lastChannelHopTime = Core::NOW();
  currentChannelIdx = (currentChannelIdx + 1) % channelSequence.size();
  radioLink.setChannel(channelSequence[currentChannelIdx]);
}

void FHSS::transmitPacket(network::DataPacket &packet) {
  int lq = linkQuality * 255.0f;
  if (lq > 255) {
    lq = 255;
  }
  packet.payload.append((uint8_t)PacketType::Data);
  packet.payload.append((uint8_t)roleReverseCounter);
  packet.payload.append((uint8_t)lq);
  packet.payload.append((uint8_t)key);
  radioLink.transmitDataframe(packet);
  packet.payload.clear();
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

  // Store values
  otherEndLinkQuality = (float)recvLinkQuality / 255.0f;
  receivedPacket = true;

  // Update sync and timing based on received packet.
  if (isRxSide) {
    roleReverseCounter = txRoleReverseCounter + 1;
    if (roleReverseCounter >= numTxPacketsToRx) {
      roleReverseCounter = 0;
    }
  }
  syncTimer(packet.timestamp);

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

  if (slotTimer.needUpdate()) {
    setDeadline(now);
  }
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

  slotTimer.update(now);

  // --- Searching mode (RX side only) ---
  // In searching mode, slowly hop through channels trying to find a signal.
  if (fhssState == FHSSState::Synced &&
      Core::NOW() - lastPacketRcvTime > 10 * slotInterval) {
    fhssState = FHSSState::Searching;
  }

  // --- Synced mode ---
  // Advance slots as needed. May advance multiple if we fell behind.
  // if (fhssState == FHSSState::Synced) {
  //   slotTimer.update(now);
  // }

  // Update link quality stats.
  updateLinkQuality();
}

void FHSS::updateLinkQuality() {

  if (receiveSuccesses.size() < 2) {
    linkQuality = 0;
    otherEndLinkQuality = 0;
    return;
  }

  size_t successCount = 0;
  for (size_t i = 0; i < receiveSuccesses.size(); i++) {
    if (receiveSuccesses[i]) {
      successCount++;
    }
  }

  linkQuality = (float)successCount / (float)receiveSuccesses.size() + 0.1f;

  if (!receiveSuccesses(-1) && receiveSuccesses(-2)) {
    linkQuality = 0;
    otherEndLinkQuality = 0;
  }

  if (Core::NOW() - lastPacketRcvTime > 1 * Core::SECONDS) {
    otherEndLinkQuality = 0;
  }
}

void FHSS::timerEvent(bool isSlotStart) {
  if (isSlotStart) {

    if (isTransmitSlot) {
      transmitPacket(packetToSend);
    } else {
      receiveSuccesses.placeBack(receivedPacket, true);
    }

    slotCounter = (slotCounter + 1) % slotsPerHop;
    roleReverseCounter = (roleReverseCounter + 1) % numTxPacketsToRx;

    isTransmitSlot = !isRxSide;
    receivedPacket = false;

  } else {

    if (roleReverseCounter == 0) {
      isTransmitSlot = isRxSide && fhssState == FHSSState::Synced;
    }

    if (slotCounter == 0 &&
        (!isRxSide || fhssState == FHSSState::Synced ||
         Core::NOW() - lastChannelHopTime > 10 * slotInterval)) {
      hopChannel();
    }
  }
}

} // namespace VCTR::ExVectrLink::datalink
