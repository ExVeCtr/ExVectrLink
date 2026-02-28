#ifndef EXVECTRLINK_LINKMANAGER_HPP
#define EXVECTRLINK_LINKMANAGER_HPP

#include <cstdint>

#include "ExVectrCore/task_types.hpp"

#include "ExVectrLink/ExVectrLinkI.hpp"

namespace VCTR::ExVectrLink /* ExVectrLinkI */ {

/**
 * @brief The Link Manager is responsible for managing the high level features
 * such as binding, Power management, failsafe detection and recovery.
 */
class LinkManager : public Core::Task_Periodic {
public:
  /**
   * @brief Construct a new Link Manager object.
   * @param link The ExVectrLink interface to use for managing the link.
   * @param mak The media access key to use for FHSS. Must be different from all
   * other links on this network.
   */
  LinkManager(VCTR::ExVectrLink::ExVectrLinkI &link, uint8_t mak);

  ~LinkManager() = default;

  void taskInit() override;
  void taskCheck() override;
  void taskThread() override;

  void enableFhss(bool enable);

  void setMaxTxPower(uint8_t maxDBm);

  bool isFailsafe() const;

  // Returns a value from 0-100 with 100 being the best quality.
  uint8_t getLinkQuality() const;
  uint8_t getLinkRSSI() const;
  uint8_t getLinkSNR() const;
  uint8_t getLinkAntenna() const;

  void startBinding();
  uint32_t getFhssSequenceKey() const;

  void setMak(uint8_t mak);

private:
  // Extension hooks for higher-level link policies.
  // Intentionally simple stubs in the cpp for later expansion.
  void updateFailsafeState();
  void updateDynamicPowerManagement();
  void updateBindingState();

  VCTR::ExVectrLink::ExVectrLinkI &link;
  uint8_t mak;

  uint8_t maxTxPowerDBm = 20;
  uint8_t currentTxPowerDBm = 20;

  uint32_t fhssSequenceKey = 0;

  bool fhssEnabled = false;
  bool bindingInProgress = false;
  bool failsafe = false;

  int64_t lastHealthyLinkTime = 0;
};

} // namespace VCTR::ExVectrLink

#endif // EXVECTRLINK_LINKMANAGER_HPP