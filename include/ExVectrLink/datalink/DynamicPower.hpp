#ifndef EXVECRLINK_DYNAMICPOWER_HPP
#define EXVECRLINK_DYNAMICPOWER_HPP

#include <cstddef>
#include <cstdint>

namespace VCTR::ExVectrLink::datalink {

class DynamicPower {
public:
  static constexpr uint8_t kPowerLevels[] = {10, 14, 17, 20, 24, 27, 30, 33};
  static constexpr size_t kNumPowerLevels =
      sizeof(kPowerLevels) / sizeof(kPowerLevels[0]);

  /// @brief  Sets the absolute max power that can be used. Hardware Limit.
  void setMaxPower(uint8_t maxPowerDBm);
  /// @brief Sets the absolute min power that can be used. Hardware Limit.
  void setMinPower(uint8_t minPowerDBm);

  /// @brief Sets the max power the dynamic power system will use.
  void setDynMaxPower(uint8_t maxPowerDBm);
  /// @brief Sets the min power the dynamic power system will use.
  void setDynMinPower(uint8_t minPowerDBm);
  void setEnableDynamicPower(bool enable);

  bool isDynamicPowerEnabled() const;
  int8_t getPower() const;

  /**
   * @brief If any of the values are below the given ones, then power is
   * incremented.
   */
  void setIncParameters(int8_t minRssi, int8_t minSnr, uint8_t minLq);
  /**
   * @brief If all of the values are above the given ones, then power is
   * decremented.
   */
  void setDecParameters(int8_t maxRssi, int8_t maxSnr, uint8_t maxLq);

  void incPower();
  void decPower();
  void setPower(uint8_t powerDBm);

  void update(bool receivedPacket, int8_t rssi, int8_t snr, uint8_t lq);

private:
  uint8_t maxPowerDBm = 33;
  uint8_t minPowerDBm = 10;
  uint8_t maxDynPowerDBm = 33;
  uint8_t minDynPowerDBm = 10;
  uint8_t currentPowerDBm = minPowerDBm;
  uint8_t currentLq = 100;
  uint8_t missedPacketCount = 0;
  bool dynamicPowerEnabled = true;
  int64_t lastDecTime = 0;

  int8_t minRssi = -100;
  int8_t minSnr = 1;
  uint8_t minLq = 70;

  int8_t maxRssi = -110;
  int8_t maxSnr = 6;
  uint8_t maxLq = 90;
};

} // namespace VCTR::ExVectrLink::datalink
#endif // EXVECRLINK_DYNAMICPOWER_HPP