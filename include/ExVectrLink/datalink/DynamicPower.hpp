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

  /// @brief Sets the max power the dynamic power system will use. This
  /// doubles as the fixed operating power while dynamic power is disabled, so
  /// changing it there re-applies immediately (see setEnableDynamicPower()).
  void setDynMaxPower(uint8_t maxPowerDBm);
  /// @brief Sets the min power the dynamic power system will use.
  void setDynMinPower(uint8_t minPowerDBm);
  /// @brief Enables/disables the automatic (dynamic) power algorithm. When
  /// disabled the radio is pinned to the dynamic-max power (setDynMaxPower())
  /// rather than being left wherever the algorithm last set it.
  void setEnableDynamicPower(bool enable);

  /// @brief Absolute max power (dBm), hardware limit -- see setMaxPower().
  /// Already snapped down to the nearest entry in kPowerLevels.
  uint8_t getMaxPower() const;
  /// @brief Absolute min power (dBm), hardware limit -- see setMinPower().
  /// Already snapped up to the nearest entry in kPowerLevels.
  uint8_t getMinPower() const;
  /// @brief Max power (dBm) the dynamic power system is currently allowed to
  /// use -- not necessarily snapped/clamped to [getMinPower(), getMaxPower()]
  /// at set-time, but every level the dynamic power system actually applies
  /// is (see incPower()/decPower()/setPower()).
  uint8_t getDynMaxPower() const;
  /// @brief Min power (dBm) the dynamic power system is currently allowed to
  /// use -- see getDynMaxPower()'s note on clamping.
  uint8_t getDynMinPower() const;

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

  /// @brief Increments power by one step until the max is reached.
  /// @param force If true, will ignore the dynamic power limit max.
  void incPower(bool force = false);
  void decPower();
  void setPower(uint8_t powerDBm);

  void update(bool receivedPacket, int8_t rssi, int8_t snr, uint8_t lq);

private:
  /// @brief Re-clamps the current power into the active limits immediately:
  /// pinned to the dynamic-max while disabled, otherwise snapped inside the
  /// dynamic [min, max] range. Called whenever a limit or the enable flag
  /// changes so the change takes effect at once (callers must still push the
  /// result to the radios, e.g. via applyDynamicPowerToRadios()).
  void constrainPowerToLimits();

  uint8_t maxPowerDBm = 33;
  uint8_t minPowerDBm = 10;
  uint8_t maxDynPowerDBm = 33;
  uint8_t minDynPowerDBm = 10;
  uint8_t currentPowerDBm = minDynPowerDBm;
  uint8_t currentLq = 100;
  uint8_t missedPacketCount = 0;
  bool dynamicPowerEnabled = true;
  int64_t lastDecTime = 0;

  int8_t minRssi = -100;
  int8_t minSnr = 1;
  uint8_t minLq = 85;

  int8_t maxRssi = -110;
  int8_t maxSnr = 8;
  uint8_t maxLq = 95;
};

} // namespace VCTR::ExVectrLink::datalink
#endif // EXVECRLINK_DYNAMICPOWER_HPP