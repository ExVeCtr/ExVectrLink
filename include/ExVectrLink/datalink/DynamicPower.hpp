#ifndef EXVECRLINK_DYNAMICPOWER_HPP
#define EXVECRLINK_DYNAMICPOWER_HPP

#include <cstdint>

namespace VCTR::ExVectrLink::datalink {

class DynamicPower {
public:
  void setMaxPower(uint8_t maxPowerDBm);
  void setMinPower(uint8_t minPowerDBm);
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
  static constexpr int8_t powerLevels[] = {10, 14, 17, 20, 24, 27, 30, 33};

  uint8_t maxPowerDBm = 33;
  uint8_t minPowerDBm = 10;
  uint8_t currentPowerDBm = minPowerDBm;
  uint8_t currentLq = 100;
  bool dynamicPowerEnabled = true;

  int8_t minRssi = -100;
  int8_t minSnr = -1;
  uint8_t minLq = 70;

  int8_t maxRssi = -110;
  int8_t maxSnr = 6;
  uint8_t maxLq = 90;
};

} // namespace VCTR::ExVectrLink::datalink
#endif // EXVECRLINK_DYNAMICPOWER_HPP