#pragma once

#include <stdint.h>

namespace thrust_mapping {

class CollectiveThrustMapping {
 public:
  CollectiveThrustMapping();
  CollectiveThrustMapping(const double thrust_map_a, const double thrust_map_b,
                          const double thrust_map_c,
                          const bool perform_thrust_voltage_compensation,
                          const double thrust_ratio_voltage_map_a,
                          const double thrust_ratio_voltage_map_b,
                          const int n_lipo_cells);

  virtual ~CollectiveThrustMapping();

  uint16_t inverseThrustMapping(const double thrust,
                                const double battery_voltage) const;

  bool loadParameters();

 private:
  double thrust_map_a_;
  double thrust_map_b_;
  double thrust_map_c_;

  bool perform_thrust_voltage_compensation_;
  double thrust_ratio_voltage_map_a_;
  double thrust_ratio_voltage_map_b_;
  int n_lipo_cells_;

  // Constants
  static constexpr double kMinBatteryCompensationVoltagePerCell_ = 3.5;
  static constexpr double kMaxBatteryCompensationVoltagePerCell_ = 4.2;
};

}  // namespace thrust_mapping
