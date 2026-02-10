#include "LV-BMS/LV-BMS.hpp"

//ST_LIB::TimerWrapper<timer_us_tick_def> *LV_BMS::global_us_timer = nullptr;
// ST_LIB::DigitalOutputDomain::Instance *LV_BMS::operational_led = nullptr;
// ST_LIB::DigitalOutputDomain::Instance *LV_BMS::fault_led = nullptr;

#if 0
void LV_BMS::add_protections() {
  Protection* soc_protection = &ProtectionManager::_add_protection(
    &Data::SOC, Boundary<float, OUT_OF_RANGE>{24.0f, 80.0f});
  set_protection_name(soc_protection, "SOC");

  /* Protection* temperature_1_protection = &ProtectionManager::_add_protection(
    &Data::temperature[0], Boundary<float, OUT_OF_RANGE>{10.0f, 50.0f});
  set_protection_name(temperature_1_protection, "Temperature 1");

  Protection* temperature_2_protection = &ProtectionManager::_add_protection(
    &Data::temperature[1], Boundary<float, OUT_OF_RANGE>{10.0f, 50.0f});
  set_protection_name(temperature_2_protection, "Temperature 2");

  Protection* temperature_3_protection = &ProtectionManager::_add_protection(
    &Data::temperature[2], Boundary<float, OUT_OF_RANGE>{10.0f, 50.0f});
  set_protection_name(temperature_3_protection, "Temperature 3");

  Protection* temperature_4_protection = &ProtectionManager::_add_protection(
    &Data::temperature[3], Boundary<float, OUT_OF_RANGE>{10.0f, 50.0f});
  set_protection_name(temperature_4_protection, "Temperature 4");

  Protection* bms_disconnection_protection =
    &ProtectionManager::_add_protection(&Data::conv_rate,
                                        Boundary<float, BELOW>{0.5f});
  set_protection_name(bms_disconnection_protection, "BMS Conversion Rate"); */
}
#endif
