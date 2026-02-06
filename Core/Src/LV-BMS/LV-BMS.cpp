#include "LV-BMS/LV-BMS.hpp"

void LV_BMS::init() {
//   LV_BMS::state = BMS_State_Machine->general_sm.current_state;

#if 0
  ProtectionManager::link_state_machine(&LV_BMS::BMS_State_Machine,
                                        static_cast<uint8_t>(BMS_State::FAULT));
#endif

  ProtectionManager::add_standard_protections();
  //add_protections();
  ProtectionManager::initialize();
  ProtectionManager::set_id(Boards::ID::BMSA);

  Data::init();
  DCLV::init();
}

void LV_BMS::start() {
  Data::start();
  Comms::init();

  Scheduler::register_task(1000, []() {
    LV_BMS::BMS_State_Machine.check_transitions();
  });
  BMS_State_Machine.start();
}

void LV_BMS::set_protection_name(Protection* protection,
                                 const std::string& name) {
  protection->set_name((char*)malloc(name.size() + 1));
  sprintf(protection->get_name(), "%s", name.c_str());
}

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

void LV_BMS::update() {
    if(Comms::received_turn_on_pfm) {
        DCLV::turn_on_pfm();
    }
    if(Comms::received_turn_off_pfm) {
        DCLV::turn_off_pfm();
    }
    if(Comms::received_set_pfm_frequency) {
        DCLV::set_pfm_frequency(DCLV::frequency);
    }
    if(Comms::received_set_pfm_dead_time) {
        DCLV::set_pfm_dead_time(DCLV::dead_time);
    }
}