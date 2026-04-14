#define BCC_STLIB_IMPLEMENTATION
#include "LV-BMS/LV-BMS.hpp"
#include "float.h"

ST_LIB::DigitalOutputDomain::Instance *spi_cs;
ST_LIB::SPIDomain::SPIWrapper<spi_def> *spi_wrapper;

#if LV_BMS_VERSION_MAJOR == 10
void LV_BMS::BMSConfig::SPI_transmit(const span<uint8_t> data) {
  spi_wrapper->send(data);
}
void LV_BMS::BMSConfig::SPI_receive(span<uint8_t> buffer) {
  spi_wrapper->receive(buffer);
}

void LV_BMS::BMSConfig::SPI_CS_turn_off() {
  spi_cs->turn_off();
}
void LV_BMS::BMSConfig::SPI_CS_turn_on() {
  spi_cs->turn_on();
}

int32_t LV_BMS::BMSConfig::get_tick() {
  return GetMicroseconds();
}
#endif

//---------------------------------------------------------------

void LV_BMS::init() {
  /* Comms init */ {
    DataPackets::Battery_Voltages_init(
      battery[0].cells[0], battery[0].cells[1], battery[0].cells[2],
      battery[0].cells[3], battery[0].cells[4], battery[0].cells[5], 
      min_cell, max_cell, total_voltage);

    DataPackets::Battery_Temperatures_init(
      temperature[0], temperature[1], 
      temperature[2], temperature[3], 
      min_temperature, max_temperature);

    DataPackets::State_of_Charge_init(SOC);
    DataPackets::Battery_Current_init(current);

    DataPackets::Current_State_init(state);
  }
  DataPackets::start();

  OrderPackets::Brake_init();
  OrderPackets::start();

#if LV_BMS_VERSION_MAJOR == 10
  Scheduler::register_task((READING_PERIOD_US / 2) - 100, []() {
    bms.update();
  });
#endif

  ProtectionManager::link_state_machine(LV_BMS::BMS_State_Machine,
                                        static_cast<uint8_t>(BMS_State::FAULT));
  ProtectionManager::add_standard_protections();
  ProtectionManager::set_id(Boards::ID::BMSA);
  //LV_BMS::add_protections();
  ProtectionManager::initialize();

  //DCLV::init();
}

void LV_BMS::start() {
  last_reading_time = HAL_GetTick();

  Scheduler::register_task(1000*10, [](){
    ProtectionManager::check_protections();
  });

  BMS_State_Machine.start();
  Scheduler::register_task(1000, []() {
    BMS_State prev_state = LV_BMS::state;
    LV_BMS::BMS_State_Machine.check_transitions();
    LV_BMS::state = BMS_State_Machine.get_current_state();
    if(LV_BMS::state != prev_state) [[unlikely]] {
      DataPackets::control_station_udp->send_packet(*DataPackets::Current_State_packet);
    }
  });

  Scheduler::register_task(100'000, []() {
    read();
  });
}

void LV_BMS::set_protection_name(Protection* protection,
                                 const std::string& name) {
  protection->set_name((char*)malloc(name.size() + 1));
  sprintf(protection->get_name(), "%s", name.c_str());
}

void LV_BMS::add_protections() {
  Protection* soc_protection = &ProtectionManager::_add_protection(
    &SOC, Boundary<float, OUT_OF_RANGE>{24.0f, 80.0f});
  set_protection_name(soc_protection, "SOC");
}

//------------------- SOC ------------------------

float LV_BMS::coulomb_counting_SOC(float current) {
  uint32_t current_time = HAL_GetTick();

  float delta_time = (current_time - last_reading_time) / 1000.0f;
  last_reading_time = current_time;

  float delta_SOC = current * delta_time / CAPACITY_AH * 3600.0f;
  return delta_SOC;
}

float LV_BMS::ocv_battery_SOC() {
  float total_voltage = battery[0].cells[0] + battery[0].cells[1] + battery[0].cells[2] +
                        battery[0].cells[3] + battery[0].cells[4] + battery[0].cells[5];
  float x =
    total_voltage - 20.0;  // to get a bigger difference between values so
                           // the polynomial is more accurate
  float result = -62.5 + (14.9 * x) + (21.9 * x * x) + (-4.18 * x * x * x);
  return result;
}

void LV_BMS::update_SOC() {
  /* if (first_soc_flag == true) {
    SOC = ocv_battery_SOC();
    first_soc_flag = false;
  } else {
    if (std::abs(*current) < REST_THRESHOLD) {
      SOC += coulomb_counting_SOC(0 - *current);
    } else { */
      SOC = ocv_battery_SOC();
      if((SOC > -100.0f) && (SOC < 200.0f) && ((SOC < 20.0f) || (SOC > 80.0f))) [[unlikely]] {
        ErrorHandler("Fault: LV battery State of charge not in range [20, 80]");
        BMS_State_Machine.force_change_state(static_cast<size_t>(BMS_State::FAULT));
      }
   /*  }
  } */
}

//------------------------------------------------

void LV_BMS::get_max_min_cells() {
  float maximum = FLT_MIN;
  float minimum = FLT_MIN;
  for(unsigned int i = 0; i < ARRAY_LENGTH(LV_BMS::battery[0].cells); i++) {
    float v = LV_BMS::battery[0].cells[i];
    maximum = std::max(v, maximum);
    minimum = std::min(v, minimum);
  }

  max_cell = maximum;
  min_cell = minimum;
}

void LV_BMS::get_max_min_temperatures() {
  float maximum = FLT_MIN;
  float minimum = FLT_MIN;
  for(unsigned int i = 0; i < ARRAY_LENGTH(LV_BMS::temperature); i++) {
    float v = LV_BMS::temperature[i];
    maximum = std::max(v, maximum);
    minimum = std::min(v, minimum);
  }

  max_temperature = maximum;
  min_temperature = minimum;
}

void LV_BMS::read_temperature(const float voltage, float* temperature) {
  auto resistance =
    (voltage * RESISTANCE_REFERENCE) / (VOLTAGE_REFERENCE - voltage);
  *temperature = (resistance - R0) / (TCR * R0);
}

void LV_BMS::read() {
  get_max_min_cells();
#if LV_BMS_VERSION_MAJOR == 10
  update_SOC();
  current_sensor.read();
  read_temperature(GPIO_voltage_1, &temperature[0]);
  read_temperature(GPIO_voltage_2, &temperature[1]);
  read_temperature(GPIO_voltage_3, &temperature[2]);
  read_temperature(GPIO_voltage_4, &temperature[3]);
#else
  update_SOC();
#endif
  get_max_min_temperatures();
}

#if 0
void LV_BMS::add_protections_old() {
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
