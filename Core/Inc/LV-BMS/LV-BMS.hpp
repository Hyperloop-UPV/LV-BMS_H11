#ifndef LV_BMS_HPP
#define LV_BMS_HPP

#include "ST-LIB.hpp"

#include "LV-BMS_Data.hpp"
#include "LV-BMS_Comms.hpp"
#include "LV-BMS_Pinout.hpp"

#include "BMS.hpp"
//#include "DCLV/DCLV.hpp"

#include "Communications/Packets/DataPackets.hpp"
//#include "Communications/Packets/OrderPackets.hpp"

using BMS_State = DataPackets::State;

inline constexpr ST_LIB::TimerDomain::Timer timer_us_tick_def{{
  .request = ST_LIB::TimerRequest::GeneralPurpose32bit_5,
}};
inline constexpr ST_LIB::DigitalOutputDomain::DigitalOutput operational_led_def{ST_LIB::LED_OPERATIONAL};
inline constexpr ST_LIB::DigitalOutputDomain::DigitalOutput fault_led_def{ST_LIB::LED_FAULT};

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
struct LV_BMS {
  static inline ST_LIB::TimerWrapper<global_tim_t> *global_us_timer;
  static inline ST_LIB::DigitalOutputDomain::Instance *operational_led;
  static inline ST_LIB::DigitalOutputDomain::Instance *fault_led;

  static inline BMS_State state{};

  struct BMSConfig {
    static inline uint8_t spi_id{};
    static constexpr size_t n_LTC6810{1};
    static void SPI_transmit(const std::span<uint8_t> data);
    static void SPI_receive(std::span<uint8_t> buffer);
    static void SPI_CS_turn_on(void);
    static void SPI_CS_turn_off(void);
    static int32_t get_tick(void);
    static constexpr int32_t tick_resolution_us{1};
    static constexpr int32_t period_us{READING_PERIOD_US};
    static constexpr int32_t conv_rate_time_ms{100};
  };
  static constexpr BMS<BMSConfig> bms{};
  static inline auto &battery = bms.get_data();

  static inline float max_cell{};
  static inline float min_cell{};

  static inline std::array<float, 4> temperature{};

  static inline float max_temperature{};
  static inline float min_temperature{};

  static inline float SOC{50.0f};

  static inline uint32_t last_reading_time{};
  static inline bool first_soc_flag{true};

  static inline LinearSensor<float> *current_sensor{};
  static inline float current{};

  static inline float &total_voltage{battery[0].total_voltage};
  static inline float &GPIO_voltage_1{battery[0].GPIOs[0]};
  static inline float &GPIO_voltage_2{battery[0].GPIOs[1]};
  static inline float &GPIO_voltage_3{battery[0].GPIOs[2]};
  static inline float &GPIO_voltage_4{battery[0].GPIOs[3]};

  static inline float &conv_rate{battery[0].conv_rate};

  //////////////////////////////////////

  static void set_global_us_timer(ST_LIB::TimerWrapper<timer_us_tick_def> *tim) {
    global_us_timer = tim;
  }

  static uint32_t GetMicroseconds() {
    return global_us_timer->instance->tim->CNT;
  }

  static void set_protection_name(Protection *protection, const std::string &name);
  static void init();
  static void start();
  static void add_protections();
  static void update();

  static float coulomb_counting_SOC(float current);
  static float ocv_battery_SOC();
  static void update_SOC();

  static void get_max_min_cells();
  static void get_max_min_temperatures();

  static void read();
  static void read_temperature(const float voltage, float* temperature);


  /*-----State Machine declaration------*/
  static constexpr auto connecting_state = make_state(BMS_State::CONNECTING,
    Transition<BMS_State>{BMS_State::OPERATIONAL,
      []() {
        return DataPackets::control_station_tcp->is_connected();
      }
    }
  );

  static constexpr auto operational_state = make_state(BMS_State::OPERATIONAL,
    Transition<BMS_State>{BMS_State::FAULT,
      []() {
        return !DataPackets::control_station_tcp->is_connected();
      }
    }
  );

  static constexpr auto fault_state = make_state(BMS_State::FAULT);

  static inline constinit auto BMS_State_Machine = []() consteval
  {
    auto sm = make_state_machine(BMS_State::CONNECTING,
      connecting_state,
      operational_state,
      fault_state
    );

    //////////////////////////////////////////////
    // Enter/Exit actions

    sm.add_enter_action([]() {
      LV_BMS::operational_led->turn_on();
    }, operational_state);

    sm.add_exit_action([]() {
      LV_BMS::operational_led->turn_off();
    }, operational_state);


    sm.add_enter_action([]() {
      LV_BMS::fault_led->turn_on();
    }, fault_state);

    sm.add_enter_action([]() {
      LV_BMS::fault_led->turn_off();
    }, fault_state);

    //////////////////////////////////////////////
    // Cyclic actions

    // BMS_State::CONNECTING
    sm.add_cyclic_action([]() {
      read();
    }, std::chrono::milliseconds(100), connecting_state);

    //sm.add_cyclic_action([]() {
    //  DCLV::read_sensors();
    //}, std::chrono::milliseconds(100), connecting_state);

    // BMS_State::OPERATIONAL
    sm.add_cyclic_action([]() {
      read();
    }, std::chrono::milliseconds(100), operational_state);

    //sm.add_cyclic_action([]() {
    //  DCLV::read_sensors();
    //}, std::chrono::milliseconds(100), operational_state);

    sm.add_cyclic_action([]() {
      ProtectionManager::check_protections();
    }, std::chrono::milliseconds(100), operational_state);

    // BMS_State::FAULT
    sm.add_cyclic_action([]() {
      read();
    }, std::chrono::milliseconds(100), fault_state);

    //sm.add_cyclic_action([]() {
    //  DCLV::read_sensors();
    //}, std::chrono::milliseconds(100), fault_state);

    sm.add_cyclic_action([]() {
      ProtectionManager::check_protections();
    }, std::chrono::milliseconds(100), fault_state);

    return sm;
  }();
}; // struct LV_BMS

//ST_LIB::TimerWrapper<timer_us_tick_def> *LV_BMS::global_us_timer=nullptr;

//--------------- BMS CONFIG FOR LTC6810-DRIVER ----------------

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
void LV_BMS<global_tim_t>::BMSConfig::SPI_transmit(const span<uint8_t> data) {
  SPI::Instance* spi = SPI::registered_spi[spi_id];
  HAL_SPI_Transmit(spi->hspi, data.data(), data.size(), 10);
}

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
void LV_BMS<global_tim_t>::BMSConfig::SPI_receive(span<uint8_t> buffer) {
  SPI::Instance* spi = SPI::registered_spi[spi_id];
  HAL_SPI_Receive(spi->hspi, buffer.data(), buffer.size(), 10);
}

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
void LV_BMS<global_tim_t>::BMSConfig::SPI_CS_turn_off() {
  SPI::Instance* spi = SPI::registered_spi[spi_id];
  SPI::turn_off_chip_select(spi);
}

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
void LV_BMS<global_tim_t>::BMSConfig::SPI_CS_turn_on() {
  SPI::Instance* spi = SPI::registered_spi[spi_id];
  SPI::turn_on_chip_select(spi);
}

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
int32_t LV_BMS<global_tim_t>::BMSConfig::get_tick() {
  return GetMicroseconds();
}

//---------------------------------------------------------------

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
void LV_BMS<global_tim_t>::init() {
#if 0
  ProtectionManager::link_state_machine(&LV_BMS::BMS_State_Machine,
                                        static_cast<uint8_t>(BMS_State::FAULT));
#endif

  ProtectionManager::add_standard_protections();
  //add_protections();
  ProtectionManager::initialize();
  ProtectionManager::set_id(Boards::ID::BMSA);

  global_us_timer->set_prescaler(global_us_timer->get_clock_frequency() / 1000'000);
  global_us_timer->instance->tim->ARR = UINT32_MAX;
  global_us_timer->counter_enable();

  current_sensor =
    new LinearSensor<float>(CURRENT_SENSOR, 10.236f, -0.581f, &current);

  BMSConfig::spi_id = SPI::inscribe(SPI::spi3);

  //DCLV::init();
}

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
void LV_BMS<global_tim_t>::start() {
  last_reading_time = HAL_GetTick();
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

  Scheduler::register_task(1000, []() {
    BMS_State_Machine.check_transitions();
  });
  BMS_State_Machine.start();
}

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
void LV_BMS<global_tim_t>::set_protection_name(Protection* protection,
                                 const std::string& name) {
  protection->set_name((char*)malloc(name.size() + 1));
  sprintf(protection->get_name(), "%s", name.c_str());
}

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
void LV_BMS<global_tim_t>::add_protections() {
  Protection* soc_protection = &ProtectionManager::_add_protection(
    &SOC, Boundary<float, OUT_OF_RANGE>{24.0f, 80.0f});
  set_protection_name(soc_protection, "SOC");
}

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
void LV_BMS<global_tim_t>::update() {
    state = BMS_State_Machine.get_current_state();
}

//------------------- SOC ------------------------

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
float LV_BMS<global_tim_t>::coulomb_counting_SOC(float current) {
  uint32_t current_time = HAL_GetTick();

  float delta_time = (current_time - last_reading_time) / 1000.0f;
  last_reading_time = current_time;

  float delta_SOC = current * delta_time / CAPACITY_AH * 3600.0f;
  return delta_SOC;
}

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
float LV_BMS<global_tim_t>::ocv_battery_SOC() {
  float total_voltage = battery[0].cells[0] + battery[0].cells[1] + battery[0].cells[2] +
                        battery[0].cells[3] + battery[0].cells[4] + battery[0].cells[5];
  float x =
    total_voltage - 20.0;  // to get a bigger difference between values so
                           // the polynomial is more accurate
  float result = -62.5 + (14.9 * x) + (21.9 * x * x) + (-4.18 * x * x * x);
  return result;
}

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
void LV_BMS<global_tim_t>::update_SOC() {
  /* if (first_soc_flag == true) {
    SOC = ocv_battery_SOC();
    first_soc_flag = false;
  } else {
    if (std::abs(*current) < REST_THRESHOLD) {
      SOC += coulomb_counting_SOC(0 - *current);
    } else { */
      SOC = ocv_battery_SOC();
   /*  }
  } */
}

//------------------------------------------------

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
void LV_BMS<global_tim_t>::get_max_min_cells() {
  max_cell = *std::max_element(battery[0].cells.begin(), battery[0].cells.end());
  min_cell = *std::min_element(battery[0].cells.begin(), battery[0].cells.end());
}

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
void LV_BMS<global_tim_t>::get_max_min_temperatures() {
  max_temperature = *std::max_element(temperature.begin(), temperature.end());
  min_temperature = *std::min_element(temperature.begin(), temperature.end());
}

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
void LV_BMS<global_tim_t>::read_temperature(const float voltage, float* temperature) {
  auto resistance =
    (voltage * RESISTANCE_REFERENCE) / (VOLTAGE_REFERENCE - voltage);
  *temperature = (resistance - R0) / (TCR * R0);
}

template<const ST_LIB::TimerDomain::Timer &global_tim_t>
void LV_BMS<global_tim_t>::read() {
  bms.update();
  get_max_min_cells();
  current_sensor->read();
  update_SOC();
  read_temperature(GPIO_voltage_1, &temperature[0]);
  read_temperature(GPIO_voltage_2, &temperature[1]);
  read_temperature(GPIO_voltage_3, &temperature[2]);
  read_temperature(GPIO_voltage_4, &temperature[3]);
  get_max_min_temperatures();
}

#endif // LV_BMS_HPP