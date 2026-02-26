#ifndef LV_BMS_HPP
#define LV_BMS_HPP

#include "ST-LIB.hpp"

#include "LV-BMS_Data.hpp"
#include "LV-BMS_Pinout.hpp"
#include "LV-BMS_Domains.hpp"

#include "BMS.hpp"
//#include "DCLV/DCLV.hpp"

#include "Communications/Packets/DataPackets.hpp"
#include "Communications/Packets/OrderPackets.hpp"

using BMS_State = DataPackets::State;

extern TIM_TypeDef *global_us_timer;

#define GetMicroseconds() global_us_timer->CNT

struct LV_BMS {
  static inline ST_LIB::DigitalOutputDomain::Instance *operational_led;
  static inline ST_LIB::DigitalOutputDomain::Instance *fault_led;
  static inline ST_LIB::SPIDomain::Instance *spi_pins;
  static inline ST_LIB::DigitalOutputDomain::Instance *spi_cs;
  static inline std::optional<ST_LIB::SPIDomain::SPIWrapper<spi_def>> spi_wrapper;

  static inline BMS_State state{};

  //--------------- BMS CONFIG FOR LTC6810-DRIVER ----------------
  struct BMSConfig {
    static inline uint8_t spi_id{};
    static constexpr size_t n_LTC6810{1};
    static void SPI_transmit(const std::span<uint8_t> data);
    static void SPI_receive(std::span<uint8_t> buffer);
    static void SPI_CS_turn_on(void);
    static void SPI_CS_turn_off(void);
    static int32_t get_tick(void);
    static constexpr int32_t tick_resolution_us{100};
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

  static inline float current{};
  static inline LinearSensor<float> current_sensor{};

  static inline float &total_voltage{battery[0].total_voltage};
  static inline float &GPIO_voltage_1{battery[0].GPIOs[0]};
  static inline float &GPIO_voltage_2{battery[0].GPIOs[1]};
  static inline float &GPIO_voltage_3{battery[0].GPIOs[2]};
  static inline float &GPIO_voltage_4{battery[0].GPIOs[3]};

  static inline float &conv_rate{battery[0].conv_rate};

  //////////////////////////////////////

  static void set_protection_name(Protection *protection, const std::string &name);
  static void init();
  static void start();
  static void add_protections();

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
        return OrderPackets::control_station_tcp->is_connected();
      }
    }
  );

  static constexpr auto operational_state = make_state(BMS_State::OPERATIONAL,
    Transition<BMS_State>{BMS_State::FAULT,
      []() {
        return !OrderPackets::control_station_tcp->is_connected();
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

    sm.add_cyclic_action([]() {
      LV_BMS::operational_led->toggle();
    }, std::chrono::milliseconds(300), connecting_state);

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

#endif // LV_BMS_HPP