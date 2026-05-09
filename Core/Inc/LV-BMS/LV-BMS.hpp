#ifndef LV_BMS_HPP
#define LV_BMS_HPP

#include "ST-LIB.hpp"

#include "LV-BMS_Data.hpp"
#include "LV-BMS_Pinout.hpp"

#if LV_BMS_VERSION_MAJOR == 10
#include "BMS.hpp"
#elif LV_BMS_VERSION_MAJOR == 11
#include "../../../../deps/BCC_SW_Driver/bcc/bcc.h"
#endif
//#include "DCLV/DCLV.hpp"

#include "Communications/Packets/DataPackets.hpp"
#include "Communications/Packets/OrderPackets.hpp"

using BMS_State = DataPackets::State;

extern TIM_TypeDef* global_tick_timer;
extern ST_LIB::DigitalOutputDomain::Instance *spi_cs;

#define GetMicroseconds() global_tick_timer->CNT

struct LV_BMS {
  static inline ST_LIB::DigitalOutputDomain::Instance *operational_led;
  static inline ST_LIB::DigitalOutputDomain::Instance *fault_led;
  static inline ST_LIB::SPIDomain::Instance *spi_pins;

  static inline BMS_State state{};

  //--------------- BMS CONFIG FOR LTC6810-DRIVER ----------------
#if LV_BMS_VERSION_MAJOR == 10
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
#elif LV_BMS_VERSION_MAJOR == 11
  static inline bcc_drv_config_t bcc_config{};

  struct BatteryData {
    float cells[6];
  };

  static inline float SOC{50.0f};

  static inline float current{};

  static inline float min_cell{};
  static inline float max_cell{};

  static inline float total_voltage{};

  static inline float min_temperature{};
  static inline float max_temperature{};

  static inline BatteryData battery[1]{};
  static inline float temperature[4]{};

  // NOTE: For coulomb counting SOC (do I need this or does the library give me it?)
  static inline uint32_t last_reading_time{};
#endif

  //////////////////////////////////////

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
}; // struct LV_BMS

#endif // LV_BMS_HPP