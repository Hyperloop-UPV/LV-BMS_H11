#ifndef DATA_HPP
#define DATA_HPP

#include "BMS.hpp"
#include "ST-LIB.hpp"

#define N_BATTERIES 1
#define READING_PERIOD_US 17000
#define CAPACITY_AH 20.0f
#define REST_THRESHOLD 0.1f

#define RESISTANCE_REFERENCE 3900.0  // Ohm
#define VOLTAGE_REFERENCE 3.0        // V
#define R0 100.0                     // Ohm
#define TCR 0.00385

class Data {
    struct BMSConfig {
        static inline uint8_t spi_id{};
        static constexpr size_t n_LTC6810{1};
        static void SPI_transmit(const std::span<uint8_t> data);
        static void SPI_receive(std::span<uint8_t> buffer);
        static void SPI_CS_turn_on(void);
        static void SPI_CS_turn_off(void);
        static int32_t get_tick(void);
        static constexpr int32_t tick_resolution_us{500};
        static constexpr int32_t period_us{READING_PERIOD_US};
        static constexpr int32_t conv_rate_time_ms{100};
    };

    static constexpr BMS<BMSConfig> bms{};
    static inline auto& battery = bms.get_data();

    static void get_max_min_cells();
    static void get_max_min_temperatures();

   public:
    static inline std::array<std::reference_wrapper<float>, 6> cells{
        std::ref(battery[0].cells[0]), std::ref(battery[0].cells[1]),
        std::ref(battery[0].cells[2]), std::ref(battery[0].cells[3]),
        std::ref(battery[0].cells[4]), std::ref(battery[0].cells[5])};

    static inline float max_cell{};
    static inline float min_cell{};

    static inline float& total_voltage{battery[0].total_voltage};
    static inline float& GPIO_voltage_1{battery[0].GPIOs[0]};
    static inline float& GPIO_voltage_2{battery[0].GPIOs[1]};
    static inline float& GPIO_voltage_3{battery[0].GPIOs[2]};
    static inline float& GPIO_voltage_4{battery[0].GPIOs[3]};
    static inline std::array<float, 4> temperature{};
    static inline float& conv_rate{battery[0].conv_rate};

    static inline float max_temp{};
    static inline float min_temp{};

    static inline float SOC{50.0f};

    static void init();
    static void start();
    static void read();
    static void read_temperature(const float voltage, float* temperature);

    static void update_SOC();
    static float ocv_battery_SOC();
    static float coulomb_counting_SOC(float current);
    static inline uint32_t last_reading_time{};
    static inline bool first_soc_flag{true};

    static inline LinearSensor<float>* current_sensor{};
    static inline float* current{};
    static inline DigitalOutput* LED_Operational{};
    static inline DigitalOutput* LED_Fault{};

    static inline int32_t us_counter{};

   private:
};

#endif // DATA_HPP