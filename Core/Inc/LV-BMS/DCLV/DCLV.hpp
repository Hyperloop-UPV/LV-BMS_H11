#ifndef DCLV_HPP
#define DCLV_HPP

#include "ST-LIB.hpp"

struct DCLV {
    enum class BUFFER_STATES : uint8_t{
        HIGH,
        LOW
    };

    enum class RESET_STATES : uint8_t{
        LOW,
        HIGH
    };

    enum class PFM_STATES : uint8_t{
        INACTIVE,
        ACTIVE
    };
        
    static inline DualPWM* pfm{};
    static inline PFM_STATES pfm_state{};
    static inline DigitalOutput* buffer_en;
    static inline BUFFER_STATES buffer_state;
    static inline DigitalOutput* reset;
    static inline RESET_STATES reset_state;

    constexpr static float duty{50.0f};
    static inline uint32_t frequency{};
    static inline uint32_t dead_time{100};

    static inline LinearSensor<float>* output_current_sensor;
    static inline LinearSensor<float>* input_current_sensor;
    static inline LinearSensor<float>* output_voltage_sensor;
    static inline LinearSensor <float>* input_voltage_sensor;
    static inline float* output_current;
    static inline float* input_current;
    static inline float* output_voltage;
    static inline float* input_voltage;

    static void init();
    static void turn_on_pfm();
    static void turn_off_pfm();
    static void set_pfm_frequency(uint32_t freq);
    static void set_pfm_dead_time(uint32_t dt);
    static void read_sensors();
};

#endif