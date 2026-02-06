#include "LV-BMS/DCLV/DCLV.hpp"

#include "LV-BMS/LV-BMS_Pinout.hpp"
#include "LV-BMS/LV-BMS_Data.hpp"

void DCLV::init() {
  pfm = new DualPWM(PWM_HIGH, PWM_LOW);
  pfm_state = PFM_STATES::INACTIVE;
  buffer_en = new DigitalOutput(PWM_BUFFER_EN);  // LOW ACTIVE
  reset = new DigitalOutput(PWM_RESET);

  output_current = new float;
  input_current = new float;
  output_voltage = new float;
  input_voltage = new float;

  output_current_sensor = new LinearSensor<float>(
    DCLV_OUTPUT_CURRENT_SENSOR, 10.266f, -0.602f, output_current);
  input_current_sensor = new LinearSensor<float>(DCLV_INPUT_CURRENT_SENSOR,
                                                 0.0f, 0.0f, input_current);
  output_voltage_sensor = new LinearSensor<float>(
    DCLV_OUTPUT_VOLTAGE_SENSOR, 9.46f, -0.529f, output_voltage);
  input_voltage_sensor = new LinearSensor<float>(
    DCLV_INPUT_VOLTAGE_SENSOR, 159.614f, -10.155f, input_voltage);

  pfm->set_duty_cycle(duty);
  pfm->set_frequency(frequency);
  pfm->set_dead_time(std::chrono::nanoseconds(dead_time));

  buffer_en->turn_on();
  buffer_state = BUFFER_STATES::HIGH;
}

void DCLV::turn_on_pfm() {
  buffer_en->turn_off();
  buffer_state = BUFFER_STATES::LOW;
  reset->turn_on();
  reset_state = RESET_STATES::HIGH;
  pfm->turn_on();
  pfm_state = PFM_STATES::ACTIVE;
}

void DCLV::turn_off_pfm() {
  buffer_en->turn_on();
  buffer_state = BUFFER_STATES::HIGH;
  reset->turn_off();
  reset_state = RESET_STATES::LOW;
  pfm->turn_off();
  pfm_state = PFM_STATES::INACTIVE;
}

void DCLV::set_pfm_frequency(uint32_t frequency) {
  pfm->set_frequency(frequency);
}

void DCLV::set_pfm_dead_time(uint32_t dead_time) {
  pfm->set_dead_time(std::chrono::nanoseconds(dead_time));
}

void DCLV::read_sensors() {
  output_current_sensor->read();
  input_current_sensor->read();
  output_voltage_sensor->read();
  input_voltage_sensor->read();
}