#include "main.h"
//#include "lwip.h"

#include "ST-LIB.hpp"

#include "LV-BMS/LV-BMS_Pinout.hpp"
#include "LV-BMS/LV-BMS_Domains.hpp"

#if LV_BMS_VERSION_MAJOR == 11
#include "LV-BMS/BCC_ST-LIB/bcc_stlib.h"
#endif

TIM_TypeDef *global_tick_timer;
extern ST_LIB::SPIDomain::SPIWrapper<spi_def> *spi_wrapper;

int main(void) {
  Hard_fault_check();

  lvBMS_Board::init();

#ifdef STLIB_ETH
  auto eth_instance = &lvBMS_Board::instance_of<eth>();
#endif

  // setup global tick timer
  auto global_tick_timer_wrapper = get_timer_instance(lvBMS_Board, timer_us_tick_def);
  static_assert(global_tick_timer_wrapper.is_32bit_instance);
  {
    uint16_t tick_psc = (uint16_t)(global_tick_timer_wrapper.get_clock_frequency()/1'000'000);
    global_tick_timer_wrapper.set_prescaler(tick_psc - 1);
  }
  global_tick_timer = global_tick_timer_wrapper.instance->tim;
  global_tick_timer->ARR = UINT32_MAX;
  global_tick_timer_wrapper.counter_enable();

#ifdef SCHEDULER_GET_LAST_N_TASKS
  UART::Peripheral *uart = &UART::uart1;
  if(!Scheduler::init_perf(global_tick_timer, uart)) {
    FAULT("Could not init perf for scheduler");
  }
#else
  UART::Peripheral *uart = 0;
#endif

  uart = &UART::uart1;
  auto uart_id = UART::inscribe(*uart);
  UART::start();

  char message[] = "UART Message!!!!";
  if(!UART::transmit_polling(
    uart_id,
    (uint8_t*)&message[0],
    sizeof(message)
  )) {
    WARNING("UART Error while trying to transmit timing info");
  }

#if LV_BMS_VERSION_MAJOR == 11
  // setup timeout timer
  auto timeout_timer_wrapper = get_timer_instance(lvBMS_Board, timeout_timer_def);
  static_assert(timeout_timer_wrapper.is_32bit_instance);
  ST_LIB::TimerDomain::callbacks[timeout_timer_wrapper.instance->timer_idx] = timeout_timer_callback;
  timeout_timer_wrapper.set_prescaler(timeout_timer_wrapper.get_clock_frequency()/1'000'000);
#endif

  LV_BMS::operational_led = &lvBMS_Board::instance_of<operational_led_def>();
  LV_BMS::fault_led = &lvBMS_Board::instance_of<fault_led_def>();
#if LV_BMS_VERSION_MAJOR == 10
  LV_BMS::current_sensor = 
    LinearSensor(lvBMS_Board::instance_of<current_adc_def>(), 
                 10.236f, -0.581f, &LV_BMS::current);
#endif
  LV_BMS::spi_pins = &lvBMS_Board::instance_of<spi_def>();
  ST_LIB::SPIDomain::SPIWrapper<spi_def> spi_wrapper_internal(*LV_BMS::spi_pins);
  spi_wrapper = &spi_wrapper_internal;
  spi_cs = &lvBMS_Board::instance_of<spi_cs_def>();
  bms_rst = &lvBMS_Board::instance_of<bms_rst_def>();
  bms_rst->turn_on();

  LV_BMS::init();
  LV_BMS::start();

  while (1) {
    Scheduler::update();
    lvBMS_Board::evaluate_protections();
    Diagnostics::Hub::flush();
#ifdef STLIB_ETH
    eth_instance->update();
#endif
  }
}

void Error_Handler(void) {
  FAULT("HAL error handler triggered");
  while (1) {
  }
}

