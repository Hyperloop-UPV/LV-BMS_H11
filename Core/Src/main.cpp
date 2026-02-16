#include "main.h"
//#include "lwip.h"

#include "ST-LIB.hpp"

#include "LV-BMS/LV-BMS.hpp"
#include "LV-BMS/LV-BMS_Pinout.hpp"
#include "LV-BMS/LV-BMS_Domains.hpp"

int main(void) {
  Hard_fault_check();
  lvBMS_Board::init();

#ifdef STLIB_ETH
  auto eth_instance = &lvBMS_Board::instance_of<eth>();
#endif

  ST_LIB::TimerWrapper<timer_us_tick_def> global_tick = get_timer_instance(lvBMS_Board, timer_us_tick_def);
  LV_BMS::operational_led = &lvBMS_Board::instance_of<operational_led_def>();
  LV_BMS::fault_led = &lvBMS_Board::instance_of<fault_led_def>();
  LV_BMS::current_sensor = LinearSensor(lvBMS_Board::instance_of<current_adc_def>(), 
  10.236f, -0.581f, &LV_BMS::current);
  LV_BMS::spi_pins = &lvBMS_Board::instance_of<spi_def>();
  LV_BMS::spi_wrapper.emplace(*LV_BMS::spi_pins);
  LV_BMS::spi_cs = &lvBMS_Board::instance_of<spi_cs_def>();

  global_us_timer = global_tick.instance->tim;
  global_tick.set_prescaler(global_tick.get_clock_frequency() / 10'000);
  global_us_timer->ARR = UINT32_MAX;
  global_tick.counter_enable();
  
  LV_BMS::init();
  LV_BMS::start();

  Scheduler::start();

  while (1) {
    Scheduler::update();
#ifdef STLIB_ETH
    eth_instance->update();
#endif
  }
}

void Error_Handler(void) {
  ErrorHandler("HAL error handler triggered");
  while (1) {
  }
}

