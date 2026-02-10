#include "main.h"
//#include "lwip.h"

#include "ST-LIB.hpp"

#include "LV-BMS/LV-BMS.hpp"
#include "LV-BMS/LV-BMS_Pinout.hpp"

using ST_LIB::EthernetDomain;

#ifdef STLIB_ETH
#if defined(USE_PHY_LAN8742)
constexpr auto eth =
    EthernetDomain::Ethernet(EthernetDomain::PINSET_H10, "00:00:00:00:01:FE",
                             "192.168.1.11", "255.255.255.0");
#elif defined(USE_PHY_LAN8700)
constexpr auto eth =
    EthernetDomain::Ethernet(EthernetDomain::PINSET_H10, "00:00:00:00:01:FE",
                             "192.168.1.11", "255.255.255.0");
#elif defined(USE_PHY_KSZ8041)
constexpr auto eth =
    EthernetDomain::Ethernet(EthernetDomain::PINSET_H11, "00:00:00:00:01:FE",
                             "192.168.1.11", "255.255.255.0");
#else
#error "No PHY selected for Ethernet pinset selection"
#endif
#endif // STLIB_ETH

#if 0
ST_LIB::TimerWrapper<timer_us_tick_def> *global_us_timer;
#endif

int main(void) {
#if STLIB_ETH
  using lvBMS_Board = ST_LIB::Board<eth, timer_us_tick_def, operational_led_def, fault_led_def>;
#else
  using lvBMS_Board = ST_LIB::Board<timer_us_tick_def, operational_led_def, fault_led_def>;
#endif
  Hard_fault_check();
  lvBMS_Board::init();

#ifdef STLIB_ETH
  auto eth_instance = &lvBMS_Board::instance_of<eth>();
#endif

  ST_LIB::TimerWrapper<timer_us_tick_def> global_tick = get_timer_instance(lvBMS_Board, timer_us_tick_def);

  LV_BMS<timer_us_tick_def> lvbms;
  lvbms.set_global_us_timer(&global_tick);

  lvbms.operational_led = &lvBMS_Board::instance_of<operational_led_def>();
  lvbms.fault_led = &lvBMS_Board::instance_of<fault_led_def>();

 /*  if(lvbms.n == 11) {
    ErrorHandler("!!!");
  } */

  lvbms.init();
  //STLIB::start();
  lvbms.start();

  Scheduler::start();

  while (1) {
    //STLIB::update();
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

