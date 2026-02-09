#include "main.h"
//#include "lwip.h"

#include "ST-LIB.hpp"

#include "LV-BMS/LV-BMS.hpp"
#include "LV-BMS/LV-BMS_Pinout.hpp"

using ST_LIB::EthernetDomain;

#ifdef STLIB_ETH
#if defined(USE_PHY_LAN8742)
constexpr auto eth =
    EthernetDomain::Ethernet(EthernetDomain::PINSET_H10, "00:80:e1:00:01:07",
                             "192.168.1.7", "255.255.0.0");
#elif defined(USE_PHY_LAN8700)
constexpr auto eth =
    EthernetDomain::Ethernet(EthernetDomain::PINSET_H10, "00:80:e1:00:01:07",
                             "192.168.1.7", "255.255.0.0");
#elif defined(USE_PHY_KSZ8041)
constexpr auto eth =
    EthernetDomain::Ethernet(EthernetDomain::PINSET_H11, "00:80:e1:00:01:07",
                             "192.168.1.7", "255.255.0.0");
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

  ST_LIB::TimerWrapper<timer_us_tick_def> global_us_timer_decl = get_timer_instance(lvBMS_Board, timer_us_tick_def);
#if 1
  *(GetGlobalUsTimer()) = &global_us_timer_decl;
#else
  global_us_timer = &global_us_timer_decl;
#endif

  LV_BMS::operational_led = &lvBMS_Board::instance_of<operational_led_def>();
  LV_BMS::fault_led = &lvBMS_Board::instance_of<fault_led_def>();

  LV_BMS::init();
  STLIB::start("00:00:00:00:01:FE", "192.168.1.254", "255.255.255.0");
  LV_BMS::start();

  Scheduler::start();

  while (1) {
    STLIB::update();
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

