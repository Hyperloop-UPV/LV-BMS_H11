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

constexpr ST_LIB::TimerDomain::Timer timer_us_tick_def{{
  .request = ST_LIB::TimerRequest::GeneralPurpose32bit_5,
}};
constexpr ST_LIB::DigitalOutputDomain::DigitalOutput operational_led_def{ST_LIB::LED_OPERATIONAL};
constexpr ST_LIB::DigitalOutputDomain::DigitalOutput fault_led_def{ST_LIB::LED_FAULT};

#define GetMicroseconds() LV_BMS::global_us_timer->instance->tim->CNT

//static ST_LIB::TimerWrapper<timer_us_tick_def> *global_us_timer;

namespace LV_BMS {
  [[maybe_unused]] inline ST_LIB::TimerWrapper<timer_us_tick_def> *global_us_timer;
  [[maybe_unused]] inline ST_LIB::DigitalOutputDomain::Instance *operational_led;
  [[maybe_unused]] inline ST_LIB::DigitalOutputDomain::Instance *fault_led;

  inline BMS_State state;


  void set_protection_name(Protection *protection, const std::string &name);
  void init();
  void start();
  void add_protections();
  void update();


  /*-----State Machine declaration------*/
  constexpr auto connecting_state = make_state(BMS_State::CONNECTING,
    Transition<BMS_State>{BMS_State::OPERATIONAL,
      []() {
        return DataPackets::control_station_tcp->is_connected();
      }
    }
  );

  constexpr auto operational_state = make_state(BMS_State::OPERATIONAL,
    Transition<BMS_State>{BMS_State::FAULT,
      []() {
        return !DataPackets::control_station_tcp->is_connected();
      }
    }
  );

  constexpr auto fault_state = make_state(BMS_State::FAULT);

  /*static*/ inline constinit auto BMS_State_Machine = []() consteval
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
      Data::read();
    }, std::chrono::milliseconds(100), connecting_state);

    //sm.add_cyclic_action([]() {
    //  DCLV::read_sensors();
    //}, std::chrono::milliseconds(100), connecting_state);

    // BMS_State::OPERATIONAL
    sm.add_cyclic_action([]() {
      Data::read();
    }, std::chrono::milliseconds(100), operational_state);

    //sm.add_cyclic_action([]() {
    //  DCLV::read_sensors();
    //}, std::chrono::milliseconds(100), operational_state);

    sm.add_cyclic_action([]() {
      ProtectionManager::check_protections();
    }, std::chrono::milliseconds(100), operational_state);

    // BMS_State::FAULT
    sm.add_cyclic_action([]() {
      Data::read();
    }, std::chrono::milliseconds(100), fault_state);

    //sm.add_cyclic_action([]() {
    //  DCLV::read_sensors();
    //}, std::chrono::milliseconds(100), fault_state);

    sm.add_cyclic_action([]() {
      ProtectionManager::check_protections();
    }, std::chrono::milliseconds(100), fault_state);

    return sm;
  }();
}; // namespace LV_BMS

#endif // LV_BMS_HPP