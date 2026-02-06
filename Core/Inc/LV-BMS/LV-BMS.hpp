#ifndef LV_BMS_HPP
#define LV_BMS_HPP

#include "ST-LIB.hpp"

#include "LV-BMS_Data.hpp"
#include "LV-BMS_Comms.hpp"

#include "BMS.hpp"
#include "DCLV/DCLV.hpp"

#include "Communications/Packets/DataPackets.hpp"
#include "Communications/Packets/OrderPackets.hpp"

using BMS_State = DataPackets::State;

class LV_BMS {
  static void set_protection_name(Protection *protection, const std::string &name);

public:
  static BMS_State &state;

  static void init();
  static void start();
  static void add_protections();
  static void update();

  /*-----State Machine declaration------*/
  static constexpr auto connecting_state = make_state(BMS_State::CONNECTING,
    Transition<BMS_State>{BMS_State::OPERATIONAL,
      []() {
        return DataPackets::control_station_tcp->is_connected();
      }
    }
  );

  static constexpr auto operational_state = make_state(BMS_State::OPERATIONAL,
    Transition<BMS_State>{BMS_State::FAULT,
      []() {
        return !DataPackets::control_station_tcp->is_connected();
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
      Data::LED_Operational->turn_on();
    }, operational_state);

    sm.add_exit_action([]() {
      Data::LED_Operational->turn_off();
    }, operational_state);


    sm.add_enter_action([]() {
      Data::LED_Fault->turn_on();
    }, fault_state);

    sm.add_enter_action([]() {
      Data::LED_Fault->turn_off();
    }, fault_state);

    //////////////////////////////////////////////
    // Cyclic actions

    // BMS_State::CONNECTING
    sm.add_cyclic_action([]() {
      Data::read();
    }, std::chrono::milliseconds(100), connecting_state);

    sm.add_cyclic_action([]() {
      DCLV::read_sensors();
    }, std::chrono::milliseconds(100), connecting_state);

    // BMS_State::OPERATIONAL
    sm.add_cyclic_action([]() {
      Data::read();
    }, std::chrono::milliseconds(100), operational_state);

    sm.add_cyclic_action([]() {
      DCLV::read_sensors();
    }, std::chrono::milliseconds(100), operational_state);

    sm.add_cyclic_action([]() {
      ProtectionManager::check_protections();
    }, std::chrono::milliseconds(100), operational_state);

    // BMS_State::FAULT
    sm.add_cyclic_action([]() {
      Data::read();
    }, std::chrono::milliseconds(100), fault_state);

    sm.add_cyclic_action([]() {
      DCLV::read_sensors();
    }, std::chrono::milliseconds(100), fault_state);

    sm.add_cyclic_action([]() {
      ProtectionManager::check_protections();
    }, std::chrono::milliseconds(100), fault_state);

    return sm;
  }();
};

#endif // LV_BMS_HPP