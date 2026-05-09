#ifndef LV_BMS_STATE_MACHINE_HPP
#define LV_BMS_STATE_MACHINE_HPP

#include "ST-LIB.hpp"

#include "Communications/Packets/DataPackets.hpp"
#include "Communications/Packets/OrderPackets.hpp"

using BMS_State = DataPackets::State;

extern void ConnectingState_Update(void);

extern void OperationalState_OnEnter(void);
extern void OperationalState_OnExit(void);

extern void FaultState_OnEnter(void);
// NOTE: This is to be implemented, there is no fault recovery yet!
//extern void FaultState_OnExit(void);

struct LV_BMS_SM {
  /*-----State Machine declaration------*/
  static constexpr auto connecting_state = make_state(BMS_State::CONNECTING,
    Transition<BMS_State>{BMS_State::OPERATIONAL,
      []() {
        return OrderPackets::control_station_tcp->is_connected();
      }
    }
  );

  static constexpr auto operational_state = make_state(BMS_State::OPERATIONAL,
    Transition<BMS_State>{BMS_State::FAULT,
      []() {
        return !OrderPackets::control_station_tcp->is_connected();
      }
    }
  );

  static constexpr auto fault_state = make_state(BMS_State::FAULT);

  static inline constinit auto State_Machine = []() consteval
  {
    auto sm = make_state_machine(BMS_State::CONNECTING,
      connecting_state,
      operational_state,
      fault_state
    );

    // BMS_State::CONNECTING
    sm.add_cyclic_action(ConnectingState_Update, LV_BMS_CONNECTING_UPDATE_FREQ, 
                         connecting_state);

    // BMS_State::OPERATIONAL
    sm.add_enter_action(OperationalState_OnEnter, operational_state);
    sm.add_exit_action(OperationalState_OnExit, operational_state);

    return sm;
  }();
};

#endif // LV_BMS_STATE_MACHINE_HPP
