#include "LV-BMS/LV-BMS_Comms.hpp"
#include "LV-BMS/LV-BMS_Data.hpp"
#include "LV-BMS/LV-BMS.hpp"
#include "LV-BMS/DCLV/DCLV.hpp"

// Callbacks
void Comms::turn_on_pfm_callback() {
  received_turn_on_pfm = true;
}
void Comms::turn_off_pfm_callback() {
  received_turn_off_pfm = true;
}
void Comms::set_pfm_frequency_callback() {
  received_set_pfm_frequency = true;
}
void Comms::set_pfm_dead_time_callback() {
  received_set_pfm_dead_time = true;
}

void Comms::init() {

  control_station = new ServerSocket(IPV4(BMSL_IP), CONTROL_STATION_PORT, 2000,1000,20);
  

  control_station_udp = new DatagramSocket(IPV4(BMSL_IP), CONTROL_STATION_UDP_PORT,IPV4(CONTROL_SATION_IP), CONTROL_STATION_UDP_PORT);
  
  add_packets();
  add_orders();
}
