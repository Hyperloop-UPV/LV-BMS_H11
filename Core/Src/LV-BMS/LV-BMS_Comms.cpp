#include "LV-BMS/LV-BMS_Comms.hpp"
#include "LV-BMS/LV-BMS_Data.hpp"
#include "LV-BMS/LV-BMS.hpp"
//#include "LV-BMS/DCLV/DCLV.hpp"

void Comms::init() {
  DataPackets::Battery_Voltages_init(
    Data::cells[0], Data::cells[1], Data::cells[2],
    Data::cells[3], Data::cells[4], Data::cells[5], 
    Data::min_cell, Data::max_cell, Data::total_voltage);

  DataPackets::Battery_Temperatures_init(
    Data::temperature[0], Data::temperature[1], 
    Data::temperature[2], Data::temperature[3], 
    Data::min_temperature, Data::max_temperature);

  DataPackets::State_of_Charge_init(Data::SOC);
  DataPackets::Battery_Current_init(Data::current);

  DataPackets::Current_State_init(LV_BMS::state);
}
