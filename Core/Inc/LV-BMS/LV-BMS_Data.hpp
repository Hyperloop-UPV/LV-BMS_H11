#ifndef DATA_HPP
#define DATA_HPP

#include "BMS.hpp"
#include "ST-LIB.hpp"
#include "Communications/Packets/DataPackets.hpp"

#define N_BATTERIES 1
#define READING_PERIOD_US 17000
#define CAPACITY_AH 20.0f
#define REST_THRESHOLD 0.1f

#define RESISTANCE_REFERENCE 3900.0  // Ohm
#define VOLTAGE_REFERENCE 3.0        // V
#define R0 100.0                     // Ohm
#define TCR 0.00385

using BMS_State = DataPackets::State;

class Data {
   public:

   private:
};

#endif // DATA_HPP