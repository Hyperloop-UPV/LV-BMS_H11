#include "main.h"
//#include "lwip.h"

#include "ST-LIB.hpp"

#include "LV-BMS/LV-BMS.hpp"

int main(void) { 
  Hard_fault_check();
  using lvBMS_Board = ST_LIB::Board<>;
  lvBMS_Board::init();

  LV_BMS::init();
  STLIB::start("00:00:00:00:01:FE", "192.168.1.254", "255.255.255.0");
  LV_BMS::start();

  Scheduler::start();

  while (1) {
    STLIB::update();
    Scheduler::update();
  }
}

void Error_Handler(void) {
  ErrorHandler("HAL error handler triggered");
  while (1) {
  }
}

