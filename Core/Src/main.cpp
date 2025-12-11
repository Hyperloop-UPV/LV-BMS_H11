#include "main.h"
#include "ST-LIB.hpp"
int main(void) {
#ifdef SIM_ON
    SharedMemory::start();
#endif
    Hard_fault_check();
    volatile uint32_t *bad = (uint32_t*)0xFFFFFFFF;
    [[maybe_unused]]uint32_t x = *bad;   // dirección ilegal
    STLIB::start();
    while (1) {
        STLIB::update();
    }
}

void Error_Handler(void) {
    ErrorHandler("HAL error handler triggered");
    while (1) {
    }
}
