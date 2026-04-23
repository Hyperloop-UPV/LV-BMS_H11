#include "main.h"
//#include "lwip.h"

#include "ST-LIB.hpp"

#include "LV-BMS/LV-BMS.hpp"
#include "LV-BMS/LV-BMS_Pinout.hpp"
#include "LV-BMS/LV-BMS_Domains.hpp"

TIM_TypeDef *global_tick_timer;

#if 0
// Used to test LVBMS-H11 board
enum class Mode { Input = 0, Output = 1, Alt = 2, Analog = 3 };
enum class Pull { None = 0, Up = 1, Down = 2 };
enum class Speed { Low = 0, Medium = 1, High = 2, VeryHigh = 3 };

template<uintptr_t gpio_addr, char portletter, uint8_t pin>
struct gpio_port_pin {
#define gpio ((GPIO_TypeDef*)gpio_addr)
    template<Mode mode, Pull pull = Pull::None, Speed speed = Speed::Low>
    static void Init(void)
    {
        static_assert(portletter >= 'A' && portletter <= 'H'); /* for our chip */
        static_assert(pin <= 16); /* for our chip */
        RCC->AHB4ENR |= (1 << (portletter - 'A'));
        gpio->MODER &= ~(GPIO_MODER_MODER0 << (pin << 1));
        gpio->MODER |= (int)mode << (pin << 1);
        gpio->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (pin << 1));
        gpio->PUPDR |= (int)pull << (pin << 1);
        if constexpr (mode == Mode::Output || mode == Mode::Alt) {
            gpio->OTYPER &= ~(GPIO_OTYPER_OT0 << pin);
            gpio->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0 << (pin << 1));
            gpio->OSPEEDR |= (int)speed << (pin << 1);
        }
    }

    static void TurnOn(void)  { gpio->BSRR = GPIO_BSRR_BS0 << pin; }
    static void TurnOff(void) { gpio->BSRR = GPIO_BSRR_BR0 << pin; }
#undef gpio

    static void Toggle(void) {
        if (active) {
            TurnOff();
            active = false;
        } else {
            TurnOn();
            active = true;
        }
    }

    static inline bool active{false};
};

using led1 = gpio_port_pin<GPIOG_BASE, 'G', 12>;
#endif

int main(void) {
  Hard_fault_check();
  //led1::Init<Mode::Output>();
  //led1::TurnOn();

  lvBMS_Board::init();

#ifdef STLIB_ETH
  auto eth_instance = &lvBMS_Board::instance_of<eth>();
#endif

  // setup global tick timer
  auto global_tick_timer_wrapper = get_timer_instance(lvBMS_Board, timer_us_tick_def);
  static_assert(global_tick_timer_wrapper.is_32bit_instance);
  {
    uint16_t tick_psc = (uint16_t)(global_tick_timer_wrapper.get_clock_frequency()/1'000'000);
    global_tick_timer_wrapper.set_prescaler(tick_psc - 1);
  }
  global_tick_timer = global_tick_timer_wrapper.instance->tim;
  global_tick_timer->ARR = UINT32_MAX;
  global_tick_timer_wrapper.counter_enable();

#if LV_BMS_VERSION_MAJOR == 11
  // setup timeout timer
  auto timeout_timer_wrapper = get_timer_instance(lvBMS_Board, timeout_timer_def);
  static_assert(timeout_timer_wrapper.is_32bit_instance);
  ST_LIB::TimerDomain::callbacks[timeout_timer_wrapper.instance->timer_idx] = timeout_timer_callback;
  timeout_timer_wrapper.set_prescaler(timeout_timer_wrapper.get_clock_frequency()/1'000'000);
#endif

  LV_BMS::operational_led = &lvBMS_Board::instance_of<operational_led_def>();
  LV_BMS::fault_led = &lvBMS_Board::instance_of<fault_led_def>();
#if LV_BMS_VERSION_MAJOR == 10
  LV_BMS::current_sensor = 
    LinearSensor(lvBMS_Board::instance_of<current_adc_def>(), 
                 10.236f, -0.581f, &LV_BMS::current);
#endif
  LV_BMS::spi_pins = &lvBMS_Board::instance_of<spi_def>();
  ST_LIB::SPIDomain::SPIWrapper<spi_def> spi_wrapper_internal(*LV_BMS::spi_pins);
  spi_wrapper = &spi_wrapper_internal;
  spi_cs = &lvBMS_Board::instance_of<spi_cs_def>();

  LV_BMS::init();
  LV_BMS::start();

  while (1) {
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

