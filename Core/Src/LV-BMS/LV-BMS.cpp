#define BCC_STLIB_IMPLEMENTATION
#include "LV-BMS/LV-BMS.hpp"
#include "LV-BMS/LV-BMS_StateMachine.hpp"
#include "LV-BMS/LV-BMS_Domains.hpp"

#if LV_BMS_VERSION_MAJOR == 11
#include "LV-BMS/BCC_ST-LIB/bcc_stlib.h"
#endif

#include <float.h>

ST_LIB::DigitalOutputDomain::Instance *spi_cs;
ST_LIB::DigitalOutputDomain::Instance *bms_rst;
ST_LIB::SPIDomain::SPIWrapper<spi_def> *spi_wrapper;

#if LV_BMS_VERSION_MAJOR == 10
void LV_BMS::BMSConfig::SPI_transmit(const span<uint8_t> data) {
  spi_wrapper->send(data);
}
void LV_BMS::BMSConfig::SPI_receive(span<uint8_t> buffer) {
  spi_wrapper->receive(buffer);
}

void LV_BMS::BMSConfig::SPI_CS_turn_off() {
  spi_cs->turn_off();
}
void LV_BMS::BMSConfig::SPI_CS_turn_on() {
  spi_cs->turn_on();
}

int32_t LV_BMS::BMSConfig::get_tick() {
  return GetMicroseconds();
}
#elif LV_BMS_VERSION_MAJOR == 11

static bcc_status_t Init_BCC_Registers()
{
  bcc_status_t status;

  for(uint8_t cid = 1; cid <= LV_BMS::bcc_config.devicesCnt; cid++) {
    for(uint8_t i = 0; i < (uint8_t)ARRAY_LENGTH(bcc_init_regs); i++) {
      if(bcc_init_regs[i].value != bcc_init_regs[i].defaultVal) {
        status = BCC_Reg_Write(&LV_BMS::bcc_config, (bcc_cid_t)cid,
                    bcc_init_regs[i].address, bcc_init_regs[i].value);
        if(status != BCC_STATUS_SUCCESS) {
          return status;
        }
      }
    }
  }

  return BCC_STATUS_SUCCESS;
}

static bcc_status_t Clear_BCC_FaultRegisters()
{
  bcc_status_t status;

  for(uint8_t cid = 1; cid <= LV_BMS::bcc_config.devicesCnt; cid++) {
    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_CELL_OV);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_CELL_UV);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_CB_OPEN);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_CB_SHORT);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_GPIO_STATUS);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_AN_OT_UT);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_GPIO_SHORT);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_COMM);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_FAULT1);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_FAULT2);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }

    status = BCC_Fault_ClearStatus(&LV_BMS::bcc_config, (bcc_cid_t)cid, BCC_FS_FAULT3);
    if(status != BCC_STATUS_SUCCESS) {
      return status;
    }
  }

  return BCC_STATUS_SUCCESS;
}

uint64_t failCountStart = 0;
bcc_status_t failStartStatuses[256];

uint64_t failCountGetRawValues = 0;
bcc_status_t failGetRawValuesStatuses[256];

#define BCC_STATUS_FAIL_REASON_COUNT (BCC_STATUS_TIMEOUT_START + 1)

static void bcc_info_failiures() {
  uint8_t fail_start_reason_count[BCC_STATUS_FAIL_REASON_COUNT] = {};
  uint8_t fail_rawvals_reason_count[BCC_STATUS_FAIL_REASON_COUNT] = {};

  for(uint32_t i = 0; i < ARRAY_LENGTH(failStartStatuses); i++) {
    fail_start_reason_count[failStartStatuses[i]]++;
  }
  for(uint32_t i = 0; i < ARRAY_LENGTH(failGetRawValuesStatuses); i++) {
    fail_rawvals_reason_count[failGetRawValuesStatuses[i]]++;
  }

  static constexpr int BUFSIZE = 2048;
  char buf[BUFSIZE];
  size_t bufidx = 0;

  for(uint8_t i = 0; i < BCC_STATUS_FAIL_REASON_COUNT; i++) {
    const char *str1 = "Failed ";
    size_t len1 = strlen(str1);
    
    const char *str2;
    size_t len2;

    const char *str3 = get_bcc_error_string((bcc_status_t)i);
    size_t len3 = strlen(str3);

    if(fail_start_reason_count[i] > 0) {
      buf[bufidx++] = '\n';

      str2 = " times at start for reason";
      len2 = strlen(str2);

      if(bufidx + len1 + 4 + len2 + len3 > BUFSIZE) {
        break;
      }
      memcpy(&buf[bufidx], str1, len1);
      bufidx += len1;

      char numbuf[3];
      numbuf[2] = '0' + fail_start_reason_count[i] % 10;
      numbuf[1] = '0' + (fail_start_reason_count[i] % 100) / 10;
      numbuf[0] = '0' + fail_start_reason_count[i] / 100;
      if(numbuf[0] > '9') numbuf[0] = '?';
      memcpy(&buf[bufidx], numbuf, 3);
      bufidx += 3;

      memcpy(&buf[bufidx], str2, len2);
      bufidx += len2;

      memcpy(&buf[bufidx], str3, len3);
      bufidx += len3;
    }

    if(fail_rawvals_reason_count[i] > 0) {
      buf[bufidx++] = '\n';

      str2 = " times at getting values for reason";
      len2 = strlen(str2);

      if(bufidx + len1 + 4 + len2 + len3 > BUFSIZE) {
        break;
      }
      memcpy(&buf[bufidx], str1, len1);
      bufidx += len1;

      char numbuf[3];
      numbuf[2] = '0' + fail_rawvals_reason_count[i] % 10;
      numbuf[1] = '0' + (fail_rawvals_reason_count[i] % 100) / 10;
      numbuf[0] = '0' + fail_rawvals_reason_count[i] / 100;
      if(numbuf[0] > '9') numbuf[0] = '?';
      memcpy(&buf[bufidx], numbuf, 3);
      bufidx += 3;

      memcpy(&buf[bufidx], str2, len2);
      bufidx += len2;

      memcpy(&buf[bufidx], str3, len3);
      bufidx += len3;
    }

    buf[bufidx] = '\0';
  }

  if(bufidx > 0) {
    WARNING(&buf[0]);
  }
}

static inline bool bcc_start_measurements() {
  bcc_status_t status = 
    BCC_Meas_StartConversion(&LV_BMS::bcc_config, (bcc_cid_t)1, LV_BMS::avg_count);
  if(status != BCC_STATUS_SUCCESS) {
    failStartStatuses[failCountStart++] = status;
    failCountStart = failCountStart % ARRAY_LENGTH(failStartStatuses);
    // WARNING("Could not start bcc measurements: %s", get_bcc_error_string(status));
    return false;
  }
  return true;
}

static void Init_BCC_Driver()
{
  // turn on rst for wakeup
  BCC_MCU_WriteRstPin(0, true);

  LV_BMS::bcc_config.drvInstance = 0U;
  LV_BMS::bcc_config.commMode = BCC_MODE_SPI;
  LV_BMS::bcc_config.devicesCnt = 1U;
  LV_BMS::bcc_config.device[0] = BCC_DEVICE_MC33772C;
  LV_BMS::bcc_config.cellCnt[0] = 6U;

#if 0
  bcc_status_t status = BCC_STATUS_COM_ECHO;
  int countfail = -1;
  while(status != BCC_STATUS_SUCCESS && status != BCC_STATUS_COM_NULL) {
    __disable_irq();
    status = BCC_Init(&LV_BMS::bcc_config);
    __enable_irq();
    countfail++;
  }

  if(countfail > 0) {
    WARNING("Failed to do bcc init %d times", countfail);
  }
#else
  __disable_irq();
  bcc_status_t status = BCC_Init(&LV_BMS::bcc_config);
  __enable_irq();
  if(status != BCC_STATUS_SUCCESS && status != BCC_STATUS_COM_NULL) {
    FAULT("Could not init BCC: %s", get_bcc_error_string(status));
    return;
  }
#endif

  status = Init_BCC_Registers();
  if(status != BCC_STATUS_SUCCESS) {
    FAULT("Could not init BCC registers: %s", 
                 get_bcc_error_string(status));
    return;
  }

  status = Clear_BCC_FaultRegisters();
  if(status != BCC_STATUS_SUCCESS) {
    FAULT("Could not clear BCC fault registers: %s", 
                 get_bcc_error_string(status));
    return;
  }

  uint64_t guid;
  status = BCC_GUID_Read(&LV_BMS::bcc_config, (bcc_cid_t)1, &guid);
  if(status != BCC_STATUS_SUCCESS) {
    WARNING("Could not read device guid: %s", get_bcc_error_string(status));
    return;
  } else {
    INFO("BCC device guid: %02X:%04X:%04X",
        (uint16_t)((guid >> 32) & 0x001FU),
        (uint16_t)((guid >> 16) & 0xFFFFU),
        (uint16_t)(guid & 0xFFFFU));
  }

  bcc_start_measurements();
}

static void bcc_get_measurements() {
  uint16_t measurements[BCC_MEAS_CNT];
  bcc_status_t status;

  bool completed = false;
  // NOTE: For HV-BMS (@JorgeCanut), probably make a round robin with the bcc_cids
  status = BCC_Meas_IsConverting(&LV_BMS::bcc_config, (bcc_cid_t)1, &completed);
  if(!completed) {
    // Wait until next call
    INFO("bcc is still converting, skipping this measurement");
    return;
  }

  status = BCC_Meas_GetRawValues(&LV_BMS::bcc_config, (bcc_cid_t)1, measurements);
  if(status != BCC_STATUS_SUCCESS) {
    failGetRawValuesStatuses[failCountGetRawValues++] = status;
    failCountGetRawValues = failCountGetRawValues % ARRAY_LENGTH(failGetRawValuesStatuses);
    // WARNING("Could not get bcc measurements: %s", get_bcc_error_string(status));
    return;
  }

  /* Content of CC registers (raw values only).
   * CC registers resets on read. */
  LV_BMS::battery[0].coulomb_counter = 
    BCC_GET_COULOMB_CNT(measurements[BCC_MSR_COULOMB_CNT1],
                        measurements[BCC_MSR_COULOMB_CNT2]);

#if 0
  // NOTE: We don't use isense for LV-BMS ?????
  int32_t isense_microvolts =
    BCC_GET_ISENSE_VOLT(measurements[BCC_MSR_ISENSE1],
                        measurements[BCC_MSR_ISENSE2]);
  int32_t isense_miliamps =
    BCC_GET_ISENSE_AMP(LV_BMS_RSHUNT_RESISTANCE, measurements[BCC_MSR_ISENSE1],
                       measurements[BCC_MSR_ISENSE2]);

  (void)isense_microvolts;
  (void)isense_miliamps;
#endif

  /* In micro volts */
  LV_BMS::battery[0].stack_voltage = (float)BCC_GET_STACK_VOLT(measurements[BCC_MSR_STACK_VOLT]) / 1'000'000.0f;

  LV_BMS::battery[0].cell_voltage[0] = (float)BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT1]) / 1'000'000.0f;
  LV_BMS::battery[0].cell_voltage[1] = (float)BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT2]) / 1'000'000.0f;
  LV_BMS::battery[0].cell_voltage[2] = (float)BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT3]) / 1'000'000.0f;
  LV_BMS::battery[0].cell_voltage[3] = (float)BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT4]) / 1'000'000.0f;
  LV_BMS::battery[0].cell_voltage[4] = (float)BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT5]) / 1'000'000.0f;
  LV_BMS::battery[0].cell_voltage[5] = (float)BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT6]) / 1'000'000.0f;

  /* NOTE: In MC33772C analog0 and analog2 are not useful */
  //LV_BMS::battery[0].analog_input[0] = measurements[BCC_MSR_AN0];
  LV_BMS::battery[0].analog_input[1] = measurements[BCC_MSR_AN1];
  //LV_BMS::battery[0].analog_input[2] = measurements[BCC_MSR_AN2];
  LV_BMS::battery[0].analog_input[3] = measurements[BCC_MSR_AN3];
  LV_BMS::battery[0].analog_input[4] = measurements[BCC_MSR_AN4];
  LV_BMS::battery[0].analog_input[5] = measurements[BCC_MSR_AN5];
  LV_BMS::battery[0].analog_input[6] = measurements[BCC_MSR_AN6];

  /* IC temperature measurement in ºC*10 */
  LV_BMS::battery[0].temp_times_ten = BCC_GET_IC_TEMP_C(measurements[BCC_MSR_ICTEMP]);
  // NOTE: This division might be too slow
  LV_BMS::battery[0].temperature = (float)LV_BMS::battery[0].temp_times_ten / 10.0f;

  /* ADCIA and ADCIB Band Gap Reference measurements, in micro volts */
  LV_BMS::battery[0].ADCIA_volts = (float)BCC_GET_VOLT(measurements[BCC_MSR_VBGADC1A]) / 1'000'000.0f;
  LV_BMS::battery[0].ADCIB_volts = (float)BCC_GET_VOLT(measurements[BCC_MSR_VBGADC1B]) / 1'000'000.0f;

  bcc_start_measurements();
}

// end LV_BMS_VERSION_MAJOR == 11
#elif LV_BMS_VERSION_MAJOR == 10

static void read_temperature(const float voltage, float* temperature) {
  auto resistance =
    (voltage * RESISTANCE_REFERENCE) / (VOLTAGE_REFERENCE - voltage);
  *temperature = (resistance - R0) / (TCR * R0);
}

#endif

//---------------------------------------------------------------

// connecting
void ConnectingState_Update(void)
{
  LV_BMS::operational_led->toggle();
}

// operational
void OperationalState_OnEnter(void)
{
  LV_BMS::operational_led->turn_on();
}

void OperationalState_OnExit(void)
{
  LV_BMS::operational_led->turn_off();
}

// fault
void FaultState_OnEnter(void)
{
  LV_BMS::fault_led->turn_on();
}

// NOTE: This is to be implemented, there is no fault recovery yet!
//void FaultState_OnExit(void)
//{
//  LV_BMS::fault_led->turn_off();
//}


//---------------------------------------------------------------

void lvbms_init_comms(void)
{
#if LV_BMS_VERSION_MAJOR == 10
  /* Comms init */ {
    DataPackets::Battery_Voltages_init(
      battery[0].cells[0], battery[0].cells[1], battery[0].cells[2],
      battery[0].cells[3], battery[0].cells[4], battery[0].cells[5], 
      min_cell, max_cell, total_voltage);

    DataPackets::Battery_Temperatures_init(
      temperature[0], temperature[1], 
      temperature[2], temperature[3], 
      min_temperature, max_temperature);

    DataPackets::State_of_Charge_init(SOC);
    DataPackets::Battery_Current_init(current);

    DataPackets::Current_State_init(state);
  }
#else
  auto battery = &LV_BMS::battery[0];

  /* Comms init */ {
    DataPackets::Battery_Voltages_init(
      battery->cell_voltage[0], battery->cell_voltage[1],
      battery->cell_voltage[2], battery->cell_voltage[3],
      battery->cell_voltage[4], battery->cell_voltage[5], 
      LV_BMS::min_cell, LV_BMS::max_cell, LV_BMS::total_voltage,
      LV_BMS::avg_cell
    );

    DataPackets::Battery_Temperatures_init(battery->temperature);

    DataPackets::State_of_Charge_init(LV_BMS::SOC);
    // NOTE: This value is not updated
    DataPackets::Battery_Current_init(LV_BMS::current);

    DataPackets::Current_State_init(LV_BMS::state);
  }
#endif
  DataPackets::start();
}

void LV_BMS::init() {
  lvbms_init_comms();

  OrderPackets::Averaging_init(*(OrderPackets::measurement_averaging*)&LV_BMS::avg_count);
  OrderPackets::start();

#if LV_BMS_VERSION_MAJOR == 10
  Scheduler::register_task((READING_PERIOD_US / 2) - 100, []() {
    bms.update();
  });
#elif LV_BMS_VERSION_MAJOR == 11
  Init_BCC_Driver();
  Scheduler::register_task(1000'000, bcc_info_failiures);
#endif

  //LV_BMS::add_protections();
  //DCLV::init();
}

void LV_BMS::start() {
  last_reading_time = HAL_GetTick();

  FaultController::start();
  Scheduler::register_task(100'000, []() {
    read();
  });
}

//------------------- SOC ------------------------

float LV_BMS::coulomb_counting_SOC(float current) {
  uint32_t current_time = HAL_GetTick();

  float delta_time = (current_time - last_reading_time) / 1000.0f;
  last_reading_time = current_time;

  float delta_SOC = current * delta_time / CAPACITY_AH * 3600.0f;
  return delta_SOC;
}

float LV_BMS::ocv_battery_SOC() {
#if LV_BMS_VERSION_MAJOR == 10
  float total_voltage = battery[0].cells[0] + battery[0].cells[1] + battery[0].cells[2] +
                        battery[0].cells[3] + battery[0].cells[4] + battery[0].cells[5];
  float x =
    total_voltage - 20.0;  // to get a bigger difference between values so
                           // the polynomial is more accurate
  float result = -62.5 + (14.9 * x) + (21.9 * x * x) + (-4.18 * x * x * x);
  return result;
#else
  static constexpr float max_voltage = 4.2f * 6;
  LV_BMS::total_voltage = battery[0].cell_voltage[0] + battery[0].cell_voltage[1] + 
                          battery[0].cell_voltage[2] + battery[0].cell_voltage[3] + 
                          battery[0].cell_voltage[4] + battery[0].cell_voltage[5];
  LV_BMS::avg_cell = LV_BMS::total_voltage / 6.0f;

  // very stupid initial estimation without any data, will make better this evening probably
  float soc = (LV_BMS::total_voltage / max_voltage)*(LV_BMS::total_voltage / max_voltage) * 100.0f;
  return soc;
#endif
}

void LV_BMS::update_SOC() {
  /* if(first_soc_flag == true) {
    SOC = ocv_battery_SOC();
    first_soc_flag = false;
  } else {
    if(std::abs(*current) < REST_THRESHOLD) {
      SOC += coulomb_counting_SOC(0 - *current);
    } else { */
      LV_BMS::SOC = ocv_battery_SOC();
#if LV_BMS_VERSION_MAJOR == 10
      if((SOC > -100.0f) && (SOC < 200.0f) && ((SOC < 20.0f) || (SOC > 80.0f))) [[unlikely]] {
        FAULT("Fault: LV battery State of charge not in range [20, 80]");
        LV_BMS_SM::State_Machine.force_change_state(static_cast<size_t>(BMS_State::FAULT));
      }
#endif
   /*  }
  } */
}

//------------------------------------------------

void LV_BMS::get_max_min_cells() {
  float maximum = FLT_MIN;
  float minimum = FLT_MAX;
#if LV_BMS_VERSION_MAJOR == 10
  for(unsigned int i = 0; i < ARRAY_LENGTH(LV_BMS::battery[0].cells); i++) {
    float v = LV_BMS::battery[0].cells[i];
    maximum = std::max(v, maximum);
    minimum = std::min(v, minimum);
  }
#else
  for(unsigned int i = 0; i < ARRAY_LENGTH(LV_BMS::battery[0].cell_voltage); i++) {
    float v = LV_BMS::battery[0].cell_voltage[i];
    maximum = std::max(v, maximum);
    minimum = std::min(v, minimum);
  }
#endif

  max_cell = maximum;
  min_cell = minimum;
}

#if LV_BMS_VERSION_MAJOR == 10
void get_max_min_temperatures() {
  float maximum = FLT_MIN;
  float minimum = FLT_MAX;
  for(unsigned int i = 0; i < ARRAY_LENGTH(LV_BMS::temperature); i++) {
    float v = LV_BMS::temperature[i];
    maximum = std::max(v, maximum);
    minimum = std::min(v, minimum);
  }

  max_temperature = maximum;
  min_temperature = minimum;
}
#else
// NOTE: H11 only has one temperature measurement
#define get_max_min_temperatures()
#endif

void LV_BMS::read() {
  update_SOC();
#if LV_BMS_VERSION_MAJOR == 10
  current_sensor.read();
  read_temperature(GPIO_voltage_1, &temperature[0]);
  read_temperature(GPIO_voltage_2, &temperature[1]);
  read_temperature(GPIO_voltage_3, &temperature[2]);
  read_temperature(GPIO_voltage_4, &temperature[3]);
#elif LV_BMS_VERSION_MAJOR == 11
  bcc_get_measurements();
#endif
  get_max_min_cells();
  get_max_min_temperatures();
}

void check_lvbms_transitions(void)
{
  BMS_State prev_state = LV_BMS::state;
  FaultController::check_transitions();
  if(FaultController::is_faulted()) {
    LV_BMS::state = BMS_State::FAULT;
  } else {
    LV_BMS::state = LV_BMS_SM::State_Machine.get_current_state();
  }

  if(LV_BMS::state != prev_state) [[unlikely]] {
#if STLIB_ETH
    DataPackets::control_station_udp->send_packet(*DataPackets::Current_State_packet);
#endif
  }
}

void LV_BMS::update() {
  check_lvbms_transitions();
  Scheduler::update();
}