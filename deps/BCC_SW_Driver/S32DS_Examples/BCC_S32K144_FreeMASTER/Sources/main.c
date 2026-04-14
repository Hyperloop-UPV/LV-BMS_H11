/*
 * Copyright 2016 - 2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/*!
** @file main.c
** @version 2.2
** @brief
**         Main module.
**         This module contains user's application code.
*/
/*!
**  @addtogroup main_module main module documentation
**  @{
*/
/* MODULE main */


/* Including necessary module. Cpu.h contains other modules needed for compiling.*/
#include "Cpu.h"

volatile int exit_code = 0;

/* User includes (#include below this line is not maintained by Processor Expert) */
#include <math.h>
#include "bcc/bcc.h"
#include "bcc_s32k144/bcc_wait.h"
#include "freemaster.h"
#include "common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Red on-board LED. */
#define RED_LED_PORT         PTD
#define RED_LED_PIN          15U

/* LPSPI_TX configuration. */
#define BCC_TX_LPSPI_DELAY_PCS_TO_SCLK         3U  /* 3us (f >= 1.75us) */
#define BCC_TX_LPSPI_DELAY_SCLK_TO_PCS         1U  /* 1us (g >= 0.60us) */
#define BCC_TX_LPSPI_DELAY_BETWEEN_TRANSFERS   5U  /* 5us (t_MCU_RES >= 4us) */

/* Used channel of LPIT0 for Freemaster timing. */
#define LPIT0_CHANNEL_FMASTER  0U

/* Used channel of LPIT0 for BCC SW driver timing. */
#define LPIT0_CHANNEL_BCCDRV   3U

/* NTC precomputed table configuration. */
/*! @brief Minimal temperature in NTC table.
 *
 * It directly influences size of the NTC table (number of precomputed values).
 * Specifically lower boundary.
 */
#define NTC_MINTEMP          (-40)

/*! @brief Maximal temperature in NTC table.
 *
 * It directly influences size of the NTC table (number of precomputed values).
 * Specifically higher boundary.
 */
#define NTC_MAXTEMP           (120)
/*! @brief Size of NTC look-up table. */
#define NTC_TABLE_SIZE        (NTC_MAXTEMP - NTC_MINTEMP + 1)
/*! @brief 0 degree Celsius converted to Kelvin. */
#define NTC_DEGC_0            273.15

/*!
 * @brief Calculates final temperature value.
 *
 * @param tblIdx Index of value in NTC table which is close
 *        to the register value provided by user.
 * @param degTenths Fractional part of temperature value.
 * @return Temperature.
 */
#define NTC_COMP_TEMP(tblIdx, degTenths) \
    ((((tblIdx) + NTC_MINTEMP) * 10) + (degTenths))

/*******************************************************************************
 * FreeMASTER definitions
 ******************************************************************************/

/* Number of FreeMASTER commands. */
#define COMMANDS_CNT 8

/* IDs of FreeMASTER commands. */
enum
{
    CMD_SLEEP = 0,              /* Puts BCC device and TPL transceiver into sleep mode. */
    CMD_WAKE_UP = 1,            /* Wakes up BCC device and TPL transceiver into normal mode. */
    CMD_SW_RESET = 2,           /* Resets BCC device by software command. */
    CMD_CONTROL_BALANCING = 3,  /* Controls cell balancing for configured cells. */
    CMD_CLEAR_FAULTS = 4,       /* Clears all faults in BCC device. */
    CMD_UPDATE_CB_TIMEOUT = 5,  /* Updates cell balancing timeout. */
    CMD_UPDATE_THRESHOLD = 6,   /* Updates internal device thresholds used for fault evaluation. */
    CMD_SET_CB_INDIVIDUALLY = 7 /* Enables/disables CB of specific cell. */
};

typedef enum
{
    TH_GPIOx_OT = 0,    /* Common GPIOx in analog input mode, overtemperature threshold. */
    TH_GPIOx_UT = 1,    /* Common GPIOx in analog input mode, undertemperature threshold. */
    TH_CTx_OV = 2,      /* Common CTx overvoltage threshold. */
    TH_CTx_UV = 3       /* Common CTx undervoltage threshold. */
} threshSel_t;

/* Size of FreeMASTER command's arguments in bytes. */
static const FMSTR_SIZE CMD_ARG_SIZE[] = {
    0,  /* BCC_Sleep() */
    0,  /* BCC_WakeUp() */
    0,  /* BCC_SoftwareReset(CID) - CID is known, only single device is used. */
    4,  /* BCC_EnableCB(CID, Enable) - CID is known. */
    0,  /* BCC_ClearFaultStatus(CID, FaultSel) - CID is known, all faults will be cleared. */
    4,  /* updateCBTimeout(uint16_t cbTimeout) */
    8,  /* updateThreshold(threshSel_t threshSel, uint16_t threshVal) */
    8,  /* BCC_SetCBIndividually(CID, uint16_t cellInx, uint16_t enable, timer) */
};

/*******************************************************************************
 * Structure definition
 ******************************************************************************/

/*!
* @brief NTC Configuration.
*
* The device has seven GPIOs which enable temperature measurement.
* NTC thermistor and fixed resistor are external components and must be set
* by the user. These values are used to calculate temperature. Beta parameter
* equation is used to calculate temperature. GPIO port of BCC device must be
* configured as Analog Input to measure temperature.
* This configuration is common for all GPIO ports and all devices (in case of
* daisy chain).
*/
typedef struct
{
    uint32_t beta;         /*!< Beta parameter of NTC thermistor in [K].
                                Admissible range is from 1 to 1000000. */
    uint32_t rntc;         /*!< R_NTC - NTC fixed resistance in [Ohm].
                                Admissible range is from 1 to 1000000. */
    uint32_t refRes;       /*!< NTC Reference Resistance in [Ohm].
                                Admissible range is from 1 to 1000000. */
    uint8_t refTemp;       /*!< NTC Reference Temperature in degrees [Celsius].
                                Admissible range is from 0 to 200. */
} ntc_config_t;

/*******************************************************************************
 * Initial BCC configuration
 ******************************************************************************/

/*! @brief  Number of MC33771 registers configured in the initialization with
 * user values. */
#define BCC_INIT_CONF_REG_CNT     56U

/* Structure containing a register name and its address. */
typedef struct
{
    const uint8_t address;
    const uint16_t defaultVal;
    const uint16_t value;
} bcc_init_reg_t;

/* Initial register configuration for MC33771C for this example. */
static const bcc_init_reg_t s_initRegsMc33771c[BCC_INIT_CONF_REG_CNT] = {
    {MC33771C_GPIO_CFG1_OFFSET, MC33771C_GPIO_CFG1_POR_VAL, MC33771C_GPIO_CFG1_VALUE},
    {MC33771C_GPIO_CFG2_OFFSET, MC33771C_GPIO_CFG2_POR_VAL, MC33771C_GPIO_CFG2_VALUE},
    {MC33771C_TH_ALL_CT_OFFSET, MC33771C_TH_ALL_CT_POR_VAL, MC33771C_TH_ALL_CT_VALUE},
    {MC33771C_TH_CT14_OFFSET, MC33771C_TH_CT14_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT13_OFFSET, MC33771C_TH_CT13_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT12_OFFSET, MC33771C_TH_CT12_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT11_OFFSET, MC33771C_TH_CT11_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT10_OFFSET, MC33771C_TH_CT10_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT9_OFFSET, MC33771C_TH_CT9_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT8_OFFSET, MC33771C_TH_CT8_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT7_OFFSET, MC33771C_TH_CT7_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT6_OFFSET, MC33771C_TH_CT6_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT5_OFFSET, MC33771C_TH_CT5_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT4_OFFSET, MC33771C_TH_CT4_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT3_OFFSET, MC33771C_TH_CT3_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT2_OFFSET, MC33771C_TH_CT2_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_CT1_OFFSET, MC33771C_TH_CT1_POR_VAL, MC33771C_TH_CTX_VALUE},
    {MC33771C_TH_AN6_OT_OFFSET, MC33771C_TH_AN6_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33771C_TH_AN5_OT_OFFSET, MC33771C_TH_AN5_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33771C_TH_AN4_OT_OFFSET, MC33771C_TH_AN4_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33771C_TH_AN3_OT_OFFSET, MC33771C_TH_AN3_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33771C_TH_AN2_OT_OFFSET, MC33771C_TH_AN2_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33771C_TH_AN1_OT_OFFSET, MC33771C_TH_AN1_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33771C_TH_AN0_OT_OFFSET, MC33771C_TH_AN0_OT_POR_VAL, MC33771C_TH_ANX_OT_VALUE},
    {MC33771C_TH_AN6_UT_OFFSET, MC33771C_TH_AN6_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33771C_TH_AN5_UT_OFFSET, MC33771C_TH_AN5_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33771C_TH_AN4_UT_OFFSET, MC33771C_TH_AN4_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33771C_TH_AN3_UT_OFFSET, MC33771C_TH_AN3_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33771C_TH_AN2_UT_OFFSET, MC33771C_TH_AN2_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33771C_TH_AN1_UT_OFFSET, MC33771C_TH_AN1_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33771C_TH_AN0_UT_OFFSET, MC33771C_TH_AN0_UT_POR_VAL, MC33771C_TH_ANX_UT_VALUE},
    {MC33771C_CB1_CFG_OFFSET, MC33771C_CB1_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB2_CFG_OFFSET, MC33771C_CB2_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB3_CFG_OFFSET, MC33771C_CB3_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB4_CFG_OFFSET, MC33771C_CB4_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB5_CFG_OFFSET, MC33771C_CB5_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB6_CFG_OFFSET, MC33771C_CB6_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB7_CFG_OFFSET, MC33771C_CB7_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB8_CFG_OFFSET, MC33771C_CB8_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB9_CFG_OFFSET, MC33771C_CB9_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB10_CFG_OFFSET, MC33771C_CB10_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB11_CFG_OFFSET, MC33771C_CB11_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB12_CFG_OFFSET, MC33771C_CB12_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB13_CFG_OFFSET, MC33771C_CB13_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_CB14_CFG_OFFSET, MC33771C_CB14_CFG_POR_VAL, MC33771C_CBX_CFG_VALUE},
    {MC33771C_OV_UV_EN_OFFSET, MC33771C_OV_UV_EN_POR_VAL, MC33771C_OV_UV_EN_VALUE},
    {MC33771C_SYS_CFG1_OFFSET, MC33771C_SYS_CFG1_POR_VAL, MC33771C_SYS_CFG1_VALUE},
    {MC33771C_SYS_CFG2_OFFSET, MC33771C_SYS_CFG2_POR_VAL, MC33771C_SYS_CFG2_VALUE},
    {MC33771C_ADC_CFG_OFFSET, MC33771C_ADC_CFG_POR_VAL, MC33771C_ADC_CFG_VALUE},
    {MC33771C_ADC2_OFFSET_COMP_OFFSET, MC33771C_ADC2_OFFSET_COMP_POR_VAL, MC33771C_ADC2_OFFSET_COMP_VALUE},
    {MC33771C_FAULT_MASK1_OFFSET, MC33771C_FAULT_MASK1_POR_VAL, MC33771C_FAULT_MASK1_VALUE},
    {MC33771C_FAULT_MASK2_OFFSET, MC33771C_FAULT_MASK2_POR_VAL, MC33771C_FAULT_MASK2_VALUE},
    {MC33771C_FAULT_MASK3_OFFSET, MC33771C_FAULT_MASK3_POR_VAL, MC33771C_FAULT_MASK3_VALUE},
    {MC33771C_WAKEUP_MASK1_OFFSET, MC33771C_WAKEUP_MASK1_POR_VAL, MC33771C_WAKEUP_MASK1_VALUE},
    {MC33771C_WAKEUP_MASK2_OFFSET, MC33771C_WAKEUP_MASK2_POR_VAL, MC33771C_WAKEUP_MASK2_VALUE},
    {MC33771C_WAKEUP_MASK3_OFFSET, MC33771C_WAKEUP_MASK3_POR_VAL, MC33771C_WAKEUP_MASK3_VALUE},
};

/*******************************************************************************
 * Global variables
 ******************************************************************************/

bcc_drv_config_t drvConfig;  /* BCC driver configuration. */
uint16_t g_ntcTable[NTC_TABLE_SIZE]; /* NTC look-up table. */

/* Last values received from FreeMASTER. */
int32_t thresholdsLast[TH_CTx_UV + 1] = {-1, -1, -1, -1};
bool cbIndividual[BCC_MAX_CELLS] = {false, false, false, false, false,
        false, false, false, false, false, false, false, false, false};
int32_t cbTimeoutLast = -1;
int32_t cbEnabledLast = -1;

/* Measurement results (used by FreeMASTER application). */
uint32_t stackVoltageUV;
uint32_t cell1VoltageUV;
uint32_t cell2VoltageUV;
uint32_t cell3VoltageUV;
uint32_t cell4VoltageUV;
uint32_t cell5VoltageUV;
uint32_t cell6VoltageUV;
uint32_t cell7VoltageUV;
uint32_t cell8VoltageUV;
uint32_t cell9VoltageUV;
uint32_t cell10VoltageUV;
uint32_t cell11VoltageUV;
uint32_t cell12VoltageUV;
uint32_t cell13VoltageUV;
uint32_t cell14VoltageUV;

int16_t an0TempDegC;
int16_t an1TempDegC;
int16_t an2TempDegC;
int16_t an3TempDegC;
int16_t an4TempDegC;
int16_t an5TempDegC;
int16_t an6TempDegC;

int16_t icTempDegC;

uint32_t vBGADC1AVoltageUV;
uint32_t vBGADC1BVoltageUV;

/* Faults (used by FreeMASTER application). */
uint16_t faultStatus1;
uint16_t faultStatus2;
uint16_t faultStatus3;

/* State variable (used as indication if SPI is accessible or not). */
bool sleepMode = false;

int32_t timeout = 0;

/*******************************************************************************
 * Function prototypes
 ******************************************************************************/

static bcc_status_t initRegisters();
static bcc_status_t clearFaultRegs();
static status_t initDemo();
static void fillNtcTable(const ntc_config_t* const ntcConfig);
static bcc_status_t getNtcCelsius(uint16_t regVal, int16_t* temp);
static void initTimeout(int32_t timeoutMs);
static bool timeoutExpired(void);
static bcc_status_t updateMeasurements(void);
static bcc_status_t updateFaultStatus(void);
static bcc_status_t updateCBTimeout(uint16_t cbTimeout);
static bcc_status_t updateThreshold(threshSel_t threshSel, uint16_t threshVal);
static bool processCmd(void);

/*******************************************************************************
 * Functions
 ******************************************************************************/

/*!
* @brief LPIT0 IRQ handler.
*/
void LPIT0_Ch0_IRQHandler(void)
{
    LPIT_DRV_ClearInterruptFlagTimerChannels(INST_LPIT1, (1 << LPIT0_CHANNEL_FMASTER));

    timeout--;
}

/*!
* @brief This function initializes timeout.
*
* @param timeoutMs timeout delay in [ms].
*/
static void initTimeout(int32_t timeoutMs)
{
    timeout = timeoutMs;
}

/*!
 * @brief Initializes BCC device registers according to BCC_INIT_CONF.
 * Registers having the wanted content already after POR are not rewritten.
 */
static bcc_status_t initRegisters()
{
    uint8_t i;
    bcc_status_t status;

    for (i = 0; i < BCC_INIT_CONF_REG_CNT; i++)
    {
        if (s_initRegsMc33771c[i].value != s_initRegsMc33771c[i].defaultVal)
        {
            status = BCC_Reg_Write(&drvConfig, BCC_CID_DEV1,
                    s_initRegsMc33771c[i].address, s_initRegsMc33771c[i].value);
            if (status != BCC_STATUS_SUCCESS)
            {
                return status;
            }
        }
    }

    return BCC_STATUS_SUCCESS;
}

/*!
 * @brief Clears all fault registers of BCC devices.
 */
static bcc_status_t clearFaultRegs()
{
    bcc_status_t status;

    status = BCC_Fault_ClearStatus(&drvConfig, BCC_CID_DEV1, BCC_FS_CELL_OV);
    if (status != BCC_STATUS_SUCCESS)
    {
        return status;
    }

    status = BCC_Fault_ClearStatus(&drvConfig, BCC_CID_DEV1, BCC_FS_CELL_UV);
    if (status != BCC_STATUS_SUCCESS)
    {
        return status;
    }

    status = BCC_Fault_ClearStatus(&drvConfig, BCC_CID_DEV1, BCC_FS_CB_OPEN);
    if (status != BCC_STATUS_SUCCESS)
    {
        return status;
    }

    status = BCC_Fault_ClearStatus(&drvConfig, BCC_CID_DEV1, BCC_FS_CB_SHORT);
    if (status != BCC_STATUS_SUCCESS)
    {
        return status;
    }

    status = BCC_Fault_ClearStatus(&drvConfig, BCC_CID_DEV1, BCC_FS_GPIO_STATUS);
    if (status != BCC_STATUS_SUCCESS)
    {
        return status;
    }

    status = BCC_Fault_ClearStatus(&drvConfig, BCC_CID_DEV1, BCC_FS_AN_OT_UT);
    if (status != BCC_STATUS_SUCCESS)
    {
        return status;
    }

    status = BCC_Fault_ClearStatus(&drvConfig, BCC_CID_DEV1, BCC_FS_GPIO_SHORT);
    if (status != BCC_STATUS_SUCCESS)
    {
        return status;
    }

    status = BCC_Fault_ClearStatus(&drvConfig, BCC_CID_DEV1, BCC_FS_COMM);
    if (status != BCC_STATUS_SUCCESS)
    {
        return status;
    }

    status = BCC_Fault_ClearStatus(&drvConfig, BCC_CID_DEV1, BCC_FS_FAULT1);
    if (status != BCC_STATUS_SUCCESS)
    {
        return status;
    }

    status = BCC_Fault_ClearStatus(&drvConfig, BCC_CID_DEV1, BCC_FS_FAULT2);
    if (status != BCC_STATUS_SUCCESS)
    {
        return status;
    }

    return BCC_Fault_ClearStatus(&drvConfig, BCC_CID_DEV1, BCC_FS_FAULT3);
}

/*!
 * @brief MCU and BCC initialization.
 */
static status_t initDemo()
{
    ntc_config_t ntcConfig;
    status_t status;
    bcc_status_t bccStatus;

    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
            g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_FORCIBLE);

    /* Pin-muxing + GPIO pin direction and initial value settings. */
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    /* Initialize LPUART instance */
    status = LPUART_DRV_Init(INST_LPUART1, &lpuart1_State, &lpuart1_InitConfig0);
    if (status != STATUS_SUCCESS)
    {
        return STATUS_ERROR;
    }

    INT_SYS_InstallHandler(LPUART1_RxTx_IRQn, FMSTR_Isr, NULL);
    INT_SYS_EnableIRQ(LPUART1_RxTx_IRQn);

    /* Initialize LPIT instance */
    LPIT_DRV_Init(INST_LPIT1, &lpit1_InitConfig);
    status = LPIT_DRV_InitChannel(INST_LPIT1, LPIT0_CHANNEL_FMASTER, &lpit1_ChnConfig0);
    if (status != STATUS_SUCCESS)
    {
        return STATUS_ERROR;
    }
    status = LPIT_DRV_InitChannel(INST_LPIT1, LPIT0_CHANNEL_BCCDRV, &lpit1_ChnConfig3);
    if (status != STATUS_SUCCESS)
    {
        return STATUS_ERROR;
    }

#ifdef TPL
    /* Initialize LPSPI instances. */
    status = LPSPI_DRV_MasterInit(LPSPITPLTX, &lpspiTplTxState, &lpspiTplTx_MasterConfig);
    if (status != STATUS_SUCCESS)
    {
        return status;
    }

    status = LPSPI_DRV_MasterSetDelay(LPSPITPLTX, BCC_TX_LPSPI_DELAY_BETWEEN_TRANSFERS,
            BCC_TX_LPSPI_DELAY_SCLK_TO_PCS, BCC_TX_LPSPI_DELAY_PCS_TO_SCLK);
    if (status != STATUS_SUCCESS)
    {
        return status;
    }

    status = LPSPI_DRV_SlaveInit(LPSPISPI_TPL1RX, &lpspiSpi_Tpl1RxState, &lpspiTpl1Rx_SlaveConfig);
    if (status != STATUS_SUCCESS)
    {
        return status;
    }
#else
    /* Initialize LPSPI instance. */
    status = LPSPI_DRV_MasterInit(LPSPISPI_TPL1RX, &lpspiSpi_Tpl1RxState, &lpspiSpi_MasterConfig);
    if (status != STATUS_SUCCESS)
    {
        return status;
    }
#endif


    /* Initialize BCC driver configuration structure. */
    drvConfig.drvInstance = 0U;
#ifdef TPL
    drvConfig.commMode = BCC_MODE_TPL;
#else
    drvConfig.commMode = BCC_MODE_SPI;
#endif
    drvConfig.devicesCnt = 1U;
    drvConfig.device[0] = BCC_DEVICE_MC33771C;
    drvConfig.cellCnt[0] = 14U;
    drvConfig.loopBack = false;

    /* Precalculate NTC look up table for fast temperature measurement. */
    ntcConfig.rntc = 6800U;                 /* NTC pull-up 6.8kOhm */
    ntcConfig.refTemp = 25U;                /* NTC resistance 10kOhm at 25 degC */
    ntcConfig.refRes = 10000U;              /* NTC resistance 10kOhm at 25 degC */
    ntcConfig.beta = 3900U;

    fillNtcTable(&ntcConfig);

    LPIT_DRV_StartTimerChannels(INST_LPIT1, (1 << LPIT0_CHANNEL_FMASTER));

    bccStatus = BCC_Init(&drvConfig);
    if (bccStatus != BCC_STATUS_SUCCESS)
    {
        return STATUS_ERROR;
    }

    bccStatus = initRegisters();
    if (bccStatus != BCC_STATUS_SUCCESS)
    {
        return STATUS_ERROR;
    }

    bccStatus = clearFaultRegs();
    if (bccStatus != BCC_STATUS_SUCCESS)
    {
        return STATUS_ERROR;
    }

    if (FMSTR_Init() == 0)
    {
        return STATUS_ERROR;
    }

    return STATUS_SUCCESS;
}

/*!
 * @brief This function fills the NTC look up table.
 *
 * NTC look up table is intended for resistance to temperature conversion.
 * An array item contains raw value from a register. Index of the item is
 * temperature value.
 *
 * ArrayItem = (Vcom * Rntc) / (0.00015258789 * (NTC + Rntc))
 * Where:
 *  - ArrayItem is an item value of the table,
 *  - Vcom is maximal voltage (5V),
 *  - NTC is the resistance of NTC thermistor (Ohm),
 *  - 0.00015258789 is resolution of measured voltage in Volts
 *    (V = 152.58789 uV * Register_value),
 *  - Rntc is value of a resistor connected to Vcom (see MC3377x datasheet,
 *    section MC3377x PCB components).
 *
 * Beta formula used to calculate temperature based on NTC resistance:
 *   1 / T = 1 / T0 + (1 / Beta) * ln(Rt / R0)
 * Where:
 *  - R0 is the resistance (Ohm) at temperature T0 (Kelvin),
 *  - Beta is material constant (Kelvin),
 *  - T is temperature corresponding to resistance of the NTC thermistor.
 *
 * Equation for NTC value is given from the Beta formula:
 *   NTC = R0 * exp(Beta * (1/T - 1/T0))
 *
 * @param ntcConfig Pointer to NTC components configuration.
 */
void fillNtcTable(const ntc_config_t* const ntcConfig)
{
    double ntcVal, expArg;
    uint16_t i = 0;
    int32_t temp;

    for (temp = NTC_MINTEMP; temp <= NTC_MAXTEMP; temp++)
    {
        expArg = ntcConfig->beta * ((1.0 / (NTC_DEGC_0 + temp)) -
                (1.0 / (NTC_DEGC_0 + ntcConfig->refTemp)));
        ntcVal = exp(expArg) * ntcConfig->refRes;
        g_ntcTable[i] = (uint16_t)round((ntcVal /
                (ntcVal + ntcConfig->rntc)) * MC33771C_MEAS_AN0_MEAS_AN_MASK);
        i++;
    }
}

/*!
 * @brief This function calculates temperature from raw value of MEAS_ANx
 * register. It uses precalculated values stored in g_ntcTable table.
 *
 * @param regVal Value of MEAS_ANx register.
 * @param temp Temperature value in deg. of Celsius * 10.
 *
 * @return bcc_status_t Error code.
 */
static bcc_status_t getNtcCelsius(uint16_t regVal, int16_t* temp)
{
    int16_t left = 0;    /* Pointer (index) to the left border of interval (NTC table). */
    /* Pointer (index) to the right border of interval (NTC table). */
    int16_t right = NTC_TABLE_SIZE - 1;
    int16_t middle;      /* Pointer (index) to the middle of interval (NTC table). */
    int8_t degTenths;    /* Fractional part of temperature value. */

    BCC_MCU_Assert(temp != NULL);

    /* Check range of NTC table. */
    if (g_ntcTable[NTC_TABLE_SIZE - 1] > regVal)
    {
        *temp = NTC_COMP_TEMP(NTC_TABLE_SIZE - 1, 0);
        return BCC_STATUS_SUCCESS;
    }
    if (g_ntcTable[0] < regVal)
    {
        *temp = NTC_COMP_TEMP(0, 0);
        return BCC_STATUS_SUCCESS;
    }

    regVal &= BCC_GET_MEAS_RAW(regVal);

    /* Search for an array item which is close to the register value provided
    * by user (regVal). Used method is binary search in sorted array. */
    while ((left + 1) != right)
    {
        /* Split interval into halves. */
        middle = (left + right) >> 1U;
        if (g_ntcTable[middle] <= regVal)
        {
            /* Select right half (array items are in descending order). */
            right = middle;
        }
        else
        {
            /* Select left half. */
            left = middle;
        }
    }

    /* Notes: found table item (left) is less than the following item in the
    * table (left + 1).
    * The last item cannot be found (algorithm property). */

    /* Calculate fractional part of temperature. */
    degTenths = (g_ntcTable[left] - regVal) / ((g_ntcTable[left] - g_ntcTable[left + 1]) / 10);
    (*temp) = NTC_COMP_TEMP(left, degTenths);

    return BCC_STATUS_SUCCESS;
}

/*!
* @brief This function indicates if the timeout expired.
*
* @return True if timeout expired, otherwise false.
*/
static bool timeoutExpired(void)
{
    return (timeout <= 0);
}

/*!
 * @brief This function reads values measured and provided via SPI
 * by BCC device (ISENSE, cell voltages, temperatures).
 *
 * @return bcc_status_t Error code.
 */
static bcc_status_t updateMeasurements(void)
{
    bcc_status_t error;
    uint16_t measurements[BCC_MEAS_CNT]; /* Array needed to store all measured values. */

    /* Step 1: Start conversion and wait for the conversion time. */
    error = BCC_Meas_StartAndWait(&drvConfig, BCC_CID_DEV1, BCC_AVG_1);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* Step 2: Convert raw measurements to appropriate units. */
    error = BCC_Meas_GetRawValues(&drvConfig, BCC_CID_DEV1, measurements);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* You can use bcc_measurements_t enumeration to index array with raw values. */
    /* Useful macros can be found in bcc.h or bcc_MC3377x.h. */

    /* Stack voltage in [uV]. */
    stackVoltageUV = BCC_GET_STACK_VOLT(measurements[BCC_MSR_STACK_VOLT]);
    /* Cells voltage in [uV]. */
    cell1VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT1]);
    cell2VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT2]);
    cell3VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT3]);
    cell4VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT4]);
    cell5VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT5]);
    cell6VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT6]);
    cell7VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT7]);
    cell8VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT8]);
    cell9VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT9]);
    cell10VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT10]);
    cell11VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT11]);
    cell12VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT12]);
    cell13VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT13]);
    cell14VoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_CELL_VOLT14]);

    /* For calculation of temperature on GPIO[0-6] in analog mode with use of NTC resistor. */
    /* To measure temperature selected GPIOs have to be set to analog input AM or RM mode! */
    error = getNtcCelsius(measurements[BCC_MSR_AN0], &an0TempDegC);
    error |= getNtcCelsius(measurements[BCC_MSR_AN1], &an1TempDegC);
    error |= getNtcCelsius(measurements[BCC_MSR_AN2], &an2TempDegC);
    error |= getNtcCelsius(measurements[BCC_MSR_AN3], &an3TempDegC);
    error |= getNtcCelsius(measurements[BCC_MSR_AN4], &an4TempDegC);
    error |= getNtcCelsius(measurements[BCC_MSR_AN5], &an5TempDegC);
    error |= getNtcCelsius(measurements[BCC_MSR_AN6], &an6TempDegC);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    /* IC temperature measurement in degC (register MEAS_IC_TEMP). */
    icTempDegC = (int16_t)(BCC_GET_IC_TEMP_C(measurements[BCC_MSR_ICTEMP]));

    /* ADCIA Band Gap Reference measurement (register MEAS_VBG_DIAG_ADC1A). */
    vBGADC1AVoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_VBGADC1A]);
    /* ADCIB Band Gap Reference measurement (register MEAS_VBG_DIAG_ADC1B). */
    vBGADC1BVoltageUV = BCC_GET_VOLT(measurements[BCC_MSR_VBGADC1B]);

    return BCC_STATUS_SUCCESS;
}

/*!
 * @brief This function reads summary fault status registers of BCC
 * device via SPI.
 *
 * @return bcc_status_t Error code.
 */
static bcc_status_t updateFaultStatus(void)
{
    bcc_status_t error;
    uint16_t faultStatus[BCC_STAT_CNT];

    error = BCC_Fault_GetStatus(&drvConfig, BCC_CID_DEV1, faultStatus);
    if (error != BCC_STATUS_SUCCESS)
    {
        return error;
    }

    faultStatus1 = faultStatus[BCC_FS_FAULT1];
    faultStatus2 = faultStatus[BCC_FS_FAULT2];
    faultStatus3 = faultStatus[BCC_FS_FAULT3];

    return BCC_STATUS_SUCCESS;
}

/*!
 * @brief This function updates cell balancing timeout.
 *
 * @param cbTimeout Cell balancing timeout in range [0 - 511] minutes
 *
 * @return bcc_status_t Error code.
 */
static bcc_status_t updateCBTimeout(uint16_t cbTimeout)
{
    bcc_status_t error;
    uint8_t cellId;

    /* Change timers. */
    cbTimeoutLast = (int32_t) cbTimeout;
    for (cellId = 0; cellId < BCC_MAX_CELLS; cellId++)
    {
        /* Turn cell balancing off. */
        if (cbIndividual[cellId])
        {
            error = BCC_CB_SetIndividual(&drvConfig, BCC_CID_DEV1, cellId, false, cbTimeout);
            if (error != BCC_STATUS_SUCCESS)
            {
                return error;
            }
        }

        /* Update the CB interval. */
        error = BCC_CB_SetIndividual(&drvConfig, BCC_CID_DEV1, cellId, cbIndividual[cellId], cbTimeout);
        if (error != BCC_STATUS_SUCCESS)
        {
            return error;
        }
    }

    return BCC_STATUS_SUCCESS;
}

/*!
 * @brief This function updates internal device threshold.
 *
 * @param threshSel Selection of threshold to be updated.
 * @param threshVal New threshold value.
 *
 * @return bcc_status_t Error code.
 */
static bcc_status_t updateThreshold(threshSel_t threshSel, uint16_t threshVal)
{
    bcc_status_t error;
    uint8_t regAddr;

    switch (threshSel)
    {
        case TH_GPIOx_OT:
            thresholdsLast[TH_GPIOx_OT] = (int32_t) threshVal;
            for (regAddr = MC33771C_TH_AN6_OT_OFFSET; regAddr <= MC33771C_TH_AN0_OT_OFFSET; regAddr++)
            {
                error = BCC_Reg_Update(&drvConfig, BCC_CID_DEV1, regAddr,
                        MC33771C_TH_AN1_OT_AN_OT_TH_MASK, MC33771C_TH_AN1_OT_AN_OT_TH(BCC_GET_TH_ANX(threshVal)));
                if (error != BCC_STATUS_SUCCESS)
                {
                    return error;
                }
            }
            break;

        case TH_GPIOx_UT:
            thresholdsLast[TH_GPIOx_UT] = (int32_t) threshVal;
            for (regAddr = MC33771C_TH_AN6_UT_OFFSET; regAddr <= MC33771C_TH_AN0_UT_OFFSET; regAddr++)
            {
                error = BCC_Reg_Update(&drvConfig, BCC_CID_DEV1, regAddr,
                        MC33771C_TH_AN1_UT_AN_UT_TH_MASK, MC33771C_TH_AN1_UT_AN_UT_TH(BCC_GET_TH_ANX(threshVal)));
                if (error != BCC_STATUS_SUCCESS)
                {
                    return error;
                }
            }
            break;

        case TH_CTx_OV:
            thresholdsLast[TH_CTx_OV] = (int32_t) threshVal;
            error = BCC_Reg_Update(&drvConfig, BCC_CID_DEV1, MC33771C_TH_ALL_CT_OFFSET,
                    MC33771C_TH_ALL_CT_ALL_CT_OV_TH_MASK, MC33771C_TH_ALL_CT_ALL_CT_OV_TH(BCC_GET_TH_CTX(threshVal)));
            if (error != BCC_STATUS_SUCCESS)
            {
                return error;
            }
            break;

        case TH_CTx_UV:
            thresholdsLast[TH_CTx_UV] = (int32_t) threshVal;
            error = BCC_Reg_Update(&drvConfig, BCC_CID_DEV1, MC33771C_TH_ALL_CT_OFFSET,
                    MC33771C_TH_ALL_CT_ALL_CT_UV_TH_MASK, MC33771C_TH_ALL_CT_ALL_CT_UV_TH(BCC_GET_TH_CTX(threshVal)));
            if (error != BCC_STATUS_SUCCESS)
            {
                return error;
            }
            break;

        default:
            return BCC_STATUS_PARAM_RANGE;
    }

    return BCC_STATUS_SUCCESS;
}

/*!
 * @brief This function checks and eventually processes FreeMASTER command.
 *
 * @return False when an error occurred, true otherwise.
 */
static bool processCmd(void)
{
    bcc_status_t error = BCC_STATUS_SUCCESS;
    FMSTR_APPCMD_CODE cmd;        /* Freemaster application command */
    FMSTR_APPCMD_PDATA cmdData;   /* Freemaster pointer to application command data */
    FMSTR_SIZE cmdSize;           /* Size of application command arguments */
    uint32_t *params = NULL;      /* Pointer to parameters of a called command. */
    uint8_t i;

    /* Fetch application command code. */
    cmd = FMSTR_GetAppCmd();

    /* if there is any command proceed to read it and take respective action */
    if ((cmd != FMSTR_APPCMDRESULT_NOCMD) && (cmd < COMMANDS_CNT))
    {
        /* Get pointer to application command data */
        cmdData = FMSTR_GetAppCmdData(&cmdSize);

        /* Check size of arguments. */
        if (cmdSize != CMD_ARG_SIZE[cmd])
        {
            /* Wrong number of parameters. */
            FMSTR_AppCmdAck(true);
            return false;
        }

        /* Set pointer to arguments. */
        params = (uint32_t *)cmdData;

        switch (cmd)
        {
            case CMD_SLEEP:
                error = BCC_Sleep(&drvConfig);
                FMSTR_AppCmdAck(error != BCC_STATUS_SUCCESS);
                if (error != BCC_STATUS_SUCCESS)
                {
                    PINS_DRV_ClearPins(RED_LED_PORT, 1U << RED_LED_PIN);
                }
                else
                {
                    sleepMode = true;
                }
                break;

            case CMD_WAKE_UP:
                BCC_WakeUp(&drvConfig);
                BCC_MCU_WaitMs(5);
                FMSTR_AppCmdAck(error != BCC_STATUS_SUCCESS);
                sleepMode = false;
                break;

            case CMD_SW_RESET:
                /* Software reset */
                error = BCC_SoftwareReset(&drvConfig, BCC_CID_DEV1);
                BCC_MCU_WaitMs(5);
                /* Init registers */
                error |= BCC_Init(&drvConfig);
                error |= initRegisters();
                error |= clearFaultRegs();
                /* Restore last values sent from FreeMASTER. */
                for (i = 0; i <= TH_CTx_UV; i++)
                {
                    if (thresholdsLast[i] >= 0)
                    {
                        error |= updateThreshold((threshSel_t)i, (uint16_t)thresholdsLast[i]);
                    }
                }
                /* Turn off cell balancing, but update the latest indiviual settings. */
                error |= BCC_CB_Enable(&drvConfig, BCC_CID_DEV1, false);
                for (i = 0; i < BCC_MAX_CELLS; i++)
                {
                    error |= BCC_CB_SetIndividual(&drvConfig, BCC_CID_DEV1, i,
                            cbIndividual[i],
                            (cbTimeoutLast < 0) ? 1 : (uint16_t)cbTimeoutLast);
                }
                FMSTR_AppCmdAck(error != BCC_STATUS_SUCCESS);
                if (error != BCC_STATUS_SUCCESS)
                {
                    PINS_DRV_ClearPins(RED_LED_PORT, 1U << RED_LED_PIN);
                }
                break;

            case CMD_CONTROL_BALANCING:
                cbEnabledLast = (int32_t)((bool)params[0]);
                error = BCC_CB_Enable(&drvConfig, BCC_CID_DEV1, (bool)params[0]);
                FMSTR_AppCmdAck(error != BCC_STATUS_SUCCESS);
                if (error != BCC_STATUS_SUCCESS)
                {
                    PINS_DRV_ClearPins(RED_LED_PORT, 1U << RED_LED_PIN);
                }
                break;

            case CMD_SET_CB_INDIVIDUALLY:
                if (BCC_IS_IN_RANGE(params[0], 0, BCC_MAX_CELLS - 1))
                {
                    cbIndividual[params[0]] = ((bool)params[1]);
                    error = BCC_CB_SetIndividual(&drvConfig, BCC_CID_DEV1,
                            (uint8_t)params[0], (bool)params[1],
                            (cbTimeoutLast < 0) ? 1 : (uint16_t)cbTimeoutLast);
                    FMSTR_AppCmdAck(error != BCC_STATUS_SUCCESS);
                    if (error != BCC_STATUS_SUCCESS)
                    {
                        PINS_DRV_ClearPins(RED_LED_PORT, 1U << RED_LED_PIN);
                    }
                }
                break;

            case CMD_CLEAR_FAULTS:
                for (i = BCC_FS_CELL_OV; i <= BCC_FS_FAULT3; i++)
                {
                    if (i == BCC_FS_COMM)
                    {
                        continue;
                    }

                    error = BCC_Fault_ClearStatus(&drvConfig, BCC_CID_DEV1, i);
                    if (error != BCC_STATUS_SUCCESS)
                    {
                        break;
                    }
                }
                FMSTR_AppCmdAck(error != BCC_STATUS_SUCCESS);
                if (error != BCC_STATUS_SUCCESS)
                {
                    PINS_DRV_ClearPins(RED_LED_PORT, 1U << RED_LED_PIN);
                }
                else
                {
                    PINS_DRV_SetPins(RED_LED_PORT, 1U << RED_LED_PIN);
                }
                break;

            case CMD_UPDATE_CB_TIMEOUT:
                error = updateCBTimeout((uint16_t)params[0]);
                FMSTR_AppCmdAck(error != BCC_STATUS_SUCCESS);
                if (error != BCC_STATUS_SUCCESS)
                {
                    PINS_DRV_ClearPins(RED_LED_PORT, 1U << RED_LED_PIN);
                }
                break;

            case CMD_UPDATE_THRESHOLD:
                error = updateThreshold((threshSel_t)params[0], (uint16_t)params[1]);
                FMSTR_AppCmdAck(error != BCC_STATUS_SUCCESS);
                if (error != BCC_STATUS_SUCCESS)
                {
                    PINS_DRV_ClearPins(RED_LED_PORT, 1U << RED_LED_PIN);
                }
                break;

            default:
                /* An error occurred. */
                FMSTR_AppCmdAck(true);
                return false;
        }
    }

    return true;
}

/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
*/
int main(void)
{
    bcc_status_t bccStatus;

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  #ifdef PEX_RTOS_INIT
    PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of Processor Expert internal initialization.                    ***/

    if (initDemo() == STATUS_SUCCESS)
    {
        /* Infinite loop for the real-time processing routines. */
        while (1)
        {
            initTimeout(200);

            /* Loops until specified timeout expires. */
            do
            {
                /* Serving FreeMASTER commands. */
                FMSTR_Poll();
                processCmd();

                if (!sleepMode)
                {
                    /* To prevent communication loss. */
                    if (BCC_SendNop(&drvConfig, BCC_CID_DEV1) != BCC_STATUS_SUCCESS)
                    {
                        PINS_DRV_ClearPins(RED_LED_PORT, 1U << RED_LED_PIN);
                    }
                }
            } while (timeoutExpired() == false);

            if (!sleepMode)
            {
                /* Update measurements once per 200 ms. */
                bccStatus = updateMeasurements();
                if (bccStatus != BCC_STATUS_SUCCESS)
                {
                    PINS_DRV_ClearPins(RED_LED_PORT, 1U << RED_LED_PIN);
                }

                /* Updates fault status once per 200 ms. */
                bccStatus = updateFaultStatus();
                if (bccStatus != BCC_STATUS_SUCCESS)
                {
                    PINS_DRV_ClearPins(RED_LED_PORT, 1U << RED_LED_PIN);
                }
            }
        }
    }
    else
    {
        PINS_DRV_ClearPins(RED_LED_PORT, 1U << RED_LED_PIN);
    }

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the Freescale S32K series of microcontrollers.
**
** ###################################################################
*/
