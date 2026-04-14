#ifndef BCC_STLIB_H
#define BCC_STLIB_H

#include "ST-LIB.hpp"

#include "../../../../deps/BCC_SW_Driver/bcc/bcc.h"

/*!
 * @brief Returns SCG system clock frequency.
 *
 * @return SCG system clock frequency.
 */
uint32_t BCC_MCU_GetSystemClockFreq(void);

/*!
 * @brief Waits for specified amount of seconds.
 *
 * @param delay Number of seconds to wait.
 */
void BCC_MCU_WaitSec(uint16_t delay);

/*!
 * @brief Waits for specified amount of milliseconds.
 *
 * @param delay Number of milliseconds to wait.
 */
void BCC_MCU_WaitMs(uint16_t delay);

/*!
 * @brief Waits for specified amount of microseconds.
 *
 * @param delay Number of microseconds to wait.
 */
void BCC_MCU_WaitUs(uint32_t delay);

/*!
 * @brief Starts a non-blocking timeout mechanism. After expiration of the time
 * passed as a parameter, function BCC_MCU_TimeoutExpired should signalize an
 * expired timeout.
 *
 * @param timeoutUs Length of the timeout in microseconds.
 *
 * @return Returns BCC_STATUS_TIMEOUT_START in case of error, BCC_STATUS_SUCCESS
 *         otherwise.
 */
bcc_status_t BCC_MCU_StartTimeout(uint32_t timeoutUs);

/*!
 * @brief Returns state of the timeout mechanism started by the function
 * BCC_MCU_StartTimeout.
 *
 * @return True if timeout expired, false otherwise.
 */
bool BCC_MCU_TimeoutExpired(void);

/*!
  * @brief This function performs one 48b transfer via SPI bus. Intended for SPI
  * mode only. 
  *
  * The byte order of buffers is given by BCC_MSG_* macros (in bcc.h).
  *
  * @param drvInstance Instance of BCC driver.
  * @param txBuf       Pointer to TX data buffer (of BCC_MSG_SIZE size).
  * @param rxBuf       Pointer to RX data buffer (of BCC_MSG_SIZE size).
  *
  * @return bcc_status_t Error code.
  */
bcc_status_t BCC_MCU_TransferSpi(const uint8_t drvInstance, uint8_t txBuf[], uint8_t rxBuf[]);

// NOTE: Unused since we don't use Tpl
bcc_status_t BCC_MCU_TransferTpl(const uint8_t drvInstance, uint8_t txBuf[],
    uint8_t rxBuf[], const uint16_t rxTrCnt);

#define BCC_MCU_Assert(expr) \
  do { \
    if(!(expr)) { \
      ErrorHandler(glue("BCC assert fail: ", stringify(expr))); \
    } \
  } while(0)

/*!
 * @brief Writes logic 0 or 1 to the CSB (SPI mode) or CSB_TX pin (TPL mode).
 *
 * @param drvInstance Instance of BCC driver.
 * @param value       Zero or one to be set to CSB (CSB_TX) pin.
 */
void BCC_MCU_WriteCsbPin(const uint8_t drvInstance, const uint8_t value);

/*!
 * @brief Writes logic 0 or 1 to the RST pin.
 *
 * @param drvInstance Instance of BCC driver.
 * @param value       Zero or one to be set to RST pin.
 */
void BCC_MCU_WriteRstPin(const uint8_t drvInstance, const uint8_t value);

// NOTE: Unused since we don't use Tpl
void BCC_MCU_WriteEnPin(const uint8_t drvInstance, const uint8_t value);
// NOTE: Unused since we don't use Tpl
uint32_t BCC_MCU_ReadIntbPin(const uint8_t drvInstance);

#ifdef BCC_STLIB_IMPLEMENTATION
uint32_t BCC_MCU_GetSystemClockFreq(void)
{
  return SystemCoreClock;
}

void BCC_MCU_WaitSec(uint16_t delay)
{
  HAL_Delay((uint32_t)delay * 1000UL);
}

void BCC_MCU_WaitMs(uint16_t delay)
{
  HAL_Delay((uint32_t)delay);
}

void BCC_MCU_WaitUs(uint32_t delay)
{
  // TODO: Use timer
}


bcc_status_t BCC_MCU_StartTimeout(uint32_t timeoutUs)
{
  // TODO: Use dedicated timer ?
}

bool BCC_MCU_TimeoutExpired(void)
{
  // TODO
}

bcc_status_t BCC_MCU_TransferSpi(const uint8_t drvInstance, uint8_t txBuf[], uint8_t rxBuf[])
{
  BCC_MCU_Assert(txBuf != NULL);
  BCC_MCU_Assert(rxBuf != NULL);

  bool ok = SPI.template transceive<uint8_t[BCC_MSG_SIZE], uint8_t[BCC_MSG_SIZE]>(txBuf, rxBuf);

  return ok ? BCC_STATUS_SUCCESS : BCC_STATUS_SPI_FAIL;
}

bcc_status_t BCC_MCU_TransferTpl(const uint8_t drvInstance, uint8_t txBuf[],
    uint8_t rxBuf[], const uint16_t rxTrCnt)
{
  return BCC_STATUS_SPI_FAIL;
}

void BCC_MCU_WriteEnPin(const uint8_t drvInstance, const uint8_t value)
{
  BCC_MCU_Assert(false && !"Used tpl function when using spi");
}

uint32_t BCC_MCU_ReadIntbPin(const uint8_t drvInstance)
{
  return 0;
}

#include "../../../../deps/BCC_SW_Driver/bcc/bcc.c"
#include "../../../../deps/BCC_SW_Driver/bcc/bcc_communication.c"
#endif // BCC_STLIB_IMPLEMENTATION
#endif // BCC_STLIB_H