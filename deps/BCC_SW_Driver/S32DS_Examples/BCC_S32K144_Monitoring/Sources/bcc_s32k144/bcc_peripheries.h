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
 * File: bcc_peripheries.h
 *
 * This file implements functions for LPSPI and GPIO operations required by BCC
 * driver. It is closely related to this demo example.
 */

#ifndef BCC_PERIPHERIES_H_
#define BCC_PERIPHERIES_H_

#include "Cpu.h"
#include "bcc_wait.h"
#include "../bcc/bcc.h"

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @addtogroup function_group
 * @{
 */

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
bcc_status_t BCC_MCU_TransferSpi(const uint8_t drvInstance, uint8_t txBuf[],
    uint8_t rxBuf[]);

/*!
 * @brief This function sends and receives data via TX and RX SPI buses.
 * Intended for TPL mode only.
 *
 * TX SPI bus always performs only one 48b SPI transfer. Expected number of RX
 * transfers is passed as a parameter. The byte order of buffers is given by
 * BCC_MSG_* macros (in bcc.h).
 *
 * @param drvInstance Instance of BCC driver.
 * @param txBuf       Pointer to TX data buffer (of BCC_MSG_SIZE size).
 * @param rxBuf       Pointer to buffer for received data. Its size is at least
 *                    (BCC_MSG_SIZE * recvTrCnt) bytes.
 * @param rxTrCnt     Number of 48b transfers to be received. A non-zero value.
 *
 * @return bcc_status_t Error code.
 */
bcc_status_t BCC_MCU_TransferTpl(const uint8_t drvInstance, uint8_t txBuf[],
    uint8_t rxBuf[], const uint16_t rxTrCnt);

/*!
 * @brief User implementation of assert.
 *
 * @param x - True if everything is OK.
 */
void BCC_MCU_Assert(const bool x);

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

/*!
 * @brief Writes logic 0 or 1 to the EN pin of MC33664.
 *
 * @param drvInstance Instance of BCC driver.
 * @param value       Zero or one to be set to EN pin.
 */
void BCC_MCU_WriteEnPin(const uint8_t drvInstance, const uint8_t value);

/*!
 * @brief Reads logic value of INTB pin of MC33664.
 *
 * @param drvInstance Instance of BCC driver.
 *
 * @return Zero value for logic zero, non-zero value otherwise.
 */
uint32_t BCC_MCU_ReadIntbPin(const uint8_t drvInstance);
/*! @} */

#endif /* BCC_PERIPHERIES_H_ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
