/*
 * Copyright 2016 - 2019 NXP
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

/*
 * File: bcc_wait.c
 *
 * This file implements functions for busy waiting for BCC driver.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "bcc_wait.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

#if defined(__thumb__) && !defined(__thumb2__) /* Thumb instruction set only */
/**
 * @brief Waits for exact number of cycles which can be expressed as multiple of 4.
 *
 * MOV - 1 cycle
 * SUB - 1 cycle
 * BNE - 1 cycle or 2 cycles if jump is realized
 *
 * Output list (empty) - which registers are output and how to map them to C code.
 * Input list (Cycles) - which registers are input and how to map them to C code.
 * Clobber list (r0, r1, cc) - which registers might have changed during 
 * execution of asm code (compiler will have to reload them).
 *
 * @param Cycles | Number of cycles to wait.
 */
#define BCC_WAIT_FOR_MUL4_CYCLES(cycles) \
  __asm( \
    "mov r0, %[cycles] \n\t" \
    "0: \n\t"                \
      "sub r0, #4 \n\t"      \
      "nop \n\t"             \
    "bne 0b \n\t"            \
     :                       \
     : [cycles] "r" (cycles) \
     : "r0", "r1", "cc"      \
  )
  
#else /* Thumb2 or A32 instruction set */

/**
 * @brief Waits for exact number of cycles which can be expressed as multiple of 4.
 *
 * @param Cycles | Number of cycles to wait.
 */
#define BCC_WAIT_FOR_MUL4_CYCLES(cycles) \
  __asm( \
    "movs r0, %[cycles] \n"  \
    "0: \n"                  \
      "subs r0, r0, #4 \n"   \
      "nop \n\t"             \
    "bne 0b \n"              \
     :                       \
     : [cycles] "r" (cycles) \
     : "r0", "r1", "cc"      \
  )
  
#endif

/*******************************************************************************
 * Global variables (constants)
 ******************************************************************************/

/**
 * System clock frequency.
 */
uint32_t g_sysClk = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_GetSystemClockFreq
 * Description   : Returns SCG system clock frequency.
 *
 *END**************************************************************************/
uint32_t BCC_MCU_GetSystemClockFreq(void)
{
    uint32_t freq;
    CLOCK_SYS_GetFreq(CORE_CLOCK, &freq);
    return freq;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WaitSec
 * Description   : Waits for specified amount of seconds.
 *
 *END**************************************************************************/
void BCC_MCU_WaitSec(uint16_t delay)
{
    for (; delay > 0U; delay--) {
        BCC_MCU_WaitMs(1000U);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WaitMs
 * Description   : Waits for specified amount of milliseconds.
 *
 *END**************************************************************************/
void BCC_MCU_WaitMs(uint16_t delay)
{
    g_sysClk = (g_sysClk) ? g_sysClk : BCC_MCU_GetSystemClockFreq();

    uint32_t cycles = (uint32_t) BCC_GET_CYCLES_FOR_MS(1U, g_sysClk);

    /* Advance to next multiple of 4. Value 0x04U ensures that the number
     * is not zero. */
    cycles = (cycles & 0xFFFFFFFCU) | 0x04U;

    for (; delay > 0U; delay--) {
        BCC_WAIT_FOR_MUL4_CYCLES(cycles);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BCC_WaitUs
 * Description   : Waits for specified amount of microseconds.
 *
 *END**************************************************************************/
void BCC_MCU_WaitUs(uint32_t delay)
{
    uint32_t cycles;

    if (delay > 1000)
    {
        BCC_MCU_WaitMs(delay / 1000);
        delay %= 1000;
    }

    g_sysClk = (g_sysClk) ? g_sysClk : BCC_MCU_GetSystemClockFreq();

    /* Correction for 48 MHz core clock. */
    delay = (delay > 2) ? delay - 2 : 0;

    cycles = (uint32_t) BCC_GET_CYCLES_FOR_US(delay, g_sysClk);

    /* Advance to next multiple of 4. Value 0x04U ensures that the number
     * is not zero. */
    cycles = (cycles & 0xFFFFFFFCU) | 0x04U;
    BCC_WAIT_FOR_MUL4_CYCLES(cycles);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
