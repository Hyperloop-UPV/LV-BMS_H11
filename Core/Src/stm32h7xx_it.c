/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
#include "HALAL/Benchmarking_toolkit/HardfaultTrace.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_fmac_preload;
extern DMA_HandleTypeDef hdma_fmac_read;
extern DMA_HandleTypeDef hdma_fmac_write;
extern FMAC_HandleTypeDef hfmac;
extern LPTIM_HandleTypeDef hlptim1;
extern LPTIM_HandleTypeDef hlptim2;
extern LPTIM_HandleTypeDef hlptim3;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim23;
extern TIM_HandleTypeDef htim24;
extern FDCAN_HandleTypeDef hfdcan1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */

//calls my_fault_handler with the MSP(main stack pointer)
#define HARDFAULT_HANDLING_ASM(_x)               \
  __asm volatile(                                \
      "mrseq r0, msp \n"                         \
      "b my_fault_handler_c \n"                  \
    )


 //create the space for the hardfault section in the flash
__attribute__((section(".hardfault_log")))
volatile uint32_t hard_fault[128];

static void flash_unlock(void){
    if((FLASH->CR1 & FLASH_CR_LOCK) != 0){
        FLASH->KEYR1 = FLASH_KEY1;
        FLASH->KEYR1 = FLASH_KEY2;
    }
}
static void flash_lock(void){
    FLASH->CR1 |= FLASH_CR_LOCK;
}
static void flash_wait(void){
    while(FLASH->SR1 & FLASH_SR_QW){}
}
static void flash_write_32bytes(uint32_t address,const uint8_t* data){
  if(address & 0X3U){
    return; // the address direction must be aligned 4;
  }
  flash_wait();
  flash_unlock();
  FLASH->CR1 |= FLASH_CR_PG; //Enable writing
  for(volatile size_t i = 0; i < 8; i++){
    *(volatile uint32_t*)(address + i*4) = ((uint32_t*)data)[i];
  }
  flash_wait();
  //Disable writing 
  FLASH->CR1 &= ~FLASH_CR_PG;
  flash_lock();
}
static void flash_write_blockwise(uint32_t address, const uint8_t *data,size_t blocks){
    for(volatile size_t i = 0; i < blocks;i++){
      flash_write_32bytes(address + i*32,data + i*32);
    }
}
// //erase hard_fault sector 6
static void flash_erase_hard_fault_sector(void){
    flash_wait();
    FLASH->CCR1 = 0xFFFFFFFFu;
    flash_unlock();
    
    FLASH->CR1 |= FLASH_CR_SER | (FLASH_SECTOR_6 << FLASH_CR_SNB_Pos);
  //start erase
    FLASH->CR1 |= FLASH_CR_START;
    flash_wait();
    FLASH->CCR1 = 0xFFFFFFFFU;
  // Clean flag
    FLASH->CR1 &= ~FLASH_CR_SER;
    flash_lock();
}
//check if we have already write.
static int hardfault_flag_is_set(void){
  return (*(volatile uint32_t *)HF_FLASH_ADDR) == HF_FLAG_VALUE;
}

#define HALT_IF_DEBUGGING()                               \
  do {                                                    \
    if ((*(volatile uint32_t *)0xE000EDF0) & (1 << 0)) {  \
      __asm("bkpt 1");                                    \
    }                                                     \
  } while (0)    
  //Breakpoint in debug mode                                         


volatile uint32_t real_fault_pc;
__attribute__((noreturn,optimize("O0")))
void my_fault_handler_c(sContextStateFrame *frame) {
  // If and only if a debugger is attached, execute a breakpoint
  // instruction so we can take a look at what triggered the fault
  const uint32_t instr_pc = frame->return_address;
  real_fault_pc = instr_pc & ~1; //clean bit 0, real direction
  volatile HardFaultLog log_hard_fault;
  volatile uint32_t *cfsr = (volatile uint32_t *)0xE000ED28;
  //keep the log in the estructure
  log_hard_fault.HF_flag = HF_FLAG_VALUE;
  log_hard_fault.frame = *frame;
  log_hard_fault.CfsrDecode.cfsr = *cfsr;
  log_hard_fault.fault_address.Nothing_Valid = 0;

  const uint8_t memory_fault = *cfsr & 0x000000ff;
  if(memory_fault){
    const uint8_t MMARVALID = memory_fault & 0b10000000; // We can find the exact place were occured the memory fault
    const uint8_t MLSPERR = memory_fault & 0b00100000; // MemManage fault FPU stack
    const uint8_t MSTKERR = memory_fault & 0b00010000; // Stack overflow while entring an exception
    const uint8_t MUNSTKERR = memory_fault & 0b00001000;  // Stack error while exiting from an exception (Corrupted stack)
    const uint8_t DACCVIOL = memory_fault & 0b00000010; //Data access violation (acceded to pointer NULL, to a protected memory region, overflow in arrays ...)
    const uint8_t IACCVIOL = memory_fault & 0b00000001; //Instruction access violation
    if(MMARVALID){
      uint32_t memory_fault_address = *(volatile uint32_t *)0xE000ED34;
      log_hard_fault.fault_address.MMAR_VALID = memory_fault_address;
    }
    HALT_IF_DEBUGGING();
  }
  const uint8_t bus_fault = (*cfsr & 0x0000ff00) >> 8;
  if(bus_fault){
    const uint8_t BFARVALID = bus_fault & 0b10000000; // BFAR is valid we can know the address which triggered the fault
    const uint8_t LSPERR = bus_fault & 0b00100000; //Fault stack FPU
    const uint8_t STKERR = bus_fault & 0b00010000;  // Fault stack while entring an exception
    const uint8_t UNSTKERR = bus_fault & 0b00001000;  // Stack error while exiting an exception
    const uint8_t IMPRECISERR = bus_fault & 0b00000010; // Bus fault, but the instruction that caused the error can be uncertain
    const uint8_t PRECISERR = bus_fault & 0b00000001; //You can read Bfar to find the eact direction of the instruction
    if(BFARVALID){
      volatile uint32_t bus_fault_address = *(volatile uint32_t *)0xE000ED38;
      log_hard_fault.fault_address.BFAR_VALID = bus_fault_address;
      //Don't trust in case IMPRECISERR == 1;
    }
    HALT_IF_DEBUGGING();
  }
  const uint16_t usage_fault = (*cfsr & 0xffff0000) >> 16;
  if(usage_fault){
    const uint16_t DIVBYZERO = usage_fault & 0x0200; // Div by ZERO hardfault;
    const uint16_t UNALIGNED = usage_fault & 0x0100; // Unaligned access operation occured
    const uint16_t NOCP = usage_fault & 0x0008; //Access to FPU when is not present
    const uint16_t INVPC = usage_fault & 0x0004; //Invalid program counter load
    const uint16_t INVSTATE = usage_fault & 0x0002; // Invalid processor state
    const uint16_t UNDEFINSTR = usage_fault & 0x0001; //Undefined instruction.
    //HALT_IF_DEBUGGING(); 
  }
  volatile uint8_t metadata_buffer[0x100];
  memcpy(metadata_buffer,(void*)METADATA_FLASH_ADDR,0x100);
  flash_erase_hard_fault_sector();
  //write log hard fault
  flash_write_blockwise(HF_FLASH_ADDR,(uint8_t*)&log_hard_fault,(sizeof(log_hard_fault) + 31)/32);
  //write log Metadata_flash_addr
  flash_write_blockwise(METADATA_FLASH_ADDR,metadata_buffer,(0x100)/32);
  //reboot the system
  volatile uint32_t *aircr = (volatile uint32_t *)0xE000ED0C;
  __asm volatile ("dsb");
  *aircr = (0x05FA << 16) | 0x1 << 2;
  __asm volatile ("dsb");
  while (1) {} // should be unreachable
  }

__attribute__((naked))
void HardFault_Handler(void)
{
  HARDFAULT_HANDLING_ASM();
  while (1){}
}

void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}
/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/** 
 * @brief This function handles FDCAN 1 Line 0 interrupt
 */
void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

/** 
 * @brief This function handles FDCAN 1 Line 1 interrupt
 */
void FDCAN1_IT1_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

/** 
 * @brief This function handles FDCAN 3 Line 0 interrupt
 */
void FDCAN3_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

/** 
 * @brief This function handles FDCAN 3 Line 1 interrupt
 */
void FDCAN3_IT1_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&hfdcan1);
}

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc2);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

  /** @brief DMA fmac configuration (hardcoded the handler, this normally is generated by the IDE)
  */
void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_fmac_preload);
}

void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_fmac_write);
}

void DMA2_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_fmac_read);
}

void FMAC_IRQHandler(void)
{
  HAL_FMAC_IRQHandler(&hfmac);
}



/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles SPI3 global interrupt.
  */
void SPI3_IRQHandler(void)
{
  /* USER CODE BEGIN SPI3_IRQn 0 */

  /* USER CODE END SPI3_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi3);
  /* USER CODE BEGIN SPI3_IRQn 1 */

  /* USER CODE END SPI3_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles Ethernet global interrupt.
  */
void ETH_IRQHandler(void)
{
  /* USER CODE BEGIN ETH_IRQn 0 */

  /* USER CODE END ETH_IRQn 0 */
  HAL_ETH_IRQHandler(&heth);
  /* USER CODE BEGIN ETH_IRQn 1 */

  /* USER CODE END ETH_IRQn 1 */
}

/**
  * @brief This function handles LPTIM1 global interrupt.
  */
void LPTIM1_IRQHandler(void)
{
  /* USER CODE BEGIN LPTIM1_IRQn 0 */

  /* USER CODE END LPTIM1_IRQn 0 */
  HAL_LPTIM_IRQHandler(&hlptim1);
  /* USER CODE BEGIN LPTIM1_IRQn 1 */

  /* USER CODE END LPTIM1_IRQn 1 */
}

/**
  * @brief This function handles LPTIM2 global interrupt.
  */
void LPTIM2_IRQHandler(void)
{
  /* USER CODE BEGIN LPTIM2_IRQn 0 */

  /* USER CODE END LPTIM2_IRQn 0 */
  HAL_LPTIM_IRQHandler(&hlptim2);
  /* USER CODE BEGIN LPTIM2_IRQn 1 */

  /* USER CODE END LPTIM2_IRQn 1 */
}

/**
  * @brief This function handles LPTIM3 global interrupt.
  */
void LPTIM3_IRQHandler(void)
{
  /* USER CODE BEGIN LPTIM3_IRQn 0 */

  /* USER CODE END LPTIM3_IRQn 0 */
  HAL_LPTIM_IRQHandler(&hlptim3);
  /* USER CODE BEGIN LPTIM3_IRQn 1 */

  /* USER CODE END LPTIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM23 global interrupt.
  */
void TIM23_IRQHandler(void)
{
  /* USER CODE BEGIN TIM23_IRQn 0 */

  /* USER CODE END TIM23_IRQn 0 */
  HAL_TIM_IRQHandler(&htim23);
  /* USER CODE BEGIN TIM23_IRQn 1 */

  /* USER CODE END TIM23_IRQn 1 */
}

/**
  * @brief This function handles TIM24 global interrupt.
  */
void TIM24_IRQHandler(void)
{
  /* USER CODE BEGIN TIM24_IRQn 0 */

  /* USER CODE END TIM24_IRQn 0 */
  HAL_TIM_IRQHandler(&htim24);
  /* USER CODE BEGIN TIM24_IRQn 1 */

  /* USER CODE END TIM24_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
