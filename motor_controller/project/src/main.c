/* add user code begin Header */
/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
  **************************************************************************
  * Copyright (c) 2025, Artery Technology, All rights reserved.
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* add user code end Header */

/* Includes ------------------------------------------------------------------*/
#include "at32m412_416_wk_config.h"
#include "wk_adc.h"
#include "wk_can.h"
#include "wk_spi.h"
#include "wk_tmr.h"
#include "wk_usart.h"
#include "wk_dma.h"
#include "wk_gpio.h"
#include "wk_system.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include "mc_interface.h"
#include "logger.h"
#include "Debug.h"
#include "dwt_profile_delay.h"
#include "storage/eeprom.h"
#include "storage/flash.h"
#include "motor_config.h"
#include "motor_config_storage_ops.h"
/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */
/* Current image is linked at 0x08000000 (see .map), so VTOR offset must be 0.
 * If using a bootloader offset (e.g. 0x1000), linker/scatter must also relocate
 * the whole image and vector table to the same flash base. */
#define APP_VECTOR_TABLE_OFFSET 0x0000 // 0x1000 for bootloader
/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */

/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  /* add user code begin 1 */
  nvic_vector_table_set(NVIC_VECTTAB_FLASH, APP_VECTOR_TABLE_OFFSET);
  /* add user code end 1 */

  /* system clock config. */
  wk_system_clock_config();

  /* config periph clock. */
  wk_periph_clock_config();

  /* nvic config. */
  wk_nvic_config();

  /* timebase config for
     void wk_delay_ms(uint32_t delay); */
  wk_timebase_init();

  /* init gpio function. */
  wk_gpio_config();

  /* init adc-common function. */
  wk_adc_common_init();

  /* init adc1 function. */
  wk_adc1_init();

  /* init adc2 function. */
  wk_adc2_init();

  /* init dma1 channel1 */
  wk_dma1_channel1_init();
  /* config dma channel transfer parameter */
  /* user need to modify define values DMAx_CHANNELy_XXX_BASE_ADDR 
     and DMAx_CHANNELy_BUFFER_SIZE in at32xxx_wk_config.h */
  wk_dma_channel_config(DMA1_CHANNEL1, 
                        (uint32_t)&ADC1->odt, 
                        DMA1_CHANNEL1_MEMORY_BASE_ADDR, 
                        DMA1_CHANNEL1_BUFFER_SIZE);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  /* init usart1 function. */
  wk_usart1_init();

  /* init can1 function. */
  wk_can1_init();

  /* init spi2 function. */
  wk_spi2_init();

  /* init tmr1 function. */
  wk_tmr1_init();

  /* init tmr6 function. */
  wk_tmr6_init();

  /* init tmr7 function. */
  wk_tmr7_init();

  /* init dma1 channel3 */
  wk_dma1_channel3_init();
  /* DMA1 channel3 is configured on-demand by logger/streaming sender. */

  /* add user code begin 2 */
  Logger_Init(LOG_OUTPUT_UART);
  LOG_INFO("Logger initialized");
  dwt_init();

  motor_cfg_storage_adapter_t motor_cfg_adapter = {
    .eeprom_read = eeprom_read,
    .eeprom_write = eeprom_write,
    .flash_read = flash_read,
    .flash_erase = flash_erase,
    .flash_write = flash_write,
  };

  // 0. 注册配置存储后端（支持三种模式，见 motor_config_storage_ops.h）
  (void)motor_config_register_storage_ops_by_mode(MOTOR_CFG_STORAGE_MODE_DEFAULT,
                                                   &motor_cfg_adapter);

  // 1. 读取电机配置
  motor_config_t motor_cfg;
  (void)motor_config_init(&motor_cfg);

  // 2. 通信初始化

  if (mc_init(&motor_cfg) != MC_INIT_OK)
  {
    while (1)
    {
      /* TODO: 上报 gate driver 初始化失败 */
    }
  }

  LOG_INFO("Motor controller initialized");
  /* add user code end 2 */

  while(1)
  {
    /* add user code begin 3 */
    mc_poll();
    VoFaDisUart();
    /* add user code end 3 */
  }
}

  /* add user code begin 4 */

  /* add user code end 4 */
