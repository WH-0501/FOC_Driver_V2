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
#include "board.h"
#include "dwt_profile_delay.h"
#include "foc.h"
#include "gate/gate_driver.h"
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
#define APP_VECTOR_TABLE_OFFSET 0x1000
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

  /* init adc1 function. */
  wk_adc1_init();

  /* init dma1 channel3 */
  wk_dma1_channel3_init();
  /* config dma channel transfer parameter */
  /* user need to modify define values DMAx_CHANNELy_XXX_BASE_ADDR 
     and DMAx_CHANNELy_BUFFER_SIZE in at32xxx_wk_config.h */
  wk_dma_channel_config(DMA1_CHANNEL3, 
                        (uint32_t)&USART1->dt, 
                        DMA1_CHANNEL3_MEMORY_BASE_ADDR, 
                        DMA1_CHANNEL3_BUFFER_SIZE);
  dma_channel_enable(DMA1_CHANNEL3, TRUE);

  /* init tmr6 function. */
  wk_tmr6_init();

  /* init tmr7 function. */
  wk_tmr7_init();

  /* add user code begin 2 */
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

  // 3. board 初始化（ADC 抢占等）
  board_init(&g_motor);

  // 4. gate driver 初始化并注册 active driver（必须先于 foc_init）
  gate_driver_init_t gate_driver_init_cfg = {
    .type = GATE_DRIVER_TYPE_MP6540,
    .hw.port_nSLEEP   = nSLEEP_GPIO_PORT,
    .hw.pin_nSLEEP    = nSLEEP_PIN,
    .hw.port_ENA      = ENA_GPIO_PORT,
    .hw.pin_ENA       = ENA_PIN,
    .hw.port_ENB      = ENB_GPIO_PORT,
    .hw.pin_ENB       = ENB_PIN,
    .hw.port_ENC      = ENC_GPIO_PORT,
    .hw.pin_ENC       = ENC_PIN,
    .hw.port_FAULT    = nFAULT_GPIO_PORT,
    .hw.pin_FAULT     = nFAULT_PIN,
  };

  if (gate_driver_init(&gate_driver_init_cfg) != GATE_DRIVER_INIT_OK)
  {
    while (1)
    {
      /* TODO: 上报故障并进入安全态 */
    }
  }

  // 5. FOC 初始化（末尾含阻塞电流零漂校准）
  foc_init(&motor_cfg);
  /* add user code end 2 */

  while(1)
  {
    /* add user code begin 3 */
    foc_update();
    /* add user code end 3 */
  }
}

  /* add user code begin 4 */

  /* add user code end 4 */
