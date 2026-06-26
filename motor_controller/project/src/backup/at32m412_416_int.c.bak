/* add user code begin Header */
/**
  **************************************************************************
  * @file     at32m412_416_int.c
  * @brief    main interrupt service routines.
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

/* includes ------------------------------------------------------------------*/
#include "at32m412_416_int.h"
#include "wk_system.h"
/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include "board.h"
#include "../APP/logger/logger.h"
/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

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

/* external variables ---------------------------------------------------------*/
/* add user code begin external variables */

/* add user code end external variables */

/**
  * @brief  this function handles nmi exception.
  * @param  none
  * @retval none
  */
void NMI_Handler(void)
{
  /* add user code begin NonMaskableInt_IRQ 0 */

  /* add user code end NonMaskableInt_IRQ 0 */

  /* add user code begin NonMaskableInt_IRQ 1 */

  /* add user code end NonMaskableInt_IRQ 1 */
}

/**
  * @brief  this function handles hard fault exception.
  * @param  none
  * @retval none
  */
void HardFault_Handler(void)
{
  /* add user code begin HardFault_IRQ 0 */

  /* add user code end HardFault_IRQ 0 */
  /* go to infinite loop when hard fault exception occurs */
  while (1)
  {
    /* add user code begin W1_HardFault_IRQ 0 */

    /* add user code end W1_HardFault_IRQ 0 */
  }
}


/**
  * @brief  this function handles memory manage exception.
  * @param  none
  * @retval none
  */
void MemManage_Handler(void)
{
  /* add user code begin MemoryManagement_IRQ 0 */

  /* add user code end MemoryManagement_IRQ 0 */
  /* go to infinite loop when memory manage exception occurs */
  while (1)
  {
    /* add user code begin W1_MemoryManagement_IRQ 0 */

    /* add user code end W1_MemoryManagement_IRQ 0 */
  }
}

/**
  * @brief  this function handles bus fault exception.
  * @param  none
  * @retval none
  */
void BusFault_Handler(void)
{
  /* add user code begin BusFault_IRQ 0 */

  /* add user code end BusFault_IRQ 0 */
  /* go to infinite loop when bus fault exception occurs */
  while (1)
  {
    /* add user code begin W1_BusFault_IRQ 0 */

    /* add user code end W1_BusFault_IRQ 0 */
  }
}

/**
  * @brief  this function handles usage fault exception.
  * @param  none
  * @retval none
  */
void UsageFault_Handler(void)
{
  /* add user code begin UsageFault_IRQ 0 */

  /* add user code end UsageFault_IRQ 0 */
  /* go to infinite loop when usage fault exception occurs */
  while (1)
  {
    /* add user code begin W1_UsageFault_IRQ 0 */

    /* add user code end W1_UsageFault_IRQ 0 */
  }
}

/**
  * @brief  this function handles svcall exception.
  * @param  none
  * @retval none
  */
void SVC_Handler(void)
{
  /* add user code begin SVCall_IRQ 0 */

  /* add user code end SVCall_IRQ 0 */
  /* add user code begin SVCall_IRQ 1 */

  /* add user code end SVCall_IRQ 1 */
}

/**
  * @brief  this function handles debug monitor exception.
  * @param  none
  * @retval none
  */
void DebugMon_Handler(void)
{
  /* add user code begin DebugMonitor_IRQ 0 */

  /* add user code end DebugMonitor_IRQ 0 */
  /* add user code begin DebugMonitor_IRQ 1 */

  /* add user code end DebugMonitor_IRQ 1 */
}

/**
  * @brief  this function handles pendsv_handler exception.
  * @param  none
  * @retval none
  */
void PendSV_Handler(void)
{
  /* add user code begin PendSV_IRQ 0 */

  /* add user code end PendSV_IRQ 0 */
  /* add user code begin PendSV_IRQ 1 */

  /* add user code end PendSV_IRQ 1 */
}


/**
  * @brief  this function handles systick handler.
  * @param  none
  * @retval none
  */
void SysTick_Handler(void)
{
  /* add user code begin SysTick_IRQ 0 */

  /* add user code end SysTick_IRQ 0 */

  wk_timebase_handler();
  /* add user code begin SysTick_IRQ 1 */

  /* add user code end SysTick_IRQ 1 */
}

/**
  * @brief  this function handles ADC1 2 handler.
  * @param  none
  * @retval none
  */
void ADC1_2_IRQHandler(void)
{
  /* add user code begin ADC1_2_IRQ 0 */

  /* add user code end ADC1_2_IRQ 0 */

  if(adc_interrupt_flag_get(ADC2, ADC_PCCE_FLAG) != RESET)
  {
    /* add user code begin ADC2_ADC_PCCE_FLAG */
    CURRENT_LOOP_IRQ_HANDLER(ADC2);
    /* clear flag */
    adc_flag_clear(ADC2, ADC_PCCE_FLAG);
    /* add user code end ADC2_ADC_PCCE_FLAG */ 
  }

  /* add user code begin ADC1_2_IRQ 1 */

  /* add user code end ADC1_2_IRQ 1 */
}

/**
  * @brief  this function handles CAN1 RX handler.
  * @param  none
  * @retval none
  */
void CAN1_RX_IRQHandler(void)
{
  can_rxbuf_type can_rxbuf_struct;

  /* add user code begin CAN1_RX_IRQ 0 */

  /* add user code end CAN1_RX_IRQ 0 */

  if(can_interrupt_flag_get(CAN1, CAN_RIF_FLAG) != RESET)
  {
    /* add user code begin CAN1_CAN_RIF_FLAG */
    /* clear flag and receive buffer release */
    can_flag_clear(CAN1, CAN_RIF_FLAG);
    can_rxbuf_read(CAN1, &can_rxbuf_struct);
    /* add user code end CAN1_CAN_RIF_FLAG */
  }

  if(can_interrupt_flag_get(CAN1, CAN_RAFIF_FLAG) != RESET)
  {
    /* add user code begin CAN1_CAN_RAFIF_FLAG */
    /* clear flag */
    can_flag_clear(CAN1, CAN_RAFIF_FLAG);
    /* add user code end CAN1_CAN_RAFIF_FLAG */
  }

  if(can_interrupt_flag_get(CAN1, CAN_RFIF_FLAG) != RESET)
  {
    /* add user code begin CAN1_CAN_RFIF_FLAG */
    /* clear flag */
    can_flag_clear(CAN1, CAN_RFIF_FLAG);
    /* add user code end CAN1_CAN_RFIF_FLAG */
  }

  if(can_interrupt_flag_get(CAN1, CAN_ROIF_FLAG) != RESET)
  {
    /* add user code begin CAN1_CAN_ROIF_FLAG */
    /* clear flag */
    can_flag_clear(CAN1, CAN_ROIF_FLAG);
    /* add user code end CAN1_CAN_ROIF_FLAG */
  }

  /* add user code begin CAN1_RX_IRQ 1 */

  /* add user code end CAN1_RX_IRQ 1 */
}

/**
  * @brief  this function handles CAN1 ERR handler.
  * @param  none
  * @retval none
  */
void CAN1_ERR_IRQHandler(void)
{
  /* add user code begin CAN1_ERR_IRQ 0 */

  /* add user code end CAN1_ERR_IRQ 0 */

  if(can_interrupt_flag_get(CAN1, CAN_BEIF_FLAG) != RESET)
  {
    /* add user code begin CAN1_CAN_BEIF_FLAG */
    /* clear flag */
    can_flag_clear(CAN1, CAN_BEIF_FLAG);
    /* add user code end CAN1_CAN_BEIF_FLAG */
  }

  if(can_interrupt_flag_get(CAN1, CAN_EIF_FLAG) != RESET)
  {
    /* add user code begin CAN1_CAN_EIF_FLAG */
    /* clear flag */
    can_flag_clear(CAN1, CAN_EIF_FLAG);
    /* add user code end CAN1_CAN_EIF_FLAG */
  }

  if(can_interrupt_flag_get(CAN1, CAN_EPIF_FLAG) != RESET)
  {
    /* add user code begin CAN1_CAN_EPIF_FLAG */
    /* clear flag */
    can_flag_clear(CAN1, CAN_EPIF_FLAG);
    /* add user code end CAN1_CAN_EPIF_FLAG */
  }

  /* add user code begin CAN1_ERR_IRQ 1 */

  /* add user code end CAN1_ERR_IRQ 1 */
}

/**
  * @brief  this function handles TMR7 handler.
  * @param  none
  * @retval none
  */
void TMR7_GLOBAL_IRQHandler(void)
{
  /* add user code begin TMR7_GLOBAL_IRQ 0 */

  /* add user code end TMR7_GLOBAL_IRQ 0 */

  /* add user code begin TMR7_GLOBAL_IRQ 1 */

  /* add user code end TMR7_GLOBAL_IRQ 1 */
}

/**
  * @brief  this function handles DMA1 channel3 global interrupt.
  * @param  none
  * @retval none
  */
void DMA1_Channel3_IRQHandler(void)
{
  logger_dma_irq_handler();
}

/* add user code begin 1 */

/* add user code end 1 */
