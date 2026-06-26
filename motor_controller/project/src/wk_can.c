/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_can.c
  * @brief    work bench config program
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
#include "wk_can.h"

/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief  init can1 function.
  * @param  none
  * @retval none
  */
void wk_can1_init(void)
{
  /* add user code begin can1_init 0 */

  /* add user code end can1_init 0 */

  gpio_init_type gpio_init_struct;
  can_bittime_type can_bittime_struct;
  can_filter_config_type can_filter_struct;

  /* add user code begin can1_init 1 */

  /* add user code end can1_init 1 */

  /*gpio-----------------------------------------------------------------------------*/ 
  gpio_default_para_init(&gpio_init_struct);

  /* configure the CAN1 TX pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE3, GPIO_MUX_9);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_3;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure the CAN1 RX pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE2, GPIO_MUX_9);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_2;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  crm_can_clock_select(CRM_CAN1, CRM_CAN_CLOCK_SOURCE_PLL);

  can_software_reset(CAN1, TRUE);

  /*can_bit_time_setting-------------------------------------------------------------*/
  can_bittime_default_para_init(&can_bittime_struct);

  /*set boudrate = pclk/(bittime_div *(bts1_size + bts2_size))-----------------------*/
  can_bittime_struct.bittime_div = 1;
  can_bittime_struct.ac_bts1_size = 144;
  can_bittime_struct.ac_bts2_size = 36;
  can_bittime_struct.ac_rsaw_size = 36;
  can_bittime_struct.fd_bts1_size  = 27;
  can_bittime_struct.fd_bts2_size = 9;
  can_bittime_struct.fd_rsaw_size = 9;
  can_bittime_struct.fd_ssp_offset = 28;
  can_bittime_set(CAN1, &can_bittime_struct);

  /* enable the ISO 11898-1:2015 protocol mode of CAN-FD */
  can_fd_iso_mode_enable(CAN1, TRUE);

  /*can_filter_0_config--------------------------------------------------------------*/
  can_filter_default_para_init(&can_filter_struct);

  can_filter_struct.mask_para.id_type = FALSE;
  can_filter_struct.code_para.id_type = CAN_ID_STANDARD;
  can_filter_struct.mask_para.id = 0x000;
  can_filter_struct.code_para.id = 0x000;
  can_filter_struct.mask_para.data_length = 0xF;
  can_filter_struct.code_para.data_length = 0x0;
  can_filter_struct.mask_para.frame_type = TRUE;
  can_filter_struct.code_para.frame_type = CAN_FRAME_DATA;
  can_filter_struct.mask_para.recv_frame = TRUE;
  can_filter_struct.code_para.recv_frame = CAN_RECV_NORMAL;
  can_filter_struct.mask_para.fd_format = TRUE;
  can_filter_struct.code_para.fd_format = CAN_FORMAT_CLASSIC;
  can_filter_struct.mask_para.fd_rate_switch = TRUE;
  can_filter_struct.code_para.fd_rate_switch = CAN_BRS_OFF;
  can_filter_struct.mask_para.fd_error_state = TRUE;
  can_filter_struct.code_para.fd_error_state = CAN_ESI_ACTIVE;
  can_filter_set(CAN1, CAN_FILTER_NUM_0, &can_filter_struct);

  /*can_filter_1_config--------------------------------------------------------------*/
  can_filter_default_para_init(&can_filter_struct);

  can_filter_struct.mask_para.id_type = FALSE;
  can_filter_struct.code_para.id_type = CAN_ID_STANDARD;
  can_filter_struct.mask_para.id = 0x7FF;
  can_filter_struct.code_para.id = 0x7FF;
  can_filter_struct.mask_para.data_length = 0xF;
  can_filter_struct.code_para.data_length = 0x0;
  can_filter_struct.mask_para.frame_type = TRUE;
  can_filter_struct.code_para.frame_type = CAN_FRAME_DATA;
  can_filter_struct.mask_para.recv_frame = TRUE;
  can_filter_struct.code_para.recv_frame = CAN_RECV_NORMAL;
  can_filter_struct.mask_para.fd_format = TRUE;
  can_filter_struct.code_para.fd_format = CAN_FORMAT_CLASSIC;
  can_filter_struct.mask_para.fd_rate_switch = TRUE;
  can_filter_struct.code_para.fd_rate_switch = CAN_BRS_OFF;
  can_filter_struct.mask_para.fd_error_state = TRUE;
  can_filter_struct.code_para.fd_error_state = CAN_ESI_ACTIVE;
  can_filter_set(CAN1, CAN_FILTER_NUM_1, &can_filter_struct);

  can_software_reset(CAN1, FALSE);

  can_filter_enable(CAN1, CAN_FILTER_NUM_0, TRUE);
  can_filter_enable(CAN1, CAN_FILTER_NUM_1, TRUE);

  /*can_base_config------------------------------------------------------------------*/
  can_retransmission_limit_set(CAN1, CAN_RE_TRANS_TIMES_UNLIMIT);
  can_rearbitration_limit_set(CAN1, CAN_RE_ARBI_TIMES_UNLIMIT);
  can_mode_set(CAN1, CAN_MODE_COMMUNICATE);
  can_stb_transmit_mode_set(CAN1, CAN_STB_TRANSMIT_BY_FIFO);
  can_rxbuf_warning_set(CAN1, 1);
  can_rxbuf_overflow_mode_set(CAN1, CAN_RXBUF_OVERFLOW_BE_OVWR);
  can_error_warning_set(CAN1, 11);
  can_restricted_operation_enable(CAN1, FALSE);
  can_receive_all_enable(CAN1, FALSE);

  /* enable error interrupt */
  can_interrupt_enable(CAN1, CAN_EIE_INT, TRUE);

  /* enable rxbuf almost full interrupt */
  can_interrupt_enable(CAN1, CAN_RAFIE_INT, TRUE);

  /* enable rxbuf full interrupt */
  can_interrupt_enable(CAN1, CAN_RFIE_INT, TRUE);

  /* enable rxbuf overflow interrupt */
  can_interrupt_enable(CAN1, CAN_ROIE_INT, TRUE);

  /* enable receiver interrupt */
  can_interrupt_enable(CAN1, CAN_RIE_INT, TRUE);

  /* enable bus error interrupt */
  can_interrupt_enable(CAN1, CAN_BEIE_INT, TRUE);

  /* enable error passive interrupt */
  can_interrupt_enable(CAN1, CAN_EPIE_INT, TRUE);

  /* add user code begin can1_init 2 */

  /* add user code end can1_init 2 */
}

/* add user code begin 1 */

/* add user code end 1 */
