#include "Debug.h"
#include "motor.h"
#include "encoder.h"
#include "common.h"
#include "../../Board/board.h"

unsigned char UartTemp[32];
SENDDATA Send;

void VoFaDisUart(void)
{
    static uint8_t s_vofa_stream_enabled = 0u;

    if (s_vofa_stream_enabled == 0u)
    {
        board_uart_stream_mode_set(1u);
        s_vofa_stream_enabled = 1u;
    }

    if (board_uart_tx_busy() != 0u)
    {
        return;
    }

    motor_handle_t *m = motor_get_handler();

    if (m == NULL)
    {
        return;
    }

    encoder_shaft_debug_t enc_dbg;

    __disable_irq();
    (void)encoder_get_shaft_debug(m, &enc_dbg);
    /* VOFA+ 通道（FireWater 浮点，建议 ch0/ch1 叠加对比）：
     * ch0 机械角 raw | ch1 机械角 filt（PLL 时平滑，默认时与 raw 相同）
     * ch2 角误差/PLL拒识 | ch3 电角 filt（FOC 使用）
     * ch4 速度 raw | ch5 速度 filt（LPF 或 PLL）
     * ch6 i_q（滤波收益：电流毛刺应减小）
     */
//     Send.Data[0].FloatData = enc_dbg.angle_rad_raw;
//     Send.Data[1].FloatData = enc_dbg.angle_rad_filt;
// #if FOC_ENCODER_USE_PLL
//     Send.Data[2].FloatData = (float)enc_dbg.pll_innovation_rejected;
// #else
//     Send.Data[2].FloatData = enc_dbg.angle_error_rad;
// #endif
//     Send.Data[3].FloatData = enc_dbg.theta_elec_filt_rad;
//     Send.Data[4].FloatData = enc_dbg.velocity_rad_s_raw;
//     Send.Data[5].FloatData = enc_dbg.velocity_rad_s_filt;
//     Send.Data[6].FloatData = m->state.i_q;

    Send.Data[0].FloatData = m->state.shaft.raw_counts;
    Send.Data[1].FloatData = m->state.shaft.counts;
    // Send.Data[1].FloatData = enc_dbg.theta_elec_filt_rad;
    Send.Data[2].FloatData = enc_dbg.angle_error_rad;
    Send.Data[3].FloatData = enc_dbg.position_rad_raw;
    Send.Data[4].FloatData = enc_dbg.position_rad_filt;
    Send.Data[5].FloatData = enc_dbg.velocity_rad_s_raw;
    Send.Data[6].FloatData = enc_dbg.velocity_rad_s_filt;
    Send.Data[7].FloatData = m->state.i_q;

    __enable_irq();

    UartTemp[0] = Send.Data[0].ByteData[0]; //The first data
    UartTemp[1] = Send.Data[0].ByteData[1]; //
    UartTemp[2] = Send.Data[0].ByteData[2]; //
    UartTemp[3] = Send.Data[0].ByteData[3]; //

    UartTemp[4] = Send.Data[1].ByteData[0]; //The second data
    UartTemp[5] = Send.Data[1].ByteData[1]; //
    UartTemp[6] = Send.Data[1].ByteData[2]; //
    UartTemp[7] = Send.Data[1].ByteData[3]; //

    UartTemp[8] =  Send.Data[2].ByteData[0]; //The third data
    UartTemp[9] =  Send.Data[2].ByteData[1]; //
    UartTemp[10] = Send.Data[2].ByteData[2]; //
    UartTemp[11] = Send.Data[2].ByteData[3]; //

    UartTemp[12] = Send.Data[3].ByteData[0]; //The fourth data
    UartTemp[13] = Send.Data[3].ByteData[1]; //
    UartTemp[14] = Send.Data[3].ByteData[2]; //
    UartTemp[15] = Send.Data[3].ByteData[3]; //
        
    UartTemp[16] = Send.Data[4].ByteData[0]; //The fifth data
    UartTemp[17] = Send.Data[4].ByteData[1]; //
    UartTemp[18] = Send.Data[4].ByteData[2]; //
    UartTemp[19] = Send.Data[4].ByteData[3]; //

    UartTemp[20] = Send.Data[5].ByteData[0]; //The sixth data
    UartTemp[21] = Send.Data[5].ByteData[1]; //
    UartTemp[22] = Send.Data[5].ByteData[2]; //
    UartTemp[23] = Send.Data[5].ByteData[3]; //

    UartTemp[24] = Send.Data[6].ByteData[0]; //The seventh data
    UartTemp[25] = Send.Data[6].ByteData[1]; //
    UartTemp[26] = Send.Data[6].ByteData[2]; //
    UartTemp[27] = Send.Data[6].ByteData[3]; //
        
    UartTemp[28] = 0x00;
    UartTemp[29] = 0x00;
    UartTemp[30] = 0x80;
    UartTemp[31] = 0x7f;

    (void)board_uart_tx_try(UartTemp, 32u);
}
