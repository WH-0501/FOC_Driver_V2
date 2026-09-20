#include "Debug.h"
#include "motor.h"
#include "common.h"
#include "../../Board/board.h"

#define VOFA_FRAME_TAIL_SIZE 4u
#define VOFA_FRAME_SIZE      (VOFA_CHANNEL_COUNT * 4u + VOFA_FRAME_TAIL_SIZE)

unsigned char UartTemp[VOFA_FRAME_SIZE];
SENDDATA Send;

void VoFaDisUart(void)
{
    static uint8_t s_vofa_stream_enabled = 0u;
    uint32_t i;
    uint32_t offset;

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

    __disable_irq();
    Send.Data[0].FloatData = m->out.duty_a;
    Send.Data[1].FloatData = m->out.duty_b;
    Send.Data[2].FloatData = m->out.duty_c;
    Send.Data[3].FloatData = m->out.duty_a - 0.5f;
    Send.Data[4].FloatData = m->out.duty_b - 0.5f;
    Send.Data[5].FloatData = m->out.duty_c - 0.5f;
    Send.Data[6].FloatData = m->state.phase_current.ampere[0];
    Send.Data[7].FloatData = m->state.phase_current.ampere[1];
    Send.Data[8].FloatData = m->state.phase_current.ampere[2];
    Send.Data[9].FloatData = m->state.phase_current.kcl_residual;
    Send.Data[10].FloatData = m->state.i_q;
    Send.Data[11].FloatData = m->state.shaft.counts;
    __enable_irq();

    for (i = 0u; i < VOFA_CHANNEL_COUNT; i++)
    {
        offset = i * 4u;
        UartTemp[offset] = Send.Data[i].ByteData[0];
        UartTemp[offset + 1u] = Send.Data[i].ByteData[1];
        UartTemp[offset + 2u] = Send.Data[i].ByteData[2];
        UartTemp[offset + 3u] = Send.Data[i].ByteData[3];
    }

    offset = VOFA_CHANNEL_COUNT * 4u;
    UartTemp[offset] = 0x00;
    UartTemp[offset + 1u] = 0x00;
    UartTemp[offset + 2u] = 0x80;
    UartTemp[offset + 3u] = 0x7f;

    (void)board_uart_tx_try(UartTemp, VOFA_FRAME_SIZE);
}
