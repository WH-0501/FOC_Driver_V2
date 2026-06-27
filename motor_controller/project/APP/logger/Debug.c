#include "Debug.h"
#include "datatypes.h"
#include "../../Board/board.h"

unsigned char UartTemp[28];
SENDDATA Send;

extern motor_handle_t g_motor;

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

    motor_phase_current_t phase_current = g_motor.state.phase_current;

    Send.Data[0].FloatData = phase_current.adc_raw[0]; // phase_current.ampere[0];
    Send.Data[1].FloatData = phase_current.adc_raw[1];
    Send.Data[2].FloatData = phase_current.adc_raw[2];
    // Send.Data[3].FloatData = phase_current.ampere[0] + phase_current.ampere[1] + phase_current.ampere[2];
    Send.Data[3].FloatData = phase_current.adc_offset[0];
    Send.Data[4].FloatData = phase_current.adc_offset[1];
    Send.Data[5].FloatData = phase_current.adc_offset[2];

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
        
    UartTemp[24] = 0x00;
    UartTemp[25] = 0x00;
    UartTemp[26] = 0x80;
    UartTemp[27] = 0x7f;

    (void)board_uart_tx_try(UartTemp, 28u);
}