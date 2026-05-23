#include "foc_motor.h"
#include "foc_filter.h"
#include "board.h"
#include "at32m412_416_tmr.h"

extern motor_handle_t g_motor;

static int32_t full_rotations = 0;

void foc_motor_init(void)
{
    // TODO: 初始化电机
}

void foc_get_motor_angle(void)
{
    unsigned short angle_raw;
    KTH71_ReadAngle(&angle_raw);

    // 机械角度. 映射到 [0, 2π]
    float angle = (float)angle_raw * RAW_ANGLE_U16_TO_RAD;

    g_motor.state.theta_elec_rad = angle_normalize((angle * POLE_PAIRS * DIR) - g_motor.config.param.theta_elec_offset_rad);
    
    static float last_angle = 0;
    float diff_angle = angle - last_angle;

    if (fabsf(diff_angle) > DIFF_ANGLE_WRAP_THRESH_RAD)
    {
        full_rotations = full_rotations + ((diff_angle > 0) ? -1 :1);
    }

    /**
     * angle / _2PI: 当前角度占一圈的百分比
     * full_rotations + angle / _2PI: 电机轴累积转了多少圈的浮点表示
     * (full_rotations + angle / _2PI) * 360: 电机轴累积转了多少圈的机械角度(°)
     * g_motor.config.param.theta_elec_offset_rad: 电角度偏移
     */
    g_motor.state.position_rad = (full_rotations + angle / _2PI) * 360 - g_motor.config.param.theta_offset_rad;
    g_motor.state.theta_joint_rad = g_motor.state.position_rad / GEAR_RATIO; // 关节角度 = 电角度 * 减速比

    last_angle = angle;
}

/**
 * @brief 获取电机速度
 * 
 * @return float 电机速度. 机械角度速度 [rad/s]
 */
void foc_get_motor_speed(void)
{
    unsigned short delta_tim;
    static unsigned short last_vel_us = 0;
    static float last_angle = 0;

    unsigned short vel_us = TMR6->cval;

    if (vel_us < last_vel_us)
    {
        delta_tim = vel_us + 0xFFFF - last_vel_us;
    }
    else
    {
        delta_tim = vel_us - last_vel_us;
    }

    if (delta_tim < 1) delta_tim = 1;

    float velocity = (g_motor.state.position_rad - last_angle) / (float)delta_tim * 1000000.0f;
    
    last_angle = g_motor.state.position_rad;
    last_vel_us = vel_us;

    // deg/s 转 RPM: (deg/s) * 60s/min / 360°/rev = deg/s / 6
    velocity = velocity / 6; 

    g_motor.state.speed_rad_s = lpf1_update(&g_motor.speed_rad_s_lpf, velocity);
}

void foc_get_motor_current(void)
{
    board_get_phase_current(&g_motor);

    clarke_transform(g_motor.state.phase_current.ampere[0], /* ia */
        g_motor.state.phase_current.ampere[1], /* ib */
        g_motor.state.phase_current.ampere[2], /* ic */
        &g_motor.state.i_alpha, &g_motor.state.i_beta);

    float id = 0.0f, iq = 0.0f;
    park_transform(g_motor.state.i_alpha, g_motor.state.i_beta, 
        g_motor.state.theta_elec_rad,
        &id, &iq); /* 更新当前 d、q 轴电流 */

    g_motor.state.i_d = lpf1_update(&g_motor.i_d_lpf, id);
    g_motor.state.i_q = lpf1_update(&g_motor.i_q_lpf, iq);

    float i_mod = sqrtf(SQ(g_motor.state.i_d) + SQ(g_motor.state.i_q)) * 1000.0f; // mA
    g_motor.state.i_mod = lpf1_update(&g_motor.i_mod_lpf, i_mod);
}
