/**
 * @file    chassis_mecanum4.c
 * @author  syhanjin
 * @date    2025-10-17
 */
#include "chassis_mecanum4.h"

#include <string.h>

/**
 * 初始化四轮麦轮底盘
 *
 * @note 轮子旋转的正方向为：使用普通轮子时让车向前移动的旋转方向
 * @param chassis 底盘
 * @param config 底盘配置
 */
void Mecanum4_Init(Mecanum4_t* chassis, const Mecanum4_Config_t config)
{
    memset(chassis, 0, sizeof(Mecanum4_t));

    chassis->heading_lock = config.heading_lock;
    chassis->chassis_type = config.chassis_type;

    if (config.chassis_type == MECANUM4_O_TYPE)
        chassis->k_omega = config.wheel_distance_x * 1e-3f * 0.5f + config.wheel_distance_y * 1e-3f * 0.5f;
    else if (config.chassis_type == MECANUM4_X_TYPE)
        chassis->k_omega = config.wheel_distance_x * 1e-3f * 0.5f - config.wheel_distance_y * 1e-3f * 0.5f;

    chassis->wheel_radius = config.wheel_radius * 1e-3f;

    chassis->wheel[MECANUM4_WHEEL_FR] = config.wheel_front_right;
    chassis->wheel[MECANUM4_WHEEL_FL] = config.wheel_front_left;
    chassis->wheel[MECANUM4_WHEEL_RL] = config.wheel_rear_left;
    chassis->wheel[MECANUM4_WHEEL_RR] = config.wheel_rear_right;
}

/**
 * 设置底盘速度
 * @param chassis 底盘
 * @param velocity 速度
 */
void Mecanum4_SetVelocity(Mecanum4_t* chassis, const Mecanum4_Velocity_t velocity)
{
    chassis->velocity = velocity;

    if (chassis->chassis_type == MECANUM4_O_TYPE)
    {
        /** Mecanum4 O 型运动学解算
         * w_fr = (+ vx + vy + (w + h) * ω) / r
         * w_fl = (+ vx - vy - (w + h) * ω) / r
         * w_rl = (+ vx + vy - (w + h) * ω) / r
         * w_rr = (+ vx - vy + (w + h) * ω) / r
         */
        Motor_VelCtrl_SetRef(
            chassis->wheel[MECANUM4_WHEEL_FR],
            RPS2RPM((velocity.vx + velocity.vy + chassis->k_omega * DEG2RAD(velocity.omega)) / chassis->wheel_radius));
        Motor_VelCtrl_SetRef(
            chassis->wheel[MECANUM4_WHEEL_FL],
            RPS2RPM((velocity.vx - velocity.vy - chassis->k_omega * DEG2RAD(velocity.omega)) / chassis->wheel_radius));
        Motor_VelCtrl_SetRef(
            chassis->wheel[MECANUM4_WHEEL_RL],
            RPS2RPM((velocity.vx + velocity.vy - chassis->k_omega * DEG2RAD(velocity.omega)) / chassis->wheel_radius));
        Motor_VelCtrl_SetRef(
            chassis->wheel[MECANUM4_WHEEL_RR],
            RPS2RPM((velocity.vx - velocity.vy + chassis->k_omega * DEG2RAD(velocity.omega)) / chassis->wheel_radius));
    }
    else if (chassis->chassis_type == MECANUM4_X_TYPE)
    {
        /** Mecanum4 X 型运动学解算
         * w_fr = (+ vx - vy + (w - h) * ω) / r
         * w_fl = (+ vx + vy - (w - h) * ω) / r
         * w_rl = (+ vx - vy - (w - h) * ω) / r
         * w_rr = (+ vx + vy + (w - h) * ω) / r
         */
        Motor_VelCtrl_SetRef(
            chassis->wheel[MECANUM4_WHEEL_FR],
            RPS2RPM((velocity.vx - velocity.vy + chassis->k_omega * DEG2RAD(velocity.omega)) / chassis->wheel_radius));
        Motor_VelCtrl_SetRef(
            chassis->wheel[MECANUM4_WHEEL_FL],
            RPS2RPM((velocity.vx + velocity.vy - chassis->k_omega * DEG2RAD(velocity.omega)) / chassis->wheel_radius));
        Motor_VelCtrl_SetRef(
            chassis->wheel[MECANUM4_WHEEL_RL],
            RPS2RPM((velocity.vx - velocity.vy - chassis->k_omega * DEG2RAD(velocity.omega)) / chassis->wheel_radius));
        Motor_VelCtrl_SetRef(
            chassis->wheel[MECANUM4_WHEEL_RR],
            RPS2RPM((velocity.vx + velocity.vy + chassis->k_omega * DEG2RAD(velocity.omega)) / chassis->wheel_radius));
    }
}

/**
 * 底盘控制更新函数
 *
 * 本函数自动处理控制逻辑，并依序调用每个轮子的 PID 更新函数
 *
 * @note 推荐控制调用频率 1kHz，调用频率将会影响轮子的 PID 参数
 * @param chassis 底盘
 */
void Mecanum4_ControlUpdate(Mecanum4_t* chassis)
{
    if (chassis->heading_lock)
    {
        float wheel_delta_theta[MECANUM4_WHEEL_MAX];

        for (size_t i = 0; i < MECANUM4_WHEEL_MAX; i++)
        {
            const float current_angle    = Motor_GetAngle(chassis->wheel[i]->motor_type, chassis->wheel[i]->motor);
            wheel_delta_theta[i]         = current_angle - chassis->last_wheel_angle[i];
            chassis->last_wheel_angle[i] = current_angle;
        }

        float delta_theta_rad;
        if (chassis->chassis_type == MECANUM4_O_TYPE)
            delta_theta_rad = DEG2RAD(chassis->wheel_radius / (4 * chassis->k_omega) *
                                      (wheel_delta_theta[MECANUM4_WHEEL_FR] - wheel_delta_theta[MECANUM4_WHEEL_FL] +
                                       wheel_delta_theta[MECANUM4_WHEEL_RR] + wheel_delta_theta[MECANUM4_WHEEL_RL]));
        else if (chassis->chassis_type == MECANUM4_X_TYPE)
            delta_theta_rad = DEG2RAD(chassis->wheel_radius / (4 * chassis->k_omega) *
                                      (wheel_delta_theta[MECANUM4_WHEEL_FR] - wheel_delta_theta[MECANUM4_WHEEL_FL] -
                                       wheel_delta_theta[MECANUM4_WHEEL_RR] + wheel_delta_theta[MECANUM4_WHEEL_RL]));
        else // never
            return;

        // 将车身速度向反方向旋转 delta_theta
        const float _sin_delta_theta = sinf(-delta_theta_rad), _cos_delta_theta = cosf(-delta_theta_rad);
        Mecanum4_SetVelocity(
            chassis, (Mecanum4_Velocity_t){
                         .vx    = chassis->velocity.vx * _cos_delta_theta - chassis->velocity.vy * _sin_delta_theta,
                         .vy    = chassis->velocity.vx * _sin_delta_theta + chassis->velocity.vy * _cos_delta_theta,
                         .omega = chassis->velocity.omega});
    }
    // 更新控制
    for (size_t i = 0; i < MECANUM4_WHEEL_MAX; i++)
    {
        Motor_VelCtrlUpdate(chassis->wheel[i]);
    }
}
