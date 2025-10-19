/**
 * @file    chassis_mecanum4.h
 * @author  syhanjin
 * @date    2025-10-17
 * @brief   the 4 wheels Mecanum chassis controller
 */
#ifndef CHASSIS_MECANUM4_H
#define CHASSIS_MECANUM4_H
#include <stdbool.h>

/**
 * Dependence: https://github.com/HITSZ-WTR2026/motor_drivers
 */
#include "interfaces/motor_if.h"

#include <math.h>
/**
 * rad/s to round/min
 * @param __RPS__ rad/s
 */
#define RPS2RPM(__RPS__) ((__RPS__) * 60.0f / (2 * M_PI))

#define DEG2RAD(__DEG__) ((__DEG__) * M_PI / 180.0f)

typedef enum
{
    MECANUM4_WHEEL_FR = 0U, ///< 右前轮
    MECANUM4_WHEEL_FL,      ///< 左前轮
    MECANUM4_WHEEL_RL,      ///< 左后轮
    MECANUM4_WHEEL_RR,      ///< 右后轮
    MECANUM4_WHEEL_MAX
} Mecanum4_WheelType_t;

typedef struct
{
    float vx;    ///< 指向车体前方 (unit: m/s)
    float vy;    ///< 指向车体左侧 (unit: m/s)
    float omega; ///< 向上（逆时针）为正 (unit: deg/s)
} Mecanum4_Velocity_t;

/**
 * @enum    Mecanum4_ChassisType_t
 * @brief   the 4 wheels Mecanum chassis layout type
 *
 * @attention 从地面侧向上看（即与地面接触侧的滚轮的构型）
 */
typedef enum
{
    MECANUM4_X_TYPE = 0U, ///< X 型布局
    MECANUM4_O_TYPE,      ///< O 型布局
} Mecanum4_ChassisType_t;

typedef struct
{
    bool heading_lock; ///< 是否锁定航向

    float                  wheel_radius; ///< 轮子半径 (unit: m)
    float                  k_omega;      ///< O 型：半宽 + 半高；X 型：半宽 - 半高 (unit: m)
    Mecanum4_ChassisType_t chassis_type; ///< 底盘构型
    Motor_VelCtrl_t*       wheel[MECANUM4_WHEEL_MAX];
    float                  last_wheel_angle[MECANUM4_WHEEL_MAX];

    Mecanum4_Velocity_t velocity;
} Mecanum4_t;

typedef struct
{
    bool heading_lock; ///< 设置速度后是否锁定航向

    float                  wheel_radius;     ///< 轮子半径 (unit: mm)
    float                  wheel_distance_x; ///< 左右轮距 (unit: mm)
    float                  wheel_distance_y; ///< 前后轮距 (unit: mm)
    Mecanum4_ChassisType_t chassis_type;     ///< 底盘构型

    Motor_VelCtrl_t* wheel_front_right; ///< 右前方
    Motor_VelCtrl_t* wheel_front_left;  ///< 左前方
    Motor_VelCtrl_t* wheel_rear_left;   ///< 左后方
    Motor_VelCtrl_t* wheel_rear_right;  ///< 右后方
} Mecanum4_Config_t;

void Mecanum4_Init(Mecanum4_t* chassis, Mecanum4_Config_t config);
void Mecanum4_SetVelocity(Mecanum4_t* chassis, Mecanum4_Velocity_t velocity);
void Mecanum4_ControlUpdate(Mecanum4_t* chassis);

#endif // CHASSIS_MECANUM4_H
