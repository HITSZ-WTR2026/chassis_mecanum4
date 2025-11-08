/**
 * @file    app.h
 * @author  syhanjin
 * @date    2025-10-17
 */
#include "app.h"

#include "bsp/can_driver.h"
#include "can.h"
#include "cmsis_os2.h"
#include "tim.h"
#include "usart.h"
#include "../drivers/chassis_mecanum4.h"
#include "drivers/DJI.h"
#include "interfaces/motor_if.h"
#include "drivers/HWT101CT.h"
#include "interfaces/chassis_if.h"

DJI_t           wheel_dji[4];
Motor_VelCtrl_t motor_vel_ctrl[4];
Chassis_t       chassis;
HWT101CT_t      sensor;

void CtrlTIM_Callback(TIM_HandleTypeDef* htim)
{
    // Mecanum4_Update(&chassis_mecanum4);
    Chassis_Update(&chassis);

    DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_1_4);
}

void Uart_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == sensor.huart)
    {
        HWT101CT_RxCallback(&sensor);
    }
}

/**
 * @brief Function implementing the initTask thread.
 * @param argument: Not used
 * @retval None
 */
void Init(void* argument)
{
    /* 初始化代码 */

    HAL_UART_RegisterCallback(&huart3, HAL_UART_RX_COMPLETE_CB_ID, Uart_RxCpltCallback);
    HWT101CT_Init(&sensor, &huart3);
    HWT101CT_ResetYaw(&sensor);

    // 初始化 CAN
    DJI_CAN_FilterInit(&hcan1, 0);
    CAN_RegisterCallback(&hcan1, 0, DJI_CAN_BaseReceiveCallback);

    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    // 初始化 DJI
    DJI_Init(&wheel_dji[0],
             (DJI_Config_t) {
                     .hcan       = &hcan1,
                     .id1        = 1,
                     .motor_type = M3508_C620,
                     .reverse    = true,
             });
    DJI_Init(&wheel_dji[1],
             (DJI_Config_t) {
                     .hcan       = &hcan1,
                     .id1        = 2,
                     .motor_type = M3508_C620,
                     .reverse    = false,
             });
    DJI_Init(&wheel_dji[2],
             (DJI_Config_t) {
                     .hcan       = &hcan1,
                     .id1        = 3,
                     .motor_type = M3508_C620,
                     .reverse    = false,
             });
    DJI_Init(&wheel_dji[3],
             (DJI_Config_t) {
                     .hcan       = &hcan1,
                     .id1        = 4,
                     .motor_type = M3508_C620,
                     .reverse    = true,
             });

    // 初始化控制器
    for (size_t i = 0; i < 4; i++)
        Motor_VelCtrl_Init(&motor_vel_ctrl[i],
                           (Motor_VelCtrlConfig_t) { .motor_type = MOTOR_TYPE_DJI,
                                                     .motor      = &wheel_dji[i],
                                                     .pid        = { //
                                                                     .Kp             = 45.0f,
                                                                     .Ki             = 0.07f,
                                                                     .Kd             = 5.00f,
                                                                     .abs_output_max = 8000.0f } });

    Chassis_Init(&chassis, &(Chassis_Config_t) {
        .driver = {
                          .chassis_type      = MECANUM4_O_TYPE,    ///< 底盘构型
                          .wheel_radius      = 77.0f,              ///< 轮子半径 (unit: mm)
                          .wheel_distance_x  = 504.60f,            ///< 左右轮距 (unit: mm)
                          .wheel_distance_y  = 579.78f,            ///< 前后轮距 (unit: mm)
                          .wheel_front_right = &motor_vel_ctrl[0], ///< 右前方
                          .wheel_front_left  = &motor_vel_ctrl[1], ///< 左前方
                          .wheel_rear_left   = &motor_vel_ctrl[2], ///< 左后方
                          .wheel_rear_right  = &motor_vel_ctrl[3], ///< 右后方
                  },
        .feedback_source = {
            .yaw = &sensor.yaw,
        },
    });

    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, CtrlTIM_Callback);
    HAL_TIM_Base_Start_IT(&htim6);

    osDelay(3000);
    Chassis_SetWorldFromCurrent(&chassis);
    // Mecanum4_SetVelocity(&chassis_mecanum4,
    //                      (Mecanum4_Velocity_t) {
    //                              .vx    = 0.5f,
    //                              .vy    = 0.0f,
    //                              .omega = 0.0f,
    //                      });
    // osDelay(5000);
    // Mecanum4_SetVelocity(&chassis_mecanum4,
    //                      (Mecanum4_Velocity_t) {
    //                              .vx    = 0.0f,
    //                              .vy    = 0.5f,
    //                              .omega = 0.0f,
    //                      });
    // osDelay(5000);

    // Chassis_SetVelBodyFrame(&chassis,
    //                         &(Chassis_Velocity_t) {
    //                                 .vx = 0.2f,
    //                                 .vy = 0.0f,
    //                                 .wz = 0.0f,
    //                         });
    // osDelay(1500);
    // Chassis_SetVelBodyFrame(&chassis,
    //                         &(Chassis_Velocity_t) {
    //                                 .vx = 0.0f,
    //                                 .vy = 0.2f,
    //                                 .wz = 0.0f,
    //                         });
    // osDelay(1500);
    // Chassis_SetVelBodyFrame(&chassis,
    //                         &(Chassis_Velocity_t) {
    //                                 .vx = 0.0f,
    //                                 .vy = 0.0f,
    //                                 .wz = 30.0f,
    //                         });
    // osDelay(1500);

    Chassis_SetTargetPostureInWorld(&chassis, &(Chassis_PostureTarget_t) {
        .posture = {
            .x   = 2.0f,
            .y   = 0.0f,
            .yaw = 180.0f,
        },
        .speed  = 0.25f,
        .omega  = 30.0f,
    });
    // osDelay(8000);
    // Chassis_SetTargetPostureInWorld(&chassis, &(Chassis_PostureTarget_t) {
    //     .posture = {
    //         .x   = 0.0f,
    //         .y   = 0.0f,
    //         .yaw = 0.0f,
    //     },
    //     .speed  = 0.5f,
    //     .omega  = 45.0f,
    // });
    // osDelay(8000);
    // Mecanum4_SetVelocity(&chassis_mecanum4,
    //                      (Mecanum4_Velocity_t) {
    //                              .vx    = -0.1f,
    //                              .vy    = 0.0f,
    //                              .omega = -10.0f,
    //                      });
    // osDelay(30000);
    // Chassis_SetVelBodyFrame(&chassis, &(Chassis_Velocity_t) { .vx = 0.0f, .vy = 0.0f, .wz = 0.0f
    // });

    /* 初始化完成后退出线程 */
    osThreadExit();
}
