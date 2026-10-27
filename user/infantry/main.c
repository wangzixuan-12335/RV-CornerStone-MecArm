#define __HANDLE_GLOBALS

#include "config.h"
#include "macro.h"
#include "handle.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tasks.h"

void PF0_3V3(){
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;

    GPIO_Init(GPIOF,&GPIO_InitStructure);
    GPIO_SetBits(GPIOF,GPIO_Pin_0);
}

int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    //初始化
    //拉起一个3.3v的供电角
    PF0_3V3();
    Delay_Init(180);
    //机械臂USART6
    BSP_USART6_Init(4000000,USART_IT_IDLE);
    //Vofa
    BSP_USART3_Init(115200,0);
    Vofa_Debug_Init(&vf_debug,3,2);
    //CAN总线
    BSP_CAN_Init();

    Unitree_Motor_Init(&Unitree_MecArm_1,0,6.33f,1,-1.48787534f);
    Unitree_Motor_Init(&Unitree_MecArm_2,1,6.33f,1,-0.627314031f);
    Unitree_Bridge_Init(&Unitree_Bridge,USART6,6);
    Unitree_Bridge_Bind(&Unitree_Bridge,&Unitree_MecArm_1);
    Unitree_Bridge_Bind(&Unitree_Bridge,&Unitree_MecArm_2);

    Motor_Init(&Motor6020_MecArm_3,GM6020,1,MOTOR_TYPE_GM6020,2.444444f,1,1);
    Motor_Set_Angle_Bias(&Motor6020_MecArm_3,55.7306595f);
    PID_Init(&Motor6020_MecArm_3_PosPID, 8.0f, 0.0f, 0.2f, 10.0f, 1.0f);
    PID_Init(&Motor6020_MecArm_3_SpeedPID, 180.0f, 20.0f, 0.0f, 16384.0f, 10000.0f);
    Bridge_Bind(&BridgeData,CAN1_BRIDGE,0x208,&Motor6020_MecArm_3);

    //关节1初始化
    Arm_Joint_Init(&Arm_Joint_1,
        JOINT_ACTUATOR_UNITREE_UART,
        &Unitree_MecArm_1,-1.59295785f,1.5306778f,0.0f,0.0f,
        Unitree_Set_Target,
        Unitree_Get_Angle,
        Unitree_Get_Speed,
    Unitree_Is_Online);

    //关节2初始化
    Arm_Joint_Init(&Arm_Joint_2,
        JOINT_ACTUATOR_UNITREE_UART,
        &Unitree_MecArm_2,-1.86728072f,2.69006801f,0.0f,0.0f,
        Unitree_Set_Target,
        Unitree_Get_Angle,
        Unitree_Get_Speed,
    Unitree_Is_Online);
    
    //关节3初始化
    Arm_Joint_Init(&Arm_Joint_3,
        JOINT_ACTUATOR_DJI_CAN,
        &Motor6020_MecArm_3,-1.65461532f,1.7253028135f,0.0f,0.0f,
        Motor6020_Set_Target
        ,Motor6020_Get_Angle
        ,Motor6020_Get_Speed
    ,Motor6020_Is_Online);

    MecArm_Init(&MecArm,
        &Arm_Joint_0,
        &Arm_Joint_1,
        &Arm_Joint_2,
        &Arm_Joint_3,
        &Arm_Joint_4,
    5,1);

    // 延时等待电机上电启动稳定
    delay_ms(1500);

    //创建机械臂任务
    xTaskCreate(
        Task_MecArm,
        "Task_MecArm",
        256,
        NULL,
        5,
        &MecArmTask_Handler
    );

    vTaskStartScheduler();
    
    while (1){

    }
}
