#define __HANDLE_GLOBALS

#include "config.h"
#include "macro.h"
#include "handle.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tasks.h"

int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    //初始化
    Delay_Init(180);
    BSP_USART6_Init(4000000,USART_IT_IDLE);

    Unitree_Motor_Init(&Unitree_MecArm_1,0,6.33f,1,0);
    Unitree_Motor_Init(&Unitree_MecArm_2,1,6.33f,1,0);
    Unitree_Bridge_Init(&Unitree_Bridge,USART6,6);
    Unitree_Bridge_Bind(&Unitree_Bridge,&Unitree_MecArm_1);
    Unitree_Bridge_Bind(&Unitree_Bridge,&Unitree_MecArm_2);

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
