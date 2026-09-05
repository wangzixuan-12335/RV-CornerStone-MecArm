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
    BSP_USART6_Init(4000000,USART_IT_IDLE);
    // BSP_DMA_Init(USART6_Rx,,34);

    //创建机械臂任务
    xTaskCreate(
        Task_MecArm,
        "Task_MecArm",
        256,
        NULL,
        5,
        &MecArmTask_Handler
    );

    while (1)
    {

    }
}
