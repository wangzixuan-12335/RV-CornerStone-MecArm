/* @brief 任务*/
#include "tasks.h"
#include "config.h"
#include "macro.h"
#include "handle.h"
#include "math.h"

void Task_MecArm(void *Parameters){
    TickType_t xLastWakeTime=xTaskGetTickCount();
    const TickType_t xFrequency=pdMS_TO_TICKS(1);

    while (1)
    {
        Unitree_Motor_Safety_Test(&Unitree_Bridge, 0, 20);
		Unitree_Motor_Safety_Test(&Unitree_Bridge, 1, 20);

		Unitree_Circular_Send(&Unitree_Bridge);
        vTaskDelayUntil(&xLastWakeTime,xFrequency);
    }
}