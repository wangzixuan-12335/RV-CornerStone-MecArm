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
        vTaskDelayUntil(&xLastWakeTime,xFrequency);
    }
}