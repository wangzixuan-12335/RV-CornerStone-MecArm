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
        // Unitree_Motor_Safety_Test(&Unitree_Bridge, 0, 20);
		// Unitree_Motor_Safety_Test(&Unitree_Bridge, 1, 20);
        
        //Vofa debug发送
        vf_debug.data[0]=Arm_Joint_2.joint_command.pos_des;
        vf_debug.data[1]=Arm_Joint_2.Get_Angle(&Arm_Joint_2);
        // vf_debug.data[2]=(Motor6020_MecArm_0_1.angle)/180.0f*PI;
        // vf_debug.data[1]=Motor6020_MecArm_0_0.input;
        // vf_debug.data[2]=Motor6020_MecArm_0_1.input;
        Vofa_Debug_Send(3);

        MecArm_Calculate(&MecArm,PI/2.0f,450.0f,0,PI/2.0f,0);
        MecArm_Control(&MecArm);
        //电机发送
		Unitree_Circular_Send(&Unitree_Bridge);
        Bridge_Send_Motor(&BridgeData, SafetyMode);

        vTaskDelayUntil(&xLastWakeTime,xFrequency);
    }
}