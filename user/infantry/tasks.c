/* @brief 任务*/
#include "tasks.h"
#include "config.h"
#include "macro.h"
#include "handle.h"
#include "math.h"

void Task_MecArm(void *Parameters) {
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency    = pdMS_TO_TICKS(1);
    float            polartheta, r, z, pitch, yaw;
    while (1) {
        polartheta = MecArm.target_point.polartheta;
        r          = MecArm.target_point.r;
        z          = MecArm.target_point.z;
        pitch      = MecArm.target_point.pitch;
        yaw        = MecArm.target_point.yaw;

        // Unitree_Motor_Safety_Test(&Unitree_Bridge, 0, 20);
        // Unitree_Motor_Safety_Test(&Unitree_Bridge, 1, 20);

        // Vofa debug发送
        vf_debug.data[0] = Arm_Joint_2.joint_command.pos_des;
        vf_debug.data[1] = Arm_Joint_2.Get_Angle(&Arm_Joint_2);
        // vf_debug.data[2]=(Motor6020_MecArm_0_1.angle)/180.0f*PI;
        // vf_debug.data[1]=Motor6020_MecArm_0_0.input;
        // vf_debug.data[2]=Motor6020_MecArm_0_1.input;
        Vofa_Debug_Send(3);

        if(remoteData.rx>50){
            MecArm.target_point.polartheta-=0.00087222222f;
        }else if(remoteData.rx<-50){
            MecArm.target_point.polartheta+=0.00087222222f;
        }

        if(remoteData.lx>50){
            MecArm.target_point.r+=0.03f;
        }else if(remoteData.lx<-50){
            MecArm.target_point.r-=0.03f;
        }

        if(remoteData.ly>50){
            MecArm.target_point.z+=0.03f;
        }else if(remoteData.ly<-50){
            MecArm.target_point.z-=0.03f;
        }

        MecArm_Calculate(&MecArm, polartheta, r, z, pitch, yaw);
        MecArm_Control(&MecArm);
        // 电机发送
        Unitree_Circular_Send(&Unitree_Bridge);
        Bridge_Send_Motor(&BridgeData, SafetyMode);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}