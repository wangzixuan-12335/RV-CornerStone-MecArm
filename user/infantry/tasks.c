/* @brief 任务*/
#include "tasks.h"
#include "config.h"
#include "macro.h"
#include "handle.h"
#include "math.h"

void Task_MecArm(void *Parameters) {
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency    = pdMS_TO_TICKS(1);

    // 状态机状态
    MecArm_State_e state = COMMUNICATION_BUILD;
    
    // COMMUNICATION_BUILD获得的初始角
    float          communication_build_InitialAngle[4];
    // COMMUNICATION_BUILD关节通信成功标志位      
    uint8_t        Is_Joint0_online, Is_Joint1_online, Is_Joint2_online, Is_Joint3_online;

    // SOLW_MOVE终点目标角(根据初始化位姿解算所得)
    float          slow_move_TargetAngle[4];
    // SOLW_MOVE时步长
    uint32_t       slow_move_step[4]={0};
    // SOLW_MOVE时每步算出来要去的角度
    float          slow_move_angle[4];
    
    float          polartheta, r, z, pitch;
    /**
     * TODO:状态机，通信-缓启动到initial角-遥控器控制
     * 0.状态机enum
     * 1.确保通信成功，然后拿所有关节初始角度，这个时候要注意，电机千万不能给目标角，inputenable标志位控制这个
     * 2.初始角度-initial角度解算出来的目标位置角（calculate方法要重构，使用一个句柄拿返回值，因为解算出来的角我是不一定直接去的），
     *      使用目标时间（步长）控制（这个功能要写新函数），所有都返回true就下一个阶段，
     *      （control方法也要重写状态机，去哪一个角可以依赖mecArm总结构体，
     *          但是如果直接包装成一个函数就把joint_command的灵活性牺牲了，
     *          把每个参数包装成一个函数，其实很多参数不是经常改的，加一个Init函数把确定不动的值初始化好,然后再给别的东西加函数)
     * 3.遥控器控制阶段，解算不出来就把控制目标换到上一次成功的值，还是要写个last状态,成功更新一切，失败就切到上一个状态
     */
    while (1) {
        switch (state) {
        case COMMUNICATION_BUILD:
                // 获取Joint是否Online
                Is_Joint0_online = Arm_Joint_0.Is_Online(&Arm_Joint_0);
                Is_Joint1_online = Arm_Joint_1.Is_Online(&Arm_Joint_1);
                Is_Joint2_online = Arm_Joint_2.Is_Online(&Arm_Joint_2);
                Is_Joint3_online = Arm_Joint_3.Is_Online(&Arm_Joint_3);
                if (Is_Joint0_online && Is_Joint1_online && Is_Joint2_online && Is_Joint3_online) {
                    for (uint8_t i = 0; i < MecArm.joint_count; i++) {
                        // 获取启动初始角度
                        communication_build_InitialAngle[i] = MecArm.joints[i]->Get_Angle(MecArm.joints[i]);
                        // 开启输出标志位
                        MecArm.joints[i]->Input_Enable(MecArm.joints[i], 1);
                    }
                    // 全部关节通信正常进入缓启动状态
                    state = SOLW_MOVE;
                }
                break;
        case SOLW_MOVE:
            // 获取初始启动目标
            polartheta = MecArm.target_point.polartheta;
            r          = MecArm.target_point.r;
            z          = MecArm.target_point.z;
            pitch      = MecArm.target_point.pitch;
            // 检测解算是否成功
            if (MecArm_Calculate(&MecArm, polartheta, r, z, pitch, slow_move_TargetAngle,NULL,1000) == 0) {
                break;
            }
            for (uint8_t i = 0; i < MecArm.joint_count; i++) {
                // 获取每次缓慢移动的目标角度
                slow_move_angle[i] = incrementTargetAngleByStep(
                    communication_build_InitialAngle[i], slow_move_TargetAngle[i], 
                    &(slow_move_step[i]), (uint32_t) (3.9f / 0.001f));

                // 只要不返回已到达就更新目标角度
                    if (slow_move_angle[i] != -1.0f) {
                    MecArm.joint_target_angle[i]=slow_move_angle[i];
                }

                // 生成针对SLOW_MOVE阶段的JointCommand
                JointCommand_SlowMove_Generate(&MecArm,i);
                // SetTarget
                MecArm.joints[i]->Set_Target(MecArm.joints[i],&(MecArm.joints[i]->joint_command));
            }
            // 全部到达就去下一个阶段
            if (slow_move_angle[0] == -1.0f && 
                slow_move_angle[1] == -1.0f && 
                slow_move_angle[2] == -1.0f && 
                slow_move_angle[3] == -1.0f) {
                    state = REMOTE_CONTROL;
            }
            break;
        case REMOTE_CONTROL:
            if (remoteData.rx > 50) {
                MecArm.target_point.polartheta -= 0.00087222222f;
            } else if (remoteData.rx < -50) {
                MecArm.target_point.polartheta += 0.00087222222f;
            }

            if (remoteData.lx > 50) {
                MecArm.target_point.r += 0.15f;
            } else if (remoteData.lx < -50) {
                MecArm.target_point.r -= 0.15f;
            }

            if (remoteData.ly > 50) {
                MecArm.target_point.z += 0.15f;
            } else if (remoteData.ly < -50) {
                MecArm.target_point.z -= 0.15f;
            }
            polartheta = MecArm.target_point.polartheta;
            r          = MecArm.target_point.r;
            z          = MecArm.target_point.z;
            pitch      = MecArm.target_point.pitch;
            if(MecArm_Calculate(&MecArm, polartheta, r, z, pitch, 
                MecArm.joint_target_angle, MecArm.joint_target_speed, 1000) == 0){
                // 解算失败
                MecArm.target_point.polartheta=MecArm.target_point_last_success.polartheta;
                MecArm.target_point.r=MecArm.target_point_last_success.r;
                MecArm.target_point.z=MecArm.target_point_last_success.z;
                MecArm.target_point.pitch=MecArm.target_point_last_success.pitch;
            }else{
                // 解算成功
                MecArm.target_point_last_success.polartheta=polartheta;
                MecArm.target_point_last_success.r=r;
                MecArm.target_point_last_success.z=z;
                MecArm.target_point_last_success.pitch=pitch;
            }
            
            for (uint8_t i = 0; i < MecArm.joint_count; i++){
                JointCommand_RemoteControl_Generate(&MecArm,i);
                MecArm.joints[i]->Set_Target(MecArm.joints[i],&(MecArm.joints[i]->joint_command));
            }
            
            break;
        case DEBUG:
            break;
        default:
            break;
        }



        // MecArm_Calculate(&MecArm, polartheta, r, z, pitch, yaw);
        // MecArm_Control(&MecArm);

        //=====================================Vofa debug发送=============================================
        vf_debug.data[0] = MecArm.joint_target_angle[1];
        vf_debug.data[1] = MecArm.joints[1]->Get_Angle(MecArm.joints[1]);
        vf_debug.data[2] = MecArm.joint_target_angle[2];
        vf_debug.data[3] = MecArm.joints[2]->Get_Angle(MecArm.joints[2]);
        Vofa_Debug_Send(3);

        //=======================================电机发送==================================================
        Unitree_Circular_Send(&Unitree_Bridge);
        Bridge_Send_Motor(&BridgeData, SafetyMode);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
