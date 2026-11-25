#include "Driver_MecArm.h"
#include "handle.h"

float max(float a,float b){
    if(a>=b){
        return a;
    }else{
        return b;
    }
}

float min(float a,float b){
    if(a<=b){
        return a;
    }else{
        return b;
    }
}

uint8_t IsInRange(float a,float min,float max){
    if(a<=max && a>=min){
        return 1;
    }else{
        return 0;
    }
}

/**
 * @brief 关节节点初始化
 * @param Arm_Joint_Type* 关节节点句柄
 * @param Joint_Actuator_Type_e 电机类型
 * @param *motor 电机句柄
 * @param min_angle 关节允许最小角度
 * @param max_angle 关节允许最大角度
 * @param max_speed 关节允许最大运动速度
 * @param max_torque 关节允许最大力矩
 * @param Set_TargetFunc 函数句柄 设置期望状态 (角度, 速度, 前馈力矩, 刚度/阻尼系数)
 * @param Get_AngleFunc 函数句柄 读取关节当前角度 (统一输出 rad)
 * @param Get_SpeedFunc 函数句柄 读取关节当前转速 (统一输出 rad/s)
 * @param Is_OnlineFunc 函数句柄 检查关节是否在线/正常
 * @param Input_EnableFunc 函数句柄 打开/关闭关节输出
 */
void Arm_Joint_Init(Arm_Joint_Type *ArmJoint,
                    Joint_Actuator_Type_e MotorType,
                    void *motor,
                    float min_angle,float max_angle,
                    float max_speed,float max_torque,
                    Set_TargetFunc Set_Target_cb,
                    Get_AngleFunc Get_Angle_cb,
                    Get_SpeedFunc Get_Speed_cb,
                    Is_OnlineFunc Is_Online_cb,
                    Input_EnableFunc Input_Enable_cb){
    ArmJoint->type=MotorType;
    ArmJoint->motor_handle=motor;
    ArmJoint->min_angle=min_angle;
    ArmJoint->max_angle=max_angle;
    ArmJoint->max_speed=max_speed;
    ArmJoint->max_torque=max_torque;
    ArmJoint->Set_Target=Set_Target_cb;
    ArmJoint->Get_Angle=Get_Angle_cb;
    ArmJoint->Get_Speed=Get_Speed_cb;
    ArmJoint->Is_Online=Is_Online_cb;
    ArmJoint->Input_Enable=Input_Enable_cb;
}

/**
 * @brief 机械臂初始化
 * @param Manipulator_Type* 机械臂句柄
 * @param Arm_Joint_Type* 关节节点句柄0
 * @param Arm_Joint_Type* 关节节点句柄1
 * @param Arm_Joint_Type* 关节节点句柄2
 * @param Arm_Joint_Type* 关节节点句柄3
 * @param Get_Tau_ff_cb Get_Tau_ffFunc函数指针
 * @param joint_count 关节节点数量
 * @param is_enabled 机械臂使能标志位
 */
void MecArm_Init(Manipulator_Type *arm,
    Arm_Joint_Type* joint0,
    Arm_Joint_Type* joint1,
    Arm_Joint_Type* joint2,
    Arm_Joint_Type* joint3,
    Get_Tau_ffFunc  Get_Tau_ff_cb,
    uint8_t joint_count,uint8_t is_enabled){
    arm->joints[0]=joint0;
    arm->joints[1]=joint1;
    arm->joints[2]=joint2;
    arm->joints[3]=joint3;
    arm->Get_Tau_ff=Get_Tau_ff_cb;
    arm->joint_count=joint_count;
    arm->is_enabled=is_enabled;
}

/**
 * @brief 机械臂初始姿态初始化
 * @param Manipulator_Type* 机械臂句柄
 * @param polartheta 初始姿态底座极坐标角度(rad)
 * @param r 初始姿态半径(mm)
 * @param z 初始姿态高度(mm)
 * @param pitch 初始姿态pitch角(rad)
 */
void MecArm_InitialPoint_Init(
    Manipulator_Type *arm,
    float polartheta,float r,float z,float pitch){
    arm->target_point.polartheta=polartheta;
    arm->target_point.r=r;
    arm->target_point.z=z;
    arm->target_point.pitch=pitch;
}

/**
 * @brief 获取指定关节的力矩前馈
 * @param Manipulator_Type *self Manipulator_Type句柄
 * @param joint_index 关节索引号（目前仅支持1，2）
 * @return 关节的力矩前馈
 */
float Get_Tau_ff(Manipulator_Type *self,uint8_t joint_index){
    Arm_Joint_Type *joint1,*joint2;
    float theta1,theta2;
    float tau_ff         =   0.0f;
    float tau_g          =   0.0f;
    float pid_i          =   0.0f;

    joint1=self->joints[1];
    joint2=self->joints[2];

    theta1=joint1->Get_Angle(joint1);
    theta2=joint2->Get_Angle(joint2);
    
    switch (joint_index)
    {
    case 1:
        //抵消静差的积分项
        pid_i=PID_Calculate(joint1->joint_command.extra_integration_pid,joint1->joint_command.pos_des,theta1);
        //偏导数推出来的重力补偿
        tau_g=(-4.8678944f)*sin(theta1)+(-2.63721157f)*sin(theta1+theta2);
        //总前馈
        tau_ff=pid_i+tau_g;
        return tau_ff;
        break;
    case 2:
        //抵消静差的积分项
        pid_i=PID_Calculate(joint2->joint_command.extra_integration_pid,joint2->joint_command.pos_des,theta2);
        //偏导数推出来的重力补偿
        tau_g=(-2.63721157f)*sin(theta1+theta2);
        //总前馈
        tau_ff=pid_i+tau_g;
        return tau_ff;
        break;
    default:
        return 0.0f;
        break;
    }
}

/**
 * TODO:可能还不完善
 * @brief 机械臂点的解算
 * @param Manipulator_Type*         机械臂结构体句柄
 * @param polarTheta                底座极坐标角度(rad)
 * @param r                         半径(mm)
 * @param z                         高度(mm)
 * @param pitch                     pitch角度(rad)
 * @param *angle_res                返回值[0-3]:[polartheta(rad),肩关节角度(rad)，肘关节角度(rad)，末端pitch角度(rad)]
 * @param *speed_res                返回速度值[0-3]:[polartheta(rad/s),肩关节角速度(rad/s)，肘关节角速度(rad/s)，末端pitch角速度(rad/s)]
 * @param control_frequency         控制频率(Hz)
 * @return 解算是否成功              (0:失败，1:成功)
 */
uint8_t MecArm_Calculate(Manipulator_Type *arm,
    float polarTheta,float r,float z,float pitch, float* angle_res, float* speed_res, float control_frequency){

    float y1,z1,cos_theta1,sin_theta1,theta1;
    float y2,z2,cos_theta2,sin_theta2,theta2;
    float y3,z3,theta3;
    float k1,k2;

    y3=r-ARM_L3*cos(pitch);
    z3=z+ARM_L3*sin(pitch);
    
    //目标位置不可达直接返回
    if((y3*y3+z3*z3)>(ARM_L1+ARM_L2)*(ARM_L1+ARM_L2)){
        return 0;
    }
    
    cos_theta2=(y3*y3+z3*z3-ARM_L1*ARM_L1-ARM_L2*ARM_L2)/(2*ARM_L1*ARM_L2);
    cos_theta2=max(min(cos_theta2, 1.0), -1.0);
    sin_theta2=sqrt(1.0f - cos_theta2*cos_theta2);
    theta2=atan2(sin_theta2,cos_theta2);

    k1=ARM_L1+ARM_L2*cos(theta2);
    k2=ARM_L2*sin(theta2);
    sin_theta1=(k1*y3-k2*z3)/(k1*k1+k2*k2);
    cos_theta1=(k2*y3+k1*z3)/(k1*k1+k2*k2);
    theta1=atan2(sin_theta1,cos_theta1);
    
    theta3=pitch+PI/2-theta2-theta1;

    if(IsInRange(theta1,arm->joints[1]->min_angle,arm->joints[1]->max_angle) &&
        IsInRange(theta2,arm->joints[2]->min_angle,arm->joints[2]->max_angle) &&
        IsInRange(theta3,arm->joints[3]->min_angle,arm->joints[3]->max_angle)){
            //速度期望
            if(speed_res!=NULL){
                speed_res[0]=(polarTheta-angle_res[0])*control_frequency;
                speed_res[1]=(theta1-angle_res[1])*control_frequency;
                speed_res[2]=(theta2-angle_res[2])*control_frequency;
                speed_res[3]=(theta3-angle_res[3])*control_frequency;
            }
            
            //位置期望
            angle_res[0]=polarTheta;
            angle_res[1]=theta1;
            angle_res[2]=theta2;
            angle_res[3]=theta3;
    }else{
        return 0;
    }
    return 1;
}



//TODO: 机械臂控制未完成，废弃代码，计划清除
void MecArm_Control(Manipulator_Type *arm){
    //临时测试
    for (uint8_t i = 0; i < 4; i++)
    {
        Joint_Command_t *joint_command=&(arm->joints[i]->joint_command);
        if(i==3){
            joint_command->kd=0;
            joint_command->kp=0;
            joint_command->tau_ff=0;
            joint_command->vel_des=0;
            joint_command->pos_des=arm->joint_target_angle[i];
            joint_command->pos_pid=&Motor6020_MecArm_3_PosPID;
            joint_command->speed_pid=&Motor6020_MecArm_3_SpeedPID;
        }else if(i==0){
            joint_command->kd=0;
            joint_command->kp=0;
            joint_command->tau_ff=0;
            joint_command->vel_des=0;
            joint_command->pos_des=arm->joint_target_angle[i];
            joint_command->pos_pid=&Motor6020_MecArm_Group_0_PosPID;
            joint_command->speed_pid=&Motor6020_MecArm_Group_0_SpeedPID;
        }else if(i==1){
			joint_command->kd=0.02f;
            joint_command->tau_ff=arm->Get_Tau_ff(arm,i);
            joint_command->vel_des=0;
            joint_command->pos_pid=NULL;
            joint_command->speed_pid=NULL;
            joint_command->pos_des=arm->joint_target_angle[i];
            joint_command->kp=0.2f;
            joint_command->extra_integration_pid=&Unitree_MecArm_1_PID;
        }else if(i==2){
			joint_command->kd=0;
            joint_command->tau_ff=arm->Get_Tau_ff(arm,i);
            joint_command->vel_des=0;
            joint_command->pos_pid=NULL;
            joint_command->speed_pid=NULL;
            joint_command->pos_des=arm->joint_target_angle[i];
            joint_command->kp=0.2f;
            joint_command->extra_integration_pid=&Unitree_MecArm_2_PID;
        }
        arm->joints[i]->Set_Target(arm->joints[i],joint_command);
    }
}

/**
 * @brief 根据步长返回角度
 * @param initialAngle 初始角度
 * @param targetAngle  目标角度
 * @param *step        uint32* 步数变量
 * @param limit        uint32  步数上线
 * @return 目标角度,(-1.0f则为递增完成)
 */
float incrementTargetAngleByStep(float initialAngle, float targetAngle, uint32_t *step, uint32_t limit) {
    // 步长达到limit则返回
    if (*step > limit) {
        return -1.0f;
    }
    float angle_diff     = targetAngle - initialAngle;
    float angle_per_step = angle_diff / ((float) limit);
    float res            = (initialAngle + ((float) (*step)) * angle_per_step);
    (*step)++;
    return res;
}

/**
 * @brief 给SLOW_MOVE阶段的机械臂关节JointCommand赋值
 * @param Manipulator_Type* 机械臂句柄
 * @param i                 第几个关节
 */
void JointCommand_SlowMove_Generate(Manipulator_Type *arm,uint8_t i){
    Joint_Command_t *joint_command=&(arm->joints[i]->joint_command);
    switch (i)
    {
    case 0:
        //底座6020电机组
        joint_command->kd=0;
        joint_command->kp=0;
        joint_command->tau_ff=0;
        joint_command->vel_des=0;
        joint_command->pos_des=arm->joint_target_angle[i];
        joint_command->pos_pid=&Motor6020_MecArm_Group_0_PosPID;
        joint_command->speed_pid=&Motor6020_MecArm_Group_0_SpeedPID;
        break;
    case 1:
        //肩关节8010电机
        joint_command->kd=0.08f;
        joint_command->tau_ff=arm->Get_Tau_ff(arm,i);
        joint_command->vel_des=0;
        joint_command->pos_pid=NULL;
        joint_command->speed_pid=NULL;
        joint_command->pos_des=arm->joint_target_angle[i];
        joint_command->kp=0.3f;
        joint_command->extra_integration_pid=&Unitree_MecArm_1_PID;
        break;
    case 2:
        //肘关节8010电机
        joint_command->kd=0.08f;
        joint_command->tau_ff=arm->Get_Tau_ff(arm,i);
        joint_command->vel_des=0;
        joint_command->pos_pid=NULL;
        joint_command->speed_pid=NULL;
        joint_command->pos_des=arm->joint_target_angle[i];
        joint_command->kp=0.4f;
        joint_command->extra_integration_pid=&Unitree_MecArm_2_PID;
        break;
    case 3:
        //pitch轴6020电机
        joint_command->kd=0;
        joint_command->kp=0;
        joint_command->tau_ff=0;
        joint_command->vel_des=0;
        joint_command->pos_des=arm->joint_target_angle[i];
        joint_command->pos_pid=&Motor6020_MecArm_3_PosPID;
        joint_command->speed_pid=&Motor6020_MecArm_3_SpeedPID;
        break;
    default:
        break;
    }
}

/**
 * @brief 给REMOTE_CONTROL阶段的机械臂关节JointCommand赋值
 * @param Manipulator_Type* 机械臂句柄
 * @param i                 第几个关节
 */
void JointCommand_RemoteControl_Generate(Manipulator_Type *arm,uint8_t i){
    Joint_Command_t *joint_command=&(arm->joints[i]->joint_command);
    switch (i)
    {
    case 0:
        //底座6020电机组
        joint_command->kd=0;
        joint_command->kp=0;
        joint_command->tau_ff=0;
        joint_command->vel_des=arm->joint_target_speed[i];
        joint_command->pos_des=arm->joint_target_angle[i];
        joint_command->pos_pid=&Motor6020_MecArm_Group_0_PosPID;
        joint_command->speed_pid=&Motor6020_MecArm_Group_0_SpeedPID;
        break;
    case 1:
        //肩关节8010电机
        joint_command->kd=0.12f;
        joint_command->tau_ff=arm->Get_Tau_ff(arm,i);
        joint_command->vel_des=arm->joint_target_speed[i];
        joint_command->pos_pid=NULL;
        joint_command->speed_pid=NULL;
        joint_command->pos_des=arm->joint_target_angle[i];
        joint_command->kp=1.3f;
        joint_command->extra_integration_pid=&Unitree_MecArm_1_PID;
        break;
    case 2:
        //肘关节8010电机
        joint_command->kd=0.1f;
        joint_command->tau_ff=arm->Get_Tau_ff(arm,i);
        joint_command->vel_des=arm->joint_target_speed[i];
        joint_command->pos_pid=NULL;
        joint_command->speed_pid=NULL;
        joint_command->pos_des=arm->joint_target_angle[i];
        joint_command->kp=1.45f;
        joint_command->extra_integration_pid=&Unitree_MecArm_2_PID;
        break;
    case 3:
        //pitch轴6020电机
        joint_command->kd=0;
        joint_command->kp=0;
        joint_command->tau_ff=0;
        joint_command->vel_des=arm->joint_target_speed[i];
        joint_command->pos_des=arm->joint_target_angle[i];
        joint_command->pos_pid=&Motor6020_MecArm_3_PosPID;
        joint_command->speed_pid=&Motor6020_MecArm_3_SpeedPID;
        break;
    default:
        break;
    }
}