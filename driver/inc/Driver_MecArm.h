#ifndef __DRIVER_MECARM_H
#define __DRIVER_MECARM_H

#include "Driver_PID.h"
#include "stm32f4xx.h"

#define ARM_L1 280
#define ARM_L2 360.25
#define ARM_L3 97.5

// 声明角度控制结构体，解决不同电机方法使用参数不一致的问题
typedef struct {
    // MIT的5变量控制
    float pos_des; // 目标角度(6020,8010)
    float vel_des; // 目标速度(8010,6020)
    float tau_ff;  // 前馈力矩(8010,6020计划支持)
    float kp;      // 刚度系数(8010)
    float kd;      // 阻尼系数(8010)
    // 双PID环控制
    PID_Type *pos_pid;   // 位置PID(6020)
    PID_Type *speed_pid; // 速度PID(6020)
    // 8010积分项
    PID_Type *extra_integration_pid; // 8010仅积分PID(8010)
} Joint_Command_t;

// 前向声明
typedef struct Arm_Joint_Type   Arm_Joint_Type;
typedef struct Manipulator_Type Manipulator_Type;

// 1. 设置期望状态 (角度, 速度, 前馈力矩, 刚度/阻尼系数)
typedef void (*Set_TargetFunc)(Arm_Joint_Type *self, Joint_Command_t *joint_command);
// 2. 读取关节当前角度 (统一输出 rad)
typedef float (*Get_AngleFunc)(Arm_Joint_Type *self);
// 3. 读取关节当前转速 (统一输出 rad/s)
typedef float (*Get_SpeedFunc)(Arm_Joint_Type *self);
// 4. 检查关节是否在线/正常
typedef uint8_t (*Is_OnlineFunc)(Arm_Joint_Type *self);
// 5. 打开/关闭关节输出
typedef void (*Input_EnableFunc)(Arm_Joint_Type *self, uint8_t NewState);
// 机械臂计算关节力矩前馈
typedef float (*Get_Tau_ffFunc)(Manipulator_Type *self, uint8_t joint_index);

typedef enum {
    JOINT_ACTUATOR_DJI_CAN,     // DJI 3508 / 6020 / 2006
    JOINT_ACTUATOR_UNITREE_UART // 宇树 RS485 / UART 电机
} Joint_Actuator_Type_e;

/**
 * TODO: max_speed max_torque都只徒有其表，没有实际业务代码
 */
// 统一机械臂关节结构体
struct Arm_Joint_Type {
    // 基础信息
    Joint_Actuator_Type_e type;         // 电机硬件类型
    void                 *motor_handle; // 指向实际的 Motor_Type 或 Unitree_Motor_Type

    // 物理与安全限制
    float min_angle;  // 关节软限位 最小角 (rad)
    float max_angle;  // 关节软限位 最大角 (rad)
    float max_speed;  // 最大运行角速度 (rad/s)
    float max_torque; // 最大力矩限制 (N·m)

    Joint_Command_t joint_command; // 电机控制结构体
    // 抽象控制接口 (方法多态绑定)
    // 1. 设置期望状态 (角度, 速度, 前馈力矩, 刚度/阻尼系数)
    void (*Set_Target)(Arm_Joint_Type *self, Joint_Command_t *joint_command);
    // 2. 读取关节当前角度 (统一输出 rad)
    float (*Get_Angle)(Arm_Joint_Type *self);
    // 3. 读取关节当前转速 (统一输出 rad/s)
    float (*Get_Speed)(Arm_Joint_Type *self);
    // 4. 检查关节是否在线/正常
    uint8_t (*Is_Online)(Arm_Joint_Type *self);
    // 5. 打开/关闭关节输出
    void (*Input_Enable)(Arm_Joint_Type *self, uint8_t NewState);
};

// 完整机械臂实体
struct Manipulator_Type {
    Arm_Joint_Type *joints[4];   // 4自由度机械臂关节数组
    uint8_t         joint_count; // 关节数量
    uint8_t         is_enabled;  // 机械臂总体使能标志

    // 目标点以及姿态
    struct {
        float polartheta;
        float r;
        float z;
        float pitch;
    } target_point;

    // 解算成功的放到这里
    struct {
        float polartheta;
        float r;
        float z;
        float pitch;
    } target_point_last_success;

    // 机械臂各关节目标角度(关节0-4分别为:底座，肩，肘，末端pitch)
    float joint_target_angle[4];

    // 机械臂各关节目标速度(关节0-4分别为:底座，肩，肘，末端pitch)
    float joint_target_speed[4];

    // 机械臂末端笛卡尔坐标反馈 (X, Y, Z, Pitch)
    float end_effector_pose[4];
    /**
     * @brief 获取指定8010关节的重力前馈
     * @param Manipulator_Type *self Manipulator_Type句柄
     * @param joint_index 关节索引号（目前仅支持1，2）
     * @return 关节的重力前馈
     */
    float (*Get_Tau_ff)(Manipulator_Type *self, uint8_t joint_index);
};

void Arm_Joint_Init(Arm_Joint_Type       *ArmJoint,
                    Joint_Actuator_Type_e MotorType,
                    void                 *motor,
                    float                 min_angle,
                    float                 max_angle,
                    float                 max_speed,
                    float                 max_torque,
                    Set_TargetFunc        Set_Target_cb,
                    Get_AngleFunc         Get_Angle_cb,
                    Get_SpeedFunc         Get_Speed_cb,
                    Is_OnlineFunc         Is_Online_cb,
                    Input_EnableFunc      Input_Enable_cb);

void MecArm_Init(Manipulator_Type *arm,
                 Arm_Joint_Type   *joint0,
                 Arm_Joint_Type   *joint1,
                 Arm_Joint_Type   *joint2,
                 Arm_Joint_Type   *joint3,
                 Get_Tau_ffFunc    Get_Tau_ff_cb,
                 uint8_t           joint_count,
                 uint8_t           is_enabled);

void MecArm_InitialPoint_Init(Manipulator_Type *arm, float polartheta, float r, float z, float pitch);

float Get_Tau_ff(Manipulator_Type *self, uint8_t joint_index);

uint8_t MecArm_Calculate(Manipulator_Type *arm,
    float polarTheta,float r,float z,float pitch, float* angle_res, float* speed_res, float control_frequency);

void MecArm_Control(Manipulator_Type *arm);

float incrementTargetAngleByStep(float initialAngle,float targetAngle,uint32_t *step,uint32_t limit);

void JointCommand_SlowMove_Generate(Manipulator_Type *arm,uint8_t i);

void JointCommand_RemoteControl_Generate(Manipulator_Type *arm,uint8_t i);

#endif
