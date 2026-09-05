#include "handle.h"

/**
 * @brief 将浮点物理力矩 (N·m) 转换为电机协议所需的 int16_t 整数
 * @param motor 宇树电机结构体
 * @param tau_ff 前馈力矩 (单位: N·m, 范围 -127.99 ~ +127.99)
 * @return int16_t 写入串口发送帧的数据
 */
int16_t Torque_To_Raw(Unitree_Motor_Type *motor,float tau_ff)
{
    // 1. 安全限幅：防止超出 2 字节 signed short 范围引起数据溢出/翻转
    if (tau_ff > 127.99f)  tau_ff = 127.99f;
    if (tau_ff < -127.99f) tau_ff = -127.99f;

    // 2. 乘 256 并强制类型转换为 signed short (int16_t)
    return (int16_t)(tau_ff * 256.0f * motor->direction);
}

/**
 * @brief 期望电机速度转换设置值
 * @param motor 宇树电机结构体
 * @param w_des 期望电机速度(rad/s)
 * @return int16_t 写入串口发送帧的数据
 */
int16_t Speed_To_Raw(Unitree_Motor_Type *motor,float w_des){
    if(w_des>804.0f)    w_des=804.0f;
    if(w_des<-804.0f)   w_des=-804.0f;
    return (int16_t)(w_des * 256.0f / 6.28318f * motor->direction);
}

/**
 * @brief 期望电机位置转换设置值
 * @param motor 宇树电机结构体
 * @param pos_des 期望电机位置(rad)
 * @return int32_t 写入串口发送帧的数据
 */
int32_t Pos_To_Raw(Unitree_Motor_Type *motor,float pos_des){
    return (int32_t)(pos_des / 6.28318f * 32768.0f * motor->reduction_rate * motor->direction);
}

/**
 * @brief 关节刚度系数 Kp转换Raw值
 * @param Kp 期待的Kp
 * @return uint16_t 写入串口发送帧的数据
 */
uint16_t Kp_To_Raw(float Kp){
    if(Kp<0.0f)    Kp=0.0f;
    if(Kp>25.599f)  Kp=25.599f;
    return (uint16_t)(Kp*1280.0f);
}

/**
 * @brief 关节阻尼系数 Kd转换Raw值
 * @param Kd 期待的Kd
 * @return uint16_t 写入串口发送帧的数据
 */
uint16_t Kd_To_Raw(float Kd){
    if(Kd<0.0f)    Kd=0.0f;
    if(Kd>25.599f)  Kd=25.599f;
    return (uint16_t)(Kd*1280.0f);
}

float Raw_To_Angle(Unitree_Motor_Type *motor,int32_t raw_pos){
    float real_pos=(float)(raw_pos / 32768.0f * 6.28318f);
    return (float)((real_pos-motor->pos_offset)*motor->direction);
}

float Raw_To_Speed(Unitree_Motor_Type *motor,int16_t raw_speed){
    float real_speed=(float)(raw_speed / 256.0f * 6.28318f);
    return (float)(real_speed*motor->direction);
}

float Raw_To_Torque(Unitree_Motor_Type *motor,int16_t raw_torque){
    float real_torque=(float)(raw_torque / 256.0f);
    return (float)(real_torque*motor->direction);
}

/**
 * @brief  初始化单个宇树电机
 * @param  motor: 电机结构体指针
 * @param  id: 电机硬件ID (0~14)
 * @param  reduction_rate: 外部减速比 (若电机输出轴直接接连杆则填 1.0f)
 * @param  dir: 旋转方向 (1 为正转, -1 为反转)
 * @param  zero_offset: 机械臂零位校准偏置 (rad)
 */
void Unitree_Motor_Init(Unitree_Motor_Type *motor, 
                        uint8_t id, 
                        float reduction_rate, 
                        int8_t dir, 
                        float zero_offset){
    
    //1.先进行默认配置
    motor->id=id;
    motor->reduction_rate=reduction_rate;
    motor->direction=dir;
    motor->pos_offset=zero_offset;
    motor->status=1;
    
    // 2. 初始安全控制量（上电默认进入阻尼模式，防止突然暴冲甩臂）
    motor->cmd.mode          = 0;              // 0: 阻尼模式 / 停止
    motor->cmd.target_angle  = 0.0f;           // 期望角度
    motor->cmd.target_speed  = 0.0f;           // 期望速度
    motor->cmd.target_torque = 0.0f;           // 前馈力矩
    motor->cmd.kp            = 0.0f;           // 上电初始刚度设为 0
    motor->cmd.kd            = 1.0f;           // 给少量阻尼 Kd，起到缓冲制动保护作用

    // 3. 反馈状态初始化
    motor->state.angle       = 0.0f;
    motor->state.speed       = 0.0f;
    motor->state.torque      = 0.0f;
    motor->state.temperature = 0;
    motor->state.online      = 0;              // 刚上电标记为离线，收到第一帧心跳后置 1
    motor->state.updated_at  = 0;
}

/**
 * @brief Unitree通讯桥初始化
 * @param bridge   通讯桥变量
 * @param usartx    Usartx
 * @param deviceID  Usartx对应的x的值
 */
void Unitree_Bridge_Init(Unitree_Bridge_Type *bridge,
                         USART_TypeDef *usartx,uint32_t deviceID){
    bridge->usart=usartx;
    bridge->deviceID=deviceID;
    bridge->motor_count=0;
    bridge->polling_index=0;
    bridge->last_send_time=0;
}

/**
 * @brief Unitree电机绑定Unitree桥
 * @param bridge    通讯桥变量
 * @param motor     Unitree电机变量
 */
void Unitree_Bridge_Bind(Unitree_Bridge_Type *bridge,Unitree_Motor_Type *motor){
    bridge->motors[motor->id]=motor;
    bridge->motor_count++;
}

/**
 * @brief 解析宇树电机反馈帧 (小端字节序)
 * @param bridge 通讯桥变量 (用于更新电机在线状态)
 * @param frame  解析结果输出结构体
 * @param data   原始接收字节流 (长度 >= sizeof(Unitree_RecvFrame_t))
 * @note  调用前应先用 Verify_CRC16_Check_Sum 校验 CRC
 */
void Unitree_Unpack(Unitree_Bridge_Type *bridge, Unitree_RecvFrame_t *frame, uint8_t *data){
    
    memcpy(frame,data,sizeof(Unitree_RecvFrame_t));

    //更新对应电机的状态 (心跳 + 反馈数据)
    if (bridge != NULL) {
        uint8_t id = frame->id & 0x0F;
        // 先检查 ID 是否在已绑定范围内, 防止数组越界
        if (id < bridge->motor_count) {
            Unitree_Motor_Type *motor = bridge->motors[id];
            if (motor != NULL && motor->id == frame->id) {
                //允许发送标志位
                motor->status            = 0;
                
                //在线心跳
                motor->state.online     = 1;
                motor->state.updated_at = xTaskGetTickCount();

                //反馈数据 (考虑方向极性, 角度叠加零偏)
                motor->state.angle       = Raw_To_Angle(motor,frame->pos);
                motor->state.speed       = Raw_To_Speed(motor,frame->speed);
                motor->state.torque      = Raw_To_Torque(motor,frame->torque);
                motor->state.temperature = frame->temperature;
            }
        }
    }
}

/**
 * @brief 宇树电机接收处理 (IDLE 中断中调用)
 * @param bridge   通讯桥变量
 * @note  流程: 清 IDLE 标志 -> 停 DMA -> 帧头校验 -> CRC 校验 -> 解包 -> 重启 DMA
 */
void Unitree_Receive(Unitree_Bridge_Type *bridge){
    uint16_t tmp, len;
    uint32_t deviceID;
    deviceID=bridge->deviceID;
    // 1. 清除 IDLE 标志: 必须先读 SR 再读 DR (顺序不能反, 否则标志清不掉导致中断风暴)
    tmp = USARTx->SR;
    tmp = USARTx->DR;

    // 2. 停止 DMA, 防止新数据覆盖当前帧
    DMA_Disable(USARTx_Rx);

    // 3. 计算实际接收长度 (缓冲区大小 - DMA 剩余计数)
    len = UNITREE_BUFFER_SIZE - DMA_Get_Data_Counter(USARTx_Rx);

    // 4. 帧头校验 (快速过滤垃圾数据, 帧头不对直接丢弃, 不算 CRC)
    if (len >= sizeof(Unitree_RecvFrame_t) &&
        bridge->rx_buf[0] == 0xFD && bridge->rx_buf[1] == 0xEE) {

        // 5. CRC16 校验 (确认数据完整无误)
        if (Verify_CRC16_Check_Sum(bridge->rx_buf, sizeof(Unitree_RecvFrame_t))) {
            // 6. 校验通过, 解包并更新电机状态
            Unitree_RecvFrame_t frame;
            Unitree_Unpack(bridge, &frame, bridge->rx_buf);
        }
    }

    // 7. 重新使能 DMA, 接收下一帧
    DMA_Enable(USARTx_Rx, UNITREE_BUFFER_SIZE);
}

void Unitree_Generate_SendFrame(Unitree_SendFrame_t *frame,Unitree_Motor_Type *motor){
    frame->head[0]=0xFE;
    frame->head[1]=0xEE;
    frame->id=(motor->id)&0x0F;
    frame->mode=motor->cmd.mode;
    frame->target_pos=Pos_To_Raw(motor,motor->cmd.target_angle);
    frame->target_speed=Speed_To_Raw(motor,motor->cmd.target_speed);
    frame->target_torque=Torque_To_Raw(motor,motor->cmd.target_torque);
    frame->kp=Kp_To_Raw(motor->cmd.kp);
    frame->kd=Kd_To_Raw(motor->cmd.kd);
    frame->crc16=Get_CRC16_Check_Sum((uint8_t *)&frame,offsetof(Unitree_SendFrame_t, crc16));
}

void Unitree_Bridge_Send(Unitree_Bridge_Type *bridge){
    uint32_t deviceID=bridge->deviceID;
    uint8_t offset=0;
    for (uint8_t i = 0; i < bridge->motor_count; i++)
    {
        Unitree_Motor_Type *motor=bridge->motors[i];
        Unitree_SendFrame_t frame;
        if(motor->status==1){
            continue;
        }
        Unitree_Generate_SendFrame(&frame,motor);
        memcpy(bridge->tx_buf+offset,&frame,sizeof(Unitree_SendFrame_t));
        offset+=sizeof(Unitree_SendFrame_t);
        motor->status=1;
    }
    if(offset>0){
        DMA_Disable(USARTx_Tx);
        DMA_Enable(USARTx_Tx,offset);
    }
}