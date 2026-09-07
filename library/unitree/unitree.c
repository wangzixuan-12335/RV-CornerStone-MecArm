#include "handle.h"

//预先计算好的 CRC16-CCITT 查找表 (多项式 0x1021，高位在前 MSB First)
static const uint16_t crc16_ccitt_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CC0, 0x0CE1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

//查表计算函数
uint16_t Get_CRC16_CCITT(const uint8_t *data, uint32_t length) {
    uint16_t crc = 0x0000; // 宇树电机协议初始值，若有其他标准需要可修改为 0xFFFF

    for (uint32_t i = 0; i < length; i++) {
        // (1) 取出当前 crc 的高 8 位与当前输入字节异或，算出来的值作为索引查找表里的预计算余数
        uint8_t index = (uint8_t)((crc >> 8) ^ data[i]);
        
        // (2) crc 本身左移 8 位（吐出高 8 位，低位补零），并与查表得到的预计算值异或
        crc = (uint16_t)((crc << 8) ^ crc16_ccitt_table[index]);
    }

    return crc;
}

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
    return (int16_t)(tau_ff * 256.0f * motor->direction / motor->reduction_rate);
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
    return (int16_t)(w_des * 256.0f / 6.28318f * motor->direction * motor->reduction_rate);
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
    float real_pos=(float)(raw_pos / 32768.0f * 6.28318f / motor->reduction_rate);
    return (float)((real_pos-motor->pos_offset)*motor->direction);
}

float Raw_To_Speed(Unitree_Motor_Type *motor,int16_t raw_speed){
    float real_speed=(float)(raw_speed / 256.0f * 6.28318f / motor->reduction_rate);
    return (float)(real_speed*motor->direction);
}

float Raw_To_Torque(Unitree_Motor_Type *motor,int16_t raw_torque){
    float real_torque=(float)(raw_torque / 256.0f * motor->reduction_rate);
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
    // 接收到数据，将总线繁忙标志位置空闲
    bridge->IsBusy=0;

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
        if (Get_CRC16_CCITT((uint8_t *)bridge->rx_buf, offsetof(Unitree_RecvFrame_t,crc16))==(((uint16_t)bridge->rx_buf[15]<<8)|(uint16_t)bridge->rx_buf[14])) {
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
    frame->crc16=Get_CRC16_CCITT((uint8_t *)frame,offsetof(Unitree_SendFrame_t, crc16));
}

void Unitree_Motor_Send(Unitree_Bridge_Type *bridge,uint8_t motorId){
    uint32_t deviceID=bridge->deviceID;
    Unitree_Motor_Type *motor=bridge->motors[motorId];
    Unitree_SendFrame_t frame;
    if(motor->status==1 || bridge->IsBusy==1){
        return;
    }
    bridge->polling_index=motorId;
    Unitree_Generate_SendFrame(&frame,motor);
    memcpy(bridge->tx_buf,&frame,sizeof(Unitree_SendFrame_t));
    motor->status=1;
    bridge->IsBusy=1;
    DMA_Disable(USARTx_Tx);
    DMA_Enable(USARTx_Tx,sizeof(Unitree_SendFrame_t));
}

void Unitree_Circular_Send(Unitree_Bridge_Type *bridge){
    uint8_t now_id=bridge->polling_index;
    now_id++;
    if(now_id>=bridge->motor_count){
        now_id=0;
    }
    Unitree_Motor_Send(bridge,now_id);
}

