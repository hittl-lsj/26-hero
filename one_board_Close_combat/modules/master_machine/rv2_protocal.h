//
// Created by 26090 on 25-1-13.
//

#ifndef RV2_PROTOCAL_H
#define RV2_PROTOCAL_H
#include "master_process.h"
#include "stdbool.h"

#define RV2_PROTOCAL_HEADER 0x5A


#pragma pack(1)
/**
 * @brief RV2协议发送数据包结构体定义
 * @details 该结构体用于定义RV2协议的发送数据包格式，包含了机器人姿态、瞄准信息等数据
 * @note 数据采用小端字节序，使用CRC16校验确保数据完整性
 */
typedef struct
{
    uint8_t header;           // 数据包头部，固定值为0x5A
    uint8_t detect_color :1;  // 敌方颜色标识位：0-红色敌方，1-蓝色敌方
    bool reset_tracker :1;    // 追踪器重置标志位：true-重置追踪器，false-不重置
    uint8_t reserved :6;      // 保留位，暂未使用
    float roll;               // 机器人滚转角，单位：弧度(rad)
    float pitch;              // 机器人俯仰角，单位：弧度(rad)
    float yaw;                // 机器人偏航角，单位：弧度(rad)
    float aim_x;              // 瞄准点X坐标，单位：米(m)
    float aim_y;              // 瞄准点Y坐标，单位：米(m)
    float aim_z;              // 瞄准点Z坐标，单位：米(m)
    uint16_t checksum;        // CRC16校验值，用于数据完整性校验
} rv2_send_protocol_s;

/**
 * @brief RV2协议接收数据包结构体定义
 * @details 该结构体用于定义RV2协议的接收数据包格式，包含了目标跟踪状态、位置信息、速度信息等数据
 * @note 数据采用小端字节序，使用CRC16校验确保数据完整性
 */
typedef struct
{
    uint8_t header;           // 数据包头部，固定值为0xA5
    bool tracking :1;         // 目标跟踪状态：true-正在跟踪目标，false-未跟踪到目标
    uint8_t id :3;            // 目标ID：0-前哨站，1-英雄机器人，2-工程机器人，3-步兵机器人3号，4-步兵机器人4号，5-步兵机器人5号，6-哨兵机器人，7-基地
    uint8_t armors_num :3;    // 装甲板数量：2-平衡步兵，3-前哨站，4-普通机器人
    uint8_t reserved :1;      // 保留位，暂未使用
    float x;                  // 目标X坐标，单位：米(m)
    float y;                  // 目标Y坐标，单位：米(m)
    float z;                  // 目标Z坐标，单位：米(m)
    float yaw;                // 目标偏航角，单位：弧度(rad)
    float vx;                 // 目标X方向速度，单位：米/秒(m/s)
    float vy;                 // 目标Y方向速度，单位：米/秒(m/s)
    float vz;                 // 目标Z方向速度，单位：米/秒(m/s)
    float v_yaw;              // 目标偏航角速度，单位：弧度/秒(rad/s)
    float r1;                 // 目标中心到前后装甲板的距离，单位：米(m)
    float r2;                 // 目标中心到左右装甲板的距离，单位：米(m)
    float dz;                 // 另一对装甲板相对于被跟踪装甲板的高度差，单位：米(m)
    uint16_t checksum;        // CRC16校验值，用于数据完整性校验
} rv2_recv_protocol_s;
#pragma pack()

rv2_recv_protocol_s *rv2_protocol_init(void);
void build_rv2_send_data(Vision_Send_s *send,uint8_t *tx_buf,uint16_t *tx_buf_len);
void parse_rv2_receive_data(Vision_Recv_s *receive, uint8_t *rx_buf, uint16_t rx_buf_len);



#endif //RV2_PROTOCAL_H
