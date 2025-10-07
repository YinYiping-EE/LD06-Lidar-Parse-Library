/*
 * radar_cal.c
 * 激光雷达数据处理源文件
 * 实现雷达数据的接收、解析、存储和相关算法处理
 * 详细使用说明请阅读md文件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 * 相关部分使用了逐飞Seekfree TC264 Opensource Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
 * 开源库：https://gitee.com/seekfree/TC264_Library
 * Created on: 2025年9月27日
 * 修改记录
 * 日期              作者                备注
 * 2025.10.7        YinYiping           first version
 */

#include <stdint.h>
#include <stdio.h>
#include "zf_common_headfile.h"
#include "isr_config.h"

// CRC校验表（手册提供）
static const uint8_t CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8
};

// 全局变量定义
int test = 0;                               // 测试变量
int fifo_used_space = 0;                    // FIFO已使用空间计数
int radar_data_ready_flag = 0;              // 雷达数据就绪标志
uint8_t radar_state = 0;                    // 雷达使能状态
uint8_t radar_receiver_fifo_buffer[1024];   // 雷达数据接收FIFO缓冲区
uint8_t radar_packet_buffer[RADAR_PACKET_SIZE]; // 雷达数据包解析缓冲区
lidar_parameter lidar;                      // 雷达参数结构体实例

// FIFO结构体和状态变量
static fifo_struct radar_receiver_fifo;     // 雷达数据FIFO结构体//fifo_struct为逐飞库中对fifo格式的定义
radar_state_t radar_parse_state = RADAR_STATE_IDLE; // 雷达数据包解析状态

/**
 * @brief CRC8校验计算函数
 * @param p 待校验数据指针
 * @param len 数据长度
 * @return 计算得到的CRC8校验值
 */
uint8_t CalCRC8(uint8_t *p, uint8_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc = CrcTable[(crc ^ *p++) & 0xff];
    }
    return crc;
}

/**
 * @brief 激光雷达串口接收回调函数
 * 在串口中断中调用，负责接收雷达数据并进行初步处理。该函数仅供参考，移植至其他单片机需重新写fifo相关部分
 */
void radar_uart_callback(void) {
    if (radar_state) {  // 检查雷达是否启用
        uint8_t dat;

        // 读取串口所有可用数据并存入FIFO
        while (uart_query_byte(UART_3, &dat)) {
            fifo_write_buffer(&radar_receiver_fifo, &dat, 1);
        }

        // 检查FIFO中是否有足够数据
        fifo_used_space = fifo_used(&radar_receiver_fifo);
        if (fifo_used_space >= RADAR_PACKET_SIZE) {
            uint8_t start_byte;
            uint32_t temp_length = 1;

            // 只读方式查看第一个字节（不消耗数据）
            fifo_read_buffer(&radar_receiver_fifo, &start_byte, &temp_length, FIFO_READ_ONLY);

            if (start_byte == 0x54) {  // 检测到数据包起始符
                // 检查数据包长度是否足够
                if (fifo_used(&radar_receiver_fifo) >= RADAR_PACKET_SIZE) {
                    // 如果没有在解析数据，则读取完整数据包
                    if (radar_parse_state != RADAR_STATE_PARSING) {
                        radar_parse_state = RADAR_STATE_RECEIVED;

                        // 读取完整数据包到解析缓冲区
                        temp_length = RADAR_PACKET_SIZE;
                        if (fifo_read_buffer(&radar_receiver_fifo, radar_packet_buffer, &temp_length, FIFO_READ_AND_CLEAN) == FIFO_SUCCESS) {
                            if (temp_length == RADAR_PACKET_SIZE) {
                                // 数据包读取成功，设置标志位供主程序解析
                                radar_data_ready_flag = 1;
                            }
                        }
                    }
                }
            } else {
                // 如果不是起始符0x54，丢弃一个字节以同步数据流
                temp_length = 1;
                fifo_read_buffer(&radar_receiver_fifo, &start_byte, &temp_length, FIFO_READ_AND_CLEAN);
            }
        }
    }
}

/**
 * @brief 雷达系统初始化函数
 * 初始化串口通信、FIFO缓冲区和雷达状态参数
 */
void radar_init(void) {
    // 初始化雷达数据FIFO
    uart_init(GPS_TAU1201_UART, 230400, GPS_TAU1201_RX, GPS_TAU1201_TX); // 初始化串口//使用了uart3为通信接口，与gps模块相同，使用时需注释uart3串口中断中gps相关内容
    uart_rx_interrupt(GPS_TAU1201_UART, 1);                             // 开启接收中断
    fifo_init(&radar_receiver_fifo, FIFO_DATA_8BIT, radar_receiver_fifo_buffer, sizeof(radar_receiver_fifo_buffer));
    radar_state = 1;  // 启用雷达

    // 初始化雷达角度距离数据
    for(int i = 0; i < FRACTION_COUNT; i++) {
        lidar.point[i].distance = 12.0f;   // 初始化所有角度距离为12.0mm
        lidar.point[i].intensity = 0.0f;   // 初始化所有角度信号强度为0
    }

    // 初始化雷达状态
    radar_parse_state = RADAR_STATE_IDLE;
    radar_data_ready_flag = 0;
}

/**
 * @brief 雷达数据处理函数
 * 在主循环中调用，解析接收到的雷达数据包
 */
void radar_data_process(void) {
    if (radar_data_ready_flag) {
        radar_data_ready_flag = 0;
        radar_parse_state = RADAR_STATE_PARSING;

        // 解析雷达数据包
        ParseLidarDataManual(radar_packet_buffer, RADAR_PACKET_SIZE);
        radar_parse_state = RADAR_STATE_IDLE;
    }
}

/**
 * @brief 更新角度距离列表
 * 根据当前测量角度和距离更新对应的数组元素
 * @param degree 测量角度（度）
 * @param distance 测量距离（mm）
 * @param intensity 信号强度
 */
void distance_list_refresh(float degree, float distance, uint16_t intensity) {
    int temp_index = 0;
    static float last_distance = 12000;  // 上一次的距离值
    static int last_index = -2;          // 上一次的角度索引

    if (distance > 12000) distance = 12000;  // 限制最远距离为12000mm

    // 只在有效角度范围内更新数据
    if (degree >= 0 && degree < 360.0f && distance >= 0.0f) {
        // 计算对应的数组索引
        temp_index = (int)(degree / 360.0f * FRACTION_COUNT);

        // 确保索引在有效范围内
        if (temp_index >= FRACTION_COUNT) {
            temp_index = FRACTION_COUNT - 1;
        }

        if (temp_index == last_index) {
            // 如果两次测距在同一角度间隔内，取平均值
            lidar.point[temp_index].distance = (distance + last_distance) / 2;
        } else {
            // 不同角度间隔，进行运动检测并更新距离
            movement_detect(temp_index, distance);
            lidar.point[temp_index].distance = distance;
        }

        // 更新信号强度
        lidar.point[temp_index].intensity = intensity;

        // 保存当前值供下次使用
        last_distance = distance;
        last_index = temp_index;
    }
}

/**
 * @brief 运动检测函数
 * 通过距离变化检测是否有运动发生
 * @param index 角度索引
 * @param distance 当前距离测量值
 */
void movement_detect(int index, float distance) {
    int threshold = 1000;  // 运动检测阈值（mm）

    // 如果距离变化超过阈值，设置运动标志
    if (abs(lidar.point[index].distance - distance) > threshold) {
        lidar.point[index].movement_flag = 1;
    }
}

/**
 * @brief 手动解析雷达数据包函数
 * 解析LD19雷达数据包，提取转速、角度、距离等信息
 * @param data 数据包指针
 * @param len 数据包长度
 * @return 成功返回0，失败返回-1
 */
int ParseLidarDataManual(uint8_t *data, int len) {
    if (len < RADAR_PACKET_SIZE) {
        printf("数据长度不足\n");
        return -1;
    }

    // 手动提取每个字段（LD19/LD06使用小端格式）
    uint8_t header = data[0];                    // 数据包头
    uint8_t ver_len = data[1];                   // 版本和长度
    uint16_t speed = data[2] | (data[3] << 8);   // 雷达转速
    uint16_t start_angle = data[4] | (data[5] << 8); // 起始角度
    uint16_t end_angle = data[42] | (data[43] << 8); // 结束角度
    uint16_t timestamp = data[44] | (data[45] << 8); // 时间戳
    uint8_t crc8 = data[46];                     // CRC校验值

    // 检查起始符和版本长度
    if (header != HEADER || ver_len != 0x2C) {
        printf("数据包头错误\n");
        return -1;
    }

    // CRC校验（计算前46字节）
    uint8_t crc_calc = CalCRC8(data, RADAR_PACKET_SIZE - 1);
    if (crc_calc != crc8) {
        printf("CRC校验失败: 计算值=0x%02X, 接收值=0x%02X\n", crc_calc, crc8);
        lidar.crc_condition = 1;  // 设置CRC校验失败标志
        return -1;
    } else {
        lidar.crc_condition = 0;  // 设置CRC校验成功标志
    }

    // 解析雷达转速和频率
    lidar.speed = speed;
    lidar.frequency = speed / 360.0f;

    // 解析角度信息（转换为度）
    float start_angle_deg = start_angle / 100.0;
    float end_angle_deg = end_angle / 100.0;
    lidar.current_angle = end_angle_deg;

    // 计算每个测量点的角度步长
    float angle_step = (end_angle_deg - start_angle_deg) / (POINT_PER_PACK - 1);

    // 解析每个测量点的距离和强度数据
    for (int i = 0; i < POINT_PER_PACK; i++) {
        int point_offset = 6 + i * 3;  // 每个点占3字节
        int distance = data[point_offset] | (data[point_offset + 1] << 8);
        uint8_t intensity = data[point_offset + 2];

        // 计算当前点的实际角度
        float angle = start_angle_deg + angle_step * i;

        // 更新角度距离列表
        distance_list_refresh(angle, distance, intensity);
    }

    // 解析时间戳
    lidar.timestamp = timestamp;

    return 0;
}
