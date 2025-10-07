/*
 * radar_cal.h
 * 激光雷达数据处理头文件
 * 定义了雷达数据解析相关的数据结构、常量和函数声明
 *
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 * 相关部分使用了逐飞Seekfree TC264 Opensource Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
 * 开源库：https://gitee.com/seekfree/TC264_Library
 * Created on: 2025年9月27日
 * 修改记录
 * 日期              作者                备注
 * 2025.10.7        YinYiping           first version
 */

#ifndef CODE_RADAR_CAL_H_
#define CODE_RADAR_CAL_H_

#include <stdint.h>

// 常量定义
#define FRACTION_COUNT 720      // 角度分辨率，例如：将360度划分为720份，每份0.5度
#define RADAR_PACKET_SIZE 47    // LD19雷达数据包大小：1+1+2+2+12*3+2+2+1=47字节
#define POINT_PER_PACK 12       // 每个数据包包含的测量点数量
#define HEADER 0x54             // 数据包起始字节

// 全局变量声明
extern uint8_t radar_state;           // 雷达使能状态
extern int radar_data_ready_flag;     // 雷达数据就绪标志
extern int fifo_used_space;           // FIFO已使用空间
extern int test;                      // 测试变量

/**
 * @brief 单个角度点测量数据结构
 */
typedef struct {
    float distance;        // 距离测量值，单位：mm
    uint8_t movement_flag; // 运动检测标志位
    uint16_t intensity;     // 信号强度值
} fraction_data;

/**
 * @brief 激光雷达参数数据结构
 * 包含雷达状态信息和所有角度点的测量数据
 */
typedef struct {
    uint16_t speed;                           // 雷达转速，单位：度/秒
    float frequency;                          // 扫描频率，单位：Hz
    fraction_data point[FRACTION_COUNT];      // 所有角度点的测量数据数组
    float current_angle;                      // 当前角度，单位：度
    uint16_t timestamp;                       // 时间戳，单位：ms
    uint8_t crc_condition;                    // CRC校验状态：0-校验成功；1-校验失败
} lidar_parameter;

// 全局雷达参数变量声明
extern lidar_parameter lidar;

/**
 * @brief 雷达数据解析状态枚举
 */
typedef enum {
    RADAR_STATE_IDLE = 0,      // 空闲状态，等待数据
    RADAR_STATE_RECEIVED,      // 数据接收完成，等待解析
    RADAR_STATE_PARSING        // 正在解析数据
} radar_state_t;

// 函数声明
/**
 * @brief 手动解析雷达数据包
 * @param data 数据包指针
 * @param len 数据包长度
 * @return 成功返回0，失败返回-1
 */
int ParseLidarDataManual(uint8_t *data, int len);

/**
 * @brief 运动检测函数
 * @param index 角度索引
 * @param distance 当前距离测量值
 */
void movement_detect(int index, float distance);

/**
 * @brief 雷达串口接收回调函数
 * 在串口中断中调用，负责接收并缓存雷达数据
 */
void radar_uart_callback(void);

/**
 * @brief 雷达系统初始化函数
 * 初始化串口、FIFO和雷达状态参数
 */
void radar_init(void);

/**
 * @brief 雷达数据处理函数
 * 在主循环中调用，解析接收到的雷达数据包
 */
void radar_data_process(void);

/**
 * @brief 更新角度距离列表
 * @param degree 角度值（度）
 * @param distance 距离值（mm）
 * @param intensity 信号强度
 */
void distance_list_refresh(float degree, float distance, uint16_t intensity);

#endif /* CODE_RADAR_CAL_H_ */
