/*
 * radar_cal.h
 * �����״����ݴ���ͷ�ļ�
 * �������״����ݽ�����ص����ݽṹ�������ͺ�������
 *
 * �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
 * �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
 * ��ز���ʹ�������Seekfree TC264 Opensource Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
 * ��Դ�⣺https://gitee.com/seekfree/TC264_Library
 * Created on: 2025��9��27��
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2025.10.7        YinYiping           first version
 */

#ifndef CODE_RADAR_CAL_H_
#define CODE_RADAR_CAL_H_

#include <stdint.h>

// ��������
#define FRACTION_COUNT 720      // �Ƕȷֱ��ʣ����磺��360�Ȼ���Ϊ720�ݣ�ÿ��0.5��
#define RADAR_PACKET_SIZE 47    // LD19�״����ݰ���С��1+1+2+2+12*3+2+2+1=47�ֽ�
#define POINT_PER_PACK 12       // ÿ�����ݰ������Ĳ���������
#define HEADER 0x54             // ���ݰ���ʼ�ֽ�

// ȫ�ֱ�������
extern uint8_t radar_state;           // �״�ʹ��״̬
extern int radar_data_ready_flag;     // �״����ݾ�����־
extern int fifo_used_space;           // FIFO��ʹ�ÿռ�
extern int test;                      // ���Ա���

/**
 * @brief �����Ƕȵ�������ݽṹ
 */
typedef struct {
    float distance;        // �������ֵ����λ��mm
    uint8_t movement_flag; // �˶�����־λ
    uint16_t intensity;     // �ź�ǿ��ֵ
} fraction_data;

/**
 * @brief �����״�������ݽṹ
 * �����״�״̬��Ϣ�����нǶȵ�Ĳ�������
 */
typedef struct {
    uint16_t speed;                           // �״�ת�٣���λ����/��
    float frequency;                          // ɨ��Ƶ�ʣ���λ��Hz
    fraction_data point[FRACTION_COUNT];      // ���нǶȵ�Ĳ�����������
    float current_angle;                      // ��ǰ�Ƕȣ���λ����
    uint16_t timestamp;                       // ʱ�������λ��ms
    uint8_t crc_condition;                    // CRCУ��״̬��0-У��ɹ���1-У��ʧ��
} lidar_parameter;

// ȫ���״������������
extern lidar_parameter lidar;

/**
 * @brief �״����ݽ���״̬ö��
 */
typedef enum {
    RADAR_STATE_IDLE = 0,      // ����״̬���ȴ�����
    RADAR_STATE_RECEIVED,      // ���ݽ�����ɣ��ȴ�����
    RADAR_STATE_PARSING        // ���ڽ�������
} radar_state_t;

// ��������
/**
 * @brief �ֶ������״����ݰ�
 * @param data ���ݰ�ָ��
 * @param len ���ݰ�����
 * @return �ɹ�����0��ʧ�ܷ���-1
 */
int ParseLidarDataManual(uint8_t *data, int len);

/**
 * @brief �˶���⺯��
 * @param index �Ƕ�����
 * @param distance ��ǰ�������ֵ
 */
void movement_detect(int index, float distance);

/**
 * @brief �״ﴮ�ڽ��ջص�����
 * �ڴ����ж��е��ã�������ղ������״�����
 */
void radar_uart_callback(void);

/**
 * @brief �״�ϵͳ��ʼ������
 * ��ʼ�����ڡ�FIFO���״�״̬����
 */
void radar_init(void);

/**
 * @brief �״����ݴ�����
 * ����ѭ���е��ã��������յ����״����ݰ�
 */
void radar_data_process(void);

/**
 * @brief ���½ǶȾ����б�
 * @param degree �Ƕ�ֵ���ȣ�
 * @param distance ����ֵ��mm��
 * @param intensity �ź�ǿ��
 */
void distance_list_refresh(float degree, float distance, uint16_t intensity);

#endif /* CODE_RADAR_CAL_H_ */
