/*
 * radar_cal.c
 * �����״����ݴ���Դ�ļ�
 * ʵ���״����ݵĽ��ա��������洢������㷨����
 * ��ϸʹ��˵�����Ķ�md�ļ�
 * �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
 * �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
 * ��ز���ʹ�������Seekfree TC264 Opensource Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
 * ��Դ�⣺https://gitee.com/seekfree/TC264_Library
 * Created on: 2025��9��27��
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2025.10.7        YinYiping           first version
 */

#include <stdint.h>
#include <stdio.h>
#include "zf_common_headfile.h"
#include "isr_config.h"

// CRCУ����ֲ��ṩ��
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

// ȫ�ֱ�������
int test = 0;                               // ���Ա���
int fifo_used_space = 0;                    // FIFO��ʹ�ÿռ����
int radar_data_ready_flag = 0;              // �״����ݾ�����־
uint8_t radar_state = 0;                    // �״�ʹ��״̬
uint8_t radar_receiver_fifo_buffer[1024];   // �״����ݽ���FIFO������
uint8_t radar_packet_buffer[RADAR_PACKET_SIZE]; // �״����ݰ�����������
lidar_parameter lidar;                      // �״�����ṹ��ʵ��

// FIFO�ṹ���״̬����
static fifo_struct radar_receiver_fifo;     // �״�����FIFO�ṹ��//fifo_structΪ��ɿ��ж�fifo��ʽ�Ķ���
radar_state_t radar_parse_state = RADAR_STATE_IDLE; // �״����ݰ�����״̬

/**
 * @brief CRC8У����㺯��
 * @param p ��У������ָ��
 * @param len ���ݳ���
 * @return ����õ���CRC8У��ֵ
 */
uint8_t CalCRC8(uint8_t *p, uint8_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc = CrcTable[(crc ^ *p++) & 0xff];
    }
    return crc;
}

/**
 * @brief �����״ﴮ�ڽ��ջص�����
 * �ڴ����ж��е��ã���������״����ݲ����г��������ú��������ο�����ֲ��������Ƭ��������дfifo��ز���
 */
void radar_uart_callback(void) {
    if (radar_state) {  // ����״��Ƿ�����
        uint8_t dat;

        // ��ȡ�������п������ݲ�����FIFO
        while (uart_query_byte(UART_3, &dat)) {
            fifo_write_buffer(&radar_receiver_fifo, &dat, 1);
        }

        // ���FIFO���Ƿ����㹻����
        fifo_used_space = fifo_used(&radar_receiver_fifo);
        if (fifo_used_space >= RADAR_PACKET_SIZE) {
            uint8_t start_byte;
            uint32_t temp_length = 1;

            // ֻ����ʽ�鿴��һ���ֽڣ����������ݣ�
            fifo_read_buffer(&radar_receiver_fifo, &start_byte, &temp_length, FIFO_READ_ONLY);

            if (start_byte == 0x54) {  // ��⵽���ݰ���ʼ��
                // ������ݰ������Ƿ��㹻
                if (fifo_used(&radar_receiver_fifo) >= RADAR_PACKET_SIZE) {
                    // ���û���ڽ������ݣ����ȡ�������ݰ�
                    if (radar_parse_state != RADAR_STATE_PARSING) {
                        radar_parse_state = RADAR_STATE_RECEIVED;

                        // ��ȡ�������ݰ�������������
                        temp_length = RADAR_PACKET_SIZE;
                        if (fifo_read_buffer(&radar_receiver_fifo, radar_packet_buffer, &temp_length, FIFO_READ_AND_CLEAN) == FIFO_SUCCESS) {
                            if (temp_length == RADAR_PACKET_SIZE) {
                                // ���ݰ���ȡ�ɹ������ñ�־λ�����������
                                radar_data_ready_flag = 1;
                            }
                        }
                    }
                }
            } else {
                // ���������ʼ��0x54������һ���ֽ���ͬ��������
                temp_length = 1;
                fifo_read_buffer(&radar_receiver_fifo, &start_byte, &temp_length, FIFO_READ_AND_CLEAN);
            }
        }
    }
}

/**
 * @brief �״�ϵͳ��ʼ������
 * ��ʼ������ͨ�š�FIFO���������״�״̬����
 */
void radar_init(void) {
    // ��ʼ���״�����FIFO
    uart_init(GPS_TAU1201_UART, 230400, GPS_TAU1201_RX, GPS_TAU1201_TX); // ��ʼ������//ʹ����uart3Ϊͨ�Žӿڣ���gpsģ����ͬ��ʹ��ʱ��ע��uart3�����ж���gps�������
    uart_rx_interrupt(GPS_TAU1201_UART, 1);                             // ���������ж�
    fifo_init(&radar_receiver_fifo, FIFO_DATA_8BIT, radar_receiver_fifo_buffer, sizeof(radar_receiver_fifo_buffer));
    radar_state = 1;  // �����״�

    // ��ʼ���״�ǶȾ�������
    for(int i = 0; i < FRACTION_COUNT; i++) {
        lidar.point[i].distance = 12.0f;   // ��ʼ�����нǶȾ���Ϊ12.0mm
        lidar.point[i].intensity = 0.0f;   // ��ʼ�����нǶ��ź�ǿ��Ϊ0
    }

    // ��ʼ���״�״̬
    radar_parse_state = RADAR_STATE_IDLE;
    radar_data_ready_flag = 0;
}

/**
 * @brief �״����ݴ�����
 * ����ѭ���е��ã��������յ����״����ݰ�
 */
void radar_data_process(void) {
    if (radar_data_ready_flag) {
        radar_data_ready_flag = 0;
        radar_parse_state = RADAR_STATE_PARSING;

        // �����״����ݰ�
        ParseLidarDataManual(radar_packet_buffer, RADAR_PACKET_SIZE);
        radar_parse_state = RADAR_STATE_IDLE;
    }
}

/**
 * @brief ���½ǶȾ����б�
 * ���ݵ�ǰ�����ǶȺ;�����¶�Ӧ������Ԫ��
 * @param degree �����Ƕȣ��ȣ�
 * @param distance �������루mm��
 * @param intensity �ź�ǿ��
 */
void distance_list_refresh(float degree, float distance, uint16_t intensity) {
    int temp_index = 0;
    static float last_distance = 12000;  // ��һ�εľ���ֵ
    static int last_index = -2;          // ��һ�εĽǶ�����

    if (distance > 12000) distance = 12000;  // ������Զ����Ϊ12000mm

    // ֻ����Ч�Ƕȷ�Χ�ڸ�������
    if (degree >= 0 && degree < 360.0f && distance >= 0.0f) {
        // �����Ӧ����������
        temp_index = (int)(degree / 360.0f * FRACTION_COUNT);

        // ȷ����������Ч��Χ��
        if (temp_index >= FRACTION_COUNT) {
            temp_index = FRACTION_COUNT - 1;
        }

        if (temp_index == last_index) {
            // ������β����ͬһ�Ƕȼ���ڣ�ȡƽ��ֵ
            lidar.point[temp_index].distance = (distance + last_distance) / 2;
        } else {
            // ��ͬ�Ƕȼ���������˶���Ⲣ���¾���
            movement_detect(temp_index, distance);
            lidar.point[temp_index].distance = distance;
        }

        // �����ź�ǿ��
        lidar.point[temp_index].intensity = intensity;

        // ���浱ǰֵ���´�ʹ��
        last_distance = distance;
        last_index = temp_index;
    }
}

/**
 * @brief �˶���⺯��
 * ͨ������仯����Ƿ����˶�����
 * @param index �Ƕ�����
 * @param distance ��ǰ�������ֵ
 */
void movement_detect(int index, float distance) {
    int threshold = 1000;  // �˶������ֵ��mm��

    // �������仯������ֵ�������˶���־
    if (abs(lidar.point[index].distance - distance) > threshold) {
        lidar.point[index].movement_flag = 1;
    }
}

/**
 * @brief �ֶ������״����ݰ�����
 * ����LD19�״����ݰ�����ȡת�١��Ƕȡ��������Ϣ
 * @param data ���ݰ�ָ��
 * @param len ���ݰ�����
 * @return �ɹ�����0��ʧ�ܷ���-1
 */
int ParseLidarDataManual(uint8_t *data, int len) {
    if (len < RADAR_PACKET_SIZE) {
        printf("���ݳ��Ȳ���\n");
        return -1;
    }

    // �ֶ���ȡÿ���ֶΣ�LD19/LD06ʹ��С�˸�ʽ��
    uint8_t header = data[0];                    // ���ݰ�ͷ
    uint8_t ver_len = data[1];                   // �汾�ͳ���
    uint16_t speed = data[2] | (data[3] << 8);   // �״�ת��
    uint16_t start_angle = data[4] | (data[5] << 8); // ��ʼ�Ƕ�
    uint16_t end_angle = data[42] | (data[43] << 8); // �����Ƕ�
    uint16_t timestamp = data[44] | (data[45] << 8); // ʱ���
    uint8_t crc8 = data[46];                     // CRCУ��ֵ

    // �����ʼ���Ͱ汾����
    if (header != HEADER || ver_len != 0x2C) {
        printf("���ݰ�ͷ����\n");
        return -1;
    }

    // CRCУ�飨����ǰ46�ֽڣ�
    uint8_t crc_calc = CalCRC8(data, RADAR_PACKET_SIZE - 1);
    if (crc_calc != crc8) {
        printf("CRCУ��ʧ��: ����ֵ=0x%02X, ����ֵ=0x%02X\n", crc_calc, crc8);
        lidar.crc_condition = 1;  // ����CRCУ��ʧ�ܱ�־
        return -1;
    } else {
        lidar.crc_condition = 0;  // ����CRCУ��ɹ���־
    }

    // �����״�ת�ٺ�Ƶ��
    lidar.speed = speed;
    lidar.frequency = speed / 360.0f;

    // �����Ƕ���Ϣ��ת��Ϊ�ȣ�
    float start_angle_deg = start_angle / 100.0;
    float end_angle_deg = end_angle / 100.0;
    lidar.current_angle = end_angle_deg;

    // ����ÿ��������ĽǶȲ���
    float angle_step = (end_angle_deg - start_angle_deg) / (POINT_PER_PACK - 1);

    // ����ÿ��������ľ����ǿ������
    for (int i = 0; i < POINT_PER_PACK; i++) {
        int point_offset = 6 + i * 3;  // ÿ����ռ3�ֽ�
        int distance = data[point_offset] | (data[point_offset + 1] << 8);
        uint8_t intensity = data[point_offset + 2];

        // ���㵱ǰ���ʵ�ʽǶ�
        float angle = start_angle_deg + angle_step * i;

        // ���½ǶȾ����б�
        distance_list_refresh(angle, distance, intensity);
    }

    // ����ʱ���
    lidar.timestamp = timestamp;

    return 0;
}
