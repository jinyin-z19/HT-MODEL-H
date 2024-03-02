#ifndef _Livelybot_Driver_H_
#define _Livelybot_Driver_H_

#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <time.h>
#include "string.h"
#include "Config.h"
#include <chrono>

using namespace std;

using std::chrono::high_resolution_clock;
using std::chrono::duration;
using std::chrono::duration_cast;

#define SPI_SPEED 6000000
#define PI 3.1415926

#define ERROR_TRANSFER_DATA 0xffffaaaa

#pragma anon_unions
#pragma pack(1)  
typedef struct
{
    uint8_t motor_id;
    int32_t motor_cmd;
    int32_t position;
    int32_t velocity;
    int32_t torque;
}motor_fb_space_s;

typedef struct 
{
    union 
    {
        motor_fb_space_s motor;
        uint8_t data[MOTOR_STATUS_LENGTH];
    };
}motor_fb_s;

typedef struct
{
    int16_t accX;
    int16_t accY;
    int16_t accZ;
    int16_t angVelX;
    int16_t angVelY;
    int16_t angVelZ;
    int16_t angle_roll;
    int16_t angle_pitch;
    int16_t angle_yaw;
    int16_t magX;
    int16_t magY;
    int16_t magZ;
}imu_space_s;

typedef struct
{
    union
    {
        imu_space_s imu_data;
        uint8_t data[YJ901S_DATA_SIZE];
    };
}imu_s;

typedef struct
{
    uint8_t motor_id;
    uint8_t motor_cmd;
    int32_t position;
    int32_t velocity;
    int32_t torque;
    float kp;
    float kd; 
}motor_set_space_s;

typedef struct
{
union 
{
    motor_set_space_s motor;
    uint8_t data[MOTOR_SET_LENGTH];
};
}motor_set_s;

#pragma pack ()

typedef enum
{
    ANG2POS = 0x20,
    RAD2POS,
    TOR2TOR,
}tranfer_send_type_e;

typedef enum
{
    POS2ANG = 0x30,
    POS2RAD,
}tranfer_rec_type_e;

class Motor_Status{
public:

    motor_fb_s motor_fb1[14];
    motor_fb_s motor_fb2[14];

    imu_s myimu;

    uint8_t foot_sensor1[3];
    uint8_t foot_sensor2[3];

    Motor_Status(){};
};

// 定义一个简单的类
class Livelybot_Driver {
private:
    string my_spi_dev;
    int8_t my_motor_type;
    int8_t my_can1_motor_num;
    int8_t my_can2_motor_num;
    int8_t my_isenable_imu;
    int8_t my_isenable_footsensor;
    Motor_Status all_motor_status;
    uint8_t spi_tx_databuffer[DATA_PKG_SIZE] = {0};
    uint8_t spi_tx_motor_num = 0;

    int spi_fd;
    uint32_t speed;
    uint16_t delay;
    uint8_t bits_per_word;
    uint8_t mode;
    high_resolution_clock::time_point t1, t2;

    bool spi_stop_flag;
public:
    // 构造函数
    Livelybot_Driver(const string spi_dev);
    ~Livelybot_Driver(){};

    motor_fb_space_s get_motor_state(int8_t motor_id);

    imu_space_s get_imu_data(void);

    uint8_t* get_footsensor_data(uint8_t switch_can);
    bool open_spi(void);
    bool spi_send(void);

    void set_motor_position(int8_t motor_id, int32_t position);
   
    void set_motor_position(int8_t motor_id, int32_t position, int32_t velocity, int32_t torque,float kp, float kd);

    void set_motor_velocity(int8_t motor_id, int32_t velocity);

    void set_motor_torque(int8_t motor_id, int32_t torque);

    int32_t transfer_send(tranfer_send_type_e type, float data);
    float transfer_rec(tranfer_rec_type_e type, int32_t data);

private:
    
    void set_robot_param(int8_t motor_type, int8_t can1_motor_num, int8_t can2_motor_num, uint8_t isenable_imu, uint8_t isenable_footsensor)
    {
        my_motor_type = motor_type;
        my_can1_motor_num = can1_motor_num;
        my_can2_motor_num = can2_motor_num;
        my_isenable_imu = isenable_imu;
        my_isenable_footsensor = isenable_footsensor;
        #ifdef DEBUG
        cout<<"spi_dev:"<<my_spi_dev<<endl;
        printf("motor_type:%u, can1_motor_num:%u, can2_motor_num:%u, enable_imu:%u, footsensor:%u\n", motor_type, can1_motor_num, can2_motor_num, isenable_imu, isenable_footsensor);
        #endif // DEBUG

        int ret;

        //使用SPI1接口
        spi_fd = open(my_spi_dev.data(), O_RDWR);
        if (spi_fd < 0) {
            perror("Error: Cann't open SPI Dev.\n");
            return;
        }

        //配置SPI参数
        speed = SPI_SPEED;
        delay = 0;
        bits_per_word = 8;
        mode = 0;
        ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
        if (ret == -1) {
            perror("Error: SPI_IOC_WR_MODE fault.\n");
            spi_tx_motor_num = 0;
            return;
        }

        ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
        if (ret == -1) {
            perror("Error: SPI_IOC_WR_BITS fault.\n");
            spi_tx_motor_num = 0;
            return;
        }

        ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
        if (ret == -1) {
            perror("Error: SPI_IOC_WR_MAX_SPEED fault.\n");
            spi_tx_motor_num = 0;
            return;
        }

        //发送数据
        spi_tx_databuffer[2] = 0x40;
        spi_tx_databuffer[3] = motor_type;
        spi_tx_databuffer[4] = can1_motor_num;
        spi_tx_databuffer[5] = can2_motor_num;
        spi_tx_databuffer[6] = isenable_imu;
        spi_tx_databuffer[7] = isenable_footsensor;
        spi_tx_databuffer[8] = ENABLE_STOP;
        #ifdef TX_DEBUG
        printf("tx_data: ");
        for(uint16_t i = 0; i < DATA_PKG_SIZE; i++)
        {
            printf("0x%02x,", spi_tx_databuffer[i]);
        }
        printf("\n");
        #endif // 
        uint8_t rx_buf[DATA_PKG_SIZE] = {};
        struct spi_ioc_transfer spi { };
        spi.tx_buf = (unsigned long)spi_tx_databuffer;
        spi.rx_buf = (unsigned long)rx_buf;
        spi.len = sizeof(spi_tx_databuffer);
        spi.delay_usecs = delay;
        spi.speed_hz = speed;
        spi.bits_per_word = bits_per_word;
        // Send wr_addr
        ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi);
        if (ret < 1) {
            perror("Error: SPI_IOC_MESSAGE fault.\n");
            spi_tx_motor_num = 0;
            return;
        }
        #ifdef DEBUG
        printf("transmit suc!\n");
        #endif // DEBUG
        #ifdef RX_DEBUG
        printf("recv data: ");
        for(uint16_t i = 0; i < DATA_PKG_SIZE; i++)
        {
            printf("0x%02x,", rx_buf[i]);
        }
        printf("\n");
        #endif // DEBUG
        parse_datas(rx_buf);
        
        close(spi_fd);
        
    }

    void motor_set(uint8_t motor_id, int32_t cmd, int32_t posorvolt, int32_t vel, int32_t torque, float kp, float kd)
    {
        #ifdef DEBUG
        cout<<"spi_dev:"<<my_spi_dev<<endl;
        #endif
        motor_set_s motor_state;

        motor_state.motor.motor_id = motor_id;
        motor_state.motor.motor_cmd = cmd;
        motor_state.motor.position = posorvolt;
        motor_state.motor.velocity = vel;
        motor_state.motor.torque = torque;
        motor_state.motor.kp = kp;
        motor_state.motor.kd = kd;
        if(spi_tx_motor_num >= (my_can1_motor_num + my_can2_motor_num))
        {
            printf("the motor num overflow!!!!!!\n");
            return;
        }
        memcpy(&spi_tx_databuffer[3 + spi_tx_motor_num * MOTOR_SET_LENGTH], motor_state.data, MOTOR_SET_LENGTH);
        spi_tx_motor_num++;

    }

    bool parse_datas(uint8_t *rx_buf)
    {
        uint8_t temp_id = 0x00;
        uint8_t temp_can = 0x00;
        //解析数据
        if(rx_buf[0] != 0xA6 || rx_buf[1] != 0x6A)
        {
            clear_tx_buffer();
            return false;
        }
        #ifdef PARSE_DEBUG
        printf("pasre_data start!\n");
        #endif
        uint8_t motor_nums = rx_buf[2];
        if(motor_nums != (my_can1_motor_num + my_can2_motor_num))
        {
            clear_tx_buffer();
            return false;
        }
        for(uint8_t i = 0; i < motor_nums; i++)
        {
            temp_can = rx_buf[3 + i * MOTOR_STATUS_LENGTH] & 0xf0;
            temp_id = rx_buf[3 + i * MOTOR_STATUS_LENGTH] & 0x0f;

            if(temp_can == 0x10)
            {
                all_motor_status.motor_fb1[temp_id - 1].motor.motor_id = temp_id;
                memcpy(&all_motor_status.motor_fb1[temp_id - 1].data[1], &rx_buf[3 + i * MOTOR_STATUS_LENGTH + 1], MOTOR_STATUS_LENGTH - 1);
            }
            else if(temp_can == 0x20)
            {
                all_motor_status.motor_fb2[temp_id - 1].motor.motor_id = temp_id;
                memcpy(&all_motor_status.motor_fb2[temp_id - 1].data[1], &rx_buf[3 + i * MOTOR_STATUS_LENGTH + 1], MOTOR_STATUS_LENGTH - 1);
            }
        }
        uint16_t imu_index = 3 + motor_nums * MOTOR_STATUS_LENGTH;
        uint16_t footsensor_index = 3 + motor_nums * MOTOR_STATUS_LENGTH;
        if(my_isenable_imu && rx_buf[imu_index] == 0xAA)
        {
            memcpy(all_motor_status.myimu.data, &rx_buf[imu_index + 1], YJ901S_DATA_SIZE);
            footsensor_index = imu_index + YJ901S_DATA_SIZE;
        }
        if(my_isenable_footsensor && rx_buf[footsensor_index] == 0xBB)
        {
            memcpy(all_motor_status.foot_sensor1, &rx_buf[footsensor_index + 1], 3);
            memcpy(all_motor_status.foot_sensor2, &rx_buf[footsensor_index + 4], 3);
        }
        clear_tx_buffer();
        return true;
    }

    void clear_tx_buffer(void)
    {
        for(uint16_t i = 0; i < DATA_PKG_SIZE; i++)
        {
            spi_tx_databuffer[i] = 0x00;
        }
        spi_tx_databuffer[0] = 0xA5;
        spi_tx_databuffer[1] = 0x5A;
        spi_tx_motor_num = 0;
    }
    
};

#endif // !_Livelybot_Driver_H_
