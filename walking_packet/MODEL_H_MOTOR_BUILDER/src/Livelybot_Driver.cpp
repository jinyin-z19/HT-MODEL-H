#include "Livelybot_Driver.h"
#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <time.h>
#include "string.h"

using namespace std;


// 构造函数
/*
spi_dev：SPI设备
*/
Livelybot_Driver::Livelybot_Driver(const string spi_dev) {
    #ifdef DEBUG
    cout<<"init_class spi_dev:"<<spi_dev<<endl;
    printf("motor_type:%u, can1_motor_num:%u, can2_motor_num:%u\n", MOTOR_TYPE, CAN1_NUM, CAN2_NUM);
    #endif // DEBUG

    spi_stop_flag = false;
    my_spi_dev = spi_dev;
    spi_tx_motor_num = 0;
    spi_tx_databuffer[0] = 0xA5;
    spi_tx_databuffer[1] = 0x5A;
    set_robot_param(MOTOR_TYPE, CAN1_NUM, CAN2_NUM, ENABLE_IMU, ENABLE_FOOTSENSOR);
    t1 = high_resolution_clock::now();
    usleep(1000);
    if(!open_spi())
    {
        printf("open spi fail!!!!!!!!!\n");
        exit(0);
    }
}

motor_fb_space_s Livelybot_Driver::get_motor_state(int8_t motor_id)
{
    int8_t switch_can = motor_id & 0xf0;
    int8_t id = motor_id & 0x0f;
    if(switch_can == 0x10)
    {
        return all_motor_status.motor_fb1[id-1].motor;
    }
    if(switch_can == 0x20)
    {
        return all_motor_status.motor_fb2[id-1].motor;
    }
    return *(motor_fb_space_s*)nullptr;
}

imu_space_s Livelybot_Driver::get_imu_data()
{
    return all_motor_status.myimu.imu_data;
}

uint8_t* Livelybot_Driver::get_footsensor_data(uint8_t switch_can)
{
    uint8_t zero_sensor[3] = {0, 0, 0};
    if(switch_can == FOOTSENSOR1)
    {
        return all_motor_status.foot_sensor1;
    }
    else
    {
        return all_motor_status.foot_sensor2;
    }


}

bool Livelybot_Driver::open_spi(void)
{
    int ret;
    //使用SPI接口
    spi_fd = open(my_spi_dev.data(), O_RDWR);
    if (spi_fd < 0) {
        perror("Error: Cann't open SPI Dev.\n");
        return false;
    }

    //配置SPI参数
    speed = SPI_SPEED;
    delay = 0;
    bits_per_word = 8;
    mode = 0;
    ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1) {
        perror("Error: SPI_IOC_WR_MODE fault.\n");
        return false;
    }

    ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
    if (ret == -1) {
        perror("Error: SPI_IOC_WR_BITS fault.\n");
        return false;
    }

    ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret == -1) {
        perror("Error: SPI_IOC_WR_MAX_SPEED fault.\n");
        return false;
    }
    return true;
}
bool Livelybot_Driver::spi_send(void)
{
    t2 = high_resolution_clock::now();
    uint64_t time_use = duration_cast<std::chrono::microseconds>(t2 - t1).count();
    if(spi_stop_flag)
    {
        if(time_use < 10000)
        {
            clear_tx_buffer();
            return false;
        }
        else
        {
            spi_stop_flag = false;
            open_spi();
            t1 = t2;
        }
    }
    else
    {
        if(time_use < 1000)
        {
            clear_tx_buffer();
            return false;
        }
        else
        {
            t1 = t2;
        }
    }
    
    //发送数据
    spi_tx_databuffer[2] = spi_tx_motor_num | 0x20;
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
    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi);
    if (ret < 1) {
        perror("Error: SPI_IOC_MESSAGE fault.\n");
        return false;
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
    //解析数据
    if(!parse_datas(rx_buf))
    {
        spi_stop_flag = true;
        clear_tx_buffer();
        close(spi_fd);
        return false;
    }
    // //关闭spi
    // close(spi_fd);
    return true;
}

//位置模式
/*
motor_id:   电机所在的can和id
position：  位置 
*/
void Livelybot_Driver::set_motor_position(int8_t motor_id, int32_t position)
{
    motor_set(motor_id, 5, position, 0, 0, 0, 0);
}
//位置PD模式
/*
motor_id:    电机所在的can和id
position：   位置  
velocity:    速度
kp：         kp值
kd：         kd值
*/
void Livelybot_Driver::set_motor_position(int8_t motor_id, int32_t position, int32_t velocity, int32_t torque, float kp, float kd)
{
    motor_set(motor_id, 7, position, velocity, torque, kp, kd);
}

//速度模式
/*
motor_id: 电机所在的can和id
velocity：速度    
*/
void Livelybot_Driver::set_motor_velocity(int8_t motor_id, int32_t velocity)
{
    motor_set(motor_id, 6, velocity, 0, 0, 0, 0);
}

//力矩模式（单边）
/*
switch_can：选择can线
motor_id: 电机id
torque：力矩   单位：0.01N-M 
*/
void Livelybot_Driver::set_motor_torque(int8_t motor_id, int32_t torque)
{
    motor_set(motor_id, 2, torque, 0, 0, 0, 0);
}

    
int32_t Livelybot_Driver::transfer_send(tranfer_send_type_e type, float data)
{
    int32_t res = 0;
    switch (type)
    {
    case ANG2POS:
        res = data / 360.0 * 100000;
        return res; 
    case RAD2POS:
        res = data / (2 * PI) * 100000;
        return res; 
    case TOR2TOR :
        res = data * 100;
        return res; 
    
    default:
        
        break;
    }
    return ERROR_TRANSFER_DATA;
}

float Livelybot_Driver::transfer_rec(tranfer_rec_type_e type, int32_t data)
{
    float res = 0;
    switch (type)
    {
    case POS2ANG:
        res = (float)(data / 100000.0) * 360.0;
        return res; 
    case POS2RAD:
        res = (float)(data / 100000.0) * (2 * PI);
        return res; 
    default:
        
        break;
    }
    return ERROR_TRANSFER_DATA;
}
