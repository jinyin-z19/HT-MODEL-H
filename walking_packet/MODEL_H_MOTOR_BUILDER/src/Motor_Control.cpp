#include "Livelybot_Driver.h"

#define ALL_MOTOR_NUM (CAN1_NUM+CAN2_NUM)
#define MYTIMES 100000

extern "C" {

void joints_move(float angle[])
{
    Livelybot_Driver my_Driver("/dev/spidev4.1");   
    for(uint8_t i = 0; i < CAN1_NUM; i++)
    {
        uint8_t temp_id = 0x10 | (i+1); 
        int32_t num = my_Driver.transfer_send(RAD2POS, angle[i]);
        my_Driver.set_motor_position(temp_id, num);
        
    }
    for(uint8_t i = 0; i < CAN2_NUM; i++)
    {
        uint8_t temp_id = 0x20 | (i+1); 
        int32_t num = my_Driver.transfer_send(RAD2POS, angle[i + CAN1_NUM]);
        my_Driver.set_motor_position(temp_id, num);
    } 
    my_Driver.spi_send();   
}

void joints_state(float angle[])
{
    Livelybot_Driver my_Driver("/dev/spidev4.1");   
    for(uint8_t i = 0; i < CAN1_NUM; i++)
    {
        uint8_t temp_id = 0x10 | (i+1); 
        int32_t ang =  my_Driver.get_motor_state(temp_id).position;
        angle[i]  =  my_Driver.transfer_rec(POS2RAD, ang);
    }
    for(uint8_t i = 0; i < CAN2_NUM; i++)
    {
        uint8_t temp_id = 0x20 | (i+1); 
        int32_t ang =  my_Driver.get_motor_state(temp_id).position;    
        angle[i + CAN1_NUM]  =  my_Driver.transfer_rec(POS2RAD, ang);
    }    
       
}

}
