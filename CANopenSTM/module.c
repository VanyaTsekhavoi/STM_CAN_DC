#include "module.h"
#include "can.h"
#include "tim.h"
#include "adc.h"
#include "CO_OD.h"
#include "dc_motor.h"

//Global variables
extern DMA_HandleTypeDef hdma_adc1;
extern dc_motor_t dc_motor;

void module_init(void)
{
  /***** Init *****/

  /***** Start *****/
  
  HAL_CAN_Start(&hcan1);
}

extern struct sCO_OD_RAM CO_OD_RAM;

void module_poll(void)
{
  CO_OD_RAM.readAnalogueInput16Bit[0] = dc_motor.instantCurrent;
  CO_OD_RAM.readAnalogueInput16Bit[1] = dc_motor.instantVoltage;
}