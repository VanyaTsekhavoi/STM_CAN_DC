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
  uint16_t t1 = 0, t2 = 0, t3 = 0, t4 = 0;
  int32_t timCounter = htim1.Init.Period;

  t1 = CO_OD_RAM.writeAnalogueOutput16Bit[0];
  t2 = CO_OD_RAM.writeAnalogueOutput16Bit[1];
  t3 = CO_OD_RAM.writeAnalogueOutput16Bit[2];
  t4 = CO_OD_RAM.writeAnalogueOutput16Bit[3];

  t3 = CLIP((timCounter + 1) / 2 - t1 - 1, 0, timCounter);

  TIM1->CCR1 = CLIP((timCounter + 1) / 2 + t1 - 1, 0, timCounter);
  TIM1->CCR2 = CLIP((timCounter + 1) / 2 - t1 - 1, 0, timCounter);

  CO_OD_RAM.readAnalogueInput16Bit[0] = dc_motor.instantCurrent;
  CO_OD_RAM.readAnalogueInput16Bit[1] = dc_motor.instantVoltage;
}