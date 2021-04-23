#include "dc_motor.h"
#include "math_macro.h"
#include "pprof.h"
#include "phase_filter.h"
#include "stm32f7xx_ll_dma.h"

//#include "stm32f767xx.h"
//#include "stm32f7xx_hal_dma.h"
#include "tim.h"
#include "adc.h"
#include "dma.h"
#include "gpio.h"

/***** Periphery *****/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;

/***** Global *****/
dc_motor_t dc_motor;

//Transmission
int16_t adc1[16] = {
    0,
};
uint16_t bufferData[2] = {
    0,
};

volatile int8_t halfTransferState = 0;
volatile int8_t transferState = 0;

//PWM
double timCounter = 0;

//PID
double P = 3, I = 0.5, D = 3;
double velocityError = 0;
double previosVelocityError = 0;
double currentError = 0;
double previosCurrentError = 0;
double angleError = 0;
double previosAngleError = 0;
double integralSummVelocity = 0;
double integralSummCurrent = 0;
double integralSummAngle = 0;
double requiredVoltage = 0;
double requiredCurrent = 0;
double requiredVelocity = 0;
double requiredAngle = 0;
double command = 0;

//Velocity and Position
double instantVelocity = 0;
double instantAngle = 0;
double instantCurrent = 0;
double instantVoltage = 0;
double bPVoltage = 0;
double lastAngle = 0;
int32_t baseAngle = 0;
int64_t instantAngle64 = 0;
int64_t lastAngle64 = 0;
uint32_t lastAngle32 = 0;
uint32_t effectiveAngle32 = 0;

//FF + Filter
volatile pprof_t p;
volatile phase_filter_cfg_t filterConfiguration;
volatile phase_filter_t filter;
double ReqFFAcceleration = 0;
double ReqFFVelocity = 0;
double ReqFFAngle = 0;

//General
double currentOffset = 0;
double k_knstruct = 0.0047, sampleLength = 1.0 / 12500,
       R = 35, J = 0.0000875;
uint8_t curr = 0;
double currentSensorKoeff = 0.00653;
double bpVoltageKoeff = 0.004833984;

static inline void dma_copy()
{
    
    /***** ADC conversion *****/

    transferState = LL_DMA_IsActiveFlag_TC0(DMA2);
    halfTransferState = LL_DMA_IsActiveFlag_HT0(DMA2);

    if (halfTransferState)
    {
        DMA2->LIFCR = DMA_FLAG_HTIF0_4 << hdma_adc1.StreamIndex;
    }

    if (transferState)
    {
        DMA2->LIFCR = DMA_FLAG_TCIF0_4 << hdma_adc1.StreamIndex;
    }

    //Copy Data from registers
    if (halfTransferState == 1)
    {
        bufferData[0] = bufferData[1] = 0;

        for (int i = 0; i < 8; i += 2)
        {
            bufferData[0] += adc1[i];     // Current
            bufferData[1] += adc1[i + 1]; // Voltage
        }
    }

    if (transferState == 1)
    {
        bufferData[0] = bufferData[1] = 0;

        for (int i = 8; i < 16; i += 2)
        {
            bufferData[0] += adc1[i];     // Current
            bufferData[1] += adc1[i + 1]; // Voltage
        }
    }

    /*
    transferState = READ_BIT(DMA2->LISR, DMA_LISR_TCIF0) == (DMA_LISR_TCIF0);
    halfTransferState = READ_BIT(DMA2->LISR, DMA_LISR_HTIF0) == (DMA_LISR_HTIF0);

    bufferData[0] = bufferData[1] = 0;

    //Copy Data from registers
    if (halfTransferState)
    {
        DMA2->LIFCR = DMA_FLAG_HTIF0_4 << hdma_adc1.StreamIndex;

        for (int i = 0; i < 4; i += 2)
        {
            bufferData[0] += adc1[i];     // Current
            bufferData[1] += adc1[i + 1]; // Voltage
        }
    }

    if (transferState)
    {
        DMA2->LIFCR = DMA_FLAG_TCIF0_4 << hdma_adc1.StreamIndex;

        for (int i = 4; i < 8; i += 2)
        {
            bufferData[0] += adc1[i];     // Current
            bufferData[1] += adc1[i + 1]; // Voltage
        }
    }
*/
    bufferData[0] /= 4; // Current
    bufferData[1] /= 4; // Voltage
    
    
}

static inline void current_offset()
{
    if (curr == 128)
    {
        currentOffset /= curr;
        curr++;
    }
    else if (curr < 128)
    {
        currentOffset += bufferData[0];
        curr++;
    }
    instantCurrent = (bufferData[0] - currentOffset); // * currentSensorKoeff;
}

static inline void theoreticalAngle_Velocity()
{
    static uint8_t tick = 0;
    static double tempAngle = 0;

    double tempVelocity = (instantVoltage - instantCurrent * R) / k_knstruct;
    tempAngle += tempVelocity * sampleLength;

    if (++tick != 5)
    {
        double effectiveAngle = tempAngle / 5;

        lastAngle64 = instantAngle64;
        lastAngle32 = effectiveAngle32;
        //Representing angle to int value
        effectiveAngle32 = effectiveAngle / PI * 4294967295; //2147483647
        instantAngle64 = instantAngle64 + (int64_t)effectiveAngle32 - lastAngle32;

        //Angle + (right)
        if (lastAngle64 > instantAngle64 + 2147483647)
        {
            baseAngle += 1;
        }
        //Angle - (left)
        else if (lastAngle64 + 2147483647 < instantAngle64)
        {
            baseAngle -= 1;
        }
        instantAngle64 = baseAngle * 4294967295 + effectiveAngle32;

        //filter

        lastAngle = instantAngle;
        instantAngle = (double)instantAngle64 / 4294967295 * PI;

        tick = 0;
    }

    //filtered velocity

    phase_filter_update(&filter, &filterConfiguration, fit_phase(instantAngle), sampleLength);
    instantVelocity = filter.we;
}

static inline void motor_control_loop()
{
    theoreticalAngle_Velocity();
    pprof_process(&p, ReqFFAngle, ReqFFVelocity, ReqFFAcceleration, sampleLength);

    /***** Angle *****/

    angleError = p.p - instantAngle;

    //loop
    {
        integralSummAngle += 0.0 * angleError * sampleLength;

        //integral windup angle
        integralSummAngle = CLIPS(integralSummAngle, 200);

        //Resulting command
        command = (1.0 * angleError) + integralSummAngle;
        previosAngleError = angleError;
    }

    //Required Velocity through P
    requiredVelocity = command + p.v;
    requiredVelocity = CLIPS(requiredVelocity, 200);

    /***** Velocity *****/
    {
        //loop 2
        {
            velocityError = requiredVelocity - instantVelocity;
            integralSummVelocity += velocityError * sampleLength * 0.2;

            //integral windup velocity
            integralSummVelocity = CLIPS(integralSummVelocity, 1);

            //Resulting command
            command = (0.1 * velocityError) + integralSummVelocity;

            previosVelocityError = velocityError;
        }

        //Required Current through PI
        requiredCurrent = command + (p.a * J + 0.00063125) / 0.0175;
        requiredCurrent = CLIPS(requiredCurrent, 1);

        /***** Current *****/
        {
            //loop 3
            {
                currentError = requiredCurrent - instantCurrent;
                integralSummCurrent += currentError * sampleLength * 100.0;

                //integral windup Current
                integralSummCurrent = CLIPS(integralSummCurrent, 19.5);

                //Resulting command
                command = (2.0 * currentError) + integralSummCurrent;
                previosCurrentError = currentError;
            }

            //Required Voltage through PI
            requiredVoltage = command;
            requiredVoltage = CLIPS(requiredVoltage, bPVoltage);

            static double voltf = 0;
            voltf = 0.95 * voltf + 0.05 * instantVoltage;

            //Voltage loop 4
            //Proportional Load Voltage
            {
                TIM1->CCR1 = (timCounter + 1) / 2 - (timCounter + 1) * (requiredVoltage) / 2 / bPVoltage - 1;
                TIM1->CCR2 = (timCounter + 1) / 2 + (timCounter + 1) * (requiredVoltage) / 2 / bPVoltage - 1;
            }
        }
    }
}

void dc_motor_init()
{
    /***** Init *****/

    filterConfiguration.ki = 20000.0;
    filterConfiguration.kp = 120.0;
    filterConfiguration.w_max = 10000.0;

    double timCounter = htim1.Init.Period;

    /***** Start *****/

    phase_filter_init(&filter);
    pprof_init(&p);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc1, 16);
    TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void dc_motor_poll()
{
    /***** DMA transfer *****/

    dma_copy();

    /***** Current offset computing *****/

    current_offset();

    int PWM = TIM1->CCR1;
    bPVoltage = bufferData[1] * bpVoltageKoeff;
    instantVoltage = bufferData[1]; //bPVoltage * PWM / (timCounter + 1);

    if (0)
    {
        motor_control_loop();
    }

    /***** Output *****/
    dc_motor.instantVoltage = instantVoltage;
    dc_motor.instantCurrent = instantCurrent;
    dc_motor.instantVelocity = instantVelocity;
    dc_motor.instantAngle = instantAngle;
}