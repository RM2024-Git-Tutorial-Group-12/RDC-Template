/**
 * @file DR16.cpp
 * @brief Implement the function definition that is declared in DR16.hpp file
 */

#include "DR16.hpp"
#if USE_DR16

namespace DR16
{

/**
 * @brief Define a singleton RcData structure instance here.
 * @remark If you wish to, please decode the DR16 data from the buffer to here
 * @remark Refer to the definition of the structure in the "DR16.hpp" files
 */
static RcData rcData;

/*Return the constant pointer of the current decoded data*/
const RcData *getRcData() { return &rcData; }

/*================================================================================*/
/*You are free to declare your buffer, or implement your own function(callback, decoding) here*/
HAL_Ticks curTime = HAL_GetTick();
HAL_Ticks prevTime = 0;

uint8_t rxBuffer[18] = {0};
bool abnormal = false;

void CallBackFunc(UART_HandleTypeDef* huart, uint16_t s){
    decodeAndValidate(rxBuffer);
    HAL_UARTEx_ReceiveToIdle_IT(&DR16_UART,rxBuffer,18);
}

void errorHandler(){
    // resetting the values to normal
    rcData.channel0 = 1024;
    rcData.channel1 = 1024;
    rcData.channel2 = 1024;
    rcData.channel3 = 1024;
    rcData.s1=1;
    rcData.s2=1;
    // switch no normal state
}

void decodeAndValidate(uint8_t rxBuffer[]){
    abnormal = false;
    if (rxBuffer == NULL){return;}

    if(rxBuffer == NULL){return;}
    rcData.channel0 = ((uint16_t)rxBuffer[0] | ((uint16_t)rxBuffer[1] << 8)) & 0x07FF;
    if (rcData.channel0 < UART_MIN || rcData.channel0 > UART_MAX){abnormal = true;}

    rcData.channel1 = (((uint16_t)rxBuffer[1] >> 3) | ((uint16_t)rxBuffer[2] << 5))& 0x07FF;
    if (rcData.channel1 < UART_MIN || rcData.channel1 > UART_MAX){abnormal = true;}

    rcData.channel2 = (((uint16_t)rxBuffer[2] >> 6) | ((uint16_t)rxBuffer[3] << 2) | ((uint16_t)rxBuffer[4] << 10)) & 0x07FF;
    if (rcData.channel2 < UART_MIN || rcData.channel2 > UART_MAX){abnormal = true;}

    rcData.channel3 = (((uint16_t)rxBuffer[4] >> 1) | ((uint16_t)rxBuffer[5]<<7)) & 0x07FF;
    if (rcData.channel3 < UART_MIN || rcData.channel3 > UART_MAX){abnormal = true;}

    rcData.s1 = ((rxBuffer[5] >> 4) & 0x000C) >> 2;
    if(rcData.s1 < 1 || rcData.s1 > 3){abnormal = true;}

    rcData.s2 = ((rxBuffer[5] >> 4) & 0x0003);
    if(rcData.s2 < 1 || rcData.s2 > 3){abnormal = true;}

    if (abnormal){errorHandler();}
    curTime = HAL_GetTick();
    prevTime = curTime;
}

/*================================================================================*/
void init()
{
    if (!HAL_UART_RegisterRxEventCallback(&DR16_UART,CallBackFunc)){
        HAL_UARTEx_ReceiveToIdle_IT(&DR16_UART,rxBuffer,18);
    }
    
}

}  // namespace DR16

#endif