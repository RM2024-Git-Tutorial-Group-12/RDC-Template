/**
 * @file UserTask.cpp
 * @author JIANG Yicheng  RM2024 (EthenJ@outlook.sg)
 * @author GUO, Zilin
 * @brief RDC user task files
 * @version 0.3
 * @date 2022-10-20
 *
 * @copyright Copyright (c) 2024
 */
#include "AppConfig.h"   // Include our customized configuration
#include "DJIMotor.hpp"  // Include DR16
#include "DR16.hpp"
#include "FreeRTOS.h"  // Include FreeRTOS.h
#include "PID.hpp"     // Include PID
#include "main.h"
#include "task.h"  // Include task
#include "can.h"
#include "tim.h"

/*Allocate the stack for our PID task*/
StackType_t DR16TaskStack[configMINIMAL_STACK_SIZE];
StackType_t CANWheelTaskStack[configMINIMAL_STACK_SIZE];
StackType_t CANArmTaskStack[configMINIMAL_STACK_SIZE];
/*Declare the PCB for our PID task*/
StaticTask_t DR16TaskTCB;
StaticTask_t CANWheelTaskTCB;
StaticTask_t CANArmTaskTCB;
// StaticTask_t CANArmTaskTCB;

static DR16::RcData uartSnapshot;
const float MotorPID[4][3] = {{1.3,0.1,0.3},{1.3,0.1,0.3},{1.3,0.1,0.3},{1.3,0.1,0.3}};
const float ArmPID[2][3] = {{0.75,2.5,0.5},{10,2.5,0.5}};

static DJIMotor::MotorPair wheels = DJIMotor::MotorPair(1,4,MotorPID);
static DJIMotor::MotorPair arms = DJIMotor::MotorPair(5,2,ArmPID);
/**
 * @todo Show your control outcome of the M3508 motor as follows
 */
bool connected = true;
void DR16Communication(void *)
{
    /* Your user layer codes begin here*/
    /*=================================================*/
    
    /* Your user layer codes end here*/
    /*=================================================*/
    while (true)
    {
        /* Your user layer codes in loop begin here*/
        /*=================================================*/
        DR16::curTime = HAL_GetTick();
        // Intialize the DR16 driver
        if (DR16::curTime - DR16::prevTime > 25){
            connected = false;
            DR16::errorHandler();
        }
        else{
            connected = true;
        }
        /* Your user layer codes in loop end here*/
        /*=================================================*/
        uartSnapshot = *DR16::getRcData();
        vTaskDelay(1);  // Delay and block the task for 1ms.
    }
}

/**
 * @todo In case you like it, please implement your own tasks
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,RxData);
    int index = RxHeader.StdId - 0x201;
    switch (index){
        case 0: case 1: case 2: case 3:
            wheels[index].updateInfoFromCAN(RxData);
            break;
        case 4: case 5:
            arms[index-4].updateInfoFromCAN(RxData);
            break;
    }
    HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CANTaskWheel(void *){
    CAN_TxHeaderTypeDef txHeaderWheel = {TX_ID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
    // CAN_FilterTypeDef FilterWheel = {0x201 << 5, 0x202 << 5,0x203 << 5,0x204 << 5,
    //                                     CAN_FILTER_FIFO0,0,CAN_FILTERMODE_IDMASK,CAN_FILTERSCALE_32BIT,
    //                                     CAN_FILTER_ENABLE,0};
    CAN_FilterTypeDef FilterWheel = {0, 0,0,0,
                                        CAN_FILTER_FIFO0,0,CAN_FILTERMODE_IDMASK,CAN_FILTERSCALE_32BIT,
                                        CAN_FILTER_ENABLE,0};

    
    wheels.init(&hcan,&FilterWheel);
    
    while (true)
    {
        
        if (uartSnapshot.s2 != 3){continue;}
        
        DJIMotor::UART_ConvertMotor(uartSnapshot,wheels);

        wheels.transmit(&hcan,&txHeaderWheel,&FilterWheel);
        vTaskDelay(1);
    }
    
    
}

void CANTaskArm(void *){
    CAN_TxHeaderTypeDef txHeaderArm = {EX_TX_ID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
    CAN_FilterTypeDef FilterArm = {0, 0,0,0,
                                        CAN_FILTER_FIFO0,0,CAN_FILTERMODE_IDMASK,CAN_FILTERSCALE_32BIT,
                                        CAN_FILTER_ENABLE,0};
    arms.init(&hcan,&FilterArm);

    while (true){
        
        if (uartSnapshot.s2 == 2){
            float multiple;

            switch (uartSnapshot.s1){
            case 2: // close
                __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,100);
                multiple = 0.2;
                break;
            case 3: // open 
                __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,250);
                multiple = 1;
            default:
                break;
            }


            DJIMotor::UART_ConvertArm(uartSnapshot,arms,multiple);
        }
        arms.transmit(&hcan,&txHeaderArm,&FilterArm);
        vTaskDelay(1);
    }
}
/**
 * @brief Intialize all the drivers and add task to the scheduler
 * @todo  Add your own task in this file
*/
void startUserTasks()
{
    DR16::init();
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    
    xTaskCreateStatic(DR16Communication,
                      "DR16_Communication ",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      1,
                      DR16TaskStack,
                      &DR16TaskTCB);  // Add the main task into the scheduler
    xTaskCreateStatic(CANTaskWheel,
                      "CANTaskWheel ",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      1,
                      CANWheelTaskStack,
                      &CANWheelTaskTCB); 
    xTaskCreateStatic(CANTaskArm,
                      "CANTaskArm ",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      1,
                      CANArmTaskStack,
                      &CANArmTaskTCB); 
    /**
     * @todo Add your own task here
    */
}