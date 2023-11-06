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

/*Allocate the stack for our PID task*/
StackType_t DR16TaskStack[configMINIMAL_STACK_SIZE];
// StackType_t CANWheelTaskStack[configMINIMAL_STACK_SIZE];
// StackType_t CANArmTaskStack[configMINIMAL_STACK_SIZE];
/*Declare the PCB for our PID task*/
StaticTask_t DR16TaskTCB;
// StaticTask_t CANWheelTaskTCB;
// StaticTask_t CANArmTaskTCB;

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

        vTaskDelay(1);  // Delay and block the task for 1ms.
    }
}

/**
 * @todo In case you like it, please implement your own tasks
 */

// void CANTaskWheel(void *){
//     CAN_TxHeaderTypeDef txHeaderWheel = {TX_ID, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
//     CAN_FilterTypeDef FilterWheel = {0x201,0x202,0x203,0x204,CAN_FILTER_FIFO0,0,CAN_FILTERMODE_IDMASK,CAN_FILTERSCALE_32BIT,CAN_FILTER_ENABLE};

//     // HAL_CAN_ConfigFilter(&hcan, &FilterWheel);
//     // if (HAL_CAN_ActivateNotification(&hcan, CallBackForCAN) != HAL_OK){
// 	//     DJIMotor::ErrorHandler();
//     // }
//     while (true)
//     {
        
//         /*The Wheel code*/
        
//         vTaskDelay(1);

//     }
    
    
// }

// void CANTaskArm(void*){
//     CAN_TxHeaderTypeDef txHeaderArm = {EX_TX_ID,  0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE};
//     CAN_FilterTypeDef FilterArm = {0x205,0x206,0,0,CAN_FILTER_FIFO0,0,CAN_FILTERMODE_IDMASK,CAN_FILTERSCALE_32BIT,CAN_FILTER_ENABLE};
//     HAL_CAN_ConfigFilter(&hcan, &FilterArm);

//     while (true)
//     {
//         /* code */

//         vTaskDelay(1);
//     }
    
// }

/**
 * @brief Intialize all the drivers and add task to the scheduler
 * @todo  Add your own task in this file
*/
void startUserTasks()
{
    DR16::init();
    HAL_CAN_Start(&hcan);

    xTaskCreateStatic(DR16Communication,
                      "DR16_Communication ",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      1,
                      DR16TaskStack,
                      &DR16TaskTCB);  // Add the main task into the scheduler
    
    // xTaskCreateStatic(CANTaskWheel,
    //                   "CANTaskWheel ",
    //                   configMINIMAL_STACK_SIZE,
    //                   NULL,
    //                   1,
    //                   CANWheelTaskStack,
    //                   &CANWheelTaskTCB); 
                    
    // xTaskCreateStatic(CANTaskArm,
    //                 "CANTaskArm ",
    //                 configMINIMAL_STACK_SIZE,
    //                 NULL,
    //                 1,
    //                 CANArmTaskStack,
    //                 &CANArmTaskTCB); 
    /**
     * @todo Add your own task here
    */
}
