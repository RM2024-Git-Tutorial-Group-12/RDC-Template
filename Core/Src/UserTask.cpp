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
StackType_t testTaskStack[configMINIMAL_STACK_SIZE];
/*Declare the PCB for our PID task*/
StaticTask_t DR16TaskTCB;
StaticTask_t testTaskTCB;

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

void test(void *){
    int x = 0;
    while (true)
    {
        x+=1;
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
    xTaskCreateStatic(DR16Communication,
                      "DR16_Communication ",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      1,
                      DR16TaskStack,
                      &DR16TaskTCB);  // Add the main task into the scheduler
    xTaskCreateStatic(test,
                      "test ",
                      configMINIMAL_STACK_SIZE,
                      NULL,
                      2,
                      testTaskStack,
                      &testTaskTCB); 
    /**
     * @todo Add your own task here
    */
}
