/**
 * @file DJIMotor.hpp
 * @author - GUO, Zilin
 *         - Your Name
 * @brief This is the DJIMotor template codes for RM2024-Tutorial PA3 and RDC
 * @note  You could directly transplant your code to the RDC after finishing PA3
 * @note  If you do not like the template I provide for you, you could remove
 * all of them and use your own
 * @copyright This file is only for HKUST Enterprize RM2024 internal
 * competition. All Rights Reserved.
 *
 */

#pragma once
#include "AppConfig.h"

#if USE_DJI_MOTOR

#ifndef DJI_MOTOR_CAN
#define DJI_MOTOR_CAN hcan
#endif

#include "main.h"
#include "stdint.h"
#include "can.h"
#include "PID.hpp"
#include "DR16.hpp"

namespace DJIMotor
{

    #define TX_ID 0x200
    #define EX_TX_ID 0x1FF

/**
 * @brief A motor's handle. We do not require you to master the cpp class
 * syntax.
 * @brief However, some neccessary OOP thought should be shown in your code.
 * @brief For example, if you have multiple motors, which is going to happen in
 * RDC (You have at least 4 wheels to control)
 * @brief You are able to write a "template" module for all the abstract motors,
 * and instantiate them with different parameters
 * @brief Instead of copy and paste your codes for four times
 * @brief This is what we really appreiciate in our programming
 */ 

    class DJIMotor
    {
        private:
            uint16_t canID;  // You need to assign motor's can ID for different motor
            int16_t targetCurrent;

            int16_t mechanicalAngle;
            int16_t rotationalSpeed;
            int16_t current;
            int16_t motorTemperature;

            // control::PID motorPID; //uncomment the code once the PID has a proper constructor and then update DJIMotor accordingly
        public:
            DJIMotor(const int& ID);
            void updateInfoFromCAN(const uint8_t* rxBuffer);
            void updateTargetCurrent(const int rx);
            void getValues(int16_t* container);
            int16_t getPIDCurrent();
            

        /*======================================================*/
        /**
         * @brief Your self-defined variables are defined here
         * @note  Please refer to the GM6020, M3508 motor's user manual that we have
         * released on the Google Drive
         * @example:
         
        * uint16_t encoder;
        * uint16_t rpm;
        * float orientation; //  get the accumulated orientation of the motor
        * ......
        */

        
        /*=======================================================*/
    };

    class MotorPair{
        private:
            DJIMotor motor[4];
            int size;
        public:
            MotorPair(const int IDStart, const int s);
            void transmit(CAN_HandleTypeDef*,CAN_TxHeaderTypeDef*,CAN_FilterTypeDef*);
            void errorHandler();
            DJIMotor& operator[](const int);
            void init(CAN_HandleTypeDef* hcan,CAN_TxHeaderTypeDef* header,CAN_FilterTypeDef* filter);
            void updateCurrents(const int*);
    };

    static MotorPair wheels = MotorPair(1,4);
    static MotorPair arms = MotorPair(5,2);

    struct motorMechanics{
        int motor1;
        int motor2;
        int motor3;
        int motor4;
        motorMechanics(const int, const int, const int, const int);
        motorMechanics(int*);
        void operator=(int*);
        motorMechanics& operator+(const motorMechanics& values);
        void normalise(const int max);
        void cpyMotorVals(int*);
        // the role of the following two functions is to help to have the idea of moving pivot and then rotating
        void matrixRotateLeft(); 
        void matrixRotateRight();
    };
/**
 * @brief The whole motor's module initialization function
 * @note  You might initialize the CAN Module here
 * @retval
 */

int max(const int a,const int b);

/**
 * @brief The encoder getter fucntion
 * @param canID The unique CAN id of your motor
 * @note  You need to return the current encoder feedback outward, because you
 * need it in the PID module
 * @retval motor's raw encoder
 */
float getEncoder(uint16_t canID);

/**
 * @brief The rpm getter function
 * @param canID The unique CAN id of your motor
 * @note You need to return the current rpm feedback outward, becacause you need
 * it in the PID module
 * @retval motor's rpm
 */
float getRPM(uint16_t canID);

/**
 * @brief Set the motor's output here
 * @note  You might need to refer to the user manual to "clamp" the maximum or
 * the minimun output
 * @param output, canID The motor's output, unique can Id
 * @note
 * - For GM6020, it's the motor's voltage
 * - For M3508, it's the motor's currnet
 * @retval
 */
void setOutput(int16_t output);

/**
 * @brief Transmit the current set motor's output to the groups of motor based
 * on the CAN header
 * @param header The header of groups of motor
 * @note For clear reference, please refer to the GM6020 and M3508 User manual
 * @param
 * @retval
 */
void transmit(uint16_t header);
void UART_ConvertMotor(const DR16::RcData&,int*);
/*===========================================================*/
/**
 * @brief You can define your customized function here
 * @note  It might not be necessary in your PA3, but it's might be beneficial
for your RDC development progress
 * @example
 * float get(uint16_t canID);
 *
 * @note You could try to normalize the encoder's value and work out the
accumulated position(orientation) of the motor
 * float getPosition(uint16_t canID);
 * ..... And more .....
 *
============================================================*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);


/*===========================================================*/
}  // namespace DJIMotor
#endif  // USE_DJI_MOTOR