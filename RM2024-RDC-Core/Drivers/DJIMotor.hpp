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
#include "PID.hpp"
#include "DR16.hpp"
#include "PID.hpp"

#define UP 1
#define DOWN -1
#define REST 0
#define AXISSPEED1 3000
#define AXISSPEED2 1500
#define SPEEDLIMIT 3960

namespace DJIMotor
{

    #define TX_ID 0x200
    #define EX_TX_ID 0x1FF
    typedef unsigned int ticks;
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

    enum class TYPE{
        WHEEL,
        ARM
    };

    class DJIMotor
    {
        private:
            uint16_t canID;  // You need to assign motor's can ID for different motor
            
            // Below are the types of measurements 

            int convertedUART;
            int realAngle;

            // Below are types of 

            int16_t mechanicalAngle;
            int16_t rotationalSpeed;
            int16_t current;

            Control::PID motorPID{1,0,0}; //uncomment the code once the PID has a proper constructor and then update DJIMotor accordingly
            // ticks lastUpdated;
            TYPE motorType;

        public:
            DJIMotor(const int& ID);

            void updateInfoFromCAN(const uint8_t* rxBuffer);
            void updateTargetRPM(const int);
            void setPID(const float*);

            // void getValues(int* container);
            int getPIDRPM();
            int getPIDSpeed();
            void setRealAngle(const int&);
            void RealAngleInit(){realAngle=0;}
            int getrotationalSpeed(){return rotationalSpeed;}
            int getconvertedUART(){return convertedUART;}
            int getrealAngle(){return realAngle;}
            // int getCANID();

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
            MotorPair(const int IDStart,const int s, const float pid[][3]);

            void transmit(CAN_HandleTypeDef*,CAN_TxHeaderTypeDef*,CAN_FilterTypeDef*);

            void errorHandler(CAN_HandleTypeDef*,CAN_TxHeaderTypeDef*,CAN_FilterTypeDef*);

            DJIMotor& operator[](const int);
            void init(CAN_HandleTypeDef* hcan,CAN_FilterTypeDef* filter);
            
            void updateTargetRPM(const int*);
            int getsize(){return size;}
    };

    class motorMechanics{
        public:
    
        int motor1;
        int motor2;
        int motor3;
        int motor4;

        // motorMechanics(const int*);
        motorMechanics(const int, const int, const int, const int);
        void operator=(const motorMechanics&);
        // void operator=(const int*);

        motorMechanics operator+(const motorMechanics& matrix);
        motorMechanics operator*(const float multiple);

        void matrixRotateLeft(); 
        void matrixRotateRight();

        void normalise();
        void cpyMotorVals(int*);
        void reduceCornerRotate();

    };

    int max(const int a, const int b);

    int abs(const int& a);
    int absmax(int,int,int,int);
/**
 * @brief The whole motor's module initialization function
 * @note  You might initialize the CAN Module here
 * @retval
 */

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
extern void UART_ConvertMotor(const DR16::RcData&,MotorPair&);
extern void UART_ConvertArm(const DR16::RcData&,MotorPair&,const float&);
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
double sqrt(double);

/*===========================================================*/
}  // namespace DJIMotor
#endif  // USE_DJI_MOTOR