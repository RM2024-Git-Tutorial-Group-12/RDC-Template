#include "DJIMotor.hpp"


// DEF
#ifdef USE_DJI_MOTOR
#ifndef RDC_DJIMotor_MAX_NUM
#define RDC_DJIMotor_MAX_NUM 8
#endif

namespace DJIMotor
{


/* The Declarations of DJIMotor Class*/

    DJIMotor::DJIMotor(const int& i){
        canID=0x200+i;
        targetCurrent = 0;
        mechanicalAngle = 0;
        rotationalSpeed = 0;
        current = 0;
        motorTemperature = 25;
    }

    void DJIMotor::updateInfoFromCAN(const uint8_t rxMotorData[8]){
        mechanicalAngle=rxMotorData[0] <<8 | rxMotorData[1];
        rotationalSpeed=rxMotorData[2] <<8 | rxMotorData[3];
        current=rxMotorData[4] <<8 | rxMotorData[5];
        motorTemperature=rxMotorData[6];
    }

    void DJIMotor::updateTargetCurrent(const int targetCurrent){this->targetCurrent = targetCurrent;}

    void DJIMotor::getValues(int16_t container[5]){
        container[0] = mechanicalAngle;
        container[1] = rotationalSpeed;
        container[2] = current;
        container[3] = motorTemperature;
        container[4] = targetCurrent;
    }

    int16_t DJIMotor::getPIDCurrent(){
        int16_t information[5];
        getValues(information);
        int16_t newCurrent;

        /*Call the PID function*/

        return newCurrent;
    }

/* end of the declaration of DJIMotor class*/

/* Start of the declaration of MotorPair class */

    MotorPair::MotorPair(const int IDStart, const int numberOfMotors):motor(
        {DJIMotor(IDStart),DJIMotor(IDStart+1),DJIMotor(IDStart+2),DJIMotor(IDStart+3)}
    ){
        size = numberOfMotors;
    }

    void MotorPair::transmit(CAN_HandleTypeDef* hcan,CAN_TxHeaderTypeDef* header,CAN_FilterTypeDef* filter){
        uint8_t txMessage[8] = {0};
        uint32_t mailBox = 0;
        int offset = 0;

        for (int index = 0; index < size; index++){
            int16_t PIDCurrent = motor[index].getPIDCurrent();
            txMessage[index+offset] = PIDCurrent >> 8;
            txMessage[index+offset+1] = PIDCurrent;
            offset++;
        }

        if (HAL_CAN_AddTxMessage(hcan, header, txMessage, &mailBox) != HAL_OK){
            errorHandler();
        }
    }

    void MotorPair::errorHandler(){
        /* 
            - either we can make the motor completely stop which could be more safe
            - or we could possibly possibly just pass the last 
            
            @todo we can consider to do this aftr implementing the PID class
        */
    }

    DJIMotor& MotorPair::operator[](const int index){
        return motor[index];
    }

    void MotorPair::init(CAN_HandleTypeDef* hcan,CAN_TxHeaderTypeDef* header,CAN_FilterTypeDef* filter){
        HAL_CAN_ConfigFilter(hcan, filter);
        if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        {
            errorHandler();
        }
    }

    void MotorPair::updateCurrents(const int NewCurrent[4]){
        for (int index = 0; index < size; index++){motor[index].updateTargetCurrent(NewCurrent[index]);}
    }

/* end of the declaration of the MotorPair class*/

/* Start of the declaration of motorMechanics */

    motorMechanics& motorMechanics::operator+(const motorMechanics& otherMatrix){
        motor1 += otherMatrix.motor1;
        motor2 += otherMatrix.motor2;
        motor3 += otherMatrix.motor3;
        motor4 += otherMatrix.motor4;
        return *this;
    }

    void motorMechanics::normalise(const int upperBound){
        int total = max(motor1,-motor1) + max(motor2,-motor2) + max(motor3,-motor3) + max(motor4,-motor4);
        motor1 = (motor1/total)*upperBound;
        motor2 = (motor2/total)*upperBound;
        motor3 = (motor3/total)*upperBound;
        motor4 = (motor4/total)*upperBound;
    }

    void motorMechanics::cpyMotorVals(int container[4]){
        container[0] = motor1; 
        container[1] = motor2; 
        container[2] = motor3; 
        container[3] = motor4;
    }

    motorMechanics::motorMechanics(const int a, const int b, const int c, const int d){
        motor1 = a;
        motor2 = b;
        motor3 = c;
        motor4 = d;
    }

    motorMechanics::motorMechanics(int motorVals[4]){
        motor1 = motorVals[0];
        motor2 = motorVals[1];
        motor3 = motorVals[2];
        motor4 = motorVals[3];
    }

    void motorMechanics::operator=(int motorVals[4]){
        motor1 = motorVals[0];
        motor2 = motorVals[1];
        motor3 = motorVals[2];
        motor4 = motorVals[3];
    }
    void motorMechanics::matrixRotateLeft(){
        int newOrientation[4] = {motor2,motor4,motor1,motor3};
        *this = newOrientation;
    }

    void motorMechanics::matrixRotateRight(){
        int newOrientation[4] = {motor3,motor1,motor4,motor2};
        *this = newOrientation;
    }
/* end of the declaration of the motorMechanics*/

/* Callback functions and other supporting functions */

/*
    Channel 0: determines the X axis and speed
    Channel 1: determines the Y axis and speed
    Channel 2: determines the direction of spin and speed of spin

    There are 2 components which are calculated independently
        - the cartesian motion
        - the angular motion
    
    The way Cartesian motion will be achieved is by linearly adding x and y coordinate

    for rotational motion the following things needs to be considered
        - the degree of the x press and y press
        - the degree of press of w and the direction
    
    after all of this, the strengths of the cartesian and rotation will be added 
*/
void UART_ConvertMotor(const DR16::RcData& rcdata,int motorCurrents[4]){

    int x = (int)rcdata.channel1;
    int y = (int)rcdata.channel0;
    int w = (int)rcdata.channel2;

    // a contrained limit of 7920 has been set, which can be changed later

    int convX = ((x-364)*12-7920); 
    int convY = ((y-364)*12-7920);
    int multiple = (convX && convY)?2:1;
    int convW = ((w-364)*12-7920) * multiple;

    motorCurrents[0] = -1*convX + convY + convW;
    motorCurrents[1] = -1*convX + -1*convY + convW;
    motorCurrents[2] = convX + convY + convW;
    motorCurrents[3] = convX + -1*convY + convW;
    

    wheels.updateCurrents(motorCurrents);
    
}

    int max(const int a, const int b){
        return (a>b)?a:b;
    }

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
    int index = RxHeader.StdId - 0x200;

    switch (index)
    {
    case 0: case 1: case 2: case 3:
        if (status == HAL_OK){
            wheels[index].updateInfoFromCAN(RxData);
        }
        else{
            wheels.errorHandler();
        }
        break;
    case 4: case 5:
        if (status == HAL_OK){
            arms[index].updateInfoFromCAN(RxData);
        }
        else{
            arms.errorHandler();
        }
    default:
        return;
    }

    HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* end of the call functions and other supporting functions*/

}  // namespace DJIMotor
#endif