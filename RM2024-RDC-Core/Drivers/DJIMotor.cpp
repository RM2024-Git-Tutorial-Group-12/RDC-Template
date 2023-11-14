#include "DJIMotor.hpp"


// DEF
#ifdef USE_DJI_MOTOR
#ifndef RDC_DJIMotor_MAX_NUM
#define RDC_DJIMotor_MAX_NUM 8
#endif

namespace DJIMotor
{
    
    MotorPair arms = MotorPair(5,2);
/* The Declarations of DJIMotor Class*/

    DJIMotor::DJIMotor(const int& i){
        canID=0x200+i;
        convertedUART = 0;
        mechanicalAngle = 0;
        rotationalSpeed = 0;
        current = 0;
        motorTemperature = 25;
        lastUpdated = 0;
    }

    void DJIMotor::updateInfoFromCAN(const uint8_t rxMotorData[8]){
        mechanicalAngle = rxMotorData[0] <<8 | rxMotorData[1];
        rotationalSpeed = rxMotorData[2] <<8 | rxMotorData[3];
        current =         rxMotorData[4] <<8 | rxMotorData[5];
        motorTemperature= rxMotorData[6];
    }

    void DJIMotor::updateTargetCurrent(const int TC){
        this->convertedUART = TC;
    }

    void DJIMotor::getValues(int container[5]){
        container[0] = mechanicalAngle;
        container[1] = rotationalSpeed;
        container[2] = current;
        container[3] = motorTemperature;
        container[4] = convertedUART;
    }

    int DJIMotor::getPIDCurrent(){
        float newCurrent;
        ticks currentTime = HAL_GetTick();
        if (current > 16384){
            current = 16384;
        }
        else if (current < -16384){
            current = -16384;
        }
        newCurrent = motorPID.update(convertedUART,current,currentTime-lastUpdated) + current;

        if (newCurrent > 16384){
            newCurrent = 16384;
        }
        else if (newCurrent < -16384){
            newCurrent = -16384;
        }
        lastUpdated = currentTime;

        return newCurrent;
    }

    int DJIMotor::getCANID(){
        return canID;
    }
    void DJIMotor::setPID(const float* pid){
        motorPID = Control::PID{pid[0],pid[1],pid[2]};
    }
/* end of the declaration of DJIMotor class*/

/* Start of the declaration of MotorPair class */

    MotorPair::MotorPair(const int IDStart, const int numberOfMotors):motor(
        {DJIMotor(IDStart),DJIMotor(IDStart+1),DJIMotor(IDStart+2),DJIMotor(IDStart+3)}
    ){
        size = numberOfMotors;
    }

    MotorPair::MotorPair(const int IDStart, const int numberOfMotors, const float pid[][3]):motor(
        {DJIMotor(IDStart),DJIMotor(IDStart+1),DJIMotor(IDStart+2),DJIMotor(IDStart+3)}
    ){
        size=numberOfMotors;
        for (int i=0;i<size;i++){
            motor[i].DJIMotor::setPID(pid[i]);
        }
           
    }

    void MotorPair::transmit(CAN_HandleTypeDef* hcan,CAN_TxHeaderTypeDef* header,CAN_FilterTypeDef* filter){
        uint8_t txMessage[8] = {0};
        uint32_t mailBox = 0;
        int offset = 0;

        for (int index = 0; index < size; index++){
            uint16_t PIDCurrent = motor[index].getPIDCurrent();
            txMessage[index+offset] = PIDCurrent >> 8;
            txMessage[index+offset+1] = PIDCurrent;
            offset++;
        }

        if (HAL_CAN_AddTxMessage(hcan, header, txMessage, &mailBox) != HAL_OK){
            errorHandler(hcan,header,filter);
        }
    }

    void MotorPair::errorHandler(CAN_HandleTypeDef* hcan,CAN_TxHeaderTypeDef* header,CAN_FilterTypeDef* filter){
        uint8_t txMessage[8] = {0};
        uint32_t mailBox = 0;
        HAL_CAN_AddTxMessage(hcan, header, txMessage, &mailBox);
    }

    DJIMotor& MotorPair::operator[](const int index){
        return motor[index];
    }

    void MotorPair::init(CAN_HandleTypeDef* hcan,CAN_FilterTypeDef* filter){
        HAL_CAN_ConfigFilter(hcan, filter);
    }

    void MotorPair::updateCurrents(const int NewCurrents[4]){
        for (int index = 0; index < size; index++){
            motor[index].updateTargetCurrent(NewCurrents[index]);
        }
    }

/* end of the declaration of the MotorPair class*/

/* Start of the declaration of motorMechanics */

    motorMechanics::motorMechanics(const int a, const int b, const int c, const int d){
        motor1 = a;
        motor2 = b;
        motor3 = c;
        motor4 = d;
    }

    // motorMechanics::motorMechanics(const int motorVals[4]){
    //     motor1 = motorVals[0];
    //     motor2 = motorVals[1];
    //     motor3 = motorVals[2];
    //     motor4 = motorVals[3];
    // }

    void motorMechanics::operator=(const motorMechanics& motorMatrix){
        motor1 = motorMatrix.motor1;
        motor2 = motorMatrix.motor2;
        motor3 = motorMatrix.motor3;
        motor4 = motorMatrix.motor4;
    }

    motorMechanics motorMechanics::operator+(const motorMechanics& otherMatrix){
        return motorMechanics(
            motor1+otherMatrix.motor1,
            motor2+otherMatrix.motor2,
            motor3+otherMatrix.motor3,
            motor4+otherMatrix.motor4
        );
    }

    // motorMechanics motorMechanics::operator*(const int& multiplier){
    //     return motorMechanics(
    //         motor1*multiplier,
    //         motor2*multiplier,
    //         motor3*multiplier,
    //         motor4*multiplier
    //     );
    // }

    void motorMechanics::matrixRotateLeft(){
        *this = motorMechanics(motor2,motor4,motor1,motor3);
    }

    void motorMechanics::matrixRotateRight(){
        *this = motorMechanics(motor3,motor1,motor4,motor2);
    }

//  if the value is between -12k to 12k, then it is left as it is, otherwise, it is scaled according to others.
//  if all motors are less than |12k|, then do nothing,
//  otherwise normalise it and then multiply by 12k
    void motorMechanics::normalise(const int upperBound){
        float max = 0;
        float total = sqrt(
            motor1*motor1+
            motor2*motor2+
            motor3*motor3+
            motor4*motor4
        );
        motor1 = (motor1*upperBound)/total;
        motor2 = (motor2*upperBound)/total;
        motor3 = (motor3*upperBound)/total;
        motor4 = (motor4*upperBound)/total;
        int temp[4] = {motor1,motor2,motor3,motor4};
        for (int i=0;i<4;i++){
            if (abs(temp[i])>12000 & abs(temp[i])>max) max = abs(temp[i]);
        }
        if (max>0) {
            motor1 = motor1 * 12000 / max;
            motor2 = motor2 * 12000 / max;
            motor3 = motor3 * 12000 / max;
            motor4 = motor4 * 12000 / max;
        } 
    }

    void motorMechanics::cpyMotorVals(int container[4]){
        container[0] = motor1; 
        container[1] = motor2; 
        container[2] = motor3; 
        container[3] = motor4;
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
void UART_ConvertMotor(const DR16::RcData& RCdata,MotorPair& pair){

    const int x = RCdata.channel1;
    const int y = RCdata.channel0;
    const int w = RCdata.channel2;
    if (!x || !y || !w){return;}

    // a contrained limit of 7920 has been set, which can be changed later
    const int convX = ((x-364)*8-5280); 
    const int convY = ((y-364)*8-5280);
    int multiple = (convX && convY)?2:1;
    const int convW = ((w-364)*8-5280) * multiple;


    motorMechanics xMov({convX,-convX,convX,-convX});
    motorMechanics yMov({convY,convY,-convY,-convY});
    motorMechanics wMov({convW,convW,convW,convW});

    motorMechanics linearMov = xMov + yMov;
    if (convW >= 0){
        linearMov.matrixRotateLeft();
    }
    if (convW <= 0){
        linearMov.matrixRotateRight();
    }
    linearMov = linearMov + wMov;
    linearMov.normalise((
        max(convX,-convX)+
        max(convY,-convY)+
        max(convW,-convW)
    )*1.3);
    int motorCurrents[4] = {0};
    linearMov.cpyMotorVals(motorCurrents);
    pair.updateCurrents(motorCurrents);
}

int max(const int a, const int b){
    return (a>b)?a:b;
}

double sqrt(double square)
{
    double root=square/3;
    int i;
    if (square <= 0) return 0;
    for (i=0; i<32; i++)
        root = (root + square / root) / 2;
    return root;
}

double abs(double x){
    x=(x>0)? x:-x;
    return x;
}

/* end of the call functions and other supporting functions*/

}  // namespace DJIMotor
#endif