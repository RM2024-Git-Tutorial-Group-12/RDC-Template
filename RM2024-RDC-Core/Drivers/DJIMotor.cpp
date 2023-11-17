#include "DJIMotor.hpp"


// DEF
#ifdef USE_DJI_MOTOR
#ifndef RDC_DJIMotor_MAX_NUM
#define RDC_DJIMotor_MAX_NUM 8
#define UP 1
#define DOWN -1
#define REST 0
#define AXISSPEED1 3000
#define AXISSPEED2 1500
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
        lastUpdated = 0;
        if (i >= 0 && i <= 3){
            motorType = TYPE::WHEEL;
        }
        else{
            motorType = TYPE::ARM;
        }
    }

    void DJIMotor::updateInfoFromCAN(const uint8_t rxMotorData[8]){

        mechanicalAngle = rxMotorData[0] <<8 | rxMotorData[1];
        rotationalSpeed = rxMotorData[2] <<8 | rxMotorData[3];
        current = rxMotorData[4] <<8 | rxMotorData[5];
        
    }

    void DJIMotor::updateTargetRPM(const int TC){
        convertedUART = TC;
    }

    /*
        The workflow of this functions
        If it is an arm and its uart value is 0
            - The use the position PID to feed it to the RPM PID
        else
            - Then just use RPM PID

    */
    int DJIMotor::getPIDRPM(){
        float newRPM;
        
        int newSpeed = convertedUART;
        ticks currentTime = HAL_GetTick();

        if (convertedUART == 0 && motorType == TYPE::ARM) {
            realAngle += rotationalSpeed*((currentTime-lastUpdated));
            newSpeed = getPIDSpeed(); 
        }
        else {
            realAngle = 0;
        }

        newRPM = motorPID.update(newSpeed,rotationalSpeed,(currentTime-lastUpdated));
        lastUpdated = currentTime;

        return newRPM;
    }
    


    int DJIMotor::getPIDSpeed(){
        float newSpeed;
        ticks currentTime = HAL_GetTick();
        newSpeed = motorPID.update(0, realAngle,(currentTime-lastUpdated));
        lastUpdated = currentTime;

        return newSpeed;
    }

    void DJIMotor::setPID(const float* pid){
        motorPID = Control::PID{pid[0],pid[1],pid[2]};
    }

    void DJIMotor::setRealAngle(const int& angleChange){
        realAngle += angleChange;
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
            motor[i].setPID(pid[i]);
        }
           
    }

    void MotorPair::transmit(CAN_HandleTypeDef* hcan,CAN_TxHeaderTypeDef* header,CAN_FilterTypeDef* filter){
        uint8_t txMessage[8] = {0};
        uint32_t mailBox = 0;
    
        for (int index = 0; index < size; index++){
            short PIDRPM = motor[index].getPIDRPM();
            txMessage[index*2] = PIDRPM >> 8;
            txMessage[index*2+1] = PIDRPM;
        }

        if (HAL_CAN_AddTxMessage(hcan, header, txMessage, &mailBox) != HAL_OK){
            errorHandler(hcan,header,filter);
        }
    }

    // void MotorPair::transmit_arm(CAN_HandleTypeDef* hcan,CAN_TxHeaderTypeDef* header,CAN_FilterTypeDef* filter){
    //     uint8_t txMessage[8] = {0};
    //     uint32_t mailBox = 0;

    //     for (int index = 0; index < size; index++){
    //         short PIDRPM = motor[index].getPIDRPM();
    //         txMessage[index*2] = PIDRPM >> 8;
    //         txMessage[index*2+1] = PIDRPM;
    //     }

    //     if (HAL_CAN_AddTxMessage(hcan, header, txMessage, &mailBox) != HAL_OK){
    //         errorHandler(hcan,header,filter);
    //     }
    // }

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

    void MotorPair::updateTargetRPM(const int NewCurrents[4]){

        for (int index = 0; index < size; index++){
            motor[index].updateTargetRPM(NewCurrents[index]);
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

    motorMechanics motorMechanics::operator*(const float multiplier){
        return motorMechanics(
            multiplier*motor1,
            multiplier*motor2,
            multiplier*motor3,
            multiplier*motor4
        );
    }

    void motorMechanics::matrixRotateLeft(){
        *this = motorMechanics(motor2,motor4,motor1,motor3);
    }

    void motorMechanics::matrixRotateRight(){
        *this = motorMechanics(motor3,motor1,motor4,motor2);
    }

    void motorMechanics::normalise(const int upperBound){
        int multiply = upperBound;
        if (upperBound > 12000){multiply = 12000;}
        float total = sqrt(
            motor1*motor1+
            motor2*motor2+
            motor3*motor3+
            motor4*motor4
        );
        motor1 = (motor1*multiply)/total;
        motor2 = (motor2*multiply)/total;
        motor3 = (motor3*multiply)/total;
        motor4 = (motor4*multiply)/total;
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
    // a contrained limit of +-5280 has been set, which can be changed later

    const int convX = ((x-364)*8-5280); 
    const int convY = ((y-364)*8-5280);
    float multiple = (convX && convY)?2:1;
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
    ));
    int motorCurrents[4] = {0};
    linearMov.cpyMotorVals(motorCurrents);
    pair.updateTargetRPM(motorCurrents);
}

void UART_ConvertArm(const DR16::RcData& RcData,MotorPair& pair){
    const int axis1 = RcData.channel1;
    const int axis2 = RcData.channel3;

    int status_axis1;
    int status_axis2;

    if (axis1 > 1024) status_axis1 = UP;
    else if (axis1 < 1024) status_axis1 = DOWN;
    else status_axis1 = REST;

    if (axis2 > 1024) status_axis2 = UP;
    else if (axis2 < 1024) status_axis2 = DOWN; 
    else status_axis2 = REST;
    
    int motorCurrents[2] = {0};
    motorCurrents[0] = status_axis1 * AXISSPEED1;
    motorCurrents[1] = status_axis2 * AXISSPEED2;
    pair.updateTargetRPM(motorCurrents);

}

int max(const int a, const int b){
    return (a>b)?a:b;
}


//  approx sqrt
double sqrt(double square)
{
    double root=square/3;
    int i;
    if (square <= 0) return 0;
    for (i=0; i<32; i++)
        root = (root + square / root) / 2;
    return root;
}

/* end of the call functions and other supporting functions*/

}  // namespace DJIMotor
#endif