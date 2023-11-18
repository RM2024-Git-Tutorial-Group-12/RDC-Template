#include "DJIMotor.hpp"


// DEF
#ifdef USE_DJI_MOTOR
#ifndef RDC_DJIMotor_MAX_NUM
#define RDC_DJIMotor_MAX_NUM 8

#endif

namespace DJIMotor
{
    Control::PID axis1SpeedPID{2,0.7,0};
    Control::PID axis2SpeedPID{2,0.7,0};
/* The Declarations of DJIMotor Class*/

    DJIMotor::DJIMotor(const int& i){
        canID=0x200+i;
        convertedUART = 0;
        mechanicalAngle = 0;
        rotationalSpeed = 0;
        current = 0;
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
        int newRPM;
        
        // ticks currentTime = HAL_GetTick();

        newRPM = motorPID.update(convertedUART,rotationalSpeed,1);

        if (newRPM == 0 && motorType == TYPE::ARM) {
            realAngle += rotationalSpeed;
            newRPM = getPIDSpeed(); 
        }
        else {
            realAngle = 0;
        }

        return newRPM;
    }

    int DJIMotor::getPIDSpeed(){

        int newSpeed;
        switch (canID)
        {
        case 0x205:
            newSpeed = axis1SpeedPID.update(0,realAngle,1);
            break;
        case 0x206:
            newSpeed = axis2SpeedPID.update(0,realAngle,1);
        default:
            break;
        }

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

    void motorMechanics::normalise(){
        if (abs(motor1) <= SPEEDLIMIT && abs(motor2) <= SPEEDLIMIT && abs(motor3) <= SPEEDLIMIT && abs(motor4) <= SPEEDLIMIT){return;}

        int total = absmax(motor1,motor2,motor3,motor4);

        motor1 = (motor1*SPEEDLIMIT)/total;
        motor2 = (motor2*SPEEDLIMIT)/total;
        motor3 = (motor3*SPEEDLIMIT)/total;
        motor4 = (motor4*SPEEDLIMIT)/total;
    }

    void motorMechanics::cpyMotorVals(int container[4]){
        container[0] = motor1; 
        container[1] = motor2; 
        container[2] = motor3; 
        container[3] = motor4;
    }

    void motorMechanics::reduceCornerRotate(){
        const float multiple = 0.5;
        if (motor1 && motor2 && motor3 && motor4){return;}

        if (motor1 == 0){
            motor4 *= multiple;
        }
        if (motor2 == 0){
            motor3 *= multiple;
        }
        if (motor3 == 0){
            motor2 *= multiple;
        }
        if (motor4 == 0){
            motor1 *= multiple;
        }
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

    const int multiple = 2*(SPEEDLIMIT/(UART_MAX-UART_MIN));

    const int convX = ((x - UART_MIN)*multiple - SPEEDLIMIT); 
    const int convY = ((y - UART_MIN)*multiple - SPEEDLIMIT);
    const int convW = ((w - UART_MIN)*multiple - SPEEDLIMIT);

    motorMechanics xMov({convX,-convX,convX,-convX});
    motorMechanics yMov({convY,convY,-convY,-convY});
    motorMechanics wMov({convW,convW,convW,convW});

    motorMechanics linearMov = xMov + yMov + wMov;

    int motorCurrents[4] = {0};
    linearMov.normalise();
    linearMov.cpyMotorVals(motorCurrents);
    pair.updateTargetRPM(motorCurrents);
}

void UART_ConvertArm(const DR16::RcData& RcData,MotorPair& pair, const float& multiple){
    const int axis1 = RcData.channel1;
    const int axis2 = RcData.channel3;

    int status_axis1;
    int status_axis2;

    if (axis1 > 1189) status_axis1 = DOWN;
    else if (axis1 < 859) status_axis1 = UP;
    else status_axis1 = REST;

    if (axis2 > 1189) status_axis2 = UP;
    else if (axis2 < 859) status_axis2 = DOWN; 
    else status_axis2 = REST;
    
    int motorCurrents[2] = {0};

    if (status_axis1 == UP){
        motorCurrents[0] = AXISSPEED1 * multiple * status_axis1;
    }
    else{
        motorCurrents[0] = AXISSPEED1 / multiple * status_axis1;
    }

    if (status_axis2 == UP){
        motorCurrents[1] = AXISSPEED2 * multiple * status_axis2;
    }
    else{
        motorCurrents[1] = AXISSPEED2 / multiple * status_axis2;
    }

    motorCurrents[0] = status_axis1 * AXISSPEED1;
    motorCurrents[1] = status_axis2 * AXISSPEED2;

    pair.updateTargetRPM(motorCurrents);

}

int max(const int a, const int b){
    return (a>b)?a:b;
}

int absmax(int a, int b, int c, int d){
    if (abs(a) > abs(b) && abs(a) > abs(c) && abs(a) > abs(d)){
        return abs(a);
    }
    else if (abs(b) > abs(c) && abs(b) > abs(d)){
        return abs(b);
    }
    else if (abs(c) > abs(d)){
        return abs(c);
    }
    return abs(d);
}

int abs(const int& val){
    return (val > 0)?val:-val;
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