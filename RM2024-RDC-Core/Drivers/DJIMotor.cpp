#include "DJIMotor.hpp"


// DEF
#ifdef USE_DJI_MOTOR
#ifndef RDC_DJIMotor_MAX_NUM
#define RDC_DJIMotor_MAX_NUM 8
#endif

namespace DJIMotor
{

// Initialize motor's controller instance
DJIMotor* motors[6] = {&DJIMotor(1),&DJIMotor(2),&DJIMotor(3),&DJIMotor(4),&DJIMotor(5),&DJIMotor(0x206)};



/*========================================================*/
// Your implementation of the function, or even your customized function, should
// be implemented here
/*========================================================*/
/**
 * @todo
 */
DJIMotor::DJIMotor(const int& i){
    canID=0x200+i;
    rotation = 0;
    speed = 0;
    current = 0;
    if (i < 5){type = MotorResponsibility::WHEEL_MOTOR;}
    else{type = MotorResponsibility::ARM_MOTOR;}
}
/**
 * @todo
 */
float DJIMotor::getEncoder(const uint16_t& canID){
    return 0.0f;
}

void DJIMotor::updateInfo(){
    rotation=rxMotorData[0] <<8 | rxMotorData[1];
    speed=rxMotorData[2] <<8 | rxMotorData[3];
    current=rxMotorData[4] <<8 | rxMotorData[5];
    temp=rxMotorData[6];
}

/**
 * @todo
 */
float getRPM(const uint16_t& canID) { return 0.0f; }

/**
 * @todo
 */
void setOutput(int16_t output) {}

/**
 * @todo
 */
void DJIMotor::transmit(uint16_t header,int16_t PIDOutput,CAN_TxHeaderTypeDef* header){
    int32_t mailBox;
    txMotorData[0] = PIDOutput >> 8;
    txMotorData[1] = PIDOutput;
    if (!HAL_CAN_AddTxMessage(&hcan,header,txMotorData,&mailBox)){
        ErrorHandler();
    }

}

void CallBackForCAN(CAN_HandleTypeDef *hcan){
    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rxHeader,rxMotorData);
    motors[rxHeader.StdId]->updateInfo();
    HAL_CAN_ActivateNotification(hcan,CallBackForCAN);
}

void ErrorHandler(){
    
}

}  // namespace DJIMotor
#endif