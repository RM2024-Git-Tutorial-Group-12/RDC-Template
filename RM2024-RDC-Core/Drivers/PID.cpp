#include "PID.hpp"
#if USE_PID
namespace Control
{

/*This is where you implement the PID algorithm*/
float PID::update(float target, float measurement, float dt)
{
    /*=====================================================================*/
    // Your implementation of the PID algorithm begins here
    /*=====================================================================*/
    
    error = target - measurement;  // error = target - current

    if (error == 0){ 
        return this->output;
    }
    
    pOut = Kp * error;  // pOut = Kp * error

    iOut += Ki * error * dt;  // iOut = integral(Ki * error)

    if (iOut > 1100){
        iOut = 1100;
    }
    else if (iOut < -1100){
        iOut = -1100;
    }

    dOut = Kd * (error - lastError) / dt;  // dOut = derivative(Kd * error)

    output = pOut + iOut + dOut;  // output = pOut + iOut + dOut

    lastError = error;

    /*=====================================================================*/
    // Your implementation of the PID algorithm ends here
    /*=====================================================================*/
    return this->output;  // You need to give your user the output for every update
}

}  // namespace Control
#endif
