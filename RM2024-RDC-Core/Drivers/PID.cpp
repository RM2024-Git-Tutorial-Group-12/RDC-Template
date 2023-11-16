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

    pOut = Kp * error;  // pOut = Kp * error

    iOut += Ki * error * dt;  // iOut = integral(Ki * error)

    dOut = Kd * (error - lastError) / dt;  // dOut = derivative(Kd * error)

    output = pOut + iOut + dOut;  // output = pOut + iOut + dOut

    // if (output < -16384){output = -16384;}
    // else if (output > 16384){output = 16384;}
    lastError = error;

    /*=====================================================================*/
    // Your implementation of the PID algorithm ends here
    /*=====================================================================*/
    return this->output;  // You need to give your user the output for every update
}

}  // namespace Control
#endif
