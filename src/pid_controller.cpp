#include <potential_function/pid_controller.hpp>



PIDController::PIDController(double Kp, double Ki, double Kd): 
Kp_(Kp), Ki_(Ki), Kd_(Kd)
{

}



PIDController::~PIDController()
{

}


double PIDController::getKp(void) const 
{
    return Kp_;
}



double PIDController::getKi(void) const
{
    return Ki_;
}



double PIDController::getKd(void) const
{
    return Kd_;
}



void PIDController::setKp(double Kp)
{
    Kp_ = Kp;
}



void PIDController::setKi(double Ki)
{
    Ki_ = Ki;
}



void PIDController::setKd(double Kd)
{
    Kd_ = Kd;
}



double PIDController::getPIDOutputSignal(double error, double dt, double threshold)
{
    P_ = Kp_ * error;
    I_ = I_ + Ki_ * error * dt;
    D_ = Kd_ * (error - previous_error_) / dt;

    previous_error_ = error;

    if(abs(error) <= threshold)
    {
        return 0.0;
    }

    return P_ + I_ + D_;
}
