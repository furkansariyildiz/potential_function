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
