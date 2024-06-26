#include <potential_function/pid_controller.hpp>



PIDController::PIDController(double Kp, double Ki, double Kd): 
Kp_(Kp), Ki_(Ki), Kd_(Kd)
{
    P_ = 0.0;
    I_ = 0.0;
    D_ = 0.0;
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



void PIDController::setAntiWindup(double anti_windup)
{
    anti_windup_ = anti_windup;
}



double PIDController::getPIDOutputSignal(double error, double dt, double threshold)
{
    P_ = Kp_ * error;
    I_ = I_ + Ki_ * error * dt;
    D_ = Kd_ * (error - previous_error_) / dt;

    control_input_ = P_ + I_ + D_;

    std::cout << "Kp: " << Kp_ << " Ki: " << Ki_ << " Kd: " << Kd_ << std::endl;

    std::cout << "P : " << P_ << " I : " << I_ << " D: " << D_ << std::endl;

    std::cout << "Error: " << error << std::endl;

    std::cout << "Control input: " << control_input_ << std::endl;

    previous_error_ = error;

    if(abs(error) <= threshold)
    {
        return 0.0;
    }

    if(control_input_ > anti_windup_)
    {
        return anti_windup_;
    }
    else if(control_input_ < -1 * anti_windup_)
    {
        return -1 * anti_windup_;
    }

    return control_input_;
}
