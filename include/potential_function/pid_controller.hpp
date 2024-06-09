#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP


class PIDController
{
    private:
        double Kp_;
        double Ki_;
        double Kd_;

    public:
        PIDController(double Kp, double Ki, double Kd);

        ~PIDController();

        double getKp() const;
        double getKi() const;
        double getKd() const;

        void setKp(double Kp);
        void setKi(double Ki);
        void setKd(double Kd);
};


#endif