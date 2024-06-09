#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <iostream>

using namespace std;

class PIDController
{
    private:
        /**
         * @brief
         */
        double Kp_;
        


        /**
         * @brief
         */
        double Ki_;


        
        /**
         * @brief
         */
        double Kd_;



        /**
         * @brief 
         */
        double P_, I_, D_;



        /**
         * @brief 
         */
        double previous_error_;

    public:
        /**
         * @brief
         */
        PIDController(double Kp, double Ki, double Kd);



        /**
         * @brief
         */
        ~PIDController();



        /**
         * @brief
         */
        double getKp(void) const;



        /**
         * @brief
         */
        double getKi(void) const;
        
        
        
        /**
         * @brief 
         */
        double getKd(void) const;

        
        
        /**
         * @brief
         */
        void setKp(double Kp);
        
        
        
        /**
         * @brief
         */
        void setKi(double Ki);
        
        
        
        /**
         * @brief
         */
        void setKd(double Kd);



        /**
         * @brief
         */
        double getPIDOutputSignal(double error, double dt, double threshold);
};


#endif