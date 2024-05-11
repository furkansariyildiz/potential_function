#ifndef POTENTIAL_FUNCTION_HPP
#define POTENTIAL_FUNCTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <iostream>
// #include <mpfr.hpp>

using namespace std;



class PotentialFunction
{
    private:

    public:
        /**
         * @brief
        */
        PotentialFunction();



        /**
         * @brief
        */
        ~PotentialFunction();



        void targetPoseListCallback();



        /**
         * @brief Finding robots in RS meter range
        */
        void findRobotsInRange(void);



        /**
         * @brief Alpha and Beta
        */
        void calculateAAndB(void);


        
        /**
         * @brief dA/dX
        */
        void calculateDerivativeOfAWithRespectToX(void);



        /**
         * @brief dA/dY
        */
        void calculateDerivativeOfAWithRespectToY(void);



        /**
         * @brief dB/dX
        */
        void calculateDerivativeOfBWithRespectToX(void);
        

        /**
         * @brief dB/dY
        */
        void calculateDerivativeOfBWithRespectToY(void);



        /**
         * @brief dF/dX
        */
        void calculateDerivativeOfFWithRespectToX(void);

        

        /**
         * @brief dF/dY
        */
        void calculateDerivativeOfFWithRespectToY(void);



        /**
         * @brief 
        */
        void robotController(void);



        /**
         * @brief
        */
        long double _dQ_division_dA;



        /**
         * @brief Current robot id
        */
        int _robot_id;


        /**
         * @brief
        */
        int _number_of_robots;


        /**
         * @brief Limit for distance between current robot and other robots or obstacles.
        */
        double _limit_distance;



        /**
         * @brief 
        */
        vector<vector<double>> _b_i;
};

#endif
