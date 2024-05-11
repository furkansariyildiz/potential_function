#ifndef POTENTIAL_FUNCTION_HPP
#define POTENTIAL_FUNCTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <iostream>
#include <mpfr.hpp>

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
};

#endif
