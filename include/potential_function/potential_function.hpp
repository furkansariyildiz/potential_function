#ifndef POTENTIAL_FUNCTION_HPP
#define POTENTIAL_FUNCTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <iostream>
#include <potential_function/msg/target_pose_list.hpp>
// #include <mpfr.hpp>

using namespace std;



class PotentialFunction : rclcpp::Node
{
    private:

    public:
        /**
         * @brief Constructor of PotentialFunction class
        */
        PotentialFunction();



        /**
         * @brief Destructor of PotentialFunction class
        */
        ~PotentialFunction();



        /**
         * @brief Subscriber for target pose list
        */
        rclcpp::Subscription<potential_function::msg::TargetPoseList>::SharedPtr _target_pose_list_subscriber;



        /**
         * @brief Callback function for pose list
         * @param 
         * @return void
        */
        void targetPoseListCallback(const potential_function::msg::TargetPoseList::SharedPtr message);



        /**
         * @brief Finding robots in RS meter range
        */
        void findRobotsInRange(void);



        /**
         * @brief Calculating Alpha and Beta values
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
         * @brief Robot controller function to manipulate cmd_vel topic.
        */
        void robotController(void);



        /**
         * @brief dQ/dA
        */
        long double _dQ_division_dA;



        /**
         * @brief Current robot id
        */
        int _robot_id;



        /**
         * @brief Number of robots in environment
        */
        int _number_of_robots;



        /**
         * @brief Number of obstacles to create vector object with index.
        */
        int _number_of_obstacles;


        /**
         * @brief Alpha (A)
        */
        double _alpha;



        /**
         * @brief Beta (B)
        */
        double _beta;



        /**
         * @brief Gama (Q)
        */
        double _gama;



        /**
         * @brief Fx
        */
        double _control_input_for_x;



        /**
         * @brief Fy
        */
        double _control_input_for_y;



        /**
         * @brief dA/dX
        */
        double _derivative_of_alpha_with_respect_to_x;



        /**
         * @brief dA/dY
        */
        double _derivative_of_alpha_with_respect_to_y;



        /**
         * @brief dB/dX
        */
        double _derivative_of_beta_with_respect_to_x;



        /**
         * @brief dB/dY
        */
        double _derivative_of_beta_with_respect_to_y;



        /**
         * @brief dQ/dX
        */
        double _derivative_of_gama_with_respect_to_x;



        /**
         * @brief dQ/dY
        */
        double _derivative_of_gama_with_respect_to_y;



        /**
         * @brief dQ/dA
        */
        double _derivative_of_gama_with_respect_to_alpha;



        /**
         * @brief dQ/dB
        */
        double _derivative_of_gama_with_respect_to_beta;



        /**
         * @brief dF/dX
        */
        double _derivative_of_f_with_respect_to_x;



        /**
         * @brief dF/dY
        */
        double _derivative_of_f_with_respect_to_y;


        /**
         * @brief Parameter for potential function.
        */
        int _K_gain;


        /**
         * @brief Limit for distance between current robot and other robots.
        */
        double _limit_distance;



        /**
         * @brief Limit for distance between current robot and obstacles.
        */
        double _limit_distance_for_obstacles;



        /**
         * @brief Radius of outer boundary;
        */
        double _radius_outer;


        /**
         * @brief Robot positions
        */
        vector<vector<double>> _b_;



        /**
         * @brief Target positions
        */
        vector<vector<double>> _b_g;



        /**
         * @brief Obstacles positions
        */
        vector<vector<double>> _b_obstacles;
};

#endif
