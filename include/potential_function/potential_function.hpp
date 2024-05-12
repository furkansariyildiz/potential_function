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
         * @brief
        */
        PotentialFunction();



        /**
         * @brief
        */
        ~PotentialFunction();



        /**
         * @brief
        */
        rclcpp::Subscription<potential_function::msg::TargetPoseList>::SharedPtr _target_pose_list_subscriber;



        /**
         * @brief
        */
        void targetPoseListCallback(const potential_function::msg::TargetPoseList::SharedPtr message);



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
         * @brief Number of obstacles to create vector object with index.
        */
        int _number_of_obstacles;


        /**
         * @brief Alpha
        */
        double _alpha;



        /**
         * @brief Beta
        */
        double _beta;



        /**
         * @brief Limit for distance between current robot and other robots.
        */
        double _limit_distance;



        /**
         * @brief Limit for distance between current robot and obstacles.
        */
        double _limit_distance_for_obstacles;


        /**
         * @brief Robot positions
        */
        vector<vector<double>> _b_i;



        /**
         * @brief Target positions
        */
        vector<vector<double>> _b_g;



        /**
         * @brief Restricted area position
        */
        vector<vector<double>> _b_rs;



        vector<vector<double>> _b_obstacles;
};

#endif
