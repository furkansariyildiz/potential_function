#ifndef POTENTIAL_FUNCTION_HPP
#define POTENTIAL_FUNCTION_HPP

#define PI 3.141592653589793238462

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <iostream>
#include <regex>

#include <potential_function/msg/target_pose_list.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "subscriber_info.hpp"
#include <potential_function/pid_controller.hpp>
// #include <mpfr.hpp>

using namespace std;



class PotentialFunction: public rclcpp::Node
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
         * @biref Publisher for velocity command
        */
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_publisher;



        /**
         * @brief Timer for main loop (robot controller)
        */
        rclcpp::TimerBase::SharedPtr _robot_controller_timer;



        /**
         * @brief Callback function for pose list
         * @param 
        */
        void targetPoseListCallback(const potential_function::msg::TargetPoseList::SharedPtr message);



        /**
         * @brief
         * @param
         * @param
        */
        void dynamicOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr message, unsigned int topic_index, int robot_id);



        /**
         * @brief
         * @param
         * @return
        */
        vector<string> getRobotID(const string &topic_name);



        /**
         * @brief Finding robots in RS meter range
        */
        void findRobotsInRange(void);



        /**
         * @brief Calculating Alpha and Beta values
        */
        void calculateAlphaAndBeta(void);


        
        /**
         * @brief dA/dX
        */
        void calculateDerivativeOfAlphaWithRespectToX(void);



        /**
         * @brief dA/dY
        */
        void calculateDerivativeOfAlphaWithRespectToY(void);



        /**
         * @brief dB/dX
        */
        void calculateDerivativeOfBetaWithRespectToX(void);
        

        /**
         * @brief dB/dY
        */
        void calculateDerivativeOfBetaWithRespectToY(void);



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
         * @brief Dynamic subscribers info. 
        */
        vector<SubscriberInfo> _dynamic_subscribers;



        /**
         * @brief
         */
        PIDController _linear_velocity_controller;



        /**
         * @brief
         */
        PIDController _angular_velocity_controller;



        /**
         * @brief Mapping topics via string to value unsigned int.
        */
        map<string, unsigned int> _topic_to_index;



        /**
         * @brief Velocity Command message for current robot.
        */
        geometry_msgs::msg::Twist _cmd_vel_message;



        /**
         * @brief dQ/dA
        */
        long double _dQ_division_dA;



        /**
         * @brief Velocity vector to get desired heading.
        */
        double _desired_heading;



        /**
         * @brief Current robot id.
        */
        int _robot_id;



        /**
         * @brief Number of robots in environment.
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
        double _K_gain;



        /**
         * @brief Gain parameters for linear velocity.
         */
        double _Kp_v, _Ki_v, _Kd_v;



        /**
         * @brief Gain parameters for angular velocity.
        */
        double _Kp_w, _Ki_w, _Kd_w;



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
         * @brief Radius of each robots.
        */
        double _radius_of_robots;



        /**
         * @brief Robot positions.
        */
        vector<vector<double>> _b_;



        /**
         * @brief Robot RPY (roll, pitch, yaw).
        */
        vector<vector<double>> _b_rpy;



        /**
         * @brief Target positions.
        */
        vector<vector<double>> _b_g;



        /**
         * @brief Obstacles positions.
        */
        vector<vector<double>> _b_obstacles;



        /**
         * @brief Name of the robots dynamic subscriber.
        */
        vector<string> _name_of_robots;



        /**
         * @brief Name of the odom topics for dynamic subscriber.
        */
        string _odom_topic_name;
};

#endif
