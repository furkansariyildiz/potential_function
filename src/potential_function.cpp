#include <potential_function/potential_function.hpp>

using namespace std;



PotentialFunction::PotentialFunction():
Node("potential_function_node"),
_linear_velocity_controller(0.0, 0.0, 0.0),
_angular_velocity_controller(0.0, 0.0, 0.0)
{   
    _target_pose_list_subscriber = this->create_subscription<potential_function::msg::TargetPoseList>("/target_pose_list", 1000, bind(&PotentialFunction::targetPoseListCallback, this, placeholders::_1));

    _cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/tb_0_0/cmd_vel", 10);

    _robot_controller_timer = this->create_wall_timer(10ms, bind(&PotentialFunction::robotController, this));

    declare_parameter("current_robot_id", 0);
    declare_parameter("number_of_obstacles", 0);
    declare_parameter("radius_of_robots", 1.0);
    declare_parameter("radius_outer", 15.0);
    declare_parameter("limit_distance_for_robots", 1.0);
    declare_parameter("K_gain", 0.5);
    declare_parameter("Kp_v", 0.5);
    declare_parameter("Ki_v", 0.01);
    declare_parameter("Kd_v", 0.01);
    declare_parameter("Kp_w", 0.5);
    declare_parameter("Ki_w", 0.01);
    declare_parameter("Kd_w", 0.01);
    declare_parameter("name_of_robots", vector<string>{"tb1", "tb2"});
    declare_parameter("odom_topic", "odom");

    _robot_id = this->get_parameter("current_robot_id").as_int();
    _number_of_obstacles = this->get_parameter("number_of_obstacles").as_int();
    _radius_of_robots = this->get_parameter("radius_of_robots").as_double();
    _radius_outer = this->get_parameter("radius_outer").as_double();
    _limit_distance = this->get_parameter("limit_distance_for_robots").as_double();
    _K_gain = this->get_parameter("K_gain").as_double();
    _Kp_v = this->get_parameter("Kp_v").as_double();
    _Ki_v = this->get_parameter("Ki_v").as_double();
    _Kd_v = this->get_parameter("Kd_v").as_double();
    _Kp_w = this->get_parameter("Kp_w").as_double();
    _Ki_w = this->get_parameter("Ki_w").as_double();
    _Kd_w = this->get_parameter("Kd_w").as_double();
    _name_of_robots = this->get_parameter("name_of_robots").as_string_array();
    _odom_topic_name = this->get_parameter("odom_topic").as_string();
    
    _number_of_robots = _name_of_robots.size();

    _linear_velocity_controller.setKp(_Kp_v);
    _linear_velocity_controller.setKi(_Ki_v);
    _linear_velocity_controller.setKd(_Kd_v);

    _angular_velocity_controller.setKp(_Kp_w);
    _angular_velocity_controller.setKi(_Ki_w);
    _angular_velocity_controller.setKd(_Kd_w);

    _b_ = vector<vector<double>>(_number_of_robots, vector<double>(3, 0.0));
    _b_rpy = vector<vector<double>>(_number_of_robots, vector<double>(3, 0.0));
    _b_g = vector<vector<double>>(_number_of_robots, vector<double>(2, 0.0));
    _b_obstacles = vector<vector<double>>(_number_of_obstacles, vector<double>(2, 0.0));

    for(int i=0; i<_name_of_robots.size(); i++)
    {
        SubscriberInfo subscriber_info;
        string topic = _name_of_robots[i] + "/" + _odom_topic_name;
        
        unsigned int topic_index = _dynamic_subscribers.size();
        _topic_to_index[topic] = topic_index;

        vector<string> robot_description = getRobotID(topic);

        subscriber_info._subscriber = this->create_subscription<nav_msgs::msg::Odometry>(topic, 1000, [this, topic_index, robot_description](const nav_msgs::msg::Odometry::SharedPtr message) 
        {
            this->dynamicOdometryCallback(message, topic_index, stoi(robot_description[1]));
        });

        subscriber_info._process = true;

        _dynamic_subscribers.push_back(subscriber_info);
    }
}



PotentialFunction::~PotentialFunction()
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Potential function node is closing...");
}



void PotentialFunction::targetPoseListCallback(const potential_function::msg::TargetPoseList::SharedPtr message)
{
    for(int i=0; i<message->target_pose_list.size(); i++)
    {
        // RCLCPP_INFO_STREAM(this->get_logger(), "_b_[" << i << "][0]: " <<  message->target_pose_list[i].target_pose.position.x << endl);

        _b_g[message->target_pose_list[i].robot_id][0] = message->target_pose_list[i].target_pose.position.x;
        _b_g[message->target_pose_list[i].robot_id][1] = message->target_pose_list[i].target_pose.position.y;
    }
}



void PotentialFunction::dynamicOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr message, unsigned int topic_index, int robot_id)
{
    if(_dynamic_subscribers[topic_index]._process == true)
    {
        _b_[robot_id][0] = message->pose.pose.position.x;
        _b_[robot_id][1] = message->pose.pose.position.y;

        tf2::Quaternion quaternion(
            message->pose.pose.orientation.x,
            message->pose.pose.orientation.y,
            message->pose.pose.orientation.z,
            message->pose.pose.orientation.w
        );

        tf2::Matrix3x3 matrix(quaternion);

        matrix.getRPY(_b_rpy[robot_id][0], _b_rpy[robot_id][1], _b_rpy[robot_id][2]);
    }
}



vector<string> PotentialFunction::getRobotID(const string &topic_name)
{
    vector<string> result;
    regex re(R"(/([a-zA-Z]+)_([0-9]+)_([0-9]+)/)");
    smatch match;

    if(regex_search(topic_name, match, re) && match.size() > 3)
    {    
        result.push_back(match[1].str());
        result.push_back(match[2].str());
        result.push_back(match[3].str());

    }

    return result;
}



void PotentialFunction::findRobotsInRange(void)
{
    for(int robot_id_counter=0; robot_id_counter<_number_of_robots; robot_id_counter++)
    {
        double distance = sqrt(pow((_b_[_robot_id][0] - _b_[robot_id_counter][0]), 2) + pow((_b_[_robot_id][1] - _b_[robot_id_counter][1]), 2));

        if(distance > _limit_distance)
        {
            _b_[robot_id_counter][2] = 0;
        }
        else
        {
            _b_[robot_id_counter][2] = _radius_of_robots;
        }
    }
}



void PotentialFunction::calculateAlphaAndBeta(void)
{
    _alpha = 0.0;
    
    for(int i=0; i<_number_of_robots; i++)
    {
        if(_b_[i][2] != 0)
        {
            double distance = pow((_b_[i][0] - _b_g[i][0]), 2) + pow((_b_[i][1] - _b_g[i][1]), 2);
            RCLCPP_INFO_STREAM(this->get_logger(), "I counter "  << i << " Alpha: " << distance);
            _alpha = _alpha + distance;
        }
    }

    _beta = 1.0;

    for(int i=0; i<_number_of_robots; i++)
    {
        if(i != _robot_id)
        {
            double distance = -1;

            if(_b_[i][2] != 0)
            {
                // Caution, pow(_b_[robot_id][2]) process is required?
                distance = abs(pow((_b_[_robot_id][0] - _b_[i][0]), 2) + pow((_b_[_robot_id][1] - _b_[i][1]), 2) - pow((_b_[_robot_id][2] + _b_[i][2]), 2));
                RCLCPP_INFO_STREAM(this->get_logger(), "I counter "  << i << " Beta: " << distance);
                RCLCPP_INFO_STREAM(this->get_logger(), "I counter " << i << " Distance " << pow((_b_[_robot_id][0] - _b_[i][0]), 2) + pow((_b_[_robot_id][1] - _b_[i][1]), 2));
            }
            if(distance > 0)
            {
                _beta = _beta * distance;
            }
        }
    }

    // Calculating beta value for other robots
    for(int i=0; i<_number_of_robots; i++)
    {
        for(int j=i+1; j<_number_of_robots; j++)
        {
            if((i!= _robot_id) && (j != _robot_id))
            {
                double distance = -1;

                if((_b_[i][2] != 0) && (_b_[j][2] != 0))
                {   
                    // Caution, pow(_b_[i][2]) process is required?
                    distance = abs(pow((_b_[i][0] - _b_[j][0]), 2) + pow((_b_[i][1] - _b_[j][1]), 2) - pow((_b_[i][2] + _b_[j][2]), 2));
                }
                if(distance > 0)
                {
                    _beta = _beta * distance;
                }
            }
        }
    }

    // Calculating beta for obstacles relative to current robot
    for(int i=0; i<_number_of_obstacles; i++)
    {
        double distance = abs(pow((_b_[_robot_id][0] - _b_obstacles[i][0]), 2) + pow((_b_[_robot_id][1] - _b_obstacles[i][1]), 2) - pow((_b_[_robot_id][2] - _b_obstacles[i][2]), 2));

        if(distance < _limit_distance_for_obstacles)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Testing Beta for obstacles...");
            _beta = _beta * distance;
        }
    }

    // Calculating beta for outer (bounder)
    for(int i=0; i<_number_of_robots; i++)
    {
        double distance = -1;
        
        if(_b_[i][2] != 0)
        {
            distance = abs(pow((_radius_outer - _b_[i][2]), 2) - pow(_b_[i][0], 2) - pow(_b_[i][1], 2));
        }
        if(distance > 0)
        {
            _beta = _beta * distance;
        }
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Last value for Beta: " << _beta);

}



void PotentialFunction::calculateDerivativeOfAlphaWithRespectToX(void)
{
    _derivative_of_alpha_with_respect_to_x = 2 * (_b_[_robot_id][0] - _b_g[_robot_id][0]);
}



void PotentialFunction::calculateDerivativeOfAlphaWithRespectToY(void)
{
    _derivative_of_alpha_with_respect_to_y = 2 *(_b_[_robot_id][1] - _b_g[_robot_id][1]);
}



void PotentialFunction::calculateDerivativeOfBetaWithRespectToX(void)
{
    _derivative_of_beta_with_respect_to_x = 0;

    // For current robot and other robots    
    for(int i=0; i<_number_of_robots; i++)
    {
        if(i != _robot_id)
        {
            double derivative_value_for_each_robot;

            if(_b_[i][2] != 0)
            {
                derivative_value_for_each_robot = 2 * (_b_[_robot_id][0] - _b_[i][0]) / (pow((_b_[_robot_id][0] - _b_[i][0]), 2) + pow(_b_[_robot_id][1] - _b_[i][1], 2) - pow(_b_[_robot_id][2] + _b_[i][2], 2));
                _derivative_of_beta_with_respect_to_x = _derivative_of_beta_with_respect_to_x + derivative_value_for_each_robot;
            }
        }
    }

    // For current robot and boundary
    double derivative_value_for_boundary = (-2 * _b_[_robot_id][0]) / (pow(_radius_outer - _b_[_robot_id][2], 2) - pow(_b_[_robot_id][0], 2) - pow(_b_[_robot_id][1], 2));

    _derivative_of_beta_with_respect_to_x = _derivative_of_beta_with_respect_to_x + derivative_value_for_boundary;

    // For current robot and obstacles
    for(int i=0; i<_number_of_obstacles; i++)
    {
        double distance = fabs(pow(_b_[_robot_id][0] - _b_obstacles[i][0], 2) + pow(_b_[_robot_id][1] - _b_obstacles[i][1], 2) - pow(_b_[_robot_id][2] + _b_obstacles[i][2], 2));

        if(distance < _limit_distance_for_obstacles * _limit_distance_for_obstacles)
        {
            double derivative_value_for_each_obstacle = 2 * (_b_[_robot_id][0] - _b_obstacles[i][0]) / (pow(_b_[_robot_id][0] - _b_obstacles[i][0], 2) + pow(_b_[_robot_id][1] - _b_obstacles[i][1], 2) - pow(_b_[_robot_id][2] + _b_obstacles[_robot_id][2], 2));

            _derivative_of_beta_with_respect_to_x = _derivative_of_beta_with_respect_to_x + derivative_value_for_each_obstacle;
        } 
    }

    _derivative_of_beta_with_respect_to_x = _derivative_of_beta_with_respect_to_x * _beta;
}



void PotentialFunction::calculateDerivativeOfBetaWithRespectToY(void)
{
    _derivative_of_beta_with_respect_to_y = 0;

    // For current robot and other robots
    for(int i=0; i<_number_of_robots; i++)
    {
        if(i != _robot_id)
        {
            double derivative_value_for_each_robot;

            if(_b_[i][2] != 0)
            {
                derivative_value_for_each_robot = 2 * (_b_[_robot_id][1] - _b_[i][1]) / (pow(_b_[_robot_id][0] - _b_[i][0], 2) + pow(_b_[_robot_id][1] - _b_[i][1], 2) - pow(_b_[_robot_id][2] + _b_[i][2], 2));
                _derivative_of_beta_with_respect_to_y = _derivative_of_beta_with_respect_to_y + derivative_value_for_each_robot;
            }
        }
    }

    // For current robot and boundary
    double derivative_value_for_boundary = (-2 * _b_[_robot_id][0]) / (pow(_radius_outer - _b_[_robot_id][2], 2) - pow(_b_[_robot_id][0], 2) - pow(_b_[_robot_id][1], 2));

    _derivative_of_beta_with_respect_to_y = _derivative_of_beta_with_respect_to_y + derivative_value_for_boundary;
    
    // For current robot and obstacles
    for(int i=0; i<_number_of_obstacles; i++)
    {
        double distance = fabs(pow(_b_[_robot_id][0] - _b_obstacles[i][0], 2) + pow(_b_[_robot_id][1] - _b_obstacles[i][1], 2));

        if(distance < _limit_distance_for_obstacles * _limit_distance_for_obstacles)
        {
            double derivative_value_for_each_obstacle = 2 * (_b_[_robot_id][1] - _b_obstacles[i][1]) / (pow(_b_[_robot_id][0] - _b_obstacles[i][0], 2) + pow(_b_[_robot_id][1] - _b_obstacles[i][1], 2) - pow(_b_[_robot_id][2] + _b_obstacles[_robot_id][2], 2));

            _derivative_of_beta_with_respect_to_y = _derivative_of_beta_with_respect_to_y + derivative_value_for_each_obstacle;
        }
    }

    _derivative_of_beta_with_respect_to_y = _derivative_of_beta_with_respect_to_y * _beta;
}



void PotentialFunction::calculateDerivativeOfFWithRespectToX(void)
{   
    /** Q = A^k / B **/
    _gama = pow(_alpha, _K_gain) / _beta;

    /** dQ/dA = k * A^(k-1) / B **/
    _derivative_of_gama_with_respect_to_alpha = _K_gain * pow(_alpha, _K_gain - 1) / _beta;

    /** dQ/dB = -1 * A^k / B^2 **/
    _derivative_of_gama_with_respect_to_beta = -1 * pow(_alpha, _K_gain) / pow(_beta, 2);

    /** dQ/dX = dQ/dA * dA/dX + dQ/dB * dB/dX **/
    _derivative_of_gama_with_respect_to_x = _derivative_of_gama_with_respect_to_alpha * _derivative_of_alpha_with_respect_to_x + _derivative_of_gama_with_respect_to_beta * _derivative_of_beta_with_respect_to_x;

    /** dF/dX = 1/k * (Q / (1 + Q))^(1/k - 1) * (1 + Q)^-2 * dQ/dX **/
    _derivative_of_f_with_respect_to_x = (1 / _K_gain) * pow((_gama / (1 + _gama)), (1 / _K_gain - 1)) * pow((1 + _gama), -2) * _derivative_of_gama_with_respect_to_x; 
}



void PotentialFunction::calculateDerivativeOfFWithRespectToY(void)
{
    /** Q = A^k / B **/
    _gama = pow(_alpha, _K_gain) / _beta;

    /** dQ/dA = k * A^(k-1) / B **/
    _derivative_of_gama_with_respect_to_alpha = _K_gain * pow(_alpha, _K_gain - 1) / _beta;

    /** dQ/dB = -1 * A^k / B^2 **/
    _derivative_of_gama_with_respect_to_beta = -1 * pow(_alpha, _K_gain) / pow(_beta, 2);

    /** dQ/dY = dQ/dA * dA/dY + dQ/dB * dB/dY **/
    _derivative_of_gama_with_respect_to_y = _derivative_of_gama_with_respect_to_alpha * _derivative_of_alpha_with_respect_to_y + _derivative_of_gama_with_respect_to_beta * _derivative_of_beta_with_respect_to_y;

    /** dF/dY =  1/k * (Q / (1 + Q))^(1/k - 1) * (1 + Q)^-2 * dQ/dY **/
    _derivative_of_f_with_respect_to_y = (1 / _K_gain) * pow((_gama / (1 + _gama)), (1 / _K_gain - 1)) * pow((1 + _gama), -2) * _derivative_of_gama_with_respect_to_y; 
}



void PotentialFunction::robotController(void)
{
    findRobotsInRange();

    calculateAlphaAndBeta();
    calculateDerivativeOfAlphaWithRespectToX();
    calculateDerivativeOfBetaWithRespectToX();
    
    calculateDerivativeOfAlphaWithRespectToY();;
    calculateDerivativeOfBetaWithRespectToY();
    
    calculateDerivativeOfFWithRespectToX();
    calculateDerivativeOfFWithRespectToY();

    double b_out_x = -1 * _derivative_of_f_with_respect_to_x;
    double b_out_y = -1 * _derivative_of_f_with_respect_to_y;


    _desired_heading = atan2(b_out_y, b_out_x);
    double heading_error =  _desired_heading - _b_rpy[_robot_id][2];

    heading_error = atan2(sin(heading_error), cos(heading_error));

    // _cmd_vel_message.angular.z = PIDController(0.1, 0.0, 0.0, 0.1, heading_error);
    // _cmd_vel_message.linear.x = 0.1;

    _cmd_vel_publisher->publish(_cmd_vel_message);


    RCLCPP_INFO_STREAM(this->get_logger(), "b_out_x: " << b_out_x);
    RCLCPP_INFO_STREAM(this->get_logger(), "b_out_y: " << b_out_y);

    RCLCPP_INFO_STREAM(this->get_logger(), "CMD Vel Anguler Z: " << _cmd_vel_message.angular.z);
    RCLCPP_INFO_STREAM(this->get_logger(), "Actual Heading: " << _b_rpy[_robot_id][2] * 180 / PI);
    RCLCPP_INFO_STREAM(this->get_logger(), "Desired Heading: " << _desired_heading * 180 / PI );
    RCLCPP_INFO_STREAM(this->get_logger(), "Heading Error: " << heading_error * 180 / PI);
    RCLCPP_INFO_STREAM(this->get_logger(), "----------------------------");
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<PotentialFunction>());
    return 0;
}

