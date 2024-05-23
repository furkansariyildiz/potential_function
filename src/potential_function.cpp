#include <potential_function/potential_function.hpp>

using namespace std;



PotentialFunction::PotentialFunction():
Node("potential_function_node")
{   
    _target_pose_list_subscriber = this->create_subscription<potential_function::msg::TargetPoseList>("/target_pose_list", 1000, bind(&PotentialFunction::targetPoseListCallback, this, placeholders::_1));

    _robot_controller_timer = this->create_wall_timer(1ms, bind(&PotentialFunction::robotController, this));

    declare_parameter("number_of_robots", 100);
    declare_parameter("number_of_obstacles", 100);
    declare_parameter("K_gain", 1);
    declare_parameter("name_of_robots", vector<string>{"tb1", "tb2"});
    declare_parameter("odom_topic", "odom");


    _number_of_robots = this->get_parameter("number_of_robots").as_int();
    _number_of_obstacles = this->get_parameter("number_of_obstacles").as_int();
    _K_gain = this->get_parameter("K_gain").as_int();
    _name_of_robots = this->get_parameter("name_of_robots").as_string_array();
    _odom_topic_name = this->get_parameter("odom_topic").as_string();
    

    for(int i=0; i<_name_of_robots.size(); i++)
    {
        SubscriberInfo subscriber_info;
        string topic = _name_of_robots[i] + "/" + _odom_topic_name;
        
        unsigned int topic_index = _dynamic_subscribers.size();
        _topic_to_index[topic] = topic_index;
        
        RCLCPP_INFO_STREAM(this->get_logger(), "Odom topic name: " << topic << endl);

        subscriber_info._subscriber = this->create_subscription<nav_msgs::msg::Odometry>(topic, 1000, [this, topic_index](const nav_msgs::msg::Odometry::SharedPtr message) 
        {
            this->dynamicOdometryCallback(message, topic_index);
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
        _b_g[message->target_pose_list[i].robot_id][0] = message->target_pose_list[i].target_pose.position.x;
        _b_g[message->target_pose_list[i].robot_id][1] = message->target_pose_list[i].target_pose.position.y;
    }
}



void PotentialFunction::dynamicOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr message, unsigned int topic_index)
{
    if(_dynamic_subscribers[topic_index]._process == true)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Dynamic topic index: " << topic_index << endl);
    }
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

            if(_b_[i][3] != 0)
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

            if(_b_[i][3] != 0)
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
    _b_ = vector<vector<double>>(_number_of_robots, vector<double>(3, 0.0));
    _b_g = vector<vector<double>>(_number_of_robots, vector<double>(2, 0.0));
    _b_obstacles = vector<vector<double>>(_number_of_obstacles, vector<double>(2, 0.0));

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

    // RCLCPP_INFO_STREAM(this->get_logger(), "b_out_x: " << b_out_x);
    // RCLCPP_INFO_STREAM(this->get_logger(), "b_out_y: " << b_out_y);
    // RCLCPP_INFO_STREAM(this->get_logger(), "----------------------------" << endl);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<PotentialFunction>());
    return 0;
}

