#include <potential_function/potential_function.hpp>

using namespace std;



PotentialFunction::PotentialFunction():
Node("potential_function_node")
{   
    _target_pose_list_subscriber = this->create_subscription<potential_function::msg::TargetPoseList>("/target_pose_list", 1000, bind(&PotentialFunction::targetPoseListCallback, this, placeholders::_1));

    _b_ = vector<vector<double>>(_number_of_robots, vector<double>(3));
    _b_g = vector<vector<double>>(_number_of_robots, vector<double>(2));
    _b_obstacles = vector<vector<double>>(_number_of_obstacles, vector<double>(2));
}



PotentialFunction::~PotentialFunction()
{
    
}



void PotentialFunction::targetPoseListCallback(const potential_function::msg::TargetPoseList::SharedPtr message)
{
    for(int i=0; i<message->target_pose_list.size(); i++)
    {
        _b_g[message->target_pose_list[i].robot_id][0] = message->target_pose_list[i].target_pose.position.x;
        _b_g[message->target_pose_list[i].robot_id][1] = message->target_pose_list[i].target_pose.position.y;
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



void PotentialFunction::calculateAAndB(void)
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



void PotentialFunction::calculateDerivativeOfAWithRespectToX(void)
{
    _derivative_of_alpha_with_respect_to_x = 2 * (_b_[_robot_id][0] - _b_g[_robot_id][0]);
}



void PotentialFunction::calculateDerivativeOfAWithRespectToY(void)
{
    _derivative_of_alpha_with_respect_to_y = 2 *(_b_[_robot_id][1] - _b_g[_robot_id][1]);
}



void PotentialFunction::calculateDerivativeOfBWithRespectToX(void)
{
    _derivative_of_beta_with_respect_to_x = 0;
    
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
}



void PotentialFunction::calculateDerivativeOfBWithRespectToY(void)
{

}



int main(int argc, char *argv[])
{
    return 0;
}

