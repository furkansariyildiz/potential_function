#include <potential_function/potential_function.hpp>

using namespace std;



PotentialFunction::PotentialFunction():
Node("potential_function_node")
{   
    _target_pose_list_subscriber = this->create_subscription<potential_function::msg::TargetPoseList>("/target_pose_list", 1000, bind(&PotentialFunction::targetPoseListCallback, this, placeholders::_1));

    _b_i = vector<vector<double>>(_number_of_robots, vector<double>(3));
    _b_g = vector<vector<double>>(_number_of_robots, vector<double>(2));
    _b_rs = vector<vector<double>>(_number_of_robots, vector<double>(3));
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
        double distance = sqrt(pow((_b_i[_robot_id][0] - _b_i[robot_id_counter][0]), 2) + pow((_b_i[_robot_id][1] - _b_i[robot_id_counter][1]), 2));

        if(distance > _limit_distance)
        {
            _b_i[robot_id_counter][2] = 0;
        }
    }
}



void PotentialFunction::calculateAAndB(void)
{
    _alpha = 0.0;
    
    for(int i=0; i<_number_of_robots; i++)
    {
        if(_b_i[i][2] != 0)
        {
            double distance = pow((_b_i[i][0] - _b_g[i][0]), 2) + pow((_b_i[i][1] - _b_g[i][1]), 2);
        
            _alpha = _alpha + distance;
        }
    }

    _beta = 1.0;

    for(int i=0; i<_number_of_robots; i++)
    {
        if(i != _robot_id)
        {
            double distance = -1;

            if(_b_i[i][2] != 0)
            {
                // Caution, pow(_b_i[robot_id][2]) process is required?
                distance = abs(pow((_b_i[_robot_id][0] - _b_i[i][0]), 2) + pow((_b_i[_robot_id][1] - _b_i[i][1]), 2) - pow((_b_i[_robot_id][2] + _b_i[i][2]), 2));
            }
            else if(_b_rs[i][2] != 0)
            {
                distance = abs(pow((_b_i[_robot_id][0] - _b_rs[i][0]), 2) + pow((_b_i[_robot_id][1] - _b_rs[i][1]), 2) - pow((_b_i[_robot_id][2] + _b_rs[i][2]), 2));
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

                if((_b_i[i][2] != 0) && (_b_i[j][2] != 0))
                {   
                    // Caution, pow(_b_i[i][2]) process is required?
                    distance = abs(pow((_b_i[i][0] - _b_i[j][0]), 2) + pow((_b_i[i][1] - _b_i[j][1]), 2) - pow((_b_i[i][2] + _b_i[j][2]), 2));
                }
                else if((_b_rs[i][2] != 0) && (_b_i[j][2] != 0))
                {
                    distance = abs(pow((_b_rs[i][0] - _b_i[j][0]), 2) + pow((_b_rs[i][1] - _b_i[j][1]), 2) - pow((_b_rs[i][2] + _b_i[j][2]), 2));
                }
                else if((_b_i[i][2] != 0) && (_b_rs[j][2] != 0))
                {
                    distance = abs(pow((_b_i[i][0] - _b_rs[j][0]), 2) + pow((_b_i[i][1] - _b_rs[j][1]), 2) - pow((_b_i[i][2] + _b_rs[j][2]), 2));
                }
                else if((_b_rs[i][2] != 0) && (_b_rs[j][2] != 0))
                {
                    distance = abs(pow((_b_rs[i][0] - _b_rs[j][0]), 2) + pow((_b_rs[i][1] - _b_rs[j][1]), 2) - pow((_b_rs[i][2] + _b_rs[j][2]), 2));
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
        double distance = abs(pow((_b_i[_robot_id][0] - _b_obstacles[i][0]), 2) + pow((_b_i[_robot_id][1] - _b_obstacles[i][1]), 2) - pow((_b_i[_robot_id][2] - _b_obstacles[i][2]), 2));

        if(distance < _limit_distance_for_obstacles)
        {
            _beta = _beta * distance;
        }
    }

    // Calculating beta for outer (bounder)
    for(int i=0; i<_number_of_robots; i++)
    {
        double distance = -1;
        
        if(_b_i[i][2] != 0)
        {
            distance = abs(pow((_radius_outer - _b_i[i][2]), 2) - pow(_b_i[i][0], 2) - pow(_b_i[i][1], 2));
        }
        else if(_b_rs[i][2] != 0)
        {
            distance = abs(pow((_radius_outer - _b_rs[i][2]), 2) - pow(_b_rs[i][0], 2) - pow(_b_rs[i][1], 2));
        }

        if(distance > 0)
        {
            _beta = _beta * distance;
        }
    }
}



void PotentialFunction::calculateDerivativeOfAWithRespectToX(void)
{

}



int main(int argc, char *argv[])
{
    return 0;
}

