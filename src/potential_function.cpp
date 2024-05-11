#include <potential_function/potential_function.hpp>

using namespace std;



PotentialFunction::PotentialFunction():
Node("potential_function_node")
{   
    _target_pose_list_subscriber = this->create_subscription<potential_function::msg::TargetPoseList>("/target_pose_list", 1000, bind(&PotentialFunction::targetPoseListCallback, this, placeholders::_1));

    _b_i = vector<vector<double>>(_number_of_robots, vector<double>(3));
}



PotentialFunction::~PotentialFunction()
{
    
}



void PotentialFunction::targetPoseListCallback(const potential_function::msg::TargetPoseList::SharedPtr message)
{
    
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
    for(int i=0; i<_number_of_robots; i++)
    {
        // if()
    }
}



void PotentialFunction::calculateDerivativeOfAWithRespectToX(void)
{

}



int main(int argc, char *argv[])
{
    return 0;
}

