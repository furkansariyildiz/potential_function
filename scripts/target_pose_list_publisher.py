#!/usr/bin/env python3

# import rclpy
# import math
# from rclpy.node import Node

# from potential_function.msg import TargetPose, TargetPoseList


# class TargetPoseListPublisher(Node):
#     def __init__(self):
#         super().__init__("target_pose_list_publisher_node")

#         self._target_pose_list_publisher = self.create_publisher(TargetPoseList, "/target_pose_list", 10)

        

#         self._target_pose_list_message = TargetPoseList()

#         self._dt = 0.01

#         self._main_loop_timer = self.create_timer(self._dt, self.mainLoop)



#     def debug(self, message):
#         self.get_logger().info(str(message))



#     def prepareTargetPoseList(self):
#         target_pose_for_current_robot = TargetPose()

#         target_pose_for_current_robot.robot_id = 0
#         target_pose_for_current_robot.target_pose.position.x = 1.0
#         target_pose_for_current_robot.target_pose.position.y = 1.0

#         self._target_pose_list_message.target_pose_list.append(target_pose_for_current_robot)
        


#     def mainLoop(self):
#         self.debug("test")



# def main(args=None):
#     rclpy.init(args=args)

#     target_pose_list_publisher = TargetPoseListPublisher()
    
#     rclpy.spin(target_pose_list_publisher)

#     rclpy.shutdown()



# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class TargetPoseListPublisher(Node):

    def __init__(self):
        super().__init__('target_pose_list_publisher_node')
        
        # Create a publisher
        self.publisher_ = self.create_publisher(Pose, 'target_pose_list', 10)

        # Timer to publish poses
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.publish_poses)

    def publish_poses(self):
        # Dynamically get all parameters that start with 'tb_'
        
        parameters = self.get_parameters_by_prefix('tb_')
        
        print(parameters)

        for param_name, param_value in parameters.items():
            if isinstance(param_value, dict) and 'x' in param_value and 'y' in param_value:
                x = param_value['x']
                y = param_value['y']
                
                # Create a Pose message
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = 0.0

                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                pose.orientation.w = 1.0

                # Publish the Pose
                self.publisher_.publish(pose)
                self.get_logger().info(f'Published pose: {pose.position.x}, {pose.position.y}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseListPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
