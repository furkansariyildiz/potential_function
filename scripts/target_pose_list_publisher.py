#!/usr/bin/env python3


import rclpy
import re
from rclpy.node import Node
from geometry_msgs.msg import Pose
from potential_function.msg import TargetPoseList, TargetPose


class TargetPoseListPublisher(Node):
    def __init__(self):
        super().__init__('target_pose_list_publisher_node')

        # Create a publisher
        self._target_pose_list_publisher = self.create_publisher(TargetPoseList, 'target_pose_list', 10)

        # Declare parameters dynamically based on prefix 'tb_'
        self.declare_parameters(namespace='',parameters=[('tb_0_0.x', 0.0), ('tb_0_0.y', 0.0), ('tb_1_0.x', 0.0), ('tb_1_0.y', 0.0), ('tb_2_0.x', 0.0), ('tb_2_0.y', 0.0)])

        # Timer to publish poses
        self._dt = 0.1  # seconds
        self._timer = self.create_timer(self._dt, self.publishTargetPoseList)



    def publishTargetPoseList(self):
        parameter_names = self._parameters.keys()

        parameter_pairs = {}

        target_pose_list = TargetPoseList()

        for parameter_name in parameter_names:
            if parameter_name.startswith('tb_'):
                base_name = parameter_name[:-2]  # remove '.x' or '.y'
                if base_name not in parameter_pairs:
                    parameter_pairs[base_name] = {}
                if parameter_name.endswith('.x'):
                    parameter_pairs[base_name]['x'] = self.get_parameter(parameter_name).value
                elif parameter_name.endswith('.y'):
                    parameter_pairs[base_name]['y'] = self.get_parameter(parameter_name).value

        for base_name, coordinates in parameter_pairs.items():
            if 'x' in coordinates and 'y' in coordinates:
                regex_match = re.match(r'tb_(\d+)_\d+', base_name)
                robot_id = regex_match.group(1)

                x = coordinates['x']
                y = coordinates['y']

                target_pose = TargetPose()

                target_pose.robot_id =  int(robot_id)
                target_pose.target_pose.position.x = x
                target_pose.target_pose.position.y = y

                target_pose_list.target_pose_list.append(target_pose)

        self._target_pose_list_publisher.publish(target_pose_list)



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
