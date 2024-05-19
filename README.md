# potential_function

<p align="center">
  <img src="documents/function.png" style="width: 50%; height: 50%"/>
</p>

### Clonning potential_function package to your workspace
```bash
cd ~ros2_ws/src
git clone git@github.com:furkansariyildiz/potential_function.git
```

### Building package
```bash
source /opt/ros/$ROS_ENV/setup.bash
colcon build --symlink-install --packages-select potential_function
```

### Running ros2 potential_function package
```bash
ros2 launch potential_function potential_function.launch.py
```