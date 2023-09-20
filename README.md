__Tinymovr ROS Hardware Interface__

A ROS package that provides hardware interfacing for Tinymovr devices. This interface allows for seamless integration of Tinymovr devices with ROS-based robotic systems, offering joint state and control through ROS topics and services.

## Features
- Real-time reading of joint positions, velocities, and efforts.
- Ability to send joint command setpoints.
- Error handling and robust exception management.
- Compatible with standard ROS controllers, enabling a plug-and-play experience.

## Prerequisites

- ROS (Robot Operating System) - Tested with ROS Noetic, but should be compatible with other versions.
- SocketCAN tools and utilities installed.
- Tinymovr devices properly set up and calibrated.

## Installation

1. Navigate to your catkin workspace's source folder:

```bash
cd ~/catkin_ws/src/
```

2. Clone the repository:

```bash
git clone git@github.com:tinymovr/Tinymovr-ROS.git
```

3. Build your catkin workspace:

```bash
cd ~/catkin_ws/
catkin_make
```

4. Source the workspace:

```bash
source devel/setup.bash
```

## Run the Diffbot demo!

1. Ensure your Tinymovr instances are calibrated and well tuned, test functioning using Tinymovr Studio or CLI.

2. Configure your hardware in `config/hardware.yaml` and diff drive config in `config/diff_drive_config.yaml`

3. Start the `tinymovr_diffbot_demo_node` node:

```bash
roslaunch tinymovr_ros tinymovr_diffbot_demo_node.launch
```

4. Spin up a keyboard teleop and drive your robot:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/diff_drive_controller/cmd_vel
```

## Configuration

To customize the behavior of the Tinymovr ROS, adjust the parameters in the `config` directory. Here, you can set specifics about each joint, including joint names, IDs, and other parameters relevant to your hardware setup.

## API Documentation

Further details about the API and individual functions can be found in the generated Doxygen documentation. Please refer to the documentation for advanced use cases.

## Contributing

Contributions to improve and expand the functionality of Tinymovr ROS are welcome! Please open an issue or submit a pull request on the GitHub repository.

## License

This package is licensed under the [MIT License](LICENSE).
