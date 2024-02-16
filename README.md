__Tinymovr ROS Hardware Interface__

A ROS package that provides hardware interfacing for the [Tinymovr](https://tinymovr.com) motor controller. This interface allows for seamless integration of Tinymovr devices with ROS-based robotic systems, offering joint state and control through ROS topics and services.

## Features
- Real-time reading of joint positions, velocities, and efforts.
- Ability to send joint command setpoints.
- Error handling and robust exception management.
- Compatible with standard ROS controllers, enabling a plug-and-play experience.

## Prerequisites

- ROS (Robot Operating System) - Tested with ROS Noetic, but should be compatible with other versions.
- SocketCAN tools and utilities installed.
- Tinymovr devices with firmware 1.6.x
- Devices properly set up and calibrated.

> [!NOTE]
> If you plan to use the CANine adapter, you need to flash it with the Candlelight firmware, which is compatible with socketcan. Use [this web-based flasher](https://canable.io/updater/canable1.html) for easy upgrade. Use Chrome and choose the Candlelight firmware from the drop-down list.

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

## Bring up Socketcan

Depending on your device you may need to add the correct module to the kernel. Following that, bring up the interface as follows:

```bash
sudo ip link set can0 type can bitrate 1000000
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

## External Links

[ROS Wiki Page](http://wiki.ros.org/Robots/tinymovr)

## License

This package is licensed under the [Apache 2.0 License](LICENSE).
