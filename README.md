# Cubemars_AK_Series_Driver

This repository allows the ability to control the CubeMars AK series motors through CAN communication.

> Tested on Ubuntu 20.04 with ROS2 Galactic

## Hardware 

The hardware required for this repository is the Innomaker USB2CAN-X2

<p align="center">
    <img src="https://www.inno-maker.com/wp-content/uploads/2021/10/Dual-Channels-USB-CAN-Module_07.jpg" width="250" height="250">
</p>

## Setup

First run the setup.sh script.
This will install the can-utils commands used to interface with the USB2CAN device

```
./dependencies/setup.sh
```
Next, build the ros workspace

```
colcon build --symlink-install
```

# Run Demo

To run this demo start by launching the launch file located in the ak_driver_status package
This will launch the ak_series_driver and a python package that will capture the can frames and create the respected topics in real time!

Make sure to setup the motor names and motor address in the params.yaml file inside the ak_driver_status package. The address is defined using the CubeMars upper computer software. Look for the controller id setting and enter that in the motor_addr parameter.

The motors are assumbed to be running in servo mode and connected via canbus.

To launch the launch file:

```
ros2 launch ak_driver_status ak_driver_status.launch.py
```

To send commands to the motors run the following ros2 node:

```
ros2 run ak_driver_status control
```

# Viewing the current motor state

To view the motor status, echo the topic /ak_driver/motor_status/{motor_name}

```
ros2 topic echo /ak_driver/motor_status/<motor_name>
```





