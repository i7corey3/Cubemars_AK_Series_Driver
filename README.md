# Cubemars_AK_Series_Driver

This repository allows the ability to control the CubeMars AK series motors through CAN communication.

> Tested on Ubuntu 20.04

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

## Single Motor Demo

To control one motor source the ros workspace then in one terminal run:

```
ros2 run ak_series_driver single
```

Then in another terminal run:

```
ros2 run control control
```

Inside the second terminal, follow the instructions to set the motors parameters and run the motor.

Make sure to add spaces between each of the five values before sending.

Additional commands to use are start, stop and reset.

# Discaimer

This code has not been fully tested, Use at your own risk!



