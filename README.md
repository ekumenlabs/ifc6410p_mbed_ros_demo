# Inforce's mbed PAC shield rosserial demo


## Subscribed Topics

- cmd_vel       - a geometry_msgs/Twist message with the velocity command to move the robot's servors [-1.0, 1.0]
- toggle_led    - a std_msgs/Empty message that toggles the red LED on the board
- rgb           - a std_msgs/ColorRGBA message that sets the RGB LED to a specific color (doesn't support PWM)

## Published Topics

- imu           - a sensor_msgs/Imu message with linear acceleration and angular velocities (orientation quaternions missing) read from the onboard MPU9250 IMU
- temperature   - a std_msgs/Float64 with the temperature read from the onboard MPu9250 IMU
- magnetometer  - a geometry_msgs/Vector3 message with the magnetometer readings from the onboard MPU9250 IMU
- pressure      - a  std_msgs/Float64 with the barometric pressure from the MPL3115A2 barometer

## To run the on the IFC6410

1. Connect to your IFC6410p board (via ssh or with a screen, mouse and keyboard)
2. In one terminal start the master node by running:
    ```
    roscore
    ```
2. In another terminal run the rosserial serial node:
    ```
    rosrun rosserial_python serial_node.py /dev/ttyHS2
    ```
3. Now you can list all the available topics by running on another terminal:
    ```
    rostopic list
    ```
4. You can also connect to the ROS session running on the IFC6410p by doing on your computer:
    ```
    export ROS_MASTER_URI=http://<ifc6410p-ip-address>:11311
    rostopic list
    ```
If the last command doesn't work, check http://wiki.ros.org/ROS/Tutorials/MultipleMachines
