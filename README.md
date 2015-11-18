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


