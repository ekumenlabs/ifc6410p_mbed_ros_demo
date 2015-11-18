/*
 * rosserial Example for Inforce's mbed PAC shield
 */
#include "mbed.h"

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#include "MPU9250.h"
#include "MPL3115A2.h"
#include "Servo.h"

// Adjust according to your servos
#define L_SERVO_STOP    0.485f;
#define R_SERVO_STOP    0.547f;

DigitalOut red_led(LED1);
DigitalOut blue_led(LED2);
DigitalOut green_led(LED3);

// Servos
Servo lServo(D5);
Servo rServo(D6);

// Accelerometer
SPI spi(SPI_MOSI, SPI_MISO, SPI_SCK);
mpu9250_spi imu(spi,SPI_CS);   // define the mpu9250 object

// Pressure sensor
#define MPL3115A2_I2C_ADDRESS (0x60<<1)
MPL3115A2 wigo_sensor1( I2C_SDA, I2C_SCL, MPL3115A2_I2C_ADDRESS);/*SDA,SCL,I2C ADDR*/
I2C i2c(I2C_SDA, I2C_SCL);

// Override the serial pins via the nodehandle
class NewHardware : public MbedHardware
{
  public:
  NewHardware():MbedHardware(SERIAL2_TX, SERIAL2_RX, 57600){};
};

ros::NodeHandle_<NewHardware>  nh;

// Publisher for imu
sensor_msgs::Imu imu_msg;
ros::Publisher pub_imu("imu", &imu_msg);

// Publisher for temperature
std_msgs::Float64 temp_msg;
ros::Publisher pub_temp("temperature", &temp_msg);

// Publisher for magnetometer
geometry_msgs::Vector3 mag_msg;
ros::Publisher pub_mag("magnetometer", &mag_msg);

// Publisher for pressure
std_msgs::Float64 pressure_msg;
ros::Publisher pub_pressure("pressure", &pressure_msg);

// Tickers for reading sensors and publish the data
Ticker mpu;
Ticker mpl;

void getMPUvalues(){
    imu.read_all();

    Now = t.read_us();
    deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    MadgwickQuaternionUpdate(imu.accelerometer_data[0], imu.accelerometer_data[1], imu.accelerometer_data[2],
                             imu.gyroscope_data[0]*PI/180.0f, imu.gyroscope_data[1]*PI/180.0f, imu.gyroscope_data[2]*PI/180.0f,
                             imu.Magnetometer[1], imu.Magnetometer[0], imu.Magnetometer[2]);

    // Fill header
    imu_msg.header.frame_id = "imu";

    // TODO: Fill rotation quaternion

    // Fill angular velocities
    imu_msg.angular_velocity.x = imu.gyroscope_data[0];
    imu_msg.angular_velocity.y = imu.gyroscope_data[1];
    imu_msg.angular_velocity.z = imu.gyroscope_data[2];
    imu_msg.angular_velocity_covariance[0] = 0.0003;
    imu_msg.angular_velocity_covariance[4] = 0.0003;
    imu_msg.angular_velocity_covariance[8] = 0.0003;

    // Fill linear accelerations
    imu_msg.linear_acceleration.x = imu.accelerometer_data[0];
    imu_msg.linear_acceleration.y = imu.accelerometer_data[1];
    imu_msg.linear_acceleration.z = imu.accelerometer_data[2];
    imu_msg.linear_acceleration_covariance[0] = 0.0003;
    imu_msg.linear_acceleration_covariance[4] = 0.0003;
    imu_msg.linear_acceleration_covariance[8] = 0.0003;

    pub_imu.publish(&imu_msg);

    // Send temperature info
    temp_msg.data = imu.Temperature;
    pub_temp.publish(&temp_msg);

    // Send magnetometer info
    mag_msg.x = imu.Magnetometer[0];
    mag_msg.y = imu.Magnetometer[1];
    mag_msg.z = imu.Magnetometer[2];
    pub_mag.publish(&mag_msg);

    wait(0.1);
}

void getMPLvalues(){
    float sensor_data[2];
    // Data acquisition using polling method and delta values
    bool flag = false;

    // Set over sampling value (see MPL3115A2.h for details)
    wigo_sensor1.Oversample_Ratio( OVERSAMPLE_RATIO_128);
    // Configure the sensor as Barometer.
    wigo_sensor1.Barometric_Mode();

    while(flag == false) {
        if ( wigo_sensor1.getAllData( &sensor_data[0])) {
            // Send temperature info
            pressure_msg.data = sensor_data[0];
            pub_pressure.publish(&pressure_msg);

            flag = true;
        }
        wait(0.5);
    }
}

// Velocity commands callback
void cmdVelCb( const geometry_msgs::Twist& cmd_msg) {
  if ( cmd_msg.angular.z == 0.0 && cmd_msg.linear.x == 0.0 ) {
    lServo = L_SERVO_STOP;
    rServo = R_SERVO_STOP;
  } else {
    if ( cmd_msg.angular.z < -1.0 ) {
        lServo = -1.0;
        rServo = -1.0;
    } else if ( cmd_msg.angular.z > 1.0 ) {
        lServo = 1.0;
        rServo = 1.0;
    } else if ( cmd_msg.linear.x < -0.1 ) {
        lServo = (-cmd_msg.linear.x + 1) / 2;
        rServo = (cmd_msg.linear.x + 1) / 2;
    } else if ( cmd_msg.linear.x > 0.1 ) {
        lServo = (cmd_msg.linear.x - 1) / 2;
        rServo = (cmd_msg.linear.x + 1) / 2;
    }
  }
}

void toggleCb(const std_msgs::Empty& toggle_msg){
    red_led = !red_led;   // blink the red led
}

// RGB led callback
void rgbCb(const std_msgs::ColorRGBA& rgb_msg){
    red_led = (rgb_msg.r > 0.0f) ? 0 : 1;
    green_led = (rgb_msg.g > 0.0f) ? 0 : 1;
    blue_led = (rgb_msg.b > 0.0f) ? 0 : 1;
}

ros::Subscriber<std_msgs::Empty> sub_toggle("toggle_led", &toggleCb);
ros::Subscriber<geometry_msgs::Twist> sub_vel("cmd_vel", &cmdVelCb);
ros::Subscriber<std_msgs::ColorRGBA> sub_led("rgb", &rgbCb);

int main() {
    // Turn all leds off
    red_led = 1;
    green_led = 1;
    blue_led = 1;

    // These are the values to keep the servos still.
    // Adjust according to your servos
    lServo = L_SERVO_STOP;
    rServo = R_SERVO_STOP;

    // Configure IMU
    imu.set_gyro_scale(BITS_FS_2000DPS);
    wait(0.1);
    imu.set_acc_scale(BITS_FS_16G);
    wait(0.1);
    imu.AK8963_calib_Magnetometer();
    wait(0.1);

    // Init the ROS nodehandle
    nh.initNode();

    // Setup subscribers
    nh.subscribe(sub_toggle);
    nh.subscribe(sub_vel);
    nh.subscribe(sub_led);

    // Advertise topics
    nh.advertise(pub_imu);
    nh.advertise(pub_temp);
    nh.advertise(pub_mag);
    nh.advertise(pub_pressure);

    // Configure tickers for reading the sensors
    mpu.attach(&getMPUvalues, 0.2);
    mpl.attach(&getMPLvalues, 1.0);

    while (1) {
        nh.spinOnce();
        wait_ms(1);
    }
}
