#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#define I2C_ADDR 0x68
#define PWR_MGMT_1 0x6B

using namespace std;

float read_word_2c(int fd, int addr) {
  int high = wiringPiI2CReadReg8(fd, addr);
  int low = wiringPiI2CReadReg8(fd, addr+1);
  int val = (high << 8) + low;
  return float((val >= 0x8000) ? -((65535 - val) + 1) : val);
}

int main(int argc, char **argv) {

  // Connect to device.
  int fd = wiringPiI2CSetup(I2C_ADDR);
  if (fd == -1) {
    printf("no i2c device found?\n");
    return -1;
  }
  // Device starts in sleep mode so wake it up.
  wiringPiI2CWriteReg16(fd, PWR_MGMT_1, 0);

  // Start ROS node stuff.
  ros::init(argc, argv, "mpu6050");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<sensor_msgs::Imu>("mpu6050", 10);
  ros::Rate rate(10);  // hz

  // Publish in loop.
  while(ros::ok()) {
    sensor_msgs::Imu msg;

    // Read yroscope values.
    // At default sensitivity of 250deg/s we need to scale by 131.
    msg.angular_velocity.x = read_word_2c(fd, 0x43) / 131;
    msg.angular_velocity.y = read_word_2c(fd, 0x45) / 131;
    msg.angular_velocity.z = read_word_2c(fd, 0x47) / 131;

    // Read accelerometer values.
    // At default sensitivity of 2g we need to scale by 16384.
    // Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
    msg.linear_acceleration.x = read_word_2c(fd, 0x3b) / 16384;
    msg.linear_acceleration.y = read_word_2c(fd, 0x3d) / 16384;
    msg.linear_acceleration.z = read_word_2c(fd, 0x3f) / 16384;

    // Pub & sleep.
    pub.publish(msg);
    rate.sleep();
  }
  return 0;
}

