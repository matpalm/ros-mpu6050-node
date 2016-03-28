#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
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
  ros::Publisher pub = node.advertise<std_msgs::Float32MultiArray>("mpu6050", 10);
  ros::Rate rate(10);  // hz

  // Publish in loop.
  while(ros::ok()) {
    std_msgs::Float32MultiArray msg;

    // Read raw gyroscope values.
    float gx = read_word_2c(fd, 0x43);
    float gy = read_word_2c(fd, 0x45);
    float gz = read_word_2c(fd, 0x47);
    // At default sensitivity of 250deg/s we need to scale by 131.
    msg.data.push_back(gx / 131);
    msg.data.push_back(gy / 131);
    msg.data.push_back(gz / 131);

    // Read raw accelerometer values.
    float ax = read_word_2c(fd, 0x3b);
    float ay = read_word_2c(fd, 0x3d);
    float az = read_word_2c(fd, 0x3f);
    // At default sensitivity of 2g we need to scale by 16384.
    msg.data.push_back(ax / 16384);
    msg.data.push_back(ay / 16384);
    msg.data.push_back(az / 16384);
    // Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
    pub.publish(msg);
    
    rate.sleep();
  }
  return 0;
}

