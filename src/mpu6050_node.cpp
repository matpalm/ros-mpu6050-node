#include <ros/ros.h>
#include <std_msgs/Int32.h>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "mpu6050");
  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<std_msgs::Int32>("counter", 10);

  ros::Rate rate(1);  // 1hz
  int count = 0;

  while(ros::ok()) {
    rate.sleep();
    std_msgs::Int32 msg;
    msg.data = count;
    pub.publish(msg);
    cout << "published " << count << endl;
    ++count;
  }
  return 0;
}

