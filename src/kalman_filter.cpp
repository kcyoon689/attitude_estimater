#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "KF_node");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher data_pub =
      n.advertise<std_msgs::Int32MultiArray>("data", 1000);
  ros::Rate loop_rate(10);

  int count = 0;

  while (ros::ok()) {
    std_msgs::String msg;
    std_msgs::Int32MultiArray data;
    std::stringstream ss;

    ss << "hello world " << count;  // "hello" + str(count)
    msg.data = ss.str();

    // data.data.resize(5);
    // data.data[0] = 5; // std::vector
    // data.data[2] = 3;
    // data.data[1] = 4;
    // data.data[3] = 2;
    // data.data[4] = 1; // c
    // data.data.at(4) = 1; // c++
    data.data.push_back(5);
    // STL: Standard Template Library -> Vector - dynamic array
    data.data.push_back(4);
    data.data.push_back(3);
    data.data.push_back(2);

    ROS_INFO("%s", msg.data.c_str());  // c_str: string(c++) -> char array(c)
    ROS_INFO("%d %d %d %d", data.data[0], data.data[1], data.data[2],
             data.data[3]);

    chatter_pub.publish(msg);
    data_pub.publish(data);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}