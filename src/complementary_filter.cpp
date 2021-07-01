#include <math.h>
#include <iostream>
#include <vector>

#include "ros/ros.h"
// #include "std_msgs/String.h"
// #include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/Imu.h"

//    # rospy.loginfo(rospy.get_caller_id() + 'num_pub: [%f,%f,%f,%f]',
//    msg.data[0], msg.data[1], msg.data[2], msg.data[3]) # msg.data : list

using namespace std;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  static bool isFirstRun = true;
  static double prevTime = 0.0;

  double curTime = 0.0;
  double dt = 0.0;
  double weight = 0.95;

  static vector<double> rpyFromGyro_deg(3);
  vector<double> rpyFromAcc_deg(3);
  vector<double> rpy_deg(3);

  vector<double> gyroXyz_rps = {};
  gyroXyz_rps.push_back(msg->angular_velocity.x);
  gyroXyz_rps.push_back(-msg->angular_velocity.y);
  gyroXyz_rps.push_back(-msg->angular_velocity.z);

  vector<double> accXyz_mpss = {0.0, 0.0, 0.0};  // size: 3
  // vector<double> acc_mpss(3); // size: 3
  accXyz_mpss.at(0) = msg->linear_acceleration.x;
  accXyz_mpss.at(1) = msg->linear_acceleration.y;
  accXyz_mpss.at(2) = msg->linear_acceleration.z;

  vector<double> gyroXyz_dps = {0., 0., 0.};
  gyroXyz_dps.at(0) = gyroXyz_rps.at(0) * 180.0 / M_PI;
  gyroXyz_dps.at(1) = gyroXyz_rps.at(1) * 180.0 / M_PI;
  gyroXyz_dps.at(2) = gyroXyz_rps.at(2) * 180.0 / M_PI;

  curTime = msg->header.stamp.sec + msg->header.stamp.nsec / 1000000000.0;
  dt = curTime - prevTime;
  prevTime = curTime;

  rpyFromAcc_deg[0] = atan2(accXyz_mpss[1], accXyz_mpss[2]) * 180.0 / M_PI;
  rpyFromAcc_deg[1] =
      atan2(accXyz_mpss[0], std::sqrt((accXyz_mpss[1] * accXyz_mpss[1]) +
                                      (accXyz_mpss[2] * accXyz_mpss[2]))) *
      180.0 / M_PI;

  if (isFirstRun == true) {
    isFirstRun = false;
  } else {
    rpyFromGyro_deg[0] = rpyFromGyro_deg[0] + gyroXyz_dps[0] * dt;
    rpyFromGyro_deg[1] = rpyFromGyro_deg[1] + gyroXyz_dps[1] * dt;
  }

  rpy_deg[0] = weight * rpyFromAcc_deg[0] + (1 - weight) * rpyFromGyro_deg[0];
  rpy_deg[1] = weight * rpyFromAcc_deg[1] + (1 - weight) * rpyFromGyro_deg[1];

  // ROS_INFO("[dt] %lf", dt);

  ROS_INFO("[roll gyro, acc, cf] %lf %lf %lf", rpyFromGyro_deg[0],
           rpyFromAcc_deg[0], rpy_deg[0]);
  ROS_INFO("[pitch gyro, acc, cf] %lf %lf %lf", rpyFromGyro_deg[1],
           rpyFromAcc_deg[1], rpy_deg[1]);

  // ROS_INFO("[angular_velocity] %lf %lf %lf ", msg->angular_velocity.x,
  // msg->angular_velocity.y, msg->angular_velocity.z);
  // ROS_INFO("[linear_acceleration] %lf %lf %lf ", msg->linear_acceleration.x,
  // msg->linear_acceleration.y, msg->linear_acceleration.z);
}

// void dataCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
//   ROS_INFO("[%d, %d, %d, %d]", msg->data[0], msg->data[1], msg->data[2],
//   msg->data[3]);
// //   ROS_INFO("{}", msg->data);
// }

int main(int argc, char** argv) {
  ros::init(argc, argv, "CF_node");
  ros::NodeHandle n;
  ros::Subscriber imu_sub = n.subscribe("imu/data_raw", 1000, imuCallback);
  ros::spin();
  return 0;
}