#include <math.h>

#include "ros/ros.h"
// #include "std_msgs/String.h"
// #include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/Imu.h"

//    # rospy.loginfo(rospy.get_caller_id() + 'num_pub: [%f,%f,%f,%f]',
//    msg.data[0], msg.data[1], msg.data[2], msg.data[3]) # msg.data : list

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  static double prev_time = 0.0;
  static double roll_gyro_deg = 0.0;
  static double pitch_gyro_deg = 0.0;
  static bool isFirstRun = true;

  double cur_time = 0.0;
  double dt = 0.0;

  double weight = 0.95;
  double roll_deg = 0.0;
  double pitch_deg = 0.0;

  double roll_acc_deg = 0.0;
  double pitch_acc_deg = 0.0;

  double gyro_x_rad = msg->angular_velocity.x;
  double gyro_y_rad = msg->angular_velocity.y;

  double acc_x_mpss = msg->linear_acceleration.x;
  double acc_y_mpss = msg->linear_acceleration.y;
  double acc_z_mpss = msg->linear_acceleration.z;

  double gyro_x_deg = gyro_x_rad * 180.0 / M_PI;
  double gyro_y_deg = -gyro_y_rad * 180.0 / M_PI;

  cur_time = msg->header.stamp.sec + msg->header.stamp.nsec / 1000000000.0;
  dt = cur_time - prev_time;
  prev_time = cur_time;

  roll_acc_deg = atan2(acc_y_mpss, acc_z_mpss) * 180.0 / M_PI;
  pitch_acc_deg = atan2(acc_x_mpss, std::sqrt((acc_y_mpss * acc_y_mpss) +
                                              (acc_z_mpss * acc_z_mpss))) *
                  180.0 / M_PI;

  if (isFirstRun == true) {
    isFirstRun = false;
  } else {
    roll_gyro_deg = roll_gyro_deg + gyro_x_deg * dt;
    pitch_gyro_deg = pitch_gyro_deg + gyro_y_deg * dt;
  }

  roll_deg = weight * roll_acc_deg + (1 - weight) * roll_gyro_deg;
  pitch_deg = weight * pitch_acc_deg + (1 - weight) * pitch_gyro_deg;

  // ROS_INFO("[dt] %lf", dt);
  // ROS_INFO("[roll_gyro_deg] %lf", roll_gyro_deg);

  ROS_INFO("[roll gyro, acc, cf] %lf %lf %lf", roll_gyro_deg, roll_acc_deg,
           roll_deg);
  ROS_INFO("[pitch gyro, acc, cf] %lf %lf %lf", pitch_gyro_deg, pitch_acc_deg,
           pitch_deg);

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