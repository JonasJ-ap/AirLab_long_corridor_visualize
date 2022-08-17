#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include "super_odometry_msgs/OptimizationStats.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

ros::Publisher uncertainty_x_pub;
ros::Publisher uncertainty_y_pub;
ros::Publisher uncertainty_z_pub;
ros::Publisher uncertainty_roll_pub;
ros::Publisher uncertainty_pitch_pub;
ros::Publisher uncertainty_yaw_pub;
ros::Publisher pose_cov_pub;

void super_odom_stat_callback(const super_odometry_msgs::OptimizationStats::ConstPtr& msg, const nav_msgs::Odometry::ConstPtr& odom) {
  std_msgs::Float32 x_msg;
  x_msg.data = msg->uncertainty_x;
  uncertainty_x_pub.publish(x_msg);
  
  std_msgs::Float32 y_msg;
  y_msg.data = msg->uncertainty_y;
  uncertainty_y_pub.publish(y_msg);
  
  std_msgs::Float32 z_msg;
  z_msg.data = msg->uncertainty_z;
  uncertainty_z_pub.publish(z_msg);
  
  std_msgs::Float32 roll_msg;
  roll_msg.data = msg->uncertainty_roll;
  uncertainty_roll_pub.publish(roll_msg);
  
  std_msgs::Float32 pitch_msg;
  pitch_msg.data = msg->uncertainty_pitch;
  uncertainty_pitch_pub.publish(pitch_msg);
  
  std_msgs::Float32 yaw_msg;
  yaw_msg.data = msg->uncertainty_yaw;
  uncertainty_yaw_pub.publish(yaw_msg);

  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header = odom->header;
  pose_msg.pose.pose.position.x = odom->pose.pose.position.x;
  pose_msg.pose.pose.position.y = odom->pose.pose.position.y;
  pose_msg.pose.pose.position.z = odom->pose.pose.position.z;
  pose_msg.pose.pose.orientation.x = odom->pose.pose.orientation.x;
  pose_msg.pose.pose.orientation.y = odom->pose.pose.orientation.y;
  pose_msg.pose.pose.orientation.z = odom->pose.pose.orientation.z;
  pose_msg.pose.pose.orientation.w = odom->pose.pose.orientation.w;
  pose_msg.pose.covariance[0] = msg->uncertainty_x;
  pose_msg.pose.covariance[7] = msg->uncertainty_y;
  pose_msg.pose.covariance[14] = msg->uncertainty_z;
  // pose_msg.pose.covariance[21] = msg->uncertainty_roll;
  // pose_msg.pose.covariance[28] = msg->uncertainty_pitch;
  // pose_msg.pose.covariance[35] = msg->uncertainty_yaw;
  
  pose_cov_pub.publish(pose_msg);

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "uncertainty_visual_node");
  ros::NodeHandle nh;
  uncertainty_x_pub = nh.advertise<std_msgs::Float32>("/uncertainty_x", 10);
  uncertainty_y_pub = nh.advertise<std_msgs::Float32>("/uncertainty_y", 10);
  uncertainty_z_pub = nh.advertise<std_msgs::Float32>("/uncertainty_z", 10);
  uncertainty_roll_pub = nh.advertise<std_msgs::Float32>("/uncertainty_roll", 10);
  uncertainty_pitch_pub = nh.advertise<std_msgs::Float32>("/uncertainty_pitch", 10);
  uncertainty_yaw_pub = nh.advertise<std_msgs::Float32>("/uncertainty_yaw", 10);
  pose_cov_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_cov", 10);
  // ros::Subscriber super_stats_sub = nh.subscribe("/cmu_rc2/super_odometry_stats", 10, &super_odom_stat_callback);
  message_filters::Subscriber<super_odometry_msgs::OptimizationStats> stats_sub(nh, "/cmu_rc2/super_odometry_stats", 10);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/cmu_rc2/aft_mapped_to_init_imu", 10);

  message_filters::TimeSynchronizer<super_odometry_msgs::OptimizationStats, nav_msgs::Odometry> sync(stats_sub, odom_sub, 10);
  sync.registerCallback(boost::bind(&super_odom_stat_callback, _1, _2));
  ros::spin();
}