#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "super_odometry_msgs/OptimizationStats.h"
#include <jsk_rviz_plugins/OverlayText.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include "visualization_msgs/MarkerArray.h"

ros::Publisher uncertainty_x_pub;
ros::Publisher uncertainty_y_pub;
ros::Publisher uncertainty_z_pub;
ros::Publisher uncertainty_roll_pub;
ros::Publisher uncertainty_pitch_pub;
ros::Publisher uncertainty_yaw_pub;
ros::Publisher pose_cov_pub;
ros::Publisher uncertainty_shape_pub;
ros::Publisher rc2_speed_pub;
ros::Publisher prediction_pub;
ros::Publisher constraint_pub;
ros::Publisher uncertainties_pub;
double frequency = 5;
Eigen::Vector3d rc2_prev_pt;
double small_offset = 1;
visualization_msgs::MarkerArray uncertainty_shapes;
int count = 0;
long last_publish_time = 0;
float min_speed = 0.3;

void super_odom_stat_callback(
    const super_odometry_msgs::OptimizationStats::ConstPtr &msg,
    const nav_msgs::Odometry::ConstPtr &odom) {
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

  // update speed
  Eigen::Vector3d rc2_pt;
  rc2_pt(0) = odom->pose.pose.position.x;
  rc2_pt(1) = odom->pose.pose.position.y;
  rc2_pt(2) = odom->pose.pose.position.z;

  std_msgs::Float32 rc2_speed;
  rc2_speed.data = (rc2_pt - rc2_prev_pt).norm() / (1 / frequency);
  rc2_prev_pt(0) = rc2_pt(0);
  rc2_prev_pt(1) = rc2_pt(1);
  rc2_prev_pt(2) = rc2_pt(2);
  rc2_speed_pub.publish(rc2_speed);

  // update uncertainty shape
  visualization_msgs::Marker marker;
  marker.header.frame_id = "cmu_rc2_sensor_init";
  marker.header.stamp = odom->header.stamp;
  marker.ns = "uncertainty_x" + std::to_string(count);
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  // apply 90 degree rotation on odom's pitch
  Eigen::Quaterniond q;
  q.x() = odom->pose.pose.orientation.x;
  q.y() = odom->pose.pose.orientation.y;
  q.z() = odom->pose.pose.orientation.z;
  q.w() = odom->pose.pose.orientation.w;
  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Matrix3d R90;
  R90 << 1, 0, 0, 0, 0, -1, 0, 1, 0;
  Eigen::Matrix3d R_new = R90 * R;
  Eigen::Quaterniond q_new(R_new);
  


  marker.pose.position.x = odom->pose.pose.position.x;
  marker.pose.position.y = odom->pose.pose.position.y;
  marker.pose.position.z = odom->pose.pose.position.z;
  marker.pose.orientation.x = odom->pose.pose.orientation.x;
  marker.pose.orientation.y = odom->pose.pose.orientation.y;
  marker.pose.orientation.z = odom->pose.pose.orientation.z;
  marker.pose.orientation.w = odom->pose.pose.orientation.w;
  marker.scale.x = (1 - msg->uncertainty_x) * 2.5 + small_offset;
  marker.scale.y = 1;
  marker.scale.z = 1;

  marker.color.a = 0.1;
  if (msg->uncertainty_x < 0.2) {
    marker.color.a = 0.7;
  }
  marker.color.r = 242.0 / 255.0;
  marker.color.g = 75.0 / 255.0;
  marker.color.b = 231.0 / 255.0;
  // marker.color.r = 1.0;
  // marker.color.g = 0.0;
  // marker.color.b = 0.0;
  marker.lifetime = ros::Duration(500);

  // if (((long) odom->header.stamp.toSec()) % 5  == 0) {
  //   if (((long) odom->header.stamp.toSec()) > last_publish_time) {
  //     uncertainty_shapes.markers.push_back(marker);
  //     last_publish_time = (long) odom->header.stamp.toSec();
  //     count += 1;
  //   }
  // }
  if (rc2_speed.data > min_speed) {
    uncertainty_shapes.markers.push_back(marker);
    count += 1;
  }
  uncertainties_pub.publish(uncertainty_shapes);
  // uncertainty_shape_pub.publish(marker);
  // uncertainty_shape_pub.publish(marker2);
  // uncertainty_shape_pub.publish(marker3);
}

void message_callback(const std_msgs::StringConstPtr &msg) {
  jsk_rviz_plugins::OverlayText msg_overlay;
  if (msg->data == "IMU Prediction Constrained by Laser") {
    msg_overlay.text = "by Laser";
    msg_overlay.fg_color.r = 0.0;
    msg_overlay.fg_color.g = 1.0;
    msg_overlay.fg_color.b = 0.0;
    msg_overlay.fg_color.a = 1.0;
  } else if (msg->data == "IMU Prediction Constrained by Visual") {
    msg_overlay.text = "by Visual";
    msg_overlay.fg_color.r = 1.0;
    msg_overlay.fg_color.g = 0.0;
    msg_overlay.fg_color.b = 1.0;
    msg_overlay.fg_color.a = 1.0;
  } else {
    msg_overlay.text = "No Match";
  }
  msg_overlay.bg_color.a = 0.0;
  msg_overlay.font = "DejaVu Sans Mono";
  constraint_pub.publish(msg_overlay);
  std_msgs::String msg_string;
  msg_string.data = "IMU Prediction Constrained ";
  prediction_pub.publish(msg_string);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "uncertainty_visual_node");
  ros::NodeHandle nh;
  uncertainty_x_pub = nh.advertise<std_msgs::Float32>("/uncertainty_x", 10);
  uncertainty_y_pub = nh.advertise<std_msgs::Float32>("/uncertainty_y", 10);
  uncertainty_z_pub = nh.advertise<std_msgs::Float32>("/uncertainty_z", 10);
  uncertainty_roll_pub =
      nh.advertise<std_msgs::Float32>("/uncertainty_roll", 10);
  uncertainty_pitch_pub =
      nh.advertise<std_msgs::Float32>("/uncertainty_pitch", 10);
  uncertainty_yaw_pub = nh.advertise<std_msgs::Float32>("/uncertainty_yaw", 10);
  pose_cov_pub =
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_cov", 10);
  rc2_speed_pub = nh.advertise<std_msgs::Float32>("/cmu_rc2/speed_custom", 10);
  uncertainty_shape_pub =
      nh.advertise<visualization_msgs::Marker>("/uncertainty_shape", 10);
  prediction_pub = nh.advertise<std_msgs::String>("/prediction", 10);
  constraint_pub =
      nh.advertise<jsk_rviz_plugins::OverlayText>("/constraint", 10);
  uncertainties_pub = nh.advertise<visualization_msgs::MarkerArray>("uncertainties", 10);
  // ros::Subscriber super_stats_sub =
  // nh.subscribe("/cmu_rc2/super_odometry_stats", 10,
  // &super_odom_stat_callback);
  ros::Subscriber prediction_sub =
      nh.subscribe("/cmu_rc2/prediction_source", 10, &message_callback);
  message_filters::Subscriber<super_odometry_msgs::OptimizationStats> stats_sub(
      nh, "/cmu_rc2/super_odometry_stats", 10);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(
      nh, "/cmu_rc2/aft_mapped_to_init_imu", 10);

  message_filters::TimeSynchronizer<super_odometry_msgs::OptimizationStats,
                                    nav_msgs::Odometry>
      sync(stats_sub, odom_sub, 10);
  sync.registerCallback(boost::bind(&super_odom_stat_callback, _1, _2));
  ros::spin();
}