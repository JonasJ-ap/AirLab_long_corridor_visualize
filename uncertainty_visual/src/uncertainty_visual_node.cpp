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
float move_since_last_mark = 0;

std::string system_id;

float min_speed;
float marker_dist_interval;
float marker_size;

double color1_r;
double color1_g;
double color1_b;
double color1_a;
double color2_r;
double color2_g;
double color2_b;
double color2_a;

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
  move_since_last_mark += (rc2_pt - rc2_prev_pt).norm();
  
  rc2_prev_pt(0) = rc2_pt(0);
  rc2_prev_pt(1) = rc2_pt(1);
  rc2_prev_pt(2) = rc2_pt(2);
  rc2_speed_pub.publish(rc2_speed);

  std::string system_prefix2;
  if (system_id == "") {
    system_prefix2 = "";
  } else {
    system_prefix2 = system_id + "_";
  }

  // update uncertainty shape
  visualization_msgs::Marker marker;
  marker.header.frame_id = system_prefix2 + "sensor_init";
  marker.header.stamp = odom->header.stamp;
  marker.ns = "uncertainty_x" + std::to_string(count);
  count += 1;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = odom->pose.pose.position.x;
  marker.pose.position.y = odom->pose.pose.position.y;
  marker.pose.position.z = odom->pose.pose.position.z;
  marker.pose.orientation.x = odom->pose.pose.orientation.x;
  marker.pose.orientation.y = odom->pose.pose.orientation.y;
  marker.pose.orientation.z = odom->pose.pose.orientation.z;
  marker.pose.orientation.w = odom->pose.pose.orientation.w;
  marker.scale.x = marker_size;
  marker.scale.y = marker_size;
  marker.scale.z = marker_size;

  marker.color.a = color1_a;
  marker.color.r = color1_r/ 255.0;
  marker.color.g = color1_g / 255.0;
  marker.color.b = color1_b / 255.0;
  // marker.color.r = 1.0;
  // marker.color.g = 0.0;
  // marker.color.b = 0.0;
  marker.lifetime = ros::Duration(500);


  visualization_msgs::Marker marker2;
  marker2.header = marker.header;
  marker2.ns = "uncertainty_y" + std::to_string(count);
  count += 1;
  marker2.id = 1;
  marker2.type = visualization_msgs::Marker::SPHERE;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.pose.position.x = odom->pose.pose.position.x;
  marker2.pose.position.y = odom->pose.pose.position.y;
  marker2.pose.position.z = odom->pose.pose.position.z;
  marker2.pose.orientation.x = odom->pose.pose.orientation.x;
  marker2.pose.orientation.y = odom->pose.pose.orientation.y;
  marker2.pose.orientation.z = odom->pose.pose.orientation.z;
  marker2.pose.orientation.w = odom->pose.pose.orientation.w;
  marker2.scale.x = marker_size;
  marker2.scale.y = marker_size;
  marker2.scale.z = marker_size;
  marker2.color.a = color2_a;
  marker2.color.r = color2_r/ 255.0;
  marker2.color.g = color2_g / 255.0;
  marker2.color.b = color2_b / 255.0;
  marker2.lifetime = ros::Duration(500);



  if (move_since_last_mark >= marker_dist_interval) {
     if (rc2_speed.data > min_speed) {
      if (msg->PredictionSource  == 0) {
        // laser
        uncertainty_shapes.markers.push_back(marker2);
      } else {
        // visual
        uncertainty_shapes.markers.push_back(marker);
      }
      move_since_last_mark = 0;
    } 
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
  nh.getParam("system_id", system_id);
  nh.getParam("marker_size", marker_size);
  nh.getParam("marker_dist_interval", marker_dist_interval);
  nh.getParam("min_speed", min_speed);
  nh.getParam("color1_r", color1_r);
  nh.getParam("color1_g", color1_g);
  nh.getParam("color1_b", color1_b);
  nh.getParam("color1_a", color1_a);
  nh.getParam("color2_r", color2_r);
  nh.getParam("color2_g", color2_g);
  nh.getParam("color2_b", color2_b);
  nh.getParam("color2_a", color2_a);
  std::string system_prefix;
  if (system_id == "") {
    system_prefix = "/";
  } else {
    system_prefix = system_id + "/";
  }

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
  rc2_speed_pub = nh.advertise<std_msgs::Float32>(system_prefix + "speed_custom", 10);
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
      nh.subscribe(system_prefix + "/prediction_source", 10, &message_callback);
  message_filters::Subscriber<super_odometry_msgs::OptimizationStats> stats_sub(
      nh, system_prefix + "super_odometry_stats", 10);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(
      nh, system_prefix + "aft_mapped_to_init_imu", 10);

  message_filters::TimeSynchronizer<super_odometry_msgs::OptimizationStats,
                                    nav_msgs::Odometry>
      sync(stats_sub, odom_sub, 10);
  sync.registerCallback(boost::bind(&super_odom_stat_callback, _1, _2));
  ros::spin();
}
