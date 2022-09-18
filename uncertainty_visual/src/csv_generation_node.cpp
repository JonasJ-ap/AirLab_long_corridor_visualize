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

double frequency = 5;
Eigen::Vector3d rc2_prev_pt;
double small_offset = 1;

void super_odom_stat_callback(
    const super_odometry_msgs::OptimizationStats::ConstPtr &msg,
    const nav_msgs::Odometry::ConstPtr &odom) {
  
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "uncertainty_visual_node");
  ros::NodeHandle nh;

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