#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdlib.h>
#include <boost/make_shared.hpp>
#include <math.h>
#include <vector>
#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

// PCL include files
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

class jackalNavigation{
protected:
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  move_base_msgs::MoveBaseGoal goalPose;

  ros::NodeHandle nh;
  ros::Subscriber odomSub;

public:
  geometry_msgs::Pose currentPose;

  jackalNavigation();
  void odomCb(const nav_msgs::Odometry::ConstPtr& odom);
  void moveToGoal(geometry_msgs::Pose goal);
};

class obstacleAvoidance : public jackalNavigation{
protected:
  ros::NodeHandle nh;
  ros::Subscriber pointCloudSub;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, obstaclePoints;
  pcl::ModelCoefficients::Ptr coefficients;
  pcl::PointIndices::Ptr inliers;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::SACSegmentation<pcl::PointXYZ> seg;

public:
  geometry_msgs::Pose finalPose, waypointPose; // finalPose set from the driver code (navigation.cpp)

  obstacleAvoidance();
  ~obstacleAvoidance();
  void pointcloudCb(const sensor_msgs::PointCloud2::ConstPtr& scan);
  void identifyObstaclePose(); // Returns pose and dimensions of obstacle
  void calculateSafeZones(); // Calculates safe zones around the obstacle
  bool isInPath(pcl::PointXYZ minPt, pcl::PointXYZ maxPt);
  //vector<geometry_msgs::Pose> getPossibleWaypoints(pcl::PointXYZ minPt, pcl::PointXYZ maxPt);
  double findDistanceToGoal(geometry_msgs::Pose nextPossiblePose);
};

#endif
