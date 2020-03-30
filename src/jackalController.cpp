/*
Waypoint based motion planning
Done using bicycle model of the robot
*/

#include <stdlib.h>
#include <boost/make_shared.hpp>
#include <math.h>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>

using namespace std;

geometry_msgs::Vector3 quaternion_to_euler(geometry_msgs::Quaternion q){
	geometry_msgs::Vector3 euler_angles;
	double sinr = 2*(q.w*q.x + q.y*q.z);
	double cosr = 1-2*(q.x*q.x + q.y*q.y);
	euler_angles.x = atan2(sinr, cosr);

	double sinp = 2 * (q.w*q.y - q.z*q.x);
	if(abs(sinp)>=1) euler_angles.y = copysign(M_PI/2,sinp);
	else euler_angles.y = asin(sinp);

	double siny = 2*(q.w*q.z + q.x*q.y);
  double cosy = 1 - 2*(q.y*q.y + q.z*q.z);
  euler_angles.z = atan2(siny, cosy);

	return euler_angles;
}

class jackalController{
public:
  ros::NodeHandle nh;
  ros::Publisher velocityPub, pclPub;
  ros::Subscriber pointCloudSub, odomSub;

  sensor_msgs::PointCloud cloud;
  geometry_msgs::Pose currentPose, home;
  geometry_msgs::Twist vel;
  geometry_msgs::Vector3 theta;
  bool homeSet;
	double delta_x, delta_y, rho, steering_angle;

  jackalController():nh(){
    velocityPub = nh.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel",1, false);
    pclPub = nh.advertise<sensor_msgs::PointCloud>("/cloud", 1, false);
    pointCloudSub = nh.subscribe("/velodyne_points", 1, &jackalController::pointcloudCb, this);
    odomSub = nh.subscribe("/odometry/filtered", 1, &jackalController::odomCb, this);

    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = 0;
		rho = 0;
		steering_angle = 0;
		homeSet = false;
  }

  void pointcloudCb(const sensor_msgs::PointCloud2::ConstPtr& scan){
    try{
      sensor_msgs::convertPointCloud2ToPointCloud(*scan, cloud);
    }
    catch(ros::Exception& e){
      ROS_WARN("Hold on!! No point cloud due to %s", e.what());
    }
  }

  void odomCb(const nav_msgs::Odometry::ConstPtr& msg){
    currentPose = msg->pose.pose;
		// Setting home pose
		if(!homeSet){
			home = currentPose;
			homeSet = true;
		}
  }

  geometry_msgs::Pose getNextPose(){
		geometry_msgs::Pose nextPose;
		vector<geometry_msgs::Point32> poi;
		geometry_msgs::Point32 median;

		if(!cloud.points.empty()) {
			for(int i=0; i<cloud.points.size(); i++){
				if(cloud.points[i].z > 0) poi.push_back(cloud.points[i]);
			}
			// Compute median
			if(poi.size()%2) median = poi[poi.size()/2];
			else{
				median.x = (poi[poi.size()/2].x + poi[(poi.size()/2)+1].x)/2;
				median.y = (poi[poi.size()/2].y + poi[(poi.size()/2)+1].y)/2;
				median.z = (poi[poi.size()/2].z + poi[(poi.size()/2)+1].z)/2;
			}
		}
	}

  void navigate(geometry_msgs::Pose nextPose){
    theta = quaternion_to_euler(currentPose.orientation);
    delta_x = nextPose.position.x - currentPose.position.x;
    delta_y = nextPose.position.y - currentPose.position.y;
    rho = sqrt(delta_x*delta_x + delta_y*delta_y);
		steering_angle = 0; // Steering angle in degrees

		// Turn in-place
		if(abs(delta_x) < 0.5 && abs(delta_y) > 0.1){
			// The two steering angles assigned below are in degrees
			if(delta_y > 0) steering_angle = 90;
			else if(delta_y < 0) steering_angle = 270;

			while (steering_angle*M_PI/180 - theta.z < 0.1) {
				ros::spinOnce();
				theta = quaternion_to_euler(currentPose.orientation);
				vel.linear.x = 0;
				vel.angular.z = 0.5*(steering_angle*M_PI/180 - theta.z);
			}

			vel.linear.x = 0;
			vel.angular.z = 0;
			steering_angle = 0;
			rho = 0;
			velocityPub.publish(vel);
			return;
		}

		// Travel an arc
    while(rho > 0.1){
			ros::spinOnce();
			delta_x = nextPose.position.x - currentPose.position.x;
	    delta_y = nextPose.position.y - currentPose.position.y;
	    rho = sqrt(delta_x*delta_x + delta_y*delta_y);

      vel.linear.x = 0.3*rho;
  		steering_angle = atan2(delta_y,delta_x); // Returned as radians ==> No need of conversion
  		vel.angular.z = 0.3*(steering_angle - theta.z);
      velocityPub.publish(vel);
    }

    vel.linear.x = 0;
    vel.angular.z = 0;
		rho = 0;
		steering_angle = 0;
    velocityPub.publish(vel);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "jackalNode");
  ros::NodeHandle nh;
  ros::Rate loop_rate(0.2);
	// Sleep for complete initialization (point clouds, odometry, etc)
	ros::Duration(0.5).sleep();

  jackalController *robot = new jackalController;
  geometry_msgs::Pose nextPose = robot->currentPose;
	nextPose.position.x = nextPose.position.x + 3;

  while(ros::ok()){
    ros::spinOnce();
		//if(robot->rho == 0 && robot->steering_angle == 0) nextPose = robot->getNextPose();
		robot->navigate(nextPose);
		break;
  }
  return 0;
}
