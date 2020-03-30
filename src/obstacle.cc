#include <jackal/navigation.h>
#include <math.h>

using namespace std;

obstacleAvoidance::obstacleAvoidance():
  jackalNavigation(),
  nh(),
  cloud(new pcl::PointCloud<pcl::PointXYZ>),
  obstaclePoints(new pcl::PointCloud<pcl::PointXYZ>),
  coefficients(new pcl::ModelCoefficients),
  inliers(new pcl::PointIndices),
  tree(new pcl::search::KdTree<pcl::PointXYZ>)
  {
    pointCloudSub = nh.subscribe("/velodyne_points", 1, &obstacleAvoidance::pointcloudCb, this);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
  }

obstacleAvoidance::~obstacleAvoidance(){}

void obstacleAvoidance::pointcloudCb(const sensor_msgs::PointCloud2::ConstPtr& scan){
  pcl::fromROSMsg(*scan, *cloud);
}

void obstacleAvoidance::identifyObstaclePose(){
  pcl::PointCloud<pcl::PointXYZ>::Ptr clusterInput(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
  // Segment the incoming point cloud
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  // Points not in the inliers ==> Part of the obstacle/s
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  // Extract inliers (ground cloud)
  extract.setNegative(false);
  extract.filter(*groundCloud);
  // Extract outliers (Non ground cloud)
  extract.setNegative(true);
  extract.filter(*obstaclePoints);

  // Check if it's a valid incoming point cloud
  if(obstaclePoints->width > 0 && obstaclePoints->height > 0){
    vector<int> indices;
    pcl::removeNaNFromPointCloud(*obstaclePoints, *clusterInput, indices);

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*obstaclePoints, minPt, maxPt);

    if(isInPath(minPt, maxPt)){
      /* Steps to avoid the obstacle/s
      1) Find all the possible next waypoints from current Pose
      2) Assign weights based on distance to goal from each of these waypoints
      3) Choose the waypoint with least weight
      */

      // Step 1 ==> Find all possible waypoints
      geometry_msgs::Pose nextPossiblePose;

      nextPossiblePose.position.x = minPt.x;
      nextPossiblePose.position.y = maxPt.y;
      double rho1 = findDistanceToGoal(nextPossiblePose);

      nextPossiblePose.position.x = maxPt.x;
      nextPossiblePose.position.y = minPt.y;
      double rho2 = findDistanceToGoal(nextPossiblePose);

      if(rho1 < rho2){
        // Assign waypoints with a 1m distance from the obstacle edge
        waypointPose.position.x = minPt.x - 1;
        waypointPose.position.y = maxPt.y + 1;
        waypointPose.position.z = currentPose.position.z;
        // Align yaw
        double yaw = atan2((waypointPose.position.y - currentPose.position.y),
                           (waypointPose.position.x - currentPose.position.x));
        waypointPose.orientation = euler_to_quaterion(double(0), double(0), yaw);
      }
      else{
        waypointPose.position.x = maxPt.x + 1;
        waypointPose.position.y = minPt.y - 1;
        waypointPose.position.z = currentPose.position.z;
        // Align yaw
        double yaw = atan2((waypointPose.position.y - currentPose.position.y),
                           (waypointPose.position.x - currentPose.position.x));
        waypointPose.orientation = euler_to_quaterion(double(0), double(0), yaw);
      }
    }
    // the waypoint given wrt to the current pose of the jackal
    else{
      waypointPose = finalPose;
      // Theta is the yaw.z value
      /*geometry_msgs::Vector3 yaw = quaternion_to_euler(currentPose.orientation);
      double theta = yaw.z;

      waypointPose.position.x = currentPose.position.x*cos(yaw.z)*finalPose.position.x -
                                currentPose.position.y*sin(yaw.z)*finalPose.position.y;

      waypointPose.position.y = currentPose.position.x*sin(yaw.z)*finalPose.position.x +
                                currentPose.position.y*cos(yaw.z)*finalPose.position.y;*/
    }
  }
}

double obstacleAvoidance::findDistanceToGoal(geometry_msgs::Pose nextPossiblePose){
  double delta_x = finalPose.position.x - nextPossiblePose.position.x;
  double delta_y = finalPose.position.y - nextPossiblePose.position.y;

  return sqrt(delta_x*delta_x + delta_y*delta_y);
}

bool obstacleAvoidance::isInPath(pcl::PointXYZ minPt, pcl::PointXYZ maxPt){
  // Goal orientation orientation wrt to current jackal pose
  double theta = atan2((finalPose.position.y-currentPose.position.y),
                      (finalPose.position.x-currentPose.position.x));

  double alpha_1 = atan2((maxPt.y-currentPose.position.y), (maxPt.x-currentPose.position.x));
  double alpha_2 = atan2((minPt.y-currentPose.position.y), (maxPt.x-currentPose.position.x));

  // Normalize the angles from the range [-180,180] to [-360,360]
  double alpha_1_normalized = normalize(alpha_1*180/M_PI);
  double alpha_2_normalized = normalize(alpha_2*180/M_PI);
  double theta_normalized = normalize(theta*180/M_PI);

  if(theta_normalized < alpha_2_normalized && theta_normalized > alpha_1_normalized && alpha_2_normalized > alpha_1_normalized) return true;
  else if(theta_normalized > alpha_2_normalized && theta_normalized < alpha_1_normalized && alpha_2_normalized < alpha_1_normalized) return true;
  else return false;
}
