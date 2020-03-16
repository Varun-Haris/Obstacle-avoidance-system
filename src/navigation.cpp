#include <jackal/navigation.h>

using namespace std;

jackalNavigation::jackalNavigation():nh(){
  odomSub = nh.subscribe("/odometry/filtered", 1, &jackalNavigation::odomCb, this);
}

void jackalNavigation::odomCb(const nav_msgs::Odometry::ConstPtr& odom){
  currentPose = odom->pose.pose;
}

// Must give all waypoints relative to the current jackal pose (considered as (0,0,0) for the action server)
void jackalNavigation::moveToGoal(geometry_msgs::Pose goal){
  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO_STREAM("Waiting for server");
  }

  goalPose.target_pose.header.frame_id = "base_link";
  goalPose.target_pose.header.stamp = ros::Time::now();
  goalPose.target_pose.pose = goal;

  ac.sendGoal(goalPose);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) ROS_INFO_STREAM("Moved !!");
  else ROS_INFO_STREAM("Couldn't reach goal");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "jackalNode");
  ros::NodeHandle nh;
	// Sleep for complete initialization (point clouds, odometry, etc)
	ros::Duration(0.5).sleep();

  //jackalNavigation *robot = new jackalNavigation;
  obstacleAvoidance *sys = new obstacleAvoidance;
  geometry_msgs::Pose goal;
  float x,y;
  string str;

  cout << "Enter final goal x y: ";
  cin >> x >> y ;
  sys->finalPose.position.x = x;
  sys->finalPose.position.y = y;
  sys->finalPose = sys->currentPose;

  while(ros::ok()){
    ros::spinOnce();
    //cout << sys->currentPose << endl;
    if(sys->finalPose.orientation.w > 0.985 ){
      double delta_x = sys->finalPose.position.x - sys->currentPose.position.x;
      double delta_y = sys->finalPose.position.y - sys->currentPose.position.y;
      // 1 cm threshold for the jackal
      if(sqrt(delta_x*delta_x + delta_y*delta_y) > 0.01){
          sys->identifyObstaclePose();
          cout << sys->waypointPose << endl;
          cout << "Go ahead? : ";
          cin >> str;
          if(str == "yes" || str == "y") sys->moveToGoal(sys->waypointPose);
          else cout << "Not appropriate waypoint" << endl;
      }
      else return 0;
    }
    else{
      ROS_INFO_STREAM("Hold on !!");
      sys->finalPose = sys->currentPose;
      sys->finalPose.position.x = x;
      sys->finalPose.position.y = y;
    }
  }
}
