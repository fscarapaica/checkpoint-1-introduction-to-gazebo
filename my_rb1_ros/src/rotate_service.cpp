#include "geometry_msgs/Twist.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/init.h"
#include "ros/ros.h"
#include "tf/tf.h"

const double PI = 3.141592653589793238463;
int degree = INT_MAX;
int slowdownTarget = 0;
int target = 0;

void odom_sub_callback(const nav_msgs::Odometry::ConstPtr &odomMsg) {
  tf::Quaternion q(
      odomMsg->pose.pose.orientation.x, odomMsg->pose.pose.orientation.y,
      odomMsg->pose.pose.orientation.z, odomMsg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  degree = std::round((yaw * 180.0) / PI);
}

void normalizeDegrees(int &targetDegree) {
  if (targetDegree > 360) {
    targetDegree -= 360;
    normalizeDegrees(targetDegree);
  } else if (targetDegree > 180) {
    targetDegree = targetDegree - 360;
  }
}

void calculateTargetRotation(int &targetDegree) {
  int targetPN = -1;
  if (targetDegree >= 0)
    targetPN = 1;
  targetDegree = abs(targetDegree);
  normalizeDegrees(targetDegree);
  targetDegree *= targetPN;
}

bool service_callback(my_rb1_ros::Rotate::Request &req,
                      my_rb1_ros::Rotate::Response &res) {
  ROS_INFO("Service Called: Rotate %d Degrees", req.degrees);

  /* set_up */
  ros::NodeHandle nh;
  ros::Publisher pub_cmd_vel =
      nh.advertise<geometry_msgs::Twist>("cmd_vel", 1001);
  ros::Subscriber sub = nh.subscribe("odom", 100, odom_sub_callback);

  // loads odom callback
  // makes sure degree has relevant information
  degree = INT_MAX;
  while (degree == INT_MAX) {
    ros::spinOnce();
  }

  // calculate target
  target = req.degrees + degree;
  slowdownTarget = target;
  calculateTargetRotation(target);

  /* service implementation */

  // Rotation velocity
  geometry_msgs::Twist rotate;
  rotate.angular.z = 0.4;
  // Check if we need to go opposite direction
  int pn = 1;
  if (req.degrees < 0) {
    pn = -1;
    rotate.angular.z *= pn;
    slowdownTarget += 5;
  } else {
    slowdownTarget -= 5;
  }
  calculateTargetRotation(slowdownTarget);

  ros::Rate loop_rate(50);
  while (ros::ok()) {
    if (target == degree)
      break;
    if (slowdownTarget == degree)
      rotate.angular.z = 0.04 * pn;
    pub_cmd_vel.publish(rotate);
    ros::spinOnce();
    loop_rate.sleep();
  }

  // stop rotation
  int i = 0;
  rotate.angular.z = 0.0;
  while (i < 4) {
    pub_cmd_vel.publish(rotate);
    ros::spinOnce();
    loop_rate.sleep();
    i++;
  }

  /* tear_down */
  sub.shutdown();
  pub_cmd_vel.shutdown();
  res.result = "/rotate_robot service successful";
  ROS_INFO("Service called complete");
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "service_server");
  ros::NodeHandle nh;
  ROS_INFO("Service Ready: /rotate_robot");
  ros::ServiceServer my_service =
      nh.advertiseService("/rotate_robot", service_callback);
  ros::spin();

  return 0;
}