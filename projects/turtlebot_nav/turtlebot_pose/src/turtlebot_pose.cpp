#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#include <math.h>

#define PI              3.14159265359
#define DEG_TO_RAD(deg) ((deg)*PI/180.0)
#define RAD_TO_DEG(rad) ((rad)*180.0/PI)

geometry_msgs::Pose2D current_pose;
ros::Publisher pub_pose2d;

// convert degrees and in range [0 to 360]; So -PI/2 becomes 270 degrees
float convert0to360(float rad) {
  float deg = RAD_TO_DEG(rad);
  if(deg < 0) {
    deg += 360.0;
  }
  return deg;
}

// odom callback
void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    // linear position
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;

    // quaternion to RPY conversion
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // angular position
    current_pose.theta = convert0to360(yaw);
    pub_pose2d.publish(current_pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot_pose_node");
    ros::NodeHandle n;
    ros::Subscriber sub_odometry = n.subscribe("odom", 1, odomCallback);
    pub_pose2d = n.advertise<geometry_msgs::Pose2D>("turtlebot_pose2d",1000);

    ros::spin();

    return 0;
}
