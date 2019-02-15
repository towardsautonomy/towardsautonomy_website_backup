#include <ros/ros.h>
#include "geometry_msgs/TwistWithCovariance.h"
#include "nav_msgs/Odometry.h"
#include "gazebo_msgs/LinkState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "turtlebot_custom_msg/environment_msgs.h"

#include "pid.h"

#define PI                      3.14159265359
#define DEG_TO_RAD(deg)         ((deg)*PI/180.0)
#define RAD_TO_DEG(rad)         ((rad)*180.0/PI)
#define NS_TO_SEC(ns)           ((ns)/1000000000.0)
#define MAX_N_MISSED_DETECTION  3

bool initialPoseReceived = false;
geometry_msgs::Pose2D currentPose2d;
turtlebot_custom_msg::environment_msgs envObj;

/* theta PID params */
float theta_pid_kp, theta_pid_ki, theta_pid_kd;
PID theta_pid;

/* velocity PID params */
float vel_pid_kp, vel_pid_ki, vel_pid_kd;
PID velocity_pid;

/* Set destination params */
bool initialObjsReceived = false;
bool leftWallFound, rightWallFound;
float leftXY[2], rightXY[2], destinationXY[2];
uint8_t n_detection_missed = 0;

/* Step */
uint8_t step = 0;
void pose2dCallback(const geometry_msgs::Pose2D& msg) {
  currentPose2d = msg;
  if(!initialPoseReceived) {
    initialPoseReceived = true;
  }
}

void envObjCallback(const turtlebot_custom_msg::environment_msgs& msg) {
  // Update the destination based on the environment only during 1st two steps */
  if(step <= 2) {
    envObj = msg;
    leftWallFound = false;
    rightWallFound = false;
    for(unsigned i=0; i<(unsigned)envObj.walls.size(); i++) {
      if(envObj.walls[i].direction == "left") {
        leftXY[0] = envObj.walls[i].upper_xy[0];
        leftXY[1] = envObj.walls[i].upper_xy[1];
        leftWallFound = true;
      }
      else if(envObj.walls[i].direction == "right") {
        rightXY[0] = envObj.walls[i].upper_xy[0];
        rightXY[1] = envObj.walls[i].upper_xy[1];
        rightWallFound = true;
      }
    }
    if(leftWallFound && rightWallFound) {
      // Setting the destination coordinate at the center of upper corner of both walls
      destinationXY[0] = (leftXY[0] + rightXY[0])/2.0;
      destinationXY[1] = (leftXY[1] + rightXY[1])/2.0;
      n_detection_missed = 0;
      if(! initialObjsReceived) {
        initialObjsReceived = true;
      }
      //ROS_INFO("Setting the destination relative coordinates to [%f, %f]", destinationXY[0], destinationXY[1]);
    }
    else {
      n_detection_missed++;
      if(n_detection_missed >= MAX_N_MISSED_DETECTION) {
        // Wall not detected, stop the robot
        destinationXY[0] = 0;
        destinationXY[1] = 0;
      }
    }
  }
}

int main(int argc, char **argv){

ros::init(argc,argv,"turtlebot_controller_node");
    // create private node handle
    ros::NodeHandle n("~");

    ros::Subscriber sub_pose2d = n.subscribe("/turtlebot_pose2d", 1000, pose2dCallback);
    ros::Subscriber sub_envObj = n.subscribe("/turtlebot_env", 1000, envObjCallback);
    ros::Publisher move_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);

    // Get params
    n.param("theta_Kp", theta_pid_kp, (float)DEFAULT_KP);
    n.param("theta_Ki", theta_pid_ki, (float)DEFAULT_KI);
    n.param("theta_Kd", theta_pid_kd, (float)DEFAULT_KD);
    n.param("velocity_Kp", vel_pid_kp, (float)DEFAULT_KP);
    n.param("velocity_Ki", vel_pid_ki, (float)DEFAULT_KI);
    n.param("velocity_Kd", vel_pid_kd, (float)DEFAULT_KD);

    // Init theta PID controller
    theta_pid.init(theta_pid_kp, theta_pid_ki, theta_pid_kd, -0.2, 0.2);
    // Init velocity PID controller
    velocity_pid.init(vel_pid_kp, vel_pid_ki, vel_pid_kd, 0.0, 0.3);

    ros::Rate rate(10);

    geometry_msgs::Pose2D initialPose;
    while(ros::ok() && !initialObjsReceived) {
      ros::spinOnce();
      rate.sleep();
    }
    initialPose = currentPose2d;

    // Move the robot till the end of wall
    float dist_to_dst, theta_to_dst, relative_theta;
    float prev_timestamp = 0;
    float linear_velocity, anglular_velocity;

    geometry_msgs::Twist move;
    // Step 1: Turn the robot to align with the goal
    step++;
    theta_pid.reset();
    relative_theta = PI;
    ROS_INFO("STEP 1: Aligning robot orientation towards the goal");
    while(ros::ok() && (fabs(relative_theta) > 0.015))
    {
      ros::Time now = ros::Time::now();
      float time_sec = now.sec + NS_TO_SEC(now.nsec);
      dist_to_dst = sqrt(pow(destinationXY[0], 2.0) + pow(destinationXY[1], 2.0));
      theta_to_dst = atan(destinationXY[1]/destinationXY[0]);
      if(theta_to_dst >= 0) relative_theta = (PI/2.0) - theta_to_dst;
      else relative_theta = -1.0*(theta_to_dst + (PI/2.0));
      if(prev_timestamp != 0) {
        float sample_time = time_sec - prev_timestamp;
        anglular_velocity = -1.0*theta_pid.step(relative_theta, sample_time);
        // linear velocity
        move.linear.x = 0;
        // anglular velocity
        move.angular.z = anglular_velocity;
        move_pub.publish(move);
      }
      prev_timestamp = time_sec;
      ros::spinOnce();
      rate.sleep();
    }

    // Step 2: Move robot to the goal
    step++;
    theta_pid.reset();
    velocity_pid.reset();
    dist_to_dst = 100.0;
    ROS_INFO("STEP 2: Moving towards the goal");
    while(ros::ok() && (dist_to_dst > 0.1))
    {
      ros::Time now = ros::Time::now();
      float time_sec = now.sec + NS_TO_SEC(now.nsec);
      dist_to_dst = sqrt(pow(destinationXY[0], 2.0) + pow(destinationXY[1], 2.0));
      theta_to_dst = atan(destinationXY[1]/destinationXY[0]);
      if(theta_to_dst >= 0) relative_theta = (PI/2.0) - theta_to_dst;
      else relative_theta = -1.0*(theta_to_dst + (PI/2.0));
      if(prev_timestamp != 0) {
        float sample_time = time_sec - prev_timestamp;
        anglular_velocity = -1.0*theta_pid.step(relative_theta, sample_time);
        linear_velocity = velocity_pid.step(dist_to_dst, sample_time);
        // linear velocity
        move.linear.x = linear_velocity;
        // anglular velocity
        move.angular.z = anglular_velocity;
        move_pub.publish(move);
      }
      prev_timestamp = time_sec;
      ros::spinOnce();
      rate.sleep();
    }

    // Step 3: Turn robot towards its initial pose
    step++;
    destinationXY[1] = currentPose2d.x - initialPose.x;
    destinationXY[0] = -1.0*(currentPose2d.y - initialPose.y);
    theta_pid.reset();
    theta_to_dst = RAD_TO_DEG(atan(destinationXY[1]/destinationXY[0]));
    if(currentPose2d.theta <= 90) {
      if(theta_to_dst < 0)
        relative_theta = -1.0*(270.0 - currentPose2d.theta + theta_to_dst);
      else
        relative_theta = -1.0*(90.0 - currentPose2d.theta + theta_to_dst);
    }
    else {
      relative_theta = (currentPose2d.theta - 90.0 + theta_to_dst);
    }
    float desired_theta = DEG_TO_RAD(currentPose2d.theta - relative_theta);

    ROS_INFO("STEP 3: Turning towards the initial pose; turn angle = %f", relative_theta);
    float theta_err = PI;
    while(ros::ok() && (fabs(theta_err) > 0.1))
    {
      ros::Time now = ros::Time::now();
      float time_sec = now.sec + NS_TO_SEC(now.nsec);

      theta_err = DEG_TO_RAD(currentPose2d.theta) - desired_theta;
      if(prev_timestamp != 0) {
        float sample_time = time_sec - prev_timestamp;
        anglular_velocity = -1.0*theta_pid.step(theta_err, sample_time);
        // linear velocity
        move.linear.x = 0;
        // anglular velocity
        move.angular.z = anglular_velocity;
        move_pub.publish(move);
      }
      prev_timestamp = time_sec;
      ros::spinOnce();
      rate.sleep();
    }

    // Step 4: Move robot to the initial position
    step++;
    theta_pid.reset();
    velocity_pid.reset();
    dist_to_dst = 100.0;
    ROS_INFO("STEP 4: Moving towards the initial pose");
    while(ros::ok() && (dist_to_dst > 0.1))
    {
      ros::Time now = ros::Time::now();
      float time_sec = now.sec + NS_TO_SEC(now.nsec);
      destinationXY[1] = currentPose2d.x - initialPose.x;
      destinationXY[0] = -1.0*(currentPose2d.y - initialPose.y);
      dist_to_dst = sqrt(pow(destinationXY[0], 2.0) + pow(destinationXY[1], 2.0));
      theta_to_dst = atan(destinationXY[1]/destinationXY[0]);
      if(theta_to_dst >= 0) relative_theta = (PI/2.0) - theta_to_dst;
      else relative_theta = -1.0*(theta_to_dst + (PI/2.0));
      if(prev_timestamp != 0) {
        float sample_time = time_sec - prev_timestamp;
        anglular_velocity = -1.0*theta_pid.step(relative_theta, sample_time);
        linear_velocity = velocity_pid.step(dist_to_dst, sample_time);
        // linear velocity
        move.linear.x = linear_velocity;
        // anglular velocity
        move.angular.z = anglular_velocity;
        move_pub.publish(move);
      }
      prev_timestamp = time_sec;
      ros::spinOnce();
      rate.sleep();
    }

    // Step 5: Turn robot to match its initial pose
    step++;
    theta_pid.reset();
    desired_theta = DEG_TO_RAD(initialPose.theta);

    ROS_INFO("STEP 5: Turning the robot to match its initial pose");
    theta_err = PI;
    while(ros::ok() && (fabs(theta_err) > 0.1))
    {
      ros::Time now = ros::Time::now();
      float time_sec = now.sec + NS_TO_SEC(now.nsec);

      theta_err = DEG_TO_RAD(currentPose2d.theta) - desired_theta;
      if(prev_timestamp != 0) {
        float sample_time = time_sec - prev_timestamp;
        anglular_velocity = -1.0*theta_pid.step(theta_err, sample_time);
        // linear velocity
        move.linear.x = 0;
        // anglular velocity
        move.angular.z = anglular_velocity;
        move_pub.publish(move);
      }
      prev_timestamp = time_sec;
      ros::spinOnce();
      rate.sleep();
    }

    // Stop the robot
    // linear velocity
    move.linear.x = 0;
    // anglular velocity
    move.angular.z = 0;
    move_pub.publish(move);
    ROS_INFO("Robot back to the base");
    // blocking loop
    while(ros::ok()) {
      ros::spinOnce();
      rate.sleep();
    }
}
