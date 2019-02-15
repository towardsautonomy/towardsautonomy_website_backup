#include <math.h>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "turtlebot_custom_msg/wall_obj.h"
#include "turtlebot_custom_msg/environment_msgs.h"

#include "opencv2/imgcodecs.hpp"
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

#define PI              3.14159265359
#define DEG_TO_RAD(deg) ((deg)*PI/180.0)
#define RAD_TO_DEG(rad) ((rad)*180.0/PI)

#define GRID_X_LEN      4.0   // Grid X length in meters
#define GRID_Y_LEN      4.0   // Grid Y length in meters
#define GRID_LEN        50.0  // Grid length in pixels
#define GRID_SCALE      (GRID_LEN/GRID_X_LEN) // Meters/Pixels scale

#define GRID_CENTER_X   (GRID_X_LEN*GRID_SCALE/2)
#define GRID_CENTER_Y   (GRID_Y_LEN*GRID_SCALE/2)

// objects in robot environment
turtlebot_custom_msg::wall_obj wall;
turtlebot_custom_msg::environment_msgs turtlebot_env;
ros::Publisher turtlebot_env_pub;

// Publish the 2D grid as an image
ros::Publisher grid_pub;

// A top-view grid 50x50 corresponding to x=5, y=5 scaled by 10
// 0 represents navigable area, 1 represents boundaries
uint8_t grid[(int)(GRID_X_LEN*GRID_SCALE)][(int)(GRID_Y_LEN*GRID_SCALE)];

// Set the entire grid as navigable
void resetGrid() {
  for(int i=0; i<(int)(GRID_Y_LEN*GRID_SCALE); i++) {
    for(int j=0; j<(int)(GRID_X_LEN*GRID_SCALE); j++) {
      grid[i][j] = 0;
    }
  }
}

// Print grid
void printGrid() {
  printf("\n=============================================|| GRID ||=============================================\n");
  for(int i=0; i<(int)(GRID_Y_LEN*GRID_SCALE); i++) {
    for(int j=0; j<(int)(GRID_X_LEN*GRID_SCALE); j++) {
      if((i == GRID_CENTER_Y) && (j == GRID_CENTER_X)) printf("X");
      else if((i == GRID_CENTER_Y) && (j == (GRID_CENTER_X-1))) printf("||");
      else if((i == GRID_CENTER_Y) && (j == (GRID_CENTER_X+1))) printf("|| ");
      else if((i == (GRID_CENTER_Y-1)) && (j == GRID_CENTER_X)) printf("^ ");
      else printf("%d ",grid[i][j]);
    }
    printf("\n");
  }
  printf("====================================================================================================\n");
}

// cross product of two vectors a and b
float crossProduct(float a[], float b[]) {
    return (a[0]*b[1] - a[1]*b[0]);
}

/* Laser Scan callback */
void scanCallback(const sensor_msgs::LaserScan& msg) {
  resetGrid();
  /* Set frame_id and timestamp from the laser scan */
  turtlebot_env.header = msg.header;

  for(int angle=(int)RAD_TO_DEG(msg.angle_min);
          angle<(int)RAD_TO_DEG(msg.angle_max);
          angle+=(int)RAD_TO_DEG(msg.angle_increment)) {
            if(msg.ranges[angle] < (float)GRID_X_LEN) {
              /* Laser Scans go 0 to 360 degrees anticlockwise according to right-hand rule */
              float abs_x, abs_y;
              int x_idx, y_idx;
              // 0 to 90 degrees
              if((angle >= 0) && (angle < 90)) {
                abs_x = msg.ranges[angle]*sin(DEG_TO_RAD(angle));
                abs_y = msg.ranges[angle]*cos(DEG_TO_RAD(angle));
                x_idx = GRID_CENTER_X - (int)(abs_x*(float)GRID_SCALE);
                y_idx = GRID_CENTER_Y - (int)(abs_y*(float)GRID_SCALE);
                if((x_idx >= 0) && (y_idx >= 0)) {
                  grid[y_idx][x_idx] = 1;
                }
              }
              // 90 to 180 degrees
              if((angle >= 90) && (angle < 180)) {
                abs_x = msg.ranges[angle]*sin(DEG_TO_RAD(180-angle));
                abs_y = msg.ranges[angle]*cos(DEG_TO_RAD(180-angle));
                x_idx = GRID_CENTER_X - (int)(abs_x*(float)GRID_SCALE);
                y_idx = GRID_CENTER_Y + (int)(abs_y*(float)GRID_SCALE);
                if((x_idx >= 0) && (y_idx < (int)(GRID_Y_LEN*GRID_SCALE))) {
                  grid[y_idx][x_idx] = 1;
                }
              }
              // 180 to 270 degrees
              if((angle >= 180) && (angle < 270)) {
                abs_x = msg.ranges[angle]*sin(DEG_TO_RAD(angle-180));
                abs_y = msg.ranges[angle]*cos(DEG_TO_RAD(angle-180));
                x_idx = GRID_CENTER_X + (int)(abs_x*(float)GRID_SCALE);
                y_idx = GRID_CENTER_Y + (int)(abs_y*(float)GRID_SCALE);
                if((x_idx < (int)(GRID_X_LEN*GRID_SCALE)) && (y_idx < (int)(GRID_Y_LEN*GRID_SCALE))) {
                  grid[y_idx][x_idx] = 1;
                }
              }
              // 270 to 360 degrees
              if((angle >= 270) && (angle < 360)) {
                abs_x = msg.ranges[angle]*sin(DEG_TO_RAD(360-angle));
                abs_y = msg.ranges[angle]*cos(DEG_TO_RAD(360-angle));
                x_idx = GRID_CENTER_X + (int)(abs_x*(float)GRID_SCALE);
                y_idx = GRID_CENTER_Y - (int)(abs_y*(float)GRID_SCALE);
                if((x_idx < (int)(GRID_X_LEN*GRID_SCALE)) && (y_idx >= 0)) {
                  grid[y_idx][x_idx] = 1;
                }
              }
            }
    }
    Mat grid_mat = cv::Mat((int)(GRID_X_LEN*GRID_SCALE), (int)(GRID_Y_LEN*GRID_SCALE), CV_8UC1);
    memcpy(grid_mat.data, grid, (int)(GRID_X_LEN*GRID_SCALE)*(int)(GRID_Y_LEN*GRID_SCALE)*sizeof(uint8_t));

    // Publish the grid as a message
    cv_bridge::CvImage grid;
    grid.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    grid.image    = grid_mat*255; // either 0 or 255 for easy visualization
    grid_pub.publish(grid.toImageMsg());

    vector<Vec4i> lines;
    HoughLinesP( grid_mat, lines, 1, CV_PI/180, 15, 10, 5);
    //ROS_INFO("Number of walls: %u", (unsigned)lines.size());

    turtlebot_env.walls.clear();
    for( int i = 0; i < lines.size(); i++ ) {
        float wall_len = sqrt(pow(((float)lines[i][0] - (float)lines[i][2]), 2) + pow(((float)lines[i][1] - (float)lines[i][3]), 2))/GRID_SCALE;
        float slope, slope_deg;
        // Y is flipped in the image coordinates
        if(lines[i][3] <= lines[i][1]) {
          // divide-by-zero condition
          if((lines[i][2] - lines[i][0]) == 0) {
            slope = ((float)lines[i][1] - (float)lines[i][3])/0.001;
            slope_deg = 90.0;
          }
          else
            slope = ((float)lines[i][1] - (float)lines[i][3])/((float)lines[i][2] - (float)lines[i][0]);
            slope_deg = RAD_TO_DEG(atan(slope));
        }
        else {
          // divide-by-zero condition
          if((lines[i][0] - lines[i][2]) == 0) {
            slope = ((float)lines[i][3] - (float)lines[i][1])/0.001;
            slope_deg = 90.0;
          }
          else {
            slope = ((float)lines[i][3] - (float)lines[i][1])/((float)lines[i][0] - (float)lines[i][2]);
            slope_deg = RAD_TO_DEG(atan(slope));
          }
        }
        // Find equation of line: y = mx + b => b = y - mx
        // y axis is flipped
        float b = -1.0*(float)lines[i][1] - slope*(float)lines[i][0];
        // In the form Ax + By + C = 0 => A = -slope, B = 1, C = -b; Robots coordinate (m, n) = (GRID_CENTER_X, -GRID_CENTER_Y)
        // Dist to wall = |Am + Bn + C| / sqrt(A^2 + B^2)
        float dist_to_wall = fabs((-1.0*slope*GRID_CENTER_X - GRID_CENTER_Y - b)/sqrt(pow(slope, 2) + 1))/GRID_SCALE;

        // vectors for cross product
        float A[2], B[2];
        if(lines[i][3] <= lines[i][1]) {
          A[0] = (float)lines[i][0] - (float)lines[i][2];
          A[1] = (float)lines[i][1] - (float)lines[i][3];
          B[0] = (float)lines[i][0] - (float)GRID_CENTER_X;
          B[1] = (float)lines[i][1] - (float)GRID_CENTER_Y;

          wall.lower_xy[0] = ((float)lines[i][0] - GRID_CENTER_X)/GRID_SCALE;
          wall.lower_xy[1] = (GRID_CENTER_Y - (float)lines[i][1])/GRID_SCALE;
          wall.upper_xy[0] = ((float)lines[i][2] - GRID_CENTER_X)/GRID_SCALE;
          wall.upper_xy[1] = (GRID_CENTER_Y - (float)lines[i][3])/GRID_SCALE;
        }
        else {
          A[0] = (float)lines[i][2] - (float)lines[i][0];
          A[1] = (float)lines[i][3] - (float)lines[i][1];
          B[0] = (float)lines[i][2] - (float)GRID_CENTER_X;
          B[1] = (float)lines[i][3] - (float)GRID_CENTER_Y;

          wall.lower_xy[0] = ((float)lines[i][2] - GRID_CENTER_X)/GRID_SCALE;
          wall.lower_xy[1] = (GRID_CENTER_Y - (float)lines[i][3])/GRID_SCALE;
          wall.upper_xy[0] = ((float)lines[i][0] - GRID_CENTER_X)/GRID_SCALE;
          wall.upper_xy[1] = (GRID_CENTER_Y - (float)lines[i][1])/GRID_SCALE;
        }
        float cp = crossProduct(B, A);

        wall.orientation = slope_deg;
        wall.length = wall_len;
        wall.distance = dist_to_wall;
        // cp == 0 case does not apply because the robot can never be on the wall
        if(cp >= 0) {
          wall.direction = "right";
          //ROS_INFO("Right Wall [%d]: Length = %f meters, Slope_Deg = %f deg, y = %fx + %f, dist = %f meters", i, wall_len, slope_deg, slope, b, dist_to_wall);
        }
        else {
          wall.direction = "left";
          //ROS_INFO("Left Wall [%d]: Length = %f meters, Slope_Deg = %f deg, y = %fx + %f, dist = %f meters", i, wall_len, slope_deg, slope, b, dist_to_wall);
        }
        turtlebot_env.walls.push_back(wall);
    }
    turtlebot_env_pub.publish(turtlebot_env);

printGrid();
}

int main(int argc, char **argv){

ros::init(argc,argv,"turtlebot_scan_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/scan", 1000, scanCallback);
    turtlebot_env_pub = n.advertise<turtlebot_custom_msg::environment_msgs>("/turtlebot_env",1000);
    grid_pub = n.advertise<sensor_msgs::Image>("/turtlebot_scan/grid", 1000);

    ros::spin();
}
