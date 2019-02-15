#!/usr/bin/env python

from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math

## Params for optimum_policy_path
forward = [[-1,  0], # go up
           [ 0, -1], # go left
           [ 1,  0], # go down
           [ 0,  1]] # go right
forward_name = ['up', 'left', 'down', 'right']
# action has 3 values: right turn, no turn, left turn
action = [-1, 0, 1]
action_name = ['R', '#', 'L']
cost_action = [2, 1, 2] # cost has 3 values, corresponding to making
                        # a right turn, no turn, and a left turn

## Params for optimum_policy
cost = 1 # the cost associated with moving from a cell to an adjacent one
delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right
delta_name = ['^', '<', 'v', '>']

## Robot pose, robot is at the center of the grid facing up
robot_pose = [25, 25, 0] # given in the form [row,col,direction]
                         # direction = 0: up
                         #             1: left
                         #             2: down
                         #             3: right

class TurtlebotPlanner(object):
    def __init__(self):
        rospy.init_node('turtlebot_planner')

        # CV Bridge
        self.bridge = CvBridge()

        # Get the destination
        self.goal_x = rospy.get_param('~goal_x', 45.0)
        self.goal_y = rospy.get_param('~goal_y', 25.0)

        # Grid
        self.grid_mat = np.zeros((50, 50), dtype=np.uint8)

        # subscribe to the 2D grid msg
        rospy.Subscriber('/turtlebot_scan/grid', Image, self.turtlebot_grid_cb)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def turtlebot_grid_cb(self, msg):
        grid = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.grid_mat = np.asarray(grid/255)

        ## Print the grid for verification
        # rospy.loginfo("Grid Matrix")
        # for i in range(self.grid_mat.shape[0]):
        #     for j in range(self.grid_mat.shape[1]):
        #         print(self.grid_mat[i][j], end =" ")
        #     print("")

        # policy = np.array(self.optimum_policy(self.grid_mat, [self.goal_y, self.goal_x], cost))
        # rospy.loginfo("Optimum Policy")
        # for i in range(policy.shape[0]):
        #     for j in range(policy.shape[1]):
        #         print(policy[i][j], end =" ")
        #     print("")

        path = np.array(self.optimum_policy_path(self.grid_mat, robot_pose, [self.goal_y, self.goal_x, 0], cost_action))
        rospy.loginfo("Optimum Path")
        for i in range(path.shape[0]):
            for j in range(path.shape[1]):
                print(path[i][j], end =" ")
            print("")

    def optimum_policy(self, grid, goal, cost):
        x_len = len(grid[0])
        y_len = len(grid)
        value = [[999 for col in range(x_len)] for row in range(y_len)]
        policy = [[' ' for col in range(x_len)] for row in range(y_len)]

        # changed indicates if there was a change in cost
        changed = True
        while changed:
            changed = False

            for y in range(y_len):
                for x in range(x_len):
                    # Check if the goal is reached
                    if goal[0] == y and goal[1] == x:
                        if value[y][x] > 0:
                            value[y][x] = 0
                            policy[y][x] = '*'
                            changed = True

                    # if the cell is navigable, then find direction to lower cost
                    elif grid[y][x] == 0:
                        for a in range(len(delta)):
                            y2 = y + delta[a][0]
                            x2 = x + delta[a][1]
                            if x2 >= 0 and x2 < x_len and y2 >= 0 and y2 < y_len and grid[y2][x2] == 0:
                                v2 = value[y2][x2] + cost

                                # Lower cost found
                                if v2 < value[y][x]:
                                    changed = True
                                    value[y][x] = v2
                                    policy[y][x] = delta_name[a]

        return policy

    def optimum_policy_path(self, grid,init,goal,cost):
        x_len = len(grid[0])
        y_len = len(grid)
        value = [[[999 for col in range(x_len)] for row in range(y_len)],
                 [[999 for col in range(x_len)] for row in range(y_len)],
                 [[999 for col in range(x_len)] for row in range(y_len)],
                 [[999 for col in range(x_len)] for row in range(y_len)]]
        policy = [[[' ' for col in range(x_len)] for row in range(y_len)],
                  [[' ' for col in range(x_len)] for row in range(y_len)],
                  [[' ' for col in range(x_len)] for row in range(y_len)],
                  [[' ' for col in range(x_len)] for row in range(y_len)]]
        policy3D = [['.' for col in range(x_len)] for row in range(y_len)]
        changed = True
        while changed:
            changed = False
            # Go through all grid cells and calculate values
            for y in range(y_len):
                for x in range(x_len):
                    for orientation in range(4):
                        if goal[0] == y and goal[1] == x:
                            if value[orientation][y][x] > 0:
                                value[orientation][y][x] = 0
                                policy[orientation][y][x] = '*'
                                changed = True

                        elif grid[y][x] == 0:
                            # calculate the three ways to propagate value
                            for i in range(3):
                                o2 =(orientation + action[i]) % 4
                                y2 = y + forward[o2][0]
                                x2 = x + forward[o2][1]

                                if x2 >= 0 and x2 < x_len and y2 >= 0 and y2 < y_len and grid[y2][x2] == 0:
                                    v2 = value[o2][y2][x2] + cost[i]
                                    if v2 < value[orientation][y][x]:
                                        value[orientation][y][x] = v2
                                        policy[orientation][y][x] = action_name[i]
                                        changed = True

                        elif grid[y][x] == 1:
                            # display walls
                            policy3D[y][x] = "o"

        y = init[0]
        x = init[1]
        orientation = init[2]
        policy3D[y][x] = policy[orientation][y][x]
        while  policy[orientation][y][x] != '*':
            if  policy[orientation][y][x] == '#':
                o2 = orientation
            elif  policy[orientation][y][x] == 'R':
                o2 = (orientation - 1)%4
            elif  policy[orientation][y][x] == 'L':
                o2 = (orientation + 1)%4

            y = y + forward[o2][0]
            x = x + forward[o2][1]
            orientation = o2
            policy3D[y][x] = policy[orientation][y][x]

        return policy3D

if __name__ == '__main__':
    try:
        TurtlebotPlanner()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start turtlebot planner node.')
