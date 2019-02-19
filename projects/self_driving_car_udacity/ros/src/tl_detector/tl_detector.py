#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np

STATE_COUNT_THRESHOLD = 3
RELEVANT_TRAFFIC_LIGHT_DIST = 100.

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.base_waypoints = []
        self.upcoming_traffic_light = None
        self.current_pose = None

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        simulator_mode = not self.config['is_site']

        # set buffer size for /current_pose in order to avoid delay in receiving updated pose
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1, buff_size=2**24)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        # Uncomment the line below if running the simulator and do not want to use camera data
        #sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(simulator_mode)
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        image_topic = '/image_raw'
        if simulator_mode:
            image_topic = '/image_color'
        sub6 = rospy.Subscriber(image_topic, Image, self.image_cb)

        rospy.spin()

    def pose_cb(self, msg):
        self.current_pose = msg.pose
        closest_traffic_light_idx = -1
        closest_traffic_light_dist = 10000.0

        #rospy.loginfo('Current Pose = [%f, %f]', self.current_pose.position.x, self.current_pose.position.y)
        # check if a traffic light is approaching
        for i in range(len(self.config['stop_line_positions'])):
            closest_wp_idx = self.get_closest_waypoint(self.current_pose)

            if closest_wp_idx is not None:
                stop_line_wp_pose = Pose()
                stop_line_wp_pose.position.x = self.config['stop_line_positions'][i][0]
                stop_line_wp_pose.position.y = self.config['stop_line_positions'][i][1]
                stop_line_wp_idx = self.get_closest_waypoint(stop_line_wp_pose)

                stop_line_dist = self.distance(self.base_waypoints, closest_wp_idx, stop_line_wp_idx)
                # make sure the traffic light is within a relevant distance and ahead of vehicle
                if(stop_line_dist < RELEVANT_TRAFFIC_LIGHT_DIST):
                   # pick the traffic light closest to vehicle
                   if(stop_line_dist < closest_traffic_light_dist):
                       closest_traffic_light_dist = stop_line_dist
                       closest_traffic_light_idx = i

        if (closest_traffic_light_idx is not -1):
           self.upcoming_traffic_light = self.config['stop_line_positions'][closest_traffic_light_idx]
           #rospy.loginfo('Upcoming traffic light = [%f, %f]; Dist = %f', self.config['stop_line_positions'][closest_traffic_light_idx][0], self.config['stop_line_positions'][closest_traffic_light_idx][1], closest_traffic_light_dist)
        else:
           self.upcoming_traffic_light = None

    def euclidean_dist_2d(self, wp1, wp2):
        return math.sqrt((wp1[0]-wp2[0])**2 + (wp1[1]-wp2[1])**2)

    def waypoints_cb(self, waypoints):
        if not len(self.base_waypoints):
            for i in range(len(waypoints.waypoints)):
                self.base_waypoints.append([waypoints.waypoints[i].pose.pose.position.x,
                                            waypoints.waypoints[i].pose.pose.position.y])

    def traffic_cb(self, msg):
        self.lights = msg.lights
        light_pose = Pose()
        if self.upcoming_traffic_light is not None:
            # find the state of traffic light
            smallest_dist_between_wp = 10000.0
            upcoming_traffic_light_idx = None
            for i in range(len(self.lights)):
                dist_between_wp = self.euclidean_dist_2d([self.lights[i].pose.pose.position.x,
                                                         self.lights[i].pose.pose.position.y],
                                                         self.upcoming_traffic_light)
                if dist_between_wp < smallest_dist_between_wp:
                    smallest_dist_between_wp = dist_between_wp
                    upcoming_traffic_light_idx = i

            # get state of the light
            state = self.lights[upcoming_traffic_light_idx].state
            light_pose.position.x = self.upcoming_traffic_light[0]
            light_pose.position.y = self.upcoming_traffic_light[1]
            light_wp_idx = self.get_closest_waypoint(light_pose)

            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''
            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                light_wp_idx = light_wp_idx if state == TrafficLight.RED else -1
                self.last_wp = light_wp_idx
                self.upcoming_red_light_pub.publish(Int32(light_wp_idx))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))

            self.state_count += 1

        else:
            self.state = TrafficLight.UNKNOWN
            self.last_state = TrafficLight.UNKNOWN
            self.last_wp = -1
            self.state_count = 0

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        # try to classify the image only if a traffic light exests in the ROI
        if self.upcoming_traffic_light is not None:
            light_wp, state = self.process_traffic_lights()

            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''
            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
        if(wp2 < wp1):
            for i in range(wp1, len(waypoints)-1):
                dist += dl(waypoints[i], waypoints[i+1])
            for i in range(0, wp2):
                dist += dl(waypoints[i], waypoints[i+1])
        else:
            for i in range(wp1, wp2+1):
                dist += dl(waypoints[i], waypoints[i+1])
        return dist

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        idx = None
        waypoint = None
        min_dist = 100000.
        if ((self.base_waypoints is not None) and (pose is not None)):
            for i in range(len(self.base_waypoints)):
                dist = self.euclidean_dist_2d(self.base_waypoints[i], [pose.position.x, pose.position.y])
                if(dist < min_dist):
                    idx = i
                    min_dist = dist

            if idx is not None:
                # Check if the closest_waypoint is ahead of vehicle, if not then use idx+i
                closest_coord = self.base_waypoints[idx]
                prev_coord = self.base_waypoints[idx-1]
                # Equation for hyperplane through closest_coords
                cl_vect = np.array(closest_coord)
                prev_vect = np.array(prev_coord)
                pos_vect = np.array([pose.position.x, pose.position.y])

                # dot product of (x1 + j.y1) and (x2 + j.y2) will be positive if they are in the same direction
                # If vectors (cl_vect-prev_vect) and (pos_vect-cl_vect) are in the same direction then pos_vect
                # will be ahead of cl_vect
                val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
                if val > 0:
                    idx = (idx + 1) % len(self.base_waypoints)

        return idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_pose = Pose()
        light_wp_idx = None
        if self.upcoming_traffic_light is not None:
            # find the state of traffic light
            smallest_dist_between_wp = 10000.0
            upcoming_traffic_light_idx = None
            for i in range(len(self.lights)):
                dist_between_wp = self.euclidean_dist_2d([self.lights[i].pose.pose.position.x,
                                                         self.lights[i].pose.pose.position.y],
                                                         self.upcoming_traffic_light)
                if dist_between_wp < smallest_dist_between_wp:
                    smallest_dist_between_wp = dist_between_wp
                    upcoming_traffic_light_idx = i

            light_pose.position.x = self.upcoming_traffic_light[0]
            light_pose.position.y = self.upcoming_traffic_light[1]
            light_wp_idx = self.get_closest_waypoint(light_pose)

            if light_pose:
                state = self.get_light_state(light_pose)
                #rospy.loginfo("Predicted: %d, Actual: %d", state, self.lights[upcoming_traffic_light_idx].state)
                return light_wp_idx, state

        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
