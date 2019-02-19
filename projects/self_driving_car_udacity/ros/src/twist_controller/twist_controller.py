import rospy
from geometry_msgs.msg import TwistStamped
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

THROTTLE_MIN = -1.0 # Actual minimum is 0.0, it is set to -1.0 to find the amount of braking
THROTTLE_MAX = 1.0
BRAKE_SLOWDOWN_TORQUE = 200.
BRAKE_MAX_TORQUE = 700.

class Controller(object):
    def __init__(self, *args, **kwargs):
        self.throttle_pid = PID(kwargs['throttle_pid_gain'][0],
                                kwargs['throttle_pid_gain'][1],
                                kwargs['throttle_pid_gain'][2],
                                THROTTLE_MIN,
                                THROTTLE_MAX)
        self.steer_yaw_controller = YawController(kwargs['wheel_base'],
                                                  kwargs['steer_ratio'],
                                                  kwargs['min_speed'],
                                                  kwargs['max_lat_accel'],
                                                  kwargs['max_steer_angle'])
        self.throttle_lpf = LowPassFilter(0.2,0.1)
        self.yaw_lpf = LowPassFilter(0.2,0.1)

        self.prev_timestamp = None
        self.cur_timestamp = None

    def control(self, dbw_enabled,
                      desired_linear_vel,
                      desired_angular_vel,
                      cur_velocity):
        rospy.loginfo('desired linear vel = [%f,%f]; desired ang vel[roll, pitch, yaw] = [%f,%f,%f]; cur_vel = [%f,%f]',
                                                    desired_linear_vel.x, desired_linear_vel.y,
                                                    desired_angular_vel.x, desired_angular_vel.y, desired_angular_vel.z,
                                                    cur_velocity.x, cur_velocity.y)

        self.cur_timestamp = rospy.get_time()
        throttle = 0.
        brake = 0.
        steering = 0.
        if self.prev_timestamp is None:
            self.prev_timestamp = self.cur_timestamp
            return 0.,0.,0.
        else:
            sample_time = self.cur_timestamp - self.prev_timestamp
            error = desired_linear_vel.x - cur_velocity.x
            if dbw_enabled is False:
                # Reset the controller so that the integral term does not keep accumulating
                self.throttle_pid.reset()

            throttle = self.throttle_pid.step(error, sample_time)
            # smooth out the throttle command
            throttle = self.throttle_lpf.filt(throttle)

            # check if braking is required
            if throttle < 0:
                # apply brakes
                brake = -1.0*throttle*BRAKE_SLOWDOWN_TORQUE
                throttle = 0.0

            # check if the vehicle needs to come at a full stop
            if((desired_linear_vel.x < 0.2) and (throttle < 0.1)):
                brake = BRAKE_MAX_TORQUE

            steering = self.steer_yaw_controller.get_steering(desired_linear_vel.x, desired_angular_vel.z, cur_velocity.x)
            # smooth out the steering command
            steering = self.yaw_lpf.filt(steering)

            self.prev_timestamp = self.cur_timestamp

            # Return throttle, brake, steer
            return throttle, brake, steering
