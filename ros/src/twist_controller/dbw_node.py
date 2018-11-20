#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller


# import rospy

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        # Get car parameters
        self.carParams = {}
        self.carParams['vehicle_mass'] = rospy.get_param('~vehicle_mass', 1736.35)
        self.carParams['fuel_capacity'] = rospy.get_param('~fuel_capacity', 13.5)
        self.carParams['brake_deadband'] = rospy.get_param('~brake_deadband', .1)
        self.carParams['decel_limit'] = rospy.get_param('~decel_limit', -5)
        self.carParams['accel_limit'] = rospy.get_param('~accel_limit', 1.)
        self.carParams['wheel_radius'] = rospy.get_param('~wheel_radius', 0.2413)
        self.carParams['wheel_base'] = rospy.get_param('~wheel_base', 2.8498)
        self.carParams['steer_ratio'] = rospy.get_param('~steer_ratio', 14.8)
        self.carParams['max_lat_accel'] = rospy.get_param('~max_lat_accel', 3.)
        self.carParams['max_steer_angle'] = rospy.get_param('~max_steer_angle', 8.)

        # Create all publishers
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # Subscribe to the topics
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbwenable_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.cVelocity_cb)

        # Create `Controller` object
        self.controller = Controller(self.carParams)

        # Set init values
        self.dbwEnable = False
        self.desiredLinearSpeed = 0
        self.desiredAngularSpeed = 0
        self.curentSpeed = 0

        # Loop
        self.loop()

    def loop(self):
        rate = rospy.Rate(50)  # Set control rate to 50Hz

        while not rospy.is_shutdown():

            if self.dbwEnable:

                # Get predicted throttle, brake, and steering using `twist_controller`
                throttle, brake, steering = self.controller.control(self.desiredLinearSpeed,
                                                                self.desiredAngularSpeed,
                                                                self.curentSpeed)

                # Convert break value to N*m
                brake *= self.carParams['vehicle_mass']*self.carParams['wheel_radius']

                # Set hold brake value
                if abs(self.desiredLinearSpeed) < 0.1:
                    brake = max(brake, 700)
                    steering = 0  # prevent from unnecessary steering

                # Prevent car from driving too slow before stop
                if brake < 200:
                    brake = 0

                # Publish the control commands
                self.publish(throttle, brake, steering)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def twist_cb(self, twistStamped):
        self.desiredLinearSpeed = twistStamped.twist.linear.x
        self.desiredAngularSpeed = twistStamped.twist.angular.z
        # rospy.loginfo("DLS: {}, DAS: {}".format(self.desiredLinearSpeed, self.desiredAngularSpeed))
        pass

    def dbwenable_cb(self, enable):
        if not enable and self.dbwEnable:
            self.controller.reset()  # reset controller if car switched to manual mode
        self.dbwEnable = enable

    def cVelocity_cb(self, twistStamped):
        self.curentSpeed = twistStamped.twist.linear.x

if __name__ == '__main__':
    DBWNode()
