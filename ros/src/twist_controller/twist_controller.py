
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

import rospy


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, carParams):

        # Create steering controller
        self.yawController = YawController(carParams['wheel_base'], carParams['steer_ratio'], 0.1,
                                           carParams['max_lat_accel'], carParams['max_steer_angle'])
        # Create throttle and brake controller
        self.velController = PID(0.8, 0.1, 0.1, mn=carParams['decel_limit'], mx=carParams['accel_limit'])

        # Filter for speed input
        self.dLSFilter = LowPassFilter(0.5, 0.02)
        # self.dASFilter = LowPassFilter(0.1, 0.02)

        # previous iteration time
        self.prevT = rospy.get_time()
        pass

    def control(self, desiredLinearSpeed, desiredAngularSpeed, curentSpeed):

        # Filter input value of desired speed
        desiredLinearSpeed = self.dLSFilter.filt(desiredLinearSpeed)
        # desiredAngularSpeed = self.dASFilter.filt(desiredAngularSpeed)

        # Calculate dt
        t = rospy.get_time()
        dt = t - self.prevT
        self.prevT = t

        # Run PID controller for throttle and break control
        acc = self.velController.step(desiredLinearSpeed - curentSpeed, dt)
        # rospy.loginfo("DesV: {}, CurV: {}, diff: {}, pid: {}, dt: {}".format(desiredLinearSpeed, curentSpeed,
        #                     desiredLinearSpeed-curentSpeed, acc, dt))
        if (acc > 0):
            cThrottle = acc
            cBreak = 0
        else:
            cThrottle = 0
            cBreak = -acc

        cSteer = self.yawController.get_steering(abs(desiredLinearSpeed), desiredAngularSpeed, curentSpeed)
        # rospy.loginfo("steer: {}, DAS: {}, DLS: {}, CS: {}".format(cSteer, desiredAngularSpeed, desiredLinearSpeed, curentSpeed))

        # Return throttle, brake, steer
        return cThrottle, cBreak, cSteer

    def reset(self):
        self.velController.reset()
