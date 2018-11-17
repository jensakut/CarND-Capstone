
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

import rospy



GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, carParams):

        self.yawController = YawController(carParams['wheel_base'], carParams['steer_ratio'], 4,
                                           carParams['max_lat_accel'], carParams['max_steer_angle'])
        self.velController = PID(0.8, 0.1, 0.1, mn=carParams['decel_limit'], mx=carParams['accel_limit'])

        self.dLSFilter = LowPassFilter(0.2, 0.05)
        self.dASFilter = LowPassFilter(0.2, 0.05)
        pass

    def control(self, desiredLinearSpeed, desiredAngularSpeed, curentSpeed):

        # desiredLinearSpeed = abs(desiredLinearSpeed)

        # desiredLinearSpeed = self.dLSFilter.filt(desiredLinearSpeed)
        # desiredAngularSpeed = self.dASFilter.filt(desiredAngularSpeed)

        # Run PID controller for acceleration and break control
        acc = self.velController.step(desiredLinearSpeed - curentSpeed, 0.05)
        # rospy.loginfo("DesV: {}, CurV: {}, diff: {}, pid: {}".format(desiredLinearSpeed, curentSpeed,
        #                     desiredLinearSpeed-curentSpeed, acc))
        if (acc > 0):
            cThrottle = acc
            cBreak = 0
        else:
            cThrottle = 0
            cBreak = -acc

        cSteer = self.yawController.get_steering(abs(desiredLinearSpeed), desiredAngularSpeed, curentSpeed)*0.1
        # cSteer = desiredAngularSpeed
        rospy.loginfo("steer: {}, DAS: {}, DLS: {}, CS: {}".format(cSteer, desiredAngularSpeed, desiredLinearSpeed, curentSpeed))

        # Return throttle, brake, steer
        return cThrottle, cBreak, cSteer
