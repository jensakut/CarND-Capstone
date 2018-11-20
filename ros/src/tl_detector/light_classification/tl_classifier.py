from styx_msgs.msg import TrafficLight
import numpy as np
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        red1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 100, 100])
        upper_red = np.array([180, 255, 255])
        red2 = cv2.inRange(hsv, lower_red, upper_red)

        converted_img_red = cv2.addWeighted(red1, 1.0, red2, 1.0, 0.0)

        blur_img_red = cv2.GaussianBlur(converted_img_red, (15,15), 0)

        circles_red = cv2.HoughCircles(blur_img_red, cv2.HOUGH_GRADIENT, 0.5, 41, param1=70, param2=30, minRadius=5, maxRadius=150)

        if circles_red is not None:
            return TrafficLight.RED

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        blur_img_yellow = cv2.GaussianBlur(yellow, (15,15), 0)

        circles_yellow = cv2.HoughCircles(blur_img_yellow, cv2.HOUGH_GRADIENT, 0.5, 41, param1=70, param2=30, minRadius=5, maxRadius=150)

        if circles_yellow is not None:
            return TrafficLight.YELLOW

        # Do not differentiate green and unknown
        return TrafficLight.UNKNOWN

