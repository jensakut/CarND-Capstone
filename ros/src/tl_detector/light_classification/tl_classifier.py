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

# Solution for using an inference graph

"""
from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import datetime


class TLClassifier(object):
    def __init__(self, is_site):
        if is_site:
		# TODO add inference graph 
		PATH_TO_GRAPH = r'light_classification/inference_graph_real_data_ssd_inception_v2_coco_2018_01_28/frozen_inference_graph.pb'
	
	else: 
		PATH_TO_GRAPH = r'light_classification/inference_graph_sim_data_ssd_inception_v2_coco_2018_01_28/frozen_inference_graph.pb'
        
 	#self.graph = tf.Graph()
	#with self.graph.as_default():
		od_graph_def = tf.GraphDef()
		with tf.gfile.GFile(PATH_TO_GRAPH, 'rb') as fid:
		    serialized_graph = fid.read()
		    od_graph_def.ParseFromString(serialized_graph)
		    tf.import_graph_def(od_graph_def, name='')

	with tf.Session(graph=self.graph) as self.sess: 
		self.image_tensor     = self.sess.graph.get_tensor_by_name('image_tensor:0')
		self.boxes  = self.sess.graph.get_tensor_by_name('detection_boxes:0')
		self.scores = self.sess.graph.get_tensor_by_name('detection_scores:0')
		self.classes= self.sess.graph.get_tensor_by_name('detection_classes:0')
		self.num_detections   = self.sess.graph.get_tensor_by_name('num_detections:0')
		


    def get_classification(self, image):
        #"""Determines the color of the traffic light in the image

#        Args:
 #           image (cv::Mat): image containing the traffic light

  #      Returns:
   #         int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

	image_exp = np.expand_dims(np.asarray(image, dtype=np.uint8),0)
	start = datetime.datetime.now()
	(boxes, scores, classes, num_detections) = sess.run([self.boxes, self.scores, self.classes, self.num_detections], feed_dict={self.image_tensor: img_exp})
	end = datetime.datetime.now()
	c = end-start; 
        print(c.total_seconds)
	#remove unnecessary dimensions
	boxes = np.squeeze(boxes)
	scores = np.squeeze(scores)
	classes = np.squeeze(classes).astype(np.int32)

	confidence_cutoff = 0.8

	print('SCORES: ', scores[0])
	print('CLASSES: ', classes[0])
	
	if scores[0] > confidence_cutoff: 
		if classes[0] == 1:
			print('GREEN')
			return TrafficLight.GREEN
		elif classes[0] == 2:
			print('RED')
			return TrafficLight.RED
		elif classes[0] == 3:
			print('YELLOW')
			return TrafficLight.YELLOW
	 	elif classes[0] == 4: 
			print('off')
			return TrafficLight.UNKNOWN
 	print('No traffic light in sight')
        return TrafficLight.UNKNOWN

"""


