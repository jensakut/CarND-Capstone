#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped

import math
import copy

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    """This class takes all track waypoints, current car position and generate next LOOKAHEAD_WPS number waypoints"""

    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.cVelocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.close_waypointN_pub = rospy.Publisher('close_waypoint_n', Int32, queue_size=1)

        self.allWPs = None  # List for all track waypoints

        self.finalWPS = Lane()  # Prepare message for publication
        self.finalWPS.header.seq = 0
        self.finalWPS.header.frame_id = "/world"
        self.trafficWPStopN = -1
        self.traficStopDist = 50

        self.closesetPointN = None

        rospy.spin()

    def pose_cb(self, msg):
        """Callback for '/current_pose' message receive, also publishes 'final_waypoints' message """

        if not self.allWPs:
            return

        # Find closest waypoint near the car
        self.closesetPointN = self.findClosestWaypoint(msg.pose.position, self.closesetPointN)

        # Detect if we have already passed this point and choose next point in this case
        if self.closesetPointN + 1 < len(self.allWPs):  # if this waypoint is not the last one
            v_path_x = self.allWPs[self.closesetPointN + 1].pose.pose.position.x - \
                   self.allWPs[self.closesetPointN].pose.pose.position.x
            v_path_y = self.allWPs[self.closesetPointN + 1].pose.pose.position.y - \
                   self.allWPs[self.closesetPointN].pose.pose.position.y
            c_path_x = msg.pose.position.x - self.allWPs[self.closesetPointN].pose.pose.position.x
            c_path_y = msg.pose.position.y - self.allWPs[self.closesetPointN].pose.pose.position.y
            # if dot product is more than zero we choose next waypoint:
            dp = v_path_x*c_path_x + v_path_y*c_path_y
            if dp > 0:
                self.closesetPointN = self.closesetPointN + 1

        # Update final waypoints message
        self.finalWPS.header.seq += 1
        self.finalWPS.header.stamp = rospy.rostime.Time().now()
        self.finalWPS.waypoints = []
        for wpn in range(self.closesetPointN, min(self.closesetPointN+LOOKAHEAD_WPS, len(self.allWPs))):
            wp = self.allWPs[wpn]
            # if there are red light in front we update waypoint speeds accordingly
            if self.trafficWPStopN >= self.closesetPointN:  # set stop speed
                if self.trafficWPStopN - wpn > self.traficStopDist:
                    wp.twist.twist.linear.x = self.originalSpeeds[wpn]
                elif self.trafficWPStopN > wpn:  # linear decrease speed of close points
                    wp.twist.twist.linear.x = self.originalSpeeds[wpn] * (self.trafficWPStopN - wpn) / self.traficStopDist
                else:
                    wp.twist.twist.linear.x = 0  # set speed of far points to 0
            else:
                wp.twist.twist.linear.x = self.originalSpeeds[wpn]  # restore original speed value
            self.finalWPS.waypoints.append(wp)

        self.final_waypoints_pub.publish(self.finalWPS)
        self.close_waypointN_pub.publish(self.closesetPointN)

    def waypoints_cb(self, waypoints):
        """Callback for '/base_waypoints' messages."""
        # Copy original speeds
        self.originalSpeeds = [wp.twist.twist.linear.x for wp in waypoints.waypoints]
        self.allWPs = waypoints.waypoints

    def traffic_cb(self, msg):
        self.trafficWPStopN = msg.data - 4
        pass

    def cVelocity_cb(self, twistStamped):
        self.curentSpeed = twistStamped.twist.linear.x

    # def obstacle_cb(self, msg):
    #     # TODO: Callback for /obstacle_waypoint message. We will implement it later
    #     pass

    # def get_waypoint_velocity(self, waypoint):
    #     return waypoint.twist.twist.linear.x

    # def set_waypoint_velocity(self, waypoints, waypoint, velocity):
    #     waypoints[waypoint].twist.twist.linear.x = velocity

    def length(self, a, b):
        """ Calculate distance between two points."""

        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)

    # def distance(self, waypoints, wp1, wp2):
    #     dist = 0
    #     for i in range(wp1, wp2+1):
    #         dist += self.length(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
    #         wp1 = i
    #     return dist

    def findClosestWaypoint(self, currentPosition, previousWayPointN = None):

        closesetPointDist = float("inf")
        closesetPointN = -1

        if previousWayPointN is None:
            previousWayPointN = 0

        # Look for closest waypoint starting from previous closest waypoint upto ...
        for wpn in range(previousWayPointN, len(self.allWPs)):
            d = self.length(self.allWPs[wpn].pose.pose.position, currentPosition)
            if d < closesetPointDist:
                closesetPointDist = d
                closesetPointN = wpn
            else:   #  ... upto the moment when distance starts to increase
                # rospy.loginfo("n: {}, x1: {}, y1: {}, xc: {}, yc: {}"\
                #             .format(closesetPointN, self.allWPs[closesetPointN].pose.pose.position.x, \
                #             self.allWPs[closesetPointN].pose.pose.position.y, \
                #             msg.pose.position.x, msg.pose.position.y))
                return closesetPointN
        return closesetPointN



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
