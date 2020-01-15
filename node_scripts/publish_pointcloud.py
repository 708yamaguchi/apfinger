#!/usr/bin/env python

import numpy as np
import rospy

from sensor_msgs.msg import PointCloud2, PointField
from apfinger.msg import ProximityDistances


class ProximityPointCloud(object):

    def __init__(self):
        self.pubs = {}
        for finger in ['/left', '/right']:
            for frame in ['/top', '/side0', '/side1', '/side2', '/side3']:
                pub_name = '/proximity_pointcloud'+finger+frame
                pub = self.advertise(pub_name, PointCloud2, queue_size=1)
                self.pubs[finger][frame] = pub
        self.subscribe()

    def __del__(self):
        self.unsubscribe()

    def subscribe(self):
        self.subs = {}
        for finger in ['/left', '/right']:
            for frame in ['/top', '/side0', '/side1', '/side2', '/side3']:
                sub_name = '/proximity_distance'+finger+frame
                sub = rospy.Subscriber(
                    sub_name,
                    ProximityDistances,
                    self._cb_pub_pointcloud,
                    (finger, frame))
                self.subs[finger][frame] = sub

    def unsubscribe(self):
        for finger in ['/left', '/right']:
            for frame in ['/top', '/side0', '/side1', '/side2', '/side3']:
                self.subs[finger][frame].unregister()

    def _cb_pub_pointcloud(self, msg, args):
        self.msg_to_pointcloud(msg, args[0], args[1])

    # main method of this class, convert xyz points to sensor_msgs/PointCloud2
    def msg_to_pointcloud(self, msg, finger, frame):
        points = PointCloud2()
        points.header.stamp = rospy.Time.now()
        points.header.frame_id = frame
        points.height = 1
        points.width = len(msg)
        points.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        points.is_bigendian = False
        points.point_step = 12
        points.row_step = 12*len(msg)
        points.is_dense = False  # this pointcloud may have invalid data

        # TODO : main conversion xyz -> pointcloud
        # points.data = np.asarray(points, np.float32).tostring()
        # TODO
        self.pubs[finger][frame].publish(points)


if __name__ == '__main__':
    rospy.init_node('proximity_pointcloud')
    ProximityPointCloud()
    rospy.spin()
