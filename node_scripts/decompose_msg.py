#!/usr/bin/env python

import numpy as np
import rospy

from APFinger.msg import Proximities
from APFinger.msg import ProximityStates
from APFinger.msg import ProximityArray
from APFinger.msg import Proximity
from jsk_topic_tools import ConnectionBasedTransport


class DecomposeMsg(ConnectionBasedTransport):

    def __init__(self):
        super(DecomposeMsg, self).__init__()
        if rospy.has_param('~l_r'):
            l_r = rospy.get_param('~l_r')
        else:
            rospy.logerr('set l_r param')

        self.pubs = {}
        for side_name in ['top', 'side0', 'side1', 'side2', 'side3']:
            pub_name = 'proximity_sensors' + l_r + '/' + side_name
            pub = self.advertise(pub_name, ProximityArray, queue_size=1)
            self.pubs[side_name] = pub

       # Sensitivity of touch/release detection
        if rospy.has_param('~sensitivity'):
            self.sensitivity = rospy.get_param('~sensitivity')
        else:
            self.sensitivity = 1000
        # exponential average weight parameter
        # cut-off frequency for high-pass filter
        if rospy.has_param('~ea_weight'):
            self.ea = rospy.get_param('~ea_weight')
        else:
            self.ea = 0.3
        # low-pass filtered proximity reading
        self.average = np.empty((36,))
        self.average[:] = np.nan
        # FA-II value
        self.fa2 = np.zeros((36,))
        self.n_sensor = {
            'top': 4,
            'side0': 8,
            'side1': 8,
            'side2': 8,
            'side3': 8,
         }

    def subscribe(self):
        self.sub = rospy.Subscriber('input', Proximities, self._cb)

    def unsubscribe(self):
        self.sub.unregister()


    def _cb(self, msg):
        data = msg.proximities
        out_msgs = []
        for i, (raw_data, average, fa2) in enumerate(zip(data, self.average, self.fa2)):
            out_msg = Proximity()
            out_msg.proximity = raw_data
            if np.isnan(average):
                average = raw_data
            out_msg.average = average
            out_msg.fa2derivative = average - raw_data - fa2
            self.fa2[i] = average - raw_data
            out_msg.fa2 = fa2
            if self.fa2[i] < -self.sensitivity:
                out_msg.mode = "T"
            elif self.fa2[i] > self.sensitivity:
                out_msg.mode = "R"
            else:
                out_msg.mode = "0"
            self.average[i] = (1 - self.ea) * average
            self.average[i] += self.ea * raw_data
            out_msgs.append(out_msg)

        stamp = rospy.Time.now()
        for side_name in ['top', 'side0', 'side1', 'side2', 'side3']:
            out_array = ProximityArray()
            out_array.proximities = out_msgs[:self.n_sensor[side_name]]
            out_array.header.frame_id = '/' + side_name
            out_array.header.stamp = stamp
            out_msgs = out_msgs[self.n_sensor[side_name]:]
            self.pubs[side_name].publish(out_array)


if __name__ == '__main__':
    rospy.init_node('decompose_msg')
    app = DecomposeMsg()
    rospy.spin()
