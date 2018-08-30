#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from dvs_msgs.msg import EventArray, Event

import numpy as np
import sys

from . import pushbot

class PushBotRos(pushbot.PushBot):
    def initialize(self):
        self.sensor_publishers = {}
        super(PushBotRos, self).initialize()

        self.action_sub = rospy.Subscriber("action", Float64MultiArray, self.action_callback)
        self.event_pub = rospy.Publisher("dvs/events", EventArray, queue_size=1)

        self.action = [0., 0.]
        self.last_action_time = -float('inf')
        self.max_action_delay = 0.2
        self.filtered_acc = 0.

    def add_sensor(self, name, bit, range, length, base=10):
        super(PushBotRos, self).add_sensor(name, bit, range, length, base)
        self.sensor_publishers[name] = rospy.Publisher(name, Float64MultiArray, queue_size=1)

    def process_retina(self, data):
        # see https://inivation.com/support/hardware/edvs/#event-recording-formats
        packet_size = self.retina_packet_size
        y = data[::packet_size] & 0x7f
        x = data[1::packet_size] & 0x7f
        polarity = data[1::packet_size] & int('10000000', 2)
        polarity[polarity > 0] = 1

        now = rospy.Time.now()
        events = map(lambda (x,y,p): Event(x=x, y=y, polarity=p, ts=now),
                     zip(x,y, polarity))
        msg = EventArray(
            height = 128,
            width = 128,
            events = events
        )
        self.event_pub.publish(msg)

    def process_ascii(self, message):
        try:
            if message[:2] == '-S':
                data = message[2:].split()
                index = int(data[0])
                scale = self.sensor_scale[index]
                base = self.sensor_base[index]
                values = [float(int(x, base))*scale for x in data[1:]]
                self.sensor[index] = values
                self.sensor[self.sensor_map[index]] = values
                msg = Float64MultiArray(data = values)
                self.sensor_publishers[self.sensor_map[index]].publish(msg)
        except:
            print('Error processing "%s"' % message)
            import traceback
            traceback.print_exc()

    def action_callback(self, msg):
        self.action = np.array(msg.data)
        scale = 0.3
        self.action[0:2] = self.action[0:2] * scale # left / right motor
        self.last_action_time = rospy.get_time()

    def control(self, *args):
        if rospy.get_time() - self.last_action_time < self.max_action_delay:
            self.motor(self.action[0], self.action[1])
        else:
            self.motor(0., 0.)
