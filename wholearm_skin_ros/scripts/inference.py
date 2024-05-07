#!/usr/bin/env python3

import rospy
from wholearm_skin_ros.msg import TaxelData
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import message_filters
from collections import deque
from scipy import signal
from digitalfilter import LiveSosFilter
import pickle
import scipy.linalg
import matplotlib.pyplot as plt
import numpy as np
from numpy_ringbuffer import RingBuffer
import threading
lock = threading.Lock()

num_taxels = 56  # TODO edit this to match

# put calibration parameters here
model = []
model = np.array(model).reshape(56,5)

class DataCollection:
    def __init__(self) -> None:
        rospy.init_node('calibration', anonymous=True, disable_signals=True)
        self.skin_sub = rospy.Subscriber(
            "/skin/taxel_fast", TaxelData, self.callback)
        self.pub = rospy.Publisher(
            "/calibration", Float32MultiArray, queue_size=10)
        # Taring variables
        self.taring = False
        # ring buffer with capacity 100, extra dimesions (num_taxels) for list of taxel readings
        self.skin_history = RingBuffer(
            capacity=100, dtype=(float, (num_taxels)))
        self.tare_value = [0] * num_taxels
        self.last_value = [0] * num_taxels

        # Filter variables
        sos = signal.iirfilter(
            2, Wn=1.5, fs=60, btype="lowpass", ftype="butter", output="sos")

        self.live_sosfilter = []
        for i in range(num_taxels):
            self.live_sosfilter.append(LiveSosFilter(sos))

    def callback(self, skin_msg):
        if self.skin_history.is_full and not self.taring:
            self.taring = True
            for i in range(num_taxels):
                self.tare_value[i] = int(
                    sum(self.skin_history[:, i]))/len(self.skin_history[:, i])

        if not self.taring:
            print("Taring...")
            self.skin_history.extend(
                np.array(skin_msg.cdc).reshape(1, num_taxels))
        else:
            skin_data = np.zeros(num_taxels)
            for i in range(num_taxels):
                skin_data[i] = skin_msg.cdc[i] - self.tare_value[i]
            msg = Float32MultiArray()
            msg.data = [0] * num_taxels
            self.last_value = skin_data

            # fit data to calibration model
            for i in range(num_taxels):
                msg.data[i] = model[i][0]*skin_data[i]**3 + \
                            model[i][1]*skin_data[i]**2 + \
                            model[i][2]*skin_data[i] + \
                            model[i][3]
                if (msg.data[i] < 0.5):
                    msg.data[i] = 0
                
            self.pub.publish(msg)
            print(msg.data[0])
            
    def filter(self, skin_data, filter_fn):
        return filter_fn(skin_data)

if __name__ == "__main__":
    data_collector = DataCollection()
    lock.acquire()

    rospy.loginfo("Starting inference data collection")
    rospy.spin()