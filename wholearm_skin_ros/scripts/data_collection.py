#!/usr/bin/env python3

import rospy
from wholearm_skin_ros.msg import TaxelData
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32
import message_filters
from collections import deque
from scipy import signal
from digitalfilter import LiveSosFilter
import pickle
from std_srvs.srv import SetBool, SetBoolResponse
import threading 
lock = threading.Lock()

# TODO: specify the taxel number and the file name
taxel = 36
filename = "link3_36"

class DataCollection:
    def __init__(self) -> None:
        rospy.init_node('data_collection', anonymous=True,disable_signals=True)
        s = rospy.Service('data_collection/reset_skin_bias', SetBool, self.tare_skin_data)
        self.ft_sub = message_filters.Subscriber("/ft_sensor/netft_data", WrenchStamped)
        self.skin_sub = message_filters.Subscriber("/skin/taxel_fast", TaxelData)
        self.sync_sub = message_filters.ApproximateTimeSynchronizer([self.ft_sub, self.skin_sub],
                                                               queue_size=10, slop=0.1)
        self.pub = rospy.Publisher("/calibration", Float32, queue_size=10)
        self.sync_sub.registerCallback(self.sync_callback)
        self.calibration_data = {'ft': [], 'skin': [], 'ft_time': [], 'skin_time': []}

        self.skin_history = deque(maxlen=1500)
        self.tare_value = 0
        self.taring = False
        self.prev_val = 0

        # filter variables
        sos = signal.iirfilter(2, Wn=1.5, fs=60, btype="lowpass",
                             ftype="butter", output="sos")
        self.live_sosfilter = LiveSosFilter(sos)
        self.filter_count = 0

        # add magnitude filter: maybe just use this for non-conductive material
        self.magnitude_filter = False
        self.max_value = 500

    def sync_callback(self, ft_msg, skin_msg):
        if len(self.skin_history) == self.skin_history.maxlen and not self.taring:
            self.taring = True
            self.tare_value = int(sum(self.skin_history)/len(self.skin_history))
            self.filter_count = 0

        self.filter_count += 1
        skin_data = self.filter(skin_msg.cdc[taxel] - self.tare_value)

        if not self.taring:
            self.skin_history.append(skin_data)
        elif self.filter_count > 300:
            msg = Float32()
            msg.data = skin_data
            self.pub.publish(msg)
            print("FT: ", ft_msg.wrench.force.z, " Skin: ", skin_data, "Time diff: ", ft_msg.header.stamp - skin_msg.header.stamp)
            self.store_data(ft_msg.wrench.force.z, skin_data, skin_msg.header.stamp.to_sec(), ft_msg.header.stamp.to_sec())

    def tare_skin_data(self, req):
        if req.data is True:
            self.taring = False
            self.skin_history.clear()
            self.tare_value = 0
            self.filter_count = 0
            print ("Taring skin data")
            self.calibration_data['ft'].clear()
            self.calibration_data['skin'].clear()
            self.calibration_data['ft_time'].clear()
            self.calibration_data['skin_time'].clear()
            return SetBoolResponse(True, "Started taring process")

    def filter(self, skin_data):
        val = self.live_sosfilter(skin_data)
        if self.magnitude_filter and self.taring:
            if skin_data > self.max_value:
                return self.max_value
        return val

    def calibrate(self):
        pass

    def store_data(self, ft_data, skin_data, ft_time, skin_time):
        self.calibration_data['ft'].append(ft_data)
        self.calibration_data['skin'].append(skin_data)
        self.calibration_data['ft_time'].append(ft_time)
        self.calibration_data['skin_time'].append(skin_time)

    def save_data(self):
        rospy.loginfo('Saving calibration data') 
        with open("data_collection_" + filename + ".pickle", 'wb') as handle: 
            pickle.dump(self.calibration_data, handle, protocol=pickle.HIGHEST_PROTOCOL)

if __name__ == "__main__":
    data_collector = DataCollection()
    lock.acquire()
    try:
        rospy.loginfo("Starting calibration data collection")
        rospy.spin()
    finally:
        data_collector.save_data()

