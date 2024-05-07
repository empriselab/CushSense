#!/usr/bin/env python3

import rospy
from wholearm_skin_ros.msg import TaxelData
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32

class FastTaxelPub:
    def __init__(self) -> None:
        rospy.init_node('fast_taxel_pub', anonymous=True,disable_signals=True)
        self.skin_sub = rospy.Subscriber("/skin/taxels", TaxelData, self.skin_callback1)
        
        self.skin_fast_pub = rospy.Publisher("/skin/taxel_fast", TaxelData, queue_size=10)

        self.prev_taxel_data = TaxelData()
    
    def skin_callback1(self, taxel_data):
        self.prev_taxel_data = taxel_data
        
    def fast_pub(self):
        self.prev_taxel_data.header.stamp = rospy.Time.now()
        self.skin_fast_pub.publish(self.prev_taxel_data)

if __name__ == "__main__":
    fast_taxel_pub = FastTaxelPub()
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        fast_taxel_pub.fast_pub()
        rate.sleep()
