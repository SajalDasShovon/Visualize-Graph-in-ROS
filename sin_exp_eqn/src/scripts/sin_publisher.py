#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math

def sin_publisher():
    rospy.init_node('sin_publisher', anonymous=True)
    pub = rospy.Publisher('sin_topic', Float64, queue_size=10)
    rate = rospy.Rate(10)  
    
    start_time = rospy.get_time()  

    while not rospy.is_shutdown():
        current_time = rospy.get_time()  
        t = current_time - start_time   # Calculate elapsed time since start
        sin_value = math.sin(t)         #  sin(t)
        rospy.loginfo(f"Publishing sin(t) at time {t:.2f}s: {sin_value}")
        pub.publish(sin_value)
        rate.sleep()

if __name__ == '__main__':
    try:
        sin_publisher()
    except rospy.ROSInterruptException:
        pass
