#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math

B = 1.0  # constant B value
start_time = None  

def callback(sin_value):
    global start_time
    if start_time is None:
        start_time = rospy.get_time()  
   
    current_time = rospy.get_time()   
    t = current_time - start_time     
    exp_value = math.exp(B * t)       # Calculate e^(Bt)
    output_y = exp_value * sin_value.data  # Output
   
    # Publish the calculated values of y and e^(Bt)
    y_publisher.publish(output_y)
    exp_publisher.publish(exp_value)

    # Log the values
    rospy.loginfo(f"Time: {t:.2f}s, sin(t): {sin_value.data}, e^(Bt): {exp_value}, y: {output_y}")

def exp_subscriber():
    global y_publisher, exp_publisher
    rospy.init_node('exp_subscriber', anonymous=True)

    # Initialize the publishers for the calculated y value and e^(Bt)
    y_publisher = rospy.Publisher('/calculated_y', Float64, queue_size=10)
    exp_publisher = rospy.Publisher('/exp_topic', Float64, queue_size=10)

    # Subscriber to the /sin_topic
    rospy.Subscriber('/sin_topic', Float64, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        exp_subscriber()
    except rospy.ROSInterruptException:
        pass