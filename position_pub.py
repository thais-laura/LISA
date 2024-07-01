#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool

def publish_position():
    rospy.init_node('position_publisher')
    pos_pub = rospy.Publisher('/position', Bool, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        pos_pub.publish(True)
        rate.sleep()

if __name__ == '__main__':
  try:
    publish_position()
  except rospy.ROSInterruptException:
    pass