#!/usr/bin/env python2

# import rospy
from std_msgs.msg import Float64
from email import message
from pickletools import float8
from std_msgs.msg import String
from avhishekbiswas_roslab.msg import scan_range

from sensor_msgs.msg import LaserScan

try:
    import rospy
except ImportError:
    import sys
    ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
    sys.path.remove(ros_path)
    import rospy
    sys.path.append(ros_path)

rospy.init_node('Laser_listenerSubscriber', anonymous=True)


def Laser_publish(close,far):
    # rospy.loginfo("here in publish")
    msg = scan_range()
    msg.closest_point = close
    msg.farthest_point = far
    close_pub = rospy.Publisher('/Closest_Point', Float64, queue_size=10)
    far_pub = rospy.Publisher('/Farthest_Point', Float64, queue_size=10)
    # message publisher
    s_range = rospy.Publisher('/scan_range', scan_range, queue_size=10)
    
    while not rospy.is_shutdown():
        # rospy.loginfo(msg)
        s_range.publish(msg)
        close_pub.publish(close)
        far_pub.publish(far)

def Lasercallback(data):
    # rospy.loginfo("here in callback")
    closest_point = data.range_min
    farthest_point = data.range_max
    Laser_publish(closest_point,farthest_point)
    
def listener():
    
    rospy.Subscriber('/scan', LaserScan, Lasercallback, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

