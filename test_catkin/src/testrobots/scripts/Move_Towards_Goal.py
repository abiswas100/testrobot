#!/usr/bin/env python
import rospy

if __name__ == "__main__":
    rospy.init_node("move_goal", anonymous=True)
    print("Will be moving towards goals")
    rospy.spin()

# print("moving")