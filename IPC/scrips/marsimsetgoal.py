#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped


# =========================
# Goal (edit here)
# =========================
GOAL_X = -5.89
GOAL_Y = 8.25
GOAL_Z = 1.0

# Publish behavior
TOPIC = "/goal"
FRAME_ID = "world"
PUBLISH_TIMES = 20
PUBLISH_RATE_HZ = 20.0


def main():
    rospy.init_node("marsim_set_goal_once", anonymous=True)
    pub = rospy.Publisher(TOPIC, PoseStamped, queue_size=1)

    # Wait briefly for connections so first message is not lost.
    rospy.sleep(0.3)

    msg = PoseStamped()
    msg.header.frame_id = FRAME_ID
    msg.pose.position.x = GOAL_X
    msg.pose.position.y = GOAL_Y
    msg.pose.position.z = GOAL_Z
    msg.pose.orientation.w = 1.0

    rate = rospy.Rate(PUBLISH_RATE_HZ)
    for _ in range(PUBLISH_TIMES):
        if rospy.is_shutdown():
            break
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo(
        "Goal sent to %s: x=%.3f y=%.3f z=%.3f",
        TOPIC, GOAL_X, GOAL_Y, GOAL_Z
    )


if __name__ == "__main__":
    main()
