#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class OdomPathPublisher:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.path_topic = rospy.get_param("~path_topic", "/odom_visualization/uav_path")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.max_points = int(rospy.get_param("~max_points", 3000))
        self.min_dist = float(rospy.get_param("~min_dist", 0.02))
        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))

        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.last_pose = None
        self.dirty = False

        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=20
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish)

        rospy.loginfo(
            "[odom_path_pub] %s -> %s, max_points=%d, min_dist=%.3f",
            self.odom_topic,
            self.path_topic,
            self.max_points,
            self.min_dist,
        )

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        if self.last_pose is not None:
            dx = p.x - self.last_pose[0]
            dy = p.y - self.last_pose[1]
            dz = p.z - self.last_pose[2]
            if math.sqrt(dx * dx + dy * dy + dz * dz) < self.min_dist:
                return

        pose = PoseStamped()
        pose.header = msg.header
        if not pose.header.frame_id:
            pose.header.frame_id = self.frame_id
        pose.pose = msg.pose.pose

        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = pose.header.frame_id
        self.path.poses.append(pose)
        self.last_pose = (p.x, p.y, p.z)

        if self.max_points > 0 and len(self.path.poses) > self.max_points:
            self.path.poses = self.path.poses[-self.max_points:]

        self.dirty = True

    def publish(self, _event):
        if self.dirty:
            self.path_pub.publish(self.path)
            self.dirty = False


def main():
    rospy.init_node("odom_path_pub")
    OdomPathPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
