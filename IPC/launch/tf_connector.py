#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg

def publish_tf():
    rospy.init_node('tf_connector_node')
    br = tf2_ros.StaticTransformBroadcaster()

    # 创建一个转换：camera_init -> map (把 LIO 原点连到 PX4 地图原点)
    t1 = geometry_msgs.msg.TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = "camera_init"
    t1.child_frame_id = "map"
    t1.transform.translation.x = 0.0
    t1.transform.translation.y = 0.0
    t1.transform.translation.z = 0.0
    t1.transform.rotation.w = 1.0

    # 创建一个转换：body -> base_link (把 LIO 机器人主体连到 MAVROS 飞控主体)
    t2 = geometry_msgs.msg.TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "body"
    t2.child_frame_id = "base_link"
    t2.transform.translation.x = 0.0
    t2.transform.translation.y = 0.0
    t2.transform.translation.z = 0.0
    t2.transform.rotation.w = 1.0

    br.sendTransform([t1, t2])
    rospy.spin()

if __name__ == '__main__':
    publish_tf()