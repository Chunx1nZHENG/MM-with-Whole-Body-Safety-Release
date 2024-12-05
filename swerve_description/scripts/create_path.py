#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Path
from std_msgs import msg
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
import tf


frame_id = "world"


def pathPublisher():
    # pub = rospy.Publisher("/mission_control/planned_path", Path, queue_size=1)

    # rospy.init_node("pathPublisher", anonymous=True)

    # global listener_tf
    # listener_tf = tf.TransformListener()
    # while pub.get_num_connections() == 0 and not rospy.is_shutdown():
    #     rospy.sleep(1)
    # pub.publish(pathMessage())
    pass


def pathMessage():
    time = rospy.Time().now()
    seq = 0
    msg = Path()
    (trans, rot) = listener_tf.lookupTransform(frame_id, "ENDEFFECTOR", rospy.Time(0))
    to_path_start_x = np.linspace(trans[0], 0, 100)
    to_path_start_y = np.linspace(trans[1], 1, 100)
    to_path_start_z = np.linspace(trans[2], 1.5, 100)
    for i in range(0, 100):
        pose = PoseStamped()
        pose.pose.position.x = to_path_start_x[i]
        pose.pose.position.z = to_path_start_z[i]
        pose.pose.position.y = to_path_start_y[i]
        pose.pose.orientation.w = 1
        pose.header = Header()
        pose.header.stamp = time
        pose.header.seq = seq
        pose.header.frame_id = frame_id

        msg.poses.append(pose)
        seq += 1
        time = rospy.Time(time.to_sec() + 0.1)

    path_to_follow = np.linspace(0, 100, 1000)
    msg.header.stamp = rospy.Time(time.to_sec())
    msg.header.frame_id = frame_id

    for value in path_to_follow:
        pose = PoseStamped()
        pose.pose.position.x = 0.1 * value
        pose.pose.position.z = 0.1 * np.sin(value) + 1.5
        pose.pose.position.y = 1
        pose.pose.orientation.w = 1
        pose.header = Header()
        pose.header.stamp = time
        pose.header.seq = seq
        pose.header.frame_id = frame_id

        msg.poses.append(pose)
        seq += 1
        time = rospy.Time(time.to_sec() + 0.1)

    return msg


if __name__ == "__main__":
    try:
        pathPublisher()
    except rospy.ROSInterruptException:
        pass
