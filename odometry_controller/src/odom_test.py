import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation as R

# 角度转换为弧度
angle_degrees = 45
angle_radians = np.deg2rad(angle_degrees)

# 创建绕z轴的旋转四元数
rotation = R.from_euler('z', angle_radians)

# 输出四元数
quat = rotation.as_quat()  # 输出格式为 [x, y, z, w]
x, y, z, w = quat
def publish_odom():
    rospy.init_node('odom_publisher', anonymous=True)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # 设置位置
        odom_msg.pose.pose.position = Point(1, 1, 0)
        # 设置姿态（四元数）
        odom_msg.pose.pose.orientation = Quaternion(x, y, z, w )

        # 发布消息
        odom_pub.publish(odom_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_odom()
    except rospy.ROSInterruptException:
        pass