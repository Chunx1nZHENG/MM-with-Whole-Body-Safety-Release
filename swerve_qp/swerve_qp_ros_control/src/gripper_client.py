import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

client = actionlib.SimpleActionClient('gripper_controller/gripper_cmd', GripperCommandAction)
client.wait_for_server()
goal = GripperCommandGoal()
goal.command.position = 0.1  # 目标位置（开/合）
goal.command.max_effort = 10.0  # 最大力矩
client.send_goal(goal)
client.wait_for_result()