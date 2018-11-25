import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_uthai',anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("left_leg")
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)
print "============ Waiting for RVIZ..."
# rospy.sleep(10)
print "============ Starting tutorial "

print "============ Reference frame: %s" % group.get_planning_frame()
print "============ End effector: %s" % group.get_end_effector_link()
print "============ Robot Groups:"
print robot.get_group_names()
print "============ Printing robot state"
print robot.get_current_state()

print "============ Printing endf state"
group.set_end_effector_link("l_foot_ft_link")
# print "enf pose L :" ,group.get_current_pose()
print "============"
# group_variable_values = group.get_current_joint_values()
# print "============ Joint values: ", group_variable_values
# # group_variable_values = [0,0,0.54,0.54,0.2,0]
group_variable_values = [0,0,0,0,0,0]
group.set_joint_value_target(group_variable_values)
plan2=group.plan()
group.execute(plan2)
print "enf pose L :" ,group.get_current_pose()
print "============"
# rospy.sleep(5)
# group.execute(plan2)


# first orient gripper and move forward (+x)
# wpose = geometry_msgs.msg.Pose()
# wpose.orientation.x = 0
# wpose.orientation.y = 0
# wpose.orientation.z = 0
# wpose.orientation.w = 1.0
# wpose.position.x = 0
# wpose.position.y = 0
# wpose.position.z = -0.1

# group.set_pose_target(wpose,"l_foot_ft_link")
# print "goal tolerance = " , group.get_goal_tolerance()
# print "planing time = " , group.get_planning_time()
# group.set_goal_tolerance(0.1)
# group.set_planning_time(10)
# print "goal tolerance = " , group.get_goal_tolerance()
# plan3 = group.plan()
# print "============ Waiting while RVIZ displays plan3..."
# rospy.sleep(5)
# group.execute(plan2)
# print "enf pose L :" ,group.get_current_pose()
# display_trajectory = moveit_msgs.msg.DisplayTrajectory()
# display_trajectory.trajectory_start = robot.get_current_state()
# display_trajectory.trajectory.append(plan2)
# display_trajectory_publisher.publish(display_trajectory)