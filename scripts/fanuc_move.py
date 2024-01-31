#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input
import numpy as np
import quaternion
import random
import sys
import actionlib
import tf
import copy
import rospy
from gazebo_msgs.srv import SpawnModel
import franka_gripper.msg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
    
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        d = dist((x1, y1, z1), (x0, y0, z0))
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm_group"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        group_name_gripper = "hand_group"
        move_group_gripper = moveit_commander.MoveGroupCommander(group_name_gripper)
        
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=10,
        )
        planning_frame = move_group.get_planning_frame()

        eef_link = move_group.get_end_effector_link()

        group_names = robot.get_group_names()

        self.move_group_gripper = move_group_gripper
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


    def go_to_joint_state(self):

        move_group = self.move_group


        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = -pi / 3
        joint_goal[1] = -pi / 2
        joint_goal[2] = pi / 2
        joint_goal[3] = -pi / 2
        joint_goal[4] = pi / 2
        joint_goal[5] = pi

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()

        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, x_length,y_length, z_length=0.2):

        move_group = self.move_group
        
        quaternion = tf.transformations.quaternion_from_euler(0.0, 1.5708, 0.0)  # Поворот на 90 градусов вокруг оси X

        pose_goal = geometry_msgs.msg.Pose()  
        pose_goal.orientation.w = 1.0
        # pose_goal.orientation.x = quaternion[0]
        # pose_goal.orientation.y = quaternion[1]
        # pose_goal.orientation.z = quaternion[2]
        # pose_goal.orientation.w = quaternion[3]
        pose_goal.position.x = x_length  
        pose_goal.position.y = y_length
        pose_goal.position.z = z_length

        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)

        move_group.stop()

        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    # def plan_cartesian_path(self, scale=1):

    #     move_group = self.move_group

    #     waypoints = []

    #     wpose = move_group.get_current_pose().pose
    #     print(wpose)
    #     wpose.position.z -= scale * 0.1  
    #     wpose.position.y += scale * 0.2 
    #     waypoints.append(copy.deepcopy(wpose))

    #     wpose.position.x += scale * 0.1
    #     waypoints.append(copy.deepcopy(wpose))

    #     wpose.position.y -= scale * 0.1 
    #     waypoints.append(copy.deepcopy(wpose))

    #     (plan, fraction) = move_group.compute_cartesian_path(
    #         waypoints, 0.01, 0.0  
    #     )  
    #     return plan, fraction


    # def display_trajectory(self, plan):

    #     robot = self.robot
    #     display_trajectory_publisher = self.display_trajectory_publisher

    #     display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    #     display_trajectory.trajectory_start = robot.get_current_state()
    #     display_trajectory.trajectory.append(plan)

    #     display_trajectory_publisher.publish(display_trajectory)
    # def execute_plan(self, plan):

    #     move_group = self.move_group

    #     move_group.execute(plan, wait=True)


    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):

        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():

            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = box_name in scene.get_known_object_names()

            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False


    def add_box(self, name,x,y,z,timeout=4):
        box_name = self.box_name
        scene = self.scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"#"world" 
        box_pose.pose.orientation.w = 1.0
        # box_pose.pose.position.z = 0.11
        box_name = str(name)
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z
        scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))
        
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, name,timeout=4):

        box_name = str(name)
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        grasping_group = "hand_group"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self,name1, timeout=4):

        box_name = str(name1)
        scene = self.scene
        eef_link = self.eef_link

        scene.remove_attached_object(eef_link, name=box_name)

        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, name, timeout=4):

        box_name = self.box_name
        scene = self.scene

        scene.remove_world_object(str(name))

        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )
    def open_close(self, name_fun):
        move_group_gripper = self.move_group_gripper

        move_group_gripper.set_named_target(name_fun)
        plan_success, plan, planning_time, error_code = move_group_gripper.plan()

        move_group_gripper.execute(plan, wait=True)

    def home(self):
        move_group = self.move_group
        
        move_group.set_named_target("home")
        plan_success, plan, planning_time, error_code = move_group.plan()

        move_group.execute(plan, wait=True)

# def move_gripper(w, s):
#     client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)

#     client.wait_for_server()

#     goal = franka_gripper.msg.MoveGoal(width=w, speed=s)

#     client.send_goal(goal)

#     client.wait_for_result()

#     return client.get_result() 
# def spawn_square_model():

#     rospy.wait_for_service('/gazebo/spawn_sdf_model')
#     spawn_sdf_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

#     with open('/home/anton/my_box.sdf', 'r') as file:
#         model_xml = file.read()

#     initial_pose = geometry_msgs.msg.Pose()
#     initial_pose.position.x = 0.2  
#     initial_pose.position.y = 0.2 
#     initial_pose.position.z = 0.02  

#     spawn_sdf_model("3", model_xml, "", initial_pose, "world")
def main():
    try:
        tutorial = MoveGroupPythonInterfaceTutorial()
        tutorial.open_close("open")
        tutorial.go_to_pose_goal(0.3,0.5)
        tutorial.add_box(1,0.5,0.5,0.0)
        tutorial.add_box(2,0.45,0.5,0.0)
        tutorial.add_box(3,0.55,0.5,0.0)
        tutorial.add_box(4,0.5,0.5,0.05)
        

        tutorial.go_to_pose_goal(0.3,0.5,0.06)
        tutorial.go_to_pose_goal(0.4,0.5,0.06)
        tutorial.attach_box(4)
        tutorial.open_close("close")
        tutorial.go_to_pose_goal(0.4,0.5,0.3)
        tutorial.home()
        tutorial.go_to_pose_goal(0.4,-0.5,0.0)
        tutorial.open_close("open")
        tutorial.detach_box(4)
        tutorial.home()
        tutorial.go_to_pose_goal(0.35,0.5,0.01)
        tutorial.attach_box(2)
        tutorial.open_close("close")
        tutorial.go_to_pose_goal(0.35,0.5,0.3)
        tutorial.home()
        tutorial.go_to_pose_goal(0.4,-0.5,0.055)
        tutorial.open_close("open")
        tutorial.detach_box(2)
        tutorial.home()
        tutorial.go_to_pose_goal(0.4,0.5,0.01)
        tutorial.attach_box(1)
        tutorial.open_close("close")
        tutorial.go_to_pose_goal(0.4,0.5,0.3)
        tutorial.home()
        tutorial.go_to_pose_goal(0.4,-0.5,0.11)
        tutorial.open_close("open")
        tutorial.detach_box(1)
        tutorial.home()
        tutorial.go_to_pose_goal(0.45,0.5,0.01)
        tutorial.attach_box(3)
        tutorial.open_close("close")
        tutorial.go_to_pose_goal(0.45,0.5,0.3)
        tutorial.home()
        tutorial.go_to_pose_goal(0.4,-0.5,0.165)
        tutorial.open_close("open")
        tutorial.detach_box(3)
        tutorial.home()

        # tutorial.go_to_joint_state()

        # tutorial.remove_box(i)
        
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=1)
        # tutorial.display_trajectory(cartesian_plan)
        # tutorial.execute_plan(cartesian_plan)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
