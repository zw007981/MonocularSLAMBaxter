#!/usr/bin/env python
import sys
from copy import copy
import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
import baxter_interface
import numpy
import math
from moveit_commander import conversions
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in
                                             ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

def transform(rpy_pose):
    limb = 'left'
    node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    ik_service = rospy.ServiceProxy(node, SolvePositionIK)
    quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")
    ik_request = SolvePositionIKRequest()
    ik_request.pose_stamp.append(quaternion_pose)
    ik_response = ik_service(ik_request)
    # limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
    # ik_response.joints[0].position = [s0, s1, e0, e1, w0, w1, w2], this is also the target point we want.
    return(ik_response.joints[0].position)

def main():
    limb = 'left'
    t1 = 6
    rpy_pose = [0.7, 0.6, 0.4, -1.0 * math.pi, 0, 0]

    rospy.init_node("rsdk_joint_trajectory_client_%s" % (limb,))
    traj = Trajectory(limb)
    rospy.on_shutdown(traj.stop)
    limb_interface = baxter_interface.limb.Limb(limb)
    current_angles = [limb_interface.joint_angle(
        joint) for joint in limb_interface.joint_names()]
    traj.add_point(current_angles, 0.0)
    pose = transform(rpy_pose)
    traj.add_point(pose, 2 * t1)
    traj.start()
    traj.wait(10 * t1 + 3)
    traj.clear(limb)

    while 1:
        limb_interface = baxter_interface.limb.Limb(limb)
        current_angles = [limb_interface.joint_angle(
            joint) for joint in limb_interface.joint_names()]
        traj.add_point(current_angles, 0.0)

        rpy_pose[1] = rpy_pose[1] - 0.1
        pose = transform(rpy_pose)
        traj.add_point(pose, t1)

        traj.start()
        traj.wait(t1 + 1.2)
        traj.clear(limb)


if __name__ == "__main__":
    main()
