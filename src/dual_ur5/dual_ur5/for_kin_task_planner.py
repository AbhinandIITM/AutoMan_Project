#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from linkattacher_msgs.srv import AttachLink, DetachLink
from gazebo_msgs.srv import GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState

import threading
import time
import copy
import math
import numpy as np


class ForwardKinematicsPlanner(Node):
    def __init__(self):
        super().__init__('for_kin_task_planner')

        # =========================
        # Publishers (BOTH ARMS)
        # =========================
        self.arm1_traj_pub = self.create_publisher(
            JointTrajectory,
            '/arm_1/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.arm2_traj_pub = self.create_publisher(
            JointTrajectory,
            '/arm_2/joint_trajectory_controller/joint_trajectory',
            10
        )

        # =========================
        # Services
        # =========================
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        self.get_entity_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')

        # =========================
        self.speed_factor = 0.5
        self.TOOL_LENGTH = 0.16

        # Arm 2 discard position
        self.discard_pos = {
            'shoulder_pan_joint': 0.00,
            'shoulder_lift_joint': 5.417,
            'elbow_joint': 1.341,
            'wrist_1_joint': -0.441,
            'wrist_2_joint': 0.102,
            'wrist_3_joint': 1.732
        }

        self.thread = threading.Thread(target=self.execute_sequence)
        self.thread.start()

    # ==========================================================
    # GENERIC ARM UTILS
    # ==========================================================
    def get_joint_names(self, arm_id):
        prefix = f'{arm_id}_'
        return [
            prefix + 'shoulder_pan_joint',
            prefix + 'shoulder_lift_joint',
            prefix + 'elbow_joint',
            prefix + 'wrist_1_joint',
            prefix + 'wrist_2_joint',
            prefix + 'wrist_3_joint'
        ]

    def move_arm(self, arm_id, joint_dict, duration_sec=3.0):
        msg = JointTrajectory()
        joint_names = self.get_joint_names(arm_id)
        msg.joint_names = joint_names

        point = JointTrajectoryPoint()

        # Fill positions in correct order
        point.positions = [
            joint_dict[name.split(f"{arm_id}_")[1]]
            for name in joint_names
        ]

        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        msg.points = [point]

        if arm_id == 'arm_1':
            self.arm1_traj_pub.publish(msg)
        else:
            self.arm2_traj_pub.publish(msg)

        time.sleep(duration_sec + 0.5)

    # ==========================================================
    # FK (unchanged)
    # ==========================================================
    def dh_transform(self, a, alpha, d, theta):
        return np.array([
            [math.cos(theta), -math.sin(theta)*math.cos(alpha),  math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
            [math.sin(theta),  math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
            [0,               math.sin(alpha),                   math.cos(alpha),                  d],
            [0,               0,                                 0,                                1]
        ])

    def forward_kinematics(self, joints):
        d = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
        a = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
        alpha = [math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0]

        T = np.eye(4)
        for i in range(6):
            T = np.dot(T, self.dh_transform(a[i], alpha[i], d[i], joints[i]))
        return T

    # ==========================================================
    # SCREW TASK (ARM 2)
    # ==========================================================
    def remove_screw(self, screw_name, approach_angles):
        self.get_logger().info(f"Removing {screw_name}")

        self.move_arm('arm_2', approach_angles, 4.0 * self.speed_factor)

        twist = copy.deepcopy(approach_angles)
        twist['wrist_3_joint'] += 0.0

        self.move_arm('arm_2', twist, 2.0 * self.speed_factor)

        # Attach
        att = AttachLink.Request()
        att.model1_name = 'arm_2'
        att.link1_name = 'arm_2_screwdriver'
        att.model2_name = 'table_laptop'
        att.link2_name = screw_name

        future = self.attach_client.call_async(att)
        while not future.done():
            time.sleep(0.1)

        # Retract
        self.move_arm('arm_2', approach_angles, 2.0 * self.speed_factor)

        # Discard
        self.move_arm('arm_2', self.discard_pos, 4.0 * self.speed_factor)

        # Detach
        det = DetachLink.Request()
        det.model1_name = 'arm_2'
        det.link1_name = 'arm_2_screwdriver'
        det.model2_name = 'table_laptop'
        det.link2_name = screw_name

        future = self.detach_client.call_async(det)
        while not future.done():
            time.sleep(0.1)

        self.get_logger().info(f"{screw_name} removed")

    # ==========================================================
    # MAIN SEQUENCE
    # ==========================================================
    def execute_sequence(self):
        self.get_logger().info("Waiting for services...")
        self.attach_client.wait_for_service()
        self.detach_client.wait_for_service()
        self.get_entity_client.wait_for_service()

        time.sleep(5.0)

        

        # =============================
        # ARM 2 TASKS
        # =============================
        approach_screw_3 = {
            'shoulder_pan_joint': -0.075,
            'shoulder_lift_joint': 5.197,
            'elbow_joint': 2.089,
            'wrist_1_joint': -1.301,
            'wrist_2_joint': 0.102,
            'wrist_3_joint': 1.732
        }

        approach_screw_4 = {
            'shoulder_pan_joint': 0.082,
            'shoulder_lift_joint': 5.281,
            'elbow_joint': 1.887,
            'wrist_1_joint': -1.053,
            'wrist_2_joint': 0.102,
            'wrist_3_joint': 1.732
        }
        away_pos = {
            'shoulder_pan_joint': -0.373,
            'shoulder_lift_joint': 5.417,
            'elbow_joint': 1.341,
            'wrist_1_joint': -0.441,
            'wrist_2_joint': 0.102,
            'wrist_3_joint': 1.732
        }

        self.get_logger().info("Starting ARM 2 screw tasks")

        self.remove_screw('back_plate_screw_3', approach_screw_3)
        self.remove_screw('back_plate_screw_4', approach_screw_4)
        self.move_arm(arm_id='arm_2', joint_dict=away_pos, duration_sec=3.0)
        # =============================
        # ARM 1 TASK (NEW)
        # =============================
        arm1_pose = {
            'shoulder_pan_joint': -0.646,
            'shoulder_lift_joint': 6.004,
            'elbow_joint': 0.560,
            'wrist_1_joint': 1.302,
            'wrist_2_joint': 1.601,
            'wrist_3_joint': 1.732
        }
        arm1_pose_lift = {
            'shoulder_pan_joint': -0.646,
            'shoulder_lift_joint': 5.910,
            'elbow_joint': 0.662,
            'wrist_1_joint': 1.302,
            'wrist_2_joint': 1.601,
            'wrist_3_joint': 1.732
        }
        arm1_pose_slide = {
            'shoulder_pan_joint': -0.146,
            'shoulder_lift_joint': 5.910,
            'elbow_joint': 0.662,
            'wrist_1_joint': 1.302,
            'wrist_2_joint': 1.601,
            'wrist_3_joint': 1.732
        }
        self.get_logger().info("Moving ARM 1 to target pose")
        self.move_arm('arm_1', arm1_pose, 3.0)
        time.sleep(1)
        # =============================
        # 🔥 ATTACH BACK PLATE (ARM 1)
        # =============================
        self.get_logger().info("Attaching back_plate to arm_1...")

        att_req = AttachLink.Request()
        att_req.model1_name = 'arm_1'
        att_req.link1_name = 'arm_1_wrist_3_link'
        att_req.model2_name = 'back_plate'
        att_req.link2_name = 'back_plate_link'

        future = self.attach_client.call_async(att_req)

        while rclpy.ok() and not future.done():
            time.sleep(0.1)

        if future.result() is not None:
            self.get_logger().info("✅ back_plate attached to arm_1")
        else:
            self.get_logger().error("❌ Failed to attach back_plate")
        self.move_arm('arm_1', arm1_pose_lift, 1.0)
        time.sleep(1)
        self.move_arm('arm_1', arm1_pose_slide, 2.0)
        self.get_logger().info("All tasks complete")


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

