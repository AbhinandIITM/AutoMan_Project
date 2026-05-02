#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
from gazebo_msgs.srv import GetEntityState
from linkattacher_msgs.srv import AttachLink, DetachLink
import threading
import time
import math
import numpy as np

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class TaskPlanner(Node):
    def __init__(self):
        super().__init__('task_planner')
        
        # Publishers for Inverse Kinematics targets
        self.arm2_pose_pub = self.create_publisher(Pose, '/arm_2/target_pose', 10)
        
        # Gazebo Services
        self.get_entity_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        
        # Start the sequential task in a separate thread to prevent ROS 2 deadlocks
        self.thread = threading.Thread(target=self.execute_disassembly)
        self.thread.start()

    def get_world_pose(self, entity_name):
        req = GetEntityState.Request()
        req.name = entity_name
        req.reference_frame = 'world'
        
        future = self.get_entity_client.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.1) # Wait synchronously
            
        if future.result() is not None and future.result().success:
            return future.result().state.pose
        return None

    def transform_to_arm2_local(self, world_pose):
        # IK node calculates relative to the arm's base.
        # Arm 2 is spawned at x=-0.2, y=1.0, z=0.72. We must offset the world pose!
        local_pose = Pose()
        local_pose.position.x = world_pose.position.x - (-0.2)
        local_pose.position.y = world_pose.position.y - 1.0
        local_pose.position.z = world_pose.position.z - 0.72
        return local_pose

    def execute_disassembly(self):
        self.get_logger().info("Waiting for Gazebo services...")
        self.get_entity_client.wait_for_service()
        self.attach_client.wait_for_service()
        self.detach_client.wait_for_service()
        
        # Wait for the world models and controllers to fully load
        time.sleep(15.0)
        self.get_logger().info("Starting Generalized Disassembly Sequence...")

        # List of parts to iterate through. 
        # (These must match the link names in your assembled_obj SDF)
        screws_to_remove = [
            'back_plate_screw_1', 
            # 'back_plate_screw_2', 
            # 'back_plate_screw_3'
        ]
        
        # Orientations: Screwdriver pointing straight down, and twisted 90 deg.
        down_quat = euler_to_quaternion(0.0, 1.5708, 0.0)
        twist_quat = euler_to_quaternion(0.0, 1.5708, 1.5708)

        for screw_link in screws_to_remove:
            target_name = f"table_laptop::{screw_link}"
            world_pose = self.get_world_pose(target_name)
            
            if not world_pose:
                self.get_logger().error(f"Failed to locate {target_name}. Skipping.")
                continue

            local_pose = self.transform_to_arm2_local(world_pose)

            self.get_logger().info(f"Target Acquired: {screw_link}. Approaching...")
            
            # 1. Approach Pose (10cm above screw)
            target_pose = Pose()
            target_pose.position.x = local_pose.position.x
            target_pose.position.y = local_pose.position.y
            target_pose.position.z = local_pose.position.z + 0.10
            target_pose.orientation = down_quat
            self.arm2_pose_pub.publish(target_pose)
            time.sleep(4.0)

            # 2. Engage Pose (Touching screw)
            target_pose.position.z = local_pose.position.z + 0.015
            self.arm2_pose_pub.publish(target_pose)
            time.sleep(3.0)

            # 3. Rotate (Unscrew motion)
            self.get_logger().info(f"Unscrewing...")
            target_pose.orientation = twist_quat
            self.arm2_pose_pub.publish(target_pose)
            time.sleep(3.0)

            # 4. Attach Screw to Tool (Using IFRA Link Attacher)
            self.get_logger().info("Magnetizing tool to screw...")
            att_req = AttachLink.Request()
            att_req.model1_name = 'arm_2'
            att_req.link1_name = 'arm_2_screwdriver' # The tool link name
            att_req.model2_name = 'assembled_obj'
            att_req.link2_name = screw_link
            future = self.attach_client.call_async(att_req)
            while rclpy.ok() and not future.done():
                time.sleep(0.1)

            # 5. Retract (Pull screw out)
            target_pose.position.z = local_pose.position.z + 0.15
            target_pose.orientation = down_quat
            self.arm2_pose_pub.publish(target_pose)
            time.sleep(3.0)

            # 6. Move to drop area (Discard pile relative to Arm 2 base)
            target_pose.position.x = 0.3
            target_pose.position.y = -0.3
            self.arm2_pose_pub.publish(target_pose)
            time.sleep(4.0)

            # 7. Detach Screw (Drop it)
            self.get_logger().info(f"Discarding {screw_link}...")
            det_req = DetachLink.Request()
            det_req.model1_name = 'arm_2'
            det_req.link1_name = 'arm_2_screwdriver'
            det_req.model2_name = 'assembled_obj'
            det_req.link2_name = screw_link
            future = self.detach_client.call_async(det_req)
            while rclpy.ok() and not future.done():
                time.sleep(0.1)

            time.sleep(2.0) # Rest before moving to next screw

        self.get_logger().info("Screw removal sequence completed successfully!")

def main(args=None):
    rclpy.init(args=args)
    node = TaskPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()