#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import math

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inv_kin_node')
        
        # Accept namespace to target the correct arm (e.g. arm_1 or arm_2)
        self.declare_parameter('namespace', 'arm_1')
        ns = self.get_parameter('namespace').value
        
        self.target_pose_sub = self.create_subscription(
            Pose,
            f'/{ns}/target_pose',
            self.pose_callback,
            10
        )
        
        self.traj_pub = self.create_publisher(
            JointTrajectory, 
            f'/{ns}/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # UR5e Kinematics Parameters (Targeting exclusively up to wrist_3_link)
        # Using standard DH parameters for UR5e (in meters)
        self.d = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
        self.a = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
        self.alpha = [math.pi/2, 0.0, 0.0, math.pi/2, -math.pi/2, 0.0]
        
        self.get_logger().info(f"Newton-Raphson Inverse Kinematics node started for namespace: {ns}")
        self.get_logger().info(f"Listening for target poses on: /{ns}/target_pose")

    def get_transform(self, theta, a, d, alpha):
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d],
            [0,   0,      0,     1]
        ])

    def forward_kinematics(self, joints):
        """Computes Forward Kinematics up to wrist_3_link"""
        T = np.eye(4)
        for i in range(6):
            T_i = self.get_transform(joints[i], self.a[i], self.d[i], self.alpha[i])
            T = T @ T_i
        return T

    def get_jacobian(self, joints):
        """Computes the Numerical Jacobian for the UR5e"""
        J = np.zeros((6, 6))
        delta = 1e-5
        T0 = self.forward_kinematics(joints)
        pos0 = T0[:3, 3]
        rot0 = T0[:3, :3]
        
        for i in range(6):
            joints_delta = joints.copy()
            joints_delta[i] += delta
            T_delta = self.forward_kinematics(joints_delta)
            
            # Position derivative
            pos_delta = T_delta[:3, 3]
            J[:3, i] = (pos_delta - pos0) / delta
            
            # Orientation derivative (angular velocity)
            rot_delta = T_delta[:3, :3]
            R_diff = rot_delta @ rot0.T
            
            rx = R_diff[2, 1] - R_diff[1, 2]
            ry = R_diff[0, 2] - R_diff[2, 0]
            rz = R_diff[1, 0] - R_diff[0, 1]
            J[3:, i] = np.array([rx, ry, rz]) / (2 * delta)
            
        return J

    def inverse_kinematics(self, target_T, initial_guess):
        """Newton-Raphson Method with Damped Least Squares for numerical stability"""
        joints = np.array(initial_guess, dtype=float)
        max_iter = 100
        tolerance = 1e-4
        
        for _ in range(max_iter):
            current_T = self.forward_kinematics(joints)
            
            # Position error
            pos_err = target_T[:3, 3] - current_T[:3, 3]
            
            # Orientation error
            R_err = target_T[:3, :3] @ current_T[:3, :3].T
            rx = R_err[2, 1] - R_err[1, 2]
            ry = R_err[0, 2] - R_err[2, 0]
            rz = R_err[1, 0] - R_err[0, 1]
            ori_err = np.array([rx, ry, rz]) / 2.0
            
            error = np.concatenate((pos_err, ori_err))
            
            if np.linalg.norm(error) < tolerance:
                return joints
                
            J = self.get_jacobian(joints)
            
            # Damped least squares to handle singularities gracefully
            lambda_sq = 0.01
            J_pinv = J.T @ np.linalg.inv(J @ J.T + lambda_sq * np.eye(6))
            
            delta_theta = J_pinv @ error
            joints += delta_theta
            
        self.get_logger().warn("IK did not converge entirely, publishing best effort.")
        return joints

    def quaternion_to_matrix(self, q):
        """Converts Geometry_msgs Quaternion to 4x4 Rotation Matrix"""
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z),     2*(x*z + w*y),     0],
            [2*(x*y + w*z),     1 - 2*(x**2 + z**2), 2*(y*z - w*x),     0],
            [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x**2 + y**2), 0],
            [0,                 0,                 0,                 1]
        ])

    def pose_callback(self, msg):
        target_T = self.quaternion_to_matrix(msg.orientation)
        target_T[0, 3] = msg.position.x
        target_T[1, 3] = msg.position.y
        target_T[2, 3] = msg.position.z
        
        # Initial guess (Slightly bent configuration to prevent starting in singularity)
        initial_guess = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        
        self.get_logger().info("Target pose received. Computing Inverse Kinematics...")
        joints = self.inverse_kinematics(target_T, initial_guess)
        
        # Publish joint trajectory
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        point = JointTrajectoryPoint()
        # Normalize angles from -pi to pi for the controller
        point.positions = [(j + math.pi) % (2 * math.pi) - math.pi for j in joints]
        point.time_from_start = Duration(sec=2, nanosec=0) 
        
        traj_msg.points = [point]
        self.traj_pub.publish(traj_msg)
        self.get_logger().info(f"Published IK joint angles: {point.positions}")

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()