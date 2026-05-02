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
        self.alpha = [math.pi/2, 0, 0, math.pi/2, -math.pi/2, 0]

    def dh_transform(self, a, alpha, d, theta):
        """Computes the Denavit-Hartenberg transformation matrix."""
        return np.array([
            [math.cos(theta), -math.sin(theta)*math.cos(alpha),  math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
            [math.sin(theta),  math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
            [0,               math.sin(alpha),                   math.cos(alpha),                  d],
            [0,               0,                                 0,                                1]
        ])

    def forward_kinematics(self, joints):
        """Computes the forward kinematics to find the end-effector transform."""
        T = np.eye(4)
        for i in range(6):
            T = np.dot(T, self.dh_transform(self.a[i], self.alpha[i], self.d[i], joints[i]))
        return T

    def jacobian(self, joints):
        """Computes the analytical Jacobian matrix."""
        J = np.zeros((6, 6))
        T = np.eye(4)
        z_axes = [np.array([0, 0, 1])]
        positions = [np.array([0, 0, 0])]

        for i in range(6):
            T = np.dot(T, self.dh_transform(self.a[i], self.alpha[i], self.d[i], joints[i]))
            z_axes.append(T[:3, 2])
            positions.append(T[:3, 3])

        end_effector_pos = positions[-1]

        for i in range(6):
            z_i = z_axes[i]
            p_i = positions[i]
            
            # Linear velocity part
            J[:3, i] = np.cross(z_i, end_effector_pos - p_i)
            # Angular velocity part
            J[3:, i] = z_i

        return J

    def inverse_kinematics(self, target_T, initial_guess, max_iterations=1000, tolerance=1e-4):
        """Solves IK using Damped Least Squares (DLS) with Multi-Start to avoid local minima."""
        
        # We define a list of different starting poses. 
        # If the first guess gets stuck, it tries the next one!
        guesses = [
            initial_guess,
            [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],     # Arm pointing straight up
            [1.57, -1.57, 1.57, -1.57, -1.57, 0.0], # Arm turned 90 degrees left
            [-1.57, -1.57, 1.57, -1.57, -1.57, 0.0] # Arm turned 90 degrees right
        ]

        target_pos = target_T[:3, 3]
        target_rot = target_T[:3, :3]
        
        # Damping factor to prevent matrix explosion near singularities
        lambda_sq = 0.01 

        for attempt, guess in enumerate(guesses):
            joints = np.array(guess, dtype=float)

            for step in range(max_iterations):
                current_T = self.forward_kinematics(joints)
                current_pos = current_T[:3, 3]
                current_rot = current_T[:3, :3]

                # Position error
                err_pos = target_pos - current_pos

                # Orientation error (axis-angle representation)
                rot_diff = np.dot(target_rot, current_rot.T)
                angle = np.arccos(np.clip((np.trace(rot_diff) - 1) / 2, -1.0, 1.0))
                
                if angle < 1e-6:
                    err_rot = np.zeros(3)
                else:
                    axis = np.array([
                        rot_diff[2, 1] - rot_diff[1, 2],
                        rot_diff[0, 2] - rot_diff[2, 0],
                        rot_diff[1, 0] - rot_diff[0, 1]
                    ]) / (2 * math.sin(angle))
                    err_rot = angle * axis

                # Combine errors
                error = np.concatenate((err_pos, err_rot))

                # Check convergence
                if np.linalg.norm(error) < tolerance:
                    if attempt > 0:
                        self.get_logger().info(f"IK solved successfully on backup guess #{attempt + 1}")
                    return joints

                # Get Jacobian
                J = self.jacobian(joints)
                
                # Damped Least Squares (DLS) Integration
                # Formula: J_pinv = J.T * inv(J * J.T + lambda^2 * I)
                J_T = J.T
                J_pinv = np.dot(J_T, np.linalg.inv(np.dot(J, J_T) + lambda_sq * np.eye(6)))

                # Update joints (No more need to artificially slow it down with 0.5)
                delta_theta = np.dot(J_pinv, error)
                joints += delta_theta 

        self.get_logger().warn("IK did not achieve perfect convergence. Returning closest approximation.")
        return joints

    def quaternion_to_matrix(self, q):
        """Converts a geometry_msgs Quaternion to a 3x3 rotation matrix."""
        w, x, y, z = q.w, q.x, q.y, q.z
        return np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z,     2*x*z + 2*w*y,     0],
            [2*x*y + 2*w*z,     1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x,     0],
            [2*x*z - 2*w*y,     2*y*z + 2*w*x,     1 - 2*x*x - 2*y*y, 0],
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
        ns = self.get_parameter('namespace').value
        traj_msg.joint_names = [
            f'{ns}_shoulder_pan_joint', f'{ns}_shoulder_lift_joint', f'{ns}_elbow_joint', 
            f'{ns}_wrist_1_joint', f'{ns}_wrist_2_joint', f'{ns}_wrist_3_joint'
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()