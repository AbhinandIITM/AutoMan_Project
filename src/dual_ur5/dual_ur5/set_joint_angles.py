#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys

class PoseSetter(Node):
    def __init__(self):
        super().__init__('set_initial_angles')
        
        # Accept namespace and joint positions as parameters
        self.declare_parameter('namespace', 'arm_1')
        self.declare_parameter('joint_positions', [0.0, 5.417, 1.341, -0.441, 0.102, 1.732])
        
        ns = self.get_parameter('namespace').value
        topic_name = f'/{ns}/joint_trajectory_controller/joint_trajectory'
        
        self.publisher_ = self.create_publisher(JointTrajectory, topic_name, 10)
        
        # Wait 1.5 seconds to ensure the controller subscriber is fully ready
        self.timer = self.create_timer(1.5, self.publish_trajectory)
        self.published = False

    def publish_trajectory(self):
        if self.published:
            return
            
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = self.get_parameter('joint_positions').value
        # Smoothly move to position over 2 seconds
        point.time_from_start = Duration(sec=2, nanosec=0) 
        
        msg.points = [point]
        
        self.get_logger().info(f'Publishing initial joint angles to {self.publisher_.topic_name}')
        self.publisher_.publish(msg)
        self.published = True
        
        # Allow 1 second for the message to transmit before letting the node die
        self.create_timer(1.0, self.exit_node)

    def exit_node(self):
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = PoseSetter()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()