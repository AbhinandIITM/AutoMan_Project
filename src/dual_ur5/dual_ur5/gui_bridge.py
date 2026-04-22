import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class GuiToGazeboBridge(Node):
    def __init__(self):
        super().__init__('gui_to_gazebo_bridge')
        
        # FIX: Removed leading slashes so topics are relative to the namespace
        self.subscription = self.create_subscription(
            JointState,
            'joint_states_publisher', 
            self.listener_callback,
            10)
            
        # FIX: Removed leading slash
        self.publisher = self.create_publisher(
            JointTrajectory,
            'joint_trajectory_controller/joint_trajectory', 
            10)
            
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

    def listener_callback(self, msg):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        positions = []
        
        # Match the slider values to the correct joints
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
            else:
                positions.append(0.0) # Fallback if joint is missing
        
        point.positions = positions
        
        # Set a very short execution time (0.1 seconds) so the robot updates 
        # quickly as you drag the sliders, without jerky delays.
        point.time_from_start = Duration(sec=0, nanosec=100000000) 
        
        traj_msg.points = [point]
        self.publisher.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GuiToGazeboBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()