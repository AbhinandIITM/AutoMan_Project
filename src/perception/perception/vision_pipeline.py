import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

class VisionPipelineNode(Node):
    def __init__(self):
        super().__init__('vision_pipeline')
        
        # Subscribe to the simulated camera over the workbench
        self.camera_subscriber = self.create_subscription(
            Image,
            '/workbench_camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher to trigger MoveIt trajectories or handoff
        self.trajectory_trigger = self.create_publisher(
            String,
            '/automan/trajectory_command',
            10
        )
        self.get_logger().info("AutoMan Perception Node Initialized.")

    def image_callback(self, msg):
        # AI Vision logic will go here
        # Example: Run PyTorch inference on 'msg' to identify phone brand
        
        phone_detected = "Standard_Model_A" # Placeholder
        
        if phone_detected:
            # Trigger the kinematic sequence for this specific model
            command_msg = String()
            command_msg.data = f"EXECUTE_DISASSEMBLY_{phone_detected}"
            self.trajectory_trigger.publish(command_msg)
            
            # Throttle output to avoid spamming the console
            self.get_logger().info(f"Published command: {command_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionPipelineNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()