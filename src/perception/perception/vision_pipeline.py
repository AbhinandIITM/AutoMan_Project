from dataclasses import dataclass
from typing import Dict, List
import json
import os
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


@dataclass(frozen=True)
class DisassemblyTask:
    """Represents a single phone-processing step in the workcell."""

    device_id: str
    model: str
    name: str
    category: str
    complexity: str
    repetitive: bool
    robot_cycle_s: int
    human_cycle_s: int


class VisionPipelineNode(Node):
    """Routes only simple repetitive tasks to the automation cell."""

    def __init__(self):
        super().__init__('vision_pipeline')

        self.camera_subscriber = self.create_subscription(
            Image,
            '/workbench_camera/image_raw',
            self.image_callback,
            10
        )
        self.trajectory_trigger = self.create_publisher(
            String,
            '/automan/trajectory_command',
            10
        )
        self.status_subscriber = self.create_subscription(
            String,
            '/automan/task_status',
            self.status_callback,
            10
        )

        self.task_queue = self._build_task_queue()
        self.device_ids = {task.device_id for task in self.task_queue}
        self.completed_devices = set()
        self.automated_tasks = 0
        self.manual_tasks = 0
        self.automation_time_s = 0
        self.human_baseline_time_s = 0
        self.manual_time_s = 0
        self.pending_commands: Dict[str, DisassemblyTask] = {}
        self.reported_summary = False

        self.dispatch_timer = self.create_timer(0.2, self.dispatch_next_task)

        self.get_logger().info(
            'AutoMan perception node initialized with simple-task automation '
            'policy for repetitive dismantling steps.'
        )

    def _build_task_queue(self) -> List[DisassemblyTask]:
        """Create a mixed queue that reflects the case-study task variety."""
        dataset_path = os.path.join(
            get_package_share_directory('perception'),
            'data',
            'dataset.json'
        )
        with open(dataset_path, 'r') as f:
            data = json.load(f)
            
        return [
            DisassemblyTask(**task)
            for task in data
        ]

    def image_callback(self, _msg):
        # Camera input is kept for future perception integration. The current
        # simulation uses a task queue so we can focus on automation policy.
        return

    def dispatch_next_task(self):
        if not self.task_queue:
            if not self.pending_commands and not self.reported_summary:
                self.report_summary()
                self.reported_summary = True
            return

        task = self.task_queue.pop(0)
        self.human_baseline_time_s += task.human_cycle_s

        if self._should_automate(task):
            self.automated_tasks += 1
            self.automation_time_s += task.robot_cycle_s
            command_key = f'{task.device_id}:{task.name}'
            self.pending_commands[command_key] = task

            command_msg = String()
            command_msg.data = (
                f'AUTO_SIMPLE_TASK|{task.device_id}|{task.model}|'
                f'{task.name}|{task.robot_cycle_s}'
            )
            self.trajectory_trigger.publish(command_msg)
            self.get_logger().info(
                f'Automating repetitive dismantling task {task.name} for '
                f'{task.device_id} ({task.model}).'
            )
            return

        self.manual_tasks += 1
        self.manual_time_s += task.human_cycle_s
        reason = self._manual_routing_reason(task)
        self.get_logger().info(
            f'Routing {task.name} on {task.device_id} to a human operator '
            f'because it is {reason}.'
        )

    def _should_automate(self, task: DisassemblyTask) -> bool:
        return (
            task.category == 'dismantling'
            and task.complexity == 'simple'
            and task.repetitive
        )

    def _manual_routing_reason(self, task: DisassemblyTask) -> str:
        reasons = []
        if task.category != 'dismantling':
            reasons.append(f'in the {task.category} category')
        if task.complexity != 'simple':
            reasons.append(task.complexity)
        if not task.repetitive:
            reasons.append('non-repetitive')
        return ' and '.join(reasons) if reasons else 'better suited to a specialist'

    def status_callback(self, msg: String):
        parts = msg.data.split('|')
        if len(parts) < 4 or parts[0] != 'TASK_COMPLETE':
            return

        command_key = f'{parts[1]}:{parts[2]}'
        task = self.pending_commands.pop(command_key, None)
        if task is None:
            return

        self.completed_devices.add(task.device_id)
        self.get_logger().info(
            f'Completed automated task {task.name} on {task.device_id} '
            f'using {parts[3]}.'
        )

    def report_summary(self):
        automated_baseline_s = self.human_baseline_time_s - self.manual_time_s
        # Dual-arm system effectively halves the makespan for the automated portion
        dual_arm_makespan_s = self.automation_time_s / 2.0
        saved_time_s = automated_baseline_s - dual_arm_makespan_s
        productivity_gain = 0.0
        if self.human_baseline_time_s > 0:
            productivity_gain = (
                saved_time_s / self.human_baseline_time_s
            ) * 100.0

        self.get_logger().info('--- AutoMan simple-task simulation summary ---')
        self.get_logger().info(
            f'Devices in study: {len(self.device_ids)} | '
            f'Automated tasks: {self.automated_tasks} | '
            f'Manual tasks: {self.manual_tasks}'
        )
        self.get_logger().info(
            f'Human-only baseline: {self.human_baseline_time_s}s | '
            f'Automation cell time (Dual-Arm): {dual_arm_makespan_s:.1f}s | '
            f'Manual specialist time: {self.manual_time_s}s'
        )
        self.get_logger().info(
            'Time saved by restricting automation to simple repetitive tasks: '
            f'{saved_time_s}s ({productivity_gain:.1f}% productivity gain).'
        )

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
