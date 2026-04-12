from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleTaskExecutorNode(Node):
    """Simulates dual-arm execution for repetitive dismantling tasks."""

    def __init__(self):
        super().__init__('simple_task_executor')

        self.command_subscriber = self.create_subscription(
            String,
            '/automan/trajectory_command',
            self.command_callback,
            10
        )
        self.status_publisher = self.create_publisher(
            String,
            '/automan/task_status',
            10
        )

        self.command_queue = deque()
        self.active_tasks = {'arm1': None, 'arm2': None}

        self.execution_timer = self.create_timer(0.1, self.process_queue)
        self.get_logger().info(
            'Simple task executor initialized for dual-arm dismantling simulation.'
        )

    def command_callback(self, msg: String):
        parts = msg.data.split('|')
        if len(parts) != 5 or parts[0] != 'AUTO_SIMPLE_TASK':
            self.get_logger().warn(f'Ignoring unsupported command: {msg.data}')
            return

        device_id, model, task_name, cycle_s = parts[1], parts[2], parts[3], int(parts[4])
        self.command_queue.append(
            {
                'device_id': device_id,
                'model': model,
                'task_name': task_name,
                'cycle_s': cycle_s,
            }
        )
        self.get_logger().info(
            f'Queued {task_name} for {device_id} ({model}). '
            f'Queue depth: {len(self.command_queue)}'
        )

    def process_queue(self):
        now_ns = self.get_clock().now().nanoseconds

        # Check for completed tasks on both arms
        for arm in ['arm1', 'arm2']:
            task = self.active_tasks[arm]
            if task and now_ns >= task['deadline']:
                status = String()
                status.data = (
                    f'TASK_COMPLETE|{task["device_id"]}|'
                    f'{task["task_name"]}|{arm}'
                )
                self.status_publisher.publish(status)
                self.get_logger().info(
                    f'Finished {task["task_name"]} on '
                    f'{task["device_id"]} with {arm}.'
                )
                self.active_tasks[arm] = None

        # Assign queued tasks to available arms
        for arm in ['arm1', 'arm2']:
            if self.active_tasks[arm] is None and self.command_queue:
                new_task = self.command_queue.popleft()
                new_task['deadline'] = now_ns + (new_task['cycle_s'] * 1_000_000_000)
                self.active_tasks[arm] = new_task
                self.get_logger().info(
                    f'Starting {new_task["task_name"]} on {new_task["device_id"]} using '
                    f'{arm}. Simulated cycle time: {new_task["cycle_s"]}s.'
                )


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTaskExecutorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
