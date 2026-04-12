import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        # Create publisher for 'motor_cmds' topic
        self.publisher_ = self.create_publisher(Float64MultiArray, 'motor_cmds', 1)
        
        # Subscribe to current joint states to get feedback
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10)
        
        # Timer to publish commands periodically (e.g., 50 Hz for smoothness)
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Target positions provided by user
        self.target_pos = np.array([0.0, 0.08, 0.56, -1.12, -0.57, 0.0, -0.08, -0.56, 1.12, 0.57])
        
        self.current_joint_pos = None
        self.start_joint_pos = None
        self.interpolation_time = 0.0
        self.moving_duration = 2.0 # Move to target in 2 seconds

    def joint_callback(self, msg: JointState):
        # Store current joint positions
        if len(msg.position) >= 10:
            self.current_joint_pos = np.array(msg.position[:10])

    def timer_callback(self):
        if self.current_joint_pos is None:
            self.get_logger().info('Waiting for joint states to determine starting position...', throttle_duration_sec=2.0)
            return

        # Initialize starting position once joint states are received
        if self.start_joint_pos is None:
            self.start_joint_pos = np.copy(self.current_joint_pos)
            self.get_logger().info('Starting joint states received. Beginning smooth transition.')

        # Update movement progress
        self.interpolation_time += self.timer_period
        fraction = min(self.interpolation_time / self.moving_duration, 1.0)
        
        # Linear interpolation between start and target
        interpolated_pos = self.start_joint_pos + (self.target_pos - self.start_joint_pos) * fraction
        
        msg = Float64MultiArray()
        msg.data = interpolated_pos.tolist()
        self.publisher_.publish(msg)
        
        if fraction < 1.0:
            self.get_logger().info(f'Transitioning to target pose... {fraction*100:.1f}%', throttle_duration_sec=1.0)
        else:
            self.get_logger().info('Target pose reached. Maintaining position.', throttle_duration_sec=10.0)

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
