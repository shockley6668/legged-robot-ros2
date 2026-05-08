import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState, Imu, Joy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Vector3
import os

from .rdk_inference import TinkerRealInference

class InferenceNode(Node):
    def __init__(self):
        super().__init__('inference_node')

        # Parameters
        self.declare_parameter('model_path', '/root/legged-robot/src/robot_control/robot_control/finall.onnx')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        
        self.get_logger().info(f'Loading model from: {model_path}')
        
        try:
            self.inference = TinkerRealInference(model_path)
            self.get_logger().info('Model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            # We might want to exit or handle this, but for now we'll let it crash later if used
            self.inference = None

        # State variables
        self.latest_euler = np.zeros(3, dtype=np.float32) # roll, pitch, yaw
        self.latest_imu_gyro = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        # 10 joints
        self.latest_joint_pos = np.zeros(10, dtype=np.float32)
        self.latest_joint_vel = np.zeros(10, dtype=np.float32)
        
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_dyaw = 0.0

        # State machine
        self.STATE_IDLE = -1
        self.STATE_SQUAT = 0
        self.STATE_TRANSITIONING = 1
        self.STATE_STAND = 2
        self.STATE_INFERENCE = 3

        self.state = self.STATE_IDLE
        self.next_state = self.STATE_IDLE

        self.target_pos_stand = np.array([0.0, 0.08, 0.56, -1.12, -0.57, 0.0, -0.08, -0.56, 1.12, 0.57], dtype=np.float32)
        # Power-on squat pos based on user data
        self.target_pos_squat = np.array([
            -0.039101600646972656, 0.032998085021972656, 1.9686040878295898, -2.4729156494140625, -0.49267578125,
            -0.00934600830078125, 0.10738563537597656, -1.8488216400146484, 2.585068702697754, 0.6300067901611328
        ], dtype=np.float32)
        
        self.transition_start_pos = None
        self.transition_target_pos = None
        self.transition_progress = 0.0
        self.transition_step = 0.0

        self.a_pressed_last = False
        self.b_pressed_last = False
        
        # Subscriptions
        self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Publisher
        self.motor_cmd_pub = self.create_publisher(Float64MultiArray, 'motor_cmds', 10)

        # Timer for inference loop (50Hz)
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        self.get_logger().info('Inference node started')

    def joint_callback(self, msg: JointState):
        # Taking the first 10 joints assuming they match the 10 motors in order
        # bridge_node publishes joint_0 to joint_9 in order.
        if len(msg.position) >= 10:
            self.latest_joint_pos = np.array(msg.position[:10], dtype=np.float32)
        if len(msg.velocity) >= 10:
            self.latest_joint_vel = np.array(msg.velocity[:10], dtype=np.float32)

    def imu_callback(self, msg: Imu):
        # Convert quaternion to euler angles
        q = msg.orientation
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2 # use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        self.latest_euler = np.array([roll, pitch, yaw], dtype=np.float32)
        self.latest_imu_gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], dtype=np.float32)

    def cmd_callback(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_dyaw = msg.angular.z

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) < 3:
            return
            
        a_pressed = msg.buttons[0] == 1
        b_pressed = msg.buttons[1] == 1
        
        if a_pressed and not self.a_pressed_last:
            if self.state == self.STATE_IDLE:
                # 初始待机时，按A先过渡到蹲下状态
                if self.target_pos_squat is not None:
                    self.start_transition(self.target_pos_squat, self.STATE_SQUAT, 2.0)
            elif self.state == self.STATE_SQUAT:
                if self.target_pos_squat is not None:
                    self.start_transition(self.target_pos_stand, self.STATE_STAND, 2.0)
            elif self.state in [self.STATE_STAND, self.STATE_INFERENCE]:
                if self.target_pos_squat is not None:
                    self.start_transition(self.target_pos_squat, self.STATE_SQUAT, 2.0)
                
        if b_pressed and not self.b_pressed_last:
            if self.state == self.STATE_STAND:
                self.state = self.STATE_INFERENCE
                self.get_logger().info('Starting inference')
            elif self.state == self.STATE_INFERENCE:
                self.start_transition(self.target_pos_stand, self.STATE_STAND, 1.0)
                self.get_logger().info('Stopping inference')
                
        self.a_pressed_last = a_pressed
        self.b_pressed_last = b_pressed

    def start_transition(self, target_pos, next_state, duration):
        self.transition_start_pos = np.copy(self.latest_joint_pos)
        self.transition_target_pos = np.copy(target_pos)
        self.transition_step = self.dt / duration
        self.transition_progress = 0.0
        self.next_state = next_state
        self.state = self.STATE_TRANSITIONING
        self.get_logger().info(f'Transitioning to state {next_state}')

    def timer_callback(self):
        if self.inference is None:
            return

        cmd_msg = Float64MultiArray()

        if self.state == self.STATE_IDLE:
            # 初始状态什么都不发
            return

        elif self.state == self.STATE_SQUAT:
            if self.target_pos_squat is not None:
                cmd_msg.data = self.target_pos_squat.tolist()
                self.motor_cmd_pub.publish(cmd_msg)

        elif self.state == self.STATE_TRANSITIONING:
            self.transition_progress += self.transition_step
            if self.transition_progress >= 1.0:
                self.transition_progress = 1.0
                cmd_msg.data = self.transition_target_pos.tolist()
                self.motor_cmd_pub.publish(cmd_msg)
                self.state = self.next_state
                self.get_logger().info(f'Reached state {self.state}')
            else:
                interp = self.transition_start_pos + (self.transition_target_pos - self.transition_start_pos) * self.transition_progress
                cmd_msg.data = interp.tolist()
                self.motor_cmd_pub.publish(cmd_msg)

        elif self.state == self.STATE_STAND:
            cmd_msg.data = self.target_pos_stand.tolist()
            self.motor_cmd_pub.publish(cmd_msg)

        elif self.state == self.STATE_INFERENCE:
            # 如果有速度指令，最小速度设置为0.15
            cmd_vx = self.cmd_vx
            if abs(cmd_vx) > 0.01 and abs(cmd_vx) < 0.15:
                cmd_vx = np.sign(cmd_vx) * 0.15
            
            try:
                target_q = self.inference.get_action(
                    self.latest_euler,
                    self.latest_imu_gyro,
                    self.latest_joint_pos,
                    self.latest_joint_vel,
                    cmd_vx,
                    self.cmd_vy,
                    self.cmd_dyaw
                )
                cmd_msg.data = target_q.tolist()
                self.motor_cmd_pub.publish(cmd_msg)
            except Exception as e:
                self.get_logger().error(f'Error during inference: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
