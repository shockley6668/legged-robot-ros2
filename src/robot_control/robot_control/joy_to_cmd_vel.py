import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToCmdVel(Node):
    def __init__(self):
        super().__init__('joy_to_cmd_vel')
        
        # Parameters for axis mapping
        self.declare_parameter('axis_linear_x', 1)  # Left stick vertical
        self.declare_parameter('axis_linear_y', 0)  # Left stick horizontal
        self.declare_parameter('axis_angular_z', 3) # Right stick horizontal
        
        # Scale factors
        self.declare_parameter('scale_linear', 0.5)
        self.declare_parameter('scale_angular', 1.0)

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('Joy to CmdVel node started')

    def joy_callback(self, msg):
        axis_x = self.get_parameter('axis_linear_x').value
        axis_y = self.get_parameter('axis_linear_y').value
        axis_z = self.get_parameter('axis_angular_z').value
        
        scale_l = self.get_parameter('scale_linear').value
        scale_a = self.get_parameter('scale_angular').value

        twist = Twist()
        
        # Basic mapping with safety checks for array length
        if len(msg.axes) > max(axis_x, axis_y, axis_z):
            twist.linear.x = msg.axes[axis_x] * scale_l
            twist.linear.y = msg.axes[axis_y] * scale_l
            twist.angular.z = msg.axes[axis_z] * scale_a
            
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
