import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ShapeNode(Node):
    def __init__(self):
        super().__init__('shape_node')
        self.publisher_ = self.create_publisher(String, 'shape_command', 10)
        self.timer = self.create_timer(1.0, self.run)
        self.get_logger().info("Shape Node Started! Available shapes: infinity, heart, astroid, stop")
        
    def get_user_input(self):
        print("\n=== Turtle Shape Drawer ===")
        print("1. infinity - Draw an infinity symbol (lemniscate)")
        print("2. heart - Draw a heart shape")
        print("3. astroid - Draw an astroid shape")
        print("4. stop - Stop the turtle")
        return input("Enter your choice (infinity/heart/astroid/stop): ").strip().lower()
    
    def run(self):
        shape = self.get_user_input()
        if shape in ['infinity', 'heart', 'astroid', 'stop']:
            msg = String()
            msg.data = shape
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published shape: {shape}")
        else:
            self.get_logger().warn("Invalid shape! Choose: infinity, heart, astroid, or stop")

def main(args=None):
    rclpy.init(args=args)
    node = ShapeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
