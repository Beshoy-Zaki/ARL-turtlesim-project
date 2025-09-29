import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute
from std_srvs.srv import Empty
import math


class TurtleCommander(Node):
    def __init__(self):
        super().__init__('turtle_commander')

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Services
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.clear_client = self.create_client(Empty, '/clear')

        # Subscriptions
        self.create_subscription(String, 'shape_command', self.shape_callback, 10)

        # State
        self.current_shape = None
        self.points = []
        self.idx = 0

        # Timer
        self.timer = self.create_timer(0.02, self.follow_path)

        self.get_logger().info("Turtle Commander ready: send 'heart', 'astroid', 'infinity', or 'stop'")

    # ---------------- Commands ----------------
    def shape_callback(self, msg):
        shape = msg.data.strip().lower()

        if shape == 'stop':
            self.current_shape = None
            self.stop_turtle()
            return

        if shape in ['heart', 'astroid', 'infinity']:
            self.clear_screen()
            self.disable_pen()
            self.teleport_to(5.5, 5.5, 0.0)  
            self.enable_pen()

            if shape == 'heart':
                self.points = self.generate_heart()
            elif shape == 'astroid':
                self.points = self.generate_astroid()
            elif shape == 'infinity':
                self.points = self.generate_infinity()

            self.current_shape = shape
            self.idx = 0
            self.get_logger().info(f"Now drawing {shape} infinitely")
        else:
            self.get_logger().warn(f"Unknown shape: {shape}")

    # ---------------- Drawing Loop ----------------
    def follow_path(self):
        if not self.current_shape or not self.points:
            return

        # Current target point
        x, y = self.points[self.idx]
        self.teleport_to(x, y, 0.0)

        self.idx += 1
        if self.idx >= len(self.points):
            self.idx = 0  # loop forever

    # ---------------- Helpers ----------------
    def stop_turtle(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def clear_screen(self):
        if self.clear_client.wait_for_service(timeout_sec=1.0):
            self.clear_client.call_async(Empty.Request())

    def teleport_to(self, x, y, theta=0.0):
        if self.teleport_client.wait_for_service(timeout_sec=1.0):
            req = TeleportAbsolute.Request()
            req.x = float(x)
            req.y = float(y)
            req.theta = float(theta)
            self.teleport_client.call_async(req)

    def disable_pen(self):
        if self.pen_client.wait_for_service(timeout_sec=1.0):
            req = SetPen.Request()
            req.off = 1
            self.pen_client.call_async(req)

    def enable_pen(self):
        if self.pen_client.wait_for_service(timeout_sec=1.0):
            req = SetPen.Request()
            req.off = 0
            self.pen_client.call_async(req)

    # ---------------- Shapes ----------------
    def generate_heart(self):
        points = []
        for t in [i * 0.02 for i in range(0, 315)]:
            x = 16 * (math.sin(t) ** 3)
            y = 13 * math.cos(t) - 5 * math.cos(2*t) - 2 * math.cos(3*t) - math.cos(4*t)
            x = x * 0.06 + 5.5
            y = -y * 0.06 + 6.0
            points.append((x, y))
        return points

    def generate_astroid(self):
        points = []
        a = 4.5
        for t in [i * 0.02 for i in range(0, 315)]:
            x = a * (math.cos(t) ** 3) + 5.5
            y = a * (math.sin(t) ** 3) + 5.5
            points.append((x, y))
        return points

    def generate_infinity(self):
        points = []
        a = 4.0
        for t in [i * 0.02 for i in range(0, 315)]:
            x = a * math.cos(t) / (1 + math.sin(t)**2)
            y = a * math.sin(t) * math.cos(t) / (1 + math.sin(t)**2)
            x = x * 1.2 + 5.5
            y = y * 1.2 + 5.5
            points.append((x, y))
        return points


# ---------------- Main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
