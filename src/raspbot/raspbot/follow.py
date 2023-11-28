import rclpy
from rclpy.node import Node
from .car import Car
from std_msgs.msg import Int16MultiArray
import time
import math

# yaw = servo 2, left/right camera movement
# pitch = servo 1, up/down camera movement
straight_yaw = 78  # My raspbot's camera seems to be crooked, YMMV
straight_pitch = 90

min_yaw = 0
max_yaw = 180
min_pitch = 45
max_pitch = 140

center_x = 320
center_y = 240
x_tolerance = 50
y_tolerance = 50
angle_step = 1
hz = 2


class FollowNode(Node):
    def __init__(self):
        super().__init__("follower")
        self.last_yaw = 0
        self.last_pitch = 0
        self.tracking = True
        self.following = False
        self.sub = self.create_subscription(
            Int16MultiArray, 'ball_tracking', self.listener_callback, 10)
        self.car = Car()
        self.move_cam(2, 0)
        self.move_cam(1, 0)

    def listener_callback(self, circle):
        x = circle.data[0]
        y = circle.data[1]
        r = circle.data[2]
#        self.get_logger().info(f"x: {x}, y: {y}, r: {r}")
        if x > -1 and r >= 10:
            if self.tracking:
                self.track_ball(x, y, r)
            if self.following:
                self.follow_ball(x, y, r)

    def move_cam(self, servo_id, angle):
#        self.get_logger().info(f"Setting {servo_id} to {angle}")
#        self.get_logger().info(f"Last angle b: {self.last_yaw}")
        # Re-write logic such that 0 is straight, negative is left/down, positive is right/up
        if servo_id == 1:  # Pitch
            adj_angle = straight_pitch + angle
            if adj_angle < min_pitch:
                adj_angle = min_pitch
            if adj_angle > max_pitch:
                adj_angle = max_pitch
            self.car.set_servo(1, adj_angle)
            self.last_pitch = angle
        elif servo_id == 2:  # Yaw
            adj_angle = straight_yaw - angle
            if adj_angle < min_yaw:
                adj_angle = min_yaw
            if adj_angle > max_yaw:
                adj_angle = max_yaw
            self.car.set_servo(2, adj_angle)
            self.last_yaw = angle
#            self.get_logger().info(f"Last angle a: {self.last_yaw}")
#        time.sleep(1/hz)

    def track_ball(self, x, y, r):
        # Car will track ball with camera
        x_dist = abs(x - center_x)
        x_step = math.ceil(x_dist/100)
        if x > (center_x + x_tolerance):
            self.move_cam(2, self.last_yaw + x_step)
        if x < (center_x - x_tolerance):
            self.move_cam(2, self.last_yaw - x_step)

        y_dist = abs(y - center_y)
        y_step = math.ceil(y_dist/100)
        if y < (center_y - y_tolerance):
            self.move_cam(1, self.last_pitch - y_step)
        if y > (center_y + y_tolerance):
            self.move_cam(1, self.last_pitch + y_step)


    def follow_ball(self, x, y, r):
        # Car will move to follow ball
        pass

    def circle_around(self):
        # Car will perform semi-circle routine to get on the other side of the tennis ball
        pass


def main(args=None):
    rclpy.init(args=args)
    node = FollowNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print("Exiting")
        node.car.control_car(0, 0)

    node.destroy_node()
    rclpy.shutdown()
