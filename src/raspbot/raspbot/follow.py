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
min_pitch = 40
max_pitch = 130

center_x = 320
center_y = 240
x_tolerance = 50
y_tolerance = 38
turn_tolerance = 5

turn_speed_90 = 100
turn_time_90 = 0.46

move_speed_6ft = 50
move_time_6ft = 8
left_scale = 1.1  # My car pulls to the left. YMMV
charge_left_scale = 1.1

target_radius = 120  # apparent radius when ball is ~2.5 inches away
min_radius = 10  # apparent radius when ball is at 6ft
max_radius = 400  # apparent radius when ball is touching car


def approximate_distance(radius):
    # Given the radius, about how far away is the ball?
    # Obtained by measuring radius at various distances, and using a power series trendline
    # Returns approx distance in feet
    return 71.9 * (radius ** -1.06)


class FollowNode(Node):
    def __init__(self):
        super().__init__("follower")
        self.last_yaw = 0
        self.last_pitch = 0
        self.tracking = True
        self.following = False
        self.unchanged_frames = 0
        self.sub1 = self.create_subscription(
            Int16MultiArray, 'ball_tracking', self.track_callback, 10)
        self.car = Car()
        self.move_cam(2, 0)
        self.move_cam(1, 0)

    def track_callback(self, msg):
        x = msg.data[0]
        y = msg.data[1]
        r = msg.data[2]
        self.following = bool(msg.data[3])

        if x > -1 and r >= 10:
            if self.tracking:
                self.track_ball(x, y, r)
            if self.following and self.unchanged_frames >= 10:
                self.follow_ball(x, y, r)

    def move_cam(self, servo_id, angle, remember=True):
        # Re-write logic such that 0 is straight, negative is left/down, positive is right/up
        self.unchanged_frames = 0
        if servo_id == 1:  # Pitch
            adj_angle = straight_pitch + angle
            if adj_angle < min_pitch:
                adj_angle = min_pitch
            if adj_angle > max_pitch:
                adj_angle = max_pitch
                # If the ball is too close, back up
                self.car.control_car(int(-move_speed_6ft * left_scale), -move_speed_6ft)
                time.sleep(0.01)
                self.car.control_car(0, 0)
            self.car.set_servo(1, adj_angle)
            if remember:
                self.last_pitch = angle
        elif servo_id == 2:  # Yaw
            adj_angle = straight_yaw - angle
            if adj_angle < min_yaw:
                adj_angle = min_yaw
            if adj_angle > max_yaw:
                adj_angle = max_yaw
            self.car.set_servo(2, adj_angle)
            if remember:
                self.last_yaw = angle

    def track_ball(self, x, y, r):
        # Car will track ball with camera
        x_dist = abs(x - center_x)
        x_step = math.ceil(x_dist / 100)
        changed = False
        if x > (center_x + x_tolerance):
            self.move_cam(2, self.last_yaw + x_step)
            changed = True
        if x < (center_x - x_tolerance):
            self.move_cam(2, self.last_yaw - x_step)
            changed = True

        y_dist = abs(y - center_y)
        y_step = math.ceil(y_dist / 100)
        if y < (center_y - y_tolerance):
            self.move_cam(1, self.last_pitch - y_step)
            changed = True
        if y > (center_y + y_tolerance):
            self.move_cam(1, self.last_pitch + y_step)
            changed = True

        if not changed:
            self.unchanged_frames += 1

    def follow_ball(self, x, y, r):
        if not self.following:
            return
        if self.last_yaw not in range(-turn_tolerance, turn_tolerance):
            self.fix_heading()
        elif target_radius >= r >= min_radius:
            dist = approximate_distance(r) - approximate_distance(target_radius)
            max_rate = move_time_6ft / 6
            move_time = max_rate * dist
            self.car.control_car(int(move_speed_6ft * left_scale), move_speed_6ft)
            time.sleep(move_time)
            self.car.control_car(0, 0)
        elif self.last_yaw not in range(-turn_tolerance, turn_tolerance):
            self.fix_heading()
        else:
            # print("Ready to hit! (I think)")
            self.circle_and_charge()

    def fix_heading(self):
        # Point car roughly towards ball
        t = abs(self.last_yaw) / 90 * turn_time_90
        if self.last_yaw < 0:
            self.car.control_car(-turn_speed_90, turn_speed_90)
        else:
            self.car.control_car(turn_speed_90, -turn_speed_90)
        time.sleep(t)
        self.car.control_car(0, 0)
        self.move_cam(2, 0)

    def circle_and_charge(self):
        # Car will perform semi-circle routine to get on the other side of the tennis ball, then hit it back towards origin
        # Found with skid-steer formula and good old-fashioned trial-and-error
        self.car.control_car(int(1.1 * 100), -100)
        time.sleep(0.42)
        self.car.control_car(32, 150)
        time.sleep(1.4)
        self.car.control_car(-int(1.1 * 100), 100)
        time.sleep(0.42)
        self.car.control_car(0, 0)
        time.sleep(0.1)
        self.car.control_car(100, 100)
        time.sleep(1)
        self.car.control_car(0, 0)


def main(args=None):
    rclpy.init(args=args)
    node = FollowNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt as e:
        print("Exiting")
        node.car.control_car(0, 0)
