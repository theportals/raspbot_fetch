import rclpy
from rclpy.node import Node
from car import Car
from std_msgs.msg import Int16MultiArray

# yaw = servo 2, left/right camera movement
# pitch = servo 1, up/down camera movement
straight_yaw = 78  # My raspbot's camera seems to be crooked, YMMV
straight_pitch = 90
last_yaw = straight_yaw
last_pitch = straight_pitch

min_yaw = 0
max_yaw = 180
min_pitch = 35
max_pitch = 140

tracking = True
following = False

center_x = 320
center_y = 240
tolerance = 20
angle_step = 5


class FollowNode(Node):
    def __init__(self):
        super().__init__("follower")
        self.sub = self.create_subscription(
            Int16MultiArray, 'ball_tracking', self.listener_callback, 10)
        self.car = Car()
        self.move_cam(2, 0)
        self.move_cam(1, 0)

    def listener_callback(self, circle):
        x = circle[0]
        y = circle[1]
        r = circle[2]
        if x > -1:
            if tracking:
                self.track_ball(x, y, r)
            if following:
                self.follow_ball(x, y, r)

    def move_cam(self, servo_id, angle):
        # Re-write logic such that 0 is straight, negative is left/down, positive is right/up
        if servo_id == 1:  # Pitch
            angle = straight_pitch + angle
            if angle < min_pitch:
                angle = min_pitch
            if angle > max_pitch:
                angle = max_pitch
            self.car.set_servo(1, angle)
            last_pitch = angle
        elif servo_id == 2:  # Yaw
            angle = straight_yaw - angle
            if angle < min_yaw:
                angle = min_yaw
            if angle > max_yaw:
                angle = max_yaw
            self.car.set_servo(2, angle)
            last_yaw = angle

    def track_ball(self, x, y, r):
        # Car will track ball with camera
        if x > (center_x + tolerance):
            self.move_cam(2, last_yaw - angle_step)
        if x < (center_x - tolerance):
            self.move_cam(2, last_yaw + angle_step)
        if y < (center_y - tolerance):
            self.move_cam(1, last_pitch + angle_step)
        if y > (center_y + tolerance):
            self.move_cam(1, last_pitch - angle_step)


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
        print(e)
        node.car.control_car(0, 0)

    node.destroy_node()
    rclpy.shutdown()
