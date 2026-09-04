import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

TOP_SPEED = 6
ACCELARATION = 6
TURN_SPEED = 9
SENSOR_RANGE = 9

assert TOP_SPEED + ACCELARATION + TURN_SPEED + SENSOR_RANGE == 30

CELL_SIZE = 1.0
WALL_OPEN_THRESH = 0.75
GOAL_OPEN_THRESH = 1.3

DRIVE_SPEED = 0.4
TURN_ANGULAR_SPEED = 1.4
TURN_DURATION = (math.pi / 2) / TURN_ANGULAR_SPEED

# Direction vectors
# 0 = North, 1 = East, 2 = South, 3 = West
row_change = [-1, 0, 1, 0]
col_change = [0, 1, 0, -1]


class StudentSolver(Node):

    def __init__(self):
        super().__init__('student_solver')
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/mouse/scan',
            self.scan_callback,
            10
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            '/mouse/cmd_vel',
            10
        )
        self.row = 0
        self.col = 0
        self.heading = 0
        self.visited = {(0, 0)}
        self.stack = [(0, 0)]
        self.parent_heading = {}
        self.state = 'DECIDE'
        self.target_heading = None
        self.turn_elapsed = 0.0
        self.dist_traveled = 0.0
        self.last_time = self.get_clock().now()
        self.get_logger().info("Student Solver initialized")

    def scan_callback(self, msg):
        d_left = msg.ranges[0]
        d_front = msg.ranges[1]
        d_right = msg.ranges[2]
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        dt = max(dt, 0.0)
        cmd = Twist()

        if self.state == 'DONE':
            self.cmd_pub.publish(cmd)
            return

        if self.state == 'DECIDE':
            if (
                d_left > GOAL_OPEN_THRESH and
                d_front > GOAL_OPEN_THRESH and
                d_right > GOAL_OPEN_THRESH
            ):
                self.get_logger().info(
                    f"Goal found at ({self.row}, {self.col})"
                )
                self.state = 'DONE'
                self.cmd_pub.publish(cmd)
                return

            open_front = d_front > WALL_OPEN_THRESH
            open_left = d_left > WALL_OPEN_THRESH
            open_right = d_right > WALL_OPEN_THRESH
            front_heading = self.heading
            left_heading = (self.heading - 1) % 4
            right_heading = (self.heading + 1) % 4
            candidates = []
            for is_open, heading in (
                (open_front, front_heading),
                (open_left, left_heading),
                (open_right, right_heading)
            ):
                if not is_open:
                    continue

                next_row = self.row + row_change[heading]
                next_col = self.col + col_change[heading]
                if (next_row, next_col) not in self.visited:
                    candidates.append(
                        (heading, next_row, next_col)
                    )
            if candidates:
                heading, next_row, next_col = candidates[0]
                self.visited.add((next_row, next_col))
                self.parent_heading[(next_row, next_col)] = heading
                self.stack.append((next_row, next_col))
                self.row = next_row
                self.col = next_col
                if heading == self.heading:
                    self.heading = heading
                    self.state = 'FORWARD'
                    self.dist_traveled = 0.0
                else:
                    self.target_heading = heading
                    self.turn_elapsed = 0.0
                    self.state = 'TURNING'
            else:
                if len(self.stack) <= 1:
                    self.get_logger().warn(
                        "Maze explored completely"
                    )
                    self.state = 'DONE'
                    self.cmd_pub.publish(cmd)
                    return
                current_cell = self.stack.pop()
                back_heading = (
                    self.parent_heading[current_cell] + 2
                ) % 4
                self.row, self.col = self.stack[-1]
                if back_heading == self.heading:
                    self.heading = back_heading
                    self.state = 'FORWARD'
                    self.dist_traveled = 0.0
                else:
                    self.target_heading = back_heading
                    self.turn_elapsed = 0.0
                    self.state = 'TURNING'
            self.cmd_pub.publish(cmd)
            return

        if self.state == 'TURNING':
            diff = (self.target_heading - self.heading) % 4
            signed_diff = diff if diff <= 2 else diff - 4
            turn_sign = 1 if signed_diff > 0 else -1
            is_about_face = (diff == 2)
            cmd.angular.z = TURN_ANGULAR_SPEED * turn_sign
            cmd.linear.x = 0.0
            self.turn_elapsed += dt
            required = TURN_DURATION * (
                2 if is_about_face else 1
            )

            if self.turn_elapsed >= required:
                self.heading = self.target_heading
                self.state = 'FORWARD'
                self.dist_traveled = 0.0
                cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        if self.state == 'FORWARD':
            cmd.linear.x = DRIVE_SPEED
            have_left_wall = d_left < GOAL_OPEN_THRESH
            have_right_wall = d_right < GOAL_OPEN_THRESH

            if have_left_wall and have_right_wall:
                error = d_left - d_right
                cmd.angular.z = -error * 2.0
            elif have_left_wall:
                error = d_left - (CELL_SIZE / 2.0)
                cmd.angular.z = error * 3.0
            elif have_right_wall:
                error = (CELL_SIZE / 2.0) - d_right
                cmd.angular.z = error * 3.0
            else:
                cmd.angular.z = 0.0
            self.dist_traveled += DRIVE_SPEED * dt
            if self.dist_traveled >= CELL_SIZE:
                self.state = 'DECIDE'
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StudentSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()