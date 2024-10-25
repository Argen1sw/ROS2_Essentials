#!/usr/bin/env python3
import rclpy
import random
import math
from rclpy.node import Node


from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        # First target will always be a point on top right side
        self.target_x = 7.0
        self.target_y = 7.0
        self.pose = None

        # pose subscriber
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_pose, 10
        )

        # cmd_vel publisher
        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Control loop that will be running every 0.01 s
        self.timer_ = self.create_timer(0.01, self.loop_controller)
        self.get_logger().info("Turtle controller node has been started.")

    def callback_pose(self, msg):
        # Extract the pose of the turtle
        self.pose = msg

    def loop_controller(self):
        """
        Main control loop of turtle.
        This controls the velocity, based on the current robot's pose
        and target coordinates.
        """
        if self.pose == None:
            return

        # Calculate the angular velocity needed
        x_coheficient = self.target_x - self.pose.x
        y_coheficient = self.target_y - self.pose.y
        distance = math.sqrt(
            x_coheficient * x_coheficient + y_coheficient * y_coheficient
        )

        twist_message = Twist()

        if distance >= 0.5:
            # Position
            twist_message.linear.x = 2 * distance

            # Orientation
            goal_theta = math.atan2(y_coheficient, x_coheficient)
            diff = goal_theta - self.pose.theta
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi

            twist_message.angular.z = 6 * diff
        else:
            twist_message.linear.x = 0.0
            twist_message.angular.z = 0.0
            self.update_target()
            self.get_logger().info("Target Updated Sucesfully")

        self.velocity_publisher.publish(twist_message)

    def update_target(self):
        """
        Method that Updates the target of the turtle
        Useful for debugging the turtle trajectory and
        control loop
        """
        self.get_logger().debug("This is a debug message from update_target")
        self.target_x = random.randint(0, 10)
        self.target_y = random.randint(0, 10)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
