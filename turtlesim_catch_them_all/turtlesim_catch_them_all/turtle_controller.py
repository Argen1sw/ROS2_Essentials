#!/usr/bin/env python3
import rclpy
import random
import math
from rclpy.node import Node


from turtlesim.msg import Pose
from functools import partial
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        # First target will always be a point on top right side
        self.turtle_to_catch = None
        self.pose = None

        # pose subscriber
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_pose, 10
        )

        # cmd_vel publisher
        self.velocity_publisher = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)

        # alive turtles subscriber
        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10
        )

        # Control loop that will be running every 0.01 s
        self.timer_ = self.create_timer(0.01, self.loop_controller)
        self.get_logger().info("Turtle controller node has been started.")

    def callback_pose(self, msg):
        # Extract the pose of the turtle
        self.pose = msg

    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0:
            self.turtle_to_catch = msg.turtles[0]
    
    def loop_controller(self):
        """
        Main control loop of turtle.
        This controls the velocity, based on the current robot's pose
        and target coordinates.
        """
        if self.pose == None or self.turtle_to_catch == None:
            return

        # Calculate the angular velocity needed
        x_coheficient = self.turtle_to_catch.x - self.pose.x
        y_coheficient = self.turtle_to_catch.y - self.pose.y
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
            # target reached!
            twist_message.linear.x = 0.0
            twist_message.angular.z = 0.0
            
            # Call the service to catch the turtle
            self.call_catch_turtle_server(self.turtle_to_catch.name)
            self.turtle_to_catch = None
            
        self.velocity_publisher.publish(twist_message)

    def call_catch_turtle_server(self, turtle_name):

        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server to Kill turtle")

        request = CatchTurtle.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_catch_turtle, turtle_name=turtle_name)
        )

    def callback_call_catch_turtle(self, future, turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().info("Turtle " + turtle_name + " could not be caught")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
