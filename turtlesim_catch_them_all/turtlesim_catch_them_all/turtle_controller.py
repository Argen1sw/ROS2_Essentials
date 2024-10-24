#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")

        # pose subscriber
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_pose, 10
        )
        
        # cmd_vel publisher
        self.velocity_publisher = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10
        )
        
        self.get_logger().info("Turtle controller node has been started.")
        

# Next steps:
#   Create a control_loop, a random target will be created.
#   Robot needs to get to random target.
#   Once the robot has get to target, another target will be created.
#   Repeat
        
    def callback_pose(self, msg):
        message = str(msg.x)
        
        self.get_logger().info(message)
        
    
    def control_loop(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
