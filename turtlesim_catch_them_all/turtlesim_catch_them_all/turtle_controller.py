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

        # self.target_x = random.randint(0, 10)
        # self.target_y = random.randint(0, 10)
        self.target_x = 7
        self.target_y = 7
        self.pose = None

        # pose subscriber
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_pose, 10
        )

        # cmd_vel publisher
        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.get_logger().info("Turtle controller node has been started.")

    # Next steps:
    #   Create a control_loop, a random target will be created.
    #   Robot needs to get to random target.
    #   Once the robot has get to target, another target will be created.
    #   Repeat

    def callback_pose(self, msg):
        # Extract the pose of the turtle
        self.pose = msg
        
        pose_x = self.pose.x 
        pose_y = self.pose.y
        # self.target_x = msg.x
        # self.target_y = msg.y

        # Calculate the angular velocity needed
        x_coheficient = pose_x - self.target_x
        y_coheficient = pose_y - self.target_y

        distance = math.sqrt(x_coheficient * x_coheficient + y_coheficient * y_coheficient)
        twist_message = Twist()

        if(distance >= 0.5):
            
            # Position
            twist_message.linear.x = 2*distance
            
            # Orientation
            goal_theta = math.atan2(y_coheficient, x_coheficient)
            diff = goal_theta - self.pose.theta
            if diff > math.pi:
                diff -= 2*math.pi
            else:
                diff += 2*math.pi
            
            twist_message.angular.z = 6*diff
        else:
            twist_message.linear.x = 0.0
            twist_message.angular.z = 0.0
            self.update_target
            self.get_logger().info("Target Updated Sucesfully")            
        
        
        
        self.velocity_publisher.publish(twist_message)
        

        # # No rotation needed angular velocity = 0
        # if x_coheficient == 0 or y_coheficient == 0:
        #     angle_of_rotation = 0
        # else:  # Calculates the angle of rotation needed
        # angle_of_rotation = math.atan2(y_coheficient / x_coheficient)
            
        # self.get_logger().info(str(angle_of_rotation))

    def update_target(self):
        self.target_x = random.randint(0, 10)
        self.target_y = random.randint(0, 10)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
