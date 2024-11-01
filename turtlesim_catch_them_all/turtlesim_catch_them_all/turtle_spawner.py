# With a timer, Spawn a new turtle at a given rate
# Use service /spawn to do so

# /Spawn Service interface details:
# float32 x
# float32 y
# float32 theta
# string name # Optional.  A unique name will be created and returned if this is empty
# ---
# string name


#!/usr/bin/env python3
import rclpy
import random
import math

from rclpy.node import Node
from turtlesim.srv import Spawn
from functools import partial


class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        # Initializing a counter for the names of the turtles
        self.prefix_name = "turtle"
        self.turtle_counter = 0

        # Calling the timer every once in a while
        self.spawn_turtle_timer_ = self.create_timer(3, self.spawn_turtle)

    def spawn_turtle(self):
        self.turtle_counter += 1
        name = self.prefix_name + str(self.turtle_counter)
        
        self.turtle_spawner(turtle_name=name)
        

    # Method that calls the service /spawn
    def turtle_spawner(self, turtle_name):
        # Initialize the client
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server to spawn turtle")

        request = Spawn.Request()
        request.x = random.uniform(0.0, 10.0)
        request.y = random.uniform(0.0, 10.0)
        request.theta = random.uniform(0.0, 2*math.pi)

        future = client.call_async(request)
        future.add_done_callback(
            partial(
                self.callback_call_spawn_turtle,
                turtle_name=turtle_name,
                x=request.x,
                y=request.y,
                theta=request.theta,
            )
        )

    def callback_call_spawn_turtle(self, future, turtle_name, x, y, theta):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info("Turtle " + str(response.name) + " is now alive")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
