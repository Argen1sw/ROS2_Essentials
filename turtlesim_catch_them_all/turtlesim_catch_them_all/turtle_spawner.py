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
        self.suffix_name = 0
        
        # Calling the timer every once in a while
        self.timer_ = self.create_timer(0.5, self.turtle_spawner)

    # Method that calls the service /spawn
    def turtle_spawner(self):
        # Increase the suffix, to differentiate the turtles
        self.suffix_name = self.suffix_name+1
        
        # Initialize the client
        client = self.create_client(Spawn, "turtle_spawner_client")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server to spawn turtle")

        prefix = "turtle"
        request = Spawn()
        request.x = random.uniform(0.0, 10.0)
        request.y = random.uniform(0.0, 10.0)
        request.theta = random.uniform(0.0, 2*math.pi)
        request.name = prefix + self.suffix_name
        
        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn_turtle, ))

    def callback_call_spawn_turtle(self, future, turtle_name, x, y, theta):
        try:
            response = future.result()
            self.get_logger().info(str(response.name))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
        
        
        
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
