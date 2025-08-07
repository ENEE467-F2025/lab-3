import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from turtle_control_interfaces.action import SendGoal
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import time
import numpy as np

class TurtleControlServer(Node):
    """A simple action server to control the turtle.

    ...
    """
    def __init__(self):
        """Initialize the turtle control server node.

        """

        super().__init__(
            node_name="turtle_control_server_node"
        )

        # class variables for current pose
        self.x: float = None
        self.y: float = None
        self.theta: float = None

        # Example parameter for controller frequency
        self.declare_parameter(
            name="controller_rate",
            value=30. # Hz
        )
        self.controller_rate: float = self.get_parameter("controller_rate").value

        # Example declaration of multiple parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("K_1", 0.5),
                ("K_2", 2.0),
            ]
        )
        self.K_1: float = self.get_parameter("K_1").value
        self.K_2: float = self.get_parameter("K_2").value

        # Create an action server
        self.action_server = ActionServer(
            node=self,
            action_type=SendGoal,
            action_name="send_goal",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback
        )
        # create class variable for the goal request
        self.goal: SendGoal.Goal = None

        # create a separate callback group for the pub/subs
        self.pub_sub_cb_group = MutuallyExclusiveCallbackGroup()
        # create cmd_vel publisher
        self.cmd_vel_publisher = self.create_publisher(
            msg_type=Twist,
            topic="/turtle1/cmd_vel",
            qos_profile=10,
            callback_group=self.pub_sub_cb_group
        )

        # create a pose subscriber
        self.create_subscription(
            msg_type=Pose,
            topic="/turtle1/pose",
            callback=self.pose_callback,
            qos_profile=10,
            callback_group=self.pub_sub_cb_group
        )

    def pose_callback(self, msg: Pose):
        """Process pose data and store it in memory.

        """

        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def goal_callback(self, goal_request: SendGoal.Goal) -> GoalResponse:
        """Accept or reject a client request.

        """

        # verify that we have recieved pose data
        if self.x is None:
            return GoalResponse.REJECT

        x_d = goal_request.goal.x
        y_d = goal_request.goal.y

        # verify that this is a valid goal request
        if x_d < 0 or x_d > 11 or y_d < 0 or y_d > 11:
            # goal out of bounds
            return GoalResponse.REJECT
        
        # valid goal
        self.get_logger().info(f"Accepted Goal Request ({x_d:0.02f}, {y_d:0.02f})")
        self.goal = goal_request

        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute a request for navigation.

        """
        
        self.get_logger().info("Executing Goal")

        x_d = self.goal.goal.x
        y_d = self.goal.goal.y
        goal_tol = self.goal.goal_tol

        while 1:
            x_e = x_d - self.x
            y_e = y_d - self.y
            c = np.cos(self.theta)
            s = np.sin(self.theta)

            theta_e = np.arctan2( y_e*c - x_e*s, x_e*c + y_e*s )
            d_e = np.sqrt( x_e**2 + y_e**2 )

            v = Twist()
            v.linear.x = self.K_1*d_e
            v.angular.z = self.K_2*theta_e

            self.cmd_vel_publisher.publish(v)

            if d_e <= goal_tol:
                break

            time.sleep(1. / self.controller_rate)
        
        goal_handle.succeed()
        result = SendGoal.Result()
        result.success = True
        self.get_logger().info("Goal Reached")

        return result



def main():
    rclpy.init()

    my_node = TurtleControlServer()
    executor = MultiThreadedExecutor()
    executor.add_node(my_node)

    executor.spin()