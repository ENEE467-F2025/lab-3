import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rclpy.task import Future

from turtle_control_interfaces.action import SendGoal

class TurtleControllerClient(Node):
    def __init__(self):
        super().__init__("turtle_controller_client_node")

        self._action_client = ActionClient(
            node=self,
            action_type=SendGoal,
            action_name="turtle_goal"
        )

        self._done = False
    
    def send_goal(self, x, y):
        goal_msg = SendGoal.Goal()
        goal_msg.goal.x = x
        goal_msg.goal.y = y
        goal_msg.goal_tol = 0.1

        self._action_client.wait_for_server()

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future: Future):
        goal_handle = future.result()

        if goal_handle.accepted:
            self.get_logger().info("Goal Accepted, Navigating")
        else:
            self.get_logger().info("Goal Rejected")
            self._done = True
            return
        
        future = goal_handle.get_result_async()
        future.add_done_callback(self.result_callback)

    def result_callback(self, future: Future):
        result = future.result().result
        self.get_logger().info(f"Navigation Successful: {result.success}")
        self._done = True
        
    def is_done(self):
        if self._done:
            self._done = False
            return True
        else:
            return False

# end TurtleControllerClient    

def main(args=None):
    rclpy.init(args=args)
    client_node = TurtleControllerClient()

    try:
        while 1:
            data = input("\Goal Position x,y: ")
            x, y = data.split(",")
            try:
                x = float(x)
                y = float(y)
            except ValueError:
                print("Wrong data format!")
                continue
                
            client_node.send_goal(x, y)

            while not client_node.is_done():
                rclpy.spin_once(client_node, timeout_sec=0)
    
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # using print since rclpy context is destroyed after exception
        print("\nExiting...")
        client_node.destroy_node()

if __name__ == "__main__":
    main()