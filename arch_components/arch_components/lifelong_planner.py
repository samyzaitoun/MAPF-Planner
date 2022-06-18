
import rclpy
from rclpy.node import Node, Publisher, Subscription

# Plan & Transform still not defined
from arch_interfaces.msg import Paths, Plan, Transform # type: ignore


class LifeLongPlanner(Node):
    curr_frame = None
    publisher: Publisher = None
    plan_subscription: Subscription = None
    tf_subscription: Subscription = None
     
    def __init__(self, path_topic: str, plan_topic: str, tf_topic: str):
        super().__init__('lifelong_planner')
        self.curr_frame = None
        self.publisher = self.create_publisher(Paths, path_topic, 1)
        self.plan_subscription = self.create_subscription(
            Plan,
            plan_topic,
            self.plan_callback,
            10
        )
        self.tf_subscription = self.create_subscription(
            Transform,
            tf_topic,
            self.tf_callback,
            10
        )

    def plan_callback(self, msg):
        # msg is a list of goals (currently abstract)
        # Things to do:
        # 1. Extract the goals from the message
        # 2. Get a frame from the scene
        # 3. Create a state map for the algorithm
        # 4. Solve for paths
        # 5. Extract paths & change them to the appropriate syntax
        # 6. Publish paths
        paths = []

        # Part 6:
        msg_publish = Paths()
        msg_publish.paths = paths
        self.publisher.publish(msg_publish)
        self.get_logger().info("Published paths for executors!")
    
    def tf_callback(self, msg):
        # Assuming frame attribute
        self.curr_frame = msg.frame

    def sample_frame(self):
        return self.curr_frame
    
    def start(self):
        rclpy.spin(self)
