
from time import sleep
import rclpy
from threading import Thread
from .utils import AgentTestExecutor, GoalPublisher, FixedFrameBroadcaster

from arch_components.planner import Planner
from arch_components.manager import Manager
from arch_interfaces.msg import Position
from geometry_msgs.msg import Vector3

def test_planner_publishing():
    rclpy.init()
    manager = Manager()
    planner = Planner()
    goal_publisher = GoalPublisher()
    a_01_executor = AgentTestExecutor("A_01")
    a_02_executor = AgentTestExecutor("A_02")
    # a_03_executor = AgentTestExecutor("A_03")
    # a_04_executor = AgentTestExecutor("A_04")
    node_list = [
        manager, 
        planner, 
        goal_publisher, 
        a_01_executor,
        a_02_executor,
        FixedFrameBroadcaster("world", "arena", Vector3(x=0.0, y=0.0, z=0.0)),
        FixedFrameBroadcaster("arena", "A_01", Vector3(x=50.0, y=0.0, z=50.0)),
        FixedFrameBroadcaster("arena", "A_02", Vector3(x=150.0, y=0.0, z=150.0)),
        FixedFrameBroadcaster("arena", "A_03", Vector3(x=250.0, y=0.0, z=50.0)),
        # FixedFrameBroadcaster("arena", "A_04", Vector3(x=150.0, y=0.0, z=250.0))
    ]

    executor = rclpy.executors.MultiThreadedExecutor()
    for node in node_list:
        executor.add_node(node)

    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    sleep(0.5)

    goal_publisher.publish_goal(Position(x=550.0, y=550.0, w=1.0))
    a_01_executor.request_and_wait_for_response()
    while manager.unassigned_agents != []:
        sleep(0.5)
    
    goal_publisher.publish_goal(Position(x=450.0, y=550.0, w=1.0))
    goal_publisher.publish_goal(Position(x=350.0, y=550.0, w=1.0))
    goal_publisher.publish_goal(Position(x=250.0, y=550.0, w=1.0))
    a_02_executor.request_and_wait_for_response()
    # a_03_executor.request_and_wait_for_response()
    # sleep(1.1)
    # a_04_executor.request_and_wait_for_response()
    while manager.unassigned_agents != []:
        sleep(0.5)

    for node in node_list:
        node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
    executor_thread.join()


def main(args=None):
    tests = [
        test_planner_publishing,
    ]
    for test in tests:
        print(f"Running test: {test.__qualname__}", end="")
        test()
        print(" - Success")

if __name__ == '__main__':
    main()
