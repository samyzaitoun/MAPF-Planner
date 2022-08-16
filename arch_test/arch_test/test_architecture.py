
from time import sleep
import rclpy
from threading import Thread
from .utils import AgentTestExecutor, GoalPublisher, FixedFrameBroadcaster, SingleThreadNodePool

from arch_components.planner import Planner
from arch_components.manager import Manager
from arch_interfaces.msg import Position
from geometry_msgs.msg import Vector3

def test_planner_publishing():
    rclpy.init()
    planner = Planner()
    manager = Manager()
    goal_publisher = GoalPublisher()
    a_01_executor = AgentTestExecutor("A_01")
    a_02_executor = AgentTestExecutor("A_02")
    a_03_executor = AgentTestExecutor("A_03")
    a_04_executor = AgentTestExecutor("A_04")
    node_list = [
        manager, 
        planner, 
        goal_publisher, 
        a_01_executor,
        a_02_executor,
        a_03_executor,
        a_04_executor,
        FixedFrameBroadcaster("world", "arena", Vector3(x=0.0, z=0.0, y=0.0)),
        FixedFrameBroadcaster("arena", "A_01", Vector3(x=50.0, z=0.0, y=50.0)),
        FixedFrameBroadcaster("arena", "A_02", Vector3(x=150.0, z=0.0, y=150.0)),
        FixedFrameBroadcaster("arena", "A_03", Vector3(x=250.0, z=0.0, y=50.0)),
        FixedFrameBroadcaster("arena", "A_04", Vector3(x=150.0, z=0.0, y=250.0)),
        FixedFrameBroadcaster("arena", "O_01", Vector3(x=150.0, z=0.0, y=450.0)),
        FixedFrameBroadcaster("arena", "O_02", Vector3(x=250.0, z=0.0, y=450.0)),
        FixedFrameBroadcaster("arena", "O_03", Vector3(x=350.0, z=0.0, y=450.0)),
        FixedFrameBroadcaster("arena", "O_04", Vector3(x=450.0, z=0.0, y=450.0)),
        FixedFrameBroadcaster("arena", "O_05", Vector3(x=550.0, z=0.0, y=450.0))
    ]
    thread_pool = SingleThreadNodePool()
    thread_pool.add_nodes(*node_list)

    thread_pool.start()
    sleep(0.5)

    goal_publisher.publish_goal(Position(x=550.0, y=550.0, w=1.0))
    a_01_executor.request_and_wait_for_response()
    while manager.unassigned_agents != []:
        sleep(0.5)
    
    goal_publisher.publish_goal(Position(x=450.0, y=550.0, w=1.0))
    goal_publisher.publish_goal(Position(x=350.0, y=550.0, w=1.0))
    goal_publisher.publish_goal(Position(x=250.0, y=550.0, w=1.0))
    a_02_executor.request_and_wait_for_response()
    a_03_executor.request_and_wait_for_response()
    a_04_executor.request_and_wait_for_response()

    while manager.unassigned_agents != []:
        sleep(0.5)
    
    a_04_executor.disconect_and_reconnect()

    while manager.unassigned_agents != []:
        sleep(0.5)
    
    a_04_executor.disconect_and_reconnect()
    goal_publisher.publish_goal(Position(x=150.0, y=550.0, w=1.0))
    
    while manager.unassigned_agents != []:
        sleep(0.5)
    
    assert len(manager.unassigned_goals) == 1

    thread_pool.stop()
    rclpy.shutdown()


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
