
from time import sleep
from typing import Tuple
from threading import Thread
import rclpy
from geometry_msgs.msg import Vector3

from arch_components.planner import Planner, PlannerResponseTypes
from arch_components.manager import Manager, ManagerRequestTypes, ManagerResponseTypes
from arch_interfaces.msg import Position
from arch_interfaces.srv import AgentRequest
from mock import MagicMock

from .utils import FixedFrameBroadcaster, GoalPublisher, ManagerTestClient, SingleThreadNodePool

def test_transform_broadcast():
    rclpy.init()
    arena_broadcaster = FixedFrameBroadcaster("world", "arena", Vector3(x=0.0, y=0.0, z=0.0), 0.01)
    planner = Planner()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(arena_broadcaster)
    executor.add_node(planner)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    sleep(0.5)
    frame_ids = planner.get_all_frame_ids()
    assert "arena" in frame_ids and len(frame_ids) == 1

    executor.shutdown()
    arena_broadcaster.destroy_node()
    planner.destroy_node()
    rclpy.shutdown()
    executor_thread.join()

def test_agent_requests():
    rclpy.init()
    planner = Planner()
    manager = Manager()

    req_mock = MagicMock()
    req_mock.agent_msg = ManagerRequestTypes.IDLE
    req_mock.agent_id = "agent_1"
    
    # Check if agent is actually listed
    manager.agent_callback(req_mock, MagicMock())

    assert manager.unassigned_agents == ["agent_1"]

    # Try to disconnect an agent ( should succeed )
    req_mock.agent_msg = ManagerRequestTypes.AGENT_DISCONNECTED
    response_mock = MagicMock()
    manager.agent_callback(req_mock, response_mock)

    assert manager.unassigned_agents == []

    planner.destroy_node()
    manager.destroy_node()
    rclpy.shutdown()

def test_goal_input():
    rclpy.init()
    planner = Planner()
    manager = Manager()
    goal_publisher = GoalPublisher()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(planner)
    executor.add_node(manager)
    executor.add_node(goal_publisher)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    goal_1 = Position(x=50.0, y=50.0, w=1.0)
    goal_publisher.publish_goal(goal_1)
    sleep(0.1)
    assert manager.unassigned_goals == [goal_1]
    
    executor.shutdown()
    manager.destroy_node()
    goal_publisher.destroy_node()
    rclpy.shutdown()
    executor_thread.join()

def test_manager_live_request():
    rclpy.init()
    planner = Planner()
    manager = Manager()
    goal_publisher = GoalPublisher()
    client = ManagerTestClient()
    thread_pool = SingleThreadNodePool()

    thread_pool.add_nodes(planner, manager, goal_publisher, client)
    thread_pool.start()
    goal_publisher.publish_goal(Position(x=50.0, y=50.0, w=1.0))
    client.create_request(ManagerRequestTypes.IDLE, "agent_1")

    sleep(0.1)
    # A plan request should've triggered & also fail because no transforms are published
    # Agents are wiped on plan results, Goals are not
    assert len(manager.unassigned_agents) == 0
    assert len(manager.unassigned_goals) == 1
    assert client.response.result().error_msg == ManagerResponseTypes.WAIT_PLAN
    assert manager.future_response.result().error_msg == PlannerResponseTypes.TRANSFORM_FAILURE

    # Add tf publishings
    arena_broadcaster = FixedFrameBroadcaster("world", "arena", Vector3(x=0.0, y=0.0, z=0.0), 0.01)
    agent_1_broadcaster = FixedFrameBroadcaster("arena", "agent_1", Vector3(x=500.0, y=500.0, z=0.0), 0.01)

    thread_pool.add_nodes_after_start(arena_broadcaster, agent_1_broadcaster)
    sleep(0.1)
    client.create_request(ManagerRequestTypes.IDLE, "agent_1")
    sleep(0.1)
    
    # Wait for future response to be ready
    while not manager.future_response.done():
        sleep(0.1)
    sleep(0.1)
    
    # Check that indeed the goal & agent were assigned together
    assert len(manager.assigned_goals) == 1
    assert manager.unassigned_goals == []
    assert manager.unassigned_agents == []

    thread_pool.stop()
    manager.destroy_node()
    goal_publisher.destroy_node()
    client.destroy_node()
    planner.destroy_node()
    rclpy.shutdown()

def main(args=None):
    tests = [
        test_transform_broadcast,
        test_agent_requests,
        test_goal_input,
        test_manager_live_request
    ]
    for test in tests:
        print(f"Running test: {test.__qualname__}", end="")
        test()
        print(" - Success")
        

if __name__ == '__main__':
    main()

