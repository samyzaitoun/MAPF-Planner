
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

from .utils import FixedFrameBroadcaster, GoalPublisher, ManagerTestClient

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
    manager = Manager()

    req_mock = MagicMock()
    req_mock.agent_msg = ManagerRequestTypes.IDLE
    req_mock.agent_id = "agent_1"
    
    # Check if agent is actually listed
    manager.agent_callback(req_mock, MagicMock())

    assert manager.unassigned_agents == ["agent_1"]

    # Check to see that the bus actually goes off
    planner = Planner()
    manager.unassigned_goals = [Position(x=0.0, y=0.0, w=0.0)]
    manager.timer_callback()

    assert manager.future_response

    # Try to disconnect an agent ( should fail )
    req_mock.agent_msg = ManagerRequestTypes.AGENT_DISCONNECTED
    response_mock = MagicMock()
    manager.agent_callback(req_mock, response_mock)

    assert response_mock.error_msg == ManagerResponseTypes.RETRY
    assert len(manager.unassigned_agents) == 1

    manager.future_response = None
    # Try to disconnect an agent ( should succeed )
    manager.agent_callback(req_mock, response_mock)
    
    assert response_mock.error_msg == ManagerResponseTypes.AGENT_PLAN_CANCELED
    assert len(manager.unassigned_agents) == 0

    planner.destroy_node()
    manager.destroy_node()
    rclpy.shutdown()

def test_goal_input():
    rclpy.init()
    manager = Manager()
    goal_publisher = GoalPublisher()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(manager)
    executor.add_node(goal_publisher)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    goal_1 = Position(x=50.0, y=50.0, w=1.0)
    goal_publisher.publish_goal(goal_1)
    sleep(0.5)
    assert manager.unassigned_goals == [goal_1]
    
    executor.shutdown()
    manager.destroy_node()
    goal_publisher.destroy_node()
    rclpy.shutdown()
    executor_thread.join()

def test_manager_live_request():
    rclpy.init()
    manager = Manager()
    goal_publisher = GoalPublisher()
    client = ManagerTestClient()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(manager)
    executor.add_node(goal_publisher)
    executor.add_node(client)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    goal_publisher.publish_goal(Position(x=50.0, y=50.0, w=1.0))
    client.create_request(ManagerRequestTypes.IDLE, "agent_1")

    sleep(0.5)
    # See that nothing was triggered - the bus is still waiting ( since the service is not ready )
    assert manager.unassigned_agents == ["agent_1"]
    assert client.response.result().error_msg == ManagerResponseTypes.WAIT_PLAN
    assert manager.unassigned_goals != []

    # Add planner & tf publishings
    arena_broadcaster = FixedFrameBroadcaster("world", "arena", Vector3(x=0.0, y=0.0, z=0.0), 0.01)
    agent_1_broadcaster = FixedFrameBroadcaster("arena", "agent_1", Vector3(x=500.0, y=0.0, z=500.0), 0.01)
    planner = Planner()

    executor.add_node(arena_broadcaster)
    executor.add_node(agent_1_broadcaster)
    sleep(0.1)
    executor.add_node(planner)

    # Wait for future response to be ready
    while manager.assigned_goals == []:
        sleep(0.1)

    # Check that indeed the goal & agent were assigned together
    assert (
        len(manager.assigned_goals) == 1 
        and manager.unassigned_goals == [] 
        and manager.unassigned_agents == []
    )

    executor.shutdown()
    manager.destroy_node()
    goal_publisher.destroy_node()
    client.destroy_node()
    arena_broadcaster.destroy_node()
    agent_1_broadcaster.destroy_node()
    planner.destroy_node()
    rclpy.shutdown()
    executor_thread.join()

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

