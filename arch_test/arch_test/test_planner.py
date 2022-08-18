
import rclpy

from arch_components.planner import Planner, PlannerResponseTypes # ignore-this
from arch_interfaces.msg import Position
from mock import MagicMock

from .utils import TestFrameBroadcaster

def test_transformation_listening():
    rclpy.init()
    planner = Planner()
    br = TestFrameBroadcaster()
    br.broadcast_arena()
    rclpy.spin_once(planner)

    try:
        now = rclpy.time.Time()
        tf = planner.tf_buffer.lookup_transform(
            "world",
            "arena",
            now
        )
        assert (
            tf.transform.translation.x == 1
            and tf.transform.translation.y == 1
            and tf.transform.translation.z == 1
        )
    except Exception as e:
        raise AssertionError(str(e))
    
    br.broadcast_agent("agent_1", loc=(1.0, 1.0, 1.0))
    rclpy.spin_once(planner)
    # Arena needs to be broadcasted at least once relative to world, after agent was broadcasted
    # (If we want to get agent relative to world)
    br.broadcast_arena()
    rclpy.spin_once(planner)

    try:
        now = rclpy.time.Time()
        tf = planner.tf_buffer.lookup_transform(
            "arena",
            "agent_1",
            now
        )
        assert (
            tf.transform.translation.x == 1
            and tf.transform.translation.y == 1
            and tf.transform.translation.z == 1
        )
        tf = planner.tf_buffer.lookup_transform(
            "world",
            "agent_1",
            now
        )
        assert (
            tf.transform.translation.x == 2
            and tf.transform.translation.y == 2
            and tf.transform.translation.z == 2
        )
    except Exception as e:
        raise AssertionError(str(e))
    rclpy.shutdown()

def test_successful_plan_request():
    rclpy.init()
    planner = Planner()
    br = TestFrameBroadcaster()

    mock_goal_handle = MagicMock()
    mock_goal_handle.request.assigned_goals = []
    mock_goal_handle.request.unassigned_goals = [Position(x=550.0, y=0.0, w=1.0)]
    mock_goal_handle.request.unassigned_agents = ["agent_0"]
    mock_goal_handle.is_cancel_requested = False

    br.broadcast_agent("agent_0", (50.0, 0.0, 50.0))
    rclpy.spin_once(planner)
    br.broadcast_arena()
    rclpy.spin_once(planner)

    res = planner.plan_callback(goal_handle=mock_goal_handle)
    assert res.error_msg == PlannerResponseTypes.SUCCESS

    rclpy.shutdown()

def main(args=None):
    """
    Planner default field values
    self.arena_frame: arena
    arena_height: 600
    arena_width: 600
    agent_diameter: 100
    mapf_solver: CBSSolver
    mapf_input: CBSInput
    goal_assigner: SimpleGoalAssigner
    time_limit: Infinity
    """
    tests = [
        test_transformation_listening,
        test_successful_plan_request
    ]
    for test in tests:
        print(f"Running test: {test.__qualname__}", end="")
        test()
        print(" - Success")
        


if __name__ == '__main__':
    main()
