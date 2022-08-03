
from typing import Tuple
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException, TransformStamped, TransformBroadcaster

from arch_components.planner import Planner, PlannerResponseTypes # ignore-this
from arch_interfaces.msg import Position
from mock import MagicMock

class TestFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('test_frame_broadcaster')
        self.br = TransformBroadcaster(self)

    def broadcast_arena(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'arena'
        t.transform.translation.x = 1.0
        t.transform.translation.y = 1.0
        t.transform.translation.z = 1.0
        self.br.sendTransform(t)
    
    def broadcast_agent(self, agent_id: str, loc: Tuple[float, float, float]):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'arena'
        t.child_frame_id = agent_id
        t.transform.translation.x = loc[0]
        t.transform.translation.y = loc[1]
        t.transform.translation.z = loc[2]
        self.br.sendTransform(t)


def test_generate_and_solve_map():
    rclpy.init()
    planner = Planner()

    agents = [MagicMock() for i in range(6)]
    coords = 0
    for agent in agents:
        agent.transform.translation.x = coords
        agent.transform.translation.y = coords
        coords += 100

    coords = 500
    goals = [MagicMock() for i in range(6)]
    for goal in goals:
        goal[1].x = coords
        goal[1].y = 0
        coords -= 100
    obstacles = []
    output = planner.generate_and_solve_map(agents, goals, obstacles)
    assert output
    planner.destroy_node()
    rclpy.shutdown()

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
    planner.destroy_node()
    br.destroy_node()
    rclpy.shutdown()

def test_unequal_goal_to_agent_exception():
    rclpy.init()
    planner = Planner()
    br = TestFrameBroadcaster()
    br.broadcast_arena()
    rclpy.spin_once(planner)
    
    mock_plan_request = MagicMock()
    mock_plan_request.assigned_goals = []
    mock_plan_request.unassigned_goals = []
    mock_plan_request.unassigned_agents = [f"agent_{i}" for i in range(5)]

    agent_list = [(agent_id, (float(100),) * 3) for agent_id in mock_plan_request.unassigned_agents]
    
    # broadcast agents
    for agent_id, loc in agent_list:
        br.broadcast_agent(agent_id, loc)
        rclpy.spin_once(planner)
    
    res = planner.plan_callback(request=mock_plan_request, response=MagicMock())
    assert res.error_msg == PlannerResponseTypes.FAILED_GOAL_ASSIGN

    planner.destroy_node()
    br.destroy_node()
    rclpy.shutdown()

def test_successful_plan_request():
    rclpy.init()
    planner = Planner()
    planner.publisher = MagicMock()
    br = TestFrameBroadcaster()

    mock_plan_request = MagicMock()
    mock_plan_request.assigned_goals = []
    mock_plan_request.unassigned_goals = [Position(x=550.0, y=0.0, w=1.0)]
    mock_plan_request.unassigned_agents = ["agent_0"]

    br.broadcast_agent("agent_0", (50.0, 0.0, 50.0))
    rclpy.spin_once(planner)
    br.broadcast_arena()
    rclpy.spin_once(planner)

    res = planner.plan_callback(request=mock_plan_request, response=MagicMock())
    assert res.error_msg == PlannerResponseTypes.SUCCESS

    planner.destroy_node()
    br.destroy_node()
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
        test_generate_and_solve_map, 
        test_transformation_listening,
        test_unequal_goal_to_agent_exception,
        test_successful_plan_request
    ]
    for test in tests:
        print(f"Running test: {test.__qualname__}", end="")
        test()
        print(" - Success")
        


if __name__ == '__main__':
    main()
