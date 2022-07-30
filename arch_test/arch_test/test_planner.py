
import rclpy
from tf2_ros import TransformException, TransformStamped

from arch_components.planner import Planner # ignore-this
from mock import MagicMock

def test_generate_and_solve_map(planner: Planner):
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

def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
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
    test_generate_and_solve_map(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
