
import rclpy
from .utils import *

from arch_components.planner import Planner
from arch_components.manager import Manager
from arch_interfaces.msg import Position

def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    planner.arena_frame = "world"
    arena_height = 4.3
    arena_width = 2.7
    agent_diameter = 0.3
    planner.agent_diameter = agent_diameter
    planner.rows = int(arena_height / agent_diameter)
    planner.cols = int(arena_width / agent_diameter)
    manager = Manager()
    goal_publisher = GoalPublisher()
    agent_executor = AgentDummyExecutor()
    node_list = [
        manager, 
        planner, 
        goal_publisher,
        agent_executor
    ] 
    thread_pool = SingleThreadNodePool()
    thread_pool.add_nodes(*node_list)
    thread_pool.start()
    try:
        while True:
            req = input()
            args = req.split(" ")
            if args[0] == "goal":
                goal_publisher.publish_goal(Position(x=float(args[1]), y=float(args[2]), w=1.0))
            elif args[0] == "agent":
                agent_id = args[1]
                agent_req = args[2]
                agent_executor.send_cli_request(agent_id, agent_req)
            else:
                print("Invalid input")
    except KeyboardInterrupt:
        pass

    thread_pool.stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()