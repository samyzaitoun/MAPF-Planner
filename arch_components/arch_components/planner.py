
import math
from typing import Iterable, List, Tuple, Type
import yaml

import rclpy
from rclpy.node import Node, Publisher, Subscription

from arch_interfaces.msg import AssignedGoal, Position, AssignedPath, AgentPaths
from arch_interfaces.srv import PlanRequest
from geometry_msgs.msg import Transform, Vector3

from tf2_ros import TransformException, TransformStamped
from tf2_ros.buffer import Buffer, Duration
from tf2_ros.transform_listener import TransformListener

from mapf_solver.Abstract_objects.waypoint import WayPoint
from mapf_solver.Abstract_objects.mapf_solver import MAPFSolver, MAPFInput, MAPFOutput
from mapf_solver.Abstract_objects.path import Path
from mapf_solver.Abstract_objects.map_instance import MapfInstance
from mapf_solver.Concrete_objects.concrete_waypoints import TimedWayPoint

from mapf_solver.MAPFSolvers.pbs import CBSInput, CBSSolver, PBSInput, PBSSolver
from mapf_solver.MAPFSolvers.dict_cbs import DictCBSSolver
from mapf_solver.MAPFSolvers.prioritized import PrioritizedPlanningSolver

from .goal_assigner import GoalAssigner, AssigningGoalsException, SimpleGoalAssigner


NO_TIME_LIMIT = math.inf

SOLVER_DICT = {
    "CBSSolver": CBSSolver,
    "PBSSolver": PBSSolver,
    "DictCBSSolver": DictCBSSolver,
    "PrioritizedPlanningSolver": PrioritizedPlanningSolver,
    "CBSInput": CBSInput,
    "PBSInput": PBSInput,
}

ASSIGNER_DICT = {
    "SimpleGoalAssigner": SimpleGoalAssigner
}


class Planner(Node):
    """
    The Planner component node implementation
    """

    publisher: Publisher = None
    plan_subscription: Subscription = None
    tf_subscription: Subscription = None

    def __init__(self):
        super().__init__("planner_component")

        # Retrieve launch parameters
        self.get_launch_parameters()

        self.plan_srv = self.create_service(
            PlanRequest, "plan_request", self.plan_callback
        )
        self.publisher = self.create_publisher(AgentPaths, "agent_paths", 1)

        self.rows = int(self.arena_height / self.agent_diameter)
        self.cols = int(self.arena_width / self.agent_diameter)

        # Setup buffer for published transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.mapf_solver: MAPFSolver = SOLVER_DICT[self.mapf_solver_class](self.time_limit)

        self.goal_assigner: GoalAssigner = ASSIGNER_DICT[self.goal_assigner_class]()

        self.get_logger().info("Finished Initializing planner component, Waiting for plan requests.")

    def get_launch_parameters(self) -> None:

        ##  Arena transform frame name
        self.declare_parameter("tf_tag_arena", "arena")
        self.arena_frame: str = (
            self.get_parameter("tf_tag_arena").get_parameter_value().string_value
        )

        ## Arena height
        self.declare_parameter("arena_height", 600)
        self.arena_height: int = (
            self.get_parameter("arena_height").get_parameter_value().integer_value
        )

        ## Arena width
        self.declare_parameter("arena_width", 600)
        self.arena_width: int = (
            self.get_parameter("arena_width").get_parameter_value().integer_value
        )

        ## Agent diameter
        self.declare_parameter("agent_diameter", 100)
        self.agent_diameter: int = (
            self.get_parameter("agent_diameter").get_parameter_value().integer_value
        )

        ## MAPF Algorithm Class
        self.declare_parameter("mapf_solver", "CBSSolver")
        self.mapf_solver_class: str = (
            self.get_parameter("mapf_solver").get_parameter_value().string_value
        )
        if self.mapf_solver_class not in SOLVER_DICT:
            raise AssertionError("Solver class not in Planner dictionary. Please update it (inside Planner.py).")

        ## MAPF Algorithm Input Class
        self.declare_parameter("mapf_input", "CBSInput")
        self.mapf_input_class: str = (
            self.get_parameter("mapf_input").get_parameter_value().string_value
        )
        if self.mapf_input_class not in SOLVER_DICT:
            raise AssertionError("Solver Input class not in Planner dictionary. Please update it (inside Planner.py).")

        ## goal assign Algorithm Class
        self.declare_parameter("goal_assigner", "SimpleGoalAssigner")
        self.goal_assigner_class: str = (
            self.get_parameter("goal_assigner").get_parameter_value().string_value
        )
        if self.goal_assigner_class not in ASSIGNER_DICT:
            raise AssertionError("Assigner class not in Planner dictionary. Please update it (inside Planner.py).")

        ## MAPF Algorithm Time limit
        self.declare_parameter("time_limit", NO_TIME_LIMIT)
        self.time_limit: float = (
            self.get_parameter("time_limit").get_parameter_value().double_value
        )

        self.get_logger().info(
        f"""
        Planner Parameter values:
        tf_tag_arena: {self.arena_frame}
        arena_height: {self.arena_height}
        arena_width: {self.arena_width}
        agent_diameter: {self.agent_diameter}
        mapf_solver: {self.mapf_solver_class}
        mapf_input: {self.mapf_input_class}
        goal_assigner: {self.goal_assigner_class}
        time_limit: {self.time_limit}
        """
        )

    def plan_callback(self, request, response):

        # Extract message arguments
        assigned_goals: List[AssignedGoal] = request.assigned_goals
        unassigned_goals: Iterable[Position] = request.unassigned_goals
        unassigned_agents: Iterable[str] = request.unassigned_agents

        agent_ids = unassigned_agents + [m.agent_id for m in assigned_goals]


        # Load all object ids on scene 
        obstacle_ids = set(
            yaml.safe_load(
                self.tf_buffer.all_frames_as_yaml()
            ).keys()
        ).difference(agent_ids) # This code is SUS


        # Retrieve agent/obstacle locations in arena
        agent_transformations = []
        obstacle_transformations = []

        try:
            now = rclpy.time.Time()
            for agent_id in agent_ids:
                agent_transformations.append(
                    self.tf_buffer.lookup_transform(
                        self.arena_frame, # Parent frame
                        agent_id,         # Child frame
                        now,
                        timeout=Duration(seconds=0.1)
                    )   
                )
            
            arena_transform = self.tf_buffer.lookup_transform(
                "world",
                self.arena_frame,
                now,
                timeout=Duration(seconds=0.1)
            )

            # The following code should theoretically never cause an exception
            obstacle_ids.discard(self.arena_frame) # Mocap isn't an obstacle in our arena
            for obstacle_id in obstacle_ids:
                obstacle_transformations.append(
                    self.tf_buffer.lookup_transform(
                        self.arena_frame,
                        obstacle_id,
                        now,
                        timeout=Duration(seconds=0.1)
                    )
                )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {agent_id} to {self.arena_frame}: {ex}')
            response.error_msg = "NO_AGENT_TRANSFORM"
            response.args = [agent_id]
            return response


        # Assign goals
        try:
            assigned_goals += self.goal_assigner.assign_goals_to_agents(
                unassigned_goals, 
                list(zip(agent_ids[0:len(unassigned_agents)], agent_transformations[0:len(unassigned_agents)]))
            )
        except AssigningGoalsException as ex:
            response.error_msg = "FAILED_GOAL_ASSIGN"
            response.args = [str(ex)]
            return response


        # Unpack message layers (Overwrite variable type)
        assigned_goals: List[Tuple[str, Position]] = [
            (m.agent_id, m.pos) for m in assigned_goals
        ]


        # Solve MAPF using data
        try:
            mapf_solution = self.generate_and_solve_map(
                agent_transformations, assigned_goals, obstacle_transformations
            )
        except Exception as e:
            # Just pointing this out that the library throws errors directly inherited from Exception (!!!)
            # Someone please fix their inheritance to AssertionError!
            response.error_msg = "FAILED_MAP_SOLVE"
            response.args = [str(e)]
            return response

        agent_paths: List[Path] = mapf_solution.paths
        # The results are ordered by agent order we sent (assigned_goals sets the order)

        # Transform the paths from our Discretized map to scene transforms
        agent_transform_paths = AgentPaths()
        agent_transform_paths.agent_paths = [
            AssignedPath(
                agent_id=a_goal[0],
                path=[
                    self.revert_position_to_transform(waypoint.position, arena_transform)
                    for waypoint in path
                ]
            )
            for a_goal, path in zip(assigned_goals, agent_paths)
        ]

        self.publisher.publish(agent_transform_paths)

        response.error_msg = "SUCCESS"
        response.args = [str(mapf_solution.sum_of_costs), str(mapf_solution.cpu_time)]

        return response

    def generate_and_solve_map(
        self,
        agents: List[TransformStamped],
        goals: List[Tuple[str, Position]],
        obstacles: List[TransformStamped],
    ) -> MAPFOutput:
        """
        In our lab enviornement, our "relative" view point captures & publishes agent transformations
        on the X & Z axis. X, being the same & Z playing the role of Y.
        This is important because this will look a bit weird for someone who's spectating this code on the outside.
        One assumption we make is that given the mocap coordinates (which acts as the start of the axises, (0,0)), all agents
        appear on the first quadrant, relative to the mocap view point.
        Example:
                        ^
                      5 |
                      4 |           * Beta_G
                      3 | * Alpha_A
                      2 |
                      1 |
            -> mocap  0 -------------->
                        0  1  2  3  4
        """
        # Discretize agent, goal & obstacle positions into board
        discrete_agents = [
            WayPoint(
                x=int(agent.transform.translation.x / self.agent_diameter),
                y=int(agent.transform.translation.z / self.agent_diameter),
            )
            for agent in agents
        ]
        # Algo lib uses TimedWayPoint for goals and I don't understand why. (It doesn't supply a t value...)
        discrete_goals = [
            TimedWayPoint(
                x=int(goal[1].x / self.agent_diameter),
                y=int(goal[1].y / self.agent_diameter),
            )
            for goal in goals
        ]
        discrete_obstacles = [
            (
                int(obstacle.transform.translation.x / self.agent_diameter),
                int(obstacle.transform.translation.z / self.agent_diameter),
            )
            for obstacle in obstacles
        ]
        obstacle_map = self.create_empty_map()
        for obstacle in discrete_obstacles:
            # X is col, Y is row (under our representation)
            obstacle_row, obstacle_col = obstacle[1], obstacle[0]
            if obstacle_row >= self.rows or obstacle_col >= self.cols or obstacle_row < 0 or obstacle_col < 0:
                continue # Outside bound obstacles are not an issue :)
            obstacle_map[obstacle_row][obstacle_col] = True

        mapf_instance = MapfInstance()
        mapf_instance.map = obstacle_map

        # Initialize algorithm input
        mapf_input: MAPFInput = SOLVER_DICT[self.mapf_input_class](
            map_instance=mapf_instance,
            starts_list=discrete_agents,
            goals_list=discrete_goals,
        )

        # The following method call can throw (up)
        return self.mapf_solver.solve(mapf_input)

    def create_empty_map(self) -> List[List[bool]]:
        """
        We're using the map input representation required by the algorithm library.
        In the library, a 2D list of bools represent empty & taken positions (False = Empty, True = Taken)
        """
        return [[False] * self.cols] * self.rows
    
    def revert_position_to_transform(self, pos: WayPoint, arena: TransformStamped) -> Transform:
        return Transform(
            translation=Vector3(
                x = float((pos.x * self.agent_diameter + self.agent_diameter/2) + arena.transform.translation.x),
                y = arena.transform.translation.y,
                z = float((pos.y * self.agent_diameter + self.agent_diameter/2) + arena.transform.translation.z)
            )
        )

def main(args=None):
    rclpy.init(args=args)

    planner = Planner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass

    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()