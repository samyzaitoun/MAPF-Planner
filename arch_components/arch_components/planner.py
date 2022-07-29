from typing import Iterable, List, Tuple, Type
import yaml

import rclpy
from rclpy.node import Node, Publisher, Subscription

from arch_interfaces.msg import AssignedGoal, Position
from arch_interfaces.srv import PlanRequest

from tf2_ros import TransformException, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from mapf_solver.Abstract_objects.waypoint import WayPoint
from mapf_solver.Abstract_objects.mapf_solver import MAPFSolver, MAPFInput, MAPFOutput
from mapf_solver.Abstract_objects.path import Path
from mapf_solver.Concrete_objects.concrete_waypoints import TimedWayPoint

from goal_assigner import GoalAssigner, AssigningGoalsException

NO_TIME_LIMIT = -1


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

        # Create Service PlanRequest
        self.plan_srv = self.create_service(
            PlanRequest, "plan_request", self.plan_callback
        )

        # TODO: Create Publisher to Plan topic
        # self.publisher = self.create_publisher(Paths, path_topic, 1)

        # Calculate constant values
        self.rows = int(self.arena_height / self.agent_diameter)
        self.cols = int(self.arena_width / self.agent_diameter)

        # Setup buffer for published transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize algorithm solver class
        self.mapf_solver = self.mapf_solver_class(self.time_limit)

        # set goal assigning algorithem
        self.assign_goals_to_agents = self.goal_assigner_class.assign_goals_to_agents

    def get_launch_parameters(self) -> None:

        ##  Arena transform tf name (str)
        self.declare_parameter("tf_tag_arena", "arena")
        self.arena_frame: str = (
            self.get_parameter("tf_tag_arena").get_parameter_value().string_value
        )

        ## Arena height (int)
        self.declare_parameter("arena_height")
        self.arena_height: int = (
            self.get_parameter("arena_height").get_parameter_value().integer_value
        )

        ## Arena width (int)
        self.declare_parameter("arena_width")
        self.arena_width: int = (
            self.get_parameter("arena_width").get_parameter_value().integer_value
        )

        ## Agent diameter (int)
        self.declare_parameter("agent_diameter")
        self.agent_diameter: int = (
            self.get_parameter("agent_diameter").get_parameter_value().integer_value
        )

        ## MAPF Algorithm Class (MAPFSolver)
        self.declare_parameter("mapf_solver")
        self.mapf_solver_class: Type[MAPFSolver] = self.get_parameter(
            "mapf_solver"
        ).get_parameter_value()  # Not Defined, should work..?

        # copied from  MAPF Algorithm Class, TODO: check it works and delete this comment
        ## goal assign Algorithm Class (goal_assigner)
        self.declare_parameter("goal_assigner")
        self.goal_assigner_class: Type[GoalAssigner] = self.get_parameter(
            "goal_assigner"
        ).get_parameter_value()  # Not Defined, should work..?

        ## MAPF Algorithm Input Class (MAPFInput)
        self.declare_parameter("mapf_input")
        self.mapf_input_class: Type[MAPFInput] = self.get_parameter(
            "mapf_input"
        ).get_parameter_value()  # Not Defined, should work..?

        ## MAPF Algorithm Time limit (float)
        self.declare_parameter("time_limit", NO_TIME_LIMIT)
        self.time_limit: float = (
            self.get_parameter("time_limit").get_parameter_value().double_value
        )

    def plan_callback(self, request, response):

        # Extract message arguments
        assigned_goals: List[AssignedGoal] = request.assigned_goals
        unassigned_goals: Iterable[Position] = request.unassigned_goals
        unassigned_agents: Iterable[str] = request.unassigned_agents

        # Assign goals
        # TODO: Define & Implement assignment class

        try:
            assigned_goals += self.assign_goals_to_agents(
                unassigned_goals, unassigned_agents
            )
        except AssigningGoalsException as ex:
            response.error_msg = str(ex)
            return response

        # Unpack message layers (not fun to work with 2 layers of indirection)
        assigned_goals: List[Tuple[str, Position]] = [
            (m.agent_id, m.pos) for m in assigned_goals
        ]

        # Load all object frames on scene (Agents/Arena will be removed)
        obstacle_ids = set(yaml.safe_load(self.tf_buffer.all_frames_as_yaml()).keys()) # This code is SUS

        # Retrieve agent/obstacle locations in arena
        agent_transformations = []
        obstacle_transformations = []

        try:
            obstacle_ids.discard(self.arena_frame)
            now = rclpy.time.Time()
            for agent_id, _ in assigned_goals:
                agent_transformations += self.tf_buffer.lookup_transform(
                    agent_id,
                    self.arena_frame,
                    now   
                )
                # Anything that's NOT an agent, is an obstacle
                obstacle_ids.discard(agent_id)
            
            for obstacle_id in obstacle_ids:
                obstacle_transformations += self.tf_buffer.lookup_transform(
                    obstacle_id,
                    self.arena_frame,
                    now
                )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {agent_id} to {self.arena_frame}: {ex}')
            response.error_msg = "NO_AGENT_TRANSFORM"
            response.args = [agent_id]
            return response

        # Solve MAPF using data
        try:
            mapf_solution = self.generate_and_solve_map(
                agent_transformations, assigned_goals, obstacle_transformations
            )
        except Exception as e:
            # Just pointing this out that the library throws errors directly inherited from Exception (!!!)
            # Someone please fix their inheritance to AssertionError!
            response.error_msg = str(e)
            return response

        agent_paths: List[Path] = mapf_solution.paths

        # TODO:
        # A. Figure out what message type we agree on
        # B. Transform the paths from our Discretized map to scene transforms
        # (Remember to add them relative to mocap's transform)
        # C. Publish them, and return a 'success' response to whoever initiated the plan request.

        # msg_publish = Paths()
        # msg_publish.paths = paths
        # self.publisher.publish(msg_publish)

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
                x=int(agent.x / self.agent_diameter),
                y=int(agent.z / self.agent_diameter),
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
                int(obstacle.x / self.agent_diameter),
                int(obstacle.z / self.agent_diameter),
            )
            for obstacle in obstacles
        ]
        obstacle_map = self.create_empty_map()
        for obstacle in discrete_obstacles:
            # X is col, Y is row (under our representation)
            obstacle_row, obstacle_col = obstacle[1], obstacle[0]
            obstacle_map[obstacle_row][obstacle_col] = True

        # Initialize algorithm input
        mapf_input: MAPFInput = self.mapf_input_class(
            map_instance=obstacle_map,
            starts_list=discrete_agents,
            goals_list=discrete_goals,
        )

        # The following method call can throw (up)
        return self.mapf_solver(mapf_input)

    def create_empty_map(self) -> List[List[bool]]:
        """
        We're using the map input representation required by the algorithm library.
        In the library, a 2D list of bools represent empty & taken positions (False = Empty, True = Taken)
        """
        return [[False] * self.cols] * self.rows
