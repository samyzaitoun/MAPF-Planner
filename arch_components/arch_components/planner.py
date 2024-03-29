
from .planner_config import *

class PlannerResponseTypes:
    SUCCESS = "SUCCESS"
    TRANSFORM_FAILURE = "TRANSFORM_FAILURE"
    FAILED_GOAL_ASSIGN = "FAILED_GOAL_ASSIGN"
    FAILED_MAP_SOLVE = "FAILED_MAP_SOLVE"
    INVALID_INPUT = "INVALID_INPUT"


class Planner(Node):

    plan_subscription: Subscription = None
    tf_subscription: Subscription = None

    def __init__(self):
        super().__init__("planner_component")

        self.load_launch_parameters()

        self.action_server = ActionServer(
            self,
            PlanRequest,
            "plan_request",
            execute_callback=self.plan_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.rows = int(self.arena_height / self.agent_diameter)
        self.cols = int(self.arena_width / self.agent_diameter)

        # Setup buffer for published transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_assigner: GoalAssigner = ASSIGNER_DICT[self.goal_assigner_class]()

        self.get_logger().info("Finished Initializing planner component, Waiting for plan requests.")

    def load_launch_parameters(self) -> None:

        ##  Arena transform frame name
        self.declare_parameter("tf_tag_arena", DEFAULT_ARENA_NAME)
        self.arena_frame: str = (
            self.get_parameter("tf_tag_arena").get_parameter_value().string_value
        )

        ## Arena height
        self.declare_parameter("arena_height", DEFAULT_ARENA_SIZE)
        self.arena_height: int = (
            self.get_parameter("arena_height").get_parameter_value().integer_value
        )

        ## Arena width
        self.declare_parameter("arena_width", DEFAULT_ARENA_SIZE)
        self.arena_width: int = (
            self.get_parameter("arena_width").get_parameter_value().integer_value
        )

        ## Agent diameter
        self.declare_parameter("agent_diameter", DEFAULT_AGENT_SIZE)
        self.agent_diameter: int = (
            self.get_parameter("agent_diameter").get_parameter_value().integer_value
        )

        ## Ignored TF IDs
        self.declare_parameter("ignored_tf_ids", DEFAULT_IGNORED_IDS)
        self.ignored_tf_ids: str = (
            self.get_parameter("ignored_tf_ids").get_parameter_value().string_value
        )

        ## MAPF Algorithm Class
        self.declare_parameter("mapf_solver", DEFAULT_MAPF_ALGORITHM)
        self.mapf_solver_class: str = (
            self.get_parameter("mapf_solver").get_parameter_value().string_value
        )
        if self.mapf_solver_class not in SOLVER_DICT:
            raise AssertionError("Solver class not in Planner dictionary. Please update it (inside Planner.py).")

        ## MAPF Algorithm Input Class
        self.declare_parameter("mapf_input", DEFAULT_MAPF_INPUT)
        self.mapf_input_class: str = (
            self.get_parameter("mapf_input").get_parameter_value().string_value
        )
        if self.mapf_input_class not in SOLVER_DICT:
            raise AssertionError("Solver Input class not in Planner dictionary. Please update it (inside Planner.py).")

        ## goal assign Algorithm Class
        self.declare_parameter("goal_assigner", DEFAULT_GOAL_ASSIGNER)
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

    # These are related to the action server callbacks
    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().debug('Received plan request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().debug('Received cancel request')
        return CancelResponse.ACCEPT

    def plan_callback(self, goal_handle):

        self.get_logger().debug("Plan callback triggered")

        # Extract message arguments
        request = goal_handle.request
        assigned_goals: List[AssignedGoal] = request.assigned_goals
        unassigned_goals: Iterable[Position] = request.unassigned_goals
        unassigned_agents: Iterable[str] = request.unassigned_agents

        response = PlanRequest.Result()

        if unassigned_agents == [] or unassigned_goals == []:
            return self.failed_plan_handler(
                response,
                error_msg=PlannerResponseTypes.INVALID_INPUT,
                args=[],
                goal_handle=goal_handle
            )
        
        agent_ids = unassigned_agents + [m.agent_id for m in assigned_goals]
        try:
            obstacle_ids = self.get_all_frame_ids().difference(agent_ids + [self.arena_frame])
        except AttributeError as ex:
            return self.failed_plan_handler(
                response, 
                error_msg=PlannerResponseTypes.TRANSFORM_FAILURE, 
                args=[str(ex)], 
                goal_handle=goal_handle
            )

        # Retrieve agent/obstacle locations in arena
        try:
            now = rclpy.time.Time()
            agent_transformations = self.get_frame_transformations(self.arena_frame, agent_ids, now)
            obstacle_transformations = self.get_frame_transformations(self.arena_frame, obstacle_ids, now)

        except TransformException as ex:
            return self.failed_plan_handler(
                response, 
                error_msg=PlannerResponseTypes.TRANSFORM_FAILURE, 
                args=[str(ex)] + agent_ids + list(obstacle_ids),
                goal_handle=goal_handle
            )

        # Assign goals
        try:
            newly_assigned_goals, unassigned_goals = self.goal_assigner.assign_goals_to_agents(
                unassigned_goals,
                # We only want the unassigned agent transformations
                [(agent_id, agent_transformations[agent_id]) for agent_id in unassigned_agents]
            )
            assigned_goals += newly_assigned_goals
        except AssigningGoalsException as ex:
            return self.failed_plan_handler(
                response, 
                error_msg=PlannerResponseTypes.FAILED_GOAL_ASSIGN, 
                args=[str(ex)], 
                goal_handle=goal_handle
            )

        # In case of successful planning, manager needs to know who was assigned to what
        response.assigned_goals = assigned_goals
        response.unassigned_goals = unassigned_goals

        # Unpack message layers (Overwrite variable type)
        assigned_goals: List[Tuple[str, Position]] = [
            (m.agent_id, m.pos) for m in assigned_goals
        ]

        # Solve MAPF using data
        try:
            mapf_solution = self.generate_and_solve_map(
                agent_transformations, assigned_goals, obstacle_transformations, goal_handle
            )
        except RequestAborted as ex:
            goal_handle.canceled()
            self.get_logger().debug("Plan request aborted!")
            return response
        except MapfException as ex:
            return self.failed_plan_handler(
                response, 
                error_msg=PlannerResponseTypes.FAILED_MAP_SOLVE, 
                args=[str(ex)], 
                goal_handle=goal_handle
            )

        agent_paths: List[Path] = mapf_solution.paths

        response.error_msg = PlannerResponseTypes.SUCCESS
        response.args = [str(mapf_solution.sum_of_costs), str(mapf_solution.cpu_time)]
        response.plan = self.get_plan_from_solution(agent_paths, assigned_goals)
        self.get_logger().debug("Plan request successful!")
        goal_handle.succeed()

        return response
    
    def failed_plan_handler(
        self,
        response: PlanRequest.Result, 
        error_msg: str, 
        args: List[str],
        goal_handle
    ) -> PlanRequest.Result:
        response.error_msg = error_msg
        response.args = args
        # Publish no plan
        response.plan = AgentPaths()
        self.get_logger().debug("Plan request failed!")
        goal_handle.succeed()
        return response
    
    def get_all_frame_ids(self) -> Set[str]:
        return set(
            [
                frame_id for frame_id in (yaml.safe_load(
                    self.tf_buffer.all_frames_as_yaml()
                ).keys()) if self.ignored_tf_ids not in frame_id
            ]
        )
    
    def get_frame_transformations(
        self,
        relative_frame_id: str,
        frame_ids: Iterable[str],
        time_point: rclpy.time.Time
    ) -> Dict[str, TransformStamped]:
        transformations = dict()
        for id in frame_ids:
            transformations[id] = (
                self.tf_buffer.lookup_transform(
                    relative_frame_id, # Parent frame
                    id,               # Child frame
                    time_point,
                    timeout=Duration(seconds=0.1)
                )   
            )

        return transformations
    
    def generate_and_solve_map(
        self,
        agent_transforms: Dict[str, TransformStamped],
        assigned_goals: List[Tuple[str, Position]],
        obstacle_transforms: Dict[str, TransformStamped],
        goal_handle
    ) -> MAPFOutput:
        """
        An assumption we make is that given the mocap coordinates (which acts as the start of the axises, (0,0)), all agents
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
        # Order of elements matters here (must be identical to the goals in assigned_goals)!
        assigned_agents = [a_goal[0] for a_goal in assigned_goals]
        assigned_agent_transforms = [agent_transforms[agent] for agent in assigned_agents]

        # Discretize agent, goal & obstacle positions into board
        discrete_agents = [
            WayPoint(
                *self.extract_x_and_y_dims(agent)
            )
            for agent in assigned_agent_transforms
        ]
        # Algo lib uses TimedWayPoint for goals and I don't understand why. (It doesn't supply a t value...)
        discrete_goals = [
            TimedWayPoint(
                x=int(goal[1].x / self.agent_diameter),
                y=int(goal[1].y / self.agent_diameter),
            )
            for goal in assigned_goals
        ]
        discrete_obstacles = [
            self.extract_x_and_y_dims(obstacle)
            for obstacle in obstacle_transforms.values()
        ]

        self.get_logger().debug(
            f"""
            DISCRETE_AGENTS: {discrete_agents}
            DISCRETE_GOALS: {discrete_goals}
            DISCRETE_OBSTACLES: {discrete_obstacles}
            """
        )

        obstacle_map = self.create_empty_map()
        for obstacle in discrete_obstacles:
            # X is col, Y is row (under our representation)
            obstacle_row, obstacle_col = obstacle[0], obstacle[1]
            if obstacle_row >= self.rows or obstacle_col >= self.cols or obstacle_row < 0 or obstacle_col < 0:
                continue # Outside bound obstacles are not an issue :)
            obstacle_map[obstacle_row][obstacle_col] = TRUE

        map_str = "\n"
        for row in obstacle_map:
            map_str += str(row) + '\n'
        self.get_logger().debug(map_str)

        mapf_instance = MapfInstance()
        mapf_instance.map = obstacle_map

        # Initialize algorithm class & input
        mapf_solver: MAPFSolver = SOLVER_DICT[self.mapf_solver_class](self.time_limit)
        mapf_input: MAPFInput = SOLVER_DICT[self.mapf_input_class](
            map_instance=mapf_instance,
            starts_list=discrete_agents,
            goals_list=discrete_goals,
        )

        # The following method call can throw (up)
        return mapf_solver.solve(mapf_input, cancel_indicator=goal_handle)

    def extract_x_and_y_dims(self, frame: TransformStamped) -> Tuple[int, int]:
        return (
            int(frame.transform.translation.x / self.agent_diameter),
            int(frame.transform.translation.y / self.agent_diameter)
        )

    def create_empty_map(self) -> List[List[bool]]:
        """
        We're using the map input representation required by the algorithm library.
        In the library, a 2D list of bools represent empty & taken positions (False = Empty, True = Taken)
        """
        return [[FALSE]*self.cols for i in range(self.rows)]
    
    def get_plan_from_solution(
        self, 
        agent_paths: List[Path],
        assigned_goals: List[Tuple[str, Position]]
    ) -> AgentPaths:
        # The results are ordered by agent order we sent (assigned_goals sets the order)
        # This is relevant because agent_paths doesn't mention which path belongs to which agent id
        agent_transform_paths = AgentPaths()
        agent_transform_paths.agent_paths = [
            AssignedPath(
                agent_id=a_goal[0],
                path=[
                    self.revert_position_to_transform(waypoint.position)
                    for waypoint in path
                ]
            )
            for a_goal, path in zip(assigned_goals, agent_paths)
        ]

        return agent_transform_paths

    def revert_position_to_transform(self, pos: WayPoint) -> Transform:
        return Transform(
            translation=Vector3(
                x = float(pos.x * self.agent_diameter + self.agent_diameter/2),
                y = float(pos.y * self.agent_diameter + self.agent_diameter/2),
                z = 0.0,
            )
        )

def main(args=None):
    rclpy.init(args=args)

    planner = Planner()

    executor = rclpy.executors.MultiThreadedExecutor()
    try:
        rclpy.spin(planner, executor=executor)
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()