
import math
from typing import Iterable, List, Tuple, Type, Set, Dict
import yaml
from threading import Thread

import rclpy
from rclpy.node import Node, Publisher, Subscription, Service
from rclpy.action import GoalResponse, CancelResponse, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

from arch_interfaces.msg import AssignedGoal, Position, AssignedPath, AgentPaths
from arch_interfaces.action import PlanRequest
from arch_interfaces.action import PlanRequest
from geometry_msgs.msg import Transform, Vector3

from tf2_ros import TransformException, TransformStamped
from tf2_ros.buffer import Buffer, Duration
from tf2_ros.transform_listener import TransformListener

from mapf_solver.Abstract_objects.waypoint import WayPoint
from mapf_solver.Abstract_objects.mapf_solver import MAPFSolver, MAPFInput, MAPFOutput
from mapf_solver.Abstract_objects.path import Path
from mapf_solver.Abstract_objects.map_instance import MapfInstance
from mapf_solver.MAPF_exceptions.exceptions import MapfException, RequestAborted
from mapf_solver.Concrete_objects.concrete_waypoints import TimedWayPoint

from mapf_solver.MAPFSolvers.pbs import CBSInput, CBSSolver, PBSInput, PBSSolver
from mapf_solver.MAPFSolvers.dict_cbs import DictCBSSolver
from mapf_solver.MAPFSolvers.prioritized import PrioritizedPlanningSolver

from .goal_assigner import GoalAssigner, AssigningGoalsException, SimpleGoalAssigner


DEFAULT_ARENA_SIZE = 600
DEFAULT_AGENT_SIZE = 100
DEFAULT_ARENA_NAME = "arena"
DEFAULT_MAPF_ALGORITHM = "CBSSolver"
DEFAULT_MAPF_INPUT = "CBSInput"
DEFAULT_GOAL_ASSIGNER = "SimpleGoalAssigner"
DEFAULT_IGNORED_IDS = "mocap"
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

TRUE, FALSE = 1, 0