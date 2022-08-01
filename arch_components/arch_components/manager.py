
import math
from tkinter import N
from typing import Iterable, List, Tuple, Type
import yaml

import rclpy
from rclpy.node import Node, Publisher, Subscription

from arch_interfaces.msg import AssignedGoal, Position, AssignedPath, AgentPaths
from arch_interfaces.srv import PlanRequest , AgentRequest
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

INTERVAL = 5


class Manager(Node):
    def __init__(self, assigned_goals:List[AssignedGoal] = None, unassigned_goals:List[Position] = None, unassigned_agents:List[str] = None):
        super().__init__("manager_component")
        self.assigned_goals = assigned_goals
        self.unassigned_goals = unassigned_goals
        self.unassigned_agents = unassigned_agents
        self.cli = self.create_client(PlanRequest, 'plan_request')
        self.plan_srv = self.create_service(
            AgentRequest, "agent_request", self.plan_callback
        )
        self.req = PlanRequest.Request()
        self.last_time_called = rclpy.time.Time()
        self.minimum_interval = INTERVAL
        self.get_logger().info("Finished Initializing manager component, Waiting for agent requests.")


    def plan_callback(self, request, response):
        response = None
        # there has to be a better way in python to do this but i am too tired to find it
        if request.agent_msg == "IDLE" or request.agent_msg == "REACHED_GOAL":
            for goal in self.assigned_goals:
                if goal.agent_id == request.agent_id:
                    self.assigned_goals.remove(goal)
            if not request.agent_id in self.unassigned_agents:
                self.unassigned_agents.append(request.agent_id)
            # an agent is idle we should probably see if we can assign him a new goal
            if self.unassigned_goals:
                response = self.send_request(PlanRequest(assigned_goals = self.assigned_goals, unassigned_goals = self.unassigned_goals, unassigned_agents = self.unassigned_agents))
        
        if request.agent_msg == "ACTION_FAILED":
            response = self.send_request(PlanRequest(assigned_goals = self.assigned_goals, unassigned_goals = self.unassigned_goals, unassigned_agents = self.unassigned_agents))
        
        # remove disconnected agent from all lists of goals and agents (if goals ware assigned to him they would also be removed) and try to re-plan
        if request.agent_msg == "AGENT_DISCONNECTED":
            for goal in self.assigned_goals:
                if goal.agent_id == request.agent_id:
                    self.assigned_goals.remove(goal)
            if request.agent_id in self.unassigned_agents:
                self.unassigned_agents.remove(request.agent_id)
            response = self.send_request(PlanRequest(assigned_goals = self.assigned_goals, unassigned_goals = self.unassigned_goals, unassigned_agents = self.unassigned_agents))
        return response


    
    def send_request(self, PlanRequest):
        if  rclpy.time.Time() - self.last_time_called < self.minimum_interval:
            return None
        self.last_time_called = rclpy.time.Time()
        self.req.assigned_goals = PlanRequest.assigned_goals
        self.req.unassigned_goals = PlanRequest.unassigned_goals
        self.req.unassigned_agents = PlanRequest.unassigned_agents
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()