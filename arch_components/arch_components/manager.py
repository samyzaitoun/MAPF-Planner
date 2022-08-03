

from typing import Iterable, List, Tuple, Type

import rclpy
from rclpy.node import Node, Client, Service

from arch_interfaces.msg import AssignedGoal, Position
from arch_interfaces.srv import PlanRequest , AgentRequest

from .planner import PlannerResponseTypes

DEFAULT_STALL_TIME = "0.1"
DEFAULT_IDLE_TIME = "0.05"

# MSG Request strings
class ManagerRequestTypes:
    IDLE = "IDLE"
    REACHED_GOAL = "REACHED_GOAL"
    ACTION_FAILED = "ACTION_FAILED"
    AGENT_DISCONNECTED = "AGENT_DISCONNECTED"

# MSG Response strings
class ManagerResponseTypes:
    RETRY = "RETRY"
    WAIT_PLAN = "WAIT_PLAN"
    AGENT_PLAN_CANCELED = "AGENT_PLAN_CANCELED"
    INVALID_MESSAGE = "INVALID_MESSAGE"

class Manager(Node):

    assigned_goals: List[AssignedGoal] = []
    unassigned_goals: List[Position] = []
    unassigned_agents: List[str] = []
    future_response = None

    def __init__(self):
        super().__init__("manager_component")
        self.get_launch_parameters()

        # Create client for plan request srv
        self.cli: Client = self.create_client(PlanRequest, 'plan_request')
        self.agent_srv: Service = self.create_service(
            AgentRequest, "agent_request", self.agent_callback
        )

        # Create subscription for goal updates
        self.goal_subscription = self.create_subscription(
            Position,
            'goals',
            self.goal_callback,
        )

        # The timer will be used to group a cluster of agents who want to queue again simulatenously
        self.plan_timer = self.create_timer(self.plan_bus_timer, self.timer_callback)
        self.plan_timer.cancel()

        self.get_logger().info("Finished Initializing manager component, Waiting for agent requests.")

    def get_launch_parameters(self) -> None:

        self.declare_parameter("plan_bus_timer", DEFAULT_STALL_TIME)
        self.plan_bus_timer: float = (
            self.get_parameter("plan_bus_timer").get_parameter_value().double_value
        )

        ## Agent forced idle time period
        # NOTE: This is string type instead of float because of service message type
        self.declare_parameter("force_idle_time_period", DEFAULT_IDLE_TIME)
        self.force_idle_time_period: str = (
            self.get_parameter("force_idle_time_period").get_parameter_value().string_value
        )

    def agent_callback(self, request, response) -> AgentRequest.Response:

        # Extract request arguments
        agent_msg = request.agent_msg
        agent_id = request.agent_id

        if self.future_response:
            response.error_msg = ManagerResponseTypes.RETRY
            response.args = [self.force_idle_time_period]
            return response

        if agent_msg == ManagerRequestTypes.IDLE:
            return self.idle_agent_handler(agent_id, response)
        
        if agent_msg == ManagerRequestTypes.REACHED_GOAL:
            return self.reached_goal_agent_handler(agent_id, response)

        if agent_msg == ManagerRequestTypes.ACTION_FAILED:
            return self.action_failed_agent_handler(agent_id, response)

        if request.agent_msg == ManagerRequestTypes.AGENT_DISCONNECTED:
            return self.disconnected_agent_handler(agent_id, response)
        
        # When all else fails 
        response.error_msg = ManagerResponseTypes.INVALID_MESSAGE
        response.args = [agent_msg]
        return response

    def goal_callback(self, msg: Position) -> None:
        if msg in self.unassigned_goals:
            return
        self.unassigned_goals.append(msg)
        self.get_logger().info(f"GOAL ADDED: {msg}")

    def idle_agent_handler(self, 
        agent_id: str, 
        response: AgentRequest.Response
    ) -> AgentRequest.Response:
        """
        Queues agent, starts/restarts bus timer
        """
        assert agent_id not in self.unassigned_agents
        self.unassigned_agents.append(agent_id)
        self.plan_timer.reset() # Start/Reset the timer

        response.error_msg = ManagerResponseTypes.WAIT_PLAN
        return response

    def reached_goal_agent_handler(self,
        agent_id: str, 
        response: AgentRequest.Response
    ) -> AgentRequest.Response:
        """
        Removes agent from assigned list & requeues agent
        """
        self.remove_agent_from_assigned_list()
        
        # Agent Reached goal - now is idle
        return self.idle_agent_handler(agent_id, response)

    def action_failed_agent_handler(self,
        agent_id: str,
        response: AgentRequest.Response
    ) -> AgentRequest.Response:
        """
        Removes agent from assigned list, requeues goal & requeues agent
        """
        goal = self.remove_agent_from_assigned_list(agent_id).pos
        self.unassigned_goals.append(goal)

        return self.idle_agent_handler(agent_id, response)

    def disconnected_agent_handler(self,
        agent_id: str, 
        response: AgentRequest.Response
    ) -> AgentRequest.Response:
        """
        Dequeues agent from assigned/unassigned list
        """
        self.remove_agent_from_assigned_list()
        self.remove_agent_from_unassigned_list()
        response.error_msg = ManagerResponseTypes.AGENT_PLAN_CANCELED
        return response

    def remove_agent_from_assigned_list(self, agent_id: str) -> AssignedGoal:
        for assigned_goal in self.assigned_goals:
            if assigned_goal.agent_id == agent_id:
                self.assigned_goals.remove(assigned_goal)
                return assigned_goal
    
    def remove_agent_from_unassigned_list(self, agent_id: str) -> None:
        try:
            self.unassigned_agents.remove(agent_id)
        except ValueError:
            pass

    def timer_callback(self) -> None:
        self.get_logger().info("Timer callback triggered")
        if (
            self.unassigned_goals == []
            or not self.cli.service_is_ready()
        ):
            return
        
        if self.unassigned_agents == []: # Can happen if an agent disconnects
            self.plan_timer.cancel()
        
        # Check if plan request was already made
        if not self.future_response:
            self.send_plan_request()
            return
        
        if not self.future_response.done():
            return
        response = self.future_response.result()
        if not response.error_msg == PlannerResponseTypes.SUCCESS:
            self.get_logger().info(f"Plan Request Failed: {response.error_msg}")
        else:
            self.assigned_goals = response.assigned_goals
            self.unassigned_goals = response.unassigned_goals
            self.unassigned_agents.clear()
        
        # Reset response
        self.future_response = None
        
        # Cancel timer
        self.plan_timer.cancel()
    
    def send_plan_request(self) -> None:

        pr = PlanRequest.Request()
        pr.assigned_goals = self.assigned_goals
        pr.unassigned_goals = self.unassigned_goals
        pr.unassigned_agents = self.unassigned_agents
        
        # sync request
        self.future_response = self.cli.call_async(pr)


def main(args=None):
    rclpy.init(args=args)

    manager = Manager()

    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass

    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()