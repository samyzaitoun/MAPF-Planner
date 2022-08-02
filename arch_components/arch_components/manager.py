

from typing import Iterable, List, Tuple, Type

import rclpy
from rclpy.node import Node, Client, Service

from arch_interfaces.msg import AssignedGoal, Position
from arch_interfaces.srv import PlanRequest , AgentRequest

DEFAULT_STALL_TIME = "0.1"

class Manager(Node):

    assigned_goals: List[AssignedGoal] = []
    unassigned_goals: List[Position] = []
    unassigned_agents: List[str] = []
    disconnected_agents: List[str] = []
    future_response = None

    def __init__(self):
        super().__init__("manager_component")
        self.get_launch_parameters()
        self.cli: Client = self.create_client(PlanRequest, 'plan_request')
        self.agent_srv: Service = self.create_service(
            AgentRequest, "agent_request", self.agent_callback
        )

        # The timer will be used to group a cluster of agents who want to queue again simulatenously
        self.plan_timer = self.create_timer(self.plan_bus_timer, self.timer_callback)
        self.plan_timer.cancel()

        self.get_logger().info("Finished Initializing manager component, Waiting for agent requests.")

    def get_launch_parameters(self) -> None:

        ## Agent idle-time when recieveing an "IDLE" message
        # NOTE: This is string type instead of float because of service message type
        self.declare_parameter("plan_bus_timer", DEFAULT_STALL_TIME)
        self.plan_bus_timer: float = (
            self.get_parameter("plan_bus_timer").get_parameter_value().double_value
        )

    def agent_callback(self, request, response):

        # Extract request arguments
        agent_msg = request.agent_msg
        agent_id = request.agent_id

        if agent_msg == "IDLE":
            return self.idle_agent_handler(agent_id, response)
        
        if agent_msg == "REACHED_GOAL" or agent_msg == "ACTION_FAILED":
            return self.reached_goal_agent_handler(agent_id, response)
        
        # remove disconnected agent from all lists of goals and agents (if goals ware assigned to him they would also be removed) and try to re-plan
        if request.agent_msg == "AGENT_DISCONNECTED":
            if self.future_response: # A plan request has commenced
                self.disconnected_agents.append(agent_id)
                response.error_msg = "AGENT_PLAN_CANCELED"
                return response
            else:
                return self.disconnected_agent_handler(agent_id, response)
        
        # When all else fails 
        response.error_msg = "INVALID_MESSAGE"
        response.args = [agent_msg]
        return response

    def idle_agent_handler(self, 
        agent_id: str, 
        response: AgentRequest.Response
    ) -> AgentRequest.Response:
        assert agent_id not in self.unassigned_agents
        self.unassigned_agents.append(agent_id)
        self.plan_timer.reset() # Start/Reset the timer

        response.error_msg = "WAIT_PLAN"
        return response

    def reached_goal_agent_handler(self,
        agent_id: str, 
        response: AgentRequest.Response
    ) -> AgentRequest.Response:
        self.remove_agent_from_assigned_list()
        
        # Agent Reached goal - now is idle
        return self.idle_agent_handler(agent_id, response)

    def disconnected_agent_handler(self,
        agent_id: str, 
        response: AgentRequest.Response
    ) -> AgentRequest.Response:
        self.remove_agent_from_assigned_list()
        self.remove_agent_from_unassigned_list()
        response.error_msg = "AGENT_PLAN_CANCELED"
        return response

    def remove_agent_from_assigned_list(self, agent_id: str) -> None:
        for assigned_goal in self.assigned_goals:
            if assigned_goal.agent_id == agent_id:
                self.assigned_goals.remove(assigned_goal)
                break 
    
    def remove_agent_from_unassigned_list(self, agent_id: str) -> None:
        try:
            self.unassigned_agents.remove(agent_id)
        except ValueError:
            pass

    def timer_callback(self) -> None:
        self.get_logger().info("Timer callback triggered")
        if (
            self.unassigned_goals == []
            or self.unassigned_agents == [] # Can happen if an agent disconnects
            or not self.cli.service_is_ready()
        ):
            return
        
        # Check if plan request was already made
        if not self.future_response:
            self.send_plan_request()
            return
        
        if not self.future_response.done():
            return
        response = self.future_response.result()
        if not response.error_msg == "SUCCESS":
            pass # TODO: ADD Handler
        else:
            self.assigned_goals = response.assigned_goals
            self.unassigned_goals = response.unassigned_goals
            self.unassigned_agents = []
            for agent_id in self.disconnected_agents:
                self.remove_agent_from_assigned_list(agent_id)
        
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