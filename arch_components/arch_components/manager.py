

from typing import List, Union

import rclpy
from rclpy.node import Node, Service
from rclpy.client import Future
from rclpy.action import ActionClient

from arch_interfaces.msg import AssignedGoal, Position, AgentPaths
from arch_interfaces.srv import AgentRequest
from arch_interfaces.action import PlanRequest

from .planner import PlannerResponseTypes

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
    """
    Manager component node implementation.

    WARNING:
    Do not execute this node in a multi-threaded executor!
    Only through rclpy.spin or SingleThreadedExecutor
    """

    def __init__(self):
        super().__init__("manager_component")

        self.assigned_goals: List[AssignedGoal] = []
        self.unassigned_goals: List[Position] = []
        self.unassigned_agents: List[str] = []

        # Variables to keep track of Client/Server responses
        self.future_goal: Future = None
        self.future_response: Future = None
        self.goal_handle = None

        # Client for plan request server
        self.action_cli = ActionClient(self, PlanRequest, 'plan_request')

        self.get_logger().info("Waiting for Planner Action Server...")
        self.action_cli.wait_for_server()
        self.get_logger().info("Planner Server ready!")

        # Service for agent executors
        self.agent_srv: Service = self.create_service(
            AgentRequest, "agent_request", self.agent_callback
        )

        # Subscription for goal updates
        self.goal_subscription = self.create_subscription(
            Position,
            'goals',
            self.goal_callback,
            10
        )

        # Plan publisher for agents
        self.publisher = self.create_publisher(AgentPaths, "agent_paths", 1)

        self.get_logger().info("Finished Initializing manager component, Waiting for agent requests.")

    def agent_callback(self, request, response) -> AgentRequest.Response:

        # Extract request arguments
        agent_msg = request.agent_msg
        agent_id = request.agent_id

        if agent_msg == ManagerRequestTypes.IDLE:
            return self.idle_agent_handler(agent_id, response)
        
        if agent_msg == ManagerRequestTypes.REACHED_GOAL:
            return self.reached_goal_agent_handler(agent_id, response)

        if agent_msg == ManagerRequestTypes.ACTION_FAILED:
            return self.action_failed_agent_handler(agent_id, response)

        if request.agent_msg == ManagerRequestTypes.AGENT_DISCONNECTED:
            return self.disconnected_agent_handler(agent_id, response)
        
        response.error_msg = ManagerResponseTypes.INVALID_MESSAGE
        response.args = [agent_msg]
        return response

    def goal_callback(self, msg: Position) -> None:
        """
        Add published goal to the manager, Trigger plan callback
        """
        # TODO: Check if goal in assigned list (and reject if so)
        if msg in self.unassigned_goals:
            return
        self.unassigned_goals.append(msg)
        self.get_logger().info(f"GOAL LISTED: {msg}")

        # Trigger call if there are agents
        if len(self.unassigned_agents) == 0:
            return

        self.call_planner_async()

    def action_failed_agent_handler(self,
        agent_id: str,
        response: AgentRequest.Response
    ) -> AgentRequest.Response:
        """
        Removes agent from assigned list, requeues goal and calls idle_agent handler
        """
        assigned_goal = self.remove_agent_from_assigned_list(agent_id)
        if assigned_goal != None:
            self.unassigned_goals.append(assigned_goal.pos)

        return self.idle_agent_handler(agent_id, response)

    def reached_goal_agent_handler(self,
        agent_id: str, 
        response: AgentRequest.Response
    ) -> AgentRequest.Response:
        """
        Removes agent from assigned list and calls him on idle_agent handler
        """
        self.remove_agent_from_assigned_list(agent_id)
        
        # Agent Reached goal - now is idle
        return self.idle_agent_handler(agent_id, response)

    def idle_agent_handler(self, 
        agent_id: str, 
        response: AgentRequest.Response
    ) -> AgentRequest.Response:
        """
        Queues agent, sends plan request
        """
        self.unassigned_agents.append(agent_id)
        self.get_logger().info(f"AGENT LISTED: {agent_id}")

        # Trigger call if there are goals
        if len(self.unassigned_goals) != 0:
            self.call_planner_async()

        response.error_msg = ManagerResponseTypes.WAIT_PLAN
        return response

    def disconnected_agent_handler(self,
        agent_id: str, 
        response: AgentRequest.Response
    ) -> AgentRequest.Response:
        """
        Dequeues agent from assigned/unassigned list
        """
        assigned_goal = self.remove_agent_from_assigned_list(agent_id)
        self.remove_agent_from_unassigned_list(agent_id)

        if assigned_goal != None:
            self.unassigned_goals.append(assigned_goal.pos)

        self.call_planner_async()

        response.error_msg = ManagerResponseTypes.AGENT_PLAN_CANCELED
        return response

    def call_planner_async(self) -> None:
        self.get_logger().info("---Calling Plan Request---")
        if self.future_response and not self.future_response.done():
            self.goal_handle.cancel_goal_async()
        
        self.send_plan_request()

    def remove_agent_from_assigned_list(self, agent_id: str) -> Union[AssignedGoal, None]:
        for assigned_goal in self.assigned_goals:
            if assigned_goal.agent_id == agent_id:
                self.assigned_goals.remove(assigned_goal)
                self.get_logger().info(f"AGENT DEQUEUED FROM ASSIGNED LIST: {agent_id}")
                return assigned_goal
    
    def remove_agent_from_unassigned_list(self, agent_id: str) -> None:
        try:
            self.unassigned_agents.remove(agent_id)
            self.get_logger().info(f"AGENT DEQUEUED: {agent_id}")
        except ValueError:
            pass
    
    def send_plan_request(self) -> None:
        pr = PlanRequest.Goal()
        pr.assigned_goals = self.assigned_goals
        pr.unassigned_goals = self.unassigned_goals
        pr.unassigned_agents = self.unassigned_agents
        
        # sync request
        self.future_goal = self.action_cli.send_goal_async(pr)
        self.future_goal.add_done_callback(self.goal_response_callback)
        self.future_response = None
    
    def goal_response_callback(self, future_goal: Future) -> None:
        goal_handle = future_goal.result()
        if self.future_goal != future_goal:
            goal_handle.cancel_goal_async()
            return
        self.get_logger().info("Plan goal accepted, setting up plan-done callback")
        self.goal_handle = goal_handle
        self.future_response = self.goal_handle.get_result_async()
        self.future_response.add_done_callback(self.plan_done_callback)
    
    def plan_done_callback(self, future_response: Future) -> None:
        """
        Updates internal dataset with accordance to the results & publishes them
        (Only if it passes the result validity check)
        """
        if future_response != self.future_response: # Checks if a new request was done & callback still exists
            self.get_logger().info(f"Ignoring Plan-done callback")
            return

        response = self.future_response.result().result
        if not response.error_msg == PlannerResponseTypes.SUCCESS:
            self.get_logger().info(f"Plan Request Failed: {response.error_msg}")
        else:
            self.get_logger().info(f"Plan Request Successful")
            self.assigned_goals = response.assigned_goals
            self.unassigned_goals = response.unassigned_goals
        
        self.publisher.publish(response.plan)
        
        # Reset response and clear buffers / irrelevant data
        # Agents always get cleared - executor needs to manually retry
        self.unassigned_agents.clear()


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