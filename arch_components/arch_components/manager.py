

from time import sleep
import time
from typing import Iterable, List, Tuple, Type, Union

import rclpy
from rclpy.node import Node, Client, Service
from rclpy.client import Future

from arch_interfaces.msg import AssignedGoal, Position, AgentPaths
from arch_interfaces.srv import PlanRequest , AgentRequest

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

# TODO: Edge case - goal is in assigned_goals and in unassigned_goals
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
        self.future_response: Future = None

        # Create client for plan request srv
        self.cli: Client = self.create_client(PlanRequest, 'plan_request')

        # No reason to continue when we can't access the planner
        self.get_logger().info("Waiting for Planner service...")
        while not self.cli.service_is_ready():
            pass
        self.get_logger().info("Planner service ready!")
        self.agent_srv: Service = self.create_service(
            AgentRequest, "agent_request", self.agent_callback
        )

        # Create subscription for goal updates
        self.goal_subscription = self.create_subscription(
            Position,
            'goals',
            self.goal_callback,
            10
        )

        # Create plan publisher for agents
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
        
        # When all else fails 
        response.error_msg = ManagerResponseTypes.INVALID_MESSAGE
        response.args = [agent_msg]
        return response

    def goal_callback(self, msg: Position) -> None:
        """
        Add published goal to the manager.
        While this method is quite simple, there's some thinking behind this.
        For example, how can we assure that during planner response (which btw, 
        overrides the values of the unassigned goals list), any goals listed here
        are not lost because of scheduling order? Well this is quite simple because
        there are 2 possible scenarios:
        A. The planner response is not ready - and we simply cancel it and reorder
        it with the new goal list.
        B. The planner response is ready - but we queue another planner response.
        If the scheduler has already handled the response callback - then we are working
        with the updated dataset, and adding the goal is fine. If the scheduler hasn't already
        handled the response - then it will get caught in the callback (as we checked for this
        possible race-condition).
        Coincedentally, this is also correct for all the other methods which augment 
        the underlying data structures in manager, so I won't detail this again in other methods.
        """
        if msg in self.unassigned_goals:
            return
        self.unassigned_goals.append(msg)
        self.get_logger().info(f"GOAL LISTED: {msg}")

        self.call_planner_async()

    def idle_agent_handler(self, 
        agent_id: str, 
        response: AgentRequest.Response
    ) -> AgentRequest.Response:
        """
        Queues agent, sends plan request
        """
        assert agent_id not in self.unassigned_agents
        self.unassigned_agents.append(agent_id)
        self.get_logger().info(f"AGENT LISTED: {agent_id}")

        # Trigger a plan request only when there are unassigned goals
        if len(self.unassigned_goals) != 0:
            self.call_planner_async()

        response.error_msg = ManagerResponseTypes.WAIT_PLAN
        return response

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
    
    def call_planner_async(self) -> None:
        self.get_logger().info("---Calling Plan Request---")
        if self.future_response and not self.future_response.done():
            self.get_logger().info("Canceling previous plan future callback!")
            self.future_response.cancel()
        
        self.send_plan_request()
    
    def send_plan_request(self) -> None:
        pr = PlanRequest.Request()
        pr.assigned_goals = self.assigned_goals
        pr.unassigned_goals = self.unassigned_goals
        pr.unassigned_agents = self.unassigned_agents
        pr.time = time.time()
        
        # sync request
        self.future_response = self.cli.call_async(pr)
        self.future_response.add_done_callback(self.plan_done_callback)
        self.get_logger().info("New plan request sent!")
    
    def plan_done_callback(self, future_response) -> None:
        """
        Please note that this callback is also triggered when we cancel the request.
        Moreover, future_response.done is evaluted on the manager thread.
        Also, the callback passes us the future it was called with.
        But since we locally adjust the future response when new requests are done,
        then the previous future response may no longer be relevant, hence why it is
        not needed.
        """
        # A possible race-cond scenario, where a new plan triggered but the callback
        # was already queued to the executor.
        self.get_logger().info("Plan-done callback Triggered")
        if (
            future_response != self.future_response # Checks if a new request was done & callback still exists
            or not future_response.done()           # Checks if request was canceled
        ):
            self.get_logger().info(f"Future response is not ready.")
            return

        response = self.future_response.result()
        if not response.error_msg == PlannerResponseTypes.SUCCESS:
            self.get_logger().info(f"Plan Request Failed: {response.error_msg}")
        else:
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