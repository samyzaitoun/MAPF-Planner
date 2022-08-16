
from threading import Thread
from time import sleep

from attr import s
import rclpy
from typing import List, Tuple
from rclpy.node import Node
from tf2_ros import TransformStamped, TransformBroadcaster
from geometry_msgs.msg import Vector3
from arch_components.planner import Planner, PlannerResponseTypes
from arch_components.manager import Manager, ManagerRequestTypes, ManagerResponseTypes
from arch_interfaces.msg import Position, AgentPaths, AssignedPath
from arch_interfaces.srv import AgentRequest

class TestFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('test_frame_broadcaster')
        self.br = TransformBroadcaster(self)

    def broadcast_arena(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'arena'
        t.transform.translation.x = 1.0
        t.transform.translation.y = 1.0
        t.transform.translation.z = 1.0
        self.br.sendTransform(t)
    
    def broadcast_agent(self, agent_id: str, loc: Tuple[float, float, float]):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'arena'
        t.child_frame_id = agent_id
        t.transform.translation.x = loc[0]
        t.transform.translation.y = loc[1]
        t.transform.translation.z = loc[2]
        self.br.sendTransform(t)

class FixedFrameBroadcaster(Node):
    def __init__(self, parent_frame_id: str, child_frame_id: str, pos: Vector3, freq: rclpy.time.Time = 0.00833):
        super().__init__(f'FF_{child_frame_id}_broadcaster')
        self.parent_id = parent_frame_id
        self.child_id = child_frame_id
        self.pos = pos
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(freq, self.broadcast_timer_callback)
    
    def broadcast_timer_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_id
        t.child_frame_id = self.child_id
        t.transform.translation.x = self.pos.x
        t.transform.translation.y = self.pos.y
        t.transform.translation.z = self.pos.z

        self.br.sendTransform(t)

class GoalPublisher(Node):
    def __init__(self):
        super().__init__("goal_publisher")
        self.publisher = self.create_publisher(Position, "goals", 10)
    
    def publish_goal(self, goal: Position) -> None:
        self.publisher.publish(goal)

class ManagerTestClient(Node):
    def __init__(self):
        super().__init__('manager_client')
        self.cli = self.create_client(AgentRequest, 'agent_request')
    
    def create_request(self, agent_msg: str, agent_id: str):
        self.response = self.cli.call_async(AgentRequest.Request(agent_msg=agent_msg, agent_id=agent_id))

class AgentTestExecutor(Node):
    def __init__(self, agent_id: str):
        super().__init__(f"{agent_id}_executor")
        self.agent_id = agent_id
        self.cli = self.create_client(AgentRequest, 'agent_request')
        self.subscription = self.create_subscription(AgentPaths, 'agent_paths', self.sol_callback, 10)
    
    def request_and_wait_for_response(self, agent_msg: str = ManagerRequestTypes.IDLE):
        """
        This method is not asynchronous in the sense that if you call this from the main thread,
        it will block the main thread (Unlike sol_callback which is called on the dedicated thread for the node). 
        Please be wary of that.
        """
        while not self.cli.service_is_ready():
            sleep(0.1)
        response = self.cli.call(AgentRequest.Request(agent_msg=agent_msg, agent_id=self.agent_id))
        while response.error_msg == ManagerResponseTypes.RETRY:
            sleep_time = float(response.args[0])
            sleep(sleep_time)
            response = self.cli.call(AgentRequest.Request(agent_msg=agent_msg, agent_id=self.agent_id))
    
    def disconect_and_reconnect(self):
        self.request_and_wait_for_response(ManagerRequestTypes.AGENT_DISCONNECTED)
        self.request_and_wait_for_response()
        
    def sol_callback(self, msg: AgentPaths):
        agent_paths: List[AssignedPath] = msg.agent_paths
        for assigned_path in agent_paths:
            if assigned_path.agent_id == self.agent_id:
                string_solution = '-->'.join([str((point.translation.x, point.translation.z)) for point in assigned_path.path])
                self.get_logger().info(f"PATH PUBLISHED FOR {self.agent_id}: {string_solution}")
                return
        
        self.get_logger().info(f"NO PATH PUBLISHED FOR {self.agent_id}")

class AgentDummyExecutor(Node):
    def __init__(self):
        super().__init__("dummy_executor")
        self.cli = self.create_client(AgentRequest, 'agent_request')
        self.subscription = self.create_subscription(AgentPaths, 'agent_paths', self.sol_callback, 10)
    
    def send_cli_request(self, agent_id: str, request_type: ManagerRequestTypes) -> None:
        while not self.cli.service_is_ready():
            self.get_logger().info("Manager Service not ready.")
            sleep(0.1)
        self.cli.call_async(AgentRequest.Request(agent_msg=request_type, agent_id=agent_id))
    
    def sol_callback(self, msg: AgentPaths):
        agent_paths: List[AssignedPath] = msg.agent_paths
        for assigned_path in agent_paths:
            string_solution = '-->'.join(
                [
                    f"({format(point.translation.x, '.2f')}, {format(point.translation.y, '.2f')})" 
                    for point in assigned_path.path
                ]
            )
            self.get_logger().info(
                f"""
                Agent '{assigned_path.agent_id}' received the path:
                {string_solution}
                """
            )

class SingleThreadNodePool:
    node_list: List[Node]
    executor_list: List[rclpy.executors.SingleThreadedExecutor]
    thread_list: List[Thread]

    def __init__(self):
        self.node_list = []
        self.executor_list = []
        self.thread_list = []
    
    def add_nodes(self, *nodes: Tuple[Node]) -> None:
        for node in nodes:
            self.node_list.append(node)
    
    def start(self) -> None:
        for node in self.node_list:
            personal_exec = rclpy.executors.SingleThreadedExecutor()
            personal_exec.add_node(node)
            personal_thread = Thread(target=personal_exec.spin)
            self.executor_list.append(personal_exec)
            self.thread_list.append(personal_thread)
        
        for thread in self.thread_list:
            thread.start()
    
    def add_nodes_after_start(self, *nodes: Tuple[Node]) -> None:
        local_thread_list = []
        for node in nodes:
            personal_exec = rclpy.executors.SingleThreadedExecutor()
            personal_exec.add_node(node)
            personal_thread = Thread(target=personal_exec.spin)
            self.executor_list.append(personal_exec)
            local_thread_list.append(personal_thread)
        
        for thread in local_thread_list:
            thread.start()
        self.thread_list += local_thread_list

    def stop(self) -> None:
        for exec in self.executor_list:
            exec.shutdown()
        
        for thread in self.thread_list:
            thread.join()


