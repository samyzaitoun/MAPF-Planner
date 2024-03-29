from abc import ABC, abstractmethod
from typing import List, Tuple
from arch_interfaces.msg import Position, AssignedGoal
from tf2_ros import TransformStamped

from arch_interfaces.msg import AssignedGoal, Position


class GoalAssigner(ABC):
    @abstractmethod
    def assign_goals_to_agents(
        self, 
        unassigned_goals: List[Position], 
        unassigned_agents: List[Tuple[str, TransformStamped]]
    ) -> Tuple[List[AssignedGoal], List[Position]]:
        pass


class AssigningGoalsException(AssertionError):
    def __init__(
        self,
        agents: int,
        goals: int,
        message="amount of agents is not equal to the amount of goals",
    ):
        self.agents = agents
        self.goals = goals
        self.message = message
        super().__init__(
            self.message + f". #goals: {self.goals}, #agents: {self.agents}"
        )


class SimpleGoalAssigner(GoalAssigner):
    # TODO: Handle cases where more goals than agents
    def assign_goals_to_agents(
        self, 
        unassigned_goals: List[Position], 
        unassigned_agents: List[Tuple[str, TransformStamped]]
    ) -> Tuple[List[AssignedGoal], List[Position]]:
        # assigns the goals by the order of the lists
        still_unassigned_agents = []
        still_unassigned_goals = []
        assigned_goal_agent = [
            AssignedGoal(pos=goal, agent_id=agent[0]) 
            for goal, agent in zip(unassigned_goals, unassigned_agents)
        ]
        if len(unassigned_goals) > len(unassigned_agents):
            still_unassigned_goals = [goal for goal in unassigned_goals[len(unassigned_agents):]]
        return assigned_goal_agent, still_unassigned_goals
