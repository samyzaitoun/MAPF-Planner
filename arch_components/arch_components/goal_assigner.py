from abc import ABC, abstractmethod
from typing import Iterable, List
from arch_interfaces.msg import Position, AssignedGoal
from tf2_ros import ROSException


class GoalAssigner(ABC):
    @abstractmethod
    def assign_goals_to_agents(
        self, unassigned_goals: Iterable[Position], unassigned_agents: Iterable[str]
    ) -> List[AssignedGoal]:
        pass


class AssigningGoalsException(ROSException):
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
    def assign_goals_to_agents(
        self, unassigned_goals: Iterable[Position], unassigned_agents: Iterable[str]
    ) -> List[AssignedGoal]:
        if len(unassigned_goals) != len(unassigned_agents):
            raise AssigningGoalsException(len(unassigned_agents), len(unassigned_goals))
        # assignes the goals by the order of the lists
        return zip(unassigned_goals, unassigned_agents)
