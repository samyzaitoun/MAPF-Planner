from utils import *


class BasePlanner(ABC):
    """
    Base class for the Planner component.
    A planner calculates new paths when it's method is called, and publishes them to the executors once it's done.
    A configurator will create an instance of Planner and hook it's plan method to be triggered whenever something is published by the Manager component
    If you want to implement a new planner, please inherit from this class (inheritance will be checked).
    """

    def __init__(
            self,
            executors_topic: Topic,
            tf_topic: Topic,
            destinator: BaseDestinator,
            mapf_solver: BaseAlgorithm
    ) -> None:
        raise NotImplementedError("Planner method '__init__' is not implemented!")

    def plan(self, **kwargs) -> None:
        raise NotImplementedError("Planner method 'plan' is not implemented!")
