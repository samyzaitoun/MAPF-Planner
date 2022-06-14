
from abc import ABC
from typing import Callable

class Topic:
    """
    A facade class which is meant to facilitate the publish/subscribe needs of our architecture.
    """

    def __init__(self, topic) -> None:
        """
        Save the topic instance/interface in the class.
        """
        pass

    def publish(**kwargs) -> None:
        """
        JSON-ify the passed arguments and publish them to the topic.
        """
        pass
    
    # Note to self: Don't forget to wrap func with a decorator which un-JSON-ifies the published data, before calling func. 
    def subscribe(func: Callable) -> None:
        """
        Subscribe the passed function to the topic, and call it with the published kwargs.
        """
        pass 


class BaseAlgorithm(ABC):
    """
    This class is temporary & should be replaced with Gil's abstract base class
    """
    pass


class BaseDestinator(ABC):
    """
    Base class for the Destinator component.
    Destinators simply implement an algorithm for destinating agents to goals, given a set of agent & goal locations.
    """
    
    def destinate(self, agents, goals):
        raise NotImplementedError("Destinator method 'destinate' is not implemented!")


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


class BaseManager(ABC):
    """
    Base class for the Manager component.
    A manager will "manage" the execution of the system. In a sense, the manager is responsible for:
    A. Publishing new plan requests to the Planner
    B. Handling incoming messages from the agents
    (Note that A. is usually triggered by B.)
    Inherently, the manager also manages the goal list (otherwise how would it know when to request new plans).
    """
    
    def __init__(self) -> None:
        raise NotImplementedError("Manager method '__init__' is not implemented!")

    def manage(self, msg, sender) -> None:
        pass
    
