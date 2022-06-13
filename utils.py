

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
    
    # Don't forget to wrap func with a decorator which un-JSON-ifies the published data, before calling func. 
    def subscribe(func: Callable) -> None:
        """
        Subscribe the passed function to the topic, and call it with the published kwargs.
        """
        pass 


class BasePlanner(ABC):
    """
    Base class for the Planner component.
    A planner calculates new paths when it's method is called, and publishes them to the executors once it's done.
    A configurator will create an instance of Planner and hook it's plan method to be triggered whenever something is published by the Manager component
    If you want to implement a new planner, please inherit from this class (inheritance will be checked).
    """

    def __init__(self, executors_topic, tf_topic ) -> None:
        """
        Init is called once when configurator initiates the system.
        In init the planner should only take arguments that provide a constant interface throughout the entire execution,
        instead of having them passed on everytime plan is called.
        * exeuctors_topic - an instance which provides an interface which the planner could publish through to executors.
        * 
        """
        raise NotImplementedError

    # To be decided - arguments 
    def plan(*args, **kwargs):
        raise NotImplementedError


class BaseDestinator(ABC):
    pass


class BaseManager(ABC):
    pass

