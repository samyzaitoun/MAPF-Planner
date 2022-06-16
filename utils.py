
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


