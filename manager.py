from utils import *

# BaseManager
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
