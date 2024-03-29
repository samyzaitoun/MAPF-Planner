from typing import Callable, Dict, Iterable, Tuple
import rclpy
from rclpy.node import Node

from arch_interfaces.msg import Position, AssignedGoal


class IntegrationNode(Node):
    published_msg = None
    received_msg = None

    def __init__(self, msg_type: Callable):
        super().__init__('integration_node')
        self.msg_type = msg_type
        self.publisher = self.create_publisher(msg_type, 'topic', 10)

    def test_publish(self, **kwargs):
        self.published_msg = self.msg_type()
        for attr, val in kwargs.items():
            setattr(self.published_msg, attr, val)
        self.publisher.publish(self.published_msg)
    
    def test_publish_callback(self, msg):
        self.received_msg = msg
    
    def test(self, name_type: str, **kwargs):
        self.subscription = self.create_subscription(self.msg_type, 'topic', self.test_publish_callback, 10)
        print(f"Testing Msg Publishing/Receiving: '{name_type}' ")
        self.test_publish(**kwargs)
        while rclpy.ok() and self.received_msg is None:
            rclpy.spin_once(self)
        if self.published_msg == self.received_msg:
            print(f"Success!")
        else:
            print(f"Failure :(")
        

def test_interface_list(type_list: Iterable[Tuple[Callable, str, Dict]]):
    """
    Format of type_list: 
    (Type, "TypeName", {Parameters : Values})
    """
    for t in type_list:
        test_type = IntegrationNode(t[0])
        test_type.test(
            name_type=t[1],
            **t[2]
        )
        test_type.destroy_node()
    


def main(args=None):
    rclpy.init(args=args)
    type_list = [
        (Position, "Position", 
        {
            'x': 1.0,
            'y': 2.0,
            'w': 0.0
        }),
        (AssignedGoal, "AssignedGoal",
        {
            'pos': Position(x=1.0, y=2.0, w=0.0),
            'agent_id': "Alpha_5"
        }),
    ]
    test_interface_list(type_list)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
