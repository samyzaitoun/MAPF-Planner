# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Callable, Dict, Iterable, Tuple
import rclpy
from rclpy.node import Node

from arch_interfaces.msg import (
    Position, 
    Goal, 
    PlanRequest, 
)


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
        print(f"Testing {name_type} Msg Publishing/Receiving")
        self.test_publish(**kwargs)
        while rclpy.ok() and self.received_msg is None:
            rclpy.spin_once(self)
        if self.published_msg == self.received_msg:
            print(f"Message Type {name_type} transferred successfully: Yes")
        else:
            print(f"Message Type {name_type} transferred successfully: No")
        

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
            'z': 3.0,
            'w': 0.0
        }),
        (Goal, "Goal",
        {
            'pos': Position(x=1.0, y=2.0, z=3.0, w=0.0),
            'agent_id': 5
        }),
        (PlanRequest, "PlanRequest",
        {
            'goals': [
                Goal(pos=Position(x=1.0, y=2.0, z=3.0, w=0.0), agent_id=1),
                Goal(pos=Position(x=-1.0, y=-1.0, z=-1.0, w=90.0))
            ]
        }
        )
    ]
    test_interface_list(type_list)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
