import rospy
from std_msgs.msg import Float64MultiArray, Float64
from enum import Enum

class ROSDtype(Enum):
    FLOAT = Float64
    FLOAT_ARRAY = Float64MultiArray

class RosWrapper:
    def __init__(self, rosnode_name):
        self.publishers = {}
        self.rosnode_name = rosnode_name
        rospy.init_node(rosnode_name)

    def add_publisher(self, topic, dtype, queue=5):
        pub = rospy.Publisher(self.rosnode_name + '/' + topic, dtype.value, queue_size=queue)
        self.publishers[topic] = pub