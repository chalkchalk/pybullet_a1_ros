import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import WrenchStamped
from enum import Enum
import time

class RobotJointState(JointState):
    def __init__(self, name, pos, vel, tor):
        super().__init__()
        self.name = name
        self.position = pos
        self.velocity = vel
        self.effort = tor

class ImuData(Imu):
    def __init__(self, quat, ang_vel, lin_acc):
        super().__init__()
        self.orientation.x = quat[0]
        self.orientation.y = quat[1]
        self.orientation.z = quat[2]
        self.orientation.w = quat[3]
        self.angular_velocity.x = ang_vel[0]
        self.angular_velocity.y = ang_vel[1]
        self.angular_velocity.z = ang_vel[2]
        self.linear_acceleration.x = lin_acc[0]
        self.linear_acceleration.y = lin_acc[1]
        self.linear_acceleration.z = lin_acc[2]

class ROSDtype(Enum):
    FLOAT = Float64
    FLOAT_ARRAY = Float64MultiArray
    JOINT_STATE = JointState
    WRENCH = WrenchStamped
    FORCE = WrenchStamped
    IMU = Imu

def data_to_ros_msg(data, dtype:ROSDtype):
    ros_msg = None
    if dtype == ROSDtype.FLOAT_ARRAY:
        data = np.array(data)
        ros_msg = Float64MultiArray()
        ros_msg.data = data

    elif dtype == ROSDtype.JOINT_STATE:
        ros_msg = JointState()
        ros_msg = data
        ros_msg.header.stamp = rospy.Time.now()

    elif dtype == ROSDtype.FORCE:
        data = np.array(data)
        ros_msg = WrenchStamped()
        ros_msg.wrench.force.x = data[0]
        ros_msg.wrench.force.y = data[1]
        ros_msg.wrench.force.z = data[2]
        ros_msg.wrench.torque.x = 0
        ros_msg.wrench.torque.y = 0
        ros_msg.wrench.torque.z = 0
        ros_msg.header.stamp = rospy.Time.now()

    elif dtype == ROSDtype.WRENCH:
        data = np.array(data)
        ros_msg = WrenchStamped()
        ros_msg.wrench.force.x = data[0]
        ros_msg.wrench.force.y = data[1]
        ros_msg.wrench.force.z = data[2]
        ros_msg.wrench.torque.x = data[3]
        ros_msg.wrench.torque.y = data[4]
        ros_msg.wrench.torque.z = data[5]
        ros_msg.header.stamp = rospy.Time.now()

    elif dtype == ROSDtype.IMU:
        ros_msg = JointState()
        ros_msg = data
        ros_msg.header.stamp = rospy.Time.now()
    return ros_msg


class RosWrapper:
    def __init__(self, rosnode_name):
        self.publishers = {} # name: [dtype, publisher]
        self.subscribers = {} # name: dtype, data
        self.rosnode_name = rosnode_name
        self.use_sim_time = True
        self.ros_time = 0
        rospy.init_node(rosnode_name)
        rospy.loginfo("ROS wrapper init, node name = " + rosnode_name + ".")

    def add_publisher(self, topic, dtype, use_namespace=True, queue=5):
        full_topic = topic
        if use_namespace:
            full_topic = self.rosnode_name + '/' + topic
        pub = rospy.Publisher(full_topic, dtype.value, queue_size=queue)
        self.publishers[topic] = [dtype, pub]

    def add_subscriber(self, topic, dtype, data_handle, use_namespace=True):
        """
        subsribe to a specific topic to update value for data_handle.
        @param data_handle: NOTE: must be mutable objects, including list, dict, and set, bytearray
        """
        assert type(data_handle) in (list, dict, set, bytearray), "data_handle must be mutable objects, including list, dict, and set, bytearray."
        full_topic = topic
        if use_namespace:
            full_topic = self.rosnode_name + '/' + topic
        full_topic = "/" + full_topic
        rospy.Subscriber(full_topic, dtype.value, self.topic_callback)
        self.subscribers[full_topic] = [dtype, data_handle]
        
    def topic_callback(self, msg):
        topic = msg._connection_header['topic']
        if self.subscribers[topic][0] == ROSDtype.FLOAT:
            self.subscribers[topic][1][0] = type(self.subscribers[topic][1][0])(msg.data)
        elif self.subscribers[topic][0] == ROSDtype.FLOAT_ARRAY:
            assert len(self.subscribers[topic][1][0]) == len(type(self.subscribers[topic][1][0])(msg.data)), "inbound float array leng"
            self.subscribers[topic][1][0] = type(self.subscribers[topic][1][0])(msg.data)

    def publish_msg(self, topic, msg):
        assert topic in self.publishers, "topic not registered!"
        self.publishers[topic][1].publish(data_to_ros_msg(msg, self.publishers[topic][0]))

if __name__ == "__main__":
    wrapper = RosWrapper("test_node")
    test_data = [[1,2]]
    wrapper.add_subscriber("test", ROSDtype.FLOAT_ARRAY, test_data)
    while True:
        time.sleep(0.1)
        print(test_data[0])
        
    