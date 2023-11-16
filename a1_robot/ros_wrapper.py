import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import WrenchStamped
from enum import Enum

class RobotJointState(JointState):
    def __init__(self, name, pos, vel, tor):
        super().__init__()
        self.name = name
        self.position =pos
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
        self.publishers = {}
        self.rosnode_name = rosnode_name
        self.use_sim_time = True
        self.ros_time = 0
        rospy.init_node(rosnode_name)
        rospy.loginfo("ROS wrapper init, node name = " + rosnode_name + ".")

    def add_publisher(self, topic, dtype, use_namespace=True, queue=5):
        full_topic = self.rosnode_name + '/' + topic
        if use_namespace:
            full_topic = self.rosnode_name + '/' + topic
        pub = rospy.Publisher(full_topic, dtype.value, queue_size=queue)
        self.publishers[topic] = [dtype, pub]

    def publish_msg(self, topic, msg):
        assert topic in self.publishers, "topic not registered!"
        self.publishers[topic][1].publish(data_to_ros_msg(msg, self.publishers[topic][0]))
        