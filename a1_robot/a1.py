import gin
import a1_constants
from pybullet_envs.minitaur.robots import quadruped_base
from pybullet_envs.minitaur.robots import robot_urdf_loader
from ros_wrapper import RosWrapper, ROSDtype, RobotJointState, ImuData
@gin.configurable
class A1(quadruped_base.QuadrupedBase):

    def _pre_load(self):
        """Import the Laikago specific constants.
        """
        self._urdf_loader = robot_urdf_loader.RobotUrdfLoader(
                pybullet_client=self._pybullet_client,
                urdf_path=a1_constants.URDF_PATH,
                enable_self_collision=False,
                init_base_position=a1_constants.INIT_POSITION,
                init_base_orientation_quaternion=a1_constants.INIT_ORIENTATION,
                init_joint_angles=a1_constants.INIT_JOINT_ANGLES,
                joint_offsets=a1_constants.JOINT_OFFSETS,
                joint_directions=a1_constants.JOINT_DIRECTIONS,
                motor_names=a1_constants.MOTOR_NAMES,
                end_effector_names=a1_constants.END_EFFECTOR_NAMES,
                user_group=a1_constants.MOTOR_GROUP,
        )
        self.ros_wrapper_init()

    def ros_wrapper_init(self):
        self.ros_wrapper = RosWrapper(a1_constants.ROS_NODE_NAME)
        self.ros_wrapper.add_publisher(a1_constants.ROS_MOTOR_ANG_TOPIC, ROSDtype.FLOAT_ARRAY)
        self.ros_wrapper.add_publisher(a1_constants.ROS_MOTOR_VEL_TOPIC, ROSDtype.FLOAT_ARRAY)
        self.ros_wrapper.add_publisher(a1_constants.ROS_JOINTSTATE_TOPIC, ROSDtype.JOINT_STATE)
        self.ros_wrapper.add_publisher(a1_constants.ROS_FOOT_CONTACT_FORCE_TOPIC[0], ROSDtype.FORCE)
        self.ros_wrapper.add_publisher(a1_constants.ROS_FOOT_CONTACT_FORCE_TOPIC[1], ROSDtype.FORCE)
        self.ros_wrapper.add_publisher(a1_constants.ROS_FOOT_CONTACT_FORCE_TOPIC[2], ROSDtype.FORCE)
        self.ros_wrapper.add_publisher(a1_constants.ROS_FOOT_CONTACT_FORCE_TOPIC[3], ROSDtype.FORCE)
        self.ros_wrapper.add_publisher(a1_constants.ROS_IMU_TOPIC, ROSDtype.IMU)
        
        

    def ros_info_pub(self):
        motor_angles = self.motor_angles # FL FR RL RR for ros
        motor_vel = self.motor_velocities
        contact_forces = self.feet_contact_forces()
        com_acc = self.base_acceleration_accelerometer
        motor_torques = self.motor_torques
        joint_state = RobotJointState(a1_constants.JOINT_NAMES, motor_angles, motor_vel, motor_torques)
        orientation = self.base_orientation_quaternion_default_frame
        angel_vel_rpy = self.base_roll_pitch_yaw_rate
        imu_data = ImuData(orientation, angel_vel_rpy, com_acc)
        self.ros_wrapper.publish_msg(a1_constants.ROS_MOTOR_ANG_TOPIC, motor_angles)
        self.ros_wrapper.publish_msg(a1_constants.ROS_JOINTSTATE_TOPIC, joint_state)
        self.ros_wrapper.publish_msg(a1_constants.ROS_FOOT_CONTACT_FORCE_TOPIC[0], contact_forces[0])
        self.ros_wrapper.publish_msg(a1_constants.ROS_FOOT_CONTACT_FORCE_TOPIC[1], contact_forces[1])
        self.ros_wrapper.publish_msg(a1_constants.ROS_FOOT_CONTACT_FORCE_TOPIC[2], contact_forces[2])
        self.ros_wrapper.publish_msg(a1_constants.ROS_FOOT_CONTACT_FORCE_TOPIC[3], contact_forces[3])
        self.ros_wrapper.publish_msg(a1_constants.ROS_IMU_TOPIC, imu_data)
        # print(orientation)
    
        
    def post_control_step(self):
        self.ros_info_pub()
        

    @classmethod
    def get_constants(cls):
        del cls
        return a1_constants
