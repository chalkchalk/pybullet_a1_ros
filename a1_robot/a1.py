import gin
import a1_constants
from pybullet_envs.minitaur.robots import quadruped_base
from pybullet_envs.minitaur.robots import robot_urdf_loader
from ros_wrapper import RosWrapper, ROSDtype
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
        self.ros_wrapper = RosWrapper(a1_constants.ROS_NODE_NAME)
        self.ros_wrapper.add_publisher(a1_constants.ROS_MOTOR_ANG_TOPIC, ROSDtype.FLOAT_ARRAY)
        self.ros_wrapper.add_publisher(a1_constants.ROS_MOTOR_VEL_TOPIC, ROSDtype.FLOAT_ARRAY)

    def ros_info_pub(self):
        motor_angles = self.motor_angles # FL FR RL RR for ros
        motor_vel = self.motor_velocities
        contact_forces = self.feet_contact_forces()
        com_acc = self.base_acceleration_accelerometer
        motor_torques = self.motor_torques
        orientation = self.base_orientation_quaternion_default_frame
        angel_vel_rpy = self.base_roll_pitch_yaw_rate
        # print(orientation)
    
        

    def post_control_step(self):
        self.ros_info_pub()
        

    @classmethod
    def get_constants(cls):
        del cls
        return a1_constants
