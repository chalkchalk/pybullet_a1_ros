import gin
import a1_constants
from pybullet_envs.minitaur.robots import quadruped_base
from pybullet_envs.minitaur.robots import robot_urdf_loader

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

  @classmethod
  def get_constants(cls):
    del cls
    return a1_constants
