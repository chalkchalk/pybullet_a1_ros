
import gin
import numpy as np

from pybullet_envs.minitaur.robots import robot_config
import pybullet_data as pd

import sys
sys.path.append("/root/docker_mount/a1_learn/a1_robot")

CONFIG_FILE = ("config/a1_default.gin")
# CONFIG_FILE = ("config/laikago_with_imu.gin")

_MOTOR_KD = [1.0, 2.0, 2.0] * 4


def load_sim_config(render=True):
  """Builds the environment for the quadruped robot.

  Args:
    render: Enable/disable rendering.
  """
  gin.clear_config(clear_constants=False)
  config_file = CONFIG_FILE
  gin.parse_config_file(config_file)

  # Sim bindings
  # Overwrite a few parameters.

  # action_repeat = 4
  # gin.bind_parameter("SimulationParameters.num_action_repeat", action_repeat)
#   gin.bind_parameter("laikago_v2.Laikago.action_repeat", action_repeat)

#   # Control latency is NOT modeled properly for inverse kinematics and
#   # jacobians, as we are directly calling the pybullet API. We will try to fix
#   # this by loading a separate pybullet instance, set the pose and joint
#   # angles which has latency in them, and then run the jacobian/IK.
#   gin.bind_parameter("laikago_v2.Laikago.motor_control_mode",
#                      robot_config.MotorControlMode.HYBRID)
#   # Bump up a bit the adduction/abduction motor d gain for a better tracking.
#   gin.bind_parameter("hybrid_motor_model.HybridMotorModel.kd", _MOTOR_KD)
  gin.bind_parameter("SimulationParameters.enable_rendering", render)


