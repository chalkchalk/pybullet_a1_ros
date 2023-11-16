import collections
import gin

# URDF_PATH = "a1/a1.urdf"
URDF_PATH = "/root/docker_mount/pybullet_a1_ros/a1_robot/urdf/a1.urdf"
NUM_MOTORS = 12
NUM_LEGS = 4
MOTORS_PER_LEG = 3

INIT_RACK_POSITION = [0, 0, 1]
INIT_POSITION = [0, 0, 0.48]

# Will be default to (0, 0, 0, 1) once the new a1_toes_zup.urdf checked in.
INIT_ORIENTATION = [0, 0, 0, 1]


# MOTOR_NAMES = [
#             "FR_hip_joint",
#             "FR_upper_joint",
#             "FR_lower_joint",
#             "FL_hip_joint",
#             "FL_upper_joint",
#             "FL_lower_joint",
#             "RR_hip_joint",
#             "RR_upper_joint",
#             "RR_lower_joint",
#             "RL_hip_joint",
#             "RL_upper_joint",
#             "RL_lower_joint",
#         ]

# MOTOR_NAMES = [
#             "FL_hip_joint",
#             "FL_upper_joint",
#             "FL_lower_joint",
#             "FR_hip_joint",
#             "FR_upper_joint",
#             "FR_lower_joint",
#             "RL_hip_joint",
#             "RL_upper_joint",
#             "RL_lower_joint",
#             "RR_hip_joint",
#             "RR_upper_joint",
#             "RR_lower_joint"
#         ]

MOTOR_NAMES = ["FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"]


JOINT_NAMES = MOTOR_NAMES

# END_EFFECTOR_NAMES = [
#             "FL_toe_fixed",
#             "FR_toe_fixed",
#             "RL_toe_fixed",
#             "RR_toe_fixed",
#         ]

END_EFFECTOR_NAMES = [
            "FL_foot_fixed",
            "FR_foot_fixed",
            "RL_foot_fixed",
            "RR_foot_fixed",
        ]

HIP_JOINT_OFFSET = 0.0
UPPER_LEG_JOINT_OFFSET = -0.6
KNEE_JOINT_OFFSET = 0.66

JOINT_OFFSETS = collections.OrderedDict(
    zip(JOINT_NAMES,
        [HIP_JOINT_OFFSET, UPPER_LEG_JOINT_OFFSET, KNEE_JOINT_OFFSET] *
        NUM_LEGS))


JOINT_DIRECTIONS = collections.OrderedDict(
    zip(JOINT_NAMES, (1, 1, 1, -1, 1, 1, 1, 1, 1, -1, 1, 1)))

INIT_ABDUCTION_ANGLE = 0
INIT_HIP_ANGLE = 0.9
INIT_KNEE_ANGLE = -1.8

INIT_JOINT_ANGLES = collections.OrderedDict(
    zip(JOINT_NAMES,
        (INIT_ABDUCTION_ANGLE, INIT_HIP_ANGLE, INIT_KNEE_ANGLE) * NUM_LEGS))

LEG_NAMES = (
    "front_left",
    "front_right",
    "rear_left",
    "rear_right"
)

ROS_NODE_NAME = "a1_pybullet"
ROS_MOTOR_ANG_TOPIC = "motor_angle"
ROS_MOTOR_VEL_TOPIC = "motor_vel"
ROS_JOINTSTATE_TOPIC = "joint_states"
ROS_FOOT_CONTACT_FORCE_TOPIC = ["FL_foot_force","FR_foot_force","RL_foot_force","RR_foot_force"]
ROS_IMU_TOPIC = "imu"

MOTOR_GROUP = collections.OrderedDict((
    (LEG_NAMES[0], JOINT_NAMES[0:3]),
    (LEG_NAMES[1], JOINT_NAMES[3:6]),
    (LEG_NAMES[2], JOINT_NAMES[6:9]),
    (LEG_NAMES[3], JOINT_NAMES[9:12]),
))

gin.constant("a1_constants.A1_NUM_MOTORS", NUM_MOTORS)
gin.constant("a1_constants.A1_URDF_PATH", URDF_PATH)
gin.constant("a1_constants.A1_INIT_POSITION", INIT_POSITION)
gin.constant("a1_constants.A1_INIT_ORIENTATION", INIT_ORIENTATION)
gin.constant("a1_constants.A1_INIT_JOINT_ANGLES", INIT_JOINT_ANGLES)
gin.constant("a1_constants.A1_JOINT_DIRECTIONS", JOINT_DIRECTIONS)
gin.constant("a1_constants.A1_JOINT_OFFSETS", JOINT_OFFSETS)
gin.constant("a1_constants.A1_MOTOR_NAMES", MOTOR_NAMES)
gin.constant("a1_constants.A1_END_EFFECTOR_NAMES", END_EFFECTOR_NAMES)
gin.constant("a1_constants.A1_MOTOR_GROUP", MOTOR_GROUP)
