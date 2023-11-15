import collections
import gin

URDF_PATH = "a1/a1.urdf"
NUM_MOTORS = 12
NUM_LEGS = 4
MOTORS_PER_LEG = 3

INIT_RACK_POSITION = [0, 0, 1]
INIT_POSITION = [0, 0, 0.48]

# Will be default to (0, 0, 0, 1) once the new a1_toes_zup.urdf checked in.
INIT_ORIENTATION = [0, 0, 0, 1]


MOTOR_NAMES = [
            "FR_hip_joint",
            "FR_upper_joint",
            "FR_lower_joint",
            "FL_hip_joint",
            "FL_upper_joint",
            "FL_lower_joint",
            "RR_hip_joint",
            "RR_upper_joint",
            "RR_lower_joint",
            "RL_hip_joint",
            "RL_upper_joint",
            "RL_lower_joint",
        ]

JOINT_NAMES = MOTOR_NAMES

END_EFFECTOR_NAMES = [
            "FR_toe_fixed",
            "FL_toe_fixed",
            "RR_toe_fixed",
            "RL_toe_fixed"
        ]

HIP_JOINT_OFFSET = 0.0
UPPER_LEG_JOINT_OFFSET = -0.6
KNEE_JOINT_OFFSET = 0.66

JOINT_OFFSETS = collections.OrderedDict(
    zip(JOINT_NAMES,
        [HIP_JOINT_OFFSET, UPPER_LEG_JOINT_OFFSET, KNEE_JOINT_OFFSET] *
        NUM_LEGS))


JOINT_DIRECTIONS = collections.OrderedDict(
    zip(JOINT_NAMES, (-1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1)))

INIT_ABDUCTION_ANGLE = 0
INIT_HIP_ANGLE = 0.9
INIT_KNEE_ANGLE = -1.8

INIT_JOINT_ANGLES = collections.OrderedDict(
    zip(JOINT_NAMES,
        (INIT_ABDUCTION_ANGLE, INIT_HIP_ANGLE, INIT_KNEE_ANGLE) * NUM_LEGS))

LEG_NAMES = (
    "front_right",
    "front_left",
    "rear_right",
    "rear_left",
)


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
