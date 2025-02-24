import math
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
import numpy as np
import torch
# from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR


blue_spawn = sim_utils.UsdFileCfg(
    usd_path="/home/changmin/.local/share/ov/pkg/isaac-sim-4.2.0/IsaacLab/source/changmin/hand_urdf_description/bluehand.usd",
    activate_contact_sensors=False,
    rigid_props=sim_utils.RigidBodyPropertiesCfg(
        disable_gravity=False,
        rigid_body_enabled=True,
        angular_damping=0.01,
        max_linear_velocity=1000.0,
        max_angular_velocity=1000.0,
        max_depenetration_velocity=100.0,
        enable_gyroscopic_forces=True,
    ),
    articulation_props=sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False,
        solver_position_iteration_count=4,
        solver_velocity_iteration_count=0,
        sleep_threshold=0.005,
        stabilization_threshold=0.001,
    ),
    # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    joint_drive_props=sim_utils.JointDrivePropertiesCfg(drive_type="force"),
    fixed_tendons_props=sim_utils.FixedTendonPropertiesCfg(limit_stiffness=30.0, damping=0.1),
)

blue_init_state = ArticulationCfg.InitialStateCfg(
    pos=(0, 0, 0.2),
    rot=(1, 0, 0, 0),
    joint_pos={".*": 0.0},
)


BLUE_ROBIN_CFG = ArticulationCfg(
    spawn=blue_spawn,
    init_state=blue_init_state,
    actuators={
        "fingers": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            effort_limit=0.5,
            velocity_limit=1.0,
            stiffness=5.0,
            damping=0.5,
            friction=0.01,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration for a simple hand"""


# actuator = {
#     "r1": ImplicitActuatorCfg(),
#     "r2": ImplicitActuatorCfg(),
#     "r3": ImplicitActuatorCfg(),
#     "r4": ImplicitActuatorCfg()
# }

class BlueHandController:
    def __init__(self):
        self.input_format = [0, 0, 0, 0, 0, 0, 0, 0]
        self.MODE = 0
        pass

    def kinematicsCalculation_RHand(self, values):
        DA11 = values[0]
        DA12 = values[1] # this one
        DA21 = values[2]
        DA22 = values[3] # this one
        DA31 = values[4]
        DA32 = values[5] # this one
        DA41 = values[6]
        DA42 = values[7] # this one

        finger_1 = [DA11] + self.oneFingerFlexionCalculation(DA12)
        finger_2 = [DA21] + self.oneFingerFlexionCalculation(DA22)
        finger_3 = [DA31] + self.oneFingerFlexionCalculation(DA32)
        finger_4 = [DA41] + self.oneFingerFlexionCalculation(DA42)

        finger_act = [finger_1[4], finger_2[4], finger_3[4], finger_4[4]]
        return [finger_1[:4], finger_2[:4], finger_3[:4], finger_4[:4], finger_act]
    
    
    def oneFingerFlexionCalculation(self, distance):
        # Reference : Sung, Eunho, et al. "SNU-Avatar Robot Hand: Dexterous Robot
        #             Hand with Prismatic Four-Bar Linkage for Versatile Daily
        #             Applications." 2023 IEEE-RAS 22nd International Conference on
        #             Humanoid Robots (Humanoids). IEEE, 2023.
        # unit : [m]
        l_p = 0.028
        h   = 0.0025
        l_1 = 0.01686
        l_2 = 0.02638
        l_3 = 0.00638
        l_4 = 0.03500
        l_5 = 0.04000
        l_6 = 0.00550
        l_7 = 0.01000
        l_8 = 0.03252
        l_9 = 0.03420
        l_10 = 0.01356

        # unit : [rad]
        alpha_prime = np.deg2rad(10.9)
        gamma_0 = np.deg2rad(37.8)
        zeta_prime = np.deg2rad(31.9)
        kappa_0 = np.deg2rad(24.1)

        # The length of the sliding screw (linear actuator value : 2000~6000)
        # d [m]
        d = distance

        s_1 = np.sqrt(np.power(d, 2) + np.power(h, 2))
        alpha = np.arccos((np.power(l_1, 2) + np.power(l_2, 2) - np.power(s_1, 2)) / (2*l_1*l_2))
        beta = alpha + alpha_prime
        s_2 = np.sqrt(np.power(l_3, 2) + np.power(l_4, 2) - 2*l_3*l_4*np.cos(beta))
        gamma = np.arccos((np.power(l_5, 2) + np.power(l_6, 2) - np.power(s_2, 2)) / (2*l_5*l_6))

        zeta = np.arccos((np.power(l_4, 2) + np.power(s_2, 2) - np.power(l_3, 2)) / (2*l_4*s_2)) \
            - np.arccos((np.power(l_6, 2) + np.power(s_2, 2) - np.power(l_5, 2)) / (2*l_6*s_2))
        
        l_hk1 = np.sqrt(np.power(l_6, 2)+np.power(l_4,2)-2*l_6*l_4*np.cos(zeta))
        theta_hk1 = np.arccos((np.power(l_5, 2) + np.power(l_3, 2) - np.power(l_hk1, 2)) / (2*l_5*l_3))
        eta = zeta + zeta_prime

        s_3 = np.sqrt(np.power(l_7, 2) + np.power(l_8, 2) - 2*l_7*l_8*np.cos(eta))
        kappa = np.arccos((np.power(l_10, 2) + np.power(s_3, 2) - np.power(l_9, 2)) / (2*l_10*s_3))

        s_4 = np.sqrt(np.power(l_7,2) + np.power(l_6,2) - 2*l_7*l_6*np.cos(eta))
        l_p = 0.028

        kappa_2 = np.arccos((np.power(s_3, 2) + np.power(l_p, 2) - np.power(s_4, 2)) / (2*s_3*l_p))

        theta_mcp = np.deg2rad(90) - (theta_hk1 - np.deg2rad(25))
        theta_pip = gamma - gamma_0
        theta_dip = kappa + kappa_2 - kappa_0

        #huristic compensation
        theta_mcp = theta_mcp - np.deg2rad(30)
        theta_pip = theta_pip + np.deg2rad(15)
        theta_dip = theta_dip - np.deg2rad(50)

        return [theta_mcp, theta_pip, theta_dip, 0.8*theta_mcp]
    
    # need to be modified for multiple-env
    def to_joint(self, values):
        """
        DA11 = values[0] 1st finger
        DA12 = values[1] 1st finger
        DA21 = values[2] 2nd finger
        DA22 = values[3] 2nd finger
        DA31 = values[4] 3rd finger
        DA32 = values[5] 3rd finger
        DA41 = values[6] 4th finger
        DA42 = values[7] 4th finger
        """
        joint_pos_list = self.kinematicsCalculation_RHand(values)
        finger1 = joint_pos_list[0]
        finger2 = joint_pos_list[1]
        finger3 = joint_pos_list[2]
        finger4 = joint_pos_list[3]
        act = joint_pos_list[4]

        joint_val_list = [finger1[0], finger2[0], finger3[0], finger4[0],
                          act[0], finger1[1], act[1], finger2[1],
                          act[2], finger3[1], act[3], finger4[1],
                          finger1[2], finger2[2], finger3[2], finger4[2],
                          finger1[3], finger2[3], finger3[3], finger4[3]]
        return joint_val_list
        
