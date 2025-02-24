
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
# from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR


spawn = sim_utils.UsdFileCfg(
    usd_path="/home/changmin/.local/share/ov/pkg/isaac-sim-4.2.0/IsaacLab/source/changmin/hand_urdf_description/fr5v6_dual_real.usd",
    activate_contact_sensors=False,
    rigid_props=sim_utils.RigidBodyPropertiesCfg(
        rigid_body_enabled=True,
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
    )
)

init_state = ArticulationCfg.InitialStateCfg(
    pos=(0, 0, 1),
    rot=(0, 0, 0, 0),
    joint_pos={".*": 0.0},
)


AVATAR_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
    usd_path="/home/changmin/.local/share/ov/pkg/isaac-sim-4.2.0/IsaacLab/source/changmin/hand_urdf_description/fr5v6_dual_real.usd",
    activate_contact_sensors=False,
    rigid_props=sim_utils.RigidBodyPropertiesCfg(
        rigid_body_enabled=True,
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
    )
),
    init_state=ArticulationCfg.InitialStateCfg(
    pos=(0, 0, 0.2),
    rot=(1, 0, 0, 0),
    joint_pos={".*": 0.0},
),
    actuators={
        "fingers": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            effort_limit=0.5,
            velocity_limit=100.0,
            stiffness=3.0,
            damping=0.1,
            friction=0.01,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration for a simple hand"""
