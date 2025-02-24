import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on spawning and interacting with an articulation.")
parser.add_argument("--robot", type=str, default="bluehand", help="Name of the robot.")
parser.add_argument("--num_envs", type=int, default=16, help="Number of environments to spawn.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
import numpy as np
# import copy
import omni.isaac.core.utils.prims as prim_utils

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.assets import RigidObject, RigidObjectCfg
from omni.isaac.lab.assets import AssetBaseCfg, ArticulationCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR

from scripts.bluehand import BLUE_ROBIN_CFG, BlueHandController


@configclass
class TableTopSceneCfg(InteractiveSceneCfg):
    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # mount
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/Stand/stand_instanceable.usd", scale=(2.0, 2.0, 2.0)
        ),
    )

    # hand
    robot: ArticulationCfg = BLUE_ROBIN_CFG.replace(prim_path = "{ENV_REGEX_NS}/Robot")



def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop."""
    # Extract scene entities
    robot = scene["robot"]

    # create controller
    hand_controler = BlueHandController()

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0
    count = 0
    reset_count = 0
    number_of_envs = scene.cfg.num_envs

    # resolving the scene entities
    robot_entity_cfg = SceneEntityCfg("robot", joint_names=[".*"], body_names=[".*"])
    robot_entity_cfg.resolve(scene=scene)

    # Simulation loop
    while simulation_app.is_running():
        # Reset
        if count % 1000 == 0:
            # reset counter
            sim_time = 0
            count = 0
            reset_count += 1
            
            # reset robot state
            root_state = robot.data.default_root_state.clone()
            robot.write_root_pose_to_sim(root_state[:, :7])
            robot.write_root_velocity_to_sim(root_state[:, 7:])

            # for debuging. you can delete this
            # a = root_state[:, :7]
            # b = root_state[:, 7:]

            # joint state
            joint_pos, joint_vel = robot.data.joint_pos.clone(), robot.data.joint_vel.clone()
            robot.write_joint_state_to_sim(joint_pos, joint_vel)

            # reset internal state
            robot.reset()
            print(f"[INFO]: Resetting robot state... count {reset_count}")
        
        # set test hand movement
        sin_d = 0.035 + 0.008 * np.sin(count / 100 * np.pi - np.pi / 2)
        aa_value = 0.174533 + 0.174533 * np.sin(count / 100 * np.pi - np.pi / 2)
        env_joint_pos_list = []

        # actuator value
        for _ in range(number_of_envs):
            # size: num_envs x joint
            actuator_values = [aa_value * 2, sin_d, aa_value * 2, sin_d, 0.0, sin_d, -aa_value * 2, sin_d]
            joint_pos_list = hand_controler.to_joint(actuator_values)
            env_joint_pos_list.append(joint_pos_list)        
        joint_pos_target = torch.tensor(env_joint_pos_list, device=sim.device)

        # convert actuator values into joint radian. only env 1
        """
        values[0] = 1st finger // LR
        values[1] = 1st finger // Drill
        values[2] = 2nd finger
        values[3] = 2nd finger
        values[4] = 3rd finger
        values[5] = 3rd finger
        values[6] = 4th finger
        values[7] = 4th finger
        """
        # joint_pos_list = hand_controler.to_lab_tensor(actuator_values)
        # env_joint_pos_list.append(actuator_values)
        
        """
        finger 1 [aa, mcp, pip, dip] finger 2 [aa, mcp, pip, dip] 
        finger 3 [aa, mcp, pip, dip] finger 4 [aa, mcp, pip, dip] 
        finger 1,2,3,4 [act, act, act, act]
        """

        robot.set_joint_position_target(joint_pos_target)
        robot.write_data_to_sim()

        # Perform step
        sim.step()
        sim_time += sim_dt
        count += 1

        # Update buffers
        robot.update(sim_dt)
        # cone_obj.update(sim_dt)


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)

    # Set main camera
    sim.set_camera_view([2.5, 0.0, 2.0], [0.0, 0.0, 0.3])

    # Design scene
    scene_cfg = TableTopSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    print("[INFO]: Design scene is complete...")

    # Play the simulator
    sim.reset()

    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
    

