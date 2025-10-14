# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import SceneEntityCfg
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab.envs.mdp.terminations import link_height_below_minimum

from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg

##
# Pre-defined configs
##
from isaaclab_assets.robots.unitree import UNITREE_GO2W_CFG  # isort: skip


@configclass
class UnitreeGo2WRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        self.scene.robot = UNITREE_GO2W_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # Match the base link name in go2w.usd (seen as .../go2w/base_link/...)
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/go2w/base_link"
        # scale down the terrains because the robot is small
        self.scene.terrain.terrain_generator.sub_terrains["boxes"].grid_height_range = (0.025, 0.1)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0.01, 0.06)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_step = 0.01

        # reduce action scale
        self.actions.joint_pos.scale = 0.25

        # event
        self.events.push_robot = None
        # disable mass randomization to avoid issues with custom USD and differing API expectations
        self.events.add_base_mass = None
        self.events.base_external_force_torque.params["asset_cfg"].body_names = "base"
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }
        self.events.base_com = None

        # disable scene contact sensor and feet air-time reward
        self.scene.contact_forces = None
        self.rewards.feet_air_time = None
        self.rewards.undesired_contacts = None
        self.rewards.dof_torques_l2.weight = -0.0002
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        self.rewards.track_ang_vel_z_exp.weight = 0.75
        self.rewards.dof_acc_l2.weight = -2.5e-7
        # encourage high hip height
        self.rewards.hip_max_height.weight = 5.0
        self.rewards.hip_max_height.params["asset_cfg"] = SceneEntityCfg("robot", body_names=".*_hip")

        # terminations: repurpose base_contact to hip height-based done at z<=0 (no contact sensor needed)
        self.terminations.base_contact = DoneTerm(
            func=link_height_below_minimum,
            params={
                "minimum_height": 0.0,
                "asset_cfg": SceneEntityCfg("robot", body_names=[".*_hip"]),
            },
        )

        # terminations: repurpose base_contact to hip height-based done at z<=0 (no contact sensor needed)
        self.terminations.base_contact = DoneTerm(
            func=link_height_below_minimum,
            params={
                "minimum_height": 0.0,
                "asset_cfg": SceneEntityCfg("robot", body_names=[".*_thigh"]),
            },
        )


@configclass
class UnitreeGo2WRoughEnvCfg_PLAY(UnitreeGo2WRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        # reduce the number of terrains to save memory
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing event
        self.events.base_external_force_torque = None
        self.events.push_robot = None
