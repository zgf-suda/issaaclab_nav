# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from .rough_env_cfg import AnymalCRoughEnvCfg
import isaaclab.sim as sim_utils

@configclass
class AnymalCFlatEnvCfg(AnymalCRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # override rewards
        self.rewards.flat_orientation_l2.weight = -5.0
        self.rewards.dof_torques_l2.weight = -2.5e-5
        self.rewards.feet_air_time.weight = 0.5
        # change terrain to flat
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        # no height scan
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        # no terrain curriculum
        self.curriculum.terrain_levels = None

        cfg_cuboid_01 = sim_utils.MeshCuboidCfg(
                size=(1, 1, 1),  # 立方体的尺寸
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.1, 0.1)),  # 蓝色
                rigid_props=sim_utils.RigidBodyPropertiesCfg(),
                mass_props=sim_utils.MassPropertiesCfg(mass=1000.0),
                collision_props=sim_utils.CollisionPropertiesCfg(),
            )   
        cfg_cuboid_01.func("/World/ground/Cube_01", cfg_cuboid_01, translation=(2, -1.5, 0), orientation=(0, 0.0, 0, 0.0))     

        cfg_cuboid_02 = sim_utils.MeshCuboidCfg(
            size=(1, 1, 1),  # 立方体的尺寸
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.1, 0.1)),  # 蓝色
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=1000.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        )   
        cfg_cuboid_02.func("/World/ground/Cube_02", cfg_cuboid_02, translation=(2, 2, 0), orientation=(0, 0.0, 0, 0.0))     

        cfg_cuboid_03 = sim_utils.MeshCuboidCfg(
            size=(1, 1, 1),  # 立方体的尺寸
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.1, 0.1)),  # 蓝色
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=1000.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        )   
        cfg_cuboid_03.func("/World/ground/Cube_03", cfg_cuboid_03, translation=(-2, 0, 0), orientation=(0, 0.0, 0, 0.0)) 

        # cfg_capsule = sim_utils.MeshCapsuleCfg(
        #     radius =0.3,
        #     height = 0.3,
        #     visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.1, 0.1)),  # 蓝色
        #     rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        #     mass_props=sim_utils.MassPropertiesCfg(mass=100.0),
        #     collision_props=sim_utils.CollisionPropertiesCfg(),
        # )
        # cfg_capsule.func("/World/ground/Capsule", cfg_capsule, translation=(-1.42922, 0.32566, 0), orientation=(0, 0.0, 0, 0.0)) 

        # cfg_cylinder = sim_utils.MeshCylinderCfg(
        #     radius =0.3,
        #     height = 0.3,
        #     visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.1, 0.1)),  # 蓝色
        #     rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        #     mass_props=sim_utils.MassPropertiesCfg(mass=100.0),
        #     collision_props=sim_utils.CollisionPropertiesCfg(),
        # )
        # cfg_cylinder.func("/World/ground/Cylinder",cfg_cylinder,translation=(-0.07727, 1.28982, 0), orientation=(0, 0.0, 0, 0.0))

class AnymalCFlatEnvCfg_PLAY(AnymalCFlatEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing event
        self.events.base_external_force_torque = None
        self.events.push_robot = None
