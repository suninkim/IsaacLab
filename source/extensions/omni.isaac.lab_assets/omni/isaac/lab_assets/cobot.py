# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Franka Emika robots.

The following configurations are available:

* :obj:`FRANKA_PANDA_CFG`: Franka Emika Panda robot with Panda hand
* :obj:`FRANKA_PANDA_HIGH_PD_CFG`: Franka Emika Panda robot with Panda hand with stiffer PD control

Reference: https://github.com/frankaemika/franka_ros
"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

COBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/home/sunin/workspace/City_of_Angels/asset/robot/urdf/cobot/cobot.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        rot=(0.7071068, 0.0, 0.0, 0.7071068 ),
        joint_pos={
            "cobot_joint1": 1.570796,
            "cobot_joint2": 0.5,
            "cobot_joint3": 0.3,
            "cobot_joint4": 0.8,
            "cobot_joint5": -1.570796,
            "cobot_joint6": 1.570796,
            "gripper_controller": 0.0,
        },
    ),
    actuators={
        "cobot_actuator": ImplicitActuatorCfg(
            joint_names_expr=["cobot_joint[1-6]"],
            effort_limit=87.0,
            velocity_limit=2.175,
            stiffness=80.0,
            damping=4.0,
        ),
        "cobot_hand": ImplicitActuatorCfg(
            joint_names_expr=["gripper_controller"],
            effort_limit=200.0,
            velocity_limit=0.2,
            stiffness=2e3,
            damping=1e2,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of Franka Emika Panda robot."""


COBOT_HIGH_PD_CFG = COBOT_CFG.copy()
COBOT_HIGH_PD_CFG.spawn.rigid_props.disable_gravity = True
COBOT_HIGH_PD_CFG.actuators["cobot_actuator"].stiffness = 400.0
COBOT_HIGH_PD_CFG.actuators["cobot_actuator"].damping = 80.0
"""Configuration of Franka Emika Panda robot with stiffer PD control.

This configuration is useful for task-space control using differential IK.
"""
