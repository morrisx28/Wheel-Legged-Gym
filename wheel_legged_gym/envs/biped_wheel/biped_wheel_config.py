# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from wheel_legged_gym.envs.base.legged_robot_config import (
    LeggedRobotCfg,
    LeggedRobotCfgPPO,
)

class BipedWheelCfg(LeggedRobotCfg):
    class env(LeggedRobotCfg.env):
        num_envs = 4096 # 4096

    class terrain(LeggedRobotCfg.terrain):
        mesh_type = "plane"

    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.5]  # x,y,z [m]
        # default_joint_angles = {  # target angles when action = 0.0
        #     "L_thigh_joint": 1.27,
        #     "L_calf_joint": -2.127,
        #     "L_wheel_joint": 0.0,
        #     "R_thigh_joint": 1.27,
        #     "R_calf_joint": -2.127,
        #     "R_wheel_joint": 0.0,
        # }
        default_joint_angles = {  # target angles when action = 0.0
            "L_thigh_joint": 1.22,
            "L_calf_joint": -1.92,
            "L_wheel_joint": 0.0,
            "R_thigh_joint": 1.22,
            "R_calf_joint": -1.92,
            "R_wheel_joint": 0.0,
        }

    class control(LeggedRobotCfg.control):
        pos_action_scale = 0.5
        vel_action_scale = 10.0
        # PD Drive parameters:
        leg_kp = 25
        leg_kd = 0.3
        stiffness = {"L_thigh_joint": 25.0, "L_calf_joint": 25.0,"L_wheel_joint":0.0, 
                     "R_thigh_joint": 25.0, "R_calf_joint": 25.0,"R_wheel_joint":0.0}  # [N*m/rad]
        damping = {"L_thigh_joint": 0.3, "L_calf_joint": 0.3, "L_wheel_joint": 0.3, 
                   "R_thigh_joint": 0.3, "R_calf_joint": 0.3,"R_wheel_joint": 0.3}  # [N*m*s/rad]
        decimation = 4

    class asset(LeggedRobotCfg.asset):
        file = "{WHEEL_LEGGED_GYM_ROOT_DIR}/resources/robots/quick_bipedal_urdf/urdf/quick_bipedal.urdf"
        name = "Bipedwheel"
        penalize_contacts_on = ["thigh", "calf", "base"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1  # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = False


class BipedWheelCfgPPO(LeggedRobotCfgPPO):

    class algorithm(LeggedRobotCfgPPO.algorithm):
        kl_decay = (
            LeggedRobotCfgPPO.algorithm.desired_kl - 0.002
        ) / LeggedRobotCfgPPO.runner.max_iterations

    class runner(LeggedRobotCfgPPO.runner):
        # logging
        experiment_name = "bipedwheel"
