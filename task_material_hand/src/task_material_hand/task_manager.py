#!/usr/bin/env python
# -*- coding: utf-8 -*-

# FSRobo-R Package BSDL
# ---------
# Copyright (C) 2019 FUJISOFT. All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation and/or
# other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ---------

from task_common.key import scenario_key
from task_common.action_interface import action_manager
from . import task_move
from . import task_home
from . import task_pick
from . import task_place
from . import task_get_current_joint
from . import task_get_io
from . import task_set_io
from . import task_joint_move
from . import task_get_object_position
from . import task_line_move
from . import task_wait_io
from . import task_relative_move


class TaskManager:
    """
    タスク管理クラス
    """

    def __init__(self, structure_info):
        # アクションクラスインスタンス化
        self._action_manager = action_manager.ActionManager(structure_info)

    def task_execute_home(self, arm_name):
        param = {}
        param[scenario_key.PARAM_KEY_ARM_ARM_NAME] = arm_name
        return task_home.home(self._action_manager, param)

    def task_execute_move(self, arm_name, pos, tool_name, velocity=0.5, plan_key=None):
        param = {}
        param[scenario_key.PARAM_KEY_ARM_ARM_NAME] = arm_name
        param[scenario_key.PARAM_KEY_COMMON_X] = pos.x
        param[scenario_key.PARAM_KEY_COMMON_Y] = pos.y
        param[scenario_key.PARAM_KEY_COMMON_Z] = pos.z
        param[scenario_key.PARAM_KEY_COMMON_RX] = pos.rx
        param[scenario_key.PARAM_KEY_COMMON_RY] = pos.ry
        param[scenario_key.PARAM_KEY_COMMON_RZ] = pos.rz
        param[scenario_key.PARAM_KEY_COMMON_POSTURE] = pos.posture
        param[scenario_key.PARAM_KEY_COMMON_TOOL_NAME] = tool_name
        param[scenario_key.PARAM_KEY_ARM_VELOCITY] = velocity
        param[scenario_key.PARAM_KEY_ARM_PLAN_KEY] = plan_key
        return task_move.move(self._action_manager, param)

    def task_execute_joint_move(self, arm_name, joint, velocity=0.5, plan_key=None):
        param = {}
        param[scenario_key.PARAM_KEY_ARM_ARM_NAME] = arm_name
        param[scenario_key.PARAM_KEY_ARM_JOINT_LIST] = joint
        param[scenario_key.PARAM_KEY_ARM_VELOCITY] = velocity
        param[scenario_key.PARAM_KEY_ARM_PLAN_KEY] = plan_key
        return task_joint_move.joint_move(self._action_manager, param)

    def task_execute_line_move(self, arm_name, pos, tool_name, velocity=0.5, plan_key=None):
        param = {}
        param[scenario_key.PARAM_KEY_ARM_ARM_NAME] = arm_name
        param[scenario_key.PARAM_KEY_COMMON_X] = pos.x
        param[scenario_key.PARAM_KEY_COMMON_Y] = pos.y
        param[scenario_key.PARAM_KEY_COMMON_Z] = pos.z
        param[scenario_key.PARAM_KEY_COMMON_RX] = pos.rx
        param[scenario_key.PARAM_KEY_COMMON_RY] = pos.ry
        param[scenario_key.PARAM_KEY_COMMON_RZ] = pos.rz
        param[scenario_key.PARAM_KEY_COMMON_POSTURE] = pos.posture
        param[scenario_key.PARAM_KEY_COMMON_TOOL_NAME] = tool_name
        param[scenario_key.PARAM_KEY_ARM_PRECISION_VELOCITY] = velocity
        param[scenario_key.PARAM_KEY_ARM_PLAN_KEY] = plan_key
        return task_line_move.line_move(self._action_manager, param)

    def task_execute_pick(self, arm_name, hand_name, pos, tool_name, velocity=0.5, precision_velocity=0.05, down_distance=100, plan_key=None, force_option=None):
        param = {}
        param[scenario_key.PARAM_KEY_ARM_ARM_NAME] = arm_name
        param[scenario_key.PARAM_KEY_TOOL_HAND_NAME] = hand_name
        param[scenario_key.PARAM_KEY_COMMON_X] = pos.x
        param[scenario_key.PARAM_KEY_COMMON_Y] = pos.y
        param[scenario_key.PARAM_KEY_COMMON_Z] = pos.z
        param[scenario_key.PARAM_KEY_COMMON_RX] = pos.rx
        param[scenario_key.PARAM_KEY_COMMON_RY] = pos.ry
        param[scenario_key.PARAM_KEY_COMMON_RZ] = pos.rz
        param[scenario_key.PARAM_KEY_COMMON_POSTURE] = pos.posture
        param[scenario_key.PARAM_KEY_COMMON_TOOL_NAME] = tool_name
        param[scenario_key.PARAM_KEY_ARM_VELOCITY] = velocity
        param[scenario_key.PARAM_KEY_ARM_PRECISION_VELOCITY] = precision_velocity
        param[scenario_key.PARAM_KEY_ARM_DOWN_DISTANCE] = down_distance
        param[scenario_key.PARAM_KEY_ARM_PLAN_KEY] = plan_key
        if force_option is not None:
            param[scenario_key.PARAM_KEY_FORCE_AXIS] = force_option.axis
            param[scenario_key.PARAM_KEY_FORCE_NEWTON] = force_option.newton
            param[scenario_key.PARAM_KEY_FORCE_PICK_CHECK] = force_option.pick_check
            param[scenario_key.PARAM_KEY_FORCE_FAILED_UPPER_LIMIT] = force_option.failed_upper_limit
        else:
            param[scenario_key.PARAM_KEY_FORCE_AXIS] = ""
            param[scenario_key.PARAM_KEY_FORCE_NEWTON] = 0
            param[scenario_key.PARAM_KEY_FORCE_PICK_CHECK] = False
            param[scenario_key.PARAM_KEY_FORCE_FAILED_UPPER_LIMIT] = 0
        return task_pick.pick(self._action_manager, param)

    def task_execute_place(self, arm_name, hand_name, pos, tool_name, velocity=0.5, precision_velocity=0.05, down_distance=100, plan_key=None, force_option=None):
        param = {}
        param[scenario_key.PARAM_KEY_ARM_ARM_NAME] = arm_name
        param[scenario_key.PARAM_KEY_TOOL_HAND_NAME] = hand_name
        param[scenario_key.PARAM_KEY_COMMON_X] = pos.x
        param[scenario_key.PARAM_KEY_COMMON_Y] = pos.y
        param[scenario_key.PARAM_KEY_COMMON_Z] = pos.z
        param[scenario_key.PARAM_KEY_COMMON_RX] = pos.rx
        param[scenario_key.PARAM_KEY_COMMON_RY] = pos.ry
        param[scenario_key.PARAM_KEY_COMMON_RZ] = pos.rz
        param[scenario_key.PARAM_KEY_COMMON_POSTURE] = pos.posture
        param[scenario_key.PARAM_KEY_COMMON_TOOL_NAME] = tool_name
        param[scenario_key.PARAM_KEY_ARM_VELOCITY] = velocity
        param[scenario_key.PARAM_KEY_ARM_PRECISION_VELOCITY] = precision_velocity
        param[scenario_key.PARAM_KEY_ARM_DOWN_DISTANCE] = down_distance
        param[scenario_key.PARAM_KEY_ARM_PLAN_KEY] = plan_key
        if force_option is not None:
            param[scenario_key.PARAM_KEY_FORCE_AXIS] = force_option.axis
            param[scenario_key.PARAM_KEY_FORCE_NEWTON] = force_option.newton
        else:
            param[scenario_key.PARAM_KEY_FORCE_AXIS] = ""
            param[scenario_key.PARAM_KEY_FORCE_NEWTON] = 0
        return task_place.place(self._action_manager, param)

    def task_execute_get_current_joint(self, arm_name):
        param = {}
        param[scenario_key.PARAM_KEY_ARM_ARM_NAME] = arm_name
        return task_get_current_joint.get_current_joint(self._action_manager, param)

    def task_get_io(self, io_name, address=None):
        param = {}
        param[scenario_key.PARAM_KEY_IO_NAME] = io_name
        if address is not None:
            param[scenario_key.PARAM_KEY_IO_ADDRESS] = address
        return task_get_io.get_io(self._action_manager, param)

    def task_set_io(self, io_name, address, value, wait=False):
        param = {}
        param[scenario_key.PARAM_KEY_IO_NAME] = io_name
        param[scenario_key.PARAM_KEY_IO_ADDRESS] = address
        param[scenario_key.PARAM_KEY_IO_SET_VALUE] = value
        param[scenario_key.PARAM_KEY_IO_WAIT] = wait
        return task_set_io.set_io(self._action_manager, param)

    def task_wait_io(self, io_name, address, value, time_out=None):
        param = {}
        param[scenario_key.PARAM_KEY_IO_NAME] = io_name
        param[scenario_key.PARAM_KEY_IO_ADDRESS] = address
        param[scenario_key.PARAM_KEY_IO_SET_VALUE] = value
        param[scenario_key.PARAM_KEY_IO_TIME_OUT] = time_out
        return task_wait_io.wait_io(self._action_manager, param)

    def task_get_object_position(self, arm_name, object_name, timeout=4.0, rz=None, ry=None, rx=None):
        param = {}
        param[scenario_key.PARAM_KEY_ARM_ARM_NAME] = arm_name
        param[scenario_key.PARAM_KEY_COMMON_OBJECT_NAME] = object_name
        param[scenario_key.PARAM_KEY_COMMON_TIMEOUT] = timeout
        param[scenario_key.PARAM_KEY_COMMON_RZ] = rz
        param[scenario_key.PARAM_KEY_COMMON_RY] = ry
        param[scenario_key.PARAM_KEY_COMMON_RX] = rx
        return task_get_object_position.get_object_position(self._action_manager, param)

    def task_relative_move(self, arm_name, x, y, z, rz=0, ry=0, rx=0, velocity=0.5, move_type=0, plan_key=None):
        param = {}
        param[scenario_key.PARAM_KEY_ARM_ARM_NAME] = arm_name
        param[scenario_key.PARAM_KEY_COMMON_X] = x
        param[scenario_key.PARAM_KEY_COMMON_Y] = y
        param[scenario_key.PARAM_KEY_COMMON_Z] = z
        param[scenario_key.PARAM_KEY_COMMON_RX] = rx
        param[scenario_key.PARAM_KEY_COMMON_RY] = ry
        param[scenario_key.PARAM_KEY_COMMON_RZ] = rz
        param[scenario_key.PARAM_KEY_ARM_VELOCITY] = velocity
        param[scenario_key.PARAM_KEY_ARM_MOVE_TYPE] = move_type
        param[scenario_key.PARAM_KEY_ARM_PLAN_KEY] = plan_key
        return task_relative_move.relative_move(self._action_manager, param)
