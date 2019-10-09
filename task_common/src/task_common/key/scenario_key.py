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

"""
シナリオファイルの定義
"""
# 各JSONファイルのキー名を定義
# シナリオファイルのキー
SCENARIO_KEY_TASK = "task"
SCENARIO_KEY_ORDER = "order"
SCENARIO_KEY_MOTION = "motion"
SCENARIO_KEY_MACHINE_NAME = "machine_name"
SCENARIO_KEY_PARAMETER = "parameter"
# 共通パラメータのキー
PARAM_KEY_COMMON_X = "x"
PARAM_KEY_COMMON_Y = "y"
PARAM_KEY_COMMON_Z = "z"
PARAM_KEY_COMMON_RX = "rx"
PARAM_KEY_COMMON_RY = "ry"
PARAM_KEY_COMMON_RZ = "rz"
PARAM_KEY_COMMON_POSTURE = "posture"
PARAM_KEY_COMMON_TOOL_NAME = "tool_name"
PARAM_KEY_COMMON_OBJECT_NAME = "object_name"
PARAM_KEY_COMMON_TIMEOUT = "timeout"
# アームクラスのキー
PARAM_KEY_ARM_ARM_NAME = "arm_name"
PARAM_KEY_ARM_VELOCITY = "velocity"
PARAM_KEY_ARM_JOINT_LIST = "joint_list"
PARAM_KEY_ARM_PRECISION_VELOCITY = "precision_velocity"
PARAM_KEY_ARM_DOWN_DISTANCE = "down_distance"
PARAM_KEY_ARM_PLAN_KEY = "plan_key"
PARAM_KEY_ARM_MOVE_TYPE = "move_type"
# ツールクラスのキー
PARAM_KEY_TOOL_HAND_NAME = "hand_name"
PARAM_KEY_TOOL_WAIT = "wait"
# AGVクラスのキー
TASK_KEY_AGV_TIME_OUT = "time_out"
# 力覚クラスのキー
PARAM_KEY_FORCE_AXIS = "axis"
PARAM_KEY_FORCE_NEWTON = "newton"
PARAM_KEY_FORCE_PICK_CHECK = "pick_check"
PARAM_KEY_FORCE_FAILED_UPPER_LIMIT = "failed_upper_limit"
# IOクラスのキー
PARAM_KEY_IO_ADDRESS = "address"
PARAM_KEY_IO_SET_VALUE = "set_value"
PARAM_KEY_IO_STATE = "io_state"
PARAM_KEY_IO_NAME = "io_name"
PARAM_KEY_IO_WAIT = "wait"
PARAM_KEY_IO_TIME_OUT = "time_out"
# ストラクチャーファイルのキー
STRUCTURE_KEY_STRUCTURE = "structure"
STRUCTURE_KEY_TYPE = "type"
STRUCTURE_KEY_MACHINE_NAME = "machine_name"
STRUCTURE_KEY_GROUP_NAME = "group_name"
STRUCTURE_KEY_PARAMETER = "parameter"
STRUCTURE_KEY_PARAMETER_BASE = "base"
STRUCTURE_KEY_PARAMETER_TARGET = "target"

# 各クラスの固定値の値を定義
# アームクラス
ARM_TYPE = "arm"
ARM_MOTION_MOVE = "move"
ARM_MOTION_TOOL_MOVE = "tool_move"
ARM_MOTION_MOVE_HOME = "move_home"
ARM_MOTION_LINE_MOVE = "line_move"
ARM_MOTION_JOINT_MOVE = "joint_move"
ARM_MOVE_TYPE_PTP = 0
# ハンドクラス
TOOL_TYPE = "hand"
TOOL_MOTION_HAND_OPEN = "hand_open"
TOOL_MOTION_HAND_CLOSE = "hand_close"
# AGVクラス
AGV_TYPE = "agv"
AGV_MOTION_READY = "ready"
AGV_MOTION_COMPLETE = "complete"
# 力覚
FORCE_TYPE = "force"
FORCE_MOTION_SET_STOP_FORCE = "set_stop_force"
FORCE_MOTION_CLEAR_STOP_FORCE = "clear_stop_force"
FORCE_MOTION_SET_PICK_CHECK = "set_pick_check"
FORCE_MOTION_GET_PICK_RESULT = "get_pick_result"
FORCE_MOTION_IS_STOPPED = "is_stopped"
# IO
IO_TYPE = "io"
IO_MOTION_GET_IO_STATE = "get_io_state"
IO_MOTION_GET_IO_STATE_ALL = "get_io_state_all"
IO_MOTION_SET_IO = "set_io"
IO_MOTION_WAIT_IO = "wait_io"
