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
# from task_common.action_interface import action_manager
from task_common.action_interface import common
import math
import rospy
import tf
from geometry_msgs.msg import TransformStamped, Vector3

TOOL_MOVE_FIXED_Z = 100


def home(action_manager, arm_name, param):
    """
    ホームに移動する
    """
    result, _ = action_manager.arm_operation(arm_name, "move_home", param)
    return result


def move(action_manager, arm_name, param):
    """
    指定された座標に移動する
    """
    result, _ = action_manager.arm_operation(arm_name, scenario_key.ARM_MOTION_MOVE, param)
    return result


def move_approach_position(action_manager, arm_name, param):
    """
    アプローチ座標に移動する
    """
    appr_param = param.copy()
    # 座標計算
    position, _ = _get_approach_position(appr_param)
    appr_param[scenario_key.PARAM_KEY_COMMON_X] = position[0]
    appr_param[scenario_key.PARAM_KEY_COMMON_Y] = position[1]
    appr_param[scenario_key.PARAM_KEY_COMMON_Z] = position[2]
    # キー名変更
    if scenario_key.PARAM_KEY_ARM_PLAN_KEY in param and param[scenario_key.PARAM_KEY_ARM_PLAN_KEY] is not None:
        appr_param[scenario_key.PARAM_KEY_ARM_PLAN_KEY] = param[scenario_key.PARAM_KEY_ARM_PLAN_KEY] + "_AP"
    result, _ = action_manager.arm_operation(arm_name, scenario_key.ARM_MOTION_MOVE, appr_param)
    return result


def move_target_position(action_manager, arm_name, param):
    """
    ピック位置に移動
    """
    tar_param = param.copy()
    tar_param[scenario_key.PARAM_KEY_COMMON_X] = 0
    tar_param[scenario_key.PARAM_KEY_COMMON_Y] = 0
    tar_param[scenario_key.PARAM_KEY_COMMON_RX] = 0
    tar_param[scenario_key.PARAM_KEY_COMMON_RY] = 0
    tar_param[scenario_key.PARAM_KEY_COMMON_RZ] = 0
    # 座標計算
    if scenario_key.PARAM_KEY_ARM_DOWN_DISTANCE in param:
        tar_param[scenario_key.PARAM_KEY_COMMON_Z] = param[scenario_key.PARAM_KEY_ARM_DOWN_DISTANCE]
    else:
        tar_param[scenario_key.PARAM_KEY_COMMON_Z] = TOOL_MOVE_FIXED_Z
    # キー名変更
    if scenario_key.PARAM_KEY_ARM_PLAN_KEY in param and param[scenario_key.PARAM_KEY_ARM_PLAN_KEY] is not None:
        tar_param[scenario_key.PARAM_KEY_ARM_PLAN_KEY] = param[scenario_key.PARAM_KEY_ARM_PLAN_KEY] + "_TP"
    result, _ = action_manager.arm_operation(arm_name, scenario_key.ARM_MOTION_TOOL_MOVE, tar_param)
    return result


def move_departure_position(action_manager, arm_name, param):
    """
    離脱座標に移動する
    """
    # ツール点から退避
    dep_param = param.copy()
    # if scenario_key.PARAM_KEY_ARM_DOWN_DISTANCE in param:
    #     dep_param[scenario_key.PARAM_KEY_COMMON_Z] = param[scenario_key.PARAM_KEY_COMMON_Z] + param[scenario_key.PARAM_KEY_ARM_DOWN_DISTANCE]

    # else:
    #     dep_param[scenario_key.PARAM_KEY_COMMON_Z] = param[scenario_key.PARAM_KEY_COMMON_Z] + TOOL_MOVE_FIXED_Z
    # result, _ = action_manager.arm_operation(arm_name, scenario_key.ARM_MOTION_LINE_MOVE, dep_param)
    dep_param[scenario_key.PARAM_KEY_COMMON_X] = 0
    dep_param[scenario_key.PARAM_KEY_COMMON_Y] = 0
    dep_param[scenario_key.PARAM_KEY_COMMON_RX] = 0
    dep_param[scenario_key.PARAM_KEY_COMMON_RY] = 0
    dep_param[scenario_key.PARAM_KEY_COMMON_RZ] = 0
    # 座標計算
    if scenario_key.PARAM_KEY_ARM_DOWN_DISTANCE in param:
        dep_param[scenario_key.PARAM_KEY_COMMON_Z] = -param[scenario_key.PARAM_KEY_ARM_DOWN_DISTANCE]
    else:
        dep_param[scenario_key.PARAM_KEY_COMMON_Z] = -TOOL_MOVE_FIXED_Z
    # キー名変更
    if scenario_key.PARAM_KEY_ARM_PLAN_KEY in param and param[scenario_key.PARAM_KEY_ARM_PLAN_KEY] is not None:
        dep_param[scenario_key.PARAM_KEY_ARM_PLAN_KEY] = param[scenario_key.PARAM_KEY_ARM_PLAN_KEY] + "_DP"
    result, _ = action_manager.arm_operation(arm_name, scenario_key.ARM_MOTION_TOOL_MOVE, dep_param)
    return result


def get_current_joint(action_manager, arm_name, param):
    """
    現在のジョイント情報を取得
    """
    _, data = action_manager.arm_operation(arm_name, "get_current_joint", param)
    return data["joint"]


def joint_move(action_manager, arm_name, param):
    """
    指定された軸の位置に移動する
    """
    result, _ = action_manager.arm_operation(arm_name, scenario_key.ARM_MOTION_JOINT_MOVE, param)
    return result


def line_move(action_manager, arm_name, param):
    """
    指定された座標に直線補間で移動する
    """
    result, _ = action_manager.arm_operation(arm_name, scenario_key.ARM_MOTION_LINE_MOVE, param)
    return result


def get_object_position(action_manager, arm_name, param):
    """
    オブジェクト位置情報を取得
    """
    _, data = action_manager.arm_operation(arm_name, "get_object_position", param)
    return data["position"]


def relative_move(action_manager, arm_name, param):
    """
    相対位置移動
    """
    # 現在位置から相対的な座標に移動する
    rel_param = param.copy()
    _, data = action_manager.arm_operation(arm_name, "get_position", None)
    pos = data["position"].position
    rel_param[scenario_key.PARAM_KEY_COMMON_X] = rel_param[scenario_key.PARAM_KEY_COMMON_X] + (pos.x * 1000)
    rel_param[scenario_key.PARAM_KEY_COMMON_Y] = rel_param[scenario_key.PARAM_KEY_COMMON_Y] + (pos.y * 1000)
    rel_param[scenario_key.PARAM_KEY_COMMON_Z] = rel_param[scenario_key.PARAM_KEY_COMMON_Z] + (pos.z * 1000)
    #rel_param["orientation"] = data["position"].orientation
    e = common.quaternion_to_euler(data["position"].orientation)
    rel_param[scenario_key.PARAM_KEY_COMMON_RX] = rel_param[scenario_key.PARAM_KEY_COMMON_RX] + math.degrees(e.x)
    rel_param[scenario_key.PARAM_KEY_COMMON_RY] = rel_param[scenario_key.PARAM_KEY_COMMON_RY] + math.degrees(e.y)
    rel_param[scenario_key.PARAM_KEY_COMMON_RZ] = rel_param[scenario_key.PARAM_KEY_COMMON_RZ] + math.degrees(e.z)

    if rel_param[scenario_key.PARAM_KEY_ARM_MOVE_TYPE] == scenario_key.ARM_MOVE_TYPE_PTP:
        # ptp移動
        result, _ = action_manager.arm_operation(arm_name, scenario_key.ARM_MOTION_MOVE, rel_param)
    else:
        # line移動
        result, _ = action_manager.arm_operation(arm_name, scenario_key.ARM_MOTION_LINE_MOVE, rel_param)

    return result

def _get_approach_position(param):
    """
    ターゲット座標からアプローチ座標を算出
    """
    down_dis = TOOL_MOVE_FIXED_Z
    if scenario_key.PARAM_KEY_ARM_DOWN_DISTANCE in param:
        down_dis = param[scenario_key.PARAM_KEY_ARM_DOWN_DISTANCE]

    t = tf.Transformer(True, rospy.Duration(10.0))

    # target
    tp = TransformStamped()
    tp.header.frame_id = 'BASE_LINK'
    tp.child_frame_id = 'OBJECT'
    tp.transform.translation = Vector3(param[scenario_key.PARAM_KEY_COMMON_X], param[scenario_key.PARAM_KEY_COMMON_Y], param[scenario_key.PARAM_KEY_COMMON_Z])
    tp.transform.rotation = common.degree_to_quaternion(param[scenario_key.PARAM_KEY_COMMON_RX], param[scenario_key.PARAM_KEY_COMMON_RY], param[scenario_key.PARAM_KEY_COMMON_RZ])

    t.setTransform(tp)

    # approach
    ap = TransformStamped()
    ap.header.frame_id = 'OBJECT'
    ap.child_frame_id = 'APPROACH'
    ap.transform.translation = Vector3(0, 0, -down_dis)
    ap.transform.rotation = common.degree_to_quaternion(0, 0, 0)

    t.setTransform(ap)

    position, quaternion = t.lookupTransform('BASE_LINK', 'APPROACH', rospy.Time(0))
    return position, quaternion

    
