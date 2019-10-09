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


def set_stop_force(action_manager, arm_name, param):
    """
    力覚センサの監視
    """
    stop_force_param = {}
    stop_force_param[scenario_key.PARAM_KEY_FORCE_AXIS] = param[scenario_key.PARAM_KEY_FORCE_AXIS]
    stop_force_param[scenario_key.PARAM_KEY_FORCE_NEWTON] = param[scenario_key.PARAM_KEY_FORCE_NEWTON]
    result, _ = action_manager.ft_sensor_operation(arm_name, scenario_key.FORCE_MOTION_SET_STOP_FORCE, stop_force_param)
    return result


def clear_stop_force(action_manager, arm_name):
    """
    力覚センサの監視をキャンセル
    """
    result, _ = action_manager.ft_sensor_operation(arm_name, scenario_key.FORCE_MOTION_CLEAR_STOP_FORCE, {})
    return result


def set_pick_check(action_manager, arm_name):
    """
    ピック確認前の重さを登録
    """
    result, _ = action_manager.ft_sensor_operation(arm_name, scenario_key.FORCE_MOTION_SET_PICK_CHECK, {})
    return result


def get_pick_result(action_manager, arm_name):
    """
    ピック成否を取得
    """
    result, data = action_manager.ft_sensor_operation(arm_name, scenario_key.FORCE_MOTION_GET_PICK_RESULT, {})
    print ("戻り値")
    print (data)
    return result, data['pick_result']


def is_stopped(action_manager, arm_name):
    """
    力覚センサによるアーム動作停止が行われたかチェック
    """
    _, data = action_manager.ft_sensor_operation(arm_name, scenario_key.FORCE_MOTION_IS_STOPPED, {})

    return data['stop_result']
