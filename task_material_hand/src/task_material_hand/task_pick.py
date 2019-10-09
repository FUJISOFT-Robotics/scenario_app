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
from task_common.motion_interface import arm, hand, force
from time import sleep


def pick(action_manager, param):
    # ピックタスクを実施
    arm_name = param["arm_name"]
    hand_name = param["hand_name"]
    failed_count = 0
    pick_result = False

    # 力覚センサの監視引数を確認
    use_force = False
    if param[scenario_key.PARAM_KEY_FORCE_AXIS] != "" and \
       param[scenario_key.PARAM_KEY_FORCE_NEWTON] != 0:
        use_force = True
    elif param[scenario_key.PARAM_KEY_FORCE_AXIS] != "":
        print ("newton引数に力量を指定してください")
        return False
    elif param[scenario_key.PARAM_KEY_FORCE_NEWTON] != 0:
        print ("axis引数に検査する方向を指定してください")
        return False
    else:
        pass

    if param[scenario_key.PARAM_KEY_FORCE_PICK_CHECK] is True:
        if param[scenario_key.PARAM_KEY_FORCE_FAILED_UPPER_LIMIT] > 0:
            failed_upper_limit = param[scenario_key.PARAM_KEY_FORCE_FAILED_UPPER_LIMIT]
        else:
            print ("失敗の上限が不正な値になっています。1以上の整数で指定してください")
            return False
    else:
        failed_upper_limit = 1

    while pick_result is False and failed_count < failed_upper_limit:

        # アプローチ点に移動
        result = arm.move_approach_position(action_manager, arm_name, param)
        print ('アプローチ移動:{0}'.format(result))
        if result is False:
            return result

        # ハンドを開く
        result = hand.hand_open(action_manager, hand_name)
        # ピック前力量を設定
        if param[scenario_key.PARAM_KEY_FORCE_PICK_CHECK] is True:
            sleep(1)
            result = force.set_pick_check(action_manager, arm_name)

        # 力覚センサの監視
        if use_force is True:
            result = force.set_stop_force(action_manager, arm_name, param)

        # ピック位置に移動
        result = arm.move_target_position(action_manager, arm_name, param)
        print ('ピック位置移動:{0}'.format(result))

        if use_force:
            is_stopped = force.is_stopped(action_manager, arm_name)

            if not result and not is_stopped:
                return result
        else:
            if result is False:
                return result

        # 力覚センサの監視をキャンセル
        if use_force is True:
            result = force.clear_stop_force(action_manager, arm_name)

        # ハンドを閉じる
        result = hand.hand_close(action_manager, hand_name)
        # TODO 連続でmove命令を出すとプラン時にstart位置の異常で動かない時があるのでsleepを実施
        sleep(0.1)

        # ピック位置から退避
        result = arm.move_departure_position(action_manager, arm_name, param)
        print ('退避位置移動:{0}'.format(result))
        if result is False:
            return result

        # 重さを計測する
        # ピック前力量を設定
        if param[scenario_key.PARAM_KEY_FORCE_PICK_CHECK] is True:
            sleep(1)
            result, pick_result = force.get_pick_result(action_manager, arm_name)
            if result is True and pick_result is False:
                failed_count += 1
                # 失敗回数の上限になった場合、Falseを返す
                if failed_count == failed_upper_limit:
                    result = False
        else:
            # チェックが不要の場合はTrueを返す
            pick_result = True

    return result
