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

from fsrobo_r_driver.ft_sensor_interface import FTSensorInterface
from task_common.key import scenario_key
from . import global_data


class FTSensorHard:
    """
    力覚センサ
    """
    _STRING_TRUE = "True"

    def __init__(self, name):
        """
        初期処理
        """
        # グループインスタンスを生成
        # if isinstance(name, unicode):
        #     str_name = name.encode()
        # else:
        #     str_name = name
        self.ft_sensor = FTSensorInterface(name)

    def task_run(self, func, param):
        """
        タスクを実行する
        :param func:
        :param param:
        :return:
        """
        # 他クラスと戻り値をあわせる為に空の変数を定義
        data = {}
        if func == scenario_key.FORCE_MOTION_SET_STOP_FORCE:
            result, data = self.tf_sensor_set_stop_force(param[scenario_key.PARAM_KEY_FORCE_AXIS], param[scenario_key.PARAM_KEY_FORCE_NEWTON])

        elif func == scenario_key.FORCE_MOTION_CLEAR_STOP_FORCE:
            result, data = self.tf_sensor_clear_stop_force()

        elif func == scenario_key.FORCE_MOTION_SET_PICK_CHECK:
            result, data = self.tf_sensor_set_pick_check()

        elif func == scenario_key.FORCE_MOTION_GET_PICK_RESULT:
            result, data = self.tf_sensor_get_pick_result()

        elif func == scenario_key.FORCE_MOTION_IS_STOPPED:
            result, data = self.ft_sensor_is_stopped()

        else:
            result = False
            data[global_data.RETURN_DATA_KEY_ERROR_MESSAGE] = "ツール操作に" + func + "操作は存在しません。\n"

        return result, data

    def tf_sensor_set_stop_force(self, axis, newton):
        """
        力覚センサの監視を設定
        :param axis: 監視する力の方向
        :param newton: 停止させる時の力量
        :return result:結果の成否
        :return data:エラー発生時のみエラーメッセージを返す
        """
        result = True
        data = {}
        # 力覚センサの監視を開始
        self.ft_sensor.set_stop_force(axis, newton)
        return result, data

    def tf_sensor_clear_stop_force(self):
        """
        力覚センサの監視を停止
        :return result:結果の成否
        :return data:エラー発生時のみエラーメッセージを返す
        """
        result = True
        data = {}
        # 力覚センサの監視を停止
        self.ft_sensor.clear_stop_force()
        return result, data

    def tf_sensor_set_pick_check(self):
        result = True
        data = {}
        self._last_force = self.ft_sensor.get_current_force()
        return result, data

    def tf_sensor_get_pick_result(self):
        result = True
        data = {}
        current_force = self.ft_sensor.get_current_force()
        diff_x = current_force.wrench.force.x - self._last_force.wrench.force.x
        diff_y = current_force.wrench.force.y - self._last_force.wrench.force.y
        diff_z = current_force.wrench.force.z - self._last_force.wrench.force.z
        print ("diff_x:%f" % diff_x)
        print ("diff_y:%f" % diff_y)
        print ("diff_z:%f" % diff_z)
        if diff_x > 0.5 or diff_y > 0.5 or diff_z > 0.5:
            data["pick_result"] = True
        else:
            data["pick_result"] = False
        return result, data

    def ft_sensor_is_stopped(self):
        result = True
        data = {}
        is_stopped = self.ft_sensor.is_stopped()
        data["stop_result"] = is_stopped
        return result, data
