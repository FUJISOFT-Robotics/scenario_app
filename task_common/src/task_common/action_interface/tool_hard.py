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

from fsrobo_r_driver.robot_tool_interface import RobotToolInterface
from task_common.key import scenario_key
import global_data
import common


class ToolHard:
    """
    アームロボット
    """
    _STRING_TRUE = "True"
    ROSPARAM_PATH = "/scenario/Pause"

    def __init__(self, tool_name):
        """
        初期処理
        """
        # グループインスタンスを生成
        if isinstance(tool_name, unicode):
            str_tool_name = tool_name.encode()
        else:
            str_tool_name = tool_name
        self.robot_tool_interface = RobotToolInterface(str_tool_name)

    def task_run(self, func, param):
        """
        タスクを実行する
        :param func:
        :param param:
        :return:
        """
        # 他クラスと戻り値をあわせる為に空の変数を定義
        data = {}
        if func == scenario_key.TOOL_MOTION_HAND_OPEN:
            common.scenario_pause(self.ROSPARAM_PATH)
            result, data = self.tool_hand_open(param[scenario_key.PARAM_KEY_TOOL_WAIT])

        elif func == scenario_key.TOOL_MOTION_HAND_CLOSE:
            common.scenario_pause(self.ROSPARAM_PATH)
            result, data = self.tool_hand_close(param[scenario_key.PARAM_KEY_TOOL_WAIT])

        else:
            result = False
            data[global_data.RETURN_DATA_KEY_ERROR_MESSAGE] = "ツール操作に" + func + "操作は存在しません。\n"

        return result, data

    def tool_hand_open(self, wait="True"):
        """
        ツールのハンドを開く
        :param wait:
        :return result:結果の成否
        :return data:エラー発生時のみエラーメッセージを返す
        """
        result = False
        data = {}
        # ハンド操作を実施
        if wait == self._STRING_TRUE:
            result = self.robot_tool_interface.open(True)
        else:
            result = self.robot_tool_interface.open(False)
        if result is False:
            # ハンドの操作に失敗した場合
            data[global_data.RETURN_DATA_KEY_ERROR_MESSAGE] = "ハンドを開く操作に失敗しました。\n"
        return result, data

    def tool_hand_close(self, wait="True"):
        """
        ツールのハンドを閉じる
        :param wait:
        :return result:結果の成否
        :return data:エラー発生時のみエラーメッセージを返す
        """
        result = False
        data = {}
        if wait == self._STRING_TRUE:
            result = self.robot_tool_interface.close(True)
        else:
            result = self.robot_tool_interface.close(False)
        if result is False:
            # ハンドの操作に失敗した場合
            data[global_data.RETURN_DATA_KEY_ERROR_MESSAGE] = "ハンドを閉じる操作に失敗しました。\n"
        return result, data
