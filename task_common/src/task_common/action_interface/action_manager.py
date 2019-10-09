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

import rospy

from task_common.key import scenario_key
from . import arm_robot
import tool_hard
from . import ft_sensor_hard
from . import io_hard


class ActionManager:
    """
    タスク管理クラス
    """

    def __init__(self, structure_info):
        """
        初期処理
        :param file_path:
        """
        # ros初期化
        rospy.init_node("test_scenario_node", anonymous=True)

        # 構成情報読み込み
        self._structure = structure_info

        # 構成ファイルからハードウェアクラスをインスタンス化
        self._arm_list = {}
        self._hand_list = {}
        self._ft_sensor_list = {}
        self._io_list = {}
        for machine in self._structure[scenario_key.STRUCTURE_KEY_STRUCTURE]:
            print ("インスタンス作成")
            print (machine)
            if machine[scenario_key.STRUCTURE_KEY_TYPE] == scenario_key.ARM_TYPE:
                self._arm_list[machine[scenario_key.STRUCTURE_KEY_MACHINE_NAME]] = \
                    arm_robot.ArmRobot(group=machine[scenario_key.STRUCTURE_KEY_GROUP_NAME],
                                       robot_name=machine[scenario_key.STRUCTURE_KEY_MACHINE_NAME],
                                       base_origin=machine[scenario_key.STRUCTURE_KEY_PARAMETER][scenario_key.STRUCTURE_KEY_PARAMETER_BASE],
                                       target_frame=machine[scenario_key.STRUCTURE_KEY_PARAMETER][scenario_key.STRUCTURE_KEY_PARAMETER_TARGET])

            elif machine[scenario_key.STRUCTURE_KEY_TYPE] == scenario_key.TOOL_TYPE:
                self._hand_list[machine[scenario_key.STRUCTURE_KEY_MACHINE_NAME]] = tool_hard.ToolHard(
                    tool_name=machine[scenario_key.STRUCTURE_KEY_MACHINE_NAME])

            elif machine[scenario_key.STRUCTURE_KEY_TYPE] == scenario_key.FORCE_TYPE:
                self._ft_sensor_list[machine[scenario_key.STRUCTURE_KEY_MACHINE_NAME]] = ft_sensor_hard.FTSensorHard(
                    name=machine[scenario_key.STRUCTURE_KEY_MACHINE_NAME])

            elif machine[scenario_key.STRUCTURE_KEY_TYPE] == scenario_key.IO_TYPE:
                self._io_list[machine[scenario_key.STRUCTURE_KEY_MACHINE_NAME]] = io_hard.IOHard(
                    io_name=machine[scenario_key.STRUCTURE_KEY_MACHINE_NAME])

        print ("インスタンス作成完了")

    def arm_operation(self, name, func, param):
        """
        アームロボットの操作
        """
        arm = self._arm_list[name]
        result, data = arm.task_run(func, param)
        return result, data

    def hand_operation(self, name, func, param):
        """
        ハンドの操作
        """
        hand = self._hand_list[name]
        result, data = hand.task_run(func, param)
        return result, data

    def ft_sensor_operation(self, name, func, param):
        """
        力覚操作
        """
        ft_sensor = self._ft_sensor_list[name]
        result, data = ft_sensor.task_run(func, param)
        return result, data

    def io_operation(self, name, func, param):
        """
        IO操作
        """
        io = self._io_list[name]
        result, data = io.task_run(func, param)
        return result, data
