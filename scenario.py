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

import os
import json
from task_material_hand.task_manager import TaskManager
from task_common.common import position
from task_common.common import force_option

# 構成ファイル読み込み
abs_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(abs_dir)
file1 = open("structure.json", 'r')
structure_info = json.load(file1, "utf-8")
file1.close()

# タスクマネージャ初期化
tm = TaskManager(structure_info)
print "ホーム移動"
# ホームに移動
arm_nm = ""
hand_nm = "generic_hand"
tool_nm = "manipulator"
velocity = 0.5

tm.task_execute_home(arm_nm)

# 移動
pos = position.Position(-446.19, -2.92, 327.50, 0.0, 0.0, 180.0)
tm.task_execute_move(arm_nm, pos, tool_nm, velocity)

# ピック
pick_pos = position.Position(-446.19, -2.92, 327.50, 0.0, 0.0, 180.0)
#foption = force_option.ForceOption(axis="z", newton=-2, pick_check=True, failed_upper_limit=3)
pick_result = tm.task_execute_pick( \
                     arm_name = arm_nm, \
                     hand_name = hand_nm, \
                     pos = pick_pos, \
                     tool_name = tool_nm, \
                     velocity=velocity, \
                     precision_velocity=velocity, \
                     down_distance=100, \
                    )

# プレイス
place_pos = position.Position(-346.19, -2.92, 327.50, 0.0, 0.0, 180.0)
tm.task_execute_place( \
                     arm_name = arm_nm, \
                     hand_name = hand_nm, \
                     pos = place_pos, \
                     tool_name = tool_nm, \
                     velocity=velocity, \
                     precision_velocity=velocity, \
                     down_distance=100, \
#                     force_option=foption \
                    )
