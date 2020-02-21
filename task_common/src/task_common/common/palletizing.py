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

import copy
import math


class Palletizing:

    class Pattern:
        ZIGZAG = 1
        SNAKE = 2

    """
    座標情報を保持するクラス
    """
    def __init__(self, pos1, pos2, pos3,
                 size_x, size_y, z=0, size_z=1, pattern=Pattern.ZIGZAG):
        # インスタンス変数定義
        self._max_size_x = size_x
        self._max_size_y = size_y
        self._max_size_z = size_z
        self._next_x = 0
        self._next_y = 0
        self._next_z = 0

        relative_x = self._get_relative_position(pos1, pos2)
        relative_y = self._get_relative_position(pos1, pos3)
        if size_x > 1:
            interval_x = map(lambda x: x / (size_x - 1), relative_x)
        else:
            interval_x = 0.0
        if size_y > 1:
            interval_y = map(lambda x: x / (size_y - 1), relative_y)
        else:
            interval_y = 0.0
        print("interval x:{} y:{}".format(interval_x, interval_y))
        if pattern == self.Pattern.ZIGZAG:
            self._palletizing_pos = self._create_zigzag_list(pos1, interval_x, interval_y, size_x, size_y)
        else:
            self._palletizing_pos = self._create_snake_list(pos1, interval_x, interval_y, size_x, size_y)

        if size_z > 1:
            pos4 = self._get_z_position(pos1, pos2, pos3, relative_x, relative_y, z)
            relative_z = self._get_relative_position(pos1, pos4)
            interval_x = map(lambda x: x / (size_z - 1), relative_z)
            self._palletizing_pos_3d = self._create_3d_palletizing_list(interval_x, size_z)
        elif size_z == 1:
            self._palletizing_pos_3d = self._create_3d_palletizing_list(0, size_z)
        else:
            raise Exception("引数：size_zに渡された値が不正です。\n1以上の値を指定してください")

    def get_pos(self, index_x, index_y):
        return self._palletizing_pos[index_y][index_x]

    def get_pos_3d(self, index_x, index_y, index_z):
        return self._palletizing_pos_3d[index_z][index_y][index_x]

    def get_next_pos(self):
        position = self._palletizing_pos_3d[self._next_z][self._next_y][self._next_x]

        # 次のインデックスに進める
        self._next_x += 1
        if self._max_size_x <= self._next_x:
            self._next_x = 0
            self._next_y += 1
            if self._max_size_y <= self._next_y:
                self._next_y = 0
                self._next_z += 1
                if self._max_size_z <= self._next_z:
                    # 最初のインデックスに戻る
                    self._next_z = 0
        return position

    def _get_z_position(self, pos1, pos2, pos3, relative1, relative2, height):
        position_y = copy.deepcopy(pos1)
        position_y.x = position_y.x + relative1[0]
        position_y.y = position_y.y + relative1[1]
        position_y.z = position_y.z + relative1[2]

        pos4 = copy.deepcopy(position_y)
        pos4.x = pos4.x + relative2[0]
        pos4.y = pos4.y + relative2[1]
        pos4.z = pos4.z + relative2[2]

        pos_list = [pos2, pos3, pos4]
        bottom_pos = pos1
        high_pos = pos1
        for pos in pos_list:
            if bottom_pos.z > pos.z:
                bottom_pos = pos
            elif high_pos.z < pos.z:
                high_pos = pos

        relative3 = self._get_relative_position(bottom_pos, high_pos)
        base_z = (relative3[0] ** 2 + relative3[1] ** 2) ** 0.5
        if base_z != 0:
            lean_z = math.atan(relative3[2] / base_z)
        else:
            lean_z = math.atan(0)

        base_x = (relative1[0] ** 2 + relative1[1] ** 2) ** 0.5
        if base_x != 0:
            lean_x = math.atan(relative1[2] / base_x)
        else:
            lean_x = math.atan(0)

        base_y = (relative2[0] ** 2 + relative2[1] ** 2) ** 0.5
        if base_y != 0:
            lean_y = math.atan(relative2[2] / base_y)
        else:
            lean_y = math.atan(0)

        offset_z = height * math.cos(lean_z)
        offset_x = offset_z * math.tan(lean_x)
        offset_y = offset_z * math.tan(lean_y)
        print("lean x:{} y:{} z:{}".format(math.degrees(lean_x), math.degrees(lean_y), math.degrees(lean_z)))
        print("offset x:{} y:{} z:{}".format(offset_x, offset_y, offset_z))

        result = copy.deepcopy(pos1)
        result.x = result.x + offset_x
        result.y = result.y + offset_y
        result.z = result.z + offset_z
        print("postion5 x:{} y:{} z:{} rz:{} ry:{} rx:{}".format(result.x, result.y, result.z, result.rz, result.ry, result.rx))
        return result

    def _get_relative_position(self, pos1, pos2):
        relative_x = pos2.x - pos1.x
        relative_y = pos2.y - pos1.y
        relative_z = pos2.z - pos1.z
        result = [relative_x, relative_y, relative_z]
        print("relative:{}".format(result))
        return result

    def _create_zigzag_list(self, pos1, interval_x, interval_y, size_x, size_y):
        result = []
        for index_y in range(size_y):
            position_y = copy.deepcopy(pos1)
            print("position x:{}, y:{}".format(position_y.x, position_y.y))
            if not index_y == 0:
                offset_y = map(lambda x: x * index_y, interval_y)
                print("offset_y:{}".format(offset_y))
                position_y.x = position_y.x + offset_y[0]
                position_y.y = position_y.y + offset_y[1]
                position_y.z = position_y.z + offset_y[2]
            result_x = []
            for index_x in range(size_x):
                position_x = copy.deepcopy(position_y)
                if not index_x == 0:
                    offset_x = map(lambda x: x * index_x, interval_x)
                    print("offset_x:{}".format(offset_x))
                    position_x.x = position_x.x + offset_x[0]
                    position_x.y = position_x.y + offset_x[1]
                    position_x.z = position_x.z + offset_x[2]
                result_x.append(position_x)
            result.append(result_x)
        return result

    def _create_snake_list(self, pos1, interval_x, interval_y, size_x, size_y):
        result = []
        for index_y in range(size_y):
            position_y = copy.deepcopy(pos1)
            print("position x:{}, y:{}".format(position_y.x, position_y.y))
            if not index_y == 0:
                offset_y = map(lambda x: x * index_y, interval_y)
                print("offset_y:{}".format(offset_y))
                position_y.x = position_y.x + offset_y[0]
                position_y.y = position_y.y + offset_y[1]
                position_y.z = position_y.z + offset_y[2]
            result_x = []
            range_x = range(size_x)
            if index_y % 2 == 1:
                range_x = list(reversed(range_x))
            for index_x in range_x:
                position_x = copy.deepcopy(position_y)
                if not index_x == 0:
                    offset_x = map(lambda x: x * index_x, interval_x)
                    print("offset_x:{}".format(offset_x))
                    position_x.x = position_x.x + offset_x[0]
                    position_x.y = position_x.y + offset_x[1]
                    position_x.z = position_x.z + offset_x[2]
                result_x.append(position_x)
            result.append(result_x)
        return result

    def _create_3d_palletizing_list(self, interval_z, size_z):
        result = []
        for index_z in range(size_z):
            palletizing_list = copy.deepcopy(self._palletizing_pos)
            if not index_z == 0:
                offset = map(lambda x: x * index_z, interval_z)
                for palletizing_list_x in palletizing_list:
                    for position in palletizing_list_x:
                        print("offset_x:{}".format(offset))
                        position.x = position.x + offset[0]
                        position.y = position.y + offset[1]
                        position.z = position.z + offset[2]
            result.append(palletizing_list)
        return result
