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

class Position:
    """
    座標情報を保持するクラス
    """
    def __init__(self, x, y, z, rz, ry, rx, posture=-1):
        self.x = x
        self.y = y
        self.z = z
        self.rz = rz
        self.ry = ry
        self.rx = rx
        self.posture = posture

    def replace(self, x, y, z, rz, ry, rx, posture=-1):
        self.x = x
        self.y = y
        self.z = z
        self.rz = rz
        self.ry = ry
        self.rx = rx
        self.posture = posture

    def offset(self, x=0, y=0, z=0, rz=0, ry=0, rx=0, posture=-1):
        insert_x = self.x + x
        insert_y = self.y + y
        insert_z = self.z + z
        insert_rz = self.rz + rz
        insert_ry = self.ry + ry
        insert_rx = self.rx + rx
        if posture != -1:
            insert_posture = posture
        else:
            insert_posture = self.posture
        return Position(insert_x, insert_y, insert_z, insert_rz, insert_ry, insert_rx, insert_posture)
