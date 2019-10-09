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

import math
import tf
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion


def calc_coordinate_correction(base_point, target_point, difference_value):
    """

    :param base_point: 中心座標（カメラ画像の中心位置）。{x,y}
    :param target_point: 目標座標（カメラ画像に写っている対象物の座標）{x,y}
    :param difference_value: カメラ画像の差異情報{x,y,θ}
    :return:
    """

    calc_point = {"x": 0, "y": 0, "angle": 0}

    x = target_point["x"] - base_point["x"]
    y = target_point["y"] - base_point["y"]
    angle = math.radians(difference_value["angle"])

    calc_point["x"] = x * math.cos(angle) - y * math.sin(angle) + difference_value["x"] + base_point["x"]
    calc_point["y"] = x * math.sin(angle) + y * math.cos(angle) + difference_value["y"] + base_point["y"]

    return calc_point


def calc_quaternion_plus_angle(quaternion, x=0, y=0, z=0):
    """

    :param quaternion:
    :param x:
    :param y:
    :param z:
    :return:
    """

    euler_1 = quaternion_to_euler(quaternion)

    rx = math.radians(x)
    ry = math.radians(y)
    rz = math.radians(z)

    euler_2 = Vector3(x=(euler_1.x + rx), y=(euler_1.y + ry), z=(euler_1.z + rz))
    quaternion_2 = euler_to_quaternion(euler_2)

    return euler_2, quaternion_2


def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w), 'rzyx')
    return Vector3(x=e[2], y=e[1], z=e[0])


def euler_to_quaternion(euler):
    """Convert Euler Angles to Quaternion

    euler: geometry_msgs/Vector3
    quaternion: geometry_msgs/Quaternion
    """
    q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def degree_to_quaternion(x, y, z):
    """Convert Euler Angles to Quaternion

    euler: geometry_msgs/Vector3
    quaternion: geometry_msgs/Quaternion
    """
    #euler = Vector3(x * 3.14 / 180, y * 3.14 / 180, z * 3.14 / 180)
    euler = Vector3(math.radians(x), math.radians(y), math.radians(z))
    q = tf.transformations.quaternion_from_euler(euler.z, euler.y, euler.x, 'rzyx')
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
