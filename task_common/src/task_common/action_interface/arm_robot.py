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
import tf
import moveit_commander
import common
from task_common.common import position
import global_data
from geometry_msgs.msg import Pose

from fsrobo_r_driver.geometry_util import GeometryUtil
from fsrobo_r_driver.robot_interface import RobotInterface
from fsrobo_r_driver.robot_program_interface import RobotProgramInterface
import error_func as ef
import copy
import math
from task_common.common.add_exception import PlanningException, MoveException
import time


class ArmRobot:
    """
    アームロボット
    """
    global_param = {}
    plan_cache = {}
    JOINT_TOLERANCE = 0.01
    ROSPARAM_PATH = "/scenario/pause"

    def __init__(self, group, base_origin, target_frame, robot_name=None):
        """
        初期処理
        :param group:
        :param robot_name:
        :param base_origin:
        """

        # ロボットインスタンスを生成
        self.robot = moveit_commander.RobotCommander()

        # グループインスタンスを生成
        if isinstance(group, unicode):
            str_group = group.encode()
        else:
            str_group = group
        print (str_group)
        self._group_list = {}
        group_names = self.robot.get_group_names()
        # 引数のグループ名が間違っていないかを確認
        if str_group in group_names:
            # デフォルトのグループ名を登録
            self._default_group = str_group
        else:
            ef.output_error("アームロボットに指定されたgroup名が間違っています")
        # MoveGroupを生成
        for name in group_names:
            self._group_list[name] = moveit_commander.MoveGroupCommander(name)
            self._group_list[name].set_goal_position_tolerance(0.0005)
            self._group_list[name].set_goal_orientation_tolerance(0.0005)
        self.group1 = self._group_list[self._default_group]

        self.robot_interface = RobotInterface(robot_name)
        self.base_origin = base_origin
        self._gu = GeometryUtil()
        self.target_frame = target_frame

        # init TF Likstner
        self._listener = tf.TransformListener()

        self._robot_name = robot_name
        self._default_reference_frame = self.group1.get_pose_reference_frame()

    def task_run(self, func, param):
        """
        タスクを実行する
        :param func:
        :param param:
        :return:
        """
        data = {}
        if func == "move":
            common.scenario_pause(self.ROSPARAM_PATH)
            result, data = self.move(param)
        elif func == "tool_move":
            common.scenario_pause(self.ROSPARAM_PATH)
            result, data = self.tool_move(param)
        elif func == "move_home":
            common.scenario_pause(self.ROSPARAM_PATH)
            result = self.move_home()
        elif func == "get_position":
            result, data = self.get_position()
        elif func == "get_current_joint":
            result, data = self.get_current_joint()
        elif func == "line_move":
            common.scenario_pause(self.ROSPARAM_PATH)
            result, data = self.line_move(param)
        elif func == "joint_move":
            common.scenario_pause(self.ROSPARAM_PATH)
            result, data = self.joint_move(param)
        elif func == "get_object_position":
            result, data = self.get_object_position(param)
        else:
            result = False
            data[global_data.RETURN_DATA_KEY_ERROR_MESSAGE] = "アーム操作に" + func + "操作は存在しません。\n"

        return result, data

    def move_home(self):
        home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.group1.set_joint_value_target(home)
        result = self.group1.go(wait=True)
        if result is False:
            raise MoveException("Robot Move Error")

        return result

    def move(self, param):

        return_param = {}

        # パラメータ判定
        if "x" in param and "y" in param and "z" in param:
            pass
        else:
            # パラメータ異常
            return_param[global_data.RETURN_DATA_KEY_ERROR_MESSAGE] = "アームのmove命令に失敗しました。\n"
            return False, return_param

        # ミリをメートルに変換
        pos_x = param[scenario_key.PARAM_KEY_COMMON_X] / 1000.0
        pos_y = param[scenario_key.PARAM_KEY_COMMON_Y] / 1000.0
        pos_z = param[scenario_key.PARAM_KEY_COMMON_Z] / 1000.0

        if "rx" in param and "ry" in param and "rz" in param:
            # オイラーをオリエンテーションに変換
            q = common.degree_to_quaternion(x=param[scenario_key.PARAM_KEY_COMMON_RX],
                                            y=param[scenario_key.PARAM_KEY_COMMON_RY],
                                            z=param[scenario_key.PARAM_KEY_COMMON_RZ])

        elif "orientation" in param:
            q = param["orientation"]
        else:
            # パラメータ異常
            return_param[global_data.RETURN_DATA_KEY_ERROR_MESSAGE] = "アームのmove命令に失敗しました。\n"
            return False, return_param

        if "tool_name" in param:
            self._set_move_group(param["tool_name"])
        else:
            self._set_move_group(self._default_group)

        # 姿勢設定
        if "posture" in param and param["posture"] != -1:
            self.robot_interface.set_posture(param["posture"])
            print("get posture %d" % self.robot_interface.get_posture())

        # 速度指定
        if scenario_key.PARAM_KEY_ARM_VELOCITY in param:
            self.group1.set_max_velocity_scaling_factor(param[scenario_key.PARAM_KEY_ARM_VELOCITY])
        else:
            self.group1.set_max_velocity_scaling_factor(0.5)

        pos_1 = [pos_x,  pos_y, pos_z]
        pos_2, ori = self._gu.transform_pose(self.target_frame, self.base_origin, pos_1, [q.x, q.y, q.z, q.w])
        print (pos_2)
        print (ori)
        pos_2.extend(ori)
        print (pos_2)
        self.group1.set_pose_reference_frame(self._default_reference_frame)
        self.group1.set_pose_target(pos_2)
        # joint_list = self.robot_interface.get_position_ik(pos_2, ori)
        # print ("joint_list:")
        # print (joint_list)
        # self.group1.set_joint_value_target(joint_list)
        self._plan_move(param[scenario_key.PARAM_KEY_ARM_PLAN_KEY])

        return True, return_param

    def joint_move(self, param):
        # 速度指定
        if scenario_key.PARAM_KEY_ARM_VELOCITY in param:
            self.group1.set_max_velocity_scaling_factor(param[scenario_key.PARAM_KEY_ARM_VELOCITY])
        else:
            self.group1.set_max_velocity_scaling_factor(0.5)
        joint_list = param[scenario_key.PARAM_KEY_ARM_JOINT_LIST]
        print("joint_list:{}".format(joint_list))
        rad = map(lambda x: math.radians(x), joint_list)
        print("rad:{}".format(rad))
        self.group1.set_joint_value_target(rad)
        self._plan_move(param[scenario_key.PARAM_KEY_ARM_PLAN_KEY])
        return_param = {}
        return True, return_param

    def execute_robot_program_interface(self, param):

        if "robot_name" in param:
            rpi = RobotProgramInterface(robot_name=param["robot_name"])
        else:
            rpi = RobotProgramInterface()

        # プログラムの実行
        if "param" in param:
            rpi.execute(program=param["program"], param=param["param"])
        else:
            rpi.execute(program=param["program"])

        return_param = {}
        return True, return_param

    def tool_move(self, param):

        return_param = {}
        result = False

        if "x" in param and "y" in param and "z" in param and "rx" in param and "ry" in param and "rz" in param:
            # ツールオフセットを設定
            if "tool_name" in param:
                self._set_move_group(param["tool_name"])
            else:
                self._set_move_group(self._default_group)
            # moveitでのツール移動
            self.group1.set_pose_reference_frame(self.group1.get_end_effector_link())
            wpose = Pose()
            wpose.position.x = param["x"] / 1000.0
            wpose.position.y = param["y"] / 1000.0
            wpose.position.z = param["z"] / 1000.0
            wpose.orientation.x = 0
            wpose.orientation.y = 0
            wpose.orientation.z = 0
            wpose.orientation.w = 1
            print ("tool_move_wpose:")
            print (wpose)
            # オイラーをオリエンテーションに変換
            # q = common.degree_to_quaternion(x=param[scenario_key.PARAM_KEY_COMMON_RX],
            #                                y=param[scenario_key.PARAM_KEY_COMMON_RY],
            #                                z=param[scenario_key.PARAM_KEY_COMMON_RZ])
            # print q
            # wpose.orientation.x -= q.x
            # wpose.orientation.y -= q.y
            # wpose.orientation.z -= q.z
            # wpose.orientation.w -= q.w
            waypoints = []
            waypoints.append(copy.deepcopy(wpose))
            # カテシアンパスで経路生成
            if scenario_key.PARAM_KEY_ARM_PRECISION_VELOCITY in param:
                self._plan_move_cartesian(waypoints, param[scenario_key.PARAM_KEY_ARM_PRECISION_VELOCITY], param[scenario_key.PARAM_KEY_ARM_PLAN_KEY])
            else:
                self._plan_move_cartesian(waypoints, 0.05, param[scenario_key.PARAM_KEY_ARM_PLAN_KEY])
            result = True
        else:
            return_param[global_data.RETURN_DATA_KEY_ERROR_MESSAGE] = "足りないパラメータがある為、tool_move命令に失敗しました。\n"
            pass

        return result, return_param

    def line_move(self, param):

        return_param = {}

        print("param:{}".format(param))

        if "x" in param and "y" in param and "z" in param:
            pass
        else:
            return_param[global_data.RETURN_DATA_KEY_ERROR_MESSAGE] = "足りないパラメータがある為、tool_move命令に失敗しました。\n"
            return False, return_param

        if "rx" in param and "ry" in param and "rz" in param:
            # オイラーをオリエンテーションに変換
            q = common.degree_to_quaternion(x=param[scenario_key.PARAM_KEY_COMMON_RX],
                                            y=param[scenario_key.PARAM_KEY_COMMON_RY],
                                            z=param[scenario_key.PARAM_KEY_COMMON_RZ])
        elif "orientation" in param:
            q = param["orientation"]
        else:
            return_param[global_data.RETURN_DATA_KEY_ERROR_MESSAGE] = "足りないパラメータがある為、tool_move命令に失敗しました。\n"
            return False, return_param

        # ツールオフセットを設定
        if "tool_name" in param:
            self._set_move_group(param["tool_name"])
        else:
            self._set_move_group(self._default_group)

        # 姿勢設定
        if "posture" in param and param["posture"] != -1:
            self.robot_interface.set_posture(param["posture"])
            print("get posture %d" % self.robot_interface.get_posture())

        wpose = Pose()
        # ミリをメートルに変換
        wpose.position.x = param[scenario_key.PARAM_KEY_COMMON_X] / 1000.0
        wpose.position.y = param[scenario_key.PARAM_KEY_COMMON_Y] / 1000.0
        wpose.position.z = param[scenario_key.PARAM_KEY_COMMON_Z] / 1000.0

        wpose.orientation.x = q.x
        wpose.orientation.y = q.y
        wpose.orientation.z = q.z
        wpose.orientation.w = q.w
        print ("line_move_wpose:")
        print (wpose)
        waypoints = []
        waypoints.append(copy.deepcopy(wpose))
        self.group1.set_pose_reference_frame(self._default_reference_frame)
        # カテシアンパスで経路生成
        if scenario_key.PARAM_KEY_ARM_PRECISION_VELOCITY in param:
            self._plan_move_cartesian(waypoints, param[scenario_key.PARAM_KEY_ARM_PRECISION_VELOCITY], param[scenario_key.PARAM_KEY_ARM_PLAN_KEY])
        else:
            self._plan_move_cartesian(waypoints, 0.05, param[scenario_key.PARAM_KEY_ARM_PLAN_KEY])

        return True, return_param

    def get_position(self):
        return_param = {}
        self._set_move_group(self._default_group)
        return_param["position"] = self.group1.get_current_pose().pose
        return True, return_param

    def get_current_joint(self):
        joint_values = self.group1.get_current_joint_values()
        return_param = {}
        return_param["joint"] = map(lambda x: math.degrees(x), joint_values)
        return True, return_param

    def _set_move_group(self, group_name):
        try:
            self.group1 = self._group_list[group_name]
        except Exception:
            ef.output_error("指定されたtool_nameが間違っています\n")

    def _quaternion_to_euler(self, quaternion):
        e = tf.transformations.euler_from_quaternion(quaternion, 'rzyx')

        return [math.degrees(x) for x in e]

    def get_object_position(self, param):
        target = param[scenario_key.PARAM_KEY_COMMON_OBJECT_NAME]
        timeout = param[scenario_key.PARAM_KEY_COMMON_TIMEOUT]

        if timeout is not None:
            obj_pose = self._gu.get_current_pose(self.base_origin, '/' + target, timeout)
        else:
            while True:
                obj_pose = self._gu.get_current_pose(self.base_origin, '/' + target, 4)
                if obj_pose is not None:
                    break

        return_param = {}
        if obj_pose is None:
            return_param['position'] = None
            return False, return_param
        else:
            euler = self._quaternion_to_euler(obj_pose[1])

            keys = [scenario_key.PARAM_KEY_COMMON_RZ, scenario_key.PARAM_KEY_COMMON_RY, scenario_key.PARAM_KEY_COMMON_RX]
            for i, key in enumerate(keys):
                if param[key] is not None:
                    euler[i] = param[key]

            return_param['position'] = position.Position(obj_pose[0][0] * 1000, obj_pose[0][1] * 1000, obj_pose[0][2] * 1000, euler[0], euler[1], euler[2])
            return True, return_param

    def _plan_move(self, key):
        start_time = time.time()
        # 経路生成
        if key in self.plan_cache:
            # 保存されているプランニングを利用する
            plan = self.plan_cache[key]
            if self._check_usable_plan(plan) is False:
                # スタート位置の差分が大きすぎる場合
                # 再度経路を算出
                plan = self._plan()
                self.plan_cache[key] = plan
        elif key is None:
            # プランニングを利用する
            plan = self._plan()
        else:
            # 初回だけプランニングを行い、取得したプランニングを保存する
            plan = self._plan()
            self.plan_cache[key] = plan
        end_time = time.time() - start_time
        print("_plan_move_start time:{}".format(end_time))
        result = self.group1.execute(plan, wait=True)
        if result is False:
            raise MoveException("Robot Move Error")

    def _plan(self):
        plan = self.group1.plan()
        if len(plan.joint_trajectory.points) == 0:
            raise PlanningException("Error Planning")
        return plan

    def _plan_move_cartesian(self, waypoints, velocity, key):
        start_time = time.time()
        # 経路生成
        if key in self.plan_cache:
            plan = self.plan_cache[key]
            if self._check_usable_plan(plan) is False:
                # スタート位置の差分が大きすぎる場合
                # 再度経路を算出
                plan = self._cartesian_plan(waypoints)
                self.plan_cache[key] = plan
        elif key is None:
            print("enable cartesian plan")
            plan = self._cartesian_plan(waypoints)
        else:
            print("execute cartesian plan")
            plan = self._cartesian_plan(waypoints)
            self.plan_cache[key] = plan
        end_time = time.time() - start_time
        print("_plan_move_start time:{}".format(end_time))
        # 速度設定
        retime = self.group1.retime_trajectory(self.robot.get_current_state(), plan, velocity)
        result = self.group1.execute(retime, wait=True)
        if result is False:
            raise MoveException("Robot Move Error")

    def _cartesian_plan(self, waypoints):
        state = self.robot.get_current_state()
        self.group1.set_start_state(state)
        (plan, fraction) = self.group1.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.group1.set_start_state_to_current_state()
        if fraction != 1.0:
            raise PlanningException("Planning Error")
        return plan

    def _check_usable_plan(self, plan):
        plan_start_joint = plan.joint_trajectory.points[0].positions
        current_joint = self.group1.get_current_joint_values()
        for i in range(len(plan_start_joint)):
            diff = abs(plan_start_joint[i] - current_joint[i])
            if diff > self.JOINT_TOLERANCE:
                print("スタート位置が異常です。")
                return False
        return True
