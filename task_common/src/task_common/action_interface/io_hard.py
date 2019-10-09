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

from fsrobo_r_driver.io_interface import IOInterface
from task_common.key import scenario_key
from task_common.action_interface import global_data


class IOHard:
    """
    アームロボット
    """
    global_param = {}

    def __init__(self, io_name):
        """
        初期処理
        :param group:
        :param robot_name:
        :param base_origin:
        """
        print("io_name:{}".format(io_name))
        self._io = IOInterface(io_name, init_node=False)

    def task_run(self, func, param):
        """
        タスクを実行する
        :param func:
        :param param:
        :return:
        """
        data = {}
        if func == scenario_key.IO_MOTION_GET_IO_STATE:
            result, data = self.get_io_state(param)
        elif func == scenario_key.IO_MOTION_GET_IO_STATE_ALL:
            result, data = self.get_io_state_all()
        elif func == scenario_key.IO_MOTION_SET_IO:
            result = self.set_io(param)
        elif func == scenario_key.IO_MOTION_WAIT_IO:
            result = self.wait_io(param)
        else:
            result = False
            data[global_data.RETURN_DATA_KEY_ERROR_MESSAGE] = "IO操作に" + func + "操作は存在しません。\n"

        return result, data

    def get_io_state(self, param):
        addr = param[scenario_key.PARAM_KEY_IO_ADDRESS]
        io_state = self._io.get_digital_val(addr)
        io_state_dict = {addr: io_state}
        return_param = {}
        return_param[scenario_key.PARAM_KEY_IO_STATE] = io_state_dict
        return True, return_param

    def get_io_state_all(self):
        io_state_dict = self._io.get_digital_vals()
        return_param = {}
        return_param[scenario_key.PARAM_KEY_IO_STATE] = io_state_dict
        return True, return_param

    def set_io(self, param):
        addr = param[scenario_key.PARAM_KEY_IO_ADDRESS]
        data = param[scenario_key.PARAM_KEY_IO_SET_VALUE]
        wait = param[scenario_key.PARAM_KEY_IO_WAIT]
        self._io.set_digital_val(addr, *data)
        if wait is True:
            self._io.wait_digital_val(addr, list(data))
        return True

    def wait_io(self, param):
        addr = param[scenario_key.PARAM_KEY_IO_ADDRESS]
        data = param[scenario_key.PARAM_KEY_IO_SET_VALUE]
        time_out = param[scenario_key.PARAM_KEY_IO_TIME_OUT]
        if time_out is None:
            print("time_out None")
            self._io.wait_digital_val(addr, data)
        else:
            print("time_out:{}".format(time_out))
            self._io.wait_digital_val(addr, data, time_out)
        return True
