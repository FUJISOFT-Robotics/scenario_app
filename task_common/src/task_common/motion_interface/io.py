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


def get_io(action_manager, io_name, param):
    """
    IOの状態を取得
    """
    if scenario_key.PARAM_KEY_IO_ADDRESS in param:
        result, data = action_manager.io_operation(io_name, scenario_key.IO_MOTION_GET_IO_STATE, param)
    else:
        result, data = action_manager.io_operation(io_name, scenario_key.IO_MOTION_GET_IO_STATE_ALL, param)
    return result, data[scenario_key.PARAM_KEY_IO_STATE]


def set_io(action_manager, io_name, param):
    """
    IOを設定
    """
    result, _ = action_manager.io_operation(io_name, scenario_key.IO_MOTION_SET_IO, param)
    return result


def wait_io(action_manager, io_name, param):
    """
    IOが指定する状態になるまで待機
    """
    result, _ = action_manager.io_operation(io_name, scenario_key.IO_MOTION_WAIT_IO, param)
    return result
