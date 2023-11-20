#
# Copyright (c) 2023 Analog Devices, Inc
#
# SPDX-License-Identifier: Apache-2.0

# We can do the right thing for supported subfamilies using a generic
# script, at least for openocd 0.10.0 and the openocd shipped with
# Zephyr SDK 0.10.3.

set(pre_init_cmds
  "source [find interface/ice2000.cfg]"
  "transport select swd"
  "set USE_CTI 1"
  "set GDB-PORT 3334"

)

foreach(cmd ${pre_init_cmds})
  board_runner_args(--cmd-pre-init "${cmd}")
endforeach()

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
