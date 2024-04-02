# Copyright (c) 2023-2024 Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BOARD_MAX32655EVKIT_MAX32655_M4)
  board_runner_args(openocd --cmd-pre-init "source [find interface/cmsis-dap.cfg]")
  board_runner_args(openocd --cmd-pre-init "source [find target/max32655.cfg]")
  board_runner_args(openocd --cmd-pre-init "allow_low_pwr_dbg")
  board_runner_args(jlink "--device=MAX32655" "--reset-after-load")
elseif(CONFIG_BOARD_MAX32655EVKIT_MAX32655_RV32)
  board_runner_args(openocd --cmd-pre-init "source [find interface/ftdi/olimex-arm-usb-ocd-h.cfg]")
  board_runner_args(openocd --cmd-pre-init "source [find target/max32655_riscv.cfg]")
endif()

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
