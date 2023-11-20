.. _adsp_sc83x:

ADSP SC83x
##########

Overview
********

The adsp_sc83(34/34W/35/35W) board configuration is used by Zephyr applications
that run on the m33 core of SHARC-FX ADSP-SC83x boards. It provides support for
the ADSP-SC83x ARM Cortex-M33 CPU and the following devices:

- Nested Vectored Interrupt Controller (NVIC)
- System Tick System Clock (SYSTICK)
- Cortex-M System Design

Zephyr board options
====================

The ADSP-SC83x is one core SoC with Cortex-M33 architecture on CPU2.
Zephyr provides support for building firmware images for CPU2.

The BOARD options are summarized below:

+-------------------------+-------------------------------------------------------+
| BOARD                   | Description                                           |
+=========================+=======================================================+
| adsp_sc834              |                                                       |
| adsp_sc834W             |                                                       |
| adsp_sc835              | For building Secure (or Secure-only) firmware on CPU2 |
| adsp_sc835W             |                                                       |
+-------------------------+-------------------------------------------------------+

Hardware
********

ADSP-SC83xx provides the following hardware components:

- One core SHARC-FX DSP (core 1)
- One core ARM Cortex-M33 (core 2)
- Debug

  - Debug access port (DAP) provides IEEE 1149.1 JTAG interface

Supported Features
==================

The adsp_sc83xx boards configuration supports the following hardware features:

+-----------+------------+-------------------------------------+
| Interface | Controller | Driver/Component                    |
+===========+============+=====================================+
| NVIC      | on-chip    | nested vector interrupt controller  |
+-----------+------------+-------------------------------------+
| SYSTICK   | on-chip    | systick                             |
+-----------+------------+-------------------------------------+

The default configuration can be found in the defconfig files:
``boards/arm/adsp_sc83xx/adsp_sc834_defconfig``
``boards/arm/adsp_sc83xx/adsp_sc834w_defconfig``
``boards/arm/adsp_sc83xx/adsp_sc835_defconfig``
``boards/arm/adsp_sc83xx/adsp_sc835w_defconfig``.

Memory Map (L2 SRAM - SC834/SC34W)
==================================
+-----------+------------+-------------------------------------+
| Memory    |  BASE      |  SIZE                               |
+===========+============+=====================================+
| L1_IRAM   | 0x00040000 | 0x00010000                          |
+-----------+------------+-------------------------------------+
| L1_DRAM   | 0x28AC0000 | 0x00020000                          |
+-----------+------------+-------------------------------------+
| L2_SRAM   | 0x20100000 | 0x00020000                          |
+-----------+------------+-------------------------------------+
| L3_SDRAM  | 0x90000000 | 0x10000000                          |
+-----------+------------+-------------------------------------+

Memory Map (L2 SRAM - SC835/SC35W)
==================================
+-----------+------------+-------------------------------------+
| Memory    |  BASE      |  SIZE                               |
+===========+============+=====================================+
| L1_IRAM   | 0x00040000 | 0x00010000                          |
+-----------+------------+-------------------------------------+
| L1_DRAM   | 0x28AC0000 | 0x00020000                          |
+-----------+------------+-------------------------------------+
| L2_SRAM   | 0x20000000 | 0x00020000                          |
+-----------+------------+-------------------------------------+
| L3_SDRAM  | 0x90000000 | 0x10000000                          |
+-----------+------------+-------------------------------------+

Interrupt Controller
====================

Core 2 ADSP-SC83x is a Cortex-M33 based SoC and has Nested Vectored Interrupt
Controller (NVIC).

A Cortex-M33-based board uses vectored exceptions. This means each exception
calls a handler directly from the vector table.
