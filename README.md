# ird_ethercat_hardware_interface

A [ros2_control](https://control.ros.org) hardware interface for **Infranor IRD** servo drives
(tested with IRD-eb-48/60) on an **EtherCAT** bus, based on the open-source
[SOEM](https://github.com/OpenEtherCATsociety/SOEM) EtherCAT master.

Developed by **CDEI-UPC** (Centre de Disseny d'Equips Industrials, Universitat Politècnica de
Catalunya) within the DISCOVER Horizon Europe project.

Everything about the drives is set in **one YAML file**: the bus, the PDO mapping, the registers
to read or write from ROS, units, limits and fault handling. Using it with your own drives needs
no code changes.

## Features

- **Any number of drives** on one bus, each driving one ros2_control joint.
- **Modes:** cyclic synchronous velocity (CSV), position (CSP) and torque (CST), selectable per
  drive. CSV is validated on hardware; CSP and CST are implemented but not yet validated on
  hardware.
- **Configured at every startup:** mode, PDO mapping, cycle time and synchronisation are written
  to the drives, so no fieldbus setup in Optio is needed.
- **Your registers in ROS:** any object added to the PDO mapping (currents, voltages, position,
  I/O...) becomes a ros2_control state or command interface, published by
  `joint_state_broadcaster` with no extra code.
- **Long mappings split automatically:** an IRD PDO holds 8 bytes and 4 objects, so a longer
  mapping is spread over the next mapping objects (0x1A01, 0x1A02, ...) without any extra setting.
- **Real-time process data:** one frame per cycle (4 ms by default) from a dedicated SCHED_FIFO
  thread, optionally pinned to a reserved CPU, locked to the drives' distributed clock (IRD
  HardSync or SoftSync).
- **Checked startup:** every drive's clock is checked after the clock setup (redone if a frame was
  lost) and must follow the reference drive before anything is enabled; the OPERATIONAL request
  is repeated to drives that missed it.
- **Robust operation:** automatic, bounded recovery from transient fieldbus faults (IRD errors
  49/50), a smooth restart after a fault, detection of a lost bus (e.g. an emergency stop that
  cuts the drives' power) within 100 ms, and a clean shutdown that leaves no fault latched in the
  drives.
- **Diagnostics in the log:** fault snapshots read from the drive, per-cycle timing before each
  fault, and the EtherCAT error counters. Every configuration mistake is reported with its line
  number.

## Quick start

```bash
# SOEM 2.x (the EtherCAT master library)
git clone --branch v2.0.0 https://github.com/OpenEtherCATsociety/SOEM.git ~/SOEM
cmake -S ~/SOEM -B ~/SOEM/build -DCMAKE_POSITION_INDEPENDENT_CODE=ON && cmake --build ~/SOEM/build -j
sudo cmake --install ~/SOEM/build

# This package
cd ~/ros2_ws/src && git clone https://github.com/DISCOVER-Horizon-Europe-Project/ird_ethercat_hardware_interface.git
cd ~/ros2_ws && colcon build --packages-select ird_ethercat_hardware_interface && source install/setup.bash

# Permissions for the EtherCAT master (see the guide for the library path)
sudo setcap cap_net_raw,cap_sys_nice,cap_ipc_lock+ep /opt/ros/humble/lib/controller_manager/ros2_control_node

# First test: one drive at slave position 1, velocity mode (config/minimal_example.yaml;
# copy it and pass config_file:=... to match your motor, see the guide)
ros2 launch ird_ethercat_hardware_interface example.launch.py ethercat_interface:=eno1
```

Follow the [getting started guide](docs/getting_started.md) for the full procedure.

## Documentation

| Document | Contents |
|---|---|
| [Getting started](docs/getting_started.md) | From a PC with ROS 2 to a drive turning: requirements, installation, permissions, configuration, first motion, use in your robot |
| [Configuration reference](docs/configuration.md) | Every key of the configuration file: bus, fault handling, drives, PDO mapping, exposing registers to ROS, modes, units, startup SDOs |
| [How it works](docs/how_it_works.md) | Startup sequence, the cycle, synchronisation, enabling, fault recovery, shutdown, diagnostics |
| [Troubleshooting](docs/troubleshooting.md) | Every error and warning in the log with its cause and fix; errors 49/50; AL status, SDO abort and statusword tables |
| [`config/ird_csv_example.yaml`](config/ird_csv_example.yaml) | Configuration template with every key, commented |
| [`config/minimal_example.yaml`](config/minimal_example.yaml) | The shortest valid configuration |
| [`config/other_modes_example.yaml`](config/other_modes_example.yaml) | CSP and CST mappings |
| [CHANGELOG](CHANGELOG.md) | Versions |

## Requirements

| Item | Version / notes |
|---|---|
| OS | Ubuntu 22.04; a PREEMPT_RT kernel is strongly recommended |
| ROS 2 | Humble, with `ros2_control` (tested with 2.54) |
| SOEM | 2.x (tested with v2.0.0); installed, or downloaded automatically by the build |
| Network | An Ethernet port used only for EtherCAT |

## Tests

The unit tests need no hardware:

```bash
colcon test --packages-select ird_ethercat_hardware_interface && colcon test-result --verbose
```

They cover the configuration (file reading and validation), the PDO layout, the startup SDO
sequence, the CiA 402 logic, the clock synchronisation and the enable/fault-recovery logic. That
includes a 600,000-cycle comparison with the per-cycle logic of the version validated on
hardware.

## License

Copyright © 2026 CDEI-UPC. Distributed under the **GNU General Public License v3.0** (see
[LICENSE](LICENSE)). Portions derived from the ros2_control demos are © 2021 ros2_control
Development Team, Apache License 2.0.

This package links against SOEM, which is dual-licensed: GPLv3, or a commercial license from
[RT-Labs](https://rt-labs.com). If you integrate this package into a product that you do not
distribute under GPLv3, you need a commercial SOEM license from RT-Labs.
