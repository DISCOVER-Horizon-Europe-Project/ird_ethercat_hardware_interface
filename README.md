# ird_ethercat_hardware_interface

A [ros2_control](https://control.ros.org) hardware interface for **Infranor IRD** servo drives
(tested with IRD-eb-48/60) on an **EtherCAT** bus. It uses the open-source
[SOEM](https://github.com/OpenEtherCATsociety/SOEM) EtherCAT master.

Developed by **CDEI-UPC** (Centre de Disseny d'Equips Industrials, Universitat Politècnica de
Catalunya) within the DISCOVER Horizon Europe project.

> **Status: under development.** Any number of drives in Cyclic Synchronous Velocity mode,
> configured from one YAML file. Coming next: extra PDO objects exposed to ROS and selectable
> CSP/CSV/CST modes.

## What it does

- Configures every drive at startup over SDO from a YAML file: mode of operation, PDO mapping,
  any extra objects you list, cycle period (0x1006) and synchronisation mode (0x2012).
- Exchanges process data every cycle (4 ms by default) from a dedicated real-time thread (SCHED_FIFO). The
  thread is synchronised to the drives with **distributed clocks** (DC SYNC0, IRD *HardSync*).
- Runs the CiA 402 enable sequence through the cyclic controlword. No SDO traffic happens
  during operation.
- Recovers automatically, within a bounded number of attempts, from transient fieldbus faults
  (IRD errors 49/50), then ramps the velocity references back up smoothly.
- Logs fault snapshots (by default 0x603F, 0x1001, 0x3022, 0x3024), ESC error counters and per-cycle
  timing from a separate diagnostics thread, so diagnostics never stall the control loop.

No configuration with Infranor's Optio software is needed. Every EtherCAT communication
setting is written by this interface at startup.

## Requirements

| Item | Version / notes |
|---|---|
| OS | Ubuntu 22.04 (a PREEMPT_RT kernel is strongly recommended) |
| ROS 2 | Humble, with `ros2_control` |
| SOEM | **2.x** (tested with v2.0.0). SOEM 1.x is **not** compatible. |
| Network | A dedicated Ethernet port for EtherCAT is strongly recommended. Other traffic on the same port causes timing jitter, which the drives report as error 49/50. |

## Installation

### 1. SOEM

You can either install SOEM on the system (recommended for robots) or let the build download it.

**Option A: install it on the system.**

```bash
git clone --branch v2.0.0 https://github.com/OpenEtherCATsociety/SOEM.git ~/SOEM
cmake -S ~/SOEM -B ~/SOEM/build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build ~/SOEM/build -j
sudo cmake --install ~/SOEM/build      # installs to /usr/local
```

**Option B: automatic download.** If no installed SOEM is found, the build downloads and
compiles SOEM v2.0.0 automatically (this needs internet access during the build). To forbid
the download, pass `--cmake-args -DIRD_FETCH_SOEM=OFF`.

### 2. Build the package

```bash
cd ~/ros2_ws/src
git clone https://github.com/DISCOVER-Horizon-Europe-Project/ird_ethercat_hardware_interface.git
cd ~/ros2_ws
colcon build --packages-select ird_ethercat_hardware_interface --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### 3. Grant the controller manager the capabilities it needs

The EtherCAT master needs three Linux capabilities:

- **raw network access**, to send EtherCAT frames;
- **real-time scheduling**, for the process-data thread;
- **memory locking**, to avoid page faults in the real-time thread.

You can run as root or grant these capabilities to the executable that loads the plugin
(normally `ros2_control_node`):

```bash
sudo setcap cap_net_raw,cap_sys_nice,cap_ipc_lock+ep /opt/ros/humble/lib/controller_manager/ros2_control_node
```

Executables with capabilities ignore `LD_LIBRARY_PATH`. If `ros2_control_node` then fails to
find libraries from your workspace, add those library directories to a file in
`/etc/ld.so.conf.d/` and run `sudo ldconfig`. Re-run `setcap` whenever the ROS packages are
updated, because updates replace the executable.

If one of the capabilities is missing, the interface logs it at startup. The messages are
`Failed to initialize SOEM` (raw network), `could not enable SCHED_FIFO` (real-time scheduling)
and `mlockall failed` (memory locking).

## Usage

### 1. Write the configuration file

Everything about the drives lives in one YAML file: the bus (network interface, cycle time,
synchronisation), fault handling, and each drive (EtherCAT position, ros2_control joint, units,
limits, PDO mapping, startup SDOs). Start from one of the shipped examples:

- [`config/minimal_example.yaml`](config/minimal_example.yaml): the shortest valid file, one
  drive with every other setting at its default.
- [`config/ird_csv_example.yaml`](config/ird_csv_example.yaml): **every** available key with its
  default value and an explanation. Read it as the configuration reference.

Each drive starts from the built-in IRD defaults. An optional `drive_defaults` section then
overrides them for all drives, and each entry under `drives` can override them again. At
startup the file is checked: unknown keys, wrong types or values, duplicate drives, and missing
or mistyped PDO roles are reported with their line number, and the interface does not start.

### 2. Reference it from the robot description

Add a `<ros2_control>` block to your URDF. A ready-to-use xacro macro is in
[`urdf/ird_ethercat_example.ros2_control.xacro`](urdf/ird_ethercat_example.ros2_control.xacro).

```xml
<ros2_control name="drives" type="system">
  <hardware>
    <plugin>ird_ethercat_hardware_interface/IrdEthercatSystem</plugin>
    <param name="config_file">$(find my_robot)/config/drives.yaml</param>
    <!-- optional: overrides bus.interface of the file -->
    <param name="ethercat_interface">eno1</param>
  </hardware>
  <joint name="axis1_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

Declare one joint per drive, with the names used in the configuration file. Each joint needs
one `velocity` command interface and the `position` and `velocity` state interfaces, in that
order. Commands and states are in rad/s and rad at the joint, after the gearbox. The position is
integrated from the measured velocity.

*Legacy parameters:* without `config_file`, the URDF parameters of the original three-drive
layout (`left_wheel_name`, `slave_left_motor`, `transmission_ratio_wheels`,
`encoder_ticks_per_rev`, ...) are still accepted, with every other setting at its default.

## Code structure

| File | Contents |
|---|---|
| `config.hpp/.cpp` | Configuration structures (bus, fault handling, drives), defaults and validation |
| `config_loader.hpp/.cpp` | Reading the YAML configuration file |
| `pdo.hpp/.cpp` | PDO layout (offset of each mapped object) and little-endian encoding |
| `startup_plan.hpp/.cpp` | The SDO writes that configure a drive in PRE-OP (mapping, sync objects) |
| `cia402.hpp` | CiA 402 statusword/controlword helpers |
| `dc_sync.hpp` | PI loop that locks the master's cycle to the drives' distributed clock |
| `cyclic_controller.hpp/.cpp` | Per-cycle logic: enable sequence, fault detection and recovery, reference ramp |
| `ethercat_master.hpp/.cpp` | Thin wrapper around SOEM (bus, SDO, states, process data, registers) |
| `ird_ethercat_system.hpp/.cpp` | The ros2_control plugin: lifecycle, real-time and diagnostics threads |

Everything above `ethercat_master` has no ROS or SOEM dependency and is covered by unit tests.
The tests need no hardware:

```bash
colcon build --packages-select ird_ethercat_hardware_interface
colcon test --packages-select ird_ethercat_hardware_interface --event-handlers console_direct+
```

## License

Copyright © 2026 CDEI-UPC. This package is distributed under the **GNU General Public License
v3.0** (see [LICENSE](LICENSE)). It links against SOEM, which is itself dual-licensed: GPLv3,
or a commercial license from RT-Labs. If you integrate this package into a product that you do
not distribute under GPLv3, you need a commercial SOEM license from
[RT-Labs](https://rt-labs.com).
