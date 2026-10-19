# ird_ethercat_hardware_interface

A [ros2_control](https://control.ros.org) hardware interface for **Infranor IRD** servo drives
(tested with IRD-eb-48/60) on an **EtherCAT** bus. It uses the open-source
[SOEM](https://github.com/OpenEtherCATsociety/SOEM) EtherCAT master.

Developed by **CDEI-UPC** (Centre de Disseny d'Equips Industrials, Universitat Politècnica de
Catalunya) within the DISCOVER Horizon Europe project.

> **Status: version 0.1 (under validation).** This version controls three drives in
> Cyclic Synchronous Velocity mode. Its parameters come from the URDF. Coming next: any number
> of drives, a single YAML configuration file (bus timing, PDO mapping, startup SDOs, registers
> exposed to ROS) and selectable CSP/CSV/CST modes.

## What it does

- Configures every drive at startup over SDO: mode of operation (CSV), PDO mapping, cycle
  period (0x1006) and synchronisation mode (0x2012).
- Exchanges process data every **4 ms** from a dedicated real-time thread (SCHED_FIFO). The
  thread is synchronised to the drives with **distributed clocks** (DC SYNC0, IRD *HardSync*).
- Runs the CiA 402 enable sequence through the cyclic controlword. No SDO traffic happens
  during operation.
- Recovers automatically, within a bounded number of attempts, from transient fieldbus faults
  (IRD errors 49/50), then ramps the velocity references back up smoothly.
- Logs fault snapshots (0x603F, 0x1001, 0x3022, 0x3024), ESC error counters and per-cycle
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

Add a `<ros2_control>` block to your robot description. A complete example with every
parameter explained is in
[`urdf/ird_ethercat_example.ros2_control.xacro`](urdf/ird_ethercat_example.ros2_control.xacro).

| Parameter | Required | Meaning |
|---|---|---|
| `ethercat_interface` | no (default `eno1`) | Network interface connected to the EtherCAT bus |
| `left_wheel_name`, `right_wheel_name`, `turret_name` | yes | Joint driven by each of the three drives |
| `slave_left_motor`, `slave_right_motor`, `slave_turret_motor` | yes | EtherCAT position of each drive (1 = first after the master) |
| `transmission_ratio_wheels`, `transmission_ratio_turret` | yes | Motor revolutions per joint revolution |
| `encoder_ticks_per_rev` | yes | Encoder ticks per motor revolution (the drive's velocity unit is ticks/s) |

Each joint must declare one `velocity` command interface and the `position` and `velocity`
state interfaces, in that order. Commands and states are in rad/s and rad at the joint, after
the gearbox. The position is integrated from the measured velocity.

## Code structure

| File | Contents |
|---|---|
| `config.hpp/.cpp` | Configuration structures (bus, fault handling, drives), defaults and validation |
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
