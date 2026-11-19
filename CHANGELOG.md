# Changelog

## 0.2.0 (2026-11-19)

Tested on the Naviwall robot (three IRD-eb-48/60 drives, firmware revision 0x22, CSV, ROS 2
Humble with ros2_control 2.54 on a Jetson Orin with a PREEMPT_RT kernel):
- 2026-11-03 and 04: about 1 h of joystick driving with position, current and DC-link voltage
  mapped over three TxPDOs, 20 start/stop cycles, and an e-stop that cuts the drives' power.
- 2026-11-10 to 19: the robot's normal operation (about 80 starts, including autonomous mapping
  sessions) with the same configuration.

- **Configuration file.** Everything is configured from one YAML file named by the URDF parameter
  `config_file`: bus, fault handling, and per drive the joint, units, limits, PDO mapping,
  startup SDOs and fault snapshot objects. Unknown keys and invalid values are reported with
  their line number. The parameters of 0.1.0 are still accepted when `config_file` is not given.
- **Any number of drives** (previously exactly three).
- **Configurable PDO mapping.** Sizes and offsets are computed from the mapped objects, and
  logged at startup.
- **Drive registers in ROS.** Any mapped object with an `interface` name becomes a ros2_control
  state interface (TxPDO) or command interface (RxPDO), with an optional scale.
- **Modes CSP and CST** in addition to CSV, selectable per drive. In CSP the reference holds the
  actual position until the drive is enabled and is limited to `max_motor_rpm`.
- **Joint states from the drive.** If `position_actual` is mapped, the joint position comes from
  the drive (no longer integrated from the velocity). If `torque_actual` is mapped, an `effort`
  state is added.
- Optional identity check (vendor id, product code) of each drive at startup.
- Example launch file with a one-drive robot (`launch/example.launch.py`).
- Documentation: getting started, configuration reference, how it works, troubleshooting.
- Unit tests (no hardware needed), including a comparison of the cyclic logic with 0.1.0.
- **PDO mappings longer than one PDO.** The entries are split over consecutive mapping objects
  (0x1A00, 0x1A01, ...) according to the drive's limits (`pdo_limits`; IRD: 8 bytes and 4 objects
  per PDO, measured on the robot). Position, current and DC-link voltage can now be read next to
  the velocity. A mapping that does not fit is reported when the configuration is loaded.

Field fixes from the Naviwall robot's driver (2026-10-20 to 2026-10-27), not in 0.1.0:
- **DC clock setup check:** after the clock setup, every drive's clock is compared with the PC's
  and the setup is redone if a drive was set wrong (a lost frame refused 3 of 11 starts).
- **Drive clock check before enabling:** every drive's clock must follow the reference drive
  (register 0x092C) before anything is enabled; otherwise activation fails instead of faulting
  with error 50 every few seconds.
- **OPERATIONAL request repeated** every 200 ms to drives that missed it; on failure the AL status
  code of every drive is read and explained.
- **CPU pinning** of the process-data thread (`bus.rt_cpu`; the legacy URDF parameters default to
  the last CPU, as before).
- **Diagnostics:** the timing dump shows the send time and the CPU of each cycle, and each drive's
  DC sync difference is logged periodically and with every fault.

Found in the robot tests of 0.2.0:
- **Loss of contact with the drives is detected.** An e-stop that cuts the drives' power went
  unnoticed (in 0.1.0 and in the robot's driver): no frame came back, the last statuswords still
  read *Operation Enabled*, and the joint states froze (or drifted, with the integrated position).
  Now, after `bus_loss_timeout_ms` (100 ms) without a complete frame, the interface logs an
  error, stops the drives, stops updating the joint states and returns ERROR from `read()`
  (`bus_loss_read_error`), and closes the bus without waiting for the drives on shutdown.

Documentation updated from the robot tests: reserving the CPU of the process-data thread with
`isolcpus=` (systemd `CPUAffinity=` can be undone by cgroup cpusets), and the IRD's actual
position starting at 0 at every drive power-up.

The default PDO mapping and the cyclic behaviour in CSV mode are unchanged from 0.1.0.

## 0.1.0

First version as a separate package: the EtherCAT interface of the Naviwall robot (three IRD
drives, CSV, HardSync, 4 ms cycle), configured with URDF parameters.

Fixes compared with the original driver:
- the wait for OPERATIONAL used a timeout in the wrong unit (about 2 hours instead of 8 s),
- a failed enable no longer leaves the drives with a latched fieldbus fault,
- the network socket is closed when the PDO sizes do not match,
- waiting for SAFE-OP at shutdown no longer blocks the process-data thread,
- missing or invalid URDF parameters give a clear error,
- repeated warnings in `write()` are throttled.
