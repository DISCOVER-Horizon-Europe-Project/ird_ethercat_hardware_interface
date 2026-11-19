# Configuration reference

The hardware interface is configured by one YAML file, named in the URDF. This page describes
every key. [`config/ird_csv_example.yaml`](../config/ird_csv_example.yaml) contains the same keys,
each set to its default, and is a good template.

**Contents**

- [URDF parameters](#urdf-parameters)
- [File structure](#file-structure)
- [bus](#bus)
- [fault_handling](#fault_handling)
- [Drive keys](#drive-keys) (`drive_defaults` and `drives`)
- [PDO mapping](#pdo-mapping)
- [Exposing drive registers to ROS](#exposing-drive-registers-to-ros)
- [Modes of operation](#modes-of-operation)
- [Units](#units)
- [Startup SDOs](#startup-sdos)
- [Fault snapshot](#fault-snapshot)
- [Validation](#validation)

---

## URDF parameters

The `<hardware>` block of the `<ros2_control>` tag takes:

| Parameter | Required | Meaning |
|---|---|---|
| `config_file` | yes | Absolute path of the YAML file. With xacro: `$(find my_robot)/config/drives.yaml`. |
| `ethercat_interface` | no | Network interface; if not empty, it replaces `bus.interface` of the file. Useful to choose the port from a launch argument. |

Each drive's joint must be declared in the same `<ros2_control>` block (see
[Modes of operation](#modes-of-operation) for the interfaces each joint needs).

<details>
<summary>Legacy parameters (without <code>config_file</code>)</summary>

For compatibility with the first version, a `<hardware>` block without `config_file` is read as
three IRD drives in velocity mode, with every other setting at its default:

| Parameter | Meaning |
|---|---|
| `left_wheel_name`, `right_wheel_name`, `turret_name` | joints of the three drives |
| `slave_left_motor`, `slave_right_motor`, `slave_turret_motor` | their slave positions |
| `transmission_ratio_wheels`, `transmission_ratio_turret` | gear ratios (wheels, turret) |
| `encoder_ticks_per_rev` | encoder resolution |
| `ethercat_rt_cpu` | optional: `bus.rt_cpu` (default here `-2`, the last CPU, as in the first version) |

New setups should use `config_file`.
</details>

## File structure

```yaml
bus:             # the EtherCAT bus (optional, defaults below)
  ...
fault_handling:  # enabling and fault recovery (optional, defaults below)
  ...
drive_defaults:  # drive keys shared by all drives (optional)
  ...
drives:          # one entry per drive (required)
  - name: ...
    ...
```

Each drive is built in three layers, and a later layer replaces what an earlier one set:

1. the built-in defaults for an IRD drive in velocity mode (the defaults in this page),
2. `drive_defaults`,
3. the drive's own entry in `drives`.

Lists (`rx_pdo`, `tx_pdo`, `startup_sdos`, `fault_snapshot`) are replaced as a whole, never merged.

**Notation**

- Objects of the drive's object dictionary are written as in the drive manuals: `"0xIIII:SS"`,
  index and subindex in hexadecimal, for example `"0x6040:00"`. Write them **in quotes**,
  otherwise YAML reads the colon as a separator. `"0x6040"` alone means subindex 0.
- Integers can be decimal (`4000`) or hexadecimal (`0x1600`).
- Data types: `int8`, `uint8`, `int16`, `uint16`, `int32`, `uint32`.
- Booleans: `true` or `false`.

## bus

| Key | Default | Meaning |
|---|---|---|
| `interface` | `eno1` | Network interface connected to the drives (`ip link` lists them). |
| `cycle_time_us` | `4000` | Process-data cycle in µs (250 to 100000). Written to every drive as 0x1006:00. |
| `rt_priority` | `80` | SCHED_FIFO priority (1–99) of the process-data thread. |
| `rt_cpu` | `-1` | CPU the process-data thread is pinned to: `-1` = not pinned, `-2` = the last CPU, or a CPU number. Pin it only to a CPU kept free of other real-time threads and out of deep idle states, see [How it works](how_it_works.md#4-synchronisation). |
| `distributed_clocks` | `true` | Synchronise the drives with EtherCAT distributed clocks (DC SYNC0). Keep it on. |
| `sync_mode` | `hard_sync` | IRD synchronisation mode (0x2012:01): `hard_sync` (4) or `soft_sync` (2). See [How it works](how_it_works.md#4-synchronisation). |
| `sync0_shift_us` | `2000` | Time from the frame to SYNC0, when the drives latch the new references. Must be smaller than `cycle_time_us`. The IRD needs at least 100 µs. Increase it if the master's timing is noisy. |
| `dc_lock_tolerance_us` | `200` | Before going to OPERATIONAL, the frames must stay within this distance of their target timing... |
| `dc_lock_ms` | `200` | ...for this long in a row... |
| `dc_lock_timeout_ms` | `3000` | ...or this long at most, after which startup continues with a warning. |
| `startup_settle_ms` | `1000` | Frames exchanged at startup with all outputs at zero, to let the synchronisation settle. |
| `dc_setup_tolerance_us` | `1000` | Right after the clock setup, every drive's clock must agree with the PC's within this, otherwise the setup is redone... |
| `dc_setup_attempts` | `3` | ...up to this many times in total. |
| `dc_sync_tolerance_ns` | `10000` | After the settle, every drive's clock must be within this of the reference drive's (register 0x092C)... |
| `dc_sync_timeout_ms` | `2000` | ...within this time, otherwise startup fails without enabling any drive. |

## fault_handling

| Key | Default | Meaning |
|---|---|---|
| `enable_timeout_ms` | `5000` | Activation fails if not every drive reaches *Operation Enabled* in this time. |
| `enable_stable_ms` | `40` | All drives must stay *Operation Enabled* this long before the controllers' commands are applied. |
| `enable_fault_abort_ms` | `100` | During activation, a drive that stays in FAULT this long aborts it. |
| `max_recovery_attempts` | `5` | Automatic recoveries per incident after a runtime fault. `0`: never recover automatically. |
| `recovery_cooldown_ms` | `500` | Wait after a fault before trying to reset it. |
| `reset_pulse_ms` | `200` | Longest time the fault-reset command is sent. If the fault has not cleared by then, the attempt failed. |
| `recovery_budget_refill_ms` | `30000` | After this long without faults, the count of attempts starts again from zero. |
| `bus_loss_timeout_ms` | `100` | No frame with every drive's data for this long (e-stop cutting the drives' power, cable unplugged) = contact lost: error logged, drives stopped, no automatic recovery. `0`: never detect. |
| `bus_loss_read_error` | `true` | After a bus loss, `read()` returns ERROR: ros2_control stops the controllers commanding these joints and the interface goes to *unconfigured* (on Humble a `joint_state_broadcaster` keeps running, with the last positions and zero velocities). `false`: `read()` keeps returning OK, positions hold their last value and velocities read 0. |

What happens during a fault and after a bus loss is described in [How it works](how_it_works.md#7-runtime-faults-and-automatic-recovery).

## Drive keys

These keys can be used in `drive_defaults` and in each entry of `drives`. The first three are
only allowed in `drives`.

| Key | Default | Meaning |
|---|---|---|
| `name` | (required) | Name used in log messages. Unique. |
| `joint` | (required) | ros2_control joint driven by this drive. Unique. |
| `slave_position` | (required) | Position on the bus: 1 = first drive after the PC. |
| `mode` | `csv` | Mode of operation: `csv`, `csp` or `cst`. See [Modes of operation](#modes-of-operation). |
| `encoder_ticks_per_rev` | (required, > 0) | Drive position units per motor revolution. See [Units](#units). |
| `gear_ratio` | `1.0` | Motor revolutions per joint revolution. |
| `rated_torque_nm` | `0` | Motor rated torque in Nm (the drive's 0x6076 is in mNm). Required when torque is commanded or read. |
| `max_motor_rpm` | `4910` | Motor speed limit. `csv`: faster commands are clamped (with a warning). `csp`: the position reference never moves faster. |
| `enable_ramp_motor_rad_s2` | `70` | `csv` only. After (re-)enabling, the velocity reference ramps up at this rate (rad/s² at the motor) until it reaches the command. Slowing down and stopping are never delayed. `0` turns the ramp off. |
| `expected_vendor_id` | `0` | If not 0, startup fails unless the device at `slave_position` reports this vendor id (IRD: `0x00000082`). |
| `expected_product_code` | `0` | Same, for the product code (IRD: `0x00000500`). |
| `rx_pdo` | IRD velocity mapping | Objects sent to the drive every cycle. See [PDO mapping](#pdo-mapping). |
| `tx_pdo` | IRD velocity mapping | Objects read from the drive every cycle. |
| `pdo_limits` | IRD: 8 bytes, 4 objects, 4 PDOs | What one mapping object accepts. See [Limits of the drive](#limits-of-the-drive). |
| `startup_sdos` | none | Objects written once at startup. See [Startup SDOs](#startup-sdos). |
| `fault_snapshot` | IRD error objects | Objects read and logged when the drive faults. See [Fault snapshot](#fault-snapshot). |

## PDO mapping

The PDO mapping lists the objects exchanged with the drive every cycle. The interface writes it to
the drive at every startup, so it never needs to be configured in Optio.

```yaml
rx_pdo:                 # master -> drive
  index: 0x1600         # first mapping object (default 0x1600)
  entries:
    - {object: "0x6040:00", type: uint16, role: controlword}
    - {object: "0x6060:00", type: int8, role: mode_of_operation}
    - {object: "0x60FF:00", type: int32, role: target_velocity}
tx_pdo:                 # drive -> master
  index: 0x1A00         # first mapping object (default 0x1A00)
  entries:
    - {object: "0x6041:00", type: uint16, role: statusword}
    - {object: "0x6061:00", type: int8, role: mode_display}
    - {object: "0x606C:00", type: int32, role: velocity_actual}
```

This is the default mapping. Choose additional objects from the drive's object dictionary: only
objects marked *PDO mapping: yes / possible* can be mapped, and `type` must match the object's
data type. Sizes and offsets are computed automatically; at startup the log shows where every
object sits:

```
Drive 'left' TxPDO 0x1A00 (7 bytes): 0x6041:00 uint16 @0 statusword | 0x6061:00 int8 @2 mode_display | 0x606C:00 int32 @3 velocity_actual
```

Each entry has these keys:

| Key | Meaning |
|---|---|
| `object` | `"0xIIII:SS"` (required) |
| `type` | data type (required) |
| `role` | what the interface uses the object for (see below) |
| `interface` | expose the object to ROS under this name (see [next section](#exposing-drive-registers-to-ros)) |
| `scale` | factor from the drive's raw value to the ROS value (default `1`) |

Every entry needs a `role`, an `interface`, or both. An entry with neither would be transferred
for nothing, and is reported as an error.

**Roles**

| Role | PDO | Object | Type | Use |
|---|---|---|---|---|
| `controlword` | Rx | 0x6040 | uint16 | Always required. The interface drives the CiA 402 state machine with it. |
| `mode_of_operation` | Rx | 0x6060 | int8 | Optional. If mapped, the mode is written every cycle (it is also written once by SDO at startup). |
| `target_velocity` | Rx | 0x60FF | int32 | Required in `csv`. |
| `target_position` | Rx | 0x607A | int32 | Required in `csp`. |
| `target_torque` | Rx | 0x6071 | int16 | Required in `cst`. |
| `statusword` | Tx | 0x6041 | uint16 | Always required. |
| `mode_display` | Tx | 0x6061 | int8 | Optional, informative. |
| `velocity_actual` | Tx | 0x606C | int32 | Required in `csv`. Gives the joint `velocity` state. |
| `position_actual` | Tx | 0x6064 | int32 | Required in `csp`. Gives the joint `position` state. |
| `torque_actual` | Tx | 0x6077 | int16 | Optional. Gives the joint `effort` state. |

Each role can appear once. Only the target of the drive's own mode is allowed in the RxPDO: a
target for another mode would never be written. The TxPDO must contain `position_actual` or
`velocity_actual` (or both), because the joint states are derived from them: without
`position_actual` the position is integrated from the velocity, and without `velocity_actual`
the velocity is derived from the position.

**Zero of the joint position.** With `position_actual` mapped, the joint position is the drive's
actual position. On the IRD (firmware revision 0x22, default settings) it is 0 when the drive is
powered up: it keeps its value across restarts of the interface, but not across a power cycle of
the drive (for example an emergency stop that cuts the drive's supply). Without `position_actual`,
the position is integrated from 0 at every activation. If the application needs an absolute
angle, the axis must be homed or the drive powered up at a known position.

### Limits of the drive

One IRD PDO (mapping object) holds at most **4 objects and 8 bytes** (firmware revision 0x22,
measured on the Naviwall robot: a fifth object is refused with SDO abort 0x06090011, and 10 or 11
bytes with 0x06040042). The drive has 4 RxPDOs (0x1600–0x1603) and 4 TxPDOs (0x1A00–0x1A03).

You do not have to split the mapping yourself: list the entries in one `entries` list, and the
interface fills `index`, then the next mapping objects, in order, starting a new one whenever the
next entry would not fit. The process data is the objects one after the other, so the order of
the list is kept. For example, the default TxPDO (7 bytes) plus position, current and DC-link
voltage gives:

```
Drive 'left' TxPDO 0x1A00+0x1A01+0x1A02 (17 bytes): 0x6041:00 uint16 @0 statusword | ... |
  [0x1A01] 0x6064:00 int32 @7 position_actual | 0x6078:00 int16 @11 -> drivewhl_l_joint/current_actual |
  [0x1A02] 0x6079:00 uint32 @13 -> drivewhl_l_joint/dc_link_voltage
```

The limits used for the split are the drive key `pdo_limits` (0 = no limit):

```yaml
pdo_limits:
  max_bytes: 8      # per mapping object
  max_entries: 4    # per mapping object
  max_pdos: 4       # mapping objects per direction
```

A mapping that needs more than `max_pdos` objects is reported when the configuration is loaded.
If the drive still rejects a mapping (for example an object that cannot be mapped), startup fails
and the log shows the SDO abort code (see [troubleshooting](troubleshooting.md#sdo-write-failed)).

## Exposing drive registers to ROS

Any PDO entry with an `interface` name becomes the ros2_control interface
`<joint>/<interface>`:

- **TxPDO entries** become **state interfaces**: their value (raw × `scale`) is updated every
  control cycle. `joint_state_broadcaster` publishes all of them on `/dynamic_joint_states`
  without any extra configuration.
- **RxPDO entries** become **command interfaces**: a controller writes a value, and the
  interface sends round(value / `scale`) to the drive, clamped to the range of the type. Only
  entries without a role can be commands (the interface itself writes those with a role).

They do not need to be declared in the URDF. Interface names may contain letters, digits and
`_`, must be unique per drive, and cannot be `position`, `velocity` or `effort`. An entry can have
both a role and an interface; for example, exposing the statusword lets you watch the drive state:

```yaml
tx_pdo:
  index: 0x1A00
  entries:
    - {object: "0x6041:00", type: uint16, role: statusword, interface: statusword}
    - {object: "0x606C:00", type: int32,  role: velocity_actual}
    - {object: "0x6064:00", type: int32,  role: position_actual}
    - {object: "0x6078:00", type: int16,  interface: current_actual, scale: 0.001}
    - {object: "0x6079:00", type: uint32, interface: dc_link_voltage, scale: 0.001}
```

With this mapping, `/dynamic_joint_states` contains `position`, `velocity`, `statusword`,
`current_actual` and `dc_link_voltage` for the joint. Check each object's unit in the drive
manual to choose `scale`.

To write a register from ROS, map it in `rx_pdo` with an `interface` name and use, for example, a
`forward_command_controller`:

```yaml
# controllers.yaml
controller_manager:
  ros__parameters:
    digital_outputs_controller:
      type: forward_command_controller/ForwardCommandController

digital_outputs_controller:
  ros__parameters:
    joints: [left_wheel_joint]
    interface_name: digital_outputs
```

It then accepts values on `/digital_outputs_controller/commands` (`std_msgs/msg/Float64MultiArray`).

Only expose registers that are safe to change while the drive is running.

## Modes of operation

| `mode` | CiA 402 mode | Joint command interface | Target role | Status |
|---|---|---|---|---|
| `csv` | 9, cyclic synchronous velocity | `velocity` [rad/s] | `target_velocity` | Validated on hardware |
| `csp` | 8, cyclic synchronous position | `position` [rad] | `target_position` | Implemented, not yet validated on hardware |
| `cst` | 10, cyclic synchronous torque | `effort` [Nm] | `target_torque` | Implemented, not yet validated on hardware |

The joint's state interfaces can be any of `position`, `velocity`, and `effort` (only if
`torque_actual` is mapped), plus the exposed TxPDO entries.

How each mode behaves when the drives are not (yet) enabled, after a fault, and while running:

- **csv**: zero velocity until enabled. After an enable, the reference ramps up to the command
  (`enable_ramp_motor_rad_s2`); commands above `max_motor_rpm` are clamped.
- **csp**: until enabled, and whenever commands are not applied, the reference is the drive's
  actual position, so enabling never moves the axis. When the interface is activated, the joint's
  command is initialised to its current position. The reference then moves towards the command
  at most at `max_motor_rpm`. The joint position is the drive's `position_actual`, so its zero is
  the drive's home position (set a home offset with a startup SDO if needed).
- **cst**: zero torque until enabled and after any fault.

[`config/other_modes_example.yaml`](../config/other_modes_example.yaml) shows the mapping for
`csp` and `cst`. Before testing them, set the drive's own limits (for example 0x6072 maximum
torque, 0x607F maximum profile velocity, software position limits) with startup SDOs or Optio.

## Units

The drive works at the motor shaft in its own units; the interface converts them to SI units at
the joint:

| Quantity | Drive unit | Joint unit | Conversion |
|---|---|---|---|
| Position | encoder tick | rad | ticks = rad × gear_ratio × encoder_ticks_per_rev / 2π |
| Velocity | tick/s | rad/s | same factor |
| Torque | 1/1000 of the rated torque | Nm | motor Nm = joint Nm / gear_ratio; drive unit = motor Nm × 1000 / rated_torque_nm |

`encoder_ticks_per_rev` is the number of **drive position units** per motor revolution. With the
drive's default settings this is the encoder resolution. If the drive's position factor has been
changed (CiA 402 factor group, 0x6091–0x6092), use the resulting units per revolution. The torque
conversion ignores gearbox losses.

The joint position keeps counting correctly when the drive's 32-bit position wraps around.

## Startup SDOs

Objects written once per drive at every startup, while the drive is in PRE-OP:

```yaml
startup_sdos:
  - {object: "0x6083:00", type: uint32, value: 100000}
  - {object: "0x607C:00", type: int32, value: -1500}
  - {object: "0x2012:01", type: uint16, value: 4, required: false}
```

| Key | Meaning |
|---|---|
| `object`, `type` | as in the PDO mapping |
| `value` | integer (decimal, hexadecimal or negative), must fit the type |
| `required` | default `true`: startup fails if the write fails. With `false`, a failed write is only a warning. |

Order of the writes at startup, for each drive:

1. mode of operation (0x6060), read first and written only if different,
2. the `startup_sdos`, in the order listed,
3. the PDO mapping (0x1C12/0x1C13 assignment, 0x1600/0x1A00 entries),
4. the synchronisation objects: 0x1006 (cycle period) and 0x2012:01 (sync mode).

The values are not stored permanently in the drive: they are written again at every startup.

## Fault snapshot

When a drive faults, the interface reads these objects (by SDO, from a separate thread, so the
control loop is never delayed) and logs them in one line:

```yaml
fault_snapshot:
  - {object: "0x603F:00", type: uint16, name: error_code}
  - {object: "0x1001:00", type: uint8, name: error_register}
  - {object: "0x3022:01", type: uint32, name: error_bits_1}
  - {object: "0x3022:02", type: uint32, name: error_bits_2}
  - {object: "0x3024:00", type: uint32, name: warning_code}
```

This is the default. `name` is only used in the log (default: the object address). For 0x603F the
log also gives a description of the most common EtherCAT-related error codes.

## Validation

When the hardware interface starts, the file is read and checked before anything is sent to the
drives. Problems are reported with the line number and the interface does not start. For example:

```
Configuration file: line 42: drives[1] (right): unknown key 'gear_ration' (valid keys: name, joint, ...)
Configuration error: drive 'lift': the RxPDO needs an entry with role target_position (int32)
Joint 'lift_joint': drive 'lift' has no command interface 'velocity' (available: position).
```

The checks cover:
- **Reading the file:** unknown keys, wrong types and values that are out of range.
- **Drives:** duplicate names, joints and slave positions.
- **PDO mapping:** missing, duplicated or mistyped roles, and entries with no use.
- **Exposed registers:** invalid or duplicate interface names.
- **The URDF:** joint interfaces that do not match the drive's mode or mapping.
