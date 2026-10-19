# Troubleshooting

Every problem the interface detects is reported in the log of the controller manager, with the
prefix `[IrdEthercatSystem]`. This page lists the messages by startup step, with their cause
and what to do.

**How to read a failed start.** Look for the **first** `ERROR` or `FATAL` line from
`IrdEthercatSystem`: that is the cause. When the hardware cannot start, Humble's controller
manager exits with `terminate called after throwing an instance of 'std::runtime_error'`. That
line is only a consequence.

**Contents**

1. [Configuration](#1-configuration)
2. [Permissions and libraries](#2-permissions-and-libraries)
3. [Opening the bus](#3-opening-the-bus)
4. [Configuring the drives (PRE-OP)](#4-configuring-the-drives-pre-op)
5. [SAFE-OP and synchronisation](#5-safe-op-and-synchronisation)
6. [OPERATIONAL and enabling](#6-operational-and-enabling)
7. [Runtime fault: error 49 or 50](#runtime-fault-error-49-or-50)
8. [Other runtime messages](#8-other-runtime-messages)
9. [Reference tables](#9-reference-tables)

---

## 1. Configuration

These are reported before anything is sent to the drives.

| Message | Cause and fix |
|---|---|
| `Configuration file: cannot open configuration file '...'` | The path in the URDF parameter `config_file` is wrong. Use an absolute path, e.g. `$(find my_robot)/config/drives.yaml` in xacro. |
| `Configuration file: ... YAML syntax error: ...` | Invalid YAML (indentation, missing quotes). The message gives line and column. Object addresses must be quoted: `"0x6040:00"`. |
| `Configuration file: line N: ...: unknown key 'x' (valid keys: ...)` | Misspelt or misplaced key. The message lists the keys allowed there. |
| `Configuration file: line N: ...: '...' is not ...` / `... is out of range` | Wrong value type or range. See the [configuration reference](configuration.md). |
| `Configuration error: drive 'x': the RxPDO/TxPDO needs an entry with role ...` | A role required by the drive's mode is missing from the PDO mapping. See [roles](configuration.md#pdo-mapping). |
| `Configuration error: ... has neither a role nor an interface name` | A mapped object would not be used: give it a `role` or an `interface`, or remove it. |
| `Configuration error: ... used more than once` | Duplicate drive name, joint, slave position or interface name. |
| `Joint 'j' of drive 'x' is not declared in this <ros2_control> block.` | Add a `<joint name="j">` to the `<ros2_control>` block of the URDF. |
| `Joint 'j' is declared in the <ros2_control> block but no drive drives it.` | Remove the joint from the block, or add a drive for it in the configuration. |
| `Joint 'j' must declare the command interface 'velocity'` / `... has no command/state interface '...' (available: ...)` | The joint's interfaces in the URDF do not match the drive's mode or mapping. The message lists what is available. |
| `URDF parameter 'x' is ignored: the configuration comes from config_file.` | Warning: old URDF parameters left next to `config_file`. Remove them. |

## 2. Permissions and libraries

| Message | Cause and fix |
|---|---|
| `Failed to initialize SOEM on eno1 (does the interface exist, and does the process have CAP_NET_RAW? ...)` | Either the network interface does not exist (check `ip link` and `bus.interface` / `ethercat_interface`), or `ros2_control_node` lacks the `cap_net_raw` capability. See [getting started, step 3](getting_started.md#3-give-ros2_control-the-permissions-it-needs). |
| `ros2_control_node: error while loading shared libraries: ... cannot open shared object file` | The executable has capabilities, so `LD_LIBRARY_PATH` is ignored. Register the library directories in `/etc/ld.so.conf.d/` as described in [step 3](getting_started.md#3-give-ros2_control-the-permissions-it-needs). |
| `Processdata thread could not enable SCHED_FIFO priority 80: Operation not permitted` | Warning: missing `cap_sys_nice`. The thread runs without real-time priority, so timing faults (error 49/50) become likely. |
| `mlockall failed (...)` | Warning: missing `cap_ipc_lock` (or a low `ulimit -l`). Page faults can delay the real-time thread. |

After a ROS update, re-run `setcap`: the update replaces the executable and its capabilities.

## 3. Opening the bus

| Message | Cause and fix |
|---|---|
| `No EtherCAT slaves found on eno1.` | Nothing answers on that port: wrong port, cable not connected to the drive's **IN** connector, or drives not powered (the EtherCAT part of the IRD needs its logic supply). |
| `Found N slave(s) on the bus, but the configuration uses slave position M.` | A `slave_position` is higher than the number of devices found. Check the chain (a broken cable or an unpowered drive cuts off the drives behind it) and the positions. The list of devices found is logged just before (`Slave 1: ...`). |
| `Drive 'x' (slave N): vendor_id is ..., configuration expects ...` (or `product_code`) | The device at that position is not the expected one: drives cabled in a different order, or another device on the bus. |
| More devices found than drives configured | Every device on the chain must be listed in the configuration. To test fewer drives, disconnect the others. If something fails with several drives, test each drive alone on the cable to find the faulty drive or cable segment. |

## 4. Configuring the drives (PRE-OP)

| Message | Cause and fix |
|---|---|
| `Slave N failed to reach PRE-OP. State=0x..` | The drive does not start its mailbox. Power-cycle the drive; if it persists, check its EtherCAT firmware with Infranor. |
| `Slave N: Failed to set the mode of operation to M` | The drive rejected object 0x6060. Check that the mode is supported by the drive's firmware and configuration. |
| `Drive 'x': required SDO 0xIIII:SS could not be written.` | See [SDO write failed](#sdo-write-failed) just below. |
| `Process image needs N bytes, more than the 4096 reserved.` | The PDO mappings of all drives together are too large. Map fewer objects. |
| `Drive 'x' (slave N) PDO size mismatch! RxPDO expected=A actual=B, ...` | The drive did not take the mapping as configured (an entry rejected without an error, or a `type` that does not match the object's real size). Check each object's data type in the drive manual. |

### SDO write failed

```
SDO write (uint32) failed after retries: slave=1 idx=0x1A00 sub=0x05 value=1618477072
sdo_write SOEM error: slave=1 type=1 idx=0x1A00 sub=0x05 abort=0x06040042
```

The `abort` value is the drive's reason (CiA 301 SDO abort code):

| Abort code | Meaning | Typical cause here |
|---|---|---|
| `0x06020000` | Object does not exist | wrong `object` address in `startup_sdos` or the PDO mapping |
| `0x06090011` | Subindex does not exist | wrong subindex |
| `0x06010002` | Attempt to write a read-only object | the object cannot be written (e.g. an actual value in `startup_sdos`) |
| `0x06070010` | Data type / length does not match | wrong `type` |
| `0x06090030`, `0x06090031`, `0x06090032` | Value out of range / too high / too low | wrong `value` |
| `0x06040041` | Object cannot be mapped to a PDO | the object is not PDO-mappable: remove it from the mapping |
| `0x06040042` | Mapping exceeds the PDO length | too many objects in one PDO: map fewer |
| `0x06040043` | General parameter incompatibility | the value conflicts with another setting of the drive |
| `0x08000022` | Not possible in the present device state | the object cannot be written in PRE-OP / with the drive in its current state |
| `0x05040000` | SDO protocol timed out | no answer from the drive: wiring or drive firmware |

If there is no `abort` line, the drive did not answer at all (timeout): check the cabling and
power. A write marked `required: false` only produces a warning
(`failed to write ... (not required, continuing)`).

## 5. SAFE-OP and synchronisation

| Message | Cause and fix |
|---|---|
| `Slave N failed to reach SAFE-OP. State=0x..` | The drive refused the configuration. The state value with bit 0x10 set means an error is pending: see the [AL status codes](#al-status-codes) (`AL=0x....` in the following log lines or in the fault snapshot). Usually a PDO mapping or distributed-clocks setting the drive does not accept. |
| `Slave N does not report distributed clocks capability.` | The device does not support DC: set `bus.distributed_clocks: false` (not recommended for IRD drives) or check that the right device is at that position. |
| `Slave N fell back to SoftSync (0x2012:01=2, ...). HardSync was requested in PRE-OP but did not latch at the PS transition.` | Warning: the drive runs in SoftSync although HardSync was configured. It still works, with looser synchronisation. Report it with the full startup log. |
| `Slave N sync settings unexpected (observed 0x1006:00=..., 0x2012:01=...)` | Warning: the drive reports a cycle period or sync mode different from the configuration. |
| `DC phase not settled within 3000 ms (frame at X us, SYNC0 at Y us); continuing, an error 50 right after enable is possible.` | Warning: the master's timing did not stabilise. Usually caused by the PC (no real-time priority, heavy load) or by traffic on the EtherCAT port. See the checklist [below](#runtime-fault-error-49-or-50). |

## 6. OPERATIONAL and enabling

| Message | Cause and fix |
|---|---|
| `Drive 'x' (slave N) failed to reach OPERATIONAL. State=0x.., AL=0x....` | See the [AL status code](#al-status-codes). Codes 0x001A/0x002C or 0x0030–0x0036 point to synchronisation problems, 0x001B to missing process data. |
| `Drive 'x' (slave N) starts activation in FAULT (SW=0x....). Clearing latched fault via SDO reset.` followed by `fault latched before activation ...` | Warning, normal after a session that was killed without a clean shutdown (e.g. a crash or `kill -9`): the drives latched error 49/50 when the frames stopped. The fault is reset automatically. |
| `Drive 'x' (slave N): fault reset did not clear FAULT (SW=0x....).` | The drive has a real fault that a reset cannot clear (the fault snapshot in the same log says which). Fix its cause (power stage, motor, encoder, limits...), using Optio if needed. |
| `Aborting PDO enable sequence after repeated drive faults. Last TPDO statuswords: ...` | A drive went to FAULT while being enabled. The fault snapshots that follow give the error code. |
| `Timed out waiting for PDO enable sequence. Final TPDO statuswords: ...` | A drive did not reach *Operation Enabled* in `enable_timeout_ms`. Decode the statuswords with the [table below](#cia-402-statusword). Typical causes: STO / enable input not active, DC bus not powered (the drive stays in *Switch On Disabled* or *Ready to Switch On*). |

## Runtime fault: error 49 or 50

```
Runtime drive fault detected (SW left=0x0218 right=0x0237 turret=0x0237). Auto-recovery attempt 1/5 after cooldown.
Drive 'left' (slave 1) fault snapshot: SW=0x0218  AL=0x0000 (no AL error)  error_code(0x603F:00)=0x8782 (fieldbus IP reference underflow/overflow (error 50: ...)) ...
EtherCAT timing before the fault (last 128 cycles, ...):
  cycle  -12: wake late   830 us, exchange   95 us, DC phase  1990 us, ...  <--
Drives recovered from runtime fault: Operation Enabled restored; ...
```

**What it is.** The drives check the timing of the frames:
- **Error 50** (0x8782, *IP reference underflow/overflow*): a frame arrived too late, less than
  100 µs before SYNC0, or was missing.
- **Error 49** (0x8780, *SYNC cycle error*): the cycle period seen by the drive was out of
  tolerance (master jitter).

Both clear once the frames are regular again, so the interface recovers automatically (see
[How it works](how_it_works.md#7-runtime-faults-and-automatic-recovery)). An occasional recovery
is harmless. Frequent ones mean the timing has to be improved.

**Find the cause in the timing dump** (fields explained in
[How it works](how_it_works.md#9-diagnostics-in-the-log)):

| What the marked lines show | Cause | Fix |
|---|---|---|
| `wake late` of hundreds of µs | The PC woke the real-time thread late | PREEMPT_RT kernel; check `SCHED_FIFO` and `mlockall` warnings; isolate a CPU core; reduce CPU load |
| `exchange` long, `wake late` small | The frame was delayed on the network side | Use a dedicated port for EtherCAT; disable interrupt coalescing on it (`ethtool -C eno1 rx-usecs 0`); disable energy-saving features of the NIC |
| `DC phase` close to or past `sync0_shift_us`, both above small | Frames arrive late in the cycle | Increase `bus.sync0_shift_us` (up to half the cycle) |
| `WKC` below the expected value | A frame was lost | Check cables and connectors; look at the ESC error counters (below) |
| Nothing marked | Not a timing problem on the master side | Check the fault snapshot error code; check the ESC counters and cabling |

**ESC error counters** (logged with every fault and every 10 s when they change): increasing
`invalid-frame` or `rx-err` on a drive's port point to a bad cable, connector or electrical noise
on that segment (motor cables routed next to the EtherCAT cable, missing shield grounding).

**Last resort.** If HardSync faults persist and the timing cannot be improved, `bus.sync_mode:
soft_sync` tolerates more jitter (with looser synchronisation).

**When the recovery gives up:**

| Message | Meaning |
|---|---|
| `Fault reset did not clear within N cycles. Retry k/5.` | The reset pulse did not clear the fault; another attempt follows after the cooldown. |
| `Drive fault is persistent after 5 reset attempts. Drives stay disabled.` | The fault is not a transient communication error. The fault snapshot gives the error code. Restart the interface after fixing the cause. |
| `Runtime drive fault (SW ...): recovery attempts exhausted (5). Drives stay disabled.` | Too many faults in a short time (the attempts refill after `recovery_budget_refill_ms` without faults). Improve the timing as above. |

## 8. Other runtime messages

| Message | Meaning |
|---|---|
| `SAFETY: drives stopped (force_zero); commands are ignored until they are enabled again.` | The drives are stopped (fault recovery or deactivation). Commands from the controllers are not applied meanwhile. |
| `Drive 'x' motor velocity ... exceeds ... (N rpm). Clamping to ...` | The command is above `max_motor_rpm` (csv). Logged at most once per second. |
| `Drive 'x' received a non-finite ... command (nan); ...` | A controller wrote NaN or infinity. It is replaced by 0 (csv, cst) or by the current position (csp). |
| `read(): WKC below expectation: got A expected at least B` | Frames come back with fewer drives having processed them: a drive left OPERATIONAL or a link is intermittent. Check the cabling and the drives' state. |
| `Drive 'x' (slave N) ESC error counters: ...` | Logged when a drive's EtherCAT error counters change (see above). A single change after power-on is normal; steadily increasing values are not. |
| `Not all drives confirmed SAFE-OP before stopping the process data.` | Warning at shutdown: a drive did not confirm SAFE-OP in time. It may report error 49/50 at the next start, which is then cleared automatically. |

## 9. Reference tables

### IRD error codes

Error code (0x603F) values related to the fieldbus, as described in the log:

| 0x603F | IRD error | Meaning |
|---|---|---|
| `0x8780` | 49 | Fieldbus SYNC cycle error: master cycle jitter or period out of tolerance |
| `0x8782` | 50 | Fieldbus IP reference underflow/overflow: reference frame missing or less than 100 µs before SYNC |
| `0x8130` | 51 | Fieldbus guarding error |
| `0x8181` | – | Extension error / fieldbus watchdog error |

Bit 13 (0x00002000) of the IRD error bits object 0x3022 is error 46, *RPDO watchdog overflow* (the
drive stopped receiving process data for longer than its RPDO watchdog time, 0x2321). For all
other codes, see the error table of the IRD user manual.

### AL status codes

Reported as `AL=0x....` when a drive refuses an EtherCAT state change (ETG.1000 codes):

| Code | Meaning |
|---|---|
| `0x0000` | No error |
| `0x0011` | Invalid requested state change |
| `0x0016` | Invalid mailbox configuration |
| `0x0017` | Invalid sync manager configuration |
| `0x001A` | Synchronization error |
| `0x001B` | Sync manager watchdog (process data not received) |
| `0x001D` / `0x001E` | Invalid output / input configuration |
| `0x0024` / `0x0025` | Invalid input / output mapping |
| `0x0026` | Inconsistent settings (on the IRD: inconsistent sync / DC settings) |
| `0x002C` | Fatal sync error |
| `0x0030` | Invalid DC SYNC configuration |
| `0x0034` | DC sync timeout |
| `0x0035` / `0x0036` | Invalid DC cycle time / invalid SYNC0 cycle time |

### CiA 402 statusword

Decoding the statuswords (`SW=0x....`) printed in the log:

| State | Statusword (bits shown in the mask) | Example |
|---|---|---|
| Not Ready to Switch On | `xxxx xxxx x0xx 0000` | `0x0200` |
| Switch On Disabled | `xxxx xxxx x1xx 0000` | `0x0240`, `0x0260` |
| Ready to Switch On | `xxxx xxxx x01x 0001` | `0x0221`, `0x0231` |
| Switched On | `xxxx xxxx x01x 0011` | `0x0223`, `0x0233` |
| Operation Enabled | `xxxx xxxx x01x 0111` | `0x0227`, `0x0237` |
| Quick Stop Active | `xxxx xxxx x00x 0111` | `0x0207` |
| Fault Reaction Active | `xxxx xxxx x0xx 1111` | `0x020F` |
| Fault | `xxxx xxxx x0xx 1000` | `0x0208`, `0x0218` |

Bit 4 (0x0010) is *voltage enabled*, bit 9 (0x0200) *remote*, bit 7 (0x0080) *warning*.
