# How it works

This page explains what the hardware interface does with the drives, and why, so that its
behaviour and its log messages can be understood without reading the code.

**Contents**

1. [Overview](#1-overview)
2. [Startup](#2-startup)
3. [Every cycle](#3-every-cycle)
4. [Synchronisation](#4-synchronisation)
5. [Enabling the drives (CiA 402)](#5-enabling-the-drives-cia-402)
6. [Commands and references](#6-commands-and-references)
7. [Runtime faults and automatic recovery](#7-runtime-faults-and-automatic-recovery)
8. [Stopping](#8-stopping)
9. [Diagnostics in the log](#9-diagnostics-in-the-log)
10. [Code map](#10-code-map)

---

## 1. Overview

The interface is a ros2_control *system* plugin. It runs inside the controller manager and owns
the EtherCAT bus through the SOEM master library. Three threads are involved:

```
 ros2_control update loop          process-data thread              diagnostics thread
 (controller manager, e.g. 100 Hz) (real-time, SCHED_FIFO, 250 Hz)  (normal priority)
 ───────────────────────────────   ──────────────────────────────   ──────────────────────
 read():  joint states  ◄──────┐   one EtherCAT frame per cycle      SDO fault snapshots
 write(): joint commands ─────┐│   distributed-clock sync            ESC error counters
                              ││   CiA 402 state machine             timing dump on faults
                              │└── fault detection and recovery ───►  (on request)
                              └──► (atomics, no locks, no bus access)
```

- **read()/write() never touch the bus.** They exchange values with the process-data thread
  through atomic variables, so a slow controller cannot delay a frame, and a bus problem cannot
  block the controllers.
- **The process-data thread** sends one frame per cycle (`bus.cycle_time_us`, 4 ms by default),
  at a time locked to the drives' clock. It is the only place where the drives are enabled,
  disabled and reset during operation, always through the cyclic controlword (no SDO).
- **The diagnostics thread** does the slow work: SDO reads take milliseconds and would make the
  process-data thread miss its cycle. It also does the logging that the process-data thread
  requests.

## 2. Startup

Everything happens when ros2_control *activates* the hardware (normally at launch). The bus is
opened from scratch at every activation, so a deactivate/activate cycle fully re-initialises the
drives.

| Step | EtherCAT state | What happens | Log |
|---|---|---|---|
| 1 | INIT → PRE-OP | Open the network interface, scan the bus, list every device, check positions (and identities if configured). | `Found N slave(s).`, `Slave 1: ...` |
| 2 | PRE-OP | Configure every drive by SDO: mode of operation, `startup_sdos`, PDO mapping, cycle period (0x1006) and sync mode (0x2012). | `Configuring drive 'x'...` |
| 3 | PRE-OP | Map the process image; start the distributed clocks and SYNC0 on every drive. | `Configured DC SYNC0: ...` |
| 4 | → SAFE-OP | Check that the mapped sizes match the configuration, then request SAFE-OP. The drives latch their synchronisation mode here. | `Process data mapping validated`, `All slaves reached SAFE-OP.` |
| 5 | SAFE-OP | Exchange frames for `startup_settle_ms` with all outputs at zero, so the master's timing locks to the drives' clock. Report the sync mode each drive latched. | `HardSync established` |
| 6 | SAFE-OP | Clear any fault left by a previous session (one SDO fault reset per faulted drive). | `fault latched before activation` (warning) |
| 7 | SAFE-OP | Start the process-data and diagnostics threads. Wait until the frames stay within `dc_lock_tolerance_us` of their target for `dc_lock_ms`. | `DC phase locked` |
| 8 | → OPERATIONAL | Request OPERATIONAL. From now on the drives check every frame. | `All slaves reached OPERATIONAL state.` |
| 9 | OPERATIONAL | Run the CiA 402 enable sequence through the cyclic controlword. | `All drives enabled successfully.` |

Any failure stops the startup with an error explaining which step and which drive failed, and
the bus is closed again in a safe order (see [troubleshooting](troubleshooting.md)).

**Why this order.** The IRD selects its synchronisation mode once, at the PRE-OP → SAFE-OP
transition. So 0x1006/0x2012 must be written and SYNC0 must be running **before** SAFE-OP is
requested, otherwise the drive falls back to SoftSync and ignores later changes. The interface
therefore stops SOEM from requesting SAFE-OP by itself after mapping, and requests it explicitly
in step 4. Steps 5–7 make sure the frames already arrive at a stable time before the drives
start monitoring them in OPERATIONAL. Enabling while the timing is still settling was a source of
error 50 right after startup.

## 3. Every cycle

The process-data thread repeats, every `cycle_time_us`:

1. **Sleep** until the next cycle deadline (absolute time, corrected by the clock
   synchronisation, see below).
2. **Stage the outputs** of every drive: the controlword from the enable/fault logic, the mode
   of operation (if mapped), the reference of its mode (velocity, position or torque) and the
   values of the extra command interfaces.
3. **Exchange** one frame with all drives (send outputs, receive inputs).
4. **Synchronise**: measure when the frame passed the drives (from the distributed clock) and
   correct the next deadline.
5. **Publish** the drives' feedback (statusword, position, velocity, torque, extra registers)
   for `read()`.
6. **Check for faults** and run the enable or recovery logic.

The controller manager's update rate is independent of the EtherCAT cycle. `write()` only stores
the latest command, and the next frame sends it.

## 4. Synchronisation

**Distributed clocks.** The drives share a clock (the EtherCAT *distributed clock*, DC). Each
drive generates a **SYNC0** pulse every cycle from that clock, and latches the references of
the last received frame on SYNC0. For this to work, a frame must arrive in every cycle, and
always before SYNC0.

**The master follows the drives.** A PC cannot keep perfect time with the drives' clock, so the
interface steers its own cycle: every received frame carries the DC time at which it passed the
first drive. A PI controller adjusts the next sleep so that frames pass the drives at a fixed
point of the DC cycle, the *cycle boundary*. SYNC0 fires `sync0_shift_us` later. At startup, the
first measured offset is removed in one step instead of waiting for the PI loop.

```
DC time ──┬──────────────────────────────┬──────────────────────────────┬──►
          cycle boundary                 cycle boundary                 
          │◄──── sync0_shift_us ────►│   │                              
          frame passes the drives    SYNC0: drives latch the references 
```

**`sync0_shift_us` is the safety margin** for late frames. The IRD needs the frame at least
100 µs before SYNC0 (otherwise error 50), and Infranor recommends 200 µs at 4 ms. A frame can be
late because of scheduling jitter on the PC, or because other traffic delays it on the network
port. The default 2000 µs (half the cycle) tolerates up to about 1.9 ms of delay. That is the
most a cycle allows: more would let a frame overtake the previous SYNC0. The cost is that
references take effect 2 ms after they are sent, which is negligible for most applications.
On a dedicated network port with a real-time kernel, a smaller value (e.g. 200–500 µs) reduces
that delay.

**HardSync and SoftSync** (`bus.sync_mode`, IRD object 0x2012:01):

| | HardSync (4) | SoftSync (2) |
|---|---|---|
| How the drive follows the master | locked to SYNC0 | loosely, from SYNC0 and frame arrival |
| Tolerance to master jitter | strict (error 50 if a frame is late) | more tolerant (error 49 on large jitter) |
| When to use | default; needs a reasonably stable master | if HardSync faults persist and the timing cannot be improved |

On the robot where this interface was developed (EtherCAT sharing its port with lidar traffic),
SoftSync faulted 15–25 times more often than HardSync with a 2 ms shift.

## 5. Enabling the drives (CiA 402)

The drives are enabled through the cyclic controlword, one transition per cycle:

```
Switch On Disabled ──0x0006 Shutdown──► Ready To Switch On ──0x0007 Switch On──► Switched On
      ──0x000F Enable Operation──► Operation Enabled
```

When every drive has been *Operation Enabled* for `enable_stable_ms`, the interface starts
applying the controllers' commands. Activation fails if this does not happen within
`enable_timeout_ms`, or if a drive stays in FAULT for `enable_fault_abort_ms`.

A FAULT is never reset by repeating the reset command (0x0080) continuously, because that makes
the motor brake chatter. Faults are reset only:
- by one SDO reset at startup (step 6 above), for a fault left by a previous session, and
- by a time-limited reset pulse during the automatic recovery (section 7).

## 6. Commands and references

`write()` converts each joint command to drive units (see
[Units](configuration.md#units)) and the process-data thread decides what reference is sent:

| Mode | While the drives are not enabled, or after a fault | While running |
|---|---|---|
| csv | 0 | the command, clamped to `max_motor_rpm`; after an enable it **ramps up** at `enable_ramp_motor_rad_s2` until it reaches the command (slowing down and stopping are immediate) |
| csp | the drive's **actual position** (the axis holds still) | moves towards the command, at most at `max_motor_rpm` |
| cst | 0 | the command |

The velocity ramp exists because a controller may still hold an old command when the drives
come back after a fault: without it, the reference would jump from 0 to full speed in one cycle.
That would be a jerk for the machine, and a known trigger of error 50. In csp the same problem
is avoided by starting from the actual position.

A non-finite command (NaN or infinite) is logged as an error and replaced by 0 (csv, cst) or by
the current position (csp).

## 7. Runtime faults and automatic recovery

The IRD fieldbus errors 49 and 50 (0x8780, 0x8782) are caused by late or irregular frames. They
are transient: they clear as soon as the frames are regular again. Restarting the whole
application for one of them would be excessive, so the interface recovers by itself, within
limits:

```
fault on any drive
 │ all drives: zero reference, references no longer applied
 │ diagnostics thread: fault snapshot + ESC counters + timing of the last 128 cycles
 ▼
cooldown (recovery_cooldown_ms)          healthy drives stay enabled at zero reference,
 │                                       faulted drives get controlword 0x0000
 ▼
reset pulse (≤ reset_pulse_ms)           controlword 0x0080 to the faulted drive(s)
 ├─ fault cleared ──► enable sequence ──► enable_stable_ms ──► running again (with ramp)
 └─ not cleared ────► next attempt (back to cooldown), until max_recovery_attempts
                       └─ attempts used up ──► drives stay disabled until the interface is restarted
```

- **All drives react together:** one faulted drive stops all of them, and they are re-enabled
  together. On a mobile base, one wheel must never keep driving alone.
- **The attempts are counted per incident.** After `recovery_budget_refill_ms` without faults
  the count restarts, so occasional faults over a long session never leave the machine disabled.
- **Faults that do not clear with a reset** (anything other than a communication error) use up
  the attempts quickly and leave the drives disabled, with the fault snapshot in the log.
- The controllers keep running during a recovery; their commands are ignored until the drives
  are back, and a warning in `write()` says so.

## 8. Stopping

On deactivation or shutdown (for example Ctrl+C on the launch), the order matters:

1. Stop the motors and disable the drives **through the cyclic PDO** (controlword 0x0000), while
   frames are still being sent.
2. Take the drives to **SAFE-OP**: they stop monitoring the frame timing.
3. Only then stop the process-data thread, take the drives to INIT and close the bus.

If the frames stopped while the drives were still in OPERATIONAL, the drives would latch error
49/50, and the next startup would begin with a fault to clear.

## 9. Diagnostics in the log

**Fault snapshot.** When a drive faults, one line with its statusword, the EtherCAT AL status
code and the `fault_snapshot` objects:

```
Drive 'left' (slave 1) fault snapshot: SW=0x0218  AL=0x0000 (no AL error)  error_code(0x603F:00)=0x8782
  (fieldbus IP reference underflow/overflow (error 50: ...))  error_register(0x1001:00)=0x11 ...
```

**Timing of the last cycles.** With every runtime fault, the cycles before the fault that could
explain it are printed (plus the last 8 for context), oldest first:

```
EtherCAT timing before the fault (last 128 cycles, oldest first; expected WKC 9, SYNC0 at DC phase 2000 us):
  cycle  -37: wake late   612 us, exchange   143 us, DC phase  1952 us, toff    1 us (I    0), WKC 9  <--
```

| Field | Meaning | Points to |
|---|---|---|
| wake late | how late the thread woke after its deadline | the PC (scheduling, CPU load, no RT kernel) |
| exchange | time to send and receive the frame | the network (port shared, interrupt moderation) |
| DC phase | when the frame passed the drives, from the cycle boundary; SYNC0 is at `sync0_shift_us` | close to or past SYNC0: the frame was late for the drives |
| toff (I) | correction of the cycle by the synchronisation loop (integral part) | large values: the PC clock drifts from the drives |
| WKC | working counter: how many drives processed the frame | below the expected value: a frame was lost or not processed |

Marked lines (`<--`) are the anomalous ones: WKC too low, woke > 200 µs late, DC phase within
100 µs of SYNC0 (or past it), or exchange > 1 ms.

**ESC error counters.** Every 10 s, and with every fault, the interface reads each drive's
EtherCAT controller error counters (registers 0x0300–0x0313) and logs them when they changed:

```
Drive 'left' (slave 1) ESC error counters: port0 invalid-frame=0 rx-err=0 fwd-rx-err=0 lost-link=0 | port1 ... | PU-err=0 PDI-err=0
```

They count since the drive was powered on. Increasing `invalid-frame` / `rx-err` counters point
to the cabling or electrical noise (EMI) between that drive and its neighbour, rather than to
the PC.

**Working counter.** If frames keep coming back with a lower working counter than expected,
`read()` logs `WKC below expectation` periodically: a drive is not processing the process data
(cabling, or a drive left OPERATIONAL).

## 10. Code map

| File | Contents |
|---|---|
| `config.hpp/.cpp` | Configuration structures, built-in defaults, validation |
| `config_loader.hpp/.cpp` | Reading the YAML file |
| `pdo.hpp/.cpp` | PDO layout and little-endian encoding of values |
| `startup_plan.hpp/.cpp` | The SDO writes that configure a drive in PRE-OP |
| `cia402.hpp` | CiA 402 statusword and controlword helpers |
| `dc_sync.hpp` | The PI loop that follows the drives' clock |
| `cyclic_controller.hpp/.cpp` | Per-cycle logic: enable sequence, fault recovery, references per mode |
| `ethercat_master.hpp/.cpp` | Thin wrapper around SOEM (bus, SDO, states, process data, registers) |
| `ird_ethercat_system.hpp/.cpp` | The ros2_control plugin: lifecycle, threads, read/write, diagnostics |

Everything above `ethercat_master` is independent of ROS and SOEM and is covered by unit tests
(`test/`), which need no hardware. One of them replays the per-cycle logic of the version
validated on hardware next to the current one for 600,000 cycles with random faults and checks
that both produce the same controlwords and references.
