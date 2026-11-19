# Getting started

This guide takes you from a PC with ROS 2 to one Infranor IRD drive turning its motor under
ros2_control. It takes about an hour. When it works, adding more drives only means adding them to
the configuration file.

**Contents**

1. [What you need](#1-what-you-need)
2. [Install the software](#2-install-the-software)
3. [Give ros2_control the permissions it needs](#3-give-ros2_control-the-permissions-it-needs)
4. [Find your drives on the bus](#4-find-your-drives-on-the-bus)
5. [Write the configuration file](#5-write-the-configuration-file)
6. [First motion with the example launch file](#6-first-motion-with-the-example-launch-file)
7. [Use it in your own robot](#7-use-it-in-your-own-robot)

---

## 1. What you need

**Computer**

- Ubuntu 22.04 with ROS 2 Humble.
- A Linux kernel with the PREEMPT_RT patch is strongly recommended. The interface sends one
  EtherCAT frame every cycle (4 ms by default) from a real-time thread, and the drives check
  the timing of those frames: late frames show up as IRD errors 49/50 (see
  [troubleshooting](troubleshooting.md#runtime-fault-error-49-or-50)).
- **An Ethernet port used only for EtherCAT.** Do not share it with other network traffic
  (cameras, lidars, a LAN): queued traffic delays the EtherCAT frames. It needs no IP address.
  List the ports with `ip link`; this guide calls the port `eno1`.

**Drives**

- One or more IRD drives with their motors, powered.
- Each drive already commissioned for its motor with Infranor's **Optio** software (motor and
  encoder parameters, current and velocity loops). The EtherCAT communication settings (cycle
  period, PDO mapping, synchronisation) do **not** need to be set in Optio: this interface writes
  them to the drives at every startup.
- An EtherCAT cable from the PC port to the IN port of the first drive, and from each drive's OUT
  port to the next drive's IN port.

> **Safety.** For the first tests, make sure the motor can turn freely (no load, or a load that
> can safely move), and keep the drive's emergency stop / STO within reach.

## 2. Install the software

### ROS 2 packages

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro \
                 ros-humble-robot-state-publisher
```

### SOEM (the EtherCAT master library)

The interface uses [SOEM](https://github.com/OpenEtherCATsociety/SOEM) **version 2.x** (tested with
v2.0.0). SOEM 1.x has a different API and does not work. You can install it on the system or let
the build download it. Installing it is recommended on a robot, because the build then needs no
internet access.

```bash
git clone --branch v2.0.0 https://github.com/OpenEtherCATsociety/SOEM.git ~/SOEM
cmake -S ~/SOEM -B ~/SOEM/build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build ~/SOEM/build -j
sudo cmake --install ~/SOEM/build      # installs to /usr/local
```

If SOEM is not installed, the build downloads and compiles v2.0.0 by itself. To forbid that (and
get an error instead), build with `--cmake-args -DIRD_FETCH_SOEM=OFF`.

### This package

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/DISCOVER-Horizon-Europe-Project/ird_ethercat_hardware_interface.git
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ird_ethercat_hardware_interface --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

Optionally, check the build with the unit tests (they need no hardware):

```bash
colcon test --packages-select ird_ethercat_hardware_interface && colcon test-result --verbose
```

## 3. Give ros2_control the permissions it needs

The EtherCAT master runs inside `ros2_control_node`, which needs three Linux capabilities:

| Capability | Needed for | If missing, the log says |
|---|---|---|
| `cap_net_raw` | sending raw EtherCAT frames | `Failed to initialize SOEM on eno1` |
| `cap_sys_nice` | real-time priority of the process-data thread | `could not enable SCHED_FIFO` |
| `cap_ipc_lock` | locking memory (no page faults in the real-time thread) | `mlockall failed` |

Without `cap_net_raw` nothing works. The other two only produce warnings, but without them
timing faults are much more likely.

You can run as root, but granting the capabilities to the executable is cleaner:

```bash
sudo setcap cap_net_raw,cap_sys_nice,cap_ipc_lock+ep /opt/ros/humble/lib/controller_manager/ros2_control_node
```

Linux ignores `LD_LIBRARY_PATH` for an executable that has capabilities, and `ros2_control_node`
finds its libraries only through it. Register the library directories of your ROS environment
once, after sourcing it:

```bash
source ~/ros2_ws/install/setup.bash
echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '^$' | sudo tee /etc/ld.so.conf.d/ros2_control.conf
sudo ldconfig
```

Repeat both commands when ROS packages are updated (an update replaces the executable and drops
its capabilities) or when you add workspace packages whose libraries the controllers load.

## 4. Find your drives on the bus

Each drive is identified by its **slave position**: 1 is the drive connected to the PC, 2 the next
one in the chain, and so on. When the interface starts it logs every device it finds, for example:

```
[IrdEthercatSystem]: Found 3 slave(s).
[IrdEthercatSystem]: Slave 1: '<device name>' vendor_id=0x0000abcd product_code=0x00001234 revision=0x00000001
```

(the name and numbers above are placeholders; your drives report their own). You will see this
list the first time you run the example in step 6, even if the slave positions in the configuration
are not right yet (the file itself must be valid). Use it to check the cabling order, and
optionally copy `vendor_id` / `product_code` into the configuration so that a drive connected in
the wrong position is detected.

> **List every drive on the bus.** The configuration must contain every device connected to the
> EtherCAT chain. To test one drive on its own (for example to isolate a problem), disconnect it
> from the others and connect it alone to the PC.

## 5. Write the configuration file

Everything about the drives is in one YAML file. Copy the minimal example and edit it:

```bash
mkdir -p ~/ird_test
cp ~/ros2_ws/install/ird_ethercat_hardware_interface/share/ird_ethercat_hardware_interface/config/minimal_example.yaml ~/ird_test/drives.yaml
```

```yaml
bus:
  interface: eno1                # your EtherCAT port

drive_defaults:
  encoder_ticks_per_rev: 2048    # encoder resolution of your motor, in ticks per revolution

drives:
  - name: axis1
    joint: axis1_joint           # keep this name for the example launch file
    slave_position: 1
    gear_ratio: 10.0             # motor revolutions per joint revolution (1.0 without gearbox)
```

Two values matter for the units:

- `encoder_ticks_per_rev`: the drive's position unit is one encoder tick and its velocity unit one
  tick per second, so this value converts between rad and drive units. Use the resolution
  configured in the drive.
- `gear_ratio`: joint commands and states are at the joint, after the gearbox.

Everything else (cycle time, synchronisation, PDO mapping, speed limit, fault handling) has a
default that works for an IRD drive in velocity mode. All keys are described in the
[configuration reference](configuration.md).

## 6. First motion with the example launch file

The package contains a launch file with a one-joint robot, a `joint_state_broadcaster` and a
velocity controller (`forward_command_controller`):

```bash
ros2 launch ird_ethercat_hardware_interface example.launch.py config_file:=$HOME/ird_test/drives.yaml
```

A successful start ends with these lines (other lines omitted):

```
[IrdEthercatSystem]: Configuration loaded from /home/user/ird_test/drives.yaml
[IrdEthercatSystem]: Drive 'axis1' RxPDO 0x1600 (7 bytes): 0x6040:00 uint16 @0 controlword | ...
[IrdEthercatSystem]: Found 1 slave(s).
[IrdEthercatSystem]: Slave 1: HardSync established (0x2012:01=4, 4000 us).
[IrdEthercatSystem]: DC phase locked (...): frame at 3 us, 1997 us ahead of SYNC0.
[IrdEthercatSystem]: All slaves reached OPERATIONAL state.
[IrdEthercatSystem]: All drives enabled successfully. Ready for PDO cyclic operation (...)
[spawner]: Configured and activated joint_state_broadcaster
[spawner]: Configured and activated velocity_controller
```

If it stops earlier, the last `ERROR` lines say why: see [troubleshooting](troubleshooting.md).
When the hardware interface fails to start, Humble's controller manager exits (with
`terminate called after throwing an instance of 'std::runtime_error'`). That line is a
consequence, not the cause.

In a second terminal, command 1 rad/s at the joint, watch the feedback, and stop:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub --once /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0]}"
ros2 topic echo /joint_states
ros2 topic pub --once /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

The velocity in `/joint_states` should settle at about 1.0. If it reads 10.0 or 0.1, the gear
ratio is wrong; any other factor points to `encoder_ticks_per_rev`.

Stop the launch with Ctrl+C: the interface disables the drives through the cyclic PDO, takes them
to SAFE-OP and only then stops the EtherCAT exchange, so no fault is left latched in the drives.

## 7. Use it in your own robot

1. Write the configuration with all your drives (see the [configuration reference](configuration.md)
   and `config/ird_csv_example.yaml`, which shows every key).
2. Add a `<ros2_control>` block to your URDF that points to the file and declares one joint per
   drive. The macro in `urdf/ird_ethercat_example.ros2_control.xacro` is a starting point:

   ```xml
   <ros2_control name="drives" type="system">
     <hardware>
       <plugin>ird_ethercat_hardware_interface/IrdEthercatSystem</plugin>
       <param name="config_file">$(find my_robot)/config/drives.yaml</param>
     </hardware>
     <joint name="left_wheel_joint">
       <command_interface name="velocity"/>
       <state_interface name="position"/>
       <state_interface name="velocity"/>
     </joint>
     <!-- one <joint> per drive -->
   </ros2_control>
   ```

3. Use any ros2_control controller that works with the joints' interfaces, for example
   `diff_drive_controller` for a mobile base in velocity mode.

To read other drive values (currents, voltages, position...) in ROS, add them to the PDO mapping
with an `interface` name: see [Exposing drive registers to ROS](configuration.md#exposing-drive-registers-to-ros).
