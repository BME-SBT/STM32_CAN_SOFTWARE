# SBT CAN Simulator – V1

### Notes
I did not sleep enough this week but V1 is done-ish, majority of code is originally hand-written, Claude Opus 5.5 and Sonnet 5.5 agents and subagents were utilized to fix my stuff, details:
- A 'safety' agent was tasked with revising the entire codebase and fixing parts of the code to implement (supposedly) industry-standard safety approaches where absent (I glossed over many hazards)
- An 'architect' agent was used to restructure my initial version (which was chaotic as I hate working with multiple files) and organizing the logic into seperate files/folders/functions based on IT best practices (also not my strong suite)
- A Sonnet subagent of the 'architect' also cleaned up variable/function/config/whatever names, bringing the codebase naming conventions up to best practice (from non-existent...)
- A 'publisher' agent wrote every documentation, deleted all my initial temp comments and wrote new ones where deemed necessary
- Additionally, AI was used to set up the compilation, installing the necessary packages and constructing the builder file
- I used AI to translate the raw CAN inputs as I could not be bothered to go through the hex stuff manually
- V2 will have scriptable tests and injectable faults
### The rest of the readme is courtesy of the publisher agent


V1 does one thing: **you write a message to the CAN network, and it shows what happened as a result.**

It runs the real **Control node firmware** (the C code that will run on the boat's microcontroller) on your PC, connected to a simulated CAN bus with a simulated **motor controller** and **BMS**. You type a CAN message, a pedal position or a switch change, and a live, timestamped log shows how the firmware reacted: state changes, faults, derating, and the frames the Control node puts on the bus.

```
~> MotorTemps motor_temp=105
  The simulated motor controller keeps sending motor_temp=105.0 in every MotorTemps frame ('release MotorTemps' to stop).
  Sent       : 0x200 MotorTemps    [4] 1A 04 FB 00             motor_temp=105.0 C ctrl_temp=25.1 C
  Received by: Control node
  Log:
  [+     1 ms] - Control node accepted the frame :)
  [+     4 ms] - vehicle state DRIVE -> SAFE_STATE (latched F_MOTOR_OVERTEMP_C)
  [+     4 ms] - fault SET   F_MOTOR_OVERTEMP_W (warning)
  [+     4 ms] - fault SET   F_MOTOR_OVERTEMP_C (critical)
  [+     5 ms] - motor RUNNING -> IDLE (enable=0)
  [+     5 ms] - Control.throttle: 47 -> 0 %
  [+     5 ms] - Control.enable: 1 -> 0
  [+     5 ms] - VehicleStatus.state: DRIVE -> SAFE_STATE
  [+     5 ms] - VehicleStatus.flags: 0 -> 1
  [+     5 ms] - VehicleStatus.fault_mask: 0 -> 1536  (F_MOTOR_OVERTEMP_W F_MOTOR_OVERTEMP_C)
  [+   305 ms] - settled - Control node: SAFE_STATE (cause: F_MOTOR_OVERTEMP_C), throttle 0 %, enable 0
```

## Features

- **Real firmware, not a model of it.** The application, CAN protocol layer, fault manager and scheduler are the same C sources that target the hardware. Only the hardware abstraction layer (HAL) is replaced by a PC mock.
- **Live, simulated time.** The simulation advances 1 simulated ms per real ms. Log times come from the simulation clock, so they show exactly when the firmware reacted.
- **Physical units.** You write `motor_temp=105` (°C) or `voltage=41.5` (V); the tool converts to bus bytes using the message definitions.
- **Values that stick.** A value you set in a simulated node's message stays in every frame that node sends until you `release` it.
- **A steady battery.** The simulated battery does not drain: its state of charge stays at 80 % however long you drive. Low-battery situations are set by hand (see [Battery](#battery)).
- **Raw frames.** `raw <id> <bytes…>` sends any ID, length and payload, including malformed frames, to test the protocol layer's rejection paths.
- **Driver inputs.** Accelerator pedal (both sensor channels independently) and enable switch.
- **Scriptable.** Pipe commands into the program for reproducible output.

## Requirements

| Tool | Notes |
| :--- | :--- |
| Windows 10/11 | `build.bat` and the live console target Windows |
| GCC (MinGW-w64) | C11; tested with the WinLibs POSIX/UCRT build |
| CMake ≥ 3.20 | |
| Ninja | |

All three tools come in the WinLibs package:

```
winget install --id BrechtSanders.WinLibs.POSIX.UCRT -e
```

Then **open a new terminal** (terminals that were already open don't see the new PATH) and check:

```
gcc --version
cmake --version
ninja --version
```

Without winget: download the "UCRT runtime, POSIX threads, x86_64" zip from <https://winlibs.com>, extract it to `C:\mingw64` and add `C:\mingw64\bin` to your PATH.

## Build

Open a terminal in the `V1` folder:

| Command (PowerShell: prefix with `.\`) | Effect |
| :--- | :--- |
| `build.bat` | Configure (first time) and build |
| `build.bat run` | Build, then start the console |
| `build.bat clean` | Delete the `build` folder |

The build produces `build\cansim.exe`.

- **Quit the simulator before rebuilding.** Windows won't overwrite a running `cansim.exe`, and the build fails with "Permission denied".
- If you moved or copied the folder and CMake complains, run `build.bat clean` and build again.

## Run

```
.\build\cansim.exe
```

```
  ~ can simulator v1.8 ~    for all commands type 'help', 'exit' to exit
network running: control node in STANDBY, simulated motor controller and BMS sending (t = 200 ms).
running in live mode, type at any time

~>
```

The network has booted and runs in real time; the Control node waits in STANDBY. `~>` is the prompt. Log lines appear above the line you are typing, **↑ / ↓** recall earlier commands, and `exit`, `quit` or Ctrl+C closes the program.

| Option | Effect |
| :--- | :--- |
| *(none)* | Live mode: the log runs continuously while you type |
| `--no-live` | Line-by-line mode: each action prints its log until it settles, then waits for the next command. Use it if the live display looks garbled in your terminal |

Live mode needs a real console. When input is piped in, the program switches to line-by-line mode by itself.

## Quick start

Drive the boat, then break it:

```
enable on                      # enable switch ON, pedal at 0  -> READY
pedal 50                       # -> DRIVE, throttle ramps up to 47 %
MotorTemps motor_temp=85       # warning: throttle limited to 50 %
MotorTemps motor_temp=105      # critical: SAFE_STATE, throttle 0, enable 0
release all                    # simulated nodes send their own values again
pedal 0                        # cause gone + pedal at zero -> back to READY
```

**Note:** after a critical fault clears, the boat only returns to READY once the pedal is at 0 %. If the pedal is still pressed, it stays in SAFE_STATE until you release it (`pedal 0`); if the pedal is already at 0 when the fault clears, it returns to READY by itself.

After every action the log keeps running. 300 ms with nothing new prints a `settled` summary line, but slow reactions (a message timeout after 500 ms, the 1 s grace period before `F_COMM_LOST`) still appear afterwards, timed from the same action.

## Battery

The simulated BMS keeps the state of charge fixed at **80 %**. Driving changes only the current and the voltage drop under load (about 47.5 V at full throttle).

To test low-battery behaviour, set the values yourself. The BMS keeps sending them until you release them:

| You type | Result |
| :--- | :--- |
| `BMSData soc=12` | Warning `F_BAT_LOW_W`: throttle limited to 50 % |
| `BMSData voltage=44` | Warning `F_BAT_LOW_W` (below 45 V) |
| `BMSData voltage=41.5` | Critical `F_BAT_LOW_C` (below 42 V): SAFE_STATE |
| `release BMSData` | The BMS sends its own values again |

`status` shows these under "Held values". Its "BMS" line shows the model's own values, not the ones you are holding.

## Commands

| Command | Meaning |
| :--- | :--- |
| `<Message> [field=value …]` | Send a message, e.g. `BMSData voltage=41.5 soc=12`. Fields you leave out keep their last value on the bus. `send <Message> …` and message IDs (`send 0x300 …`) also work |
| `raw <id> [byte …]` | Send any frame. ID in hex (`0x201`) or decimal, data bytes in hex; the length is the number of bytes |
| `release <Message>` / `release all` | Stop holding your values; the simulated node sends its own again |
| `pedal <pct> [pct_B]` | Accelerator pedal. Two different values make the sensor channels disagree (a sensor fault) |
| `enable on` / `enable off` | Enable switch |
| `messages` (or `list`) | All messages with their fields and units |
| `status` | State of every node, inputs, active faults, held values, the Control node's last frames |
| `wait <time>` | Scripts only: let time pass (`wait 500`, `wait 2s`) |
| `help`, `exit` / `quit` | |

Messages sent by a simulated node (MotorTemps, MotorData, BMSData, BMSExtra) are **held**: your value goes into every frame that node sends. Messages from the Control node itself (Control, VehicleStatus) are sent **once**, because the Control node keeps sending its own.

## Scripts

Pipe commands in for repeatable runs:

```
"enable on", "pedal 50", "MotorTemps motor_temp=105" | .\build\cansim.exe
```

Without a person typing there is no live clock: each action runs until it has settled (at most 5 s of simulated time), then the next line is read. The same script always gives the same output. Use `wait` to let more time pass, e.g. `wait 2s` to see a timeout. Lines starting with `#` are comments.

## Project layout

```
V1
├── README.md                     this file
├── docs/
│   ├── REFERENCE.md              messages, faults, thresholds, state machine
│   └── ARCHITECTURE.md           how the firmware and simulator are built
├── build.bat                     build / run / clean
├── CMakeLists.txt                build configuration
├── firmware/                     the Control node firmware under test
│   ├── app_control/              the Control node application
│   │   ├── control_config.h      every limit and tuning value
│   │   ├── control_node.c/.h     wiring: CAN config, periodic messages, 5 ms step, task list
│   │   ├── pedal.c/.h            pedal: range and plausibility checks, filter, deadzone
│   │   ├── safety.c/.h           message timeouts, threshold faults, throttle limit
│   │   └── vehicle_fsm.c/.h      vehicle state machine and throttle ramp
│   ├── common/                   shared firmware services
│   │   ├── can_msgs.c/.h         message IDs, structs, pack/unpack, message table
│   │   ├── can_proto.c/.h        CAN protocol layer: receive, timeouts, transmit, bus-off
│   │   ├── can_ringbuf.c/.h      queue of received frames
│   │   ├── fault_mgr.c/.h        active faults, names and severities
│   │   └── sched.c/.h            cooperative scheduler
│   └── hal/                      hardware abstraction interfaces
│       ├── hal_adc.h             pedal sensor channels
│       ├── hal_can.h             raw CAN frames
│       ├── hal_gpio.h            enable switch, status LED
│       ├── hal_time.h            millisecond clock
│       └── mock/
│           └── hal_mock.c/.h     PC implementation of the HAL
├── sim/                          simulated network and the console
│   ├── cansim.c                  the console program (all terminal output)
│   ├── sim_bus.c/.h              virtual CAN bus: arbitration, filters, delivery
│   ├── sim_dbc.c/.h              message fields by name, for typing and printing
│   ├── sim_nodes.c/.h            simulated motor controller and BMS, held values
│   ├── sim_util.h                string and number helpers
│   └── sim_world.c/.h            time, stepping order, event reporting
```

Limits and configs (temperatures, voltages, currents, throttle rate) are in `firmware/app_control/control_config.h`. Message periods, timeouts and valid ranges are in `firmware/common/can_msgs.c`.

## Scope and limitations

- The **Control node** is the device under test; the motor controller and BMS are simulated. **GPS and Position (0x400, 0x401) are not part of V1**: they are telemetry only and the Control node doesn't use them. Their fault-mask bits are reserved so the layout matches later versions.
- V1 has **no fault injection** (bus-off, dropped frames, disconnected nodes), no automatic checks and no scenario files. Those are in V2, which contains the same firmware.
- The motor and BMS models are simple approximations for exercising the firmware, not accurate physics. The battery never drains on its own.
- Thresholds, pedal calibration and pack parameters are **proposals to be validated on the bench**.
- The live console is Windows-only.
