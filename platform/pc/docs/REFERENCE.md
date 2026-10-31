# V1 Reference

Facts about the messages, faults, thresholds and state machine of the Control node firmware as implemented in V1. Source of truth: [can_msgs.c](../firmware/common/can_msgs.c), [control_config.h](../firmware/app_control/control_config.h), [fault_mgr.c](../firmware/common/fault_mgr.c), [safety.c](../firmware/app_control/safety.c), [vehicle_fsm.c](../firmware/app_control/vehicle_fsm.c), [pedal.c](../firmware/app_control/pedal.c).

Values marked *proposal* are tuning proposals to be validated on the bench.

## 1. Messages

Standard 11-bit IDs, little-endian, fields back to back, reserved bytes sent as 0 and ignored on receive. Bus units are shown in the console as physical units.

| ID | Name | DLC | Sender | Period | Timeout | Received by |
| :--- | :--- | :-: | :--- | :-: | :-: | :--- |
| 0x100 | Control | 3 | Control node | 20 ms | 100 ms | Motor controller |
| 0x101 | VehicleStatus | 4 | Control node | 100 ms | 500 ms | *(none simulated; dashboard/telemetry on the boat)* |
| 0x200 | MotorTemps | 4 | Motor controller | 100 ms | 500 ms | Control node |
| 0x201 | MotorData | 6 | Motor controller | 20 ms | 100 ms | Control node |
| 0x300 | BMSData | 6 | BMS | 100 ms | 500 ms | Control node |
| 0x301 | BMSExtra | 6 | BMS | 500 ms | 2000 ms | Control node |

Not in V1: GPS (0x400), Position (0x401).

A frame with the wrong length is rejected (`F_RX_DLC` counter). A frame with any field outside its valid range is rejected (`F_RX_RANGE` counter). A rejected frame doesn't refresh the message's timeout.

### Fields

| Message | Field | Bit offset | Bits | Signed | Scale | Unit | Valid range (physical) |
| :--- | :--- | :-: | :-: | :-: | :-: | :--- | :--- |
| Control | `throttle` | 0 | 8 | yes | 1 | % | −100 … 100 |
| | `enable` | 8 | 8 | no | 1 | | 0 … 1 |
| | `alive` | 16 | 4 | no | 1 | | 0 … 15, rolling counter |
| VehicleStatus | `state` | 0 | 8 | no | 1 | enum | INIT, STANDBY, READY, DRIVE, SAFE_STATE |
| | `flags` | 8 | 8 | no | 1 | | bit 0 = derating |
| | `fault_mask` | 16 | 16 | no | 1 | | one bit per fault, see [Faults](#3-faults) |
| MotorTemps | `motor_temp` | 0 | 16 | yes | 0.1 | °C | −40.0 … 200.0 |
| | `ctrl_temp` | 16 | 16 | yes | 0.1 | °C | −40.0 … 200.0 |
| MotorData | `rpm` | 0 | 16 | yes | 1 | rpm | −10000 … 10000 |
| | `current` | 16 | 16 | yes | 0.1 | A | −500.0 … 500.0 |
| | `torque` | 32 | 16 | yes | 0.1 | Nm | −500.0 … 500.0 |
| BMSData | `voltage` | 0 | 16 | yes | 0.1 | V | 0.0 … 150.0 |
| | `current` | 16 | 16 | yes | 0.1 | A | −500.0 … 500.0 |
| | `soc` | 32 | 16 | no | 0.1 | % | 0.0 … 100.0 |
| BMSExtra | `temp` | 0 | 16 | yes | 0.1 | °C | −40.0 … 150.0 |
| | `mppt_current` | 16 | 16 | yes | 0.1 | A | −10.0 … 100.0 |
| | `fan_rpm` | 32 | 16 | yes | 1 | rpm | 0 … 20000 |

Enum fields accept names in the console: `VehicleStatus state=DRIVE`. Message and field names are not case-sensitive.

The `messages` command prints this table from the running program.

### Receive states

Each message the Control node listens to has a communication state:

| State | Meaning |
| :--- | :--- |
| `NO_DATA` | Nothing received since init |
| `OK` | Fresh, valid data |
| `TIMEOUT` | No valid frame within the message's timeout |
| `RECOVERY` | Frames are back; `OK` again after **3** valid frames. Counts as "not valid" meanwhile |

## 2. Control node timing

| Task | Period |
| :--- | :--- |
| CAN protocol processing (RX dispatch, timeouts, periodic TX) | 1 ms |
| Application step (pedal, safety checks, state machine) | 5 ms |
| Status LED | 100 ms (on in DRIVE, blinking in SAFE_STATE) |

A vehicle **state change** sends Control and VehicleStatus immediately (event-driven transmit). Everything else goes out with the periodic frames.

CAN bus-off handling: 100 ms back-off before each restart; after 3 failed restarts, `F_CAN_BUSOFF` is raised.

## 3. Faults

`fault_mask` in VehicleStatus has one bit per fault, in this order (bit 0 first).

| Bit | Fault | Severity | Raised when |
| :-: | :--- | :--- | :--- |
| 0 | `F_RX_TIMEOUT_MOTOR_TEMPS` | warning | MotorTemps timed out. Derates to 30 % |
| 1 | `F_RX_TIMEOUT_MOTOR_DATA` | **critical** | MotorData timed out |
| 2 | `F_RX_TIMEOUT_BMS_DATA` | warning | BMSData timed out |
| 3 | `F_RX_TIMEOUT_BMS_EXTRA` | warning | BMSExtra timed out. Derates to 50 % |
| 4 | `F_RX_TIMEOUT_GPS` | warning | *Reserved (V1 has no GPS)* |
| 5 | `F_RX_TIMEOUT_POSITION` | warning | *Reserved* |
| 6 | `F_CAN_BUSOFF` | **critical** | Bus-off and 3 failed restarts |
| 7 | `F_PEDAL_RANGE` | **critical** | A pedal channel outside its raw limits (broken wire / short) |
| 8 | `F_PEDAL_PLAUS` | **critical** | Pedal channels disagree too much for too long |
| 9 | `F_MOTOR_OVERTEMP_W` | warning | Motor or controller temperature above the warning threshold |
| 10 | `F_MOTOR_OVERTEMP_C` | **critical** | Motor or controller temperature above the critical threshold |
| 11 | `F_BAT_LOW_W` | warning | Battery voltage or SOC below the warning threshold |
| 12 | `F_BAT_LOW_C` | **critical** | Battery voltage below the critical threshold |
| 13 | `F_OVERCURRENT` | **critical** | Motor or battery current above its limit |
| 14 | `F_BAT_TEMP_W` | warning | Battery temperature above the warning threshold |
| 15 | `F_BAT_TEMP_C` | **critical** | Battery temperature above the critical threshold |
| – | `F_COMM_LOST` | **critical** | MotorTemps, MotorData or BMSData stayed timed out for 1 s. *Not in the published mask (bit 16)* |

Any **critical** fault forces SAFE_STATE. A **warning** that affects driving reduces the throttle limit (derating) and sets the derating flag.

Each threshold check only runs on valid data; with no valid data the fault keeps its last state. Checks use hysteresis: once active, the value must move back past *threshold ∓ hysteresis* before the fault clears.

MotorData is critical because the motor controller stops on its own Control timeout and only leaves FAULT after `enable = 0`, which SAFE_STATE sends.

## 4. Thresholds and limits

All in [control_config.h](../firmware/app_control/control_config.h). *Proposals.*

| Quantity | Warning | Critical | Hysteresis |
| :--- | :-: | :-: | :-: |
| Motor temperature | 80 °C | 100 °C | 2 °C |
| Controller temperature | 70 °C | 85 °C | 2 °C |
| Battery voltage (below) | 45 V | 42 V | 0.5 V |
| Battery SOC (below) | 15 % | – | 1 % |
| Battery temperature | 45 °C | 55 °C | 2 °C |
| Motor current (absolute) | – | 200 A | – |
| Battery current (absolute) | – | 180 A | – |

**Derating** (throttle limit, % of full):

| Cause | Limit |
| :--- | :-: |
| Any warning-level temperature / battery fault | 50 % |
| MotorTemps timeout | 30 % |
| BMSExtra timeout | 50 % |

The lowest applicable limit wins.

**Communication grace:** a required message (MotorTemps, MotorData, BMSData) in timeout for **1000 ms** raises `F_COMM_LOST`. Total time from the last good frame is therefore the message timeout plus 1 s (e.g. BMSData: 500 ms + 1 s ≈ 1.5 s).

### Pedal

Two redundant 12-bit channels, same direction.

| Parameter | Value |
| :--- | :-: |
| Raw value at 0 % / 100 % | 400 / 3700 |
| Valid raw range (outside → `F_PEDAL_RANGE`) | 200 … 3900 |
| Max channel disagreement | 10 % |
| …for longer than (→ `F_PEDAL_PLAUS`) | 50 ms |
| Filter (per 5 ms step) | α = 0.3 |
| Deadzone | 5 % (removed, remainder rescaled to 0–100 %) |
| Hysteresis DRIVE → READY | 2 % below the deadzone |
| Throttle rate limit | 1000 %/s (0 → 100 % in 100 ms; 10 % per Control frame) |

A release (both channels at or below the deadzone) bypasses the filter, so lifting off registers in one step.

## 5. Vehicle state machine

States: `INIT`, `STANDBY`, `READY`, `DRIVE`, `SAFE_STATE`.

| From | To | Condition |
| :--- | :--- | :--- |
| any (not SAFE_STATE) | SAFE_STATE | A critical fault is active. The fault is latched as the cause |
| INIT | STANDBY | Immediately (self-test covered by pedal-range and bus-off faults) |
| STANDBY | READY | Enable switch ON **and** it was seen OFF since boot **and** pedal at zero **and** MotorTemps, MotorData, BMSData all valid |
| READY | STANDBY | Enable switch OFF |
| READY | DRIVE | Pedal OK and pedal > 0 % (after deadzone) |
| DRIVE | READY | Pedal not OK, or filtered pedal below deadzone − hysteresis |
| DRIVE | STANDBY | Enable switch OFF; throttle is ramped to 0 first |
| SAFE_STATE | STANDBY | No critical fault and enable switch OFF |
| SAFE_STATE | READY | No critical fault, pedal at zero, data valid. **A pedal held when the fault clears must be released first** |

Outputs:

| | Throttle | Enable |
| :--- | :--- | :--- |
| INIT, STANDBY, SAFE_STATE | 0 | 0 |
| READY | 0 | 1 |
| DRIVE | min(pedal, limit), rate-limited | 1 |

The start-up interlock (enable switch must be seen OFF once after boot) prevents driving away when the Control node boots with the switch already ON.

## 6. Simulated motor controller

| State | Entered when |
| :--- | :--- |
| `INIT` | Power-up; goes to IDLE at once |
| `IDLE` | Default. Goes to RUNNING when a valid Control frame has `enable = 1` and `throttle = 0` |
| `RUNNING` | Torque follows the throttle. Back to IDLE on `enable = 0` |
| `FAULT` | Control timeout (100 ms), frozen alive counter (100 ms), motor temperature > 120 °C, or current > 250 A. Leaves FAULT to IDLE only when the cause is gone and `enable = 0` |

The motor controller can only *start* with throttle 0, so a node that commands throttle while the motor is IDLE won't start it.

Model (first order, for exercising the firmware): current = 1.5 A per % throttle (150 A at 100 %); rpm = 30 per % throttle with a 0.3 s lag; torque = 0.12 Nm per A; motor temperature rises with I² (time constant 30 s), controller temperature with I² (20 s), ambient 25 °C.

## 7. Simulated BMS

48 V-class pack *(proposal)*, 20 mΩ. SOC is fixed at 80 % (no automatic drain; set a low battery by hand with `BMSData soc=…` or `voltage=…`). Starts at 25 °C. Positive current = discharge. Pack current = motor current + 2 A auxiliary − MPPT current (5 A). Voltage = 44 V + 0.08 V per % SOC − 0.02 Ω × current. Temperature rises with I² (120 s). Fan runs above 30 °C. The BMS receives no messages.

## 8. Console commands (full syntax)

| Command | Details |
| :--- | :--- |
| `<Message> field=value …` | Message name (optionally prefixed with `send`), then any fields. Unknown field lists the valid ones. A value that doesn't fit the field is clamped with a warning |
| `raw <id> [byte …]` | Up to 8 hex bytes; DLC = number of bytes. Always sent once, on the "manual" port |
| `release <Message>` / `release all` | Removes held overrides |
| `pedal <A> [B]` | Percent on channel A (and B; defaults to A), converted to raw ADC like the real sensor |
| `enable on\|off` | Enable switch |
| `wait <ms>` / `wait 2s` | Advance time (scripts) |
| `status` | `t`, Control node state, inputs, active faults, BMS values, held values, last Control node frames |
| `messages`, `help`, `quit` | |

Messages sent by a simulated node (MotorTemps, MotorData, BMSData, BMSExtra) are **held**: a field override persists in every periodic frame until released (up to 16 overrides). Messages from the Control node (Control, VehicleStatus) are sent once; the Control node keeps sending its own frames.

### Log line types

| Line | Meaning |
| :--- | :--- |
| `Control node accepted the frame` / `REJECTED … wrong length` / `… out of range` | Verdict of the protocol layer |
| `vehicle state A -> B (…)` | Control node state change |
| `fault SET / CLEAR <name> (severity)` | Fault change |
| `rx <Message> A -> B` | Receive state change (OK, TIMEOUT, RECOVERY, …) |
| `CAN controller A -> B` | CAN error state change (ERROR_ACTIVE, ERROR_PASSIVE, BUS_OFF) |
| `motor A -> B (reason)` | Simulated motor controller state change |
| `Control.<field>: a -> b`, `VehicleStatus.<field>: a -> b` | Visible reaction on the bus (alive counter omitted) |
| `settled - …` | 300 ms with nothing new; shows the Control node's state |

In piped (script) mode there is no live clock: each action runs until settled (300 ms of silence, max 5 s) before the next line is read, so output is reproducible.
