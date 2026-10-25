# V1 Architecture

How V1 is built, and where to change things. For behaviour and numbers see [REFERENCE.md](REFERENCE.md).

## 1. Idea

The firmware under test is compiled unchanged for the PC. Everything that touches hardware sits behind a small **HAL** (hardware abstraction layer). On the boat the HAL talks to the microcontroller; in the simulator it talks to a virtual CAN bus, a virtual clock and settable ADC/GPIO values.

```
            ┌──────────────────────── cansim.exe ─────────────────────────┐
            │  console (cansim.c): commands, parsing, log, live display   │
            │                          │                                  │
            │                    sim_world (1 ms step)                    │
            │    ┌──────────┬──────────┼───────────┬─────────────┐        │
            │ motor ctrl   BMS      sim_bus     "manual" port   Control    │
            │ (sim_nodes) (sim_nodes) (virtual   (typed frames)   node     │
            │    │          │        CAN bus)                    firmware  │
            │    └──────────┴──── ports ───────────────────────────┤       │
            │                                              mock HAL (PC)   │
            └──────────────────────────────────────────────────────────────┘
```

## 2. Source layout

### firmware/ – code that also runs on the target

| Module | Responsibility |
| :--- | :--- |
| `app_control/control_node.c` | Wires everything: TX providers (Control, VehicleStatus), RX list, scheduler tasks (1 ms CAN, 5 ms application, 100 ms LED), the 5 ms application step |
| `app_control/pedal.c` | Two ADC channels → filtered percent, range and plausibility checks, deadzone |
| `app_control/safety.c` | Message-timeout handling, threshold faults with hysteresis, derating limit, data validity |
| `app_control/vehicle_fsm.c` | State machine, throttle ramp, enable output, start-up interlock |
| `app_control/control_config.h` | **All** limits, gains and timings |
| `common/can_msgs.[ch]` | Message structures, explicit pack/unpack, range checks, message table (ID, DLC, period, timeout) |
| `common/can_proto.[ch]` | Frames ↔ typed messages, per-message RX state, timeouts, recovery, periodic and event TX, bus-off restart, statistics |
| `common/can_ringbuf.[ch]` | RX ring buffer |
| `common/fault_mgr.[ch]` | Fault bitmask, severity, names |
| `common/sched.[ch]` | Cooperative scheduler (`sched_run` on target, `sched_run_once` per simulated ms in the PC build) |
| `hal/hal_*.h` | HAL interfaces: `hal_can`, `hal_time`, `hal_adc`, `hal_gpio` |
| `hal/mock/hal_mock.[ch]` | PC implementation of the HAL plus test-side setters (`hal_adc_mock_set`, `hal_gpio_mock_set`, `hal_can_mock_force_state`, …) |

Rules the firmware follows:

- Bus bytes are produced and read **only** by the explicit pack/unpack functions in `can_msgs.c` (no struct casting, no bit-fields): little-endian, reserved bytes sent as 0 and ignored on receive.
- Time is compared by subtraction, so the 32-bit millisecond wrap is harmless (covered by a test).
- Faults are written only from the main loop.
- The application only sees data that is fresh and in range; invalid frames are counted and dropped by the protocol layer.

### sim/ – the simulated environment (PC only)

| Module | Responsibility |
| :--- | :--- |
| `sim_bus.[ch]` | Virtual bus. Ports attach with a TX queue and an ID filter. Each ms the bus delivers queued frames **lowest ID first** (arbitration) to every other port whose filter accepts them, and remembers the last frame per ID |
| `sim_nodes.[ch]` | Motor controller and BMS behavioural models; field **overrides**; `sim_node_send()` packs, applies overrides, queues |
| `sim_world.[ch]` | Owns time, bus, nodes and the device under test (DUT). Fixed step order, change detection → events |
| `sim_dbc.[ch]` | Message/field definitions by name (offset, bits, signedness, scale, unit, enum names) used for typing and printing |
| `sim_util.h` | Small helpers (case-insensitive compare, number parsing) |
| `cansim.c` | The console program: command parser, live line editor, log formatting, `status` |

### tests/

| File | Notes |
| :--- | :--- |
| `test_util.h` | Minimal `RUN_TEST` / assertion / summary macros |
| `test_codec.c` | Codec tests; also checks the firmware message table against `sim_dbc` |
| `test_can_proto.c` | Protocol-layer tests against the simulated bus |

## 3. One simulated millisecond

`sim_world_step()` always does the same, in this order:

1. `now += 1 ms` (the HAL's `hal_time_tick_ms()` returns this value)
2. The bus delivers frames queued in the **previous** step
3. The simulated nodes step (read their inbox, run their model, send periodic frames)
4. The Control node runs one scheduler pass
5. State changes since the last step are compared with a snapshot and reported as events (vehicle state, faults, RX states, CAN state, motor state)

Consequences: a frame sent in step *n* is received in step *n + 1* at the earliest; results are deterministic and repeatable; log times are simulation time, not PC time.

In **live mode** the console calls the step function in step with the real clock while you type. In **script mode** (stdin is not a terminal) or `--no-live`, an action runs until 300 ms pass with no new event (max 5 s), then the next input is read.

## 4. How a typed message becomes a log

1. `cansim.c` resolves `<Message>` through `sim_dbc`, sets the named fields (physical → raw) on top of the last frame seen on the bus for that ID.
2. If a simulated node normally sends it, the values are stored as **overrides** and `sim_node_send()` applies them to every later periodic frame; otherwise the frame is queued once on the "manual" port.
3. The console prints `Sent`, `Received by` (from the receivers' ID filters) and starts the action timer.
4. While time runs, `sim_event()` calls (state/fault/rx/motor changes) and field changes of the Control node's own frames are printed as `[+ t ms]` lines, timed from the action.
5. `settled` is printed after 300 ms without new lines; the log continues for slower effects.

## 5. Building blocks and extension points

### Add or change a limit

Edit [control_config.h](../firmware/app_control/control_config.h) and rebuild. Message periods, timeouts and ranges are in the table and unpack functions of [can_msgs.c](../firmware/common/can_msgs.c).

### Add a message

1. `can_msgs.h`: ID constant, payload struct, pack/unpack prototypes.
2. `can_msgs.c`: pack/unpack (with range check), `ADAPT(...)`, and a `DEF(...)` row in `defs[]`.
3. `sim_dbc.c`: field table and a `MSG(...)` row (sender: `dut`, `motor` or `bms`).
4. If the Control node receives it: add the ID to `rx_ids` and the counts in `control_node.c`; if it sends it: add a `tx_entries` row and a provider.
5. If a simulated node sends it, send it with `sim_node_send()` so overrides apply.
6. Update `test_codec.c`. `test_table_matches_dbc` fails if the firmware table and the console's field definitions disagree.

### Add a fault

Add the code to `fault_code_t` (before `F_COUNT`; **mind the 16-bit published mask**), its name/severity in `fault_mgr.c`, and the check in `safety.c` (or `app_step`). Only critical faults force SAFE_STATE.

### Add a simulated node

Attach a port with `sim_bus_attach()`, set its filter with `sim_port_set_filter()`, step it from `sim_world_step()`, and send through `sim_node_send()`.

### Port to hardware

Implement `hal_can`, `hal_time`, `hal_adc`, `hal_gpio` for the target and call `control_node_init()` then `sched_run()` from `main()`. Nothing in `firmware/app_control` or `firmware/common` includes PC-only code.

## 6. Build system

[CMakeLists.txt](../CMakeLists.txt): one static library `sbt_sim` (firmware + mock HAL + `sim/` except the console), the executable `cansim`, and one executable per test registered with CTest. Warnings: `-Wall -Wextra`. Default build type: Debug. With MinGW the executables are linked statically so they run without the MinGW DLLs on PATH. [build.bat](../build.bat) wraps configure (Ninja generator), build, `ctest`, run and clean.
