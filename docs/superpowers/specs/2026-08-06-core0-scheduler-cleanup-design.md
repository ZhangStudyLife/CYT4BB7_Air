# Core 0 Scheduler Cleanup Design

## Goal

Simplify Core 0 development without changing the verified behavior of commit
`06dd226225fcf276c4ab03ff78db25762a09dd08`.

Developers should continue adding control code through the existing
`FC_Loop_1000Hz()`, `FC_Loop_500Hz()`, `FC_Loop_100Hz()`, and
`FC_Loop_50Hz()` entry points. Sensor and communication scheduling details
remain hidden from the main loop.

## Behavior To Preserve

- IMU, position estimation, flight control, and AirComm polling at 1 kHz.
- `FC_Loop_500Hz()` at 500 Hz.
- Height estimation, CRSF receive processing, and `FC_Loop_100Hz()` at 100 Hz.
- ToF software-I2C state machine at 400 steps per second.
- `FC_Loop_50Hz()` at 50 Hz.
- Takeoff/landing state processing and non-blocking CRSF attitude telemetry at
  10 Hz, with CRSF telemetry remaining lowest priority.
- Non-blocking AirComm transmit queue and UART TX interrupt handling.
- ToF sample freshness and height-estimator freshness behavior.
- IPC state notification and retry behavior.

## Code Structure

Keep the implementation in `project/user/main_cm7_0.c` so the IAR project does
not need another source file.

Extract a small number of file-local helpers:

- A fast-loop step containing the existing 1 kHz work and the 500 Hz divider.
- A slow-slot dispatcher preserving the current slot mapping.
- An IPC-state maintenance helper containing its retry state.
- A planner/send helper for the slot 5 dependency chain.

The `main()` loop will contain only initialization, bounded 1 kHz backlog
processing, WiFi command polling, and one slow-scheduler call.

## Code To Remove

- DWT profiling initialization, macros, structures, counters, and conversions.
- The automatic 42-channel 100 Hz JustFloat performance stream.
- Performance-log queue inspection and reset logic.
- Large commented-out JustFloat debug blocks in `main_cm7_0.c`.
- Automatic JustFloat standby-context updates.
- The unused `g_tick_100HZ` counter, PIT channel 1 initialization, and its ISR
  counter update.

Keep `wifi_justfloat_Init()` and the JustFloat module available for future
small, manually-added debug streams.

## Validation

- Compare the before/after call order for every slow slot.
- Confirm theoretical rates remain 1000/500/400/100/50/10 Hz as applicable.
- Confirm CRSF transmit still uses the non-blocking hardware FIFO path.
- Confirm AirComm TX ISR and ToF freshness APIs remain unchanged.
- Confirm no automatic `wifi_justfloat()` call remains in the Core 0 main loop.
- Run `git diff --check`.
- Do not invoke the IAR build from the command line; the user will compile it.

