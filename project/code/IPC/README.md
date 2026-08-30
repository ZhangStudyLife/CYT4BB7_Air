# IPC image data

CM7_1 publishes a complete snapshot of the three-camera `image_data` array. CM7_0
copies a stable snapshot into its local `image_data` before the control loop reads it;
the two cores do not access the working array concurrently.

Fixed shared addresses are assigned in `linker_directives_tviibh.icf`:

- `image_data`: `.ipc_image_data` at `0x28001000`
- `g_image_camera_seq[3]`: `.ipc_image_camera_seq` at `0x28001150`
- `g_image_data_fresh_mask`: `.ipc_image_fresh_mask` at `0x2800115C`
- `g_image_data_seq`: `.ipc_image_seq` at `0x28001160`
- `g_image_data_guard`: `.ipc_image_guard` at `0x28001164`
- `g_ipc_camera_spi_log`: `.ipc_camera_spi_log` at `0x28001180`

CM7_1 calls `ipc_image_publish(fresh_mask)` only when at least one camera has a
real new result or a stale-result state change. The publisher uses an odd/even guard
around the copy, increments the per-camera result sequence only for `fresh_mask`,
then sends a non-blocking IPC hint. A busy hint does not lose data because CM7_0
also polls the shared sequence at 1 kHz and immediately before each 100 Hz control
cycle.

The front/rear Camera SPI payload carries an 8-bit algorithm-result sequence in
header byte 3 (protocol version 4). Repeated SPI packets with the same result
sequence are not counted as new image results. The Air CM7_1 image service period
is 5 ms (200 Hz), while the flight-control loop remains 10 ms (100 Hz).

CM7_0 calls `ipc_image_poll()` before using image results. The function validates
the guard before and after copying, then updates local `image_data`,
`g_image_camera_rx_seq[]`, and `g_image_data_rx_seq` atomically from the control
loop's point of view.

## 2BL3 runtime parameters

The remote parameter mailbox keeps the original `type + uint32 value_bits` layout:

- `type=0`: IEEE-754 `float`
- `type=1`: signed `int32`
- parameter `1`: beacon binary threshold
- parameter `2`: camera exposure time
- parameters `0x0100..0x011F`: the 32 shared front/rear 2BL3 image runtime parameters declared in `ipc_image_data.h`

Area-table cells are addressed on demand and are not registered as ordinary Air parameters:

```text
parameter_id = 0x2000 + camera * 126 + bound * 63 + row * 9 + column
camera: 0=front, 1=back
bound:  0=lower, 1=upper
row:    0..6
column: 0..8
```

The table commands are `0x2100=save`, `0x2101=reload`, and `0x2102=restore defaults`. They accept only `SET int32 1`. Camera SPI sends them to both 2BL3 boards without parameter preflight or rollback; success requires two valid ACKs. Ordinary scalar and cell writes retain dual-board preflight, readback verification, and rollback.

[返回 Air 总文档](../../../README.md) · [返回母仓库 README](https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield/blob/national-2026/README.md)
