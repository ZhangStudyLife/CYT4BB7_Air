# IPC image data

CM7_1 writes the shared `image_data[IMAGE_CAMERA_COUNT]` array. CM7_0 reads the same symbol directly.

Fixed shared addresses are assigned in `linker_directives_tviibh.icf`:

- `image_data`: `.ipc_image_data` at `0x28001000`
- `g_image_data_seq`: `.ipc_image_seq` at `0x28001160`
- `g_ipc_camera_spi_log`: `.ipc_camera_spi_log` at `0x28001180`

CM7_1 calls `ipc_image_publish()` after updating all three camera slots. The function cleans D-cache for `image_data` and `g_image_data_seq`, then sends an IPC notification.

CM7_0 calls `ipc_image_poll()` before using image results. The function invalidates D-cache for `image_data` and `g_image_data_seq`.

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
