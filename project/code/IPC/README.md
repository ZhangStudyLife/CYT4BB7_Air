# IPC image data

CM7_1 writes the shared `image_data[IMAGE_CAMERA_COUNT]` array. CM7_0 reads the same symbol directly.

Fixed shared addresses are assigned in `linker_directives_tviibh.icf`:

- `image_data`: `.ipc_image_data` at `0x28001000`
- `g_image_data_seq`: `.ipc_image_seq` at `0x28001160`
- `g_ipc_camera_spi_log`: `.ipc_camera_spi_log` at `0x28001180`

CM7_1 calls `ipc_image_publish()` after updating all three camera slots. The function cleans D-cache for `image_data` and `g_image_data_seq`, then sends an IPC notification.

CM7_0 calls `ipc_image_poll()` before using image results. The function invalidates D-cache for `image_data` and `g_image_data_seq`.
