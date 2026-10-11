# Host interface

The host uses SPI mode 0 at no more than `clk/8` (6.25 MHz at the 50 MHz
design clock). Hold `CS_n` high for at least four design clocks between
transactions, and wait at least four design clocks from `CS_n` falling to the
first SCK rising edge.

Each transaction is exactly 16 SCK cycles. MOSI sends `[cmd, din]`, most
significant bit first; MISO returns `[status, dout]`. `status` is sampled when
`CS_n` falls, and the command takes effect on the 16th rising edge.

| Command | Meaning |
| --- | --- |
| `cmd[7]=1` | Start a block; `cmd[6]` selects compression (`0`) or decompression (`1`). Other command bits are ignored. |
| `cmd[0]=1` | Push `din` when the snapshotted `in_ready` bit is set. |
| `cmd[1]=1` | Pop the output byte when the snapshotted `out_valid` bit is set. |
| `cmd[2]=1` | End input when the snapshotted `in_ready` bit is set. May be combined with PUSH. |

The returned status is `{4'b1010, error, done, out_valid, in_ready}`. The
dedicated outputs `uo[4:1]` mirror the four low status flags, with `uo[4]` as
`error` and `uo[1]` as `in_ready`. Bytes beyond the first 16 SCK cycles are
ignored until `CS_n` rises.

`ui3` selects the PSRAM read-sampling edge: `0` samples with the SCK rising
edge, and `1` samples one design clock later. Set it while `rst_n` is low.
