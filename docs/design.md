# LZ4 compressor/decompressor — Tiny Tapeout sky130, 1×1, 50 MHz

The externally visible behavior is unchanged: the host SPI pins and byte protocol remain the same,
compression is byte-identical to `test/golden.py`, and the decoder supports standard LZ4 block input.
Each START begins a new independent block. The maximum block size is 65,535 bytes.

## Pins

| pin | dir | function |
|---|---|---|
| ui0 | in | host SPI SCK (mode 0) |
| ui1 | in | host SPI MOSI |
| ui2 | in | host SPI CS_n |
| ui3 | in | PSRAM read-sample delay, latched while rst_n=0 (0 = sample on SCK rising edge, 1 = one clk later) |
| ui7:4 | in | unused |
| uo0 | out | host SPI MISO |
| uo1 | out | in_ready (same as status bit0) |
| uo2 | out | out_valid (status bit1) |
| uo3 | out | done (status bit2) |
| uo4 | out | error (status bit3) |
| uo7:5 | out | 0 |
| uio0 | out | flash CS0_n, constant 1 |
| uio1,2,4,5 | io | PSRAM SD0, SD1, SD2, SD3 |
| uio3 | out | PSRAM SCK |
| uio6 | out | PSRAM CS_n (RAM A on TT QSPI Pmod) |
| uio7 | out | RAM B CS2_n, constant 1 |

## Host SPI protocol

SCK, MOSI, and CS_n pass through two-flop synchronizers. The host uses mode 0, MSB-first, with SCK no
faster than `clk/8`, at least four system clocks from CS_n falling to the first SCK rise, and at least
four clocks of CS_n high between transactions. The host samples MISO before each SCK rise; the
synchronized rise advances the slave shift register several system clocks later.

Each transaction has exactly 16 SCK rises: `[command, data]` is sent while `[status, output]` is
received. `spi_slave` has one 8-bit shift register `sr`, a 5-bit command register, two status snapshot
bits, a 4-bit bit counter, and the input synchronizers. On CS_n falling it loads `sr` with status,
resets the counter, and snapshots `in_ready` and `out_valid`. On the eighth synchronized rise it
captures START, MODE, END, POP, and PUSH and loads `sr` with `out_byte` if the snapshot said valid,
otherwise zero. On the sixteenth rise, command outputs are combinational pulses and `push_data` is
`{sr[6:0], mosi}`. Extra rises have no effect until CS_n rises.

- START is accepted only after PSRAM initialization; it selects compression (`MODE=0`) or
  decompression (`MODE=1`) and initializes the engine.
- PUSH and END require the snapshotted `in_ready`; if both are set, the data byte is accepted and then
  EOF is marked.
- POP requires the snapshotted `out_valid`.
- `status = {4'b1010, error, done, out_valid, in_ready}`.
- `in_ready = started && !in_full && !end_flag && !done && !error`; internal EOF is
  `end_flag && !in_full`; `done` is asserted after the engine finishes and any held output byte is
  drained.

All submodules use named, narrow signal ports. Top-level pin mapping remains in `src/project.v`.

## PSRAM controller (APS6404L in QPI)

PSRAM SCK is `clk/2`. The controller sends read opcode `0xEB` with six dummy clocks and write opcode
`0x02`, then six MSB-first address nibbles. The 18-bit byte address map is history `{2'b00, ptr}` and
hash entry `{1'b1, h[15:0], 1'b0}`. One phase counter is reused for initialization, command, address,
and dummy phases; neither the engine address nor nibble data is registered by the controller.

`ui3` is latched as `rd_late`: 0 samples read data in the rising-edge sample cycle, and 1 samples one
system clock later. `sd_oe` is enabled for commands, addresses, and write data, and disabled for read
dummy/data phases. `nib_strobe` advances the engine's nibble pipe; `byte_done` marks each second
nibble. The engine supplies write nibbles combinationally.

The stream sends each logical byte low nibble first. The LY68L6400 model stores the first serial
nibble in `mem[addr][7:4]`, so the initial table mirror reconstructs a logical byte from nibble order
`byte0[7:4], byte0[3:0], byte1[7:4], byte1[3:0]`; the engine's shifts reverse the stream into
little-endian words and table entries.

At `byte_done`, the controller latches both `more` and whether the current address is at the 1 KiB
page boundary. Its following data-phase decision uses only those latched values. If continuing at a
page boundary, it raises CS_n and clocks SCK high while CS_n is high to reset the PSRAM parser, then
restarts command/address using the engine's advanced address. Table entries are two-byte aligned and
never cross a page.

After reset, initialization sends QPI command `0xF5` as two nibbles (`F`, `5`), raises CS_n and
provides an SCK rising edge, then sends SPI command `0x35` MSB-first on SD0 with SD1–3 undriven. After
another CS-high SCK rising edge, `ready` is asserted. This exits QPI and re-enters it for an APS6404L
that powers up in SPI mode; the host must provide the PSRAM power-up delay before releasing reset.
The QPI-only LY68L6400 model starts in QPI mode and ignores these two short initialization
transactions before unknown-command reporting.

## Shared-area codec engine

The engine has 16-bit `w`, `p`, `q`, `e`, `anchor`, and `ptr`; 32-bit `cur`; an 8-bit `ibyte` with
`in_full`; and an 8-bit `out_byte` with `out_valid`. Control state and small flags track mode,
termination, comparison result, match presence, and nibble progress. Datapath flops are not
asynchronously reset; START initializes the position registers. Input and output each have one byte
of holding capacity, and output-producing steps wait until the previous byte is popped.

`ptr` is the sole history address register. PSRAM address is always combinational: history uses
`{2'b00, ptr}` and table access uses `{1'b1, hash(cur), 1'b0}`. It increments by one on each completed
history byte, except the terminating mismatching byte of match extension; other pointer loads use the
shared ALU. There is no registered PSRAM address or read-data register.

Read data is consumed directly from `sd_in` on `nib_strobe`: `cur <= {sd_in,cur[31:4]}` for history
reads, `e <= {sd_in,e[15:4]}` for the two-byte table entry, and
`out_byte <= {sd_in,out_byte[7:4]}` for output/copy reads. Candidate matching rotates `cur` by one
nibble while comparing, so eight rotations restore the original word. `wnib` is a combinational mux
among input/literal data, the decompression copy byte, and the hash-table write value.

All engine arithmetic shares one 16-bit ALU:

```text
R = A + (B ^ {16{inv}}) + cin
```

FSM-selected operands implement additions, subtractions, and comparisons; the carry indicates
underflow or sufficient length as appropriate. The dedicated `ptr + 1` update is the only general
incrementer. Small phase counters and nibble shifts are separate control/data wiring.

### Compression

`cur` is the four-byte window at `p`; `q` is the end of the current candidate or match; `e` holds the
table entry or, after a match, the positive offset. The hash is
`(x ^ (x >> 7) ^ (x >> 13) ^ (x >> 21)) & 16'hffff`.

1. **RELOAD/LOAD:** When `w - p >= 12`, read four bytes at `p` into `cur`, then set `q` to the
   post-read `ptr` (`p + 4`) and access the hash table. If input is available, store it at `ptr=w`,
   set `w` to the advanced pointer, and clear `in_full`. If EOF arrives before another window, emit
   the final literal-only sequence.
2. **HASH/CHECK:** Read the two-byte table entry, then write `p` into that entry. If `e < p`, read
   four bytes at `e` and compare all four against `cur`. Any positive distance is eligible, including
   overlap distances 1–3, to match `test/golden.py`. A full match saves offset `p-e` and sets `q=p+4`;
   a miss advances the search.
3. **ADV:** If `w - q >= 9`, advance `p` by one, read one byte at `q` into the sliding window, update
   `q` from the post-read pointer, and hash again. Input bytes are serviced in the wait states. EOF
   selects final emission.
4. **EXT:** If `w - q >= 9`, read four reference bytes at `q-e`, then compare up to four bytes at `q`
   against that word. A matching byte advances `ptr`; the first mismatching byte does not. Set `q`
   from the resulting pointer and repeat only when all four bytes matched. EOF or mismatch ends the
   match.
5. **EMIT:** Compute literal length `ll=p-anchor` and match length `ml=q-p`. Emit token
   `{min(ll,15), min(ml-4,15)}`. For `ll >= 15`, emit extension bytes for `ll-15` (255 while the
   remaining value is at least 255, then the residual). Emit literals from `anchor` through `p-1`,
   then offset low byte and high byte. For `ml >= 19`, emit extension bytes for `ml-19`, again in
   255-byte chunks and a residual. Then set `p` and `anchor` to `q` and resume search.
6. **FINAL:** At EOF, emit a literal-only token and any literal-length extensions for `w-anchor`,
   emit those literals, then finish. Empty input emits `0x00`.

Length counters reuse `cur`. Because literal emission reuses its low half, match-extension count is
recomputed from `q-p-19` after the offset bytes and before its extension is emitted.

### Decompression

`q` is the output position. `p[3:0]` retains the token's match-length nibble, and `cur[15:0]` is
reused as the current literal or match count.

1. **TOKEN:** Wait for a token; EOF here finishes the block. Save the low token nibble and load the
   high nibble as the literal count.
2. **LITERAL LENGTH:** If the high nibble is 15, add extension bytes until a byte other than 255;
   EOF or 16-bit overflow is an error.
3. **LITERALS:** For each literal, write `ibyte` at output address `q`, return it through `out_byte`,
   advance `q` from the post-write pointer, and decrement the count. EOF before the count reaches zero
   is an error.
4. **OFFSET:** After literals, EOF finishes the block; otherwise read a little-endian offset. Offset
   zero or greater than `q` is an error.
5. **MATCH LENGTH/COPY:** Start with `p[3:0] + 4`; when that nibble is 15, add extension bytes to
   base length 19. For each match byte, read from `q-e`, write that byte at `q`, return it through
   `out_byte`, advance `q`, and decrement the count. Re-reading after each write supports overlapping
   offsets 1–3. EOF inside a match or 16-bit output-position overflow is an error. A completed copy
   returns to TOKEN.

On error, the engine stops accepting input and reports both `error` and `done` after any held output
byte is drained.

## Verification and area target

The Cocotb suite compares compression byte-for-byte against `test/golden.py`, checks LZ4 library
round-trips, decompression including overlapping copies and malformed offsets, interleaved input and
output, and page crossings. `test/ly68l6400_model.v` is the vendored QPI-only model; the Python table
mirror decodes its initial `mem[i] = i & 8'hff` contents with the nibble ordering above. The model
does not emulate page wrap, but the page-crossing test still exercises the controller's restart path.

The design targets one 1×1 Tiny Tapeout tile at 50 MHz (approximately 16.5k µm² placeable area).
Flattened Yosys mapping to `sky130_fd_sc_hd__tt_025C_1v80` reports 16,862.42 µm² and 221 flops.
Standalone module estimates are 12,946.17 µm² for the engine, 1,661.59 µm² for the PSRAM
controller, and 1,144.85 µm² for SPI; these independent estimates do not sum to the flattened top
area. The engine is the dominant contributor. The mapped area is below the v1 result of 22,661.73
µm² but remains above both the 14k µm² optimization target and the approximate 16.5k µm² tile area.

The full RTL suite passes with both 18 ns / `rd_late=1` and 3 ns / `rd_late=0` PSRAM-read
configurations. The cycles/byte figures logged by `test.py` are limited by the testbench SPI host
(one status-poll transaction per byte), not by the engine.
