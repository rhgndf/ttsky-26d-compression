# LZ4 compressor/decompressor — implementation spec (Tiny Tapeout sky130, 1x1, 50 MHz)

Golden model: `test/golden.py`. Hardware compressor output MUST be byte-identical
to `golden.compress(data, table)` where `table` is the PSRAM hash-table contents at START
(the testbench reads it from the PSRAM model before the job, see Tests).

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

## Host SPI protocol (slave, oversampled)
- SCK, MOSI, CS_n each pass a 2-flop synchronizer. Requirements on host (document them): SCK <= clk/8,
  >= 4 clk from CS_n fall to first SCK rise, CS_n high >= 4 clk between transactions.
- MOSI sampled on (synchronized) SCK rising edge, MSB first. MISO updated on SCK falling edge, MSB first;
  MISO register drives uo0.
- On CS_n fall: snapshot `st = status`, load MISO shift reg with `st`, bit counter = 0.
- Every transaction is exactly 2 bytes: host sends [cmd, din], receives [status, dout].
- On the falling edge after the 8th rising edge, load MISO shift reg with the output holding register
  (garbage if st.out_valid was 0).
- On the 16th rising edge (din complete) apply the command (nothing is applied for a transaction that
  ends early; extra bits beyond 16 are ignored until CS_n rises):
  - cmd[7] START: reset the job: mode = cmd[6] (0 compress, 1 decompress); clear w,p,anchor, eof, done,
    error, holding regs. Ignore other bits. (START is accepted only once PSRAM init is complete;
    otherwise ignored.)
  - else cmd[0] PUSH: if st.in_ready, input holding reg <= din, in_full <= 1.
  - cmd[2] END: if st.in_ready, set end flag (after the push, if both).
  - cmd[1] POP: if st.out_valid, out_valid <= 0 (byte was returned in dout).
- status = {4'b1010, error, done, out_valid, in_ready}.
  - in_ready = started && !in_full && !end_flag && !done && !error.
  - eof (internal) = end_flag && !in_full.
  - done = engine finished && !out_valid.
- Decisions use the snapshot `st`, never live status (in_ready only drops by the host's own PUSH/END and
  out_valid only by its own POP, so the live GPIOs uo1/uo2 are also safe for the host to poll).

## PSRAM (APS6404L, QPI) controller
The controller sends QPI command `0xEB` for reads (six dummy SCK cycles) and `0x02` for writes,
followed by six MSB-first address nibbles. Its 18-bit byte address map is history
`{2'b00, pos}` and hash entry `{1'b1, h[15:0], 1'b0}`. Data is streamed low nibble first for every
byte. Since the LY68L6400 model stores the first serial nibble in `mem[addr][7:4]`, the first
transmitted nibble is the stored byte's high nibble; the engine assembles successive nibbles back
into the logical byte.

- SCK = clk/2. Outputs change while SCK is low; the PSRAM samples on SCK rising.
- `ui3` is latched as `rd_late`: 0 samples on the rising edge, 1 samples one `clk` later.
- SD output-enable is asserted during command/address/write data and released for read dummy/data
  phases. `nib_strobe` advances the nibble pipe; `byte_done` marks its second nibble.
- The controller samples `more` at byte completion. At a 1 KB page boundary it raises CS_n, clocks
  SCK high while CS_n is high, then restarts command/address using the next byte address.
- After reset, initialization sends QPI command `0xF5` as two nibbles (`F`, `5`), raises CS_n and
  provides an SCK rising edge, then sends SPI command `0x35` MSB-first on SD0 with SD1-3 undriven.
  It raises CS_n and provides another SCK rising edge before asserting `ready`. This sequence exits
  QPI mode and re-enters it for the APS6404L, which powers up in SPI mode. The host must provide the
  PSRAM power-up delay before releasing reset.
- The vendored QPI-only LY68L6400 model starts in QPI mode. Its parser is reset on each CS-high SCK
  rising edge. Both initialization transactions end before its unknown-command handling begins, so
  they produce no `unknown cmd` output.
- The controller interface is nibble-oriented: `start`, `we`, `addr[17:0]`, `more`, `wnib[3:0]`,
  `sd_in[3:0]`, `sd_out[3:0]`, `sd_oe[3:0]`, `nib_strobe`, and `byte_done`.

## Engine (single module, registers shared between modes)
Registers (16 bit unless noted): w, p, anchor, q, e (entry/offset), cnt; cur[31:0]; state; flags.
Output holding register `obyte[7:0]` + `out_valid`; engine only emits when !out_valid.
Input holding register `ibyte[7:0]` + `in_full`; engine consumes (in_full <= 0).
Max block size 65535 bytes (input for compress, output for decompress). Each block is independent;
host issues START per block.

The engine retains an operation-based FSM and registered operation address in this revision.
`docs/microarch.md` describes the intended shared-ALU/`ptr` rewrite; that architecture is not yet
fully reflected in `src/lz4_engine.v` or `src/spi_slave.v`.

### Compress (mode 0)
Between engine steps, highest priority: if in_full, write ibyte to PSRAM at {8'h00, w}, w <= w+1,
in_full <= 0 (1-byte write transaction). The engine itself:
1. LOAD: if w - p >= 12: read 4 bytes at p into cur (byte at p -> cur[7:0]). Else if eof: FINAL.
   Else wait (keep servicing input).
2. h = hash(cur) = (x ^ x>>7 ^ x>>13 ^ x>>21)[15:0] with x = cur.
3. Read 2-byte entry e at table[h]. Write p to table[h] (2-byte write).
4. If e < p: read 4 bytes at e and compare with cur byte by byte (may abort early on mismatch).
   All 4 equal -> match: e <= p - e (offset), q <= p + 4, go EXT. Otherwise p <= p+1, LOAD.
5. EXT: if w - q >= 9: read 4 bytes at q into cur, then read bytes at q - e comparing with cur bytes in
   order, k = number of leading equal bytes (stop/abort at first mismatch). q <= q + k.
   k == 4 -> EXT again, else EMIT. Else if eof: EMIT. Else wait.
6. EMIT (ll = p - anchor, ml = q - p): output token {min(ll,15), min(ml-4,15)}; if ll >= 15 output
   length bytes for r = ll-15 (255 while r >= 255, then r); output literals: read 1 byte at anchor,
   output, anchor <= anchor+1, until anchor == p; output e[7:0], e[15:8]; if ml-4 >= 15 output length
   bytes for ml-19. Then p <= q, anchor <= q, LOAD.
7. FINAL (eof): ll = w - anchor; token {min(ll,15), 0}; length bytes; literals anchor..w-1; DONE.
Note: empty input produces the single byte 0x00.

### Decompress (mode 1)
w = output position. Input bytes come from ibyte (wait while !in_full && !eof).
1. TOKEN: if eof -> DONE (block end). Else take token; ll = tok[7:4], keep tok[3:0].
2. If ll == 15: cnt = 15 + sum of following bytes until one != 255. (eof here -> error.)
3. LIT: cnt times: take input byte, write it to PSRAM at w, output it, w++. (eof here -> error.)
4. After literals: wait for input or eof; eof -> DONE. Else read offset e (2 bytes LE).
   e == 0 or e > w -> error, DONE.
5. cnt = tok[3:0] + 4, plus length bytes if tok[3:0] == 15.
6. COPY: cnt times: read 1 byte at w - e, write it at w, output it, w++. Go TOKEN.
error: set error flag, stop (in_ready = 0, done=1 once output drained).

## Area budget
1x1 tile, core ~16.5k um2 placeable. Keep it small: reuse adders/subtractors where natural, no extra
buffers, async-reset-free flops where reset isn't needed.
