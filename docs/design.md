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
Ground truth for commands: MichaelBell/tinyQV `cpu/qspi_ctrl.v` (silicon-proven on the TT QSPI Pmod):
QPI read = 0x0B, 6 address nibbles (24-bit, MSB first), 4 dummy SCK cycles, then data;
QPI write = 0x02, 24-bit address, data. Data byte order: high nibble first.
- SCK = clk/2 (toggle register). Outputs change when SCK goes low; PSRAM samples on SCK rising.
- Read sampling: with ui3-latched `rd_late`=0 sample the SD lines in the clk edge where SCK rises;
  with `rd_late`=1 sample one clk later. (TT mux round-trip is ~20 ns, see tinyQV comments.)
- uio_oe for SD lines: driven during command/address/write data, released (0) from the start of the
  dummy cycles of a read until CS_n rises. CS_n high for >= 2 clk between transactions.
- Init after reset (once): send 0xF5 in QPI (exit QPI: 2 SCK cycles, nibbles F,5 on SD3:0), CS high,
  then 0x35 in SPI mode (enter QPI: 8 SCK cycles, MSB first on SD0 only, other SD lines driven 1 or
  released), CS high. Then `ready`=1. (Host guarantees the 150 us PSRAM power-up delay before releasing
  rst_n.) This makes init work whether the PSRAM was in SPI or QPI mode.
- The APS6404 wraps bursts at 1 KB page boundaries. The controller must restart the transaction (CS high,
  new command+address) whenever the address of the next byte crosses into a new 1 KB page.
- Engine interface (byte stream):
  - `start` (1-cycle, only when !busy) with `we`, `addr[23:0]`.
  - `bdone` 1-cycle pulse when a data byte completes; for reads `rdata[7:0]` valid then.
  - writes: `wdata[7:0]` sampled at the start of each data byte (i.e. engine may update it in the cycle
    of the previous `bdone`).
  - `more`: sampled in the `bdone` cycle; 0 ends the transaction after this byte, 1 continues with
    the next consecutive address. Engine drives `more` combinationally from its state.
  - `busy` until CS_n has risen and the min CS high time elapsed.
- Address map: history byte at position pos (16 bit) = {8'h00, pos}; hash entry h (16 bit), 2 bytes
  little-endian = 24'h020000 + {h, 1'b0}.

## Engine (single module, registers shared between modes)
Registers (16 bit unless noted): w, p, anchor, q, e (entry/offset), cnt; cur[31:0]; state; flags.
Output holding register `obyte[7:0]` + `out_valid`; engine only emits when !out_valid.
Input holding register `ibyte[7:0]` + `in_full`; engine consumes (in_full <= 0).
Max block size 65535 bytes (input for compress, output for decompress). Each block is independent;
host issues START per block.

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
