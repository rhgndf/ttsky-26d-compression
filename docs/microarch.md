# Micro-architecture v2 (area-driven rewrite)

v1 synthesized to 23.9k um2 (332 flops) vs ~16.5k um2 available. v2 keeps the external behaviour
(pins, host SPI protocol, LZ4 output identical to `test/golden.py`, block <= 65535) and changes the
internals. Target: < ~200 flops, < ~13k um2 synthesized.

## Global rules
- Async/sync reset only on control state (FSM state, flags, valid bits, sync flops, PSRAM CS/SCK).
  Datapath registers (pointers, cur, ibyte, out_byte, shift regs) have no reset.
- No registered copies of things that can be combinational. No duplicated adders.

## SPI slave (~25 flops)
- One 8-bit shift register `sr` for both directions. On CS_n fall: `sr <= status`, `cnt <= 0`,
  snapshot `st_in_ready`, `st_out_valid` (2 flops). On each synchronized SCK rise: `sr <= {sr[6:0], mosi}`,
  cnt++ (saturate at 16). MISO = sr[7] (it changes ~2-3 clk after the rising edge, which is fine for
  mode 0 since the host samples on the next rising edge). On the 8th rise, instead of shifting:
  capture cmd bits {start=sr[6]... i.e. the 8 received bits} into a 5-bit cmd reg (START, MODE, END,
  POP, PUSH) and load `sr <= out_byte` (the bit just received is part of cmd: cmd = {sr[6:0], mosi}).
- On the 16th rise the command is applied: outputs `push`, `pop`, `end_job`, `start`, `mode` are
  combinational pulses = (sck_rise && cnt == 15 && cmd bit && snapshot condition); push data =
  {sr[6:0], mosi} combinational. No registered output pulses, no separate rx/tx/data registers.
- Sync: 2 flops each for SCK, CS_n, MOSI + 1 "prev" for SCK and CS_n edge detection.

## PSRAM controller = nibble pipe (~20 flops, no address or data registers)
- Ports: clk, rst_n, rd_late, start, we, addr[17:0] (combinational from engine, must stay stable
  while busy), more, wnib[3:0] (combinational from engine), sd_in[3:0], outputs psram_sck, psram_cs_n,
  sd_out, sd_oe, ready, busy, nib_strobe, byte_done.
- Sequence per transaction: cmd nibbles (E,B for read; 0,2 for write), 6 address nibbles
  {0, 2'b0,addr[17:16]}, addr[15:12]..addr[3:0], read: 6 dummy cycles; then data nibbles.
  The read command is 0xEB as required by the QPI-only LY68L6400 test model. Use one phase counter for
  cmd/addr/dummy instead of separate counters.
- Data: for reads, `nib_strobe` pulses in the sample cycle (per rd_late) and the engine takes
  `sd_in` directly (no rdata register). For writes, `nib_strobe` pulses when the controller takes
  `wnib` (engine advances its nibble select on it). `byte_done` = nib_strobe on the second nibble.
- `more` is sampled at byte_done; page rule: if more && addr[9:0]==10'h3FF at byte_done (engine
  increments its pointer in the same cycle) -> CS high, gap, restart cmd+address with the (new)
  addr input, then continue the data phase.
- Nibble order in memory: LOW nibble first for every byte, for every access (history bytes, table
  entries). This is self-consistent because only this chip reads the PSRAM; the testbench mirror of
  the initial table contents must use the same order (byte b at an address = {first nibble, second
  nibble}, i.e. stored model byte = {n_first, n_second}).
- Initialization sends QPI command 0xF5 as two nibbles, then provides a CS-high SCK rising edge;
  it sends SPI command 0x35 MSB-first on SD0 with SD1-3 undriven, then provides another CS-high SCK
  rising edge before asserting `ready`.

## Engine registers
w, p, q, e, anchor, ptr (16 bit each); cur[31:0]; ibyte[7:0]+in_full; out_byte[7:0]+out_valid;
state; flags: started, mode, end_flag, error, ok (compare result), nib (nibble phase for writes).
- `ptr` is the ONLY PSRAM pointer: history address = {2'b00, ptr}; table address =
  {1'b1, h[15:0], 1'b0} (2-byte aligned, never crosses a page; PSRAM auto-increments within it).
  `ptr <= ptr + 1` at every byte_done of a history access (dedicated incrementer, the only one).
- Shifting in read nibbles (low nibble first): `cur <= {sd_in, cur[31:4]}` per nibble -> after 4
  bytes cur = little-endian word; 1 byte read = shift by 8 (sliding window). e (table read):
  `e <= {sd_in, e[15:4]}`. out_byte: `out_byte <= {sd_in, out_byte[7:4]}`.
- Compare of candidate bytes against cur: per nibble compare sd_in with cur[3:0] and rotate
  `cur <= {cur[3:0], cur[31:4]}` (8 rotates = original cur). For EXT, a mismatching byte aborts the
  burst (more=0) and ptr is not incremented for it (see EXT).
- Write nibble mux wnib: ibyte lo/hi (input write, decomp literal), out_byte lo/hi (decomp copy),
  p nibbles 0..3 (table write; table entry stored low nibble first => read back with the e shift).
- ONE shared 16-bit ALU: R = A + (B ^ {16{inv}}) + cin, A in {w, q, p, cur[15:0]},
  B in {q, e, ptr, p, anchor, {8'd0, byte/const}}. All comparisons are derived from R bits and the
  carry. All register updates other than ptr++/shifts go through R. Each ALU use is its own FSM
  cycle (cycles are cheap; PSRAM dominates).
- cnt for length-extension emission lives in cur[15:0] (cur is dead during emission); in decompress
  cnt lives in cur[15:0] too and the token match nibble in p[3:0] (p, anchor, w unused there except
  as listed).

## Compress FSM (q = p + 4 during search, q = next byte to extend during EXT)
START: w=p=anchor=0, cur invalid -> RELOAD.
- Input service: in the wait states marked (*), if in_full: ptr <= w; write ibyte (1 byte); w <= ptr
  (copy via R = ptr + 0), in_full <= 0. Input is not serviced during emission (host just sees
  in_ready=0 meanwhile).
- RELOAD(*): if w - p >= 12: ptr <= p; read 4 bytes into cur; q <= ptr (= p+4) -> HASH.
  elif eof -> FINAL. else wait.
- HASH: read table[h(cur)] 2 bytes into e; write p to table[h] (2 bytes).
- CHECK: R = q - e; if no borrow and R >= 5 (i.e. e < p): ptr <= e; read 4 bytes comparing with cur
  (full 4 bytes, ok flag); if ok: p stays, e <= q - ptr (= offset), -> EXT. Otherwise -> ADV.
- ADV(*): if w - q >= 9 (i.e. new p+12 <= w, since q = p+4): p <= p + 1; ptr <= q; read 1 byte (shifts into cur
  top); q <= ptr; -> HASH. elif eof -> FINAL. else wait.
- EXT(*): if w - q >= 9: ptr <= q - e; read 4 bytes into cur (candidate bytes); ptr <= q; read up to
  4 bytes comparing with cur, incrementing ptr only for matching bytes, abort at first mismatch;
  q <= ptr; all 4 matched -> EXT else EMIT. elif eof -> EMIT. else wait.
- EMIT: token = {min(p-anchor,15), min(q-p-4,15)} from R = p - anchor then R = q - p (store the ll
  nibble in out_byte while computing the ml nibble, or emit after two ALU cycles — your choice, no
  extra registers). If ll >= 15: cnt = ll; while cnt >= 270: emit 255, cnt -= 255; emit cnt-15
  (8-bit). Literals: ptr <= anchor; repeat read 1 byte into out_byte, emit, until ptr == p.
  Offset e[7:0], e[15:8]. If ml >= 19: cnt = ml; while cnt >= 274: emit 255, cnt -= 255; emit
  cnt-19 (8-bit). Then p <= q, anchor <= q -> RELOAD.
- FINAL: same emission with ll = w - anchor and no match part -> DONE.

## Decompress FSM (q = output position)
TOKEN (wait input; eof -> DONE) -> p[3:0] <= tok[3:0]; cnt <= tok[7:4]; if 15 accumulate length
bytes (cnt <= cnt + byte, until byte != 255).
LIT: per byte: ptr <= q; write ibyte; out_byte <= ibyte (take it straight from ibyte), emit;
q <= ptr; cnt--. After literals: wait input or eof (eof -> DONE).
OFFSET: e <= 2 bytes; error if e == 0 or q - e borrows. cnt <= p[3:0] + 4 (+ length bytes if 15).
COPY: per byte: ptr <= q - e; read 1 byte into out_byte; ptr <= q; write out_byte; emit; q <= ptr;
cnt--. -> TOKEN. eof mid-sequence or q overflow -> error.
