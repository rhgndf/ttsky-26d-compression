# Algorithm exploration

Compressed size as % of input (lower is better), from `test/explore_algos.py`. Inputs are capped at
65,535 bytes (one LZ4 block). The LZSS variants are bit-packed (1 flag bit, then 8 literal bits or
window+4 length bits), use min match 3 (full search: 2), and use the same single-probe hash
strategy as the hardware LZ4.

| algorithm | GPL-3 text (35147 B) | Verilog src (25799 B) | x86 binary (ls) (65535 B) | random (16384 B) |
|---|---:|---:|---:|---:|
| HW LZ4 greedy, 64K hash (this design) | 56.1% | 34.4% | 58.3% | 100.4% |
| lz4 reference (fast) | 54.1% | 32.4% | 60.5% | 100.4% |
| lz4 HC level 12 | 44.1% | 25.3% | 52.5% | 100.4% |
| LZSS 256B window, hash probe | 70.1% | 38.2% | 62.2% | 112.5% |
| LZSS 1KB window, hash probe | 56.0% | 32.1% | 57.0% | 112.5% |
| LZSS 4KB window, hash probe | 52.1% | 31.1% | 56.5% | 112.5% |
| LZSS 256B window, full search | 63.1% | 35.4% | 57.1% | 112.3% |
| PackBits RLE | 101.5% | 77.4% | 90.4% | 101.0% |
| deflate level 9 (reference only) | 34.5% | 18.9% | 43.9% | 100.0% |

Fit for a 1x1 tile with QPI PSRAM:
- **LZ4 (chosen):** byte-aligned tokens, so no bit packer or unpacker is needed. History and the
  hash table live in PSRAM. Standard format: the host can use stock `lz4` tools. The greedy
  single-probe hardware is 2 points behind the reference `lz4` on text and ahead on the binary.
- **LZSS, 4 KB window:** a few points better than LZ4 on these inputs, and expands random data by
  12.5% (LZ4: 0.4%). It needs a bit packer and unpacker, plus a non-standard format with host-side
  tooling. No area advantage is expected: the match finder is the same.
- **RLE (PackBits):** tiny, but useless except on runs.
- **Deflate/Huffman:** best ratio, but needs code tables in on-chip RAM, which does not exist in a
  1x1 tile. Out of scope.
