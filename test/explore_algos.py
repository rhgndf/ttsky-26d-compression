"""Compression-ratio comparison of LZ4 against other hardware-friendly algorithms."""
import sys, os, zlib, lz4.block
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

def lzss_bits(data, wbits, lbits, minm, probe):
    W, Lmax = 1 << wbits, (1 << lbits) - 1 + minm
    n, i, bits, table = len(data), 0, 0, {}
    def h(k): return data[k:k+minm]
    while i < n:
        best_len, best_off = 0, 0
        if i + minm <= n:
            cands = [table.get(h(i))] if probe == 'hash' else range(max(0, i - W), i)
            for c in cands:
                if c is None or i - c > W or c >= i: continue
                l = 0
                while i + l < n and l < Lmax and data[c + l] == data[i + l]: l += 1
                if l > best_len: best_len, best_off = l, i - c
        if best_len >= minm:
            bits += 1 + wbits + lbits
            for k in range(i, i + best_len):
                if k + minm <= n: table[h(k)] = k
            i += best_len
        else:
            bits += 9
            if i + minm <= n: table[h(i)] = i
            i += 1
    return (bits + 7) // 8

def packbits(data):
    out, i, n = 0, 0, len(data)
    while i < n:
        r = 1
        while i + r < n and r < 128 and data[i + r] == data[i]: r += 1
        if r >= 2: out += 2; i += r; continue
        j = i
        while j < n and j - i < 128 and not (j + 1 < n and data[j + 1] == data[j]): j += 1
        out += 1 + (j - i); i = j
    return out

def load(p): return open(p, 'rb').read()[:65535]
HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'src')
corpora = {
    'GPL-3 text': load('/usr/share/common-licenses/GPL-3'),
    'Verilog src': b''.join(open(os.path.join(SRC, f), 'rb').read()
                            for f in ('lz4_engine.v', 'psram_qpi.v', 'spi_slave.v'))[:65535],
    'x86 binary (ls)': load('/usr/bin/ls'),
    'random': os.urandom(16384),
}
algos = [
    ('HW LZ4 greedy, 64K hash (this design)', lambda d: len(golden.compress(d))),
    ('lz4 reference (fast)', lambda d: len(lz4.block.compress(d, store_size=False))),
    ('lz4 HC level 12', lambda d: len(lz4.block.compress(d, mode='high_compression', compression=12, store_size=False))),
    ('LZSS 256B window, hash probe', lambda d: lzss_bits(d, 8, 4, 3, 'hash')),
    ('LZSS 1KB window, hash probe', lambda d: lzss_bits(d, 10, 4, 3, 'hash')),
    ('LZSS 4KB window, hash probe', lambda d: lzss_bits(d, 12, 4, 3, 'hash')),
    ('LZSS 256B window, full search', lambda d: lzss_bits(d, 8, 4, 2, 'full')),
    ('PackBits RLE', packbits),
    ('deflate level 9 (reference only)', lambda d: len(zlib.compress(d, 9)) - 6),
]
print('| algorithm | ' + ' | '.join(f'{k} ({len(v)} B)' for k, v in corpora.items()) + ' |')
print('|---|' + '---:|' * len(corpora))
for name, f in algos:
    row = []
    for k, d in corpora.items():
        row.append(f'{100 * f(d) / len(d):.1f}%')
    print(f'| {name} | ' + ' | '.join(row) + ' |')
