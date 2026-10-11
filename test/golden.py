"""Bit-exact reference model of the hardware LZ4 block compressor.

Greedy, single probe, 16-bit XOR-shift hash, 64K-entry table of 16-bit
positions (initial contents arbitrary), block size <= 65535 bytes.
"""

MINMATCH = 4
MFLIMIT = 12      # a match must start at least 12 bytes before the end
LASTLITERALS = 5  # the last 5 bytes are always literals


def hw_hash(x):
    return (x ^ (x >> 7) ^ (x >> 13) ^ (x >> 21)) & 0xFFFF


def _lenext(out, r):
    while r >= 255:
        out.append(255)
        r -= 255
    out.append(r)


def compress(data, table=None):
    n = len(data)
    assert n <= 65535
    if table is None:
        import random
        rnd = random.Random(n)
        table = [rnd.randrange(65536) for _ in range(65536)]
    out = bytearray()
    p = anchor = 0
    while n - p >= MFLIMIT:
        cur = data[p:p + 4]
        h = hw_hash(int.from_bytes(cur, 'little'))
        entry = table[h]
        table[h] = p
        if entry < p and data[entry:entry + 4] == cur:
            off = p - entry
            q = p + 4
            while n - q >= 4 + LASTLITERALS:
                k = 0
                while k < 4 and data[q + k] == data[q - off + k]:
                    k += 1
                q += k
                if k < 4:
                    break
            ll, ml = p - anchor, q - p
            out.append((min(ll, 15) << 4) | min(ml - 4, 15))
            if ll >= 15:
                _lenext(out, ll - 15)
            out += data[anchor:p]
            out += bytes([off & 255, off >> 8])
            if ml - 4 >= 15:
                _lenext(out, ml - 19)
            p = anchor = q
        else:
            p += 1
    ll = n - anchor
    out.append(min(ll, 15) << 4)
    if ll >= 15:
        _lenext(out, ll - 15)
    out += data[anchor:n]
    return bytes(out)


if __name__ == '__main__':
    import sys, lz4.block, os
    files = sys.argv[1:]
    for f in files:
        d = open(f, 'rb').read()[:65535]
        c = compress(d)
        assert lz4.block.decompress(c, uncompressed_size=len(d)) == d
        ref = len(lz4.block.compress(d, store_size=False))
        print(f"{os.path.basename(f)[:20]:20} in={len(d):6} hw={len(c):6} ({len(c)/len(d):.3f}) lz4={ref:6} ({ref/len(d):.3f})")
    for n in list(range(0, 40)) + [100, 1000]:
        for d in [bytes(n), os.urandom(n), (b'ab' * n)[:n]]:
            c = compress(d)
            assert lz4.block.decompress(c, uncompressed_size=len(d)) == d, (n, d)
    print("edge cases OK")
