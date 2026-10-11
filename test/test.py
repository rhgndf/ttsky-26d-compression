import os
import random
from pathlib import Path

import cocotb
import lz4.block
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer
from cocotb.utils import get_sim_time

import golden


GATES = os.environ.get("GATES") == "yes"
CLOCK_PERIOD_NS = 20
HASH_BASE = 0x020000
RD_LATE = int(os.environ.get("PSRAM_RD_LATE", "1"))


def initial_byte(address):
    return address & 0xFF


def load_hash_table_from_model(dut, check_initial=True):
    memory = dut.psram_model.mem
    table = [0] * 65536
    for index in range(65536):
        address = HASH_BASE + 2 * index
        byte0 = int(memory[address].value)
        byte1 = int(memory[address + 1].value)
        if check_initial:
            assert byte0 == initial_byte(address)
            assert byte1 == initial_byte(address + 1)
        n0 = (byte0 >> 4) & 0xF
        n1 = byte0 & 0xF
        n2 = (byte1 >> 4) & 0xF
        n3 = byte1 & 0xF
        table[index] = (n3 << 12) | (n2 << 8) | (n1 << 4) | n0
    return table


def gpio_status(dut):
    pins = int(dut.uo_out.value)
    return (
        0xA0
        | (((pins >> 4) & 1) << 3)
        | (((pins >> 3) & 1) << 2)
        | (((pins >> 2) & 1) << 1)
        | ((pins >> 1) & 1)
    )


async def reset_design(dut, rd_late, delay_ns, powerup_delay=False):
    dut.ui_in.value = (rd_late << 3) | 0x04
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    if powerup_delay:
        await ClockCycles(dut.clk, 7490)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 40)
    assert (int(dut.uio_out.value) & 0x81) == 0x81


async def spi_transaction(dut, command=0, data=0, rd_late=1):
    ui_base = rd_late << 3
    dut.ui_in.value = ui_base | 0x04
    await ClockCycles(dut.clk, 4)
    dut.ui_in.value = ui_base
    await ClockCycles(dut.clk, 4)

    received = []
    for outgoing in (command, data):
        incoming = 0
        for bit in range(7, -1, -1):
            value = (outgoing >> bit) & 1
            dut.ui_in.value = ui_base | (value << 1)
            await ClockCycles(dut.clk, 1)
            incoming = (incoming << 1) | (int(dut.uo_out.value) & 1)
            dut.ui_in.value = ui_base | 0x01 | (value << 1)
            await ClockCycles(dut.clk, 4)
            dut.ui_in.value = ui_base | (value << 1)
            await ClockCycles(dut.clk, 4)
        received.append(incoming)

    dut.ui_in.value = ui_base | 0x04
    await ClockCycles(dut.clk, 4)
    assert (received[0] & 0xF0) == 0xA0
    return received[0], received[1]


async def run_job(dut, payload, mode, rd_late, poll_status=True, expect_error=False):
    payload = bytes(payload)
    output = bytearray()
    index = 0
    ended = False
    error_seen = False
    start_ns = int(get_sim_time(unit="ns"))

    await spi_transaction(dut, 0x80 | (mode << 6), 0, rd_late)
    rounds = 0
    while rounds < int(os.environ.get("JOB_ROUND_LIMIT", "200000")):
        rounds += 1
        if poll_status:
            status, _ = await spi_transaction(dut, 0, 0, rd_late)
        else:
            await ClockCycles(dut.clk, 1)
            status = gpio_status(dut)

        if status & 0x08:
            error_seen = True

        if status & 0x02:
            _, result_byte = await spi_transaction(dut, 0x02, 0, rd_late)
            output.append(result_byte)
        elif index < len(payload) and (status & 0x01):
            command = 0x01
            if index == len(payload) - 1:
                command |= 0x04
                ended = True
            await spi_transaction(dut, command, payload[index], rd_late)
            index += 1
        elif index == len(payload) and not ended and (status & 0x01):
            await spi_transaction(dut, 0x04, 0, rd_late)
            ended = True
        elif status & 0x04:
            break
        else:
            await ClockCycles(dut.clk, 1)
    else:
        raise AssertionError(
            f"LZ4 job timed out: input={index}/{len(payload)}, "
            f"output={len(output)}, status=0x{status:02x}, "
            f"engine_state={int(dut.user_project.engine.state.value)}, "
            f"ps_state={int(dut.user_project.psram.state.value)}, "
            f"ps_busy={int(dut.user_project.psram_busy.value)}, "
            f"psram_sck={int(dut.user_project.psram_sck.value)}, "
            f"psram_cs_n={int(dut.user_project.psram_cs_n.value)}, "
            f"w={int(dut.user_project.engine.w.value)}, "
            f"p={int(dut.user_project.engine.p.value)}, "
            f"q={int(dut.user_project.engine.q.value)}, "
            f"ptr={int(dut.user_project.engine.ptr.value)}, "
            f"cur=0x{int(dut.user_project.engine.cur.value):08x}, "
            f"in_full={int(dut.user_project.engine.in_full.value)}, "
            f"end_flag={int(dut.user_project.engine.end_flag.value)}, "
            f"started={int(dut.user_project.engine.started.value)}, "
            f"error={int(dut.user_project.engine.error.value)}, "
            f"in_ready={int(dut.user_project.in_ready.value)}, "
            f"spi_snapshot={int(dut.user_project.spi.st_in_ready.value)}, "
            f"spi_sr=0x{int(dut.user_project.spi.sr.value):02x}, "
            f"status=0x{int(dut.user_project.status.value):02x}, "
            f"byte_done={int(dut.user_project.ps_bdone.value)}"
        )

    elapsed_cycles = (
        int(get_sim_time(unit="ns")) - start_ns
    ) // CLOCK_PERIOD_NS
    assert error_seen == expect_error, (
        f"unexpected error={error_seen}, state={int(dut.user_project.engine.state.value)}, "
        f"p={int(dut.user_project.engine.p.value)}, "
        f"q={int(dut.user_project.engine.q.value)}, "
        f"e=0x{int(dut.user_project.engine.e.value):04x}, "
        f"cur=0x{int(dut.user_project.engine.cur.value):08x}, "
        f"input={index}/{len(payload)}, output={len(output)}"
    )
    assert (int(dut.uio_out.value) & 0x81) == 0x81
    return bytes(output), elapsed_cycles


def overlapping_block(offset, seed, match_length=300):
    encoded = bytearray([(len(seed) << 4) | 0x0F])
    encoded.extend(seed)
    encoded.extend((offset, 0))
    remaining = match_length - 19
    while remaining >= 255:
        encoded.append(255)
        remaining -= 255
    encoded.append(remaining)
    encoded.append(0x50)
    encoded.extend(b"12345")
    expected = bytearray(seed)
    for _ in range(match_length):
        expected.append(expected[-offset])
    expected.extend(b"12345")
    return bytes(encoded), bytes(expected)


@cocotb.test()
async def codec_test(dut):
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, unit="ns").start())
    await Timer(1, unit="ns")

    await reset_design(dut, rd_late=RD_LATE, delay_ns=18, powerup_delay=True)
    hash_table = load_hash_table_from_model(dut)

    rng = random.Random(0x4C5A34)
    license_path = Path(__file__).resolve().parents[1] / "LICENSE"
    license_text = license_path.read_bytes() if license_path.exists() else (
        b"Copyright 2026. This implementation is licensed under Apache-2.0. "
    )
    text_data = (license_text * 2)[:2048]
    random_data = bytes(rng.randrange(256) for _ in range(2300))
    long_prefix = bytes(rng.randrange(256) for _ in range(320))
    long_data = long_prefix + (b"XYZ" * 220)
    repeat_data = b"ab" * 512

    if GATES:
        compress_cases = [b"", b"gate-level LZ4 test " * 8]
    else:
        compress_cases = (
            [b""] + [bytes([value]) * value for value in range(1, 21)]
            + [bytes(256), random_data, repeat_data, text_data, long_data]
        )

    compressed_outputs = []
    throughput = {}
    for data in compress_cases:
        expected = golden.compress(data, hash_table)
        actual, cycles = await run_job(
            dut, data, mode=0, rd_late=RD_LATE, poll_status=True
        )
        assert actual == expected, (
            f"compress mismatch for {len(data)} input bytes: "
            f"{actual[:32].hex()} != {expected[:32].hex()}"
        )
        assert lz4.block.decompress(
            actual, uncompressed_size=len(data)
        ) == data
        compressed_outputs.append((data, actual))
        if data is text_data or data is random_data:
            throughput["text" if data is text_data else "random"] = (
                cycles / len(data)
            )

    if "text" in throughput:
        dut._log.info("compress cycles/input byte: text %.2f", throughput["text"])
    if "random" in throughput:
        dut._log.info(
            "compress cycles/input byte: random %.2f", throughput["random"]
        )

    await reset_design(dut, rd_late=RD_LATE, delay_ns=3)
    if GATES:
        decompress_cases = [
            (b"small gate decompress " * 8, "default"),
            (compress_cases[-1], "hardware"),
        ]
    else:
        decompress_cases = [
            (text_data[:512], "default"),
            (random_data[:256], "high"),
            (compressed_outputs[-2][0], "hardware"),
        ]

    decompress_cycles = None
    for data, source in decompress_cases:
        if source == "hardware":
            compressed = next(
                encoded for original, encoded in compressed_outputs
                if original == data
            )
        elif source == "high":
            compressed = lz4.block.compress(
                data,
                mode="high_compression",
                compression=12,
                store_size=False,
            )
        else:
            compressed = lz4.block.compress(data, store_size=False)
        actual, cycles = await run_job(
            dut, compressed, mode=1, rd_late=RD_LATE, poll_status=True
        )
        assert actual == data
        if decompress_cycles is None:
            decompress_cycles = cycles / len(compressed)

    dut._log.info(
        "decompress cycles/input byte: %.2f", decompress_cycles or 0.0
    )

    if not GATES:
        for offset, seed in ((1, b"A"), (2, b"AB"), (3, b"ABC")):
            encoded, expected = overlapping_block(offset, seed)
            actual, _ = await run_job(
                dut, encoded, mode=1, rd_late=RD_LATE, poll_status=False
            )
            assert actual == expected

        for malformed in (b"\x10A\x00\x00", b"\x10A\x02\x00"):
            actual, _ = await run_job(
                dut,
                malformed,
                mode=1,
                rd_late=RD_LATE,
                poll_status=False,
                expect_error=True,
            )
            assert actual == b"A"

        interleave_data = b"interleave-" * 64
        expected = golden.compress(interleave_data, hash_table)
        actual, _ = await run_job(
            dut, interleave_data, mode=0, rd_late=RD_LATE, poll_status=False
        )
        assert actual == expected


@cocotb.test()
async def quick_smoke_test(dut):
    cocotb.start_soon(Clock(dut.clk, CLOCK_PERIOD_NS, unit="ns").start())
    await Timer(1, unit="ns")
    await reset_design(dut, rd_late=RD_LATE, delay_ns=18, powerup_delay=True)
    hash_table = load_hash_table_from_model(dut, check_initial=False)

    payload = b"single-register SPI smoke test"
    expected = golden.compress(payload, hash_table)
    actual, _ = await run_job(dut, payload, mode=0, rd_late=RD_LATE)
    assert actual == expected

    encoded = lz4.block.compress(payload, store_size=False)
    actual, _ = await run_job(dut, encoded, mode=1, rd_late=RD_LATE)
    assert actual == payload
