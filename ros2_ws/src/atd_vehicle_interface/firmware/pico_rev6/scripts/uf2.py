#!/usr/bin/env python3
"""Encode/verify unencrypted RP2040 UF2 at Pico's flash base.

UF2 block layout and RP2040 family identifier follow:
https://github.com/microsoft/uf2
https://github.com/microsoft/uf2/blob/master/utils/uf2families.json
This tool deliberately supports only the 2 MiB original Raspberry Pi Pico.
"""
import argparse
import hashlib
from pathlib import Path
import struct

MAGIC0 = 0x0A324655
MAGIC1 = 0x9E5D5157
MAGIC_END = 0x0AB16F30
FAMILY_RP2040 = 0xE48BFF56
FAMILY_FLAG = 0x00002000
FLASH_BASE = 0x10000000
FLASH_SIZE = 2 * 1024 * 1024
PAYLOAD = 256
BLOCK = 512


def encode(binary: bytes) -> bytes:
    if not 256 <= len(binary) <= FLASH_SIZE:
        raise ValueError("binary must contain boot2 and fit original Pico's 2 MiB flash")
    count = (len(binary) + PAYLOAD - 1) // PAYLOAD
    blocks = []
    for index in range(count):
        data = binary[index * PAYLOAD:(index + 1) * PAYLOAD].ljust(PAYLOAD, b"\x00")
        header = struct.pack("<8I", MAGIC0, MAGIC1, FAMILY_FLAG,
                             FLASH_BASE + index * PAYLOAD, PAYLOAD,
                             index, count, FAMILY_RP2040)
        blocks.append(header + data + bytes(476 - PAYLOAD) + struct.pack("<I", MAGIC_END))
    return b"".join(blocks)


def verify(binary: bytes, uf2: bytes) -> None:
    if not 256 <= len(binary) <= FLASH_SIZE:
        raise ValueError("binary size outside original Pico flash bounds")
    count = (len(binary) + PAYLOAD - 1) // PAYLOAD
    if len(uf2) != count * BLOCK:
        raise ValueError("UF2 block count/size mismatch")
    recovered = bytearray()
    for index in range(count):
        block = uf2[index * BLOCK:(index + 1) * BLOCK]
        actual = struct.unpack_from("<8I", block)
        expected = (MAGIC0, MAGIC1, FAMILY_FLAG, FLASH_BASE + index * PAYLOAD,
                    PAYLOAD, index, count, FAMILY_RP2040)
        if actual != expected or struct.unpack_from("<I", block, 508)[0] != MAGIC_END:
            raise ValueError(f"invalid RP2040 UF2 header/address at block {index}")
        if any(block[32 + PAYLOAD:508]):
            raise ValueError(f"unexpected UF2 extension data at block {index}")
        recovered.extend(block[32:32 + PAYLOAD])
    if recovered[:len(binary)] != binary or any(recovered[len(binary):]):
        raise ValueError("UF2 payload differs from built binary")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("encode", "verify"))
    parser.add_argument("binary", type=Path)
    parser.add_argument("uf2", type=Path)
    args = parser.parse_args()
    binary = args.binary.read_bytes()
    if args.action == "encode":
        output = encode(binary)
        verify(binary, output)
        args.uf2.write_bytes(output)
    else:
        verify(binary, args.uf2.read_bytes())
    print(f"RP2040 UF2 {args.action}: PASS; bin={len(binary)} bytes "
          f"sha256={hashlib.sha256(binary).hexdigest()}")


if __name__ == "__main__":
    main()
