#!/usr/bin/env python3
"""Reject artifacts that could target the wrong chip/address or lose code bytes."""
import importlib.util
from pathlib import Path
import struct
import unittest

spec = importlib.util.spec_from_file_location(
    "jpbb_uf2", Path(__file__).resolve().parents[1] / "scripts" / "uf2.py")
uf2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(uf2)


class RP2040ArtifactTest(unittest.TestCase):
    def setUp(self):
        self.binary = bytes(range(256)) * 2 + b"last-byte!"
        self.image = uf2.encode(self.binary)

    def test_non_aligned_image_roundtrip(self):
        uf2.verify(self.binary, self.image)
        self.assertEqual(len(self.image), 3 * 512)
        self.assertEqual(struct.unpack_from("<I", self.image, 12)[0], 0x10000000)
        self.assertEqual(struct.unpack_from("<I", self.image, 28)[0], 0xE48BFF56)

    def test_wrong_chip_or_address_and_corrupt_payload_rejected(self):
        for offset, value in ((28, 0xE48BFF59), (12, 0x10000100),
                              (20, 1), (24, 99), (508, 0)):
            with self.subTest(offset=offset):
                image = bytearray(self.image)
                struct.pack_into("<I", image, offset, value)
                with self.assertRaises(ValueError):
                    uf2.verify(self.binary, image)
        image = bytearray(self.image)
        image[32 + 100] ^= 1
        with self.assertRaises(ValueError):
            uf2.verify(self.binary, image)

    def test_incomplete_duplicate_and_wrong_binary_rejected(self):
        for image in (self.image[:-1], self.image + self.image[:512],
                      self.image[:512] + self.image[:512] + self.image[1024:]):
            with self.assertRaises(ValueError):
                uf2.verify(self.binary, image)
        with self.assertRaises(ValueError):
            uf2.verify(self.binary + b"\xFF", self.image)

    def test_out_of_flash_and_no_boot2_rejected(self):
        for size in (0, 255, uf2.FLASH_SIZE + 1):
            with self.assertRaises(ValueError):
                uf2.encode(bytes(size))


if __name__ == "__main__":
    unittest.main()
