#!/usr/bin/env python3
"""Cross-check firmware GPIO numbers against the completed native netlist.

This verifies the U1 pin map, not soldering, final PCB continuity or voltages.
"""
from pathlib import Path
import argparse
import json
import os
import re
import xml.etree.ElementTree as ET

HEADER = Path(__file__).resolve().parents[1] / "include/rev6_hal.h"
NAMES = {
    "SBUS": "RX3_N", "SERVO": "MCU_SERVO_PWM_N", "ESC": "MCU_ESC_PWM_N",
    "UV_GOOD": "UV_GOOD_MCU", "IMU_SCK": "IMU_SCK_MCU", "IMU_MOSI": "IMU_MOSI_MCU",
    "IMU_MISO": "IMU_MISO_MCU", "IMU_CS": "IMU_CS_MCU", "IMU_INT": "IMU_INT_MCU",
    "RPM": "RPM_N", "VBEC_ADC": "VBEC_SENSE",
}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--board-dir", type=Path, default=os.environ.get("JPBB_REV6_BOARD_DIR") or None,
                        help="Rev6 KiCad board directory containing review/netlist.xml and rev6_interface_contract.json")
    args = parser.parse_args()
    if args.board_dir is None:
        parser.error("Specify --board-dir or JPBB_REV6_BOARD_DIR; hardware files are not bundled")
    board = args.board_dir.expanduser().resolve()
    header = HEADER.read_text()
    nets = ET.parse(board / "review/netlist.xml").getroot().find("nets")
    actual = {}
    for net in nets:
        for node in net.findall("node"):
            if node.attrib["ref"] == "U1":
                match = re.match(r"GP(\d+)_", node.attrib.get("pinfunction", ""))
                if match and "no_connect" not in node.attrib.get("pintype", ""):
                    actual[net.attrib["name"]] = int(match.group(1))
    for label, net in NAMES.items():
        expected = re.search(rf"REV6_GPIO_{label}\s*=\s*(\d+)", header)
        assert expected, label
        # Native hierarchy names may be prefixed by a sheet path.
        matches = [gpio for name, gpio in actual.items() if name.rsplit("/", 1)[-1] == net]
        assert len(matches) == 1, (net, actual)
        assert int(expected.group(1)) == matches[0], (label, expected.group(1), matches)
    contract = json.loads((board / "rev6_interface_contract.json").read_text())
    assert contract["pico_main_gpio_map_retained"]["UV_GOOD"] == 9
    print("Rev6 firmware/native-netlist GPIO contract: PASS (11 signals, GP9 UV_GOOD)")

if __name__ == "__main__":
    main()
