"""Check the MCU pin table, schematic and PCB against the integrated-board map."""

from __future__ import annotations

from collections import defaultdict
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
Pin = tuple[str, str]

# Signal: (GPIO number, WROOM-2 module pad number).
EXPECTED_SIGNALS = {
    "M1_PWM": (8, 12), "M1_DIR": (18, 11),
    "M1_ENC_A": (17, 10), "M1_ENC_B": (16, 9),
    "M2_PWM": (15, 8), "M2_DIR": (7, 7),
    "M2_ENC_A": (6, 6), "M2_ENC_B": (5, 5),
    "M3_PWM": (12, 20), "M3_DIR": (11, 19),
    "M3_ENC_A": (10, 18), "M3_ENC_B": (9, 17),
    "M4_PWM": (38, 31), "M4_DIR": (21, 23),
    "M4_ENC_A": (14, 22), "M4_ENC_B": (13, 21),
    "I2C0_SDA": (1, 39), "I2C0_SCL": (2, 38),
    "LED_DATA": (4, 4),
    "I2C1_SDA": (41, 34), "I2C1_SCL": (42, 35),
    "USB_D_N": (19, 13), "USB_D_P": (20, 14), "BOOT_N": (0, 27),
    "BAT_ADC": (3, 15),
}
UNUSED_PADS = {
    "16": "IO46", "26": "IO45",
    "28": "NC", "29": "NC", "30": "NC",
    "24": "IO47", "25": "IO48",
    "32": "MTCK/IO39", "33": "MTDO/IO40",
    "36": "RXD0/IO44", "37": "TXD0/IO43",
}
CONNECTORS = {
    "J1": {"1": "RAW", "2": "GND"},
    "J2": {
        "1": "VBUS", "2": "/USB_CONN_N", "3": "/USB_CONN_P",
        "4": "GND", "SH": "GND",
    },
    "J3": {"1": "+5V_LED1", "2": "GND", "3": "LED_DIN"},
    **{
        f"J{led + 2}": {"1": f"+5V_LED{led}", "2": "GND"}
        for led in range(2, 5)
    },
    **{
        f"J{motor + 6}": {
            "1": f"M{motor}_PWM", "2": "GND", "3": f"M{motor}_DIR",
        }
        for motor in range(1, 5)
    },
    **{
        f"J{motor + 10}": {
            "1": "+3V3", "2": "GND",
            "3": f"M{motor}_ENC_A", "4": f"M{motor}_ENC_B",
        }
        for motor in range(1, 5)
    },
    **{
        f"J{tof + 14}": {
            "1": "GND",
            "2": f"/Sensors / BNO055 and ToF/TOF{tof}_SCL",
            "3": f"/Sensors / BNO055 and ToF/TOF{tof}_SDA",
            "4": "+3V3",
        }
        for tof in range(1, 5)
    },
    "JP1": {"1": "VBUS", "2": "Net-(D2-A)"},
    "SW1": {"1": "GND", "2": "BOOT_N"},
    "SW2": {"1": "GND", "2": "CHIP_PU"},
    "SW3": {"1": "VIN_LOGIC", "2": "Net-(U3-EN)"},
}


def pin_table(text: str) -> dict[str, tuple[int, int]]:
    """Read the MCU table without hiding duplicate signals, GPIOs or pads."""
    assignments: dict[str, tuple[int, int]] = {}
    gpios: set[int] = set()
    pads: set[int] = set()
    in_table = False
    for line in text.splitlines():
        if not line.strip().startswith("|"):
            if in_table:
                break
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if not in_table:
            in_table = cells[:3] == ["신호명", "GPIO", "패드"]
            continue
        if all(re.fullmatch(r":?-+:?", cell) for cell in cells):
            continue
        if (
            len(cells) != 5
            or re.fullmatch(r"[A-Z][A-Z0-9_]*", cells[0]) is None
            or any(re.fullmatch(r"[0-9]+", cell) is None for cell in cells[1:3])
        ):
            raise ValueError(f"Malformed MCU pin row: {line}")
        signal, gpio, pad = cells[0], int(cells[1]), int(cells[2])
        if signal in assignments:
            raise ValueError(f"Duplicate signal: {signal}")
        if gpio in gpios:
            raise ValueError(f"Duplicate GPIO: {gpio}")
        if pad in pads:
            raise ValueError(f"Duplicate pad: {pad}")
        assignments[signal] = (gpio, pad)
        gpios.add(gpio)
        pads.add(pad)
    if not assignments:
        raise ValueError("Missing or empty MCU pin table")
    return assignments


class PinTableTest(unittest.TestCase):
    HEADER = (
        "| 신호명 | GPIO | 패드 | 입출력 | 연결 대상 및 설명 |\n"
        "|---|---:|---:|---|---|\n"
    )
    ROW = "| BOOT_N | 0 | 27 | Input | Boot switch |\n"

    def test_gpio_zero_and_unrelated_tables(self) -> None:
        text = (
            "| GPIO | 상태 | 사유 |\n| 3 | NC | Strap |\n\n"
            + self.HEADER + self.ROW
            + "\n| 패드 번호 | 신호명 | 기능 |\n| 2 | +3V3 | Supply |\n"
        )
        self.assertEqual(pin_table(text), {"BOOT_N": (0, 27)})

    def test_missing_empty_and_malformed_tables_are_rejected(self) -> None:
        for text in (
            "", self.HEADER,
            self.HEADER + "| M1_PWM | GPIO8 | 12 | Output | PWM |\n",
            self.HEADER + "| M1_PWM | 8 | x | Output | PWM |\n",
            self.HEADER + "| M1_PWM | 8 | 12 |\n",
            self.HEADER + "| | 8 | 12 | Output | PWM |\n",
        ):
            with self.subTest(text=text):
                with self.assertRaises(ValueError):
                    pin_table(text)

    def test_duplicate_signals_gpios_and_pads_are_rejected(self) -> None:
        for row, message in (
            ("| BOOT_N | 8 | 12 | Output | PWM |\n", "Duplicate signal"),
            ("| M1_PWM | 0 | 12 | Output | PWM |\n", "Duplicate GPIO"),
            ("| M1_PWM | 8 | 27 | Output | PWM |\n", "Duplicate pad"),
        ):
            with self.subTest(message=message):
                with self.assertRaisesRegex(ValueError, message):
                    pin_table(self.HEADER + self.ROW + row)

    def test_document_has_exact_expected_assignments(self) -> None:
        self.assertDictEqual(
            pin_table((ROOT / "PINMAP.md").read_text(encoding="utf-8")),
            EXPECTED_SIGNALS,
        )


class PinmapSyncTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        import pcbnew as k

        temp = tempfile.TemporaryDirectory(prefix="cleany-pinmap-sync-")
        cls.addClassCleanup(temp.cleanup)
        netlist = Path(temp.name) / "netlist.xml"
        result = subprocess.run(
            ["kicad-cli", "sch", "export", "netlist", "--format", "kicadxml",
             "--output", str(netlist), str(ROOT / "integrated_controller.kicad_sch")],
            capture_output=True, text=True, timeout=90, cwd=ROOT,
        )
        if result.returncode:
            raise AssertionError(f"Netlist export failed: {result.stdout}{result.stderr}")
        cls.netlist = ET.parse(netlist).getroot()
        cls.schematic_nets: dict[str, set[Pin]] = defaultdict(set)
        cls.schematic_pins: dict[Pin, set[str]] = defaultdict(set)
        for net in cls.netlist.findall("nets/net"):
            name = net.attrib["name"]
            for node in net.findall("node"):
                pin = (node.attrib["ref"], node.attrib["pin"])
                cls.schematic_nets[name].add(pin)
                cls.schematic_pins[pin].add(name)

        cls.board = k.LoadBoard(str(ROOT / "integrated_controller.kicad_pcb"))
        cls.pcb_nets: dict[str, set[Pin]] = defaultdict(set)
        cls.pcb_pins: dict[Pin, set[str]] = defaultdict(set)
        for footprint in cls.board.GetFootprints():
            for pad in footprint.Pads():
                if not pad.GetNumber():
                    continue
                pin = (footprint.GetReference(), pad.GetNumber())
                # XML uses literal slashes; PCB names escape slashes in labels.
                name = pad.GetNetname().replace("{slash}", "/")
                cls.pcb_nets[name].add(pin)
                cls.pcb_pins[pin].add(name)

    def test_signal_gpio_and_pad_match_document_schematic_and_pcb(self) -> None:
        documented = pin_table((ROOT / "PINMAP.md").read_text(encoding="utf-8"))
        self.assertDictEqual(documented, EXPECTED_SIGNALS)
        for signal, (gpio, pad) in EXPECTED_SIGNALS.items():
            with self.subTest(signal=signal, gpio=gpio, pad=pad):
                pin = ("U1", str(pad))
                self.assertEqual(self.schematic_pins[pin], {signal})
                self.assertEqual(self.pcb_pins[pin], {signal})
                nodes = self.netlist.findall(f"nets/net/node[@ref='U1'][@pin='{pad}']")
                self.assertEqual(len(nodes), 1)
                self.assertRegex(
                    nodes[0].attrib["pinfunction"],
                    rf"^(?:[^/]+/)?IO{gpio}(?:_{pad})?$",
                )

    def test_all_u1_pad_assignments_including_power_and_reserved_pins(self) -> None:
        expected = {
            ("U1", str(pad)): {signal}
            for signal, (_, pad) in EXPECTED_SIGNALS.items()
        }
        expected.update({
            ("U1", pad): {f"unconnected-(U1-{function}-Pad{pad})"}
            for pad, function in UNUSED_PADS.items()
        })
        expected.update({
            ("U1", "1"): {"GND"}, ("U1", "2"): {"+3V3"},
            ("U1", "3"): {"CHIP_PU"}, ("U1", "40"): {"GND"},
            ("U1", "41"): {"GND"},
        })
        self.assertEqual(len(expected), 41)
        for source, pins in (
            ("schematic", self.schematic_pins), ("PCB", self.pcb_pins),
        ):
            with self.subTest(source=source):
                self.assertDictEqual(
                    {pin: nets for pin, nets in pins.items() if pin[0] == "U1"},
                    expected,
                )

    def test_reserved_pins_have_no_connect_markers_and_no_other_members(self) -> None:
        for pad, function in UNUSED_PADS.items():
            with self.subTest(pad=pad, function=function):
                nodes = self.netlist.findall(f"nets/net/node[@ref='U1'][@pin='{pad}']")
                self.assertEqual(len(nodes), 1)
                self.assertRegex(
                    nodes[0].attrib["pinfunction"],
                    rf"^{re.escape(function)}(?:_{pad})?$",
                )
                self.assertIn("no_connect", nodes[0].attrib["pintype"].split("+"))
                net = f"unconnected-(U1-{function}-Pad{pad})"
                self.assertEqual(self.schematic_nets[net], {("U1", pad)})
                self.assertEqual(self.pcb_nets[net], {("U1", pad)})

    def test_usb_differential_pair_names_and_polarity(self) -> None:
        expected = {
            "USB_D_N": {("U1", "13"), ("U2", "6")},
            "USB_D_P": {("U1", "14"), ("U2", "4")},
            "/USB_CONN_N": {("J2", "2"), ("U2", "1")},
            "/USB_CONN_P": {("J2", "3"), ("U2", "3")},
        }
        for source, nets in (
            ("schematic", self.schematic_nets), ("PCB", self.pcb_nets),
        ):
            with self.subTest(source=source):
                self.assertDictEqual(
                    {name: pins for name, pins in nets.items()
                     if "USB" in name or name.startswith("Net-(J2-D")},
                    expected,
                )
                for prefix in ("USB_D", "/USB_CONN"):
                    self.assertIn(f"{prefix}_N", nets)
                    self.assertIn(f"{prefix}_P", nets)

    def test_external_connector_pad_contract_is_unchanged(self) -> None:
        expected = {
            (ref, pad): {net}
            for ref, pads in CONNECTORS.items() for pad, net in pads.items()
        }
        for source, pins in (
            ("schematic", self.schematic_pins), ("PCB", self.pcb_pins),
        ):
            with self.subTest(source=source):
                self.assertDictEqual(
                    {
                        pin: nets for pin, nets in pins.items()
                        if re.fullmatch(r"(?:J|JP|SW)[0-9]+", pin[0])
                    },
                    expected,
                )


if __name__ == "__main__":
    unittest.main()
