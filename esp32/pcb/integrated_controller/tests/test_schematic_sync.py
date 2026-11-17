"""Check component inventory, BOM, enable, IMU and pull-up-only reset circuits."""

from __future__ import annotations

from collections import defaultdict
import csv
import importlib.util
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
HAS_KICAD = shutil.which("kicad-cli") and importlib.util.find_spec("pcbnew")
COMPONENT_COUNT = 107
EXPECTED_REFERENCES = {
    f"{prefix}{number}"
    for prefix, count in (
        ("U", 10), ("C", 45), ("R", 23), ("J", 18),
        ("L", 5), ("D", 2), ("SW", 3), ("JP", 1),
    )
    for number in range(1, count + 1)
}
REMOVED_SIGNALS = {"IMU_RESET_N", "MUX_RESET_N"}
SWITCH_FOOTPRINT = (
    "Connector_Molex:Molex_SPOX_5268-02A_1x02_P2.50mm_Horizontal"
)
RESISTOR_FOOTPRINT = "Resistor_SMD:R_0603_1608Metric"
REFERENCE = re.compile(r"([A-Z]+)([1-9][0-9]*)")
Pin = tuple[str, str]

# Original exported-netlist contract; only the GPIO reset net is renamed.
IMU_COMPONENTS = {
    "U8": ("BNO055", "Package_LGA:LGA-28_5.2x3.8mm_P0.5mm"),
    "C40": ("100n", "Capacitor_SMD:C_0603_1608Metric"),
    "C41": ("120n", "Capacitor_SMD:C_0603_1608Metric"),
    "C42": ("1u", "Capacitor_SMD:C_0603_1608Metric"),
    "R13": ("10k", RESISTOR_FOOTPRINT),
    "R14": ("10k", RESISTOR_FOOTPRINT),
    "R15": ("4.7k", RESISTOR_FOOTPRINT),
    "R16": ("4.7k", RESISTOR_FOOTPRINT),
}
IMU_UNUSED_PADS = {
    "1": "PIN1", "7": "PIN7", "8": "PIN8", "10": "BL_IND",
    "12": "PIN12", "13": "PIN13", "14": "INT", "15": "PIN15",
    "16": "PIN16", "21": "PIN21", "22": "PIN22", "23": "PIN23",
    "24": "PIN24", "26": "XOUT32", "27": "XIN32",
}
IMU_PAD_NETS = {
    "U8": {
        "2": "GND", "3": "+3V3", "4": "Net-(U8-~{BOOT_LOAD_PIN})",
        "5": "GND", "6": "GND", "9": "Net-(U8-CAP)",
        "11": "Net-(U8-~{RESET})", "17": "GND", "18": "GND",
        "19": "I2C1_SCL", "20": "I2C1_SDA", "25": "GND", "28": "+3V3",
        **{
            pad: f"unconnected-(U8-{function}-Pad{pad})"
            for pad, function in IMU_UNUSED_PADS.items()
        },
    },
    "C40": {"1": "+3V3", "2": "GND"},
    "C41": {"1": "+3V3", "2": "GND"},
    "C42": {"1": "Net-(U8-CAP)", "2": "GND"},
    "R13": {"1": "+3V3", "2": "Net-(U8-~{RESET})"},
    "R14": {"1": "+3V3", "2": "Net-(U8-~{BOOT_LOAD_PIN})"},
    "R15": {"1": "+3V3", "2": "I2C1_SDA"},
    "R16": {"1": "+3V3", "2": "I2C1_SCL"},
}


def bom_footprints(
    rows: list[dict[str, str]], *, grouped: bool = False,
) -> dict[str, str]:
    """Expand PCB BOM rows without hiding duplicate refs or quantity errors."""
    footprints: dict[str, str] = {}
    for line, row in enumerate(rows, start=2):
        if grouped:
            if row["Scope"] == "Assembly":
                continue
            if row["Scope"] != "PCB":
                raise ValueError(f"Line {line}: unknown scope {row['Scope']!r}")
        refs = (
            [ref.strip() for ref in row["References"].split(",")]
            if grouped else [row["Reference"].strip()]
        )
        if any(REFERENCE.fullmatch(ref) is None for ref in refs):
            raise ValueError(f"Line {line}: missing or invalid reference {refs}")
        quantity = int(row["QuantityPerBoard" if grouped else "Quantity"])
        if quantity != len(refs):
            raise ValueError(f"Line {line}: quantity {quantity} != {len(refs)} refs")
        footprint = row["Footprint"].strip()
        if not footprint:
            raise ValueError(f"Line {line}: missing footprint")
        for ref in refs:
            if ref in footprints:
                raise ValueError(f"Line {line}: duplicate reference {ref}")
            footprints[ref] = footprint
    return footprints


class BOMInventoryTest(unittest.TestCase):
    def row(
        self, refs: str, quantity: str = "1", *, grouped: bool = False,
    ) -> dict[str, str]:
        if grouped:
            return {
                "Scope": "PCB", "References": refs,
                "QuantityPerBoard": quantity, "Footprint": RESISTOR_FOOTPRINT,
            }
        return {
            "Reference": refs, "Quantity": quantity,
            "Footprint": RESISTOR_FOOTPRINT,
        }

    def test_individual_and_grouped_rows_have_the_same_inventory(self) -> None:
        expected = {"R1": RESISTOR_FOOTPRINT, "R2": RESISTOR_FOOTPRINT}
        self.assertEqual(
            bom_footprints([self.row("R1"), self.row("R2")]), expected,
        )
        self.assertEqual(
            bom_footprints([self.row("R1, R2", "2", grouped=True)], grouped=True),
            expected,
        )

    def test_assembly_rows_are_not_pcb_components(self) -> None:
        rows = [
            self.row("R1", grouped=True),
            {"Scope": "Assembly", "References": "R1", "QuantityPerBoard": "10",
             "Footprint": ""},
        ]
        self.assertEqual(
            bom_footprints(rows, grouped=True), {"R1": RESISTOR_FOOTPRINT},
        )

    def test_missing_or_invalid_references_are_rejected(self) -> None:
        for grouped in (False, True):
            for refs in ("", " ", "R0", "R01", "#PWR01", "R1,"):
                with self.subTest(grouped=grouped, refs=refs):
                    with self.assertRaisesRegex(ValueError, "invalid reference"):
                        bom_footprints(
                            [self.row(refs, grouped=grouped)], grouped=grouped,
                        )

    def test_duplicate_references_are_rejected(self) -> None:
        for grouped in (False, True):
            with self.subTest(grouped=grouped):
                with self.assertRaisesRegex(ValueError, "duplicate reference"):
                    bom_footprints(
                        [self.row("R1", grouped=grouped)] * 2, grouped=grouped,
                    )
        with self.assertRaisesRegex(ValueError, "duplicate reference"):
            bom_footprints([self.row("R1, R1", "2", grouped=True)], grouped=True)

    def test_incorrect_or_noninteger_quantities_are_rejected(self) -> None:
        for grouped in (False, True):
            for quantity in ("0", "-1", "2", "", "1.5"):
                with self.subTest(grouped=grouped, quantity=quantity):
                    with self.assertRaises(ValueError):
                        bom_footprints(
                            [self.row("R1", quantity, grouped=grouped)],
                            grouped=grouped,
                        )

    def test_missing_footprints_are_rejected(self) -> None:
        for grouped in (False, True):
            with self.subTest(grouped=grouped):
                row = self.row("R1", grouped=grouped)
                row["Footprint"] = ""
                with self.assertRaisesRegex(ValueError, "missing footprint"):
                    bom_footprints([row], grouped=grouped)

    def test_unknown_order_scope_is_rejected(self) -> None:
        row = self.row("R1", grouped=True)
        row["Scope"] = "PCBB"
        with self.assertRaisesRegex(ValueError, "unknown scope"):
            bom_footprints([row], grouped=True)


@unittest.skipUnless(HAS_KICAD, "Requires kicad-cli and its pcbnew Python module")
class SchematicSyncTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        import pcbnew as k

        cls.temp = tempfile.TemporaryDirectory(prefix="cleany-schematic-sync-")
        cls.addClassCleanup(cls.temp.cleanup)
        netlist = Path(cls.temp.name) / "netlist.xml"
        result = subprocess.run(
            ["kicad-cli", "sch", "export", "netlist", "--format", "kicadxml",
             "--output", str(netlist), str(ROOT / "integrated_controller.kicad_sch")],
            capture_output=True, text=True, timeout=90,
        )
        if result.returncode:
            raise AssertionError(f"Netlist export failed: {result.stdout}{result.stderr}")
        cls.netlist = ET.parse(netlist).getroot()
        cls.components = cls.netlist.findall("components/comp")
        cls.board = k.LoadBoard(str(ROOT / "integrated_controller.kicad_pcb"))
        cls.mounting_holes = [
            fp for fp in cls.board.GetFootprints()
            if fp.GetAttributes() & k.FP_BOARD_ONLY
        ]
        cls.footprints = [
            fp for fp in cls.board.GetFootprints()
            if not fp.GetAttributes() & k.FP_BOARD_ONLY
        ]
        cls.schematic_inventory = {
            comp.attrib["ref"]: comp.findtext("footprint", "")
            for comp in cls.components
        }
        cls.pcb_inventory = {
            fp.GetReference(): (
                f"{fp.GetFPID().GetLibNickname()}:{fp.GetFPID().GetLibItemName()}"
            )
            for fp in cls.footprints
        }
        cls.boms = {}
        for filename in ("BOM.csv", "BOM_ORDER.csv"):
            with (ROOT / filename).open(encoding="utf-8-sig", newline="") as stream:
                cls.boms[filename] = list(csv.DictReader(stream))
        cls.schematic_nets: dict[str, set[Pin]] = {
            net.attrib["name"]: {
                (node.attrib["ref"], node.attrib["pin"])
                for node in net.findall("node")
            }
            for net in cls.netlist.findall("nets/net")
        }
        cls.pcb_nets: dict[str, set[Pin]] = defaultdict(set)
        for fp in cls.footprints:
            for pad in fp.Pads():
                if pad.GetNumber():
                    # Thermal pads can repeat a pin number within a footprint.
                    cls.pcb_nets[pad.GetNetname()].add(
                        (fp.GetReference(), pad.GetNumber())
                    )

    def test_physical_references_are_unique_and_exactly_retained(self) -> None:
        self.assertEqual(len(EXPECTED_REFERENCES), COMPONENT_COUNT)
        for source, refs in (
            ("schematic", [comp.attrib["ref"] for comp in self.components]),
            ("PCB", [fp.GetReference() for fp in self.footprints]),
        ):
            with self.subTest(source=source):
                self.assertEqual(len(refs), COMPONENT_COUNT)
                self.assertEqual(len(set(refs)), len(refs), "Duplicate references")
                for ref in refs:
                    self.assertIsNotNone(REFERENCE.fullmatch(ref), ref)
                self.assertSetEqual(set(refs), EXPECTED_REFERENCES)

    def test_schematic_and_pcb_references_and_footprints_match(self) -> None:
        self.assertDictEqual(self.schematic_inventory, self.pcb_inventory)
        self.assertTrue(all(self.schematic_inventory.values()))

    def test_mounting_holes_are_unplated_and_excluded_from_assembly(self) -> None:
        import pcbnew as k

        expected = {
            "H1": (68.35, 61.4), "H2": (178.3, 61.4),
            "H3": (178.3, 148.025), "H4": (68.35, 148.025),
        }
        self.assertEqual(len(self.mounting_holes), 4)
        self.assertSetEqual(
            {fp.GetReference() for fp in self.mounting_holes}, set(expected),
        )
        for fp in self.mounting_holes:
            with self.subTest(reference=fp.GetReference()):
                self.assertTrue(fp.GetAttributes() & k.FP_EXCLUDE_FROM_BOM)
                self.assertTrue(fp.GetAttributes() & k.FP_EXCLUDE_FROM_POS_FILES)
                pad, = list(fp.Pads())
                self.assertEqual(pad.GetAttribute(), k.PAD_ATTRIB_NPTH)
                self.assertEqual(pad.GetNetCode(), 0)
                self.assertEqual(pad.GetNumber(), "")
                self.assertEqual(
                    tuple(round(k.ToMM(v), 6) for v in pad.GetDrillSize()),
                    (3.4, 3.4),
                )
                self.assertEqual(
                    tuple(round(k.ToMM(v), 6) for v in pad.GetPosition()),
                    expected[fp.GetReference()],
                )

    def test_individual_bom_references_quantities_and_footprints(self) -> None:
        inventory = bom_footprints(self.boms["BOM.csv"])
        self.assertEqual(len(inventory), COMPONENT_COUNT)
        self.assertSetEqual(set(inventory), EXPECTED_REFERENCES)
        self.assertDictEqual(inventory, self.schematic_inventory)
        self.assertDictEqual(inventory, self.pcb_inventory)
        pull_down, = [
            row for row in self.boms["BOM.csv"] if row["Reference"] == "R4"
        ]
        self.assertEqual(pull_down["Value"], "100k")

    def test_grouped_pcb_bom_references_quantities_and_footprints(self) -> None:
        inventory = bom_footprints(self.boms["BOM_ORDER.csv"], grouped=True)
        self.assertEqual(len(inventory), COMPONENT_COUNT)
        self.assertSetEqual(set(inventory), EXPECTED_REFERENCES)
        self.assertDictEqual(inventory, self.schematic_inventory)
        self.assertDictEqual(inventory, self.pcb_inventory)
        self.assertEqual(
            sum(int(row["QuantityPerBoard"]) for row in self.boms["BOM_ORDER.csv"]
                if row["Scope"] == "PCB"),
            COMPONENT_COUNT,
        )

    def test_en_parts_have_required_footprints_and_pull_down_value(self) -> None:
        for source, inventory in (
            ("schematic", self.schematic_inventory), ("PCB", self.pcb_inventory),
        ):
            with self.subTest(source=source):
                self.assertEqual(inventory["SW3"], SWITCH_FOOTPRINT)
                self.assertEqual(inventory["R4"], RESISTOR_FOOTPRINT)
        resistor, = [c for c in self.components if c.attrib["ref"] == "R4"]
        footprint, = [fp for fp in self.footprints if fp.GetReference() == "R4"]
        self.assertEqual(resistor.findtext("value"), "100k")
        self.assertEqual(footprint.GetValue(), "100k")

    def assert_en_connections(self, nets: dict[str, set[Pin]]) -> str:
        pin_nets: dict[Pin, set[str]] = defaultdict(set)
        for name, members in nets.items():
            for pin in members:
                pin_nets[pin].add(name)
        self.assertEqual(len(pin_nets[("U3", "3")]), 1)
        en, = pin_nets[("U3", "3")]
        self.assertNotIn(en, ("", "VIN_LOGIC", "GND"))
        self.assertSetEqual(nets[en], {("U3", "3"), ("SW3", "2"), ("R4", "2")})
        for pin, expected in (
            (("SW3", "1"), "VIN_LOGIC"), (("SW3", "2"), en),
            (("R4", "1"), "GND"), (("R4", "2"), en),
        ):
            self.assertSetEqual(pin_nets[pin], {expected}, str(pin))
        return en

    def test_exported_schematic_en_connections(self) -> None:
        self.assert_en_connections(self.schematic_nets)
        node, = self.netlist.findall("nets/net/node[@ref='U3'][@pin='3']")
        self.assertRegex(node.attrib["pinfunction"], r"^EN(?:_3)?$")

    def test_pcb_en_pad_assignments_match_schematic(self) -> None:
        self.assertEqual(
            self.assert_en_connections(self.pcb_nets),
            self.assert_en_connections(self.schematic_nets),
        )

    def test_mux_reset_pull_up_value(self) -> None:
        resistor, = [c for c in self.components if c.attrib["ref"] == "R19"]
        footprint, = [fp for fp in self.footprints if fp.GetReference() == "R19"]
        self.assertEqual(resistor.findtext("value"), "10k")
        self.assertEqual(footprint.GetValue(), "10k")

    def assert_mux_reset_connections(self, nets: dict[str, set[Pin]]) -> str:
        pin_nets: dict[Pin, set[str]] = defaultdict(set)
        for name, members in nets.items():
            for pin in members:
                pin_nets[pin].add(name)
        self.assertEqual(len(pin_nets[("U9", "3")]), 1)
        reset, = pin_nets[("U9", "3")]
        self.assertNotIn(reset, ("", "+3V3", "GND"))
        self.assertSetEqual(nets[reset], {("U9", "3"), ("R19", "2")})
        for pin, expected in (
            (("R19", "2"), reset),
            (("R19", "1"), "+3V3"), (("U9", "24"), "+3V3"),
        ):
            self.assertSetEqual(pin_nets[pin], {expected}, str(pin))
        return reset

    def test_exported_schematic_mux_reset_is_pull_up_only(self) -> None:
        self.assert_mux_reset_connections(self.schematic_nets)
        node, = self.netlist.findall("nets/net/node[@ref='U9'][@pin='3']")
        self.assertRegex(node.attrib["pinfunction"], r"^~\{RESET\}(?:_3)?$")

    def test_pcb_mux_reset_is_pull_up_only_and_matches_schematic(self) -> None:
        self.assertEqual(
            self.assert_mux_reset_connections(self.pcb_nets),
            self.assert_mux_reset_connections(self.schematic_nets),
        )

    def test_imu_components_have_original_values_and_footprints(self) -> None:
        for source, components in (
            ("schematic", {
                comp.attrib["ref"]: (
                    comp.findtext("value"), comp.findtext("footprint"),
                )
                for comp in self.components if comp.attrib["ref"] in IMU_COMPONENTS
            }),
            ("PCB", {
                fp.GetReference(): (
                    fp.GetValue(), self.pcb_inventory[fp.GetReference()],
                )
                for fp in self.footprints if fp.GetReference() in IMU_COMPONENTS
            }),
        ):
            with self.subTest(source=source):
                self.assertDictEqual(components, IMU_COMPONENTS)

    def test_imu_components_use_original_sensor_sheet(self) -> None:
        for ref in IMU_COMPONENTS:
            with self.subTest(ref=ref):
                comp, = [c for c in self.components if c.attrib["ref"] == ref]
                self.assertEqual(
                    comp.find("sheetpath").attrib["names"],
                    "/Sensors / BNO055 and ToF/",
                )
                self.assertEqual(
                    comp.find("property[@name='Sheetfile']").attrib["value"],
                    "sensors.kicad_sch",
                )

    def assert_imu_reset_connections(self, nets: dict[str, set[Pin]]) -> str:
        pin_nets: dict[Pin, set[str]] = defaultdict(set)
        for name, members in nets.items():
            for pin in members:
                pin_nets[pin].add(name)
        self.assertEqual(len(pin_nets[("U8", "11")]), 1)
        reset, = pin_nets[("U8", "11")]
        self.assertEqual(reset, "Net-(U8-~{RESET})")
        self.assertSetEqual(nets[reset], {("U8", "11"), ("R13", "2")})
        self.assertSetEqual(pin_nets[("R13", "2")], {reset})
        self.assertSetEqual(pin_nets[("R13", "1")], {"+3V3"})
        return reset

    def test_exported_schematic_imu_reset_is_pull_up_only(self) -> None:
        self.assert_imu_reset_connections(self.schematic_nets)
        node, = self.netlist.findall("nets/net/node[@ref='U8'][@pin='11']")
        self.assertRegex(node.attrib["pinfunction"], r"^~\{RESET\}(?:_11)?$")

    def test_pcb_imu_reset_is_pull_up_only_and_matches_schematic(self) -> None:
        self.assertEqual(
            self.assert_imu_reset_connections(self.pcb_nets),
            self.assert_imu_reset_connections(self.schematic_nets),
        )

    def test_complete_imu_pad_contract(self) -> None:
        expected = {
            (ref, pad): {net}
            for ref, pads in IMU_PAD_NETS.items() for pad, net in pads.items()
        }
        self.assertEqual(set(IMU_PAD_NETS["U8"]), {str(n) for n in range(1, 29)})
        self.assertEqual(len(expected), 42)
        for source, nets in (
            ("schematic", self.schematic_nets), ("PCB", self.pcb_nets),
        ):
            with self.subTest(source=source):
                actual: dict[Pin, set[str]] = defaultdict(set)
                for name, members in nets.items():
                    for pin in members:
                        if pin[0] in IMU_COMPONENTS:
                            actual[pin].add(name)
                self.assertDictEqual(dict(actual), expected)
                for net, members in (
                    ("I2C1_SDA", {("U1", "34"), ("U8", "20"), ("R15", "2")}),
                    ("I2C1_SCL", {("U1", "35"), ("U8", "19"), ("R16", "2")}),
                    ("Net-(U8-CAP)", {("U8", "9"), ("C42", "1")}),
                    ("Net-(U8-~{BOOT_LOAD_PIN})", {("U8", "4"), ("R14", "2")}),
                ):
                    self.assertSetEqual(nets[net], members, net)
                for pad in IMU_UNUSED_PADS:
                    net = IMU_PAD_NETS["U8"][pad]
                    self.assertSetEqual(nets[net], {("U8", pad)}, net)

    def test_imu_unused_pins_have_no_connect_markers(self) -> None:
        for pad, function in IMU_UNUSED_PADS.items():
            with self.subTest(pad=pad, function=function):
                node, = self.netlist.findall(f"nets/net/node[@ref='U8'][@pin='{pad}']")
                self.assertRegex(
                    node.attrib["pinfunction"], rf"^{function}(?:_{pad})?$",
                )
                self.assertIn("no_connect", node.attrib["pintype"].split("+"))

    def test_obsolete_mcu_reset_nets_are_absent(self) -> None:
        for source, names in (
            ("schematic", [net.attrib["name"]
                           for net in self.netlist.findall("nets/net")]),
            ("PCB", [str(name) for name in self.board.GetNetsByName()]),
        ):
            for name in names:
                with self.subTest(source=source, net=name):
                    self.assertNotIn(name.rsplit("/", 1)[-1], REMOVED_SIGNALS)

    def test_battery_divider_senses_raw_at_the_adc_pin(self) -> None:
        for source, nets in (
            ("schematic", self.schematic_nets), ("PCB", self.pcb_nets),
        ):
            with self.subTest(source=source):
                self.assertSetEqual(nets["BAT_ADC"], {
                    ("U1", "15"), ("R22", "2"), ("R23", "1"), ("C45", "1"),
                })
                self.assertTrue({("J1", "1"), ("D1", "2"), ("R22", "1")} <= nets["RAW"])
                self.assertTrue({("R23", "2"), ("C45", "2")} <= nets["GND"])
                self.assertNotIn(("R22", "1"), nets["VIN_LOGIC"])
                self.assertNotIn(("R22", "1"), nets["VBUS"])

    def test_battery_values_packages_and_spare_capacitor(self) -> None:
        comps = {c.attrib["ref"]: c for c in self.components}
        fps = {fp.GetReference(): fp for fp in self.footprints}
        for ref, value in {"R22": "100k", "R23": "22k"}.items():
            self.assertEqual(comps[ref].findtext("value"), value)
            self.assertEqual(fps[ref].GetValue(), value)
            self.assertEqual(self.schematic_inventory[ref], RESISTOR_FOOTPRINT)
        bom = {row["Reference"]: row for row in self.boms["BOM.csv"]}
        self.assertEqual(comps["C45"].findtext("value"), "100n")
        self.assertEqual(bom["C45"]["MPN"], bom["C2"]["MPN"])
        self.assertIn("추가 발주 없음", bom["C45"]["SupplyStatus"])


if __name__ == "__main__":
    unittest.main()
