"""Exercise the project's fabrication rules on isolated KiCad geometry."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any
import unittest

ROOT = Path(__file__).resolve().parents[1]
HAS_KICAD = shutil.which("kicad-cli") and importlib.util.find_spec("pcbnew")


@unittest.skipUnless(HAS_KICAD, "Requires kicad-cli and its pcbnew Python module")
class DesignRulesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        import pcbnew as k

        cls.k = k
        cls.temp = tempfile.TemporaryDirectory(prefix="cleany-design-rules-")
        cls.addClassCleanup(cls.temp.cleanup)
        out = Path(cls.temp.name)
        cls.project = json.loads((ROOT / "integrated_controller.kicad_pro").read_text())
        cls.source = k.LoadBoard(str(ROOT / "integrated_controller.kicad_pcb"))
        board = k.BOARD()
        board.SetCopperLayerCount(2)
        source_settings = cls.source.GetDesignSettings()
        settings = board.GetDesignSettings()
        settings.m_SolderMaskMinWidth = source_settings.m_SolderMaskMinWidth
        settings.m_SolderMaskExpansion = source_settings.m_SolderMaskExpansion
        settings.m_AllowSoldermaskBridgesInFPs = (
            source_settings.m_AllowSoldermaskBridgesInFPs
        )
        shared = k.NETINFO_ITEM(board, "SHARED", 1)
        board.Add(shared)

        def xy(x: float, y: float) -> Any:
            return k.VECTOR2I(k.FromMM(x), k.FromMM(y))

        def pad(
            ref: str, x: float, y: float, drill: float,
            size: float, plated: bool = True, net: Any = None,
        ) -> None:
            fp = k.FOOTPRINT(board)
            fp.SetReference(ref)
            fp.Reference().SetVisible(False)
            fp.Value().SetVisible(False)
            fp.SetPosition(xy(x, y))
            item = k.PAD(fp)
            item.SetNumber("1")
            item.SetPosition(xy(x, y))
            item.SetSize(xy(size, size))
            item.SetShape(k.PAD_SHAPE_CIRCLE)
            if drill:
                item.SetAttribute(k.PAD_ATTRIB_PTH if plated else k.PAD_ATTRIB_NPTH)
                item.SetDrillSize(xy(drill, drill))
                item.SetLayerSet(k.LSET.AllCuMask())
            else:
                item.SetAttribute(k.PAD_ATTRIB_SMD)
                layers = k.LSET()
                for layer in (k.F_Cu, k.F_Mask, k.F_Paste):
                    layers.AddLayer(layer)
                item.SetLayerSet(layers)
            if net is not None:
                item.SetNet(net)
            fp.Add(item)
            board.Add(fp)

        def via(x: float, y: float, drill: float, size: float) -> Any:
            item = k.PCB_VIA(board)
            item.SetPosition(xy(x, y))
            item.SetDrill(k.FromMM(drill))
            item.SetWidth(k.FromMM(size))
            item.SetViaType(k.VIATYPE_THROUGH)
            item.SetLayerPair(k.F_Cu, k.B_Cu)
            board.Add(item)
            return item

        def line(
            x: float, y: float, width: float, layer: int, track: bool = False,
        ) -> Any:
            item = k.PCB_TRACK(board) if track else k.PCB_SHAPE()
            if not track:
                item.SetShape(k.SHAPE_T_SEGMENT)
            item.SetStart(xy(x, y))
            item.SetEnd(xy(x + 5, y))
            item.SetWidth(k.FromMM(width))
            item.SetLayer(layer)
            board.Add(item)
            return item

        pad("PTH_BAD", 20, 20, 0.20, 0.58)
        pad("PTH_GOOD", 35, 20, 0.50, 0.90)
        pad("THERMAL", 50, 20, 0.20, 0.60)
        pad("NPTH", 65, 20, 3.20, 3.20, plated=False)
        pad("DRILL_BAD", 80, 20, 0.19, 0.60)
        pad("SAME_A", 20, 45, 0.30, 0.80, net=shared)
        pad("SAME_B", 20.6, 45, 0.30, 0.80, net=shared)
        via(20, 70, 0.30, 0.60)
        via(35, 70, 0.20, 0.50)
        via(50, 70, 0.25, 0.55)
        via(65, 70, 0.19, 0.60)
        via(80, 70, 0.30, 0.49)
        microvia = via(95, 70, 0.20, 0.60)
        microvia.SetViaType(k.VIATYPE_MICROVIA)
        for x, width in ((20, 0.124), (35, 0.125), (50, 0.20)):
            line(x, 95, width, k.F_Cu, track=True)
        via(110, 70, 0.30, 0.58)
        via(125, 70, 0.30, 0.60)
        copper_a = k.NETINFO_ITEM(board, "COPPER_A", 2)
        copper_b = k.NETINFO_ITEM(board, "COPPER_B", 3)
        board.Add(copper_a)
        board.Add(copper_b)
        for x, gap in ((20, 0.124), (35, 0.125)):
            first = line(x, 105, 0.15, k.F_Cu, track=True)
            second = line(x, 105 + 0.15 + gap, 0.15, k.F_Cu, track=True)
            first.SetNet(copper_a)
            second.SetNet(copper_b)
        pad("SMD_BAD", 20, 120, 0, 0.20)
        pad("SMD_GOOD", 35, 120, 0, 0.254)
        pad("EDGE_PAD", 219.65, 20, 0, 0.50)
        via(219.6, 70, 0.20, 0.60)
        edge_track = line(215, 45, 0.10, k.F_Cu, track=True)
        # Move this track's end to leave 0.10 mm copper-to-edge clearance.
        edge_track.SetEnd(xy(219.85, 45))
        for x, width, layer in (
            (20, 0.12, k.F_SilkS), (35, 0.12, k.B_SilkS),
            (50, 0.15, k.F_SilkS), (65, 0.10, k.F_Fab),
        ):
            line(x, 145, width, layer)
        for x, height, stroke in ((20, 0.8, 0.1), (35, 1.0, 0.15)):
            text = k.PCB_TEXT(board)
            text.SetText("TEST")
            text.SetPosition(xy(x, 165))
            text.SetTextSize(xy(height, height))
            text.SetTextThickness(k.FromMM(stroke))
            text.SetLayer(k.F_SilkS)
            board.Add(text)
        for start, end in (
            ((5, 5), (220, 5)), ((220, 5), (220, 180)),
            ((220, 180), (5, 180)), ((5, 180), (5, 5)),
        ):
            edge = k.PCB_SHAPE()
            edge.SetShape(k.SHAPE_T_SEGMENT)
            edge.SetStart(xy(*start))
            edge.SetEnd(xy(*end))
            edge.SetWidth(k.FromMM(0.05))
            edge.SetLayer(k.Edge_Cuts)
            board.Add(edge)
        k.SaveBoard(str(out / "probe.kicad_pcb"), board)
        for ext in ("kicad_pro", "kicad_dru"):
            shutil.copy2(ROOT / f"integrated_controller.{ext}", out / f"probe.{ext}")
        result = subprocess.run(
            ["kicad-cli", "pcb", "drc", "--severity-all", "--exit-code-violations",
             "--format", "json", "-o", str(out / "drc.json"),
             str(out / "probe.kicad_pcb")],
            capture_output=True, text=True, timeout=90,
        )
        if result.returncode != 5:
            raise AssertionError(f"Expected deliberate DRC failures: {result.stdout}{result.stderr}")
        cls.violations = json.loads((out / "drc.json").read_text())["violations"]

    def matching(
        self, kind: str, *, ref: str = "", x: float | None = None,
        y: float | None = None,
    ) -> list[dict[str, Any]]:
        def item_matches(item: dict[str, Any]) -> bool:
            return (
                (not ref or f"of {ref}" in item["description"])
                and (x is None or abs(item["pos"]["x"] - x) < 0.001)
                and (y is None or abs(item["pos"]["y"] - y) < 0.001)
            )
        return [
            v for v in self.violations
            if v["type"] == kind and any(item_matches(i) for i in v["items"])
        ]

    def test_project_settings(self) -> None:
        rules = self.project["board"]["design_settings"]["rules"]
        for key, value in (
            ("min_clearance", 0.125), ("min_track_width", 0.125),
            ("min_through_hole_diameter", 0.2), ("min_via_diameter", 0.5),
            ("min_via_annular_width", 0.15), ("min_copper_edge_clearance", 0.5),
            ("min_hole_clearance", 0.2), ("min_hole_to_hole", 0.2),
            ("min_silk_clearance", 0.15), ("min_text_height", 1.0),
            ("min_text_thickness", 0.15), ("solder_mask_to_copper_clearance", 0.09),
        ):
            self.assertEqual(rules[key], value, key)
        self.assertEqual(self.source.GetCopperLayerCount(), 2)
        self.assertAlmostEqual(
            self.k.ToMM(self.source.GetDesignSettings().m_SolderMaskMinWidth), 0.13,
        )
        self.assertFalse(self.source.GetDesignSettings().m_AllowSoldermaskBridgesInFPs)

    def test_small_pth_cannot_bypass_annulus(self) -> None:
        failures = self.matching("annular_width", ref="PTH_BAD")
        self.assertEqual(len(failures), 1)
        self.assertEqual(failures[0]["severity"], "error")
        self.assertFalse(self.matching("annular_width", ref="PTH_GOOD"))

    def test_thermal_drill_and_npth_peg_are_allowed(self) -> None:
        for ref in ("THERMAL", "NPTH"):
            for kind in ("drill_out_of_range", "annular_width", "assertion_failure"):
                self.assertFalse(self.matching(kind, ref=ref), (ref, kind))

    def test_drill_minimum_is_error(self) -> None:
        for args in ({"ref": "DRILL_BAD"}, {"x": 65, "y": 70}):
            failures = self.matching("drill_out_of_range", **args)
            self.assertTrue(failures, args)
            self.assertTrue(all(v["severity"] == "error" for v in failures))

    def test_same_net_drill_spacing_is_error(self) -> None:
        failures = self.matching("hole_to_hole", ref="SAME_A")
        self.assertEqual(len(failures), 1)
        self.assertEqual(failures[0]["severity"], "error")
        self.assertIn("0.4500", failures[0]["description"])

    def test_edge_rule_covers_pad_via_and_track(self) -> None:
        for args in ({"ref": "EDGE_PAD"}, {"x": 219.6, "y": 70}, {"x": 215, "y": 45}):
            self.assertTrue(self.matching("copper_edge_clearance", **args), args)

    def test_trace_width_boundary(self) -> None:
        self.assertTrue(self.matching("track_width", x=20, y=95))
        self.assertFalse(self.matching("track_width", x=35, y=95))
        self.assertFalse(self.matching("track_width", x=50, y=95))

    def test_copper_clearance_boundary(self) -> None:
        self.assertTrue(self.matching("clearance", x=20, y=105))
        self.assertFalse(self.matching("clearance", x=35, y=105))

    def test_via_annulus_boundary(self) -> None:
        self.assertTrue(self.matching("annular_width", x=110, y=70))
        self.assertFalse(self.matching("annular_width", x=125, y=70))
        self.assertFalse(self.matching("via_diameter", x=110, y=70))

    def test_via_cost_is_warning_not_ring_override(self) -> None:
        for x in (35, 50):
            failures = self.matching("assertion_failure", x=x, y=70)
            self.assertEqual(len(failures), 1)
            self.assertEqual(failures[0]["severity"], "warning")
            self.assertIn("surcharge", failures[0]["description"])
            self.assertFalse(self.matching("annular_width", x=x, y=70))
        self.assertFalse(self.matching("assertion_failure", x=20, y=70))

    def test_undersize_via_copper_is_error(self) -> None:
        self.assertTrue(self.matching("via_diameter", x=80, y=70))
        self.assertTrue(self.matching("annular_width", x=80, y=70))

    def test_ordinary_profile_rejects_microvias(self) -> None:
        failures = self.matching("assertion_failure", x=95, y=70)
        self.assertTrue(any(
            v["severity"] == "error" and "Through vias only" in v["description"]
            for v in failures
        ))

    def test_smd_pad_size(self) -> None:
        self.assertTrue(self.matching("assertion_failure", ref="SMD_BAD"))
        self.assertFalse(self.matching("assertion_failure", ref="SMD_GOOD"))

    def test_silkscreen_graphics_are_scoped_to_silk(self) -> None:
        for x in (20, 35):
            failures = self.matching("assertion_failure", x=x, y=145)
            self.assertEqual(len(failures), 1)
            self.assertEqual(failures[0]["severity"], "warning")
        self.assertFalse(self.matching("assertion_failure", x=50, y=145))
        self.assertFalse(self.matching("assertion_failure", x=65, y=145))
        self.assertFalse(any(
            "Edge.Cuts" in i["description"]
            for v in self.violations if v["type"] == "assertion_failure"
            for i in v["items"]
        ))

    def test_silkscreen_text(self) -> None:
        self.assertTrue(self.matching("text_height", x=20, y=165))
        self.assertTrue(self.matching("text_thickness", x=20, y=165))
        self.assertFalse(self.matching("text_height", x=35, y=165))
        self.assertFalse(self.matching("text_thickness", x=35, y=165))


if __name__ == "__main__":
    unittest.main()
