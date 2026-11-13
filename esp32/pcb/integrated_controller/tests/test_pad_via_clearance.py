"""Check assembly clearance that electrical same-net DRC does not enforce."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(importlib.util.find_spec("pcbnew"), "Requires KiCad's pcbnew module")
class PadViaClearanceTest(unittest.TestCase):
    def test_standalone_vias_do_not_overlap_component_lands(self) -> None:
        import pcbnew as k

        board = k.LoadBoard(str(ROOT / "integrated_controller.kicad_pcb"))
        pads = [
            (footprint.GetReference(), pad.GetNumber(), layer,
             pad.GetEffectivePolygon(layer))
            for footprint in board.GetFootprints()
            for pad in footprint.Pads()
            for layer in (k.F_Cu, k.B_Cu)
            if pad.IsOnLayer(layer)
        ]
        collisions = []
        for via in board.GetTracks():
            if not isinstance(via, k.PCB_VIA):
                continue
            for reference, number, layer, copper in pads:
                # One coordinate unit also catches tangent edges.
                if copper.Collide(via.GetPosition(), via.GetWidth(layer) // 2 + 1):
                    pos = via.GetPosition()
                    collisions.append(
                        f"{via.GetNetname()} via at "
                        f"({k.ToMM(pos.x):.3f}, {k.ToMM(pos.y):.3f}) overlaps "
                        f"{reference}.{number} on {k.LayerName(layer)}"
                    )

        # Plated holes belonging to an exposed-pad footprint are PAD objects,
        # not standalone vias, and retain the component's intentional geometry.
        self.assertEqual(collisions, [], "\n".join(collisions))


if __name__ == "__main__":
    unittest.main()
