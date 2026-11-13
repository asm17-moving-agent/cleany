"""Check downloaded models and their project-relative PCB assignments."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
MODELS = {
    "L_Wurth_WE-XHMI-6060.step": (
        "bad3326ba03207b064382fc704417cf9848a609c7a8d4ec704ac398241f552df",
        {f"L{i}" for i in range(1, 6)},
        90.0,
    ),
    "SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.29x3mm.step": (
        "2d6f7e774e8143e3320cbd59eb5be9f3b7864614644e3b072006e1eb05fb7fa6",
        {f"U{i}" for i in range(3, 8)},
        0.0,
    ),
}
EASYEDA_MODEL = "easyeda/TCA9548ADGSR.step"
EASYEDA_DIGEST = "271ba4897aad65006349c54c4a2f47a401f9dfb15fb8c0b1943b6141cfa99ce3"
MOLEX_DIRECTORY = ROOT / "libraries/3dmodels/molex"


class ModelAssetsTest(unittest.TestCase):
    def test_molex_source_metadata(self) -> None:
        sources = json.loads((MOLEX_DIRECTORY / "sources.json").read_text())
        self.assertEqual(
            {model["mpn"] for model in sources["models"]},
            {"0022057035", "0022057045"},
        )
        references = []
        for model in sources["models"]:
            with self.subTest(mpn=model["mpn"]):
                self.assertRegex(model["sha256"], r"^[0-9a-f]{64}$")
                self.assertTrue(
                    model["archive_url"].startswith("https://www.farnell.com/cad/")
                )
                references.extend(model["references"])
        self.assertCountEqual(references, [f"J{i}" for i in range(7, 15)])

    @unittest.skipUnless(
        all((MOLEX_DIRECTORY / name).is_file() for name in (
            "022057035.step", "022057045.step",
        )),
        "Optional Molex CAD is local-only; see libraries/3dmodels/README.md",
    )
    def test_molex_original_assets(self) -> None:
        sources = json.loads((MOLEX_DIRECTORY / "sources.json").read_text())
        for model in sources["models"]:
            with self.subTest(mpn=model["mpn"]):
                data = (MOLEX_DIRECTORY / model["file"]).read_bytes()
                self.assertEqual(hashlib.sha256(data).hexdigest(), model["sha256"])
                self.assertTrue(data.startswith(b"ISO-10303-21;"))
                self.assertIn(b"END-ISO-10303-21;", data)

    @unittest.skipUnless(importlib.util.find_spec("pcbnew"), "Requires pcbnew")
    def test_molex_pcb_alignment(self) -> None:
        import pcbnew as k

        board = k.LoadBoard(str(ROOT / "integrated_controller.kicad_pcb"))
        footprints = {fp.GetReference(): fp for fp in board.GetFootprints()}
        sources = json.loads(
            (ROOT / "libraries/3dmodels/molex/sources.json").read_text()
        )
        for entry in sources["models"]:
            for reference in entry["references"]:
                with self.subTest(reference=reference):
                    model, = footprints[reference].Models()
                    self.assertEqual(
                        model.m_Filename,
                        "${KIPRJMOD}/libraries/3dmodels/molex/" + entry["file"],
                    )
                    self.assertEqual(
                        (model.m_Scale.x, model.m_Scale.y, model.m_Scale.z), (1, 1, 1)
                    )
                    self.assertEqual(
                        [model.m_Offset.x, model.m_Offset.y, model.m_Offset.z],
                        entry["offset_mm"],
                    )
                    self.assertEqual(
                        [model.m_Rotation.x, model.m_Rotation.y, model.m_Rotation.z],
                        entry["rotation_deg"],
                    )
                    self.assertTrue(model.m_Show)

    def test_easyeda_download(self) -> None:
        path = ROOT / "libraries/3dmodels" / EASYEDA_MODEL
        self.assertTrue(path.with_suffix(".source.json").is_file())
        data = path.read_bytes()
        self.assertEqual(hashlib.sha256(data).hexdigest(), EASYEDA_DIGEST)
        self.assertTrue(data.startswith(b"ISO-10303-21;"))
        self.assertIn(b"END-ISO-10303-21;", data)

    @unittest.skipUnless(importlib.util.find_spec("pcbnew"), "Requires pcbnew")
    def test_u9_model_alignment(self) -> None:
        import pcbnew as k

        board = k.LoadBoard(str(ROOT / "integrated_controller.kicad_pcb"))
        fp = next(fp for fp in board.GetFootprints() if fp.GetReference() == "U9")
        library = k.FootprintLoad(
            str(ROOT / "libraries/footprints/Integrated_Controller.pretty"),
            "Texas_DGS0024A_VSSOP-24_3x6.1mm_P0.5mm",
        )
        self.assertIsNotNone(library)
        for footprint in (fp, library):
            model, = footprint.Models()
            self.assertEqual(
                model.m_Filename, f"${{KIPRJMOD}}/libraries/3dmodels/{EASYEDA_MODEL}"
            )
            self.assertEqual(
                (model.m_Scale.x, model.m_Scale.y, model.m_Scale.z), (1, 1, 1)
            )
            # The unmodified STEP is centered below its seating plane.
            self.assertEqual(
                (model.m_Offset.x, model.m_Offset.y, model.m_Offset.z), (0, 0, 0.6)
            )
            self.assertEqual(
                (model.m_Rotation.x, model.m_Rotation.y, model.m_Rotation.z),
                (0, 0, 90),
            )

    def test_unmodified_step_downloads(self) -> None:
        for filename, (digest, _, _) in MODELS.items():
            with self.subTest(filename=filename):
                data = (ROOT / "libraries/3dmodels" / filename).read_bytes()
                self.assertEqual(hashlib.sha256(data).hexdigest(), digest)
                self.assertTrue(data.startswith(b"ISO-10303-21;"))
                self.assertIn(b"END-ISO-10303-21;", data)

    @unittest.skipUnless(importlib.util.find_spec("pcbnew"), "Requires pcbnew")
    def test_pcb_and_custom_library_assignments(self) -> None:
        import pcbnew as k

        board = k.LoadBoard(str(ROOT / "integrated_controller.kicad_pcb"))
        footprints = {fp.GetReference(): fp for fp in board.GetFootprints()}
        for filename, (_, references, rotation) in MODELS.items():
            for reference in references:
                with self.subTest(reference=reference):
                    model, = footprints[reference].Models()
                    self.assertEqual(
                        model.m_Filename, f"${{KIPRJMOD}}/libraries/3dmodels/{filename}"
                    )
                    self.assertEqual(
                        (model.m_Scale.x, model.m_Scale.y, model.m_Scale.z), (1, 1, 1)
                    )
                    self.assertEqual(
                        (model.m_Offset.x, model.m_Offset.y, model.m_Offset.z), (0, 0, 0)
                    )
                    self.assertEqual(
                        (model.m_Rotation.x, model.m_Rotation.y, model.m_Rotation.z),
                        (0, 0, rotation),
                    )
        fp = k.FootprintLoad(
            str(ROOT / "libraries/footprints/Integrated_Controller.pretty"),
            "Wuerth_74439346033_6.65x6.45mm",
        )
        self.assertIsNotNone(fp)
        model, = fp.Models()
        board_model, = footprints["L1"].Models()
        self.assertEqual(model.m_Filename, board_model.m_Filename)
        self.assertEqual(model.m_Rotation.z, 90)


if __name__ == "__main__":
    unittest.main()
