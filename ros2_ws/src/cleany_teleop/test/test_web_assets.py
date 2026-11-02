from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from cleany_teleop.web import ui_asset


class AssetTests(unittest.TestCase):
    def test_compiled_assets_and_cache_query(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "assets").mkdir()
            (root / "assets/app.js").write_bytes(b"export {};")
            (root / "assets/app.css").write_bytes(b"body {}")
            self.assertEqual(
                ui_asset(root, "/assets/app.js?v=1"), (b"export {};", "text/javascript")
            )
            self.assertEqual(ui_asset(root, "/assets/app.css")[1], "text/css")
            self.assertIsNone(ui_asset(root, "/assets/missing.js"))

    def test_traversal_and_symlink_cannot_expose_other_files(self):
        with TemporaryDirectory() as directory:
            root = Path(directory) / "ui"
            (root / "assets").mkdir(parents=True)
            (root / "index.html").write_text("index")
            secret = Path(directory) / "secret"
            secret.write_text("private")
            (root / "assets/link.js").symlink_to(secret)
            for path in (
                "/index.html",
                "/assets/../index.html",
                "/assets/%2e%2e/index.html",
                "/assets/../../secret",
                "/assets/link.js",
            ):
                self.assertIsNone(ui_asset(root, path), path)


def test_colcon_symlink_assets_use_the_trusted_entrypoint_tree(tmp_path):
    installed = tmp_path / "install"
    compiled = tmp_path / "build"
    (installed / "assets").mkdir(parents=True)
    (compiled / "assets").mkdir(parents=True)
    (compiled / "index.html").write_text("entry")
    (compiled / "assets/app.js").write_bytes(b"export {};")
    (installed / "index.html").symlink_to(compiled / "index.html")
    (installed / "assets/app.js").symlink_to(compiled / "assets/app.js")
    (tmp_path / "secret").write_text("private")
    (installed / "assets/secret.js").symlink_to(tmp_path / "secret")
    assert ui_asset(installed, "/assets/app.js")[0] == b"export {};"
    assert ui_asset(installed, "/assets/secret.js") is None
