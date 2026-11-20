from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from cleany_teleop.web import ui_asset, ui_index


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


def test_entrypoint_follows_rebuilt_asset_hash(tmp_path):
    (tmp_path / "assets").mkdir()
    index = tmp_path / "index.html"
    index.write_bytes(b'<script src="/assets/old.js"></script>')
    (tmp_path / "assets/old.js").write_bytes(b"old")
    assert b"old.js" in ui_index(tmp_path)
    (tmp_path / "assets/new.js").write_bytes(b"new")
    index.write_bytes(b'<script src="/assets/new.js"></script>')
    (tmp_path / "assets/old.js").unlink()
    assert b"new.js" in ui_index(tmp_path)
    assert ui_asset(tmp_path, "/assets/new.js")[0] == b"new"
