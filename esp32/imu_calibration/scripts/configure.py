"""Regenerate derived sdkconfig when reviewed defaults or Kconfig change."""
Import("env")

import hashlib
from pathlib import Path

project = Path(env.subst("$PROJECT_DIR"))
environment = env.subst("$PIOENV")
digest = hashlib.sha256()
for source in (project / "sdkconfig.defaults", project / "main/Kconfig.projbuild"):
    digest.update(source.read_bytes())
fingerprint = digest.hexdigest()
stamp = project / ".pio/cleany-sdkconfig-defaults"
if not stamp.exists() or stamp.read_text() != fingerprint:
    for generated in (
        project / f"sdkconfig.{environment}",
        project / ".pio/build" / environment / "CMakeCache.txt",
    ):
        if generated.exists():
            generated.unlink()
    stamp.parent.mkdir(parents=True, exist_ok=True)
    stamp.write_text(fingerprint)
