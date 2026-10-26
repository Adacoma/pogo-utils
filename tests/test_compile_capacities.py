"""Reject unsupported sizes at compile time; accept smaller SSR class arrays."""
from pathlib import Path
import subprocess
import sys
import tempfile

compiler, root = sys.argv[1:]
with tempfile.TemporaryDirectory() as directory:
    source = Path(directory) / "capacity.c"
    cases = [("ssr.h", "SSR_MAX_CLASSES", n, 1 <= n <= 255)
             for n in (0, 1, 2, 4, 5, 10, 255, 256)]
    cases += [("photostart.h", "PHOTOSTART_NSENS", n, n == 3) for n in (0, 1, 2, 3, 4)]
    for header, macro, size, valid in cases:
        source.write_text(f'#define {macro} {size}\n#include "src/pogo-utils/{header}"\n')
        result = subprocess.run([compiler, "-std=c11", "-fsyntax-only", "-I", root,
                                 "-I", str(Path(root) / "tests/control_stubs"), str(source)],
                                capture_output=True, text=True)
        if (result.returncode == 0) != valid:
            raise AssertionError((macro, size, result.stderr))
        if not valid and f"{macro} must be" not in result.stderr:
            raise AssertionError(result.stderr)
