"""Check both sparse reservoirs reject configurations that used to hang/truncate."""
from pathlib import Path
import subprocess
import sys
import tempfile

compiler, root = sys.argv[1:]
with tempfile.TemporaryDirectory() as temporary:
    source = Path(temporary) / "esn.c"
    for header in ("ESN_int8.h", "ESN_int8_PRANC.h"):
        for reservoir, connectivity, valid in ((2, 2, True), (256, 1, True),
                                                (2, 3, False), (257, 1, False), (2, 0, False)):
            source.write_text(f"""
#define ESN_INT8_INPUT_DIM 1
#define ESN_INT8_OUTPUT_DIM 1
#define ESN_INT8_RESERVOIR_DIM {reservoir}
#define ESN_INT8_RES_FIXED_K {connectivity}
#include "src/pogo-utils/{header}"
""")
            result = subprocess.run([compiler, "-std=c11", "-fsyntax-only", "-I", root,
                                     str(source)], capture_output=True, text=True)
            if (result.returncode == 0) != valid:
                raise AssertionError((header, reservoir, connectivity, result.stderr))
            if not valid and "requires 1 <= K" not in result.stderr:
                raise AssertionError(result.stderr)
