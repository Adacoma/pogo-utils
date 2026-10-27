"""Link motion-example Makefiles against a tiny SDK-only PFFS fixture.

No SDK checkout, installed library, firmware or Pogosim is used. The fixture
has no old local PFFS sources, so stale object lists cannot silently pass.
Generated host executables are checked for existence but never launched.
"""
from pathlib import Path
import os
import subprocess
import sys
import tempfile

cc, cxx, make, project = sys.argv[1:]
root = Path(project)
examples = ("go_straight", "go_straight_simple", "vicsek", "acu", "vicsek_u_turns")
core = ("flash_file_read", "flash_file_secure", "flash_file_name",
        "flash_file_write", "flash_file_status", "flash_log")

# Do not inherit a parent make's jobserver, goals or SDK path overrides.
env = os.environ.copy()
for key in ("MAKEFLAGS", "MFLAGS", "MAKEOVERRIDES", "GNUMAKEFLAGS", "MAKELEVEL"):
    env.pop(key, None)

with tempfile.TemporaryDirectory(prefix="pogo-example-makefiles-") as temporary:
    fixture = Path(temporary)
    local = fixture / "src" / "pogo-utils"
    local.mkdir(parents=True)
    (local / "flash_file.h").write_text((root / "src/pogo-utils/flash_file.h").read_text())
    sdk = fixture / "pogobot-sdk"
    (sdk / "includes" / "pffs").mkdir(parents=True)
    (sdk / "includes" / "pffs" / "flash_file.h").write_text(
        "#define TEST_PFFS_HEADER_VALUE 7\n")
    (sdk / "src" / "pffs").mkdir(parents=True)
    for name in core:
        (sdk / "src" / "pffs" / (name + ".c")).write_text(
            '#include <pffs/flash_file.h>\n'
            '#ifndef POGOBOT_PFFS_SIMULATOR\n#error Missing simulator backend\n#endif\n'
            f'int probe_{name}(void) {{ return TEST_PFFS_HEADER_VALUE; }}\n')
    for name in ("magnetometer_calibration_flash", "kinematics", "heading_PID",
                 "calibrated_motors", "wall_avoidance_magnetometer", "magnetometer_heading_detection"):
        (local / (name + ".c")).write_text(f"int probe_{name}(void) {{ return 0; }}\n")

    for name in examples:
        directory = fixture / "examples" / name
        directory.mkdir(parents=True)
        (directory / "Makefile").write_text((root / "examples" / name / "Makefile").read_text())
        (directory / "main.c").write_text(
            '#include "pogo-utils/flash_file.h"\n'
            'int probe_flash_file_read(void);\n'
            'int probe_magnetometer_calibration_flash(void);\n'
            'int main(void) { return TEST_PFFS_HEADER_VALUE + probe_flash_file_read() + '
            'probe_magnetometer_calibration_flash(); }\n')

    # Included Makefiles must preserve the child target name and BUILD_DIR.
    cases = [(name, "build") for name in examples]
    cases += [(name, "custom-build") for name in ("go_straight_simple", "acu")]
    for name, build in cases:
        directory = fixture / "examples" / name
        result = subprocess.run(
            [make, "-C", str(directory), "-j4", "sim", f"SIM_CC={cc}", f"SIM_CXX={cxx}",
             "SIM_CXXFLAGS=", "SIM_LDFLAGS=", "SIM_INCLUDES_EXT=", f"BUILD_DIR={build}",
             "POGO_SDK=../../pogobot-sdk", "POGOSIM_INCLUDE_DIR=../../probe-platform",
             "POGOSIM_SRC_DIR=../../probe-platform"],
            env=env, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(f"{name} ({build}):\n{result.stdout}\n{result.stderr}")
        assert (directory / name).is_file(), (name, build, "missing executable")
        assert (directory / build / "sim" / name).is_file(), (name, build, "missing copy")
        for source in core:
            assert (directory / build / "sim" / (source + "_sdk.o")).is_file(), (name, source)
        assert not (directory / build / "sim" / "flash_file_read_local.o").exists()
        print(f"{name} ({build}): linked with SDK PFFS")
