Import("env")
import re
import os
import shutil

# Post build step: copies the built .bin to firmware/ARB-V<version>.bin, taking the
# version number from the Version string in src/ARB.cpp.


def rename_firmware(source, target, env):
    src_dir     = env.subst("$PROJECT_SRC_DIR")
    project_dir = env.subst("$PROJECT_DIR")
    arb2_cpp    = os.path.join(src_dir, "ARB.cpp")

    # Pull the version number from ARB.cpp  e.g. "ARB Version 2.27, Oct 8, 2026"
    version = "unknown"
    try:
        with open(arb2_cpp) as f:
            for line in f:
                m = re.search(r'char\s+Version\[\].*?Version\s+([\d.]+)', line)
                if m:
                    version = m.group(1)
                    break
    except Exception as e:
        print(f"rename_firmware: could not read ARB.cpp: {e}")

    built_bin = str(target[0])
    out_dir   = os.path.join(project_dir, "firmware")
    out_name  = f"ARB-V{version}.bin"
    out_path  = os.path.join(out_dir, out_name)

    try:
        os.makedirs(out_dir, exist_ok=True)
        shutil.copy(built_bin, out_path)
        print("")
        print("=" * 50)
        print(f"  Firmware copied to: firmware/{out_name}")
        print("=" * 50)
        print("")
    except Exception as e:
        print(f"rename_firmware: could not copy firmware: {e}")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", rename_firmware)
