import sys

sys.path.append("scripts")

import argparse
import configparser
import os
import scripts.utils as utils


if __name__ == "__main__":
    keyboards = [
        keyboard
        for keyboard in os.listdir("keyboards")
        if os.path.isdir(os.path.join("keyboards", keyboard))
    ]

    parser = argparse.ArgumentParser(description="PlatformIO Project Setup")
    parser.add_argument(
        "--keyboard", "-k", choices=keyboards, required=True, help="Select a keyboard"
    )
    args = parser.parse_args()

    keyboard: str = args.keyboard
    driver = utils.get_driver(keyboard)

    build_flags = ["${env.build_flags}"]
    build_src_flags = [
        "${env.build_src_flags}",
        "-Werror",
        "-Wall",
        "-Wextra",
        "-Wsign-conversion",
        "-Wswitch-default",
        "-Wswitch",
        "-Wdouble-promotion",
        "-Wstrict-prototypes",
        "-Wno-unused-parameter",
    ]
    extra_scripts = [
        "pre:scripts/get_deps.py",
        "pre:scripts/validate.py",
        "pre:scripts/make.py",
        "pre:scripts/metadata.py",
    ]
    lib_deps = ["https://github.com/hathach/tinyusb.git#0.20.0"]

    pio_config = configparser.ConfigParser()
    env_config = {
        "board": driver.platformio.board,
        "board_build.ldscript": f"linker/{driver.platformio.ldscript}",
        "build_flags": "\n".join(build_flags),
        "build_src_filter": "${env.build_src_filter}",
        "build_src_flags": "\n".join(build_src_flags),
        "extra_scripts": "\n".join(extra_scripts),
        "lib_deps": "\n".join(lib_deps),
        "platform": driver.platformio.platform,
        "upload_protocol": driver.platformio.upload_protocol,
    }
    if driver.platformio.framework:
        env_config["framework"] = driver.platformio.framework
    pio_config[f"env:{keyboard}"] = env_config

    with open("platformio.ini", "w") as f:
        pio_config.write(f)
