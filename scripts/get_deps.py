# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.

import json
import os
import subprocess

Import("env")

PIP_PACKAGES = ["pip", "pydantic"]

for package in PIP_PACKAGES:
    env.Execute(f"$PYTHONEXE -m pip install {package} --upgrade")


def _sparse_checkout(repo: str, ref: str, dest: str, paths: list[str]):
    if os.path.exists(dest):
        return
    os.makedirs(dest, exist_ok=True)
    subprocess.check_call(["git", "init", dest])
    subprocess.check_call(["git", "-C", dest, "remote", "add", "origin", repo])
    subprocess.check_call(["git", "-C", dest, "config", "core.sparseCheckout", "true"])
    sparse_file = os.path.join(dest, ".git", "info", "sparse-checkout")
    with open(sparse_file, "w") as f:
        for path in paths:
            f.write(f"{path}\n")
    subprocess.check_call(
        ["git", "-C", dest, "fetch", "--depth", "1", "origin", ref]
    )
    subprocess.check_call(["git", "-C", dest, "checkout", "FETCH_HEAD"])


def _fetch_nrf52_sdk(project_dir: str):
    sdk_root = os.path.join(project_dir, ".pio", "nrf52_sdk")

    _sparse_checkout(
        repo="https://github.com/ARM-software/CMSIS_5.git",
        ref="5.9.0",
        dest=os.path.join(sdk_root, "cmsis"),
        paths=["CMSIS/Core/Include"],
    )

    _sparse_checkout(
        repo="https://github.com/NordicSemiconductor/nrfx.git",
        ref="v3.5.0",
        dest=os.path.join(sdk_root, "nrfx"),
        paths=[
            "nrfx.h",
            "drivers/nrfx_common.h",
            "drivers/nrfx_errors.h",
            "drivers/include/nrfx_clock.h",
            "drivers/include/nrfx_power_clock.h",
            "hal",
            "mdk",
            "soc",
            "templates",
        ],
    )

    _sparse_checkout(
        repo="https://github.com/adafruit/Adafruit_nRF52_Arduino.git",
        ref="1.7.0",
        dest=os.path.join(sdk_root, "s140"),
        paths=["cores/nRF5/nordic/softdevice/s140_nrf52_7.3.0_API/include"],
    )


keyboard = env["PIOENV"]
kb_json_path = os.path.join(env["PROJECT_DIR"], "keyboards", keyboard, "keyboard.json")
if os.path.exists(kb_json_path):
    with open(kb_json_path, "r") as f:
        kb_data = json.load(f)
    if kb_data.get("hardware", {}).get("driver") == "nrf52840xx":
        _fetch_nrf52_sdk(env["PROJECT_DIR"])
