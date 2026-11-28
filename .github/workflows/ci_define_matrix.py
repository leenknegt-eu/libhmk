import json
import os

# keyboard.json "hardware.driver" -> firmware crate
DRIVER_CRATES = {
    "stm32f446xx": "libhmk-stm32f446",
    "at32f405xx": "libhmk-at32f405",
    "nrf52840xx": "libhmk-nrf52840",
}
DONGLE_CRATE = "libhmk-nrf52840-dongle"

if __name__ == "__main__":
    matrix = []
    for keyboard in sorted(os.listdir("keyboards")):
        path = os.path.join("keyboards", keyboard, "keyboard.json")
        if not os.path.isfile(path):
            continue
        with open(path) as f:
            kb = json.load(f)
        driver = kb["hardware"]["driver"]
        matrix.append(
            {"keyboard": keyboard, "crate": DRIVER_CRATES[driver], "name": keyboard, "probe": True}
        )
        esb = kb.get("wireless", {}).get("esb", {})
        if "wireless" in kb and esb.get("enabled", True):
            # 2.4 GHz receiver (nRF52840 Dongle) built for this keyboard
            matrix.append(
                {"keyboard": keyboard, "crate": DONGLE_CRATE, "name": f"{keyboard}-dongle", "probe": False}
            )
    print(json.dumps(matrix))
