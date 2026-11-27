import json
import os

# keyboard.json "hardware.driver" -> firmware crate
DRIVER_CRATES = {
    "stm32f446xx": "libhmk-stm32f446",
    "at32f405xx": "libhmk-at32f405",
    "nrf52840xx": "libhmk-nrf52840",
}

if __name__ == "__main__":
    matrix = []
    for keyboard in sorted(os.listdir("keyboards")):
        path = os.path.join("keyboards", keyboard, "keyboard.json")
        if not os.path.isfile(path):
            continue
        with open(path) as f:
            driver = json.load(f)["hardware"]["driver"]
        matrix.append({"keyboard": keyboard, "crate": DRIVER_CRATES[driver]})
    print(json.dumps(matrix))
