# ArduPilot MMC5603 Compass Driver Patch

Native dual-chip support for the **Memsic MMC5603NJ** 3-axis magnetometer in ArduPilot's `AP_Compass_MMC5xx3` driver. Seamlessly integrate the popular Adafruit MMC5603 breakout with any ArduPilot-supported flight controller over I2C while preserving 100% backward compatibility with the MMC5983.

---

## Downloads & Pre-compiled Releases

Ready-to-flash binaries compiled directly against ArduCopter 4.7.1 for the **SpeedyBee F405 AIO** with the MMC5603 driver patch:

📦 **[Download v1.0.0 Firmware Release](https://github.com/DynaRTX/ardupilot-mmc5603-driver/releases/tag/v1.0.0)** [![release](https://img.shields.io/badge/release-alpha-yellow.svg)](https://github.com/DynaRTX/ardupilot-mmc5603-driver/releases/tag/v1.0.0)

| File Asset | Format | Flashing Tool |
| :--- | :--- | :--- |
| **`arducopter.apj`** | ArduPilot JSON Package | **Mission Planner** / **QGroundControl** (*Load custom firmware*) |
| **`arducopter.bin`** | Raw Binary | DFU / STM32 flashing utilities |
| **`arducopter.hex`** | Intel HEX | STM32CubeProgrammer / Betaflight DFU recovery |
| **`arducopter_with_bl.hex`** | Combined Firmware + Bootloader | Full chip erase / unbrick recovery |

### Built-in & Enabled Target Parameters
The pre-compiled release includes default SpeedyBee F405 AIO hardware mappings with only the MMC5603 driver enabled:
* **`AP_COMPASS_MMC5XX3_ENABLED = 1`**: Enables dual MMC5983 & MMC5603 autodetection
* **`AP_COMPASS_PROBING_ENABLED = 1`**: Automatic I2C compass scanning
* **`HAL_COMPASS_AUTO_ROT_DEFAULT = 2`**: Automatic compass orientation learning
* **`HAL_FRAME_TYPE_DEFAULT = 12`**: Standard Betaflight / X quad motor mapping
* **`MOT_PWM_TYPE = 6`**: DShot600 motor protocol
* **`SERVO_BLH_BDMASK = 15`**: Bidirectional DShot enabled on channels 1–4 (RPM telemetry)
* **`GPS_DRV_OPTIONS = 4`**: 115200 baud UBlox GPS communication
* **`NTF_LED_TYPES = 257`**: WS2812 addressable LED support enabled on pad M5
* **`OSD_ENABLED = 1`**: Integrated MAX7456 analog OSD

---

## Key Capabilities

- **Dual-Sensor Auto-Detection**: Dynamically probes both MMC5983 (Product ID `0x30` on reg `0x2F`) and MMC5603 (Product ID `0x10` on reg `0x39`) at startup.
- **Full 20-Bit Resolution**: Reads and unpacks the 9-byte data stream for MMC5603, delivering high precision at `0.0625 mG/LSB` sensitivity.
- **Zero-Regression Design**: Retains all existing MMC5983 registers, timings, and 16-bit algorithms untouched.
- **Clean Telemetry Reporting**: Registers `DEVTYPE_MMC5603` (`0x1A`) so Ground Control Stations (Mission Planner, QGroundControl) accurately identify the connected hardware.
- **Upstream Compliant**: Written following ArduPilot `AP_Compass` architecture, ready for direct patching or pull requests.

---

## Quick Start

### 1. Apply the Patch
Clone your ArduPilot tree and apply the unified patch directly from this repository:

```sh
cd /path/to/ardupilot
git apply /path/to/ardupilot-mmc5603-driver/patch.diff
```

Or copy the modified library files directly into your source tree:

```sh
cp -r /path/to/ardupilot-mmc5603-driver/libraries/AP_Compass/* ./libraries/AP_Compass/
```

### 2. Build and Flash
Build your target vehicle firmware (e.g. ArduCopter for SpeedyBeeF405AIO):

```sh
./waf configure --board SpeedyBeeF405AIO
./waf copter --upload
```

### 3. Connect & Calibrate
1. Connect your MMC5603 breakout to the flight controller's I2C bus (`SDA`, `SCL`, `3.3V`, `GND`).
2. Connect to **Mission Planner** or **QGroundControl**.
3. Perform standard **Onboard Mag Calibration**.

---

## Repository Structure

```text
├── README.md                                  # Project overview and instructions
├── CHANGES.md                                 # Technical diffs & implementation details
├── patch.diff                                 # Standalone Git patch file
└── libraries/
    └── AP_Compass/
        ├── AP_Compass_Backend.h               # Registered DEVTYPE_MMC5603 (0x1A)
        ├── AP_Compass_MMC5xx3.h               # Dynamic buffer sizing & variant enum
        └── AP_Compass_MMC5xx3.cpp             # Dual-probing, 20-bit extraction, & scaling
```

---

## Tested Hardware

- **Flight Controller**: SpeedyBee F405 AIO running ArduCopter 4.7
- **Magnetometer**: Adafruit MMC5603 Breakout (Memsic MMC5603NJ)
- **Bus**: Primary I2C (`PB8`/`PB9`)
- **Validation**:
  - Boot detection as `DEVTYPE_MMC5603`
  - Steady 100 Hz sampling with zero bus errors
  - Successful onboard sphere-fit calibration
  - Verified heading lock and 180° rotation checks

---

## Resources

- 📦 [GitHub Releases](https://github.com/DynaRTX/ardupilot-mmc5603-driver/releases)
- 📚 [CHANGES.md — Line-by-Line Diffs](./CHANGES.md)
- 📄 [Memsic MMC5603NJ Datasheet (Rev. B)](https://www.memsic.com/)
- 🛒 [Adafruit MMC5603 Breakout Board](https://www.adafruit.com/product/5579)
- 🚁 [ArduPilot Documentation](https://ardupilot.org/)
- 💬 [ArduPilot Discourse Community](https://discuss.ardupilot.org/)

---

## Contributing

Pull requests, bug reports, and testing feedback across different flight controllers and boards are welcome! Please open an issue or submit a PR on GitHub.

---

## License

This project is open source and distributed under the same terms as ArduPilot: **GNU General Public License v3 (GPLv3)**.
