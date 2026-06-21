[![Python](https://img.shields.io/badge/Python-3.9+-blue.svg)]() [![bleak](https://img.shields.io/badge/bleak-BLE-1F6FEB.svg)]() [![python-osc](https://img.shields.io/badge/python--osc-OSC-1F6FEB.svg)]() [![VQF](https://img.shields.io/badge/VQF-optional-1F6FEB.svg)]() [![macOS](https://img.shields.io/badge/macOS-arm64%2Fx86__64-000000.svg?logo=apple&logoColor=white)]() [![Device](https://img.shields.io/badge/device-MetaMotion%20RL%20%C2%B7%20BMI160%20%2B%20BMM150-8A2BE2.svg)]() [![Protocol](https://img.shields.io/badge/protocol-reverse--engineered-007808.svg)](docs/PROTOCOL.md) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)

# mmrl_osc - MetaMotion RL head tracker OSC bridge

Use an Mbientlab **MetaMotion RL** (MMRL) as a head tracker for spatial audio on
**macOS**.

The script connects to the MMRL over Bluetooth LE, runs the sensor's on-board
Bosch BSX NDOF (9-axis) fusion, and sends the orientation quaternion as OSC to
common spatial-audio plugins:

| OSC address | Arguments | Target |
|---|---|---|
| `/SceneRotator/quaternions` | `qw qx qy qz` | IEM Plugin Suite (SceneRotator) |
| `/ypr` | `yaw pitch roll` (degrees) | SPARTA, Atmoky, dearVR |
| `/Virtuoso/quat` | `qw qx qy qz` | APL Virtuoso |

All three are sent on every update (default `127.0.0.1:8000`), so several
plugins can be driven at once.

**Protocol:** the full reverse-engineered MetaWear GATT protocol and hardware
notes are in [docs/PROTOCOL.md](docs/PROTOCOL.md).

## Why this exists

The official `metawear` Python SDK does not run on macOS: it has a hard-coded
Darwin check and its `warble` BLE backend has no CoreBluetooth support. On
Linux and Windows the SDK works, but on macOS there is no maintained way to use
the device. This project talks to the MetaWear GATT interface directly with
[`bleak`](https://github.com/hbldh/bleak) (native CoreBluetooth), so it needs no
Mbientlab software at all.

The protocol was worked out empirically and cross-checked against the
[MetaWear-SDK-Cpp](https://github.com/mbientlab/MetaWear-SDK-Cpp) source; the
full write-up is in [docs/PROTOCOL.md](docs/PROTOCOL.md) so it can be
reimplemented in any language.

## What was discovered (short version)

- Streaming the on-board fusion quaternion is not a single command: the raw
  accelerometer, gyroscope and magnetometer must be configured and started
  first (the engine consumes them), then the fusion output is enabled and the
  quaternion register subscribed. Skipping the raw-sensor setup, or the
  subscribe, leaves the engine reporting enabled while emitting no data.
- The raw data registers are not uniform: accelerometer data is on register
  `0x04`, but gyroscope and magnetometer data are on `0x05` (used by `--vqf`).
- IEM SceneRotator listens on `/SceneRotator/quaternions` (4 floats), not
  `/SceneRotator/quat`; the wrong address is silently ignored.
- The unit is a Bosch **BMI160 + BMM150**. macOS addresses it by a per-Mac
  CoreBluetooth UUID, not a MAC.
- The device advertises indefinitely (no quick auto-sleep) and accepts one
  connection at a time, so an empty scan usually means another app holds it.

Full details, byte sequences and scales are in
[docs/PROTOCOL.md](docs/PROTOCOL.md).

## Addressing

On macOS, CoreBluetooth addresses peripherals by a stable **per-Mac UUID**, not
by their MAC address, so pass that UUID to `--device`. The value is printed
during the scan and differs on other machines.

## Fusion modes

By default the device's on-board Bosch BSX engine computes the orientation
(NDOF, 9-axis) and the script streams that quaternion.

With `--vqf`, the script instead streams the raw accelerometer, gyroscope and
magnetometer and runs [VQF](https://github.com/dlaidig/vqf) fusion on the host.
VQF includes magnetometer-disturbance detection and rejection, which keeps the
heading stable in electromagnetically noisy environments (near speakers, motors,
laptops, steel) where a plain 9-axis fusion would be pulled off course. It falls
back to 6-axis (gyro + accelerometer) until magnetometer samples arrive.

```bash
pip install vqf          # required for --vqf (pulls in numpy)
python mmrl_osc.py --vqf
```

The terminal shows a `[VQF]` prefix in this mode. OSC output, tare, LED and
reconnect behave the same as the default mode.

## Features

- Scan and pick a MetaWear/MetaMotion device, or connect directly by UUID.
- Battery level read on connect.
- Quaternion streaming at about 100 Hz, from on-board BSX fusion or, with
  `--vqf`, from host-side VQF fusion of the raw IMU.
- OSC output to the three addresses above.
- Yaw/pitch/roll shown in the terminal at about 5 Hz.
- Tare (zero the heading) with the **Enter** key or the device button.
- Device LED reflects battery level:
  - green: above 75%
  - orange (red + green): 15-75%
  - pulsing red: below 15%
  - blue: connected but battery could not be read
  - the LED is cleared on a clean stop, so a lit LED means the script is running.
- Auto-reconnect every 3 s if the link drops.
- Clean shutdown on Ctrl-C or kill (SIGTERM): stops the sensors and clears the
  LED before disconnecting.

## Usage (from source)

Requires Python 3.9 or newer.

```bash
python3 -m venv mmrl-venv
source mmrl-venv/bin/activate
pip install -r requirements.txt

python mmrl_osc.py                 # scan, then pick a device by number
```

Skip the scan with a known UUID (printed during the scan):

```bash
python mmrl_osc.py --device XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX
python mmrl_osc.py --port 9000     # different OSC port
python mmrl_osc.py --all           # list all BLE devices if no MetaWear is found
python mmrl_osc.py --vqf           # host-side VQF fusion (needs the vqf package)
```

Set the plugin's OSC receive to `127.0.0.1:<port>` (default 8000). Press
**Enter** or the device button while looking forward to zero the heading.

### Testing without a plugin

`osc_monitor.py` prints whatever arrives on a port:

```bash
python osc_monitor.py --port 8000   # in a second terminal
```

## Bluetooth permission (macOS)

The first BLE scan triggers a permission prompt for the app running it (Terminal,
iTerm, VS Code, or a built binary). Allow it. If scanning finds nothing, check
System Settings > Privacy & Security > Bluetooth.

## Standalone binary

Build a single signed executable for Apple Silicon with PyInstaller:

```bash
pip install pyinstaller
pyinstaller --onefile --name mmrl-osc \
  --hidden-import bleak.backends.corebluetooth \
  --hidden-import bleak.backends.corebluetooth.scanner \
  --hidden-import bleak.backends.corebluetooth.client \
  mmrl_osc.py
codesign --deep --force --sign - dist/mmrl-osc   # ad-hoc sign

./dist/mmrl-osc
```

For `--vqf` in the binary, add `--hidden-import vqf --hidden-import vqf.vqf
--collect-submodules numpy` to the build. The result runs on any Apple Silicon
Mac without Python installed. The `build/` and `dist/` artifacts are git-ignored;
build them locally or attach the binary to a release.

## Troubleshooting

- **Scan finds nothing:** the MMRL only advertises when it is not connected to
  another app (quit MetaBase or the phone app, including other script instances)
  and is charged. Use `--all` to list every BLE device.
- **Connected but no angles:** the full NDOF start sequence (raw acc/gyro/mag
  config and start, then fusion enable and subscribe) is in `FUSION_START_SEQ`
  and documented in [docs/PROTOCOL.md](docs/PROTOCOL.md).
- **Plugin does not move:** check the OSC port and that the plugin listens on the
  matching address. IEM SceneRotator uses `/SceneRotator/quaternions`.
- **Wrong rotation axis or direction:** this depends on how the device is
  mounted; the quaternion is sent as-is and can be remapped in
  `process_quaternion()`.

## Files

| File | Purpose |
|---|---|
| `mmrl_osc.py` | the head tracker bridge |
| `osc_monitor.py` | OSC listener for testing |
| `requirements.txt` | bleak, python-osc (optional vqf) |
| `docs/PROTOCOL.md` | full reverse-engineered protocol |

## License

MIT. See [LICENSE](LICENSE). Independent, clean-room reimplementation for
interoperability; not affiliated with or endorsed by mbientlab Inc.

## Contact

Bartłomiej Mróz · bartlomiej.mroz@pg.edu.pl · Department of Multimedia Systems, Gdańsk University of Technology · [bmroz.eu](https://bmroz.eu)
