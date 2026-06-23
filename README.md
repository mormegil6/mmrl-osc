[![Python](https://img.shields.io/badge/Python-3.9+-blue.svg)]() [![bleak](https://img.shields.io/badge/bleak-BLE-1F6FEB.svg)]() [![python-osc](https://img.shields.io/badge/python--osc-OSC-1F6FEB.svg)]() [![VQF](https://img.shields.io/badge/VQF-optional-1F6FEB.svg)]() [![macOS](https://img.shields.io/badge/macOS-arm64%2Fx86__64-000000.svg?logo=apple&logoColor=white)]() [![Device](https://img.shields.io/badge/device-MetaMotion%20RL%20%C2%B7%20BMI160%20%2B%20BMM150-8A2BE2.svg)]() [![Protocol](https://img.shields.io/badge/protocol-reverse--engineered-007808.svg)](docs/PROTOCOL.md) [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)

# mmrl_osc - MetaMotion RL head tracker OSC bridge

Use an Mbientlab **MetaMotion RL** (MMRL) as a head tracker for spatial audio on
**macOS**.

The script connects to the MMRL over Bluetooth LE, runs the sensor's on-board
Bosch BSX NDOF (9-axis) fusion (or, with `--vqf`, host-side VQF - Versatile
Quaternion-based Filter - fusion of the raw IMU), and sends the head-tracking
orientation as OSC to a chosen spatial-audio
renderer. Each renderer wants a particular OSC address, argument order, per-axis
sign convention and UDP port, so mmrl-osc uses selectable **profiles**
(`--profile`) instead of blasting a few fixed addresses at once.

<details>
<summary>Supported renderers (19 profiles; click to expand, or run <code>--list-profiles</code>)</summary>

| Profile | OSC address | Default port |
|---|---|---|
| `IEM SceneRotator (quaternion)` (default) | `/SceneRotator/qw,qx,qy,qz` | 9000 |
| `IEM SceneRotator (YPR)` | `/SceneRotator/yaw,pitch,roll` | 9000 |
| `SPARTA` | `/ypr` | 9000 |
| `APL Virtuoso` | `/Virtuoso/quat` | 8000 |
| `Dolby Atmos Renderer` | `/ypr` | 8000 |
| `dearVR` | `/ypr` | 7001 |
| `EAR Production Suite` | `/ypr` | 8000 |
| `Mach1 Monitor` | `/orientation` | 9898 |
| `Nuendo (HeadPose 25Hz)` | `/head_pose` | 7000 |
| `SPAT Revolution` | `/room/1/ypr` | 8000 |
| `Quaternion (generic)` | `/quaternion` | 8000 |
| `YPR (generic)` | `/ypr` | 8000 |
| `a1Rotate` | `/yaw,pitch,roll` | 9001 |
| `Ambi Head HD` | `/yaw,pitch,roll` | 4040 |
| `Audio Brewers` | `/yaw,pitch,roll` | 8585 |
| `DaVinci Resolve` | `/ypr` | 8000 |
| `Genelec Aural ID` | `/euler_x,euler_y,euler_z` | 5005 |
| `Mach1 VideoPlayer` | `/orientation` | 9902 |
| `Spatial Audio Designer` | `/yaw` | 7000 |

</details>

Each profile applies the correct address, per-axis sign/swap and port. Axis and
sign conventions are verified against
[Supperware Bridgehead](https://supperware.co.uk/headtracker-bridgehead)'s
profile list. You can add your own in `~/Library/Application
Support/mmrl-osc/profiles.txt` (four lines per profile - name, address, args,
port - in Supperware's `Profiles.txt` format); `mmrl_osc.py` loads them on top of
the built-in profiles.

**Protocol:** the full reverse-engineered MetaWear GATT protocol and hardware
notes are in [docs/PROTOCOL.md](docs/PROTOCOL.md).

## Related projects

Part of a set of open head-tracking tools for spatial audio:

- **Busola** - a macOS menu-bar app that turns head trackers (including this one)
  into OSC sources; the GUI front-end built on this bridge (separate repo).
- **OpenNx** ([GitLab](https://git.pg.edu.pl/p829296/opennx) / [GitHub](https://github.com/mormegil6/opennx)) - Waves Nx head tracker -> OSC bridge, cross-platform (macOS / Windows / Linux)
- **jabra-elite10-re** ([GitLab](https://git.pg.edu.pl/p829296/jabra-elite10-re) / [GitHub](https://github.com/mormegil6/jabra-elite10-re)) - Jabra Elite 10 Gen 2 BLE GATT protocol reverse-engineering (head-tracking service + Fast Pair auth)

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
- The on-board BSX fusion runs at a fixed ~100 Hz. Its config sets only the mode
  (NDOF) and the accel/gyro ranges - there is no output-rate field - so the
  quaternion rate is not selectable over BLE. A lower rate could save battery, but
  it would have to be set on-device and the firmware does not expose that.
- The raw data registers are not uniform: accelerometer data is on register
  `0x04`, but gyroscope and magnetometer data are on `0x05` (used by `--vqf`).
- IEM SceneRotator accepts the individual `/SceneRotator/qw,qx,qy,qz` parameters
  (what the default profile sends) as well as `/SceneRotator/quaternions`
  (4 floats); the legacy `/SceneRotator/quat` is silently ignored.
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
VQF includes magnetometer-disturbance detection and rejection, which helps keep
the heading from being pulled off course by magnetic interference (near speakers,
motors, laptops, steel) the way a plain 9-axis fusion would be. It falls back to
6-axis (gyro + accelerometer) until magnetometer samples arrive.

```bash
pip install vqf          # required for --vqf (pulls in numpy)
python mmrl_osc.py --vqf
```

The terminal shows a `[VQF]` prefix in this mode. The board temperature is read
and shown in both fusion modes; it is only written to the CSV in `--vqf` mode
(the log needs VQF's bias and disturbance columns). OSC output, tare, LED and
reconnect behave the same as the default mode.

## Logging (--vqf)

`--log FILE` (only with `--vqf`) writes a CSV at about 1 Hz for thermal-drift
analysis, with columns:

    timestamp, nrf_soc_temp_c, gyro_bias_x, gyro_bias_y, gyro_bias_z, quat_w,
    quat_x, quat_y, quat_z, yaw_deg, pitch_deg, roll_deg, mag_disturbance_flag

`nrf_soc_temp_c` is the nRF52 SoC on-die temperature (MetaWear temperature
module, NRF_SOC source = channel 0, scaled 1/8 C per LSB). The MMRL does not
expose the BMI160 die temperature and has no working BMP280/BME280, so the nRF52
SoC reading is used as a board-temperature proxy: it correlates with, but is not
identical to, the gyroscope die temperature. `gyro_bias_*` is VQF's gyroscope
bias estimate in rad/s, and `mag_disturbance_flag` is VQF's
magnetometer-disturbance state (1 = disturbance rejected). Logging temperature
alongside the gyro bias and heading supports characterising how much heading
drift is thermal versus random.

```bash
python mmrl_osc.py --vqf --log drift.csv
```

## Sleep and wake

By default the MMRL advertises continuously, so it never really powers off. This
bridge can deep-sleep it (lowest-power state, via the debug power-save + reset):

- **Hold the button** (~1.5 s) to sleep it immediately; a quick tap still tares.
- **Idle timeout:** it sleeps after `--sleep-timeout` seconds with no orientation
  change (default 300; `0` disables).
- **On exit:** Ctrl-C / kill also sleeps it, so it does not keep advertising once
  you are done (`--no-sleep-on-exit` leaves it awake instead).

**Wake** with a single button press: the device re-advertises, the bridge
auto-reconnects, and the LED blue-blinks to confirm. Wake is firmware-level, so
it is a single press, not a double-click. A sleeping MMRL does not advertise, so
it will not appear in a scan until the button is pressed.

## Features

- Scan and pick a MetaWear/MetaMotion device, or connect directly by UUID.
- Battery level read on connect.
- Quaternion streaming at about 100 Hz, from on-board BSX fusion or, with
  `--vqf`, from host-side VQF fusion of the raw IMU.
- Selectable per-renderer OSC profiles (`--profile` / `--list-profiles`); several
  at once with port-collision detection.
- Yaw/pitch/roll shown in the terminal at about 5 Hz.
- Tare (zero the heading) with **Enter** or a quick tap of the device button.
- Locate the device with **`i`** then Enter: the LED flashes magenta (red+blue)
  a few times, then returns to the battery colour.
- Deep-sleep on a button hold, an idle timeout, or clean exit; a single press
  wakes it (see Sleep and wake above).
- Device LED reflects battery level:
  - green: above 75%
  - orange (red + green): 15-75%
  - pulsing red: below 15%
  - blue: connected but battery could not be read
  - the LED is cleared on a clean stop, so a lit LED means the script is running.
- Auto-reconnect every 3 s if the link drops.
- Clean shutdown on Ctrl-C or kill (SIGTERM): stops the sensors and clears the
  LED before disconnecting.

## Usage (CLI, from source)

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
python mmrl_osc.py --profile SPARTA        # emit the SPARTA profile (port 9000)
python mmrl_osc.py --profile "APL Virtuoso" --port 8001   # override the port
python mmrl_osc.py --list-profiles         # show every profile and exit
python mmrl_osc.py --all           # list all BLE devices if no MetaWear is found
python mmrl_osc.py --vqf           # host-side VQF fusion (needs the vqf package)
python mmrl_osc.py --vqf --log drift.csv   # also log temp/bias/heading at ~1 Hz
python mmrl_osc.py --sleep-timeout 600     # auto-sleep after 10 min idle (0 = never)
python mmrl_osc.py --no-sleep-on-exit      # keep advertising after Ctrl-C
```

Choosing a renderer:

- `--profile NAME` picks the OSC profile (default `IEM SceneRotator (quaternion)`).
  Names match case-insensitively and by unique substring, so `--profile sparta`
  or `--profile virtuoso` work; `--list-profiles` prints them all.
- The profile sets the address, mapping **and port**; `--port` is an optional
  override of that port.
- `--profile` can be repeated to drive several renderers at once. If two selected
  profiles share a port they would collide on the same UDP socket, so mmrl-osc
  refuses and lists the clash; give them different ports or pass
  `--force-collision` to send anyway.

Point the renderer's OSC receive at `127.0.0.1` on the profile's port (shown by
`--list-profiles`). Tap the button or press **Enter** while looking forward to
zero the heading; type **`i`** then Enter to flash the LED magenta and locate the
device; hold the button to put the device to sleep.

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
- **Plugin does not move:** check the OSC port and that the renderer listens on
  the profile's address (see `--list-profiles`); the default profile uses port 9000.
- **Wrong rotation axis or direction:** pick the profile that matches your
  renderer - each applies that renderer's axis/sign convention (verified against
  [Supperware Bridgehead](https://supperware.co.uk/headtracker-bridgehead)). The
  generic `Quaternion`/`YPR` profiles send unremapped values if you need a raw feed.

## Roadmap

- Test on Linux (BlueZ) and Windows (WinRT). The bridge uses bleak, which is
  cross-platform, but only macOS is verified so far; on those platforms the
  device is addressed by its MAC rather than a per-Mac CoreBluetooth UUID.

## Files

| File | Purpose |
|---|---|
| `mmrl_osc.py` | the head tracker bridge (CLI) |
| `profiles.py` | OSC renderer profiles (shared file, kept in sync with opennx) |
| `osc_monitor.py` | OSC listener for testing |
| `requirements.txt` | bleak, python-osc (optional vqf) |
| `docs/PROTOCOL.md` | full reverse-engineered protocol |

## License

MIT. See [LICENSE](LICENSE). Independent, clean-room reimplementation for
interoperability; not affiliated with or endorsed by mbientlab Inc.

## Contact

Bartłomiej Mróz · bartlomiej.mroz@pg.edu.pl · Department of Multimedia Systems, Gdańsk University of Technology · [bmroz.eu](https://bmroz.eu)
