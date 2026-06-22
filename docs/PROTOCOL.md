# MetaMotion RL BLE GATT Protocol

Reverse-engineered protocol for streaming orientation from an Mbientlab
**MetaMotion RL** (MMRL) over Bluetooth LE, so the device can be used on macOS
without the official `metawear` Python SDK (which does not run on Darwin). All
findings below were obtained empirically with `bleak` on macOS (Apple Silicon,
macOS 14) and cross-checked against the
[MetaWear-SDK-Cpp](https://github.com/mbientlab/MetaWear-SDK-Cpp) source.

> Status: **verified.** GATT map, on-board fusion start/stop, quaternion packet
> format, raw-IMU streaming registers, scales, LED and button are all confirmed
> against live data and physical motion on a BMI160 + BMM150 unit.

---

## Device identity

Module-information reads (`[module, 0x80]`) on the tested unit:

| Module | Read | Response | Meaning |
|--------|------|----------|---------|
| Accelerometer 0x03 | `03 80` | `03 80 01 02` | Bosch BMI160 accelerometer |
| Gyroscope 0x13 | `13 80` | `13 80 00 01` | implementation 0x00 = BMI160 gyroscope |
| Magnetometer 0x15 | `15 80` | `15 80 00 02` | Bosch BMM150 magnetometer |
| Sensor fusion 0x19 | `19 80` | `19 80 00 03 03 00 06 00 02 00 01 00` | Bosch BSX fusion |

Battery level reads 100 on the tested unit via the standard characteristic.

### Addressing

macOS addresses BLE peripherals by a per-host **CoreBluetooth UUID**, not by
their MAC address, so pass that UUID to `--device`. The value differs per Mac
and is printed during the scan. The hardware MAC is not exposed by CoreBluetooth
and cannot be used on macOS.

### Advertising and connection

The default advertising timeout is 0, so the device advertises indefinitely and
does not auto-sleep within seconds. A MetaWear accepts a single connection at a
time and stops advertising while connected, so an empty scan or a failed connect
usually means another central (for example the phone app or another script
instance) holds the connection. Free it there first.

---

## GATT

| Role | UUID |
|------|------|
| MetaWear service | `326a9000-85cb-9195-d9dd-464cfbbae75a` |
| Command (write) | `326a9001-85cb-9195-d9dd-464cfbbae75a` |
| Notify | `326a9006-85cb-9195-d9dd-464cfbbae75a` |
| Battery level (read) | `00002a19-0000-1000-8000-00805f9b34fb` |

Commands are written to the command characteristic as `[module, register,
payload...]`. Sensor data and button events arrive as notifications on the
notify characteristic. Multi-byte values are little-endian.

Module ids used here: `0x01` switch/button, `0x02` LED, `0x03` accelerometer,
`0x13` gyroscope, `0x15` magnetometer, `0x19` sensor fusion.

---

## Mode 1: on-board BSX fusion (default)

The on-board Bosch BSX engine computes the orientation (NDOF, 9-axis) and the
host reads the quaternion. Enabling it is not a single command: the raw
accelerometer, gyroscope and magnetometer must be configured and started first,
because the fusion engine consumes them. Omitting the raw-sensor setup, or the
final subscribe, leaves the engine reporting "enabled" while emitting no data.

Start sequence (BMI160 + BMM150):

```
19 02 01 13   fusion mode NDOF, acc range 16 G, gyro range 2000 dps
03 03 28 0c   acc config:  100 Hz, +/-16 G
13 03 28 00   gyro config: 100 Hz, 2000 dps
15 04 04 0e   mag repetitions (regular preset, xy=9 / z=15)
15 03 02      mag data rate 25 Hz
03 02 01 00   acc enable sampling
13 02 01 00   gyro enable sampling
15 02 01 00   mag enable sampling
03 01 01      acc start
13 01 01      gyro start
15 01 01      mag start
19 03 08 00   fusion output enable: quaternion bit (1 << 3)
19 01 01      fusion enable (start engine)
19 07 01      subscribe to quaternion notifications
```

Stop sequence is the reverse: unsubscribe `19 07 00`, fusion stop `19 01 00`,
clear output mask `19 03 00 7f`, then stop and disable each of acc/gyro/mag
(`03/13/15 01 00` and `03/13/15 02 00 01`).

Quaternion notification: header `[0x19, 0x07]` followed by 4 little-endian
float32 in the order `w, x, y, z` (16 bytes, 18-byte packet).

---

## Mode 2: raw IMU for host-side VQF (--vqf)

Instead of the on-board fusion, the raw accelerometer, gyroscope and
magnetometer are streamed and fused on the host with VQF. The raw data registers
differ from the fusion register: the accelerometer data is on register `0x04`,
but the gyroscope and magnetometer data are on register `0x05` (from the
MetaWear-SDK-Cpp register headers: `DATA_INTERRUPT` = 0x04 for the accelerometer,
`DATA` = 0x05 for the gyroscope, `MAG_DATA` = 0x05 for the magnetometer).

Start sequence:

```
03 03 28 03   acc config:  100 Hz, +/-2 G
13 03 28 00   gyro config: 100 Hz, 2000 dps
15 04 04 0e   mag repetitions (regular preset, xy=9 / z=15)
15 03 02      mag data rate 25 Hz
03 02 01 00   acc enable sampling
13 02 01 00   gyro enable sampling
15 02 01 00   mag enable sampling
03 01 01      acc start
13 01 01      gyro start
15 01 01      mag start
03 04 01      subscribe acc data
13 05 01      subscribe gyro data
15 05 01      subscribe mag data
```

Each data notification carries a 2-byte header and a 6-byte payload of 3 x
int16 little-endian (x, y, z):

| Sensor | Header | Scale | Convert to |
|--------|--------|-------|------------|
| Accelerometer | `03 04` | 16384 LSB/g (+/-2 G) | divide by 16384 for g, times 9.80665 for m/s^2 |
| Gyroscope | `13 05` | 16.4 LSB/(deg/s) (+/-2000 dps) | divide by 16.4 for deg/s, times pi/180 for rad/s |
| Magnetometer | `15 05` | 16 LSB/uT | divide by 16 for uT |

The bridge feeds gyro (rad/s), accelerometer (m/s^2) and magnetometer (uT) into
VQF and reads `getQuat9D()` once magnetometer samples arrive, otherwise
`getQuat6D()`. VQF includes magnetometer-disturbance detection and rejection,
which keeps the heading stable in electromagnetically noisy environments.

---

## Temperature (module 0x04)

Read-based and multi-channel. On the MMRL the channel-to-source map (from a
module-info read `[0x04, 0x80]`) is:

| Channel | Source | Notes |
|---------|--------|-------|
| 0 | NRF_SOC (nRF52 on-die) | always available; the one used here |
| 1 | PRESET_THERM | not populated |
| 2 | EXT_THERM | not populated |
| 3 | BMP280 | not populated (read returns 0) |

The BMI160 die temperature is not exposed by the firmware, so channel 0 (the
nRF52 SoC) is used as a co-located board-temperature proxy. Poll once per second
by reading the TEMPERATURE register (0x01) with the read bit set:
`[0x04, 0x81, 0x00]`; the response is `[0x04, 0x81, channel]` + int16 LE. The
metawear SDK pre-scales this to Celsius; over raw GATT, apply the module's
1/8 C per LSB: `temp_c = raw / 8.0` (verified live: raw ~210 -> 26.3 C). This is
NOT the BMI160-native `raw/512 + 23` (that is the chip's own register). The
source is read-only, so no enable or teardown is needed; the MODE register
(0x02) only configures an external thermistor's pins and would misconfigure the
channel.

### BMI160 die temperature: not accessible

The BMI160's own die-temperature register (`0x20/0x21`, `raw/512 + 23`) cannot be
reached from the host. The firmware surfaces it through no module: the
temperature module offers only the sources above, and the Bosch acc/gyro modules
expose no temperature signal. The I2C serial-passthrough module (0x0D) is present
and functional, but a full bus scan (`[0x0D, 0x81, addr, 0x00, id, 0x01]` for
addr 0x08-0x77) acked all 112 reads and returned data on none, including 0x68 and
0x69 - so the IMU is on SPI, not the passthrough I2C bus. SPI passthrough would
need the BMI160's exact nRF52 chip-select/clock/MOSI/MISO pins (undocumented) and
would collide with the firmware's own SPI traffic during streaming. The NRF_SOC
(nRF52 on-die) reading is therefore used as a co-located board-temperature proxy:
it correlates with, but is not identical to, the gyroscope die temperature.

---

## LED (module 0x02)

Registers: `0x01` play, `0x02` stop, `0x03` pattern config. Colors are
independent channels: green = 0, red = 1, blue = 2 (red + green reads as orange).

Pattern config is a 17-byte command: `[0x02, 0x03, color, 0x02, high, low,
rise, high_t, fall, pulse, delay, repeat]` where the four times are uint16
little-endian. Writing one or more channel patterns then `02 01 01` (play)
lights them; `02 02 01` stops and clears.

The bridge uses the LED for battery status: green above 75%, red + green
(orange) for 15-75%, a pulsing red below 15%, and blue when the battery cannot
be read. The LED is cleared on a clean stop.

---

## Button (module 0x01)

Subscribe with `01 01 01`; unsubscribe with `01 01 00`. Each press/release sends
a notification `[0x01, 0x01, state]` with state 1 = pressed, 0 = released. The
bridge tares the heading on a press.

---

## OSC output

On each orientation update the bridge sends three messages to `127.0.0.1`
(default port 8000):

| OSC address | Arguments | Target |
|-------------|-----------|--------|
| `/SceneRotator/quaternions` | `qw qx qy qz` | IEM Plugin Suite (SceneRotator) |
| `/ypr` | `yaw pitch roll` (degrees, ZYX) | SPARTA, Atmoky, dearVR |
| `/Virtuoso/quat` | `qw qx qy qz` | APL Virtuoso |

IEM SceneRotator listens on `/SceneRotator/quaternions` (4 floats); the earlier
`/SceneRotator/quat` address is silently ignored by the plugin.

Taring stores the current quaternion as a reference and outputs
`inverse(reference) * current`, so the tared pose reads as identity.

---

## Sources

- [MetaWear-SDK-Cpp](https://github.com/mbientlab/MetaWear-SDK-Cpp): module
  source under `src/metawear/sensor/cpp/` (`sensor_fusion.cpp`,
  `accelerometer_bosch.cpp`, `gyro_bosch.cpp`, `magnetometer_bmm150.cpp`) and the
  exact start/stop byte sequences in `test/test_sensor_fusion.py`.
- [Mbientlab API specification](https://docs.mbientlab.com/api-specification/).
