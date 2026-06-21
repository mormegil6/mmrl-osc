#!/usr/bin/env python3
"""
mmrl_osc.py - MetaMotion RL head tracker with OSC output for macOS.

Connects to an Mbientlab MetaMotion RL over BLE (bleak / CoreBluetooth) and sends
the orientation quaternion as OSC for IEM SceneRotator, SPARTA (/ypr) and APL
Virtuoso.

Two fusion modes:
  default : on-board Bosch BSX NDOF fusion; the device streams the quaternion.
  --vqf   : stream raw accelerometer/gyroscope/magnetometer and fuse on the host
            with VQF (magnetometer-disturbance-aware). Needs the vqf package.

The Mbientlab `metawear` Python SDK does not run on macOS (its warble backend has
no CoreBluetooth support), so this speaks the Mbientlab GATT protocol directly
through bleak. CoreBluetooth addresses peripherals by a per-Mac UUID, not a MAC.

Requires: bleak, python-osc  (and vqf for --vqf)
"""

import argparse
import asyncio
import math
import signal
import struct
import sys
import time

from bleak import BleakScanner, BleakClient
from pythonosc.udp_client import SimpleUDPClient


# ---------------------------------------------------------------------------
# Mbientlab MetaWear GATT protocol
# ---------------------------------------------------------------------------
# Commands are written to MW_CMD_CHAR; sensor and button data come back as
# notifications on MW_NOTIFY_CHAR.
MW_SERVICE     = "326a9000-85cb-9195-d9dd-464cfbbae75a"
MW_CMD_CHAR    = "326a9001-85cb-9195-d9dd-464cfbbae75a"  # command characteristic (write)
MW_NOTIFY_CHAR = "326a9006-85cb-9195-d9dd-464cfbbae75a"  # notification characteristic

# Standard BLE battery level characteristic (0x2A19).
BATTERY_CHAR   = "00002a19-0000-1000-8000-00805f9b34fb"

# Commands are [module, register, payload...]. Modules: 0x03 accelerometer,
# 0x13 gyroscope, 0x15 magnetometer, 0x19 sensor fusion.

# --- Default mode: on-board BSX sensor fusion (module 0x19) ---
# NDOF quaternion streaming requires, in order: set the fusion mode, configure
# the raw acc/gyro/mag, enable and start each of them, enable the fusion
# quaternion output and start the engine, then subscribe to the quaternion
# register. The raw-sensor configuration/start and the subscribe are both
# required; without them the fusion reports enabled but emits no data. Byte
# values follow MetaWear-SDK-Cpp for BMI160 + BMM150 hardware.
FUSION_START_SEQ = [
    bytearray([0x19, 0x02, 0x01, 0x13]),  # fusion mode NDOF, acc 16G | gyro 2000dps
    bytearray([0x03, 0x03, 0x28, 0x0c]),  # acc config:  100 Hz, +/-16 G
    bytearray([0x13, 0x03, 0x28, 0x00]),  # gyro config: 100 Hz, 2000 dps
    bytearray([0x15, 0x04, 0x04, 0x0e]),  # mag repetitions (regular preset, 9/15)
    bytearray([0x15, 0x03, 0x02]),        # mag data rate: 25 Hz
    bytearray([0x03, 0x02, 0x01, 0x00]),  # acc:  enable sampling
    bytearray([0x13, 0x02, 0x01, 0x00]),  # gyro: enable sampling
    bytearray([0x15, 0x02, 0x01, 0x00]),  # mag:  enable sampling
    bytearray([0x03, 0x01, 0x01]),        # acc:  start
    bytearray([0x13, 0x01, 0x01]),        # gyro: start
    bytearray([0x15, 0x01, 0x01]),        # mag:  start
    bytearray([0x19, 0x03, 0x08, 0x00]),  # fusion output enable: quaternion (1<<3)
    bytearray([0x19, 0x01, 0x01]),        # fusion enable: start engine
    bytearray([0x19, 0x07, 0x01]),        # subscribe to quaternion notifications
]
FUSION_STOP_SEQ = [
    bytearray([0x19, 0x07, 0x00]),        # unsubscribe quaternion
    bytearray([0x19, 0x01, 0x00]),        # fusion stop
    bytearray([0x19, 0x03, 0x00, 0x7f]),  # fusion clear output mask
    bytearray([0x03, 0x01, 0x00]),        # acc  stop
    bytearray([0x13, 0x01, 0x00]),        # gyro stop
    bytearray([0x15, 0x01, 0x00]),        # mag  stop
    bytearray([0x03, 0x02, 0x00, 0x01]),  # acc  disable sampling
    bytearray([0x13, 0x02, 0x00, 0x01]),  # gyro disable sampling
    bytearray([0x15, 0x02, 0x00, 0x01]),  # mag  disable sampling
]
# BSX quaternion notification: [0x19, 0x07] + 4 little-endian float32 (w,x,y,z).
QUAT_MODULE   = 0x19
QUAT_REGISTER = 0x07

# --- VQF mode (--vqf): raw IMU streaming, host-side fusion ---
# Raw data registers: accelerometer DATA_INTERRUPT 0x04, gyroscope DATA 0x05,
# magnetometer MAG_DATA 0x05 (gyro/mag data are on 0x05, not 0x04; from the
# MetaWear-SDK-Cpp register headers).
RAW_START_SEQ = [
    bytearray([0x03, 0x03, 0x28, 0x03]),  # acc config:  100 Hz, +/-2 G
    bytearray([0x13, 0x03, 0x28, 0x00]),  # gyro config: 100 Hz, 2000 dps
    bytearray([0x15, 0x04, 0x04, 0x0e]),  # mag repetitions (regular preset, 9/15)
    bytearray([0x15, 0x03, 0x02]),        # mag data rate: 25 Hz
    bytearray([0x03, 0x02, 0x01, 0x00]),  # acc:  enable sampling
    bytearray([0x13, 0x02, 0x01, 0x00]),  # gyro: enable sampling
    bytearray([0x15, 0x02, 0x01, 0x00]),  # mag:  enable sampling
    bytearray([0x03, 0x01, 0x01]),        # acc:  start
    bytearray([0x13, 0x01, 0x01]),        # gyro: start
    bytearray([0x15, 0x01, 0x01]),        # mag:  start
    bytearray([0x03, 0x04, 0x01]),        # subscribe acc data
    bytearray([0x13, 0x05, 0x01]),        # subscribe gyro data
    bytearray([0x15, 0x05, 0x01]),        # subscribe mag data
]
RAW_STOP_SEQ = [
    bytearray([0x03, 0x04, 0x00]),        # unsubscribe acc
    bytearray([0x13, 0x05, 0x00]),        # unsubscribe gyro
    bytearray([0x15, 0x05, 0x00]),        # unsubscribe mag
    bytearray([0x03, 0x01, 0x00]),        # acc  stop
    bytearray([0x13, 0x01, 0x00]),        # gyro stop
    bytearray([0x15, 0x01, 0x00]),        # mag  stop
    bytearray([0x03, 0x02, 0x00, 0x01]),  # acc  disable sampling
    bytearray([0x13, 0x02, 0x00, 0x01]),  # gyro disable sampling
    bytearray([0x15, 0x02, 0x00, 0x01]),  # mag  disable sampling
]
# Raw-sensor scale factors. Acc +/-2 G: 16384 LSB/g. Gyro +/-2000 dps:
# 16.4 LSB/(deg/s). Mag: 16 LSB/uT.
ACC_LSB_PER_G   = 16384.0
GRAVITY         = 9.80665
GYR_LSB_PER_DPS = 16.4
DEG2RAD         = math.pi / 180.0
MAG_LSB_PER_UT  = 16.0

# Push-button (module 0x01). Subscribing delivers a notification on each
# press/release; a press triggers a tare.
CMD_BUTTON_SUB    = bytearray([0x01, 0x01, 0x01])
CMD_BUTTON_UNSUB  = bytearray([0x01, 0x01, 0x00])

# LED (module 0x02): PLAY=0x01, STOP=0x02, CONFIG=0x03. Colors green=0, red=1,
# blue=2; the channels are independent, so red+green reads as orange.
LED_PLAY          = bytearray([0x02, 0x01, 0x01])
LED_STOP_CLEAR    = bytearray([0x02, 0x02, 0x01])
LED_COLOR = {"green": 0, "red": 1, "blue": 2}

# Button notification: [0x01, 0x01, state]; state 1 = pressed, 0 = released.
SWITCH_MODULE   = 0x01
SWITCH_REGISTER = 0x01


# ---------------------------------------------------------------------------
# Module-level state
# ---------------------------------------------------------------------------
osc = None                # SimpleUDPClient, created in run()
tare_quat = None          # offset quaternion (w, x, y, z) applied to output, or None
tare_request = False      # set by the Enter key or the device button
last_print = 0.0          # last terminal update time (5 Hz throttle)
display_prefix = ""       # "[VQF] " in --vqf mode, empty otherwise

# VQF mode. vqf_filter is None in default (BSX) mode and a VQF instance in
# --vqf mode; the notification handler dispatches on it. VQF_CLASS and _np are
# imported only when --vqf is used.
VQF_CLASS = None
_np = None
vqf_filter = None
_mag_seen = False         # True once a magnetometer sample has arrived


# ---------------------------------------------------------------------------
# Quaternion math
# ---------------------------------------------------------------------------
def quat_conjugate(q):
    """Conjugate (the inverse for a unit quaternion)."""
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_multiply(a, b):
    """Hamilton product a * b."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_to_ypr(q):
    """Convert a quaternion (w, x, y, z) to yaw/pitch/roll in degrees (ZYX)."""
    w, x, y, z = q

    # roll (rotation about x)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (rotation about y), clamped to avoid NaN at the poles
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # yaw (rotation about z)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


# ---------------------------------------------------------------------------
# LED status
# ---------------------------------------------------------------------------
def led_pattern(color, high=16, low=16, rise=0, high_t=600, fall=0,
                pulse=1000, delay=0, repeat=0xFF):
    """Build the 17-byte LED pattern-config command for one color channel.

    Defaults give a steady glow. Layout: [0x02, 0x03, color, 0x02, high, low,
    rise, high_t, fall, pulse, delay, repeat] with the four times as uint16 LE.
    """
    return (bytearray([0x02, 0x03, LED_COLOR[color], 0x02, high & 0xFF, low & 0xFF])
            + struct.pack("<HHHHH", rise, high_t, fall, pulse, delay)
            + bytearray([repeat & 0xFF]))


def battery_led_commands(batt):
    """LED commands for a battery level.

    >75 green, 15-75 red+green (orange), <15 pulsing red, unknown blue.
    """
    cmds = [LED_STOP_CLEAR]  # clear any previous pattern first
    if batt is None:
        cmds.append(led_pattern("blue"))
    elif batt < 15:
        cmds.append(led_pattern("red", low=0, rise=250, high_t=300,
                                fall=250, pulse=1500))  # pulse for attention
    elif batt <= 75:
        cmds.append(led_pattern("red"))
        cmds.append(led_pattern("green"))              # red + green = orange
    else:
        cmds.append(led_pattern("green"))
    cmds.append(LED_PLAY)
    return cmds


# ---------------------------------------------------------------------------
# Quaternion output: tare, OSC, terminal display (shared by both modes)
# ---------------------------------------------------------------------------
def process_quaternion(w, x, y, z):
    """Apply tare, send OSC, and update the terminal line."""
    global tare_quat, tare_request, last_print

    q = (w, x, y, z)

    # Tare: store the current orientation as the zero reference.
    if tare_request:
        tare_quat = q
        tare_request = False
        print("\n[tare] heading zeroed")

    # output = inverse(reference) * current
    if tare_quat is not None:
        q = quat_multiply(quat_conjugate(tare_quat), q)

    qw, qx, qy, qz = q
    yaw, pitch, roll = quat_to_ypr(q)

    osc.send_message("/SceneRotator/quaternions", [qw, qx, qy, qz])  # IEM Plugin Suite
    osc.send_message("/ypr", [yaw, pitch, roll])                     # SPARTA/Atmoky/dearVR
    osc.send_message("/Virtuoso/quat", [qw, qx, qy, qz])             # APL Virtuoso

    # Update the terminal at ~5 Hz.
    now = time.monotonic()
    if now - last_print >= 0.2:
        last_print = now
        print(f"\r  {display_prefix}yaw {yaw:+7.1f}  pitch {pitch:+7.1f}  roll {roll:+7.1f}   ",
              end="", flush=True)


# ---------------------------------------------------------------------------
# Notification handling
# ---------------------------------------------------------------------------
def handle_bsx_packet(data):
    """Default mode: on-board fusion quaternion plus the button."""
    global tare_request

    # Quaternion: [0x19, 0x07] + 16 bytes (4 x float32 LE).
    if (len(data) >= 18 and data[0] == QUAT_MODULE and data[1] == QUAT_REGISTER):
        w, x, y, z = struct.unpack_from("<ffff", data, 2)
        process_quaternion(w, x, y, z)

    # Button press tares the heading.
    elif (len(data) >= 3 and data[0] == SWITCH_MODULE
          and data[1] == SWITCH_REGISTER and data[2] == 0x01):
        tare_request = True
        print("\n[tare] button pressed")


def handle_vqf_packet(data):
    """--vqf mode: feed raw acc/gyro/mag into VQF, emit on each gyro sample."""
    global tare_request, _mag_seen
    if len(data) < 3:
        return
    module, register = data[0], data[1]

    if module == 0x03 and register == 0x04 and len(data) >= 8:        # accelerometer
        ax, ay, az = struct.unpack_from("<hhh", data, 2)
        acc = _np.array([ax, ay, az]) * (GRAVITY / ACC_LSB_PER_G)     # m/s^2
        vqf_filter.updateAcc(acc)

    elif module == 0x13 and register == 0x05 and len(data) >= 8:      # gyroscope
        gx, gy, gz = struct.unpack_from("<hhh", data, 2)
        gyr = _np.array([gx, gy, gz]) * (DEG2RAD / GYR_LSB_PER_DPS)   # rad/s
        vqf_filter.updateGyr(gyr)
        # Gyro drives the filter clock; read the orientation once per gyro sample.
        q = vqf_filter.getQuat9D() if _mag_seen else vqf_filter.getQuat6D()
        process_quaternion(float(q[0]), float(q[1]), float(q[2]), float(q[3]))

    elif module == 0x15 and register == 0x05 and len(data) >= 8:      # magnetometer
        mx, my, mz = struct.unpack_from("<hhh", data, 2)
        mag = _np.array([mx, my, mz]) / MAG_LSB_PER_UT                # uT
        vqf_filter.updateMag(mag)
        _mag_seen = True

    elif (module == SWITCH_MODULE and register == SWITCH_REGISTER
          and data[2] == 0x01):                                       # button
        tare_request = True
        print("\n[tare] button pressed")


def notification_handler(_sender, data):
    """Dispatch a notification to the active mode's handler."""
    if vqf_filter is None:
        handle_bsx_packet(data)
    else:
        handle_vqf_packet(data)


# ---------------------------------------------------------------------------
# Scanning and device selection
# ---------------------------------------------------------------------------
async def scan_and_pick(scan_time, show_all=False):
    """Scan for MetaWear/MetaMotion devices and return the chosen UUID.

    Matches on advertised name or the MetaWear service UUID. With show_all,
    lists every BLE device when no MetaWear is found.
    """
    while True:
        print(f"[scan] scanning {scan_time:.0f}s for Mbientlab devices...")
        # return_adv=True also yields advertisement data (service UUIDs), not
        # just the device name.
        discovered = await BleakScanner.discover(timeout=scan_time, return_adv=True)
        items = list(discovered.values())

        def is_metawear(dev, adv):
            name = adv.local_name or dev.name or ""
            if "MetaWear" in name or "MetaMotion" in name:
                return True
            uuids = [u.lower() for u in (adv.service_uuids or [])]
            return MW_SERVICE.lower() in uuids

        found = [(d, a) for (d, a) in items if is_metawear(d, a)]

        # --all fallback: list everything, strongest signal first.
        if not found and show_all:
            print("[scan] no MetaWear match; listing ALL devices (--all).")
            found = sorted(items, key=lambda da: -(da[1].rssi or -999))

        if not found:
            print("[scan] no MetaWear/MetaMotion devices found.")
            print("       Wake the MMRL (press its button, the LED should blink),")
            print("       make sure it isn't connected to another app/phone, and")
            print("       that it's charged. Use --all to list every BLE device.")
            choice = input("Press Enter to rescan, or 'q' to quit: ").strip().lower()
            if choice == "q":
                return None
            continue

        print("\nFound devices:")
        for i, (d, a) in enumerate(found):
            name = a.local_name or d.name or "(no name)"
            print(f"  [{i}] {name:<18} {d.address}   rssi {a.rssi}")

        sel = input("\nSelect device number (r=rescan, q=quit): ").strip().lower()
        if sel == "q":
            return None
        if sel == "r":
            continue
        if sel.isdigit() and int(sel) < len(found):
            return found[int(sel)][0].address
        print("Invalid selection.")


# ---------------------------------------------------------------------------
# Streaming session (one connection); returns on disconnect so the caller retries
# ---------------------------------------------------------------------------
async def stream(address, use_vqf):
    """Connect, read battery, set the LED, start streaming until dropped."""
    global vqf_filter, _mag_seen
    disconnected = asyncio.Event()

    def on_disconnect(_client):
        print("\n[ble] disconnected")
        disconnected.set()

    async with BleakClient(address, disconnected_callback=on_disconnect) as client:
        print(f"[ble] connected to {address}")

        # Battery level: a single byte, 0-100 %.
        batt = None
        try:
            raw = await client.read_gatt_char(BATTERY_CHAR)
            batt = raw[0]
            print(f"[battery] {batt}%")
        except Exception as e:
            print(f"[battery] unavailable ({e})")

        try:
            for cmd in battery_led_commands(batt):
                await client.write_gatt_char(MW_CMD_CHAR, cmd, response=True)
        except Exception as e:
            print(f"[led] could not set LED ({e})")

        # Fresh VQF filter per connection (None selects BSX mode in the handler).
        if use_vqf:
            vqf_filter = VQF_CLASS(0.01)   # 100 Hz gyro sample period
            _mag_seen = False
        else:
            vqf_filter = None

        # Subscribe to notifications before enabling the data sources.
        await client.start_notify(MW_NOTIFY_CHAR, notification_handler)
        await client.write_gatt_char(MW_CMD_CHAR, CMD_BUTTON_SUB, response=False)

        # Small gaps keep the CoreBluetooth write-without-response queue from
        # dropping commands.
        for cmd in (RAW_START_SEQ if use_vqf else FUSION_START_SEQ):
            await client.write_gatt_char(MW_CMD_CHAR, cmd, response=False)
            await asyncio.sleep(0.05)

        if use_vqf:
            print("[fusion] raw IMU streaming, host-side VQF fusion.")
        else:
            print("[fusion] NDOF enabled, streaming quaternion.")
        print("         Press Enter or the device button to tare.  Ctrl-C to quit.\n")

        try:
            await disconnected.wait()       # until the device drops or this is cancelled
        finally:
            # Best-effort teardown. Clear the LED first so the status indicator
            # is released even if a later write fails.
            if client.is_connected:
                try:
                    await client.write_gatt_char(MW_CMD_CHAR, LED_STOP_CLEAR, response=False)
                    for cmd in (RAW_STOP_SEQ if use_vqf else FUSION_STOP_SEQ):
                        await client.write_gatt_char(MW_CMD_CHAR, cmd, response=False)
                        await asyncio.sleep(0.02)
                    await client.write_gatt_char(MW_CMD_CHAR, CMD_BUTTON_UNSUB, response=False)
                    await client.stop_notify(MW_NOTIFY_CHAR)
                except Exception:
                    pass


async def tare_listener():
    """Request a tare on each Enter keypress."""
    global tare_request
    loop = asyncio.get_running_loop()
    while True:
        line = await loop.run_in_executor(None, sys.stdin.readline)
        if line == "":             # EOF (stdin not a TTY): stop listening
            return
        tare_request = True


async def run(address, port, use_vqf):
    """Maintain the connection, reconnecting every 3 s on drop."""
    global osc
    osc = SimpleUDPClient("127.0.0.1", port)
    mode = "VQF (host-side fusion)" if use_vqf else "BSX (on-board fusion)"
    print(f"[mode] {mode}")
    print(f"[osc] sending to 127.0.0.1:{port}  "
          f"(/SceneRotator/quaternions, /ypr, /Virtuoso/quat)")

    # Cancel on SIGINT/SIGTERM so the teardown runs and the LED and sensors are
    # released instead of left lit and streaming.
    loop = asyncio.get_running_loop()
    task = asyncio.current_task()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, task.cancel)
        except (NotImplementedError, RuntimeError):
            pass

    tare_task = asyncio.create_task(tare_listener())
    try:
        while True:
            try:
                await stream(address, use_vqf)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                print(f"\n[ble] connection error: {e}")
            print("[ble] reconnecting in 3 s...")
            await asyncio.sleep(3)
    finally:
        tare_task.cancel()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="MetaMotion RL head tracker with OSC output (macOS / bleak)")
    parser.add_argument("--device", metavar="UUID",
                        help="CoreBluetooth UUID to connect to (skips scanning)")
    parser.add_argument("--port", type=int, default=8000,
                        help="OSC UDP port on localhost (default: 8000)")
    parser.add_argument("--scan-time", type=float, default=8.0,
                        help="BLE scan duration in seconds (default: 8)")
    parser.add_argument("--all", action="store_true", dest="show_all",
                        help="if no MetaWear is found, list all BLE devices to pick from")
    parser.add_argument("--vqf", action="store_true",
                        help="stream raw IMU and fuse on the host with VQF "
                             "(magnetometer-disturbance-aware) instead of on-board BSX")
    args = parser.parse_args()

    if args.vqf:
        global VQF_CLASS, _np, display_prefix
        try:
            from vqf import VQF
            import numpy as numpy_mod
        except ImportError:
            print("[vqf] --vqf needs the vqf package: pip install vqf")
            return
        VQF_CLASS = VQF
        _np = numpy_mod
        display_prefix = "[VQF] "

    async def main_async():
        address = args.device
        if not address:
            address = await scan_and_pick(args.scan_time, show_all=args.show_all)
            if not address:
                print("No device selected. Exiting.")
                return
        await run(address, args.port, args.vqf)

    try:
        asyncio.run(main_async())
    except (KeyboardInterrupt, asyncio.CancelledError):
        print("\n[exit] stopping and disconnecting...")


if __name__ == "__main__":
    main()
