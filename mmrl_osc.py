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
import csv
import math
import os
import signal
import struct
import sys
import time
from datetime import datetime

from bleak import BleakScanner, BleakClient
from pythonosc.udp_client import SimpleUDPClient

import profiles


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

# Temperature (module 0x04), used only in --vqf mode. Read-based and
# multi-channel. Channel 0 is the NRF_SOC source (the nRF52 on-die sensor, always
# available). On this unit the module also enumerates channels 1-3 as
# PRESET_THERM / EXT_THERM / BMP280, but none is populated (the BMP280 read
# returns 0). The BMI160 die temperature is not exposed by the MetaWear firmware,
# so the nRF52 SoC reading is used as a co-located board-temperature proxy.
# Poll by reading the TEMPERATURE register (0x01) with the read bit set:
# [0x04, 0x81, channel]; responses arrive as [0x04, 0x81, channel] + int16 LE.
# The metawear SDK exposes this pre-scaled to Celsius, but over raw GATT we get
# the int16 and apply the module's 1/8 C per LSB ourselves: temp_c = raw / 8.0
# (NOT the BMI160-native raw/512 + 23). Verified live: raw ~210 -> 26.3 C.
# No enable or teardown is needed (a read-based source).
TEMP_CHANNEL   = 0x00                          # NRF_SOC (nRF52 on-die)
TEMP_READ      = bytearray([0x04, 0x81, TEMP_CHANNEL])
TEMP_LSB_PER_C = 8.0

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

# Sleep / wake (debug module 0xFE). Power-save then reset powers the board down
# to its lowest state; a single button press wakes it at the firmware level,
# after which it re-advertises and the bridge reconnects. A quick button tap
# still tares; holding the button >= LONG_PRESS_S sleeps the device.
SLEEP_SEQ = [bytearray([0xFE, 0x07]),    # enable power save (deep sleep on reset)
             bytearray([0xFE, 0x01])]    # reset -> powers down
LONG_PRESS_S      = 1.5                   # button hold time that triggers sleep
MOTION_THRESH_RAD = math.radians(3.0)     # orientation change counted as movement


# ---------------------------------------------------------------------------
# Module-level state
# ---------------------------------------------------------------------------
outputs = []              # list of (profiles.Profile, SimpleUDPClient), built in run()
osc_host = "127.0.0.1"    # OSC destination host (overridable; a front-end may set it)
tare_quat = None          # offset quaternion (w, x, y, z) applied to output, or None
tare_request = False      # set by the Enter key or the device button
retare_target = None      # on a fusion-mode switch: re-zero the new engine to this
mode_settling = False     # True while a freshly switched engine converges
last_print = 0.0          # last terminal update time (5 Hz throttle)
display_prefix = ""       # "[VQF] " in --vqf mode, empty otherwise

# VQF mode. vqf_filter is None in default (BSX) mode and a VQF instance in
# --vqf mode; the notification handler dispatches on it. VQF_CLASS and _np are
# imported only when --vqf is used.
VQF_CLASS = None
_np = None
vqf_filter = None
_mag_seen = False         # True once a magnetometer sample has arrived

# Temperature and CSV logging (--vqf only). latest_temp_c is the most recent
# board temperature; latest_quat/latest_ypr hold the most recent output for the
# 1 Hz CSV row.
latest_temp_c = None
latest_quat = (1.0, 0.0, 0.0, 0.0)
latest_ypr = (0.0, 0.0, 0.0)
csv_file = None
csv_writer = None

# Live reconfiguration. A front-end (the menu-bar) can switch fusion mode and
# logging in place on the running connection - no reconnect - via
# request_reconfig(). active_vqf/active_log are the desired config; reconfig_dirty
# asks stream() to re-sync the live connection to them.
active_vqf = False
active_log = None
reconfig_dirty = False

# Sleep state. sleep_request is set by a long button press or the idle timeout
# and handled in stream() (sends SLEEP_SEQ). sleep_timeout/sleep_on_exit are set
# from the CLI in main().
sleep_request = False
button_down_at = None     # monotonic time of the current button press, or None
idle_ref_quat = None      # reference orientation for idle detection
idle_since = 0.0          # monotonic time the device last moved past the threshold
sleep_timeout = 0.0       # auto-sleep after this many idle seconds (0 = disabled)
sleep_on_exit = False     # put the device to sleep on a clean Ctrl-C / kill

# Connection status, exposed for front-ends (e.g. the menu-bar app). Updated by
# stream()/run(); read-only for consumers.
conn_status = "idle"      # idle / connecting / connected / disconnected / reconnecting
battery_pct = None        # last battery percentage, or None


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
    global tare_quat, tare_request, last_print, latest_quat, latest_ypr
    global idle_ref_quat, idle_since, sleep_request, retare_target

    q = (w, x, y, z)

    # While a freshly switched fusion engine converges, hold the last output so
    # the heading does not jump around; continuity is restored by retare below.
    if mode_settling:
        return

    if retare_target is not None:
        # After a mode switch, re-zero the new engine so its first settled sample
        # reproduces the heading that was shown before the switch (no jump).
        tare_quat = quat_multiply(q, quat_conjugate(retare_target))
        retare_target = None
    elif tare_request:
        # Tare: store the current orientation as the zero reference.
        tare_quat = q
        tare_request = False
        print("\n[tare] heading zeroed")

    # output = inverse(reference) * current
    if tare_quat is not None:
        q = quat_multiply(quat_conjugate(tare_quat), q)

    qw, qx, qy, qz = q
    yaw, pitch, roll = quat_to_ypr(q)
    latest_quat = (qw, qx, qy, qz)
    latest_ypr = (yaw, pitch, roll)

    for prof, client in outputs:
        prof.emit(client, latest_quat, latest_ypr)

    # Update the terminal at ~5 Hz (temperature shown in --vqf mode).
    now = time.monotonic()
    if now - last_print >= 0.2:
        last_print = now
        temp = "" if latest_temp_c is None else f"T={latest_temp_c:5.1f}C  "
        print(f"\r  {display_prefix}{temp}yaw {yaw:+7.1f}  pitch {pitch:+7.1f}  roll {roll:+7.1f}   ",
              end="", flush=True)

    # Idle auto-sleep: if the orientation stays within MOTION_THRESH_RAD for
    # sleep_timeout seconds, request sleep.
    if sleep_timeout > 0 and not sleep_request:
        if idle_ref_quat is None:
            idle_ref_quat, idle_since = (qw, qx, qy, qz), now
        else:
            dot = abs(qw * idle_ref_quat[0] + qx * idle_ref_quat[1]
                      + qy * idle_ref_quat[2] + qz * idle_ref_quat[3])
            if 2.0 * math.acos(min(1.0, dot)) > MOTION_THRESH_RAD:
                idle_ref_quat, idle_since = (qw, qx, qy, qz), now
            elif now - idle_since >= sleep_timeout:
                sleep_request = True
                print(f"\n[sleep] idle {sleep_timeout:.0f}s -> powering down")


# ---------------------------------------------------------------------------
# Notification handling
# ---------------------------------------------------------------------------
def handle_button(state):
    """Switch event: a quick tap tares, a long hold requests sleep."""
    global button_down_at, tare_request, sleep_request
    if state == 0x01:                       # pressed
        button_down_at = time.monotonic()
    elif button_down_at is not None:        # released
        held = time.monotonic() - button_down_at
        button_down_at = None
        if held >= LONG_PRESS_S:
            sleep_request = True
            print("\n[sleep] long-press -> powering down")
        else:
            tare_request = True


def handle_bsx_packet(data):
    """Default mode: on-board fusion quaternion plus the button."""
    # Quaternion: [0x19, 0x07] + 16 bytes (4 x float32 LE).
    if (len(data) >= 18 and data[0] == QUAT_MODULE and data[1] == QUAT_REGISTER):
        w, x, y, z = struct.unpack_from("<ffff", data, 2)
        process_quaternion(w, x, y, z)

    # Push-button press/release.
    elif (len(data) >= 3 and data[0] == SWITCH_MODULE
          and data[1] == SWITCH_REGISTER):
        handle_button(data[2])


def handle_vqf_packet(data):
    """--vqf mode: feed raw acc/gyro/mag into VQF, emit on each gyro sample."""
    global _mag_seen
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

    elif module == SWITCH_MODULE and register == SWITCH_REGISTER:     # button
        handle_button(data[2])


def notification_handler(_sender, data):
    """Dispatch a notification to the active mode's handler."""
    global latest_temp_c
    # The temperature read response is mode-independent (polled in both modes).
    if len(data) >= 5 and data[0] == 0x04 and data[1] == 0x81:
        latest_temp_c = struct.unpack_from("<h", data, 3)[0] / TEMP_LSB_PER_C
        return
    if vqf_filter is None:
        handle_bsx_packet(data)
    else:
        handle_vqf_packet(data)


# ---------------------------------------------------------------------------
# Temperature polling and CSV logging (--vqf)
# ---------------------------------------------------------------------------
CSV_HEADER = [
    "timestamp", "nrf_soc_temp_c",
    "gyro_bias_x", "gyro_bias_y", "gyro_bias_z",
    "quat_w", "quat_x", "quat_y", "quat_z",
    "yaw_deg", "pitch_deg", "roll_deg",
    "mag_disturbance_flag",
]


def write_log_row():
    """Append one CSV row (called at ~1 Hz alongside each temperature read).

    gyro_bias_* are VQF's gyroscope bias estimate in rad/s; mag_disturbance_flag
    is VQF's magnetometer-disturbance detector (1 = disturbance rejected).
    """
    if csv_writer is None or vqf_filter is None:
        return
    bias, _sigma = vqf_filter.getBiasEstimate()          # rad/s
    mag_dist = 1 if vqf_filter.getMagDistDetected() else 0
    qw, qx, qy, qz = latest_quat
    yaw, pitch, roll = latest_ypr
    csv_writer.writerow([
        datetime.now().isoformat(timespec="milliseconds"),
        "" if latest_temp_c is None else f"{latest_temp_c:.3f}",
        f"{bias[0]:.6e}", f"{bias[1]:.6e}", f"{bias[2]:.6e}",
        f"{qw:.6f}", f"{qx:.6f}", f"{qy:.6f}", f"{qz:.6f}",
        f"{yaw:.3f}", f"{pitch:.3f}", f"{roll:.3f}",
        mag_dist,
    ])
    csv_file.flush()


async def temperature_poller(client):
    """Read board temperature once per second (both modes); writes the CSV row.

    The temperature read works in either fusion mode; write_log_row() is a no-op
    unless logging is on and VQF is active (it needs the VQF bias/disturbance
    columns), so the board temperature is shown in BSX mode but only logged in VQF.
    """
    while True:
        try:
            await client.write_gatt_char(MW_CMD_CHAR, TEMP_READ, response=False)
        except Exception:
            pass
        await asyncio.sleep(1.0)   # response updates latest_temp_c during the wait
        write_log_row()


# ---------------------------------------------------------------------------
# Live reconfiguration (in-place fusion-mode / logging switch, no reconnect)
# ---------------------------------------------------------------------------
def request_reconfig(use_vqf, log_path):
    """Ask the running stream to switch fusion mode / logging in place.

    Called by a front-end (the menu-bar). Takes effect within ~0.1 s while
    connected; if not connected, the next connection uses these settings.
    """
    global active_vqf, active_log, reconfig_dirty
    active_vqf = bool(use_vqf)
    active_log = log_path if (log_path and active_vqf) else None
    reconfig_dirty = True


def set_outputs(selected):
    """Replace the live OSC output set (host-side, takes effect immediately, no
    reconnect). `selected` is a list of (profiles.Profile, port)."""
    global outputs
    outputs = [(prof, SimpleUDPClient(osc_host, p)) for prof, p in selected]


async def _switch_sensors(client, to_vqf):
    """Stop the current sensor set and start the other one on the live connection
    (the in-place BSX<->VQF switch). vqf_filter selects the mode for the handler."""
    global vqf_filter, _mag_seen
    for cmd in (RAW_STOP_SEQ if vqf_filter is not None else FUSION_STOP_SEQ):
        await client.write_gatt_char(MW_CMD_CHAR, cmd, response=False)
        await asyncio.sleep(0.02)
    if to_vqf:
        vqf_filter = VQF_CLASS(0.01)
        _mag_seen = False
    else:
        vqf_filter = None
    for cmd in (RAW_START_SEQ if to_vqf else FUSION_START_SEQ):
        await client.write_gatt_char(MW_CMD_CHAR, cmd, response=False)
        await asyncio.sleep(0.05)


def _sync_log():
    """Open or close the CSV so it matches active_log (logged only in VQF mode)."""
    global csv_file, csv_writer
    want = active_log if active_vqf else None
    have = csv_file.name if csv_file is not None else None
    if want == have:
        return
    if csv_file is not None:
        try:
            csv_file.close()
        except Exception:
            pass
        csv_file = None
        csv_writer = None
    if want:
        csv_file = open(want, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(CSV_HEADER)
        csv_file.flush()
        print(f"[log] writing CSV to {want} (~1 Hz)")


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


async def find_metawear(scan_time=6.0):
    """Non-interactive scan: return the first MetaWear/MetaMotion address or None.

    Used by GUI front-ends that cannot prompt for a selection.
    """
    discovered = await BleakScanner.discover(timeout=scan_time, return_adv=True)
    for dev, adv in discovered.values():
        name = adv.local_name or dev.name or ""
        uuids = [u.lower() for u in (adv.service_uuids or [])]
        if "MetaWear" in name or "MetaMotion" in name or MW_SERVICE.lower() in uuids:
            return dev.address
    return None


# ---------------------------------------------------------------------------
# Streaming session (one connection); returns on disconnect so the caller retries
# ---------------------------------------------------------------------------
async def put_to_sleep(client):
    """Send the power-down sequence; the device wakes on a button press."""
    print("\n[sleep] powering down; press the button to wake")
    try:
        for cmd in SLEEP_SEQ:
            await client.write_gatt_char(MW_CMD_CHAR, cmd, response=False)
            await asyncio.sleep(0.05)
        return True
    except Exception:
        return False


async def stream(address):
    """Connect, read battery, set the LED, start streaming until dropped.

    Uses the module-level active_vqf/active_log, so a front-end can switch fusion
    mode and logging in place via request_reconfig() without reconnecting.
    """
    global vqf_filter, _mag_seen, sleep_request, button_down_at, idle_ref_quat
    global conn_status, battery_pct, reconfig_dirty, display_prefix
    global mode_settling, retare_target
    disconnected = asyncio.Event()

    def on_disconnect(_client):
        global conn_status
        conn_status = "disconnected"
        print("\n[ble] disconnected")
        disconnected.set()

    conn_status = "connecting"
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
        battery_pct = batt
        conn_status = "connected"

        try:
            # Brief blue blink on (re)connect - also the "woke up" indicator -
            # then settle to the battery-status colour.
            await client.write_gatt_char(MW_CMD_CHAR, LED_STOP_CLEAR, response=False)
            await client.write_gatt_char(MW_CMD_CHAR,
                led_pattern("blue", high=31, low=0, rise=0, high_t=120, fall=120,
                            pulse=260, repeat=3), response=False)
            await client.write_gatt_char(MW_CMD_CHAR, LED_PLAY, response=False)
            await asyncio.sleep(1.0)
            for cmd in battery_led_commands(batt):
                await client.write_gatt_char(MW_CMD_CHAR, cmd, response=True)
        except Exception as e:
            print(f"[led] could not set LED ({e})")

        # Reset per-connection sleep/idle state.
        sleep_request = False
        button_down_at = None
        idle_ref_quat = None

        # Fresh VQF filter per connection if in VQF mode (None selects BSX in the
        # handler). active_vqf is the current desired mode (may change live).
        if active_vqf:
            vqf_filter = VQF_CLASS(0.01)   # 100 Hz gyro sample period
            _mag_seen = False
        else:
            vqf_filter = None
        display_prefix = "[VQF] " if active_vqf else ""

        # Subscribe to notifications before enabling the data sources.
        await client.start_notify(MW_NOTIFY_CHAR, notification_handler)
        await client.write_gatt_char(MW_CMD_CHAR, CMD_BUTTON_SUB, response=False)

        # Small gaps keep the CoreBluetooth write-without-response queue from
        # dropping commands.
        for cmd in (RAW_START_SEQ if active_vqf else FUSION_START_SEQ):
            await client.write_gatt_char(MW_CMD_CHAR, cmd, response=False)
            await asyncio.sleep(0.05)

        if active_vqf:
            print("[fusion] raw IMU streaming, host-side VQF fusion.")
        else:
            print("[fusion] NDOF enabled, streaming quaternion.")
        print("         Enter or a button tap = tare; hold the button = sleep.  "
              "Ctrl-C to quit.\n")

        # Temperature polling runs in BOTH modes (board temp is always shown); the
        # 1 Hz CSV row is only written when logging is on in VQF mode.
        _sync_log()
        reconfig_dirty = False
        temp_task = asyncio.create_task(temperature_poller(client))

        slept = False
        try:
            # Wait until the device drops, a sleep is requested (long-press or
            # idle timeout), or we are cancelled (Ctrl-C / kill). A reconfig
            # request switches fusion mode / logging in place, no reconnect.
            while not disconnected.is_set():
                if sleep_request:
                    slept = await put_to_sleep(client)
                    break
                if reconfig_dirty:
                    reconfig_dirty = False
                    if active_vqf != (vqf_filter is not None):
                        target = latest_quat        # heading to restore after the switch
                        mode_settling = True        # hold output while the engine settles
                        await _switch_sensors(client, active_vqf)
                        display_prefix = "[VQF] " if active_vqf else ""
                        await asyncio.sleep(1.2)     # let the new fusion engine converge
                        retare_target = target       # next sample re-zeros to the old heading
                        mode_settling = False
                        print(f"\n[mode] switched to {'VQF' if active_vqf else 'BSX'} "
                              f"in place")
                    _sync_log()
                try:
                    await asyncio.wait_for(disconnected.wait(), timeout=0.1)
                except asyncio.TimeoutError:
                    pass
        except asyncio.CancelledError:
            if sleep_on_exit and not slept and client.is_connected:
                slept = await put_to_sleep(client)
            raise
        finally:
            if temp_task is not None:
                temp_task.cancel()
            # Normal teardown only if we did not power the device down (sleeping
            # resets it anyway). Clear the LED first so the indicator releases.
            if client.is_connected and not slept:
                try:
                    await client.write_gatt_char(MW_CMD_CHAR, LED_STOP_CLEAR, response=False)
                    for cmd in (RAW_STOP_SEQ if vqf_filter is not None else FUSION_STOP_SEQ):
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


async def run(address, port, use_vqf, log_path, selected=None):
    """Maintain the connection, reconnecting every 3 s on drop.

    `selected` is a list of (profiles.Profile, port) from profiles.resolve().
    For backward compatibility (the menu-bar front-end calls
    run(address, port, use_vqf, log_path)), if `selected` is None the default
    profile is used with `port` as the port override.
    """
    global outputs, csv_file, csv_writer, conn_status, active_vqf, active_log
    if selected is None:
        selected, _ = profiles.resolve([profiles.DEFAULT_PROFILE], port)
    outputs = [(prof, SimpleUDPClient(osc_host, p)) for prof, p in selected]
    active_vqf = bool(use_vqf)
    active_log = log_path if (log_path and active_vqf) else None
    mode = "VQF (host-side fusion)" if active_vqf else "BSX (on-board fusion)"
    print(f"[mode] {mode}")
    print("[osc] sending:")
    for prof, p in selected:
        print(f"        {prof.name}  ->  {osc_host}:{p}  {prof.address}")
    # The CSV is opened by stream()/_sync_log so it also follows live log toggles.

    # Cancel on SIGINT/SIGTERM so the teardown runs and the LED and sensors are
    # released instead of left lit and streaming.
    loop = asyncio.get_running_loop()
    task = asyncio.current_task()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, task.cancel)
        except (NotImplementedError, RuntimeError, ValueError):
            pass  # e.g. not the main thread (GUI front-end runs run() in a thread)

    tare_task = asyncio.create_task(tare_listener())
    try:
        while True:
            try:
                await stream(address)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                print(f"\n[ble] connection error: {e}")
            conn_status = "reconnecting"
            print("[ble] reconnecting in 3 s...")
            await asyncio.sleep(3)
    finally:
        tare_task.cancel()
        if csv_file is not None:
            csv_file.close()
            csv_file = None
            csv_writer = None


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="MetaMotion RL head tracker with OSC output (macOS / bleak)")
    parser.add_argument("--device", metavar="UUID",
                        help="CoreBluetooth UUID to connect to (skips scanning)")
    parser.add_argument("--profile", action="append", metavar="NAME",
                        help='OSC profile(s) to emit; repeatable. Default: '
                             '"IEM SceneRotator (quaternion)". See --list-profiles.')
    parser.add_argument("--list-profiles", action="store_true",
                        help="list the available OSC profiles and exit")
    parser.add_argument("--host", default="127.0.0.1", metavar="HOST",
                        help="OSC destination host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=None, metavar="PORT",
                        help="override the selected profile's UDP port "
                             "(default: each profile's own port)")
    parser.add_argument("--force-collision", action="store_true",
                        help="allow multiple selected profiles to share a port "
                             "(refused by default)")
    parser.add_argument("--scan-time", type=float, default=8.0,
                        help="BLE scan duration in seconds (default: 8)")
    parser.add_argument("--all", action="store_true", dest="show_all",
                        help="if no MetaWear is found, list all BLE devices to pick from")
    parser.add_argument("--vqf", action="store_true",
                        help="stream raw IMU and fuse on the host with VQF "
                             "(magnetometer-disturbance-aware) instead of on-board BSX")
    parser.add_argument("--log", metavar="FILE",
                        help="write a ~1 Hz CSV (temperature, gyro bias, quaternion, "
                             "mag-disturbance flag) for thermal-drift analysis; needs --vqf")
    parser.add_argument("--sleep-timeout", type=float, default=300.0, metavar="SEC",
                        help="auto-sleep after this many seconds of no movement "
                             "(default 300; 0 disables). Hold the button to sleep anytime.")
    parser.add_argument("--no-sleep-on-exit", action="store_true",
                        help="on Ctrl-C, leave the device advertising instead of "
                             "putting it to sleep")
    args = parser.parse_args()

    # User-defined profiles (shared with the menu-bar app).
    profiles.add_from_file(os.path.expanduser(
        "~/Library/Application Support/mmrl-osc/profiles.txt"))

    if args.list_profiles:
        print(profiles.format_list())
        return

    global sleep_timeout, sleep_on_exit, osc_host
    sleep_timeout = max(0.0, args.sleep_timeout)
    sleep_on_exit = not args.no_sleep_on_exit
    osc_host = args.host

    if args.log and not args.vqf:
        print("[log] --log requires --vqf; ignoring --log")
        args.log = None

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

    selection, err = profiles.resolve(args.profile or [profiles.DEFAULT_PROFILE],
                                      args.port)
    if err:
        print(f"[profile] {err}; use --list-profiles to see valid names.")
        sys.exit(2)
    clash = profiles.collisions(selection)
    if clash:
        for port, ns in clash.items():
            print(f"[profile] port {port} collision: {', '.join(ns)}")
        if not args.force_collision:
            print("[profile] these profiles would share a UDP port and collide. "
                  "Give them different ports with --port, pick one profile, or pass "
                  "--force-collision to send anyway.")
            sys.exit(2)
        print("[profile] proceeding despite the collision (--force-collision).")

    async def main_async():
        address = args.device
        if not address:
            address = await scan_and_pick(args.scan_time, show_all=args.show_all)
            if not address:
                print("No device selected. Exiting.")
                return
        await run(address, args.port, args.vqf, args.log, selection)

    try:
        asyncio.run(main_async())
    except (KeyboardInterrupt, asyncio.CancelledError):
        print("\n[exit] stopping and disconnecting...")


if __name__ == "__main__":
    main()
