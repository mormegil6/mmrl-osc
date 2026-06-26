"""
profiles.py - selectable OSC output profiles for head-tracker bridges.

SHARED FILE - kept identical in the openMMRL, openNx and supperware-vqf repos. If you edit it
in one, copy it to the other so the two bridges stay consistent.

Each profile maps the bridge's internal orientation (a tared quaternion, plus
yaw/pitch/roll derived from it) to one renderer's expected OSC address, argument
order, per-axis signs, and default UDP port. Selecting a profile chooses the
address, the argument mapping, AND the port.

Axis/sign conventions verified against Supperware Bridgehead's profile list
(https://supperware.co.uk/headtracker-bridgehead). Credit to Supperware for
collecting these renderer conventions.

Mapping syntax
--------------
`args` is a comma-separated list of terms. Each term is one of:
  qw qx qy qz     a quaternion component (from the tared quaternion)
  yaw pitch roll  an Euler angle in degrees (derived from the tared quaternion)
  a number        a literal constant, e.g. 0
A leading '-' negates the term (e.g. -qy, -pitch). A trailing ':0' is the
Supperware "learnable zero" marker: the tare/zero offset is applied to that axis.
This bridge already tares the whole orientation, so every value is taken from the
tared quaternion and the marker is accepted but needs no extra handling.

Address syntax
--------------
  "/ypr"                          one OSC message; all mapped args sent as a list.
  "/SceneRotator/[qw,qx,qy,qz]"   one message per bracketed suffix - i.e.
                                  /SceneRotator/qw, /SceneRotator/qx, ... each
                                  carrying the positionally-matching mapped arg.
"""

from collections import defaultdict
from dataclasses import dataclass, field

_QUAT = {"qw": 0, "qx": 1, "qy": 2, "qz": 3}
_EULER = {"yaw": 0, "pitch": 1, "roll": 2}


def _parse_term(tok):
    """Parse one mapping term into (sign, kind, key, learn_zero)."""
    tok = tok.strip()
    learn_zero = False
    if ":" in tok:                      # Supperware learnable-zero marker, e.g. yaw:0
        tok, marker = tok.split(":", 1)
        learn_zero = marker.strip() == "0"
        tok = tok.strip()
    sign = 1.0
    if tok.startswith("-"):
        sign, tok = -1.0, tok[1:].strip()
    if tok in _QUAT:
        return (sign, "q", _QUAT[tok], learn_zero)
    if tok in _EULER:
        return (sign, "e", _EULER[tok], learn_zero)
    return (sign, "c", float(tok), learn_zero)   # literal constant


@dataclass
class Profile:
    name: str
    address: str            # may contain a [a,b,c] suffix list
    args: str               # comma-separated mapping terms
    port: int
    base: str = field(init=False)
    suffixes: list = field(init=False, default=None)
    terms: list = field(init=False, default_factory=list)

    def __post_init__(self):
        if "[" in self.address:
            i, j = self.address.index("["), self.address.index("]")
            self.base = self.address[:i]
            self.suffixes = [s.strip() for s in self.address[i + 1:j].split(",")]
        else:
            self.base = self.address
            self.suffixes = None
        self.terms = [_parse_term(t) for t in self.args.split(",")]

    def values(self, q, ypr):
        """Map quaternion q=(w,x,y,z) and ypr=(yaw,pitch,roll) degrees to the
        profile's ordered argument values."""
        out = []
        for sign, kind, key, _lz in self.terms:
            if kind == "q":
                out.append(sign * q[key])
            elif kind == "e":
                out.append(sign * ypr[key])
            else:
                out.append(sign * key)
        return out

    def emit(self, client, q, ypr):
        """Send this profile's OSC message(s) via `client` (a python-osc
        SimpleUDPClient already bound to the right host:port)."""
        vals = self.values(q, ypr)
        if self.suffixes is None:
            client.send_message(self.base, vals)
        else:
            for suffix, v in zip(self.suffixes, vals):
                client.send_message(self.base + suffix, [v])


# Profile table. Mappings follow Supperware Bridgehead's profile-list conventions.
_DEFS = [
    ("IEM SceneRotator (quaternion)", "/SceneRotator/[qw,qx,qy,qz]",   "qw,-qy,qx,-qz",            9000),
    ("IEM SceneRotator (YPR)",        "/SceneRotator/[yaw,pitch,roll]", "yaw,-pitch,-roll",        9000),
    ("SPARTA",                        "/ypr",                           "-yaw,-pitch,roll",        9000),
    ("APL Virtuoso",                  "/Virtuoso/quat",                 "qw,qy,-qx,qz",            8000),
    ("Dolby Atmos Renderer",          "/ypr",                           "yaw,pitch,roll",          8000),
    ("dearVR",                        "/ypr",                           "yaw,pitch,roll",          7001),
    ("EAR Production Suite",          "/ypr",                           "-yaw,-pitch,roll",        8000),
    ("Mach1 Monitor",                 "/orientation",                   "yaw,pitch,roll",          9898),
    ("Nuendo (HeadPose 25Hz)",        "/head_pose",                     "0,0,0,0,-pitch,-yaw,-roll", 7000),
    ("SPAT Revolution",               "/room/1/ypr",                    "yaw,pitch,roll",          8000),
    ("Quaternion (generic)",          "/quaternion",                    "qw,qx,qy,qz",             8000),
    ("YPR (generic)",                 "/ypr",                           "yaw,pitch,roll",          8000),
    # Further apps from Supperware Bridgehead's profile list.
    ("a1Rotate",                      "/[yaw,pitch,roll]",              "-yaw,pitch,roll",         9001),
    ("Ambi Head HD",                  "/[yaw,pitch,roll]",              "-yaw:0,pitch:0,-roll:0",  4040),
    ("Audio Brewers",                 "/[yaw,pitch,roll]",              "yaw:0,pitch:0,roll:0",    8585),
    ("DaVinci Resolve",               "/ypr",                           "yaw,pitch,roll",          8000),
    # Bridgehead marks Genelec's pitch term "-pitch__"; the modifier is not
    # documented here, so the plain axis is used.
    ("Genelec Aural ID",              "/[euler_x,euler_y,euler_z]",     "-pitch,yaw,-roll",        5005),
    ("Mach1 VideoPlayer",             "/orientation",                   "-yaw,pitch,roll:0",       9902),
    ("Spatial Audio Designer",        "/yaw",                           "yaw:0",                   7000),
]

_BUILTINS = {name: Profile(name, addr, args, port) for name, addr, args, port in _DEFS}
PROFILES = dict(_BUILTINS)
DEFAULT_PROFILE = "IEM SceneRotator (quaternion)"


def reset_to_builtins():
    """Drop any file-loaded profiles, keeping only the built-in set."""
    global PROFILES
    PROFILES = dict(_BUILTINS)


def add_from_file(path):
    """Add user profiles from a Bridgehead-style text file. Returns (added, bad).

    Format: blocks of four non-comment lines separated by blank lines -
        Name
        /address          (may use the [a,b,c] suffix-list form)
        args              (same mapping syntax as the built-ins)
        port              (a bare port, or 'local PORT' / 'host PORT'; the host is
                           ignored, as the destination host is set separately)
    Lines starting with '#' are comments. A profile with the same name as an
    existing one overrides it. Malformed blocks are skipped (named in `bad`)."""
    try:
        with open(path, encoding="utf-8") as f:
            text = f.read()
    except OSError:
        return 0, []

    blocks, block = [], []
    for raw in text.splitlines():
        s = raw.strip()
        if not s:
            if block:
                blocks.append(block)
                block = []
        elif not s.startswith("#"):
            block.append(s)
    if block:
        blocks.append(block)

    added, bad = 0, []
    for b in blocks:
        if len(b) != 4 or not b[1].startswith("/"):
            continue
        name, address, args, dest = b
        try:
            port = int(dest.split()[-1])     # last token is the port; host ignored
            PROFILES[name] = Profile(name, address, args, port)
            added += 1
        except (ValueError, IndexError, KeyError):
            bad.append(name)
    return added, bad


def names():
    """All profile names, in definition order."""
    return list(PROFILES.keys())


def match(name):
    """Resolve a user-typed name: exact, then case-insensitive exact, then a
    unique case-insensitive substring. Returns a Profile, or None if no unique
    match."""
    if name in PROFILES:
        return PROFILES[name]
    low = name.lower()
    for n, p in PROFILES.items():
        if n.lower() == low:
            return p
    hits = [p for n, p in PROFILES.items() if low in n.lower()]
    return hits[0] if len(hits) == 1 else None


def resolve(selected_names, port_override=None):
    """Return (selection, error). `selection` is a list of (Profile, port);
    `error` is a message string or None. `port_override` (if not None) replaces
    every profile's default port."""
    selection = []
    for n in selected_names:
        p = match(n)
        if p is None:
            return None, f"unknown profile: {n!r}"
        port = port_override if port_override is not None else p.port
        selection.append((p, port))
    return selection, None


def collisions(selection):
    """Given a [(Profile, port)] selection, return {port: [names]} for any port
    used by more than one profile (they would collide on the same UDP socket)."""
    byport = defaultdict(list)
    for p, port in selection:
        byport[port].append(p.name)
    return {port: ns for port, ns in byport.items() if len(ns) > 1}


def format_list():
    """Human-readable table for --list-profiles."""
    w = max(len(n) for n in PROFILES)
    lines = ["Available OSC profiles (name  ->  address  @ default port):", ""]
    for p in PROFILES.values():
        lines.append(f"  {p.name:<{w}}  {p.address}  @ {p.port}")
    lines += ["",
              f'Default: "{DEFAULT_PROFILE}"',
              "Select with --profile NAME (repeatable; --port overrides the port)."]
    return "\n".join(lines)
