"""
GCode Print Time Estimator — Hybrid M73 + Trapezoidal Physics
=============================================================

Accuracy model (best → fallback):

  TIER 1 — PrusaSlicer M73 markers (most accurate)
  ─────────────────────────────────────────────────
  PrusaSlicer (formerly Slic3rPE) embeds M73 progress/time commands
  throughout the gcode file:

      M73 P12 R45   ; normal mode:  12% done, 45 minutes remaining
      M73 Q12 S45   ; stealth mode: 12% done, 45 minutes remaining

  These are computed by the slicer's own look-ahead + acceleration
  simulator — the same model the Prusa LCD uses — so they are far more
  accurate than any post-hoc physics simulation.

  The estimator pre-scans the file, builds a line→remaining-seconds
  lookup table from every M73 R/S value, then uses those as "anchor"
  points.

  TIER 2 — Sub-minute physics interpolation
  ─────────────────────────────────────────
  M73 granularity is 1 minute and markers appear every several lines.
  Between two consecutive M73 anchors the estimator uses the trapezoidal
  velocity + classic-Jerk model to distribute the known delta-time across
  the individual lines proportionally. This gives per-line accuracy down
  to ~10 ms.

  TIER 3 — Pure physics (no M73 in file)
  ───────────────────────────────────────
  If the file has no M73 markers (non-PrusaSlicer slicers, old firmware),
  the original full trapezoidal simulation is used as before.

Usage (CLI):
    python gcode_time_estimator.py print.gcode              # total time
    python gcode_time_estimator.py print.gcode --verbose    # per-line
    python gcode_time_estimator.py print.gcode --line 42
    python gcode_time_estimator.py print.gcode --line 100-200
    python gcode_time_estimator.py print.gcode --pause-at 500
    python gcode_time_estimator.py print.gcode --pause-at 500 --buffer 16

Usage (as module in gcode_sender_node.py):
    from gcode_time_estimator import LiveEstimator
    est = LiveEstimator()
    est.load_file("print.gcode")                 # pre-scan M73 + build table
    ms = est.estimate_and_commit(cmd, line_idx)  # advance state, return ms
    ms = est.remaining_ms()                      # ms left on current line
    ms = est.total_remaining_ms()                # ms left in whole print
"""

import re
import math
import time as _time_module
import argparse
from copy import deepcopy
from dataclasses import dataclass
from typing import Optional, List, Tuple


# ─────────────────────────────────────────────────────────────
# Data classes
# ─────────────────────────────────────────────────────────────

@dataclass
class PrinterConfig:
    """Motion parameters — defaults match Prusa MK3S / MK4."""
    print_accel:   float = 1000.0
    travel_accel:  float = 3000.0
    retract_accel: float = 3000.0
    jerk_xy: float = 8.0
    jerk_z:  float = 0.4
    jerk_e:  float = 2.5
    junction_deviation: Optional[float] = None
    max_feed_xy: float = 500.0
    max_feed_z:  float = 20.0
    max_feed_e:  float = 120.0


@dataclass
class PrinterState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    e: float = 0.0
    feed_rate: float = 3600.0
    is_absolute:   bool = True
    is_e_absolute: bool = True
    last_vx: float = 0.0
    last_vy: float = 0.0
    last_vz: float = 0.0


@dataclass
class LineResult:
    line_number:    int
    raw:            str
    command:        str
    seconds:        float
    distance:       float = 0.0
    feed_mm_s:      float = 0.0
    entry_v:        float = 0.0
    exit_v:         float = 0.0
    note:           str   = ""
    m73_remaining_s:  float = -1.0
    m73_progress_pct: float = -1.0


# ─────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────

def _parse_val(line: str, param: str) -> Optional[float]:
    m = re.search(rf'(?<![A-Z]){param}([+-]?\d+\.?\d*(?:[eE][+-]?\d+)?)',
                  line, re.IGNORECASE)
    return float(m.group(1)) if m else None

def _strip_comment(line: str) -> str:
    return line.split(';')[0].strip()

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ─────────────────────────────────────────────────────────────
# Physics: trapezoidal velocity profile
# ─────────────────────────────────────────────────────────────

def trapezoid_time(distance: float, v_entry: float, v_exit: float,
                   v_max: float, accel: float) -> Tuple[float, float, float]:
    if distance <= 1e-9:
        return 0.0, v_entry, v_exit
    if accel <= 0:
        return (distance / v_max if v_max > 0 else 0.0), v_entry, v_exit

    v_entry = _clamp(v_entry, 0.0, v_max)
    v_exit  = _clamp(v_exit,  0.0, v_max)
    d_accel = (v_max**2 - v_entry**2) / (2.0 * accel)
    d_decel = (v_max**2 - v_exit**2)  / (2.0 * accel)

    if d_accel + d_decel > distance:
        v_peak_sq = (2.0 * accel * distance + v_entry**2 + v_exit**2) / 2.0
        v_peak = min(math.sqrt(max(v_peak_sq, 0.0)), v_max)
        return max((v_peak - v_entry + v_peak - v_exit) / accel, 0.0), v_entry, v_exit

    d_cruise = distance - d_accel - d_decel
    t_accel  = (v_max - v_entry) / accel
    t_cruise = d_cruise / v_max if v_max > 0 else 0.0
    t_decel  = (v_max - v_exit)  / accel
    return max(t_accel + t_cruise + t_decel, 0.0), v_entry, v_exit


def junction_velocity(prev_vx, prev_vy, prev_vz, ux, uy, uz,
                      target_v, cfg: PrinterConfig) -> float:
    if cfg.junction_deviation is not None:
        prev_len = math.sqrt(prev_vx**2 + prev_vy**2 + prev_vz**2)
        if prev_len < 1e-9:
            return 0.0
        cos_a = _clamp((prev_vx*ux + prev_vy*uy + prev_vz*uz) / prev_len, -1.0, 1.0)
        sh = math.sqrt((1.0 - cos_a) / 2.0)
        if sh < 1e-9:
            return target_v
        R = cfg.junction_deviation * (1.0 - sh) / sh
        return min(math.sqrt(cfg.print_accel * R), target_v)

    scale = 1.0
    if target_v > 0:
        dvx = abs(target_v * ux - prev_vx)
        dvy = abs(target_v * uy - prev_vy)
        dvz = abs(target_v * uz - prev_vz)
        if dvx > cfg.jerk_xy: scale = min(scale, cfg.jerk_xy / dvx)
        if dvy > cfg.jerk_xy: scale = min(scale, cfg.jerk_xy / dvy)
        if dvz > cfg.jerk_z:  scale = min(scale, cfg.jerk_z  / dvz)
    return target_v * scale


# ─────────────────────────────────────────────────────────────
# Core: simulate one G-code line
# ─────────────────────────────────────────────────────────────

def calc_line_time(raw_line: str,
                   state: PrinterState,
                   cfg: PrinterConfig) -> LineResult:
    """Simulate one line. MUTATES state and cfg. Returns LineResult."""
    stripped = _strip_comment(raw_line.upper())
    parts    = stripped.split()
    cmd      = parts[0] if parts else ""

    # ── G0 / G1 ──────────────────────────────────────────────
    if cmd in ("G0", "G1"):
        f = _parse_val(stripped, 'F')
        if f is not None:
            state.feed_rate = f
        x = _parse_val(stripped, 'X'); y = _parse_val(stripped, 'Y')
        z = _parse_val(stripped, 'Z'); e = _parse_val(stripped, 'E')

        if state.is_absolute:
            nx = x if x is not None else state.x
            ny = y if y is not None else state.y
            nz = z if z is not None else state.z
        else:
            nx = state.x + (x or 0.0)
            ny = state.y + (y or 0.0)
            nz = state.z + (z or 0.0)

        dx, dy, dz = nx - state.x, ny - state.y, nz - state.z
        d_xyz = math.sqrt(dx**2 + dy**2 + dz**2)
        is_e_move  = e is not None
        is_retract = is_e_move and d_xyz < 1e-9
        is_travel  = d_xyz > 1e-9 and not is_e_move

        accel = (cfg.retract_accel if is_retract else
                 cfg.travel_accel  if is_travel  else cfg.print_accel)

        v_max = min(state.feed_rate / 60.0, cfg.max_feed_xy)
        if abs(dz) > 1e-9 and abs(dx) < 1e-9 and abs(dy) < 1e-9:
            v_max = min(v_max, cfg.max_feed_z)

        seconds = entry_v = exit_v = 0.0
        if d_xyz > 1e-9:
            ux, uy, uz = dx/d_xyz, dy/d_xyz, dz/d_xyz
            entry_v = junction_velocity(state.last_vx, state.last_vy, state.last_vz,
                                        ux, uy, uz, v_max, cfg)
            seconds, entry_v, exit_v = trapezoid_time(d_xyz, entry_v, 0.0, v_max, accel)
            state.last_vx = ux * exit_v
            state.last_vy = uy * exit_v
            state.last_vz = uz * exit_v
        elif is_retract and e is not None:
            de = abs((e - state.e) if state.is_e_absolute else e)
            if de > 1e-9:
                seconds, _, _ = trapezoid_time(
                    de, 0.0, 0.0, min(v_max, cfg.max_feed_e), accel)
            state.last_vx = state.last_vy = state.last_vz = 0.0

        state.x, state.y, state.z = nx, ny, nz
        if e is not None:
            state.e = e if state.is_e_absolute else state.e + e

        kind = "retract" if is_retract else "travel" if is_travel else "print"
        note = (f"{kind} dist={d_xyz:.2f}mm F={state.feed_rate:.0f} "
                f"entry={entry_v:.2f}mm/s accel={accel:.0f}mm/s²")
        return LineResult(0, raw_line.rstrip(), cmd, seconds,
                          d_xyz, v_max, entry_v, exit_v, note)

    # ── G2 / G3 Arc ──────────────────────────────────────────
    if cmd in ("G2", "G3"):
        f = _parse_val(stripped, 'F')
        if f is not None:
            state.feed_rate = f
        x = _parse_val(stripped, 'X'); y = _parse_val(stripped, 'Y')
        i = _parse_val(stripped, 'I') or 0.0
        j = _parse_val(stripped, 'J') or 0.0
        r = _parse_val(stripped, 'R')
        nx = (x if x is not None else state.x) if state.is_absolute else state.x+(x or 0.0)
        ny = (y if y is not None else state.y) if state.is_absolute else state.y+(y or 0.0)
        if r is not None:
            chord = math.sqrt((nx-state.x)**2 + (ny-state.y)**2)
            arc_len = 2*abs(r)*math.asin(min(chord/(2*abs(r)),1.0)) if abs(r)>1e-9 else 0.0
        else:
            cx=state.x+i; cy=state.y+j
            radius = math.sqrt(i**2+j**2)
            a0 = math.atan2(state.y-cy, state.x-cx)
            a1 = math.atan2(ny-cy, nx-cx)
            if cmd=="G2":
                if a1>a0: a1-=2*math.pi
            else:
                if a1<a0: a1+=2*math.pi
            arc_len = abs(radius*(a1-a0))
        v_max = min(state.feed_rate/60.0, cfg.max_feed_xy)
        seconds, _, _ = trapezoid_time(arc_len, 0.0, 0.0, v_max, cfg.print_accel)
        state.x=nx; state.y=ny
        state.last_vx=state.last_vy=state.last_vz=0.0
        return LineResult(0, raw_line.rstrip(), cmd, seconds, arc_len, v_max, 0, 0,
                          f"arc {arc_len:.2f}mm")

    # ── G4 Dwell ─────────────────────────────────────────────
    if cmd == "G4":
        p = _parse_val(stripped,'P'); s = _parse_val(stripped,'S')
        secs = (p/1000.0) if p is not None else (s or 0.0)
        return LineResult(0, raw_line.rstrip(), cmd, secs, note=f"dwell {secs:.3f}s")

    # ── G28 Home ─────────────────────────────────────────────
    if cmd == "G28":
        state.last_vx=state.last_vy=state.last_vz=0.0
        state.x=state.y=state.z=0.0
        return LineResult(0, raw_line.rstrip(), cmd, 15.0, note="homing ~15s")

    # ── Positioning modes ─────────────────────────────────────
    if cmd=="G90": state.is_absolute=True;  return LineResult(0,raw_line.rstrip(),cmd,0,note="abs")
    if cmd=="G91": state.is_absolute=False; return LineResult(0,raw_line.rstrip(),cmd,0,note="rel")
    if cmd=="M82": state.is_e_absolute=True;  return LineResult(0,raw_line.rstrip(),cmd,0,note="E abs")
    if cmd=="M83": state.is_e_absolute=False; return LineResult(0,raw_line.rstrip(),cmd,0,note="E rel")

    # ── G92 Set position ──────────────────────────────────────
    if cmd == "G92":
        xv=_parse_val(stripped,'X'); yv=_parse_val(stripped,'Y')
        zv=_parse_val(stripped,'Z'); ev=_parse_val(stripped,'E')
        if xv is not None: state.x=xv
        if yv is not None: state.y=yv
        if zv is not None: state.z=zv
        if ev is not None: state.e=ev
        return LineResult(0,raw_line.rstrip(),cmd,0,note="set pos")

    # ── M204 Acceleration ─────────────────────────────────────
    if cmd == "M204":
        p=_parse_val(stripped,'P'); t=_parse_val(stripped,'T')
        r=_parse_val(stripped,'R'); s=_parse_val(stripped,'S')
        if s is not None: cfg.print_accel=cfg.travel_accel=s
        if p is not None: cfg.print_accel=p
        if t is not None: cfg.travel_accel=t
        if r is not None: cfg.retract_accel=r
        return LineResult(0,raw_line.rstrip(),cmd,0,
                          note=f"accel P={cfg.print_accel} T={cfg.travel_accel}")

    # ── M205 Jerk ─────────────────────────────────────────────
    if cmd == "M205":
        jx=_parse_val(stripped,'X'); jy=_parse_val(stripped,'Y')
        je=_parse_val(stripped,'E'); jz=_parse_val(stripped,'Z')
        jd=_parse_val(stripped,'J')
        if jx is not None: cfg.jerk_xy=jx
        if jy is not None: cfg.jerk_xy=min(cfg.jerk_xy,jy)
        if je is not None: cfg.jerk_e=je
        if jz is not None: cfg.jerk_z=jz
        if jd is not None: cfg.junction_deviation=jd
        return LineResult(0,raw_line.rstrip(),cmd,0,note="set jerk")

    # ── M203 Max feedrate ─────────────────────────────────────
    if cmd == "M203":
        xv=_parse_val(stripped,'X'); yv=_parse_val(stripped,'Y')
        zv=_parse_val(stripped,'Z'); ev=_parse_val(stripped,'E')
        if xv is not None: cfg.max_feed_xy=xv
        if yv is not None: cfg.max_feed_xy=min(cfg.max_feed_xy,yv)
        if zv is not None: cfg.max_feed_z=zv
        if ev is not None: cfg.max_feed_e=ev
        return LineResult(0,raw_line.rstrip(),cmd,0,note="set max feed")

    # ── M73 Progress (PrusaSlicer) ────────────────────────────
    # Zero execution time — data extracted during pre-scan.
    if cmd == "M73":
        p=_parse_val(stripped,'P'); r=_parse_val(stripped,'R')
        q=_parse_val(stripped,'Q'); s=_parse_val(stripped,'S')
        pct = p if p is not None else q
        rem = r if r is not None else s
        note = ""
        if pct is not None: note += f"progress={pct:.0f}%  "
        if rem is not None: note += f"remaining={rem:.0f}min"
        return LineResult(0, raw_line.rstrip(), cmd, 0.0,
                          m73_progress_pct=float(pct) if pct is not None else -1.0,
                          m73_remaining_s=(rem*60.0) if rem is not None else -1.0,
                          note=note.strip())

    # ── Temperature waits ─────────────────────────────────────
    if cmd == "M109":
        return LineResult(0,raw_line.rstrip(),cmd,60.0,  note="wait nozzle ~60s")
    if cmd == "M190":
        return LineResult(0,raw_line.rstrip(),cmd,120.0, note="wait bed ~120s")

    return LineResult(0, raw_line.rstrip(), cmd, 0.0, note="no time")


# ─────────────────────────────────────────────────────────────
# M73 Scanner
# ─────────────────────────────────────────────────────────────

@dataclass
class M73Anchor:
    line_number:  int
    remaining_s:  float
    progress_pct: float


class M73Scanner:
    """
    Pre-scans a gcode file and extracts all M73 time anchors.
    PrusaSlicer emits pairs of M73 lines (normal + stealth mode) at the
    same file position — we deduplicate by (line_number, remaining_s).
    """

    def __init__(self):
        self.anchors:    List[M73Anchor] = []
        self.has_m73:    bool = False
        self.total_lines: int = 0

    def scan(self, lines: List[str]) -> bool:
        self.anchors = []
        self.total_lines = len(lines)

        for i, raw in enumerate(lines, start=1):
            upper = _strip_comment(raw.upper())
            parts = upper.split()
            if not parts or parts[0] != "M73":
                continue

            p = _parse_val(upper, 'P')
            r = _parse_val(upper, 'R')

            # Only process normal-mode lines (M73 P<pct> R<min>).
            # PrusaSlicer also emits M73 Q<pct> S<min> for stealth/silent mode
            # on the very next line with a DIFFERENT (usually larger) remaining
            # time. Including both would create non-monotonic anchor sequences
            # and break the segment scaling. We always prefer the normal-mode
            # P+R values which match the standard print profile.
            if p is None or r is None:
                continue

            self.anchors.append(M73Anchor(i, r * 60.0, float(p)))

        self.has_m73 = len(self.anchors) > 0
        return self.has_m73

    def get_total_seconds(self) -> float:
        return self.anchors[0].remaining_s if self.anchors else 0.0

    def get_remaining_at(self, line_number: int) -> float:
        """Linearly interpolated remaining seconds at line_number."""
        if not self.anchors:
            return 0.0
        if line_number >= self.anchors[-1].line_number:
            return 0.0
        if line_number <= self.anchors[0].line_number:
            return self.anchors[0].remaining_s
        for idx in range(len(self.anchors) - 1):
            a0 = self.anchors[idx]
            a1 = self.anchors[idx+1]
            if a0.line_number <= line_number <= a1.line_number:
                span = a1.line_number - a0.line_number
                if span == 0:
                    return a0.remaining_s
                frac = (line_number - a0.line_number) / span
                return a0.remaining_s + frac * (a1.remaining_s - a0.remaining_s)
        return 0.0

    def get_segment(self, line_number: int
                    ) -> Tuple[Optional[M73Anchor], Optional[M73Anchor]]:
        prev = None
        for a in self.anchors:
            if a.line_number > line_number:
                return prev, a
            prev = a
        return prev, None

    def summary(self) -> str:
        if not self.has_m73:
            return "mode=pure-physics (no M73 markers)"
        total = self.get_total_seconds()
        h, r = divmod(int(total), 3600)
        m, s = divmod(r, 60)
        return (f"mode=hybrid-M73  anchors={len(self.anchors)}  "
                f"slicer_total={h}h{m}m{s}s")


# ─────────────────────────────────────────────────────────────
# Hybrid line table builder
# ─────────────────────────────────────────────────────────────

def build_hybrid_line_table(lines: List[str],
                             scanner: M73Scanner) -> List[LineResult]:
    """
    Build a per-line time table.

    If M73 anchors exist: run physics simulation, then within each
    M73 segment scale all physics times so their sum matches the slicer's
    known delta. This retains the relative shape of the physics curve
    while anchoring the absolute values to the slicer's model.
    """
    cfg   = PrinterConfig()
    state = PrinterState()
    results: List[LineResult] = []

    for i, raw in enumerate(lines, start=1):
        r = calc_line_time(raw, state, cfg)
        r.line_number = i
        results.append(r)

    if not scanner.has_m73 or len(scanner.anchors) < 2:
        return results

    # Scale physics times within each M73 segment
    anchors = scanner.anchors
    for seg in range(len(anchors) - 1):
        a0 = anchors[seg]
        a1 = anchors[seg+1]
        slicer_delta = a0.remaining_s - a1.remaining_s
        if slicer_delta <= 0:
            continue
        lo_idx = a0.line_number - 1   # 0-based
        hi_idx = a1.line_number - 1
        phys_sum = sum(r.seconds for r in results[lo_idx:hi_idx])
        if phys_sum <= 1e-9:
            continue
        scale = slicer_delta / phys_sum
        for r in results[lo_idx:hi_idx]:
            r.seconds *= scale
            if scale != 1.0 and r.note:
                r.note += f" [×{scale:.3f}]"

    return results


# ─────────────────────────────────────────────────────────────
# LiveEstimator
# ─────────────────────────────────────────────────────────────

class LiveEstimator:
    """
    Stateful per-line estimator for gcode_sender_node.

    Usage:
        est = LiveEstimator()
        est.load_file("print.gcode")           # call once before starting

        # In the send loop, just before each send_now():
        ms = est.estimate_and_commit(cmd, i)   # i = 0-based line index

        # When pause is requested:
        ms = est.remaining_ms()                # ms left on current line
        total = est.total_remaining_ms()       # ms left in whole print
    """

    def __init__(self):
        self._line_table:  List[LineResult] = []
        self._scanner      = M73Scanner()
        self._cfg          = PrinterConfig()
        self._state        = PrinterState()
        self._file_loaded  = False

        self.current_line_ms:  float = 0.0
        self.current_line_cmd: str   = ""
        self._line_sent_time:  float = 0.0
        self._current_idx:     int   = 0

    # ── File loading ──────────────────────────────────────────

    def load_file(self, filepath: str) -> bool:
        """Pre-scan file. Call before starting the print."""
        try:
            with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
                lines = f.readlines()
            return self.load_lines(lines)
        except Exception:
            return False

    def load_lines(self, lines: List[str]) -> bool:
        """Load from list of raw lines."""
        self._scanner.scan(lines)
        self._line_table = build_hybrid_line_table(lines, self._scanner)
        self._file_loaded = True
        self.reset_state()
        return self._scanner.has_m73

    def reset_state(self):
        """Call at the start of each new print."""
        self._cfg   = PrinterConfig()
        self._state = PrinterState()
        self.current_line_ms  = 0.0
        self.current_line_cmd = ""
        self._line_sent_time  = 0.0
        self._current_idx     = 0
    
    def reset(self):
        # Position state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.e = 0.0

        # Feedrate (mm/min)
        self.feedrate = 1800.0

        # Timing
        self.total_elapsed_ms = 0.0
        self.current_line_ms = 0.0
        self.current_line_start_time = None

    def estimate_and_commit(self, line: str) -> float:
        """
        Estimate time for a G-code line and update internal state.
        Returns estimated duration in milliseconds.
        """

        line = line.strip()

        if not line or line.startswith(";"):
            return 0.0

        # Feedrate update
        f_match = re.search(r"F([0-9.]+)", line)
        if f_match:
            self.feedrate = float(f_match.group(1))

        # Only estimate G0/G1
        if line.startswith("G0") or line.startswith("G1"):

            new_x = self._extract_axis(line, "X", self.x)
            new_y = self._extract_axis(line, "Y", self.y)
            new_z = self._extract_axis(line, "Z", self.z)

            dx = new_x - self.x
            dy = new_y - self.y
            dz = new_z - self.z

            distance = math.sqrt(dx*dx + dy*dy + dz*dz)

            # mm/min → mm/sec
            speed = self.feedrate / 60.0

            if speed <= 0:
                duration_ms = 0.0
            else:
                duration_ms = (distance / speed) * 1000.0

            # Update position
            self.x = new_x
            self.y = new_y
            self.z = new_z

            # Commit timing
            self.total_elapsed_ms += duration_ms
            self.current_line_ms = duration_ms
            self.current_line_start_time = _time_module.time()

            return duration_ms

        return 0.0

    def remaining_ms(self):
        """
        Time remaining in current line.
        """
        if self.current_line_start_time is None:
            return 0.0

        elapsed = (_time_module.time() - self.current_line_start_time) * 1000.0
        return max(0.0, self.current_line_ms - elapsed)

    def _extract_axis(self, line, axis, default):
        match = re.search(rf"{axis}([-+]?[0-9]*\.?[0-9]+)", line)
        return float(match.group(1)) if match else default

    def peek_line_ms(self, raw_line: str, line_idx: int = -1) -> float:
        """Estimate without advancing state."""
        if self._file_loaded and 0 <= line_idx < len(self._line_table):
            return self._line_table[line_idx].seconds * 1000.0
        sc = deepcopy(self._state); cc = deepcopy(self._cfg)
        return calc_line_time(raw_line, sc, cc).seconds * 1000.0

    # ── Timing queries ────────────────────────────────────────

    def elapsed_ms(self) -> float:
        return (_time_module.time() - self._line_sent_time) * 1000.0

    def remaining_ms(self) -> float:
        """
        Estimated ms until current in-flight line finishes.
        Prefers M73-anchored interpolation, falls back to pure physics.
        """
        elapsed = self.elapsed_ms()
        physics_rem = max(0.0, self.current_line_ms - elapsed)

        if (self._scanner.has_m73 and self._file_loaded
                and 0 <= self._current_idx < len(self._line_table)):
            # M73 remaining at the start of this line (seconds → ms)
            m73_at_line_ms = self._scanner.get_remaining_at(
                                self._current_idx + 1) * 1000.0
            m73_rem = max(0.0, m73_at_line_ms - elapsed)
            # Use physics for sub-minute resolution, M73 for absolute anchor
            return min(physics_rem, m73_rem) if m73_rem > 0 else physics_rem

        return physics_rem

    def total_remaining_ms(self) -> float:
        """Estimated ms remaining for the entire print."""
        elapsed = self.elapsed_ms()

        if (self._scanner.has_m73 and self._file_loaded
                and 0 <= self._current_idx < len(self._line_table)):
            m73_rem_ms = self._scanner.get_remaining_at(
                            self._current_idx + 1) * 1000.0
            return max(0.0, m73_rem_ms - elapsed)

        if self._file_loaded and self._current_idx < len(self._line_table):
            return max(0.0,
                       sum(r.seconds * 1000.0
                           for r in self._line_table[self._current_idx:])
                       - elapsed)
        return 0.0

    def progress_pct(self, current_idx: int) -> float:
        """Progress percentage (0–100)."""
        if self._scanner.has_m73:
            a_prev, _ = self._scanner.get_segment(current_idx + 1)
            if a_prev:
                return a_prev.progress_pct
        if self._file_loaded and self._line_table:
            elapsed_s = sum(r.seconds for r in self._line_table[:current_idx])
            total_s   = sum(r.seconds for r in self._line_table)
            return min(100.0, elapsed_s / total_s * 100.0) if total_s > 0 else 0.0
        return 0.0


# ─────────────────────────────────────────────────────────────
# Batch helpers (used by CLI)
# ─────────────────────────────────────────────────────────────

def build_line_table(lines: List[str]) -> List[LineResult]:
    scanner = M73Scanner()
    scanner.scan(lines)
    return build_hybrid_line_table(lines, scanner)


def estimate_pause_latency(filepath: str, pause_at_line: int,
                           lookahead_buffer: int = 16):
    with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
        lines = f.readlines()
    scanner = M73Scanner()
    scanner.scan(lines)
    table   = build_hybrid_line_table(lines, scanner)
    total   = len(lines)
    pause_at_line = max(1, min(pause_at_line, total))
    cur     = table[pause_at_line - 1]
    lbl_s   = cur.seconds
    buf_end = min(pause_at_line + lookahead_buffer - 1, total)
    buf_s   = sum(r.seconds for r in table[pause_at_line-1 : buf_end])
    speedup = buf_s / lbl_s if lbl_s > 1e-9 else float('inf')

    print(f"\n{'─'*72}")
    print(f"  Pause Latency Estimator  [{scanner.summary()}]")
    print(f"  File: {filepath}  |  Pause at line: {pause_at_line}/{total}")
    print(f"{'─'*72}")
    print(f"\n  LINE-BY-LINE MODE:")
    print(f"    cmd={cur.command}  note={cur.note}")
    print(f"    → Pause latency: {lbl_s*1000:.2f} ms")
    print(f"\n  BUFFERED MODE (depth={lookahead_buffer}):")
    for r in table[pause_at_line-1 : buf_end]:
        if r.seconds > 0:
            print(f"    {r.line_number:>5}  {r.command:<5}  {r.seconds*1000:>8.2f}ms  {r.note[:42]}")
    print(f"    → Pause latency: {buf_s*1000:.2f} ms")
    print(f"\n  SUMMARY: {lbl_s*1000:.2f}ms vs {buf_s*1000:.2f}ms  "
          f"({(buf_s-lbl_s)*1000:.2f}ms saved, {speedup:.1f}x faster)\n")
    return lbl_s, buf_s


def estimate_gcode_file(filepath: str, verbose: bool = False, line_range=None):
    with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
        lines = f.readlines()
    scanner = M73Scanner()
    scanner.scan(lines)
    table   = build_hybrid_line_table(lines, scanner)
    lo = line_range[0] if line_range else 1
    hi = line_range[1] if line_range else len(lines)
    lo, hi = max(1, lo), min(len(lines), hi)
    subset  = table[lo-1 : hi]
    total_s = sum(r.seconds for r in subset)

    print(f"\n{'─'*72}")
    print(f"  GCode Time Estimator  [{scanner.summary()}]")
    print(f"  File: {filepath}  |  Lines: {lo}–{hi}")
    print(f"{'─'*72}")

    if verbose or line_range:
        print(f"\n{'Line':>7}  {'Cmd':<5}  {'Dist':>8}  {'ms':>9}  Note")
        print(f"{'─'*72}")
        for r in subset:
            if r.seconds > 0 or r.command in (
                    "G90","G91","M82","M83","G92","M204","M205","M203","M73"):
                flag = "  ◄M73" if r.command == "M73" else ""
                print(f"{r.line_number:>7}  {r.command:<5}  {r.distance:>8.2f}"
                      f"  {r.seconds*1000:>9.2f}  {r.note[:40]}{flag}")

    h, rem = divmod(int(total_s), 3600)
    m, s   = divmod(rem, 60)
    print(f"\n{'─'*72}")
    if scanner.has_m73:
        sl = scanner.get_total_seconds()
        sh, sr = divmod(int(sl), 3600); sm, ss = divmod(sr, 60)
        err = abs(total_s-sl)/sl*100 if sl > 0 else 0
        print(f"  Slicer (M73)     : {sh}h {sm}m {ss}s  ← ground truth")
        print(f"  Hybrid estimate  : {h}h {m}m {s}s  (error ≈{err:.1f}%)")
    else:
        print(f"  Physics estimate : {h}h {m}m {s}s")
    print(f"  Total distance   : {sum(r.distance for r in subset):.1f} mm")
    print(f"{'─'*72}\n")
    return total_s, table


# ─────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='GCode time estimator — hybrid M73 + physics')
    parser.add_argument('gcode')
    parser.add_argument('--verbose', '-v', action='store_true')
    parser.add_argument('--line',     '-l', type=str, default=None)
    parser.add_argument('--pause-at', '-p', type=int, default=None)
    parser.add_argument('--buffer',   '-b', type=int, default=16)
    args = parser.parse_args()

    if args.pause_at is not None:
        estimate_pause_latency(args.gcode, args.pause_at, args.buffer)
        return

    line_range = None
    if args.line:
        if '-' in args.line:
            lo, hi = args.line.split('-', 1)
            line_range = (int(lo), int(hi))
        else:
            n = int(args.line)
            line_range = (n, n)

    estimate_gcode_file(args.gcode, verbose=args.verbose, line_range=line_range)


if __name__ == "__main__":
    main()