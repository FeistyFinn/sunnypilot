"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Pure MADS finger-count grant / storm / timeout state machine, shared by the live watcher
(live_watch.py, which formats these events to stdout) and the offline transcriber
(transcribe_events.py, which serializes them to a per-segment events.jsonl). Dependency-free on
purpose -- no cereal / numpy / GL imports -- so both a headless replay and the live process can use
the SAME state machine without dragging in a display context (and so there is ONE implementation to
keep correct, not two copies).

The caller decodes cereal and feeds plain scalars per message, in the same order the live SubMaster
delivers them, then calls tick(t) once per processed message for the grant-timeout check. Each on_*()
returns a list of event dicts (often empty); tick() returns the timeout event if one fired. Event
dicts carry only the semantic payload -- the caller stamps t / mono and either formats them
(live_watch) or serializes them (transcribe_events).

All time arguments are a single monotonically-increasing clock in seconds; only differences are used,
so the epoch is free (route-relative, segment-relative, wall-clock -- all give identical counts).
"""
from __future__ import annotations

# The services the state machine needs alive before it starts (mirrors MadsMonitor.SERVICES order).
MADS_SERVICES = ("pandaStates", "carState", "carControl", "selfdriveState", "onroadEventsSP")

GRANT_OK_S = 0.1          # grant latency below this is healthy
GRANT_TIMEOUT_S = 0.5     # no controlsAllowedLateral this long after a request -> warn (storm precursor)
IGNORED_SAFETY_MODELS = ("silent", "noOutput")


class MadsEventDetector:
  """Finger-count grant / storm / timeout state machine. One instance per ROUTE -- state persists
  across that route's segments, exactly as the live MadsMonitor persists across a whole replay."""

  def __init__(self):
    self.cal = None                 # controlsAllowed / controlsAllowedLateral of the chosen panda
    self.pal = None
    self.req_t = None               # time of the engage-request edge currently being timed
    self.req_kind = ""              # "lkas" or "latActive"
    self.prev_lat_active = False
    self.prev_pal = None
    self._storm_prev = False
    # cumulative counters (behind the live "MADS ..." summary line)
    self.n_lkas = 0
    self.n_grant_ok = 0
    self.n_grant_slow = 0
    self.n_timeout = 0
    self.n_storm = 0
    self.max_lat = 0.0

  def _arm(self, t: float, kind: str) -> None:
    # first request edge of an engage cycle wins; ignore once already waiting or already granted
    if self.req_t is None and not self.pal:
      self.req_t = t
      self.req_kind = kind

  def on_panda(self, t: float, panda_states: list) -> list[dict]:
    """panda_states: list of (safety_model_str, controls_allowed, controls_allowed_lateral) for the
    pandaStates frame. Picks the first non-ignored panda (if none match, cal/pal retain their prior
    value), then resolves a pending grant when controlsAllowedLateral rises. Returns [grant] or []."""
    out: list[dict] = []
    for sm_model, ca, ca_lat in panda_states:
      if sm_model not in IGNORED_SAFETY_MODELS:
        self.cal, self.pal = ca, ca_lat
        break
    if self.pal and not self.prev_pal and self.req_t is not None:    # grant resolved
      lat = t - self.req_t
      ok = lat < GRANT_OK_S
      self.n_grant_ok += int(ok)
      self.n_grant_slow += int(not ok)
      self.max_lat = max(self.max_lat, lat)
      out.append({"event": "mads_grant", "lat": lat, "ok": ok, "kind": self.req_kind})
      self.req_t = None
    if self.prev_pal and not self.pal:                               # disengage/revoke ends the cycle
      self.req_t = None
    self.prev_pal = self.pal
    return out

  def on_lat_active(self, t: float, lat_active: bool) -> None:
    if lat_active and not self.prev_lat_active:
      self._arm(t, "latActive")
    self.prev_lat_active = lat_active

  def on_lkas(self, t: float, n_pressed: int) -> None:
    for _ in range(int(n_pressed)):
      self.n_lkas += 1
      self._arm(t, "lkas")

  def on_storm(self, t: float, has_storm: bool, immediate, vego) -> list[dict]:
    """has_storm: a controlsMismatchLateral event is present this onroadEventsSP frame. Counts
    distinct storm ONSETS (rising edge). Returns [storm] or []."""
    out: list[dict] = []
    if has_storm and not self._storm_prev:
      self.n_storm += 1
      out.append({"event": "mads_storm", "immediate": bool(immediate),
                  "vego": round(vego, 2) if vego is not None else None})
    self._storm_prev = has_storm
    return out

  def tick(self, t: float) -> list[dict]:
    """Grant-timeout check: a request that never got controlsAllowedLateral within GRANT_TIMEOUT_S is
    abandoned (so a later grant can't report a stale latency, and the next edge re-arms cleanly). Call
    once per processed message. Returns [timeout] or []."""
    out: list[dict] = []
    if self.req_t is not None and not self.pal and (t - self.req_t) > GRANT_TIMEOUT_S:
      self.n_timeout += 1
      out.append({"event": "mads_grant_timeout", "elapsed": t - self.req_t, "kind": self.req_kind})
      self.req_t = None
    return out

  def summary(self) -> dict:
    """The fields behind the live "MADS ..." summary line, as a dict (live_watch formats the string)."""
    return {"lkas_taps": self.n_lkas, "grants_ok": self.n_grant_ok, "grants_slow": self.n_grant_slow,
            "timeouts": self.n_timeout, "storms": self.n_storm, "max_lat": self.max_lat}
